#include "CharacterImpactComponent.h"
#include "../GameFramework/Character.h"
#include "GameFramework/World.h"
#include "PhysicsWorld.h"
#include <algorithm>
#include <cmath>

using namespace physx;
using namespace Math;

namespace PARS
{
    bool CharacterImpactComponent::SetSettings(const CharacterImpactSettings& settings, std::string& error)
    {
        if (!settings.Validate(error))
        {
            return false;
        }
        m_Settings = settings;
        RefreshCollisionPolicy();
        return true;
    }

    void CharacterImpactComponent::RefreshCollisionPolicy()
    {
        Character& character = static_cast<Character&>(GetOwner());
        Ragdoll& ragdoll = character.GetMesh().GetRagdoll();
        // 지지가 해제된 캐릭터와 패시브 래그돌은 기존 PA 접촉으로 되돌린다.
        ragdoll.SetCharacterImpactEnabled(m_Settings.enabled && character.GetMesh().IsActiveRagdoll() &&
            ragdoll.IsRootSupported(), static_cast<uint32_t>(character.GetHandle().id));
    }

    void CharacterImpactComponent::BeginFixedStep(float deltaTime)
    {
        RefreshCollisionPolicy();
        World* world = GetOwner().GetWorld();
        PhysicsWorld* physics = world ? world->GetPhysicsWorld() : nullptr;
        Character& character = static_cast<Character&>(GetOwner());
        const uint64_t revision = character.GetMesh().GetRagdoll().GetContactRevision();
        if (!physics)
        {
            return;
        }
        if (m_ClockRevision != physics->GetClockRevision() || m_SourceRevision != revision)
        {
            m_Contacts.clear();
            m_Pending.clear();
            m_Time = 0;
            m_ClockRevision = physics->GetClockRevision();
            m_SourceRevision = revision;
        }
        m_Time += deltaTime;
        for (Contact& contact : m_Contacts)
        {
            if (physics->GetStepCount() > contact.lastSeenStep + 1 && m_Time - contact.lastSeen > m_Settings.separationTime)
            {
                contact.latched = false;
            }
        }
        m_Pending.clear();
        m_AppliedPartners.clear();
        m_Snapshot.step = physics->GetStepCount() + 1;
        m_Snapshot.commandedImpulse = 0;
        m_Snapshot.requestedImpulse = 0;
        m_Snapshot.closingSpeed = 0;
        m_Snapshot.capped = false;
        m_Snapshot.shares.clear();
        m_Snapshot.reason = character.GetMesh().GetRagdoll().UsesCharacterImpact() ?
            CharacterImpactReason::NoContact : CharacterImpactReason::Disabled;
        m_Contacts.erase(std::remove_if(m_Contacts.begin(), m_Contacts.end(), [&](const Contact& contact) -> bool
        {
            return !world->FindActor(contact.target) || m_Time - contact.lastSeen > 10;
        }), m_Contacts.end());
    }

    Vector3 CharacterImpactComponent::ConstrainMovement(Vector3 displacement, float deltaTime)
    {
        Character& source = static_cast<Character&>(GetOwner());
        Ragdoll& ragdoll = source.GetMesh().GetRagdoll();
        World* world = source.GetWorld();
        PhysicsWorld* physics = world ? world->GetPhysicsWorld() : nullptr;
        if (!physics || !ragdoll.UsesCharacterImpact() || deltaTime <= 0)
        {
            return displacement;
        }
        PxTransform capsule(PxIdentity);
        float radius = 0;
        float halfLength = 0;
        PxVec3 targetOffset(0);
        if (!ragdoll.GetCharacterImpactCapsule(m_Settings.capsuleRadiusScale, capsule, radius, halfLength, targetOffset))
        {
            return displacement;
        }
        const PxVec3 requested(float(displacement.GetX()), 0, float(displacement.GetZ()));
        const int root = ragdoll.GetImpactRootBody();
        const PxVec3 actualVelocity = ragdoll.GetBodyPointVelocity(root, capsule.p);
        m_Snapshot.actualVelocity = actualVelocity;
        m_Snapshot.requestedVelocity = requested / deltaTime;

        struct Candidate
        {
        public:
            Character* target = nullptr;
            PhysicsCapsuleHit hit;
            PxVec3 direction{0};
            int body = -1;
            float closing = 0;
        };
        std::vector<Candidate> candidates;
        std::vector<PhysicsCapsuleHit> hits;
        // 작은 반동과 바디 교대 접촉은 새로운 충돌로 세지 않는다.
        physics->SweepCharacterCapsule(capsule, radius + m_Settings.contactSkin + m_Settings.separationDistance,
            halfLength, PxVec3(0), &source, hits);
        for (Contact& contact : m_Contacts)
        {
            const Actor* target = world->FindActor(contact.target);
            if (contact.latched && target && std::any_of(hits.begin(), hits.end(), [&](const PhysicsCapsuleHit& hit) -> bool
            {
                return hit.owner == target;
            }))
            {
                contact.lastSeen = m_Time;
                contact.lastSeenStep = physics->GetStepCount();
            }
        }
        PxVec3 actualTravel = actualVelocity * deltaTime;
        actualTravel.y = 0;
        physics->SweepCharacterCapsule(capsule, radius + m_Settings.contactSkin, halfLength,
            actualTravel, &source, hits);
        for (const PhysicsCapsuleHit& hit : hits)
        {
            Character* target = nullptr;
            // userData를 임의의 Actor로 역참조하지 않고, 월드가 소유한 객체와 대조한다.
            world->ForEachActor([&](Actor& actor) -> void
            {
                if (&actor == hit.owner)
                {
                    target = dynamic_cast<Character*>(&actor);
                }
            });
            if (!target || target == &source || !target->GetMesh().GetRagdoll().UsesCharacterImpact())
            {
                continue;
            }
            Ragdoll& victim = target->GetMesh().GetRagdoll();
            const int body = victim.FindPhysicsBody(hit.actor);
            PxVec3 direction(-hit.normal.x, 0, -hit.normal.z);
            if (body < 0 || direction.normalize() < 1e-4f)
            {
                continue;
            }
            const PxVec3 sourceVelocity = ragdoll.GetBodyPointVelocity(root, hit.position);
            const PxVec3 targetVelocity = victim.GetBodyPointVelocity(body, hit.position);
            const float closing = PxMax(0.0f, (sourceVelocity - targetVelocity).dot(direction));
            // 양쪽 모두 이동 중이면 접근 기여가 큰 쪽 하나가 충돌을 담당한다.
            if (target->CanControlMovement())
            {
                const float sourceApproach = sourceVelocity.dot(direction);
                const float targetApproach = -targetVelocity.dot(direction);
                if (targetApproach > sourceApproach + 1e-4f ||
                    (std::abs(targetApproach - sourceApproach) <= 1e-4f && source.GetHandle().id > target->GetHandle().id))
                {
                    continue;
                }
            }
            std::vector<Candidate>::iterator existing = std::find_if(candidates.begin(), candidates.end(), [&](const Candidate& candidate) -> bool
            {
                return candidate.target == target;
            });
            if (existing == candidates.end())
            {
                candidates.emplace_back();
                existing = candidates.end() - 1;
            }
            else if (closing < existing->closing || (closing == existing->closing && body >= existing->body))
            {
                continue;
            }
            existing->target = target;
            existing->hit = hit;
            existing->direction = direction;
            existing->body = body;
            existing->closing = closing;
        }
        std::sort(candidates.begin(), candidates.end(), [](const Candidate& left, const Candidate& right) -> bool
        {
            return left.target->GetHandle().id < right.target->GetHandle().id;
        });
        for (const Candidate& candidate : candidates)
        {
            const ActorHandle handle = candidate.target->GetHandle();
            const uint64_t revision = candidate.target->GetMesh().GetRagdoll().GetContactRevision();
            std::vector<Contact>::iterator contact = std::find_if(m_Contacts.begin(), m_Contacts.end(), [&](const Contact& value) -> bool
            {
                return value.target == handle;
            });
            if (contact == m_Contacts.end())
            {
                m_Contacts.emplace_back();
                contact = m_Contacts.end() - 1;
                contact->target = handle;
                contact->revision = revision;
            }
            if (contact->revision != revision || (physics->GetStepCount() > contact->lastSeenStep + 1 &&
                m_Time - contact->lastSeen > m_Settings.separationTime))
            {
                contact->latched = false;
                contact->revision = revision;
            }
            contact->lastSeen = m_Time;
            contact->lastSeenStep = physics->GetStepCount();
            m_Snapshot.target = handle;
            m_Snapshot.point = candidate.hit.position;
            m_Snapshot.direction = candidate.direction;
            m_Snapshot.closingSpeed = candidate.closing;
            m_Snapshot.hitBone = candidate.target->GetMesh().GetPhysicsAsset()->bodies[candidate.body].bone;
            float requestedImpulse = 0;
            const float amount = m_Settings.ComputeImpulse(candidate.closing, requestedImpulse);
            m_Snapshot.requestedImpulse = requestedImpulse;
            m_Snapshot.capped = requestedImpulse > amount;
            if (contact->latched)
            {
                m_Snapshot.reason = CharacterImpactReason::HeldContact;
            }
            else if (m_Time - contact->lastEvent < m_Settings.cooldown)
            {
                m_Snapshot.reason = CharacterImpactReason::Cooldown;
            }
            else if (amount <= 0)
            {
                m_Snapshot.reason = CharacterImpactReason::BelowMinimumSpeed;
            }
            else
            {
                PendingImpact impact;
                impact.target = handle;
                impact.targetRevision = revision;
                impact.sourceRevision = ragdoll.GetContactRevision();
                impact.clockRevision = physics->GetClockRevision();
                impact.point = candidate.hit.position;
                impact.impulse = candidate.direction * amount;
                m_Pending.push_back(impact);
                m_Snapshot.commandedImpulse = amount;
                m_Snapshot.reason = CharacterImpactReason::Queued;
                contact->latched = true;
                contact->lastEvent = m_Time;
            }
        }

        // 목표 캡슐도 검사해 실제 몸이 막혔을 때 애니메이션 루트가 상대를 뚫고 누적되지 않게 한다.
        capsule.p += targetOffset;
        PxVec3 moved(0);
        PxVec3 remaining = requested;
        for (int iteration = 0; iteration < 3 && remaining.magnitudeSquared() > 1e-12f; ++iteration)
        {
            physics->SweepCharacterCapsule(capsule, radius + m_Settings.contactSkin, halfLength, remaining, &source, hits);
            const float distance = remaining.magnitude();
            const PxVec3 direction = remaining / distance;
            float allowed = distance;
            PxVec3 normal(0);
            for (const PhysicsCapsuleHit& hit : hits)
            {
                PxVec3 horizontalNormal(hit.normal.x, 0, hit.normal.z);
                if (horizontalNormal.normalize() > 1e-4f && horizontalNormal.dot(direction) < -1e-5f &&
                    hit.distance <= allowed)
                {
                    allowed = PxMax(0.0f, hit.distance - .001f);
                    normal = horizontalNormal;
                }
            }
            const PxVec3 travel = direction * allowed;
            moved += travel;
            capsule.p += travel;
            remaining -= travel;
            if (normal.magnitudeSquared() < 1e-8f)
            {
                break;
            }
            remaining -= normal * PxMin(0.0f, remaining.dot(normal));
        }
        return Vector3(moved.x, 0, moved.z);
    }

    void CharacterImpactComponent::ApplyPhysicsInputs(float deltaTime)
    {
        Character& source = static_cast<Character&>(GetOwner());
        World* world = source.GetWorld();
        PhysicsWorld* physics = world ? world->GetPhysicsWorld() : nullptr;
        if (!physics)
        {
            m_Pending.clear();
            return;
        }
        for (const PendingImpact& impact : m_Pending)
        {
            Character* victim = world->FindActor<Character>(impact.target);
            if (std::find(m_AppliedPartners.begin(), m_AppliedPartners.end(), impact.target) != m_AppliedPartners.end())
            {
                m_Snapshot.reason = CharacterImpactReason::HeldContact;
                m_Snapshot.commandedImpulse = 0;
                continue;
            }
            if (!victim || !m_Settings.enabled || physics->GetClockRevision() != impact.clockRevision ||
                source.GetMesh().GetRagdoll().GetContactRevision() != impact.sourceRevision ||
                victim->GetMesh().GetRagdoll().GetContactRevision() != impact.targetRevision ||
                !victim->GetMesh().GetRagdoll().ApplyBodyImpact(impact.impulse, impact.point, m_Settings.pelvisShare,
                    deltaTime, m_Snapshot.shares))
            {
                m_Snapshot.reason = CharacterImpactReason::StaleTarget;
                m_Snapshot.commandedImpulse = 0;
                continue;
            }
            ++m_Snapshot.eventCount;
            m_AppliedPartners.push_back(impact.target);
            victim->GetCharacterImpact().m_AppliedPartners.push_back(source.GetHandle());
            m_Snapshot.reason = CharacterImpactReason::Applied;
            m_LastImpact = m_Snapshot;
        }
        m_Pending.clear();
    }

    void CharacterImpactComponent::EndPlay()
    {
        m_Pending.clear();
        m_Contacts.clear();
    }
}
