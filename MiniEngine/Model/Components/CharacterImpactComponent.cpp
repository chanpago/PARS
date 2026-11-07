#include "CharacterImpactComponent.h"
#include "../GameFramework/Character.h"
#include "GameFramework/World.h"
#include "PhysicsWorld.h"

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
        const CharacterImpactSettings previous = m_Settings;
        m_Settings = settings;
        RefreshCollisionPolicy();
        if (!m_PhysicsError.empty())
        {
            error = m_PhysicsError;
            m_Settings = previous;
            RefreshCollisionPolicy();
            return false;
        }
        error.clear();
        return true;
    }

    void CharacterImpactComponent::RefreshCollisionPolicy()
    {
        Character& character = static_cast<Character&>(GetOwner());
        Ragdoll& ragdoll = character.GetMesh().GetRagdoll();
        // 컨트롤러가 있는 캐릭터에는 PA와 독립된 동적 충돌 캡슐을 만든다.
        ragdoll.SetCharacterImpactEnabled(m_Settings.enabled &&
            (ragdoll.IsSimulating() || character.GetMesh().UsesAutomaticRagdoll()),
            static_cast<uint32_t>(character.GetHandle().id));
        const bool enabled = ragdoll.IsSimulating() && ragdoll.UsesCharacterImpact() && character.GetController() != nullptr;
        ragdoll.ConfigureCharacterCapsule(enabled, m_Settings.collisionMass, m_Settings.capsuleRadiusScale,
            m_Settings.contactSkin, m_PhysicsError, m_Settings.capsuleHeightScale);
    }

    void CharacterImpactComponent::BeginFixedStep(float deltaTime)
    {
        (void)deltaTime;
        RefreshCollisionPolicy();
    }

    Vector3 CharacterImpactComponent::ConstrainMovement(Vector3 displacement, float deltaTime)
    {
        Character& source = static_cast<Character&>(GetOwner());
        Ragdoll& ragdoll = source.GetMesh().GetRagdoll();
        World* world = source.GetWorld();
        PhysicsWorld* physics = world ? world->GetPhysicsWorld() : nullptr;
        if (!physics || !ragdoll.HasCharacterCapsule() || deltaTime <= 0)
        {
            return displacement;
        }
        PxTransform capsule(PxIdentity);
        PxVec3 targetOffset(0);
        float radius = 0;
        float halfLength = 0;
        if (!ragdoll.GetCharacterImpactCapsule(m_Settings.capsuleRadiusScale, capsule, radius, halfLength, targetOffset))
        {
            return displacement;
        }
        // 스윕은 이동 목표의 누적 관통만 방지한다. 접촉 힘은 PhysX가 계산한다.
        capsule.p += targetOffset;
        PxVec3 moved(0);
        PxVec3 remaining(float(displacement.GetX()), 0, float(displacement.GetZ()));
        std::vector<PhysicsCapsuleHit> hits;
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
                if (horizontalNormal.normalize() > 1e-4f && horizontalNormal.dot(direction) < -1e-5f && hit.distance <= allowed)
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

    void CharacterImpactComponent::RecordContacts()
    {
        Character& source = static_cast<Character&>(GetOwner());
        Ragdoll& ragdoll = source.GetMesh().GetRagdoll();
        World* world = source.GetWorld();
        PhysicsWorld* physics = world ? world->GetPhysicsWorld() : nullptr;
        if (!physics)
        {
            return;
        }
        if (m_ClockRevision == physics->GetClockRevision() && m_RecordedStep == physics->GetStepCount() &&
            m_SourceRevision == ragdoll.GetContactRevision())
        {
            return;
        }
        m_ClockRevision = physics->GetClockRevision();
        m_RecordedStep = physics->GetStepCount();
        m_SourceRevision = ragdoll.GetContactRevision();
        m_AppliedImpacts.clear();
        m_Snapshot.step = m_RecordedStep;
        m_Snapshot.normalImpulse = 0;
        m_Snapshot.shares.clear();
        m_Snapshot.reason = ragdoll.HasCharacterCapsule() ? CharacterImpactReason::NoContact : CharacterImpactReason::Disabled;
        std::vector<PhysicsCapsuleContact> contacts;
        ragdoll.GetCharacterCapsuleContacts(contacts);
        for (const PhysicsCapsuleContact& contact : contacts)
        {
            Character* victim = nullptr;
            world->ForEachActor([&](Actor& actor) -> void
            {
                if (&actor == contact.owner)
                {
                    victim = dynamic_cast<Character*>(&actor);
                }
            });
            const int body = victim ? victim->GetMesh().GetRagdoll().FindPhysicsBody(contact.actor) : -1;
            if (body < 0 || contact.normalImpulse <= 1e-6f)
            {
                continue;
            }
            m_Snapshot.target = victim->GetHandle();
            m_Snapshot.hitBone = victim->GetMesh().GetPhysicsAsset()->bodies[body].bone;
            m_Snapshot.point = contact.position;
            m_Snapshot.direction = contact.normal;
            m_Snapshot.normalImpulse = contact.normalImpulse;
            m_Snapshot.reason = CharacterImpactReason::Applied;
            ++m_Snapshot.eventCount;
            BodyImpactShare share;
            share.bone = m_Snapshot.hitBone;
            share.impulse = contact.impulse;
            m_Snapshot.shares = {share};
            m_LastImpact = m_Snapshot;
            m_AppliedImpacts.push_back(m_Snapshot);
        }
    }

    const CharacterImpactSnapshot& CharacterImpactComponent::GetSnapshot()
    {
        RecordContacts();
        return m_Snapshot;
    }

    const CharacterImpactSnapshot& CharacterImpactComponent::GetLastImpact()
    {
        RecordContacts();
        return m_LastImpact;
    }

    const std::vector<CharacterImpactSnapshot>& CharacterImpactComponent::GetAppliedImpacts()
    {
        RecordContacts();
        return m_AppliedImpacts;
    }

    void CharacterImpactComponent::EndPlay()
    {
        Character& character = static_cast<Character&>(GetOwner());
        character.GetMesh().GetRagdoll().RemoveCharacterCapsule();
        m_AppliedImpacts.clear();
    }
}
