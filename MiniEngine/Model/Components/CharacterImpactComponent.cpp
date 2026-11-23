#include "CharacterImpactComponent.h"
#include "../GameFramework/Character.h"
#include "GameFramework/World.h"
#include "PhysicsWorld.h"
#include <extensions/PxRigidBodyExt.h>

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
        const bool controlled = character.GetController() != nullptr;
        const bool warmup = character.GetMesh().IsImpactWarmupActive();
        // 컨트롤러가 있는 캐릭터에는 PA와 독립된 동적 충돌 캡슐을 만든다.
        ragdoll.SetCharacterImpactEnabled(m_Settings.enabled &&
            (controlled || ragdoll.IsSimulating() || character.GetMesh().UsesAutomaticRagdoll() || warmup),
            static_cast<uint32_t>(character.GetHandle().id), warmup);
        const bool enabled = ragdoll.UsesCharacterImpact() && controlled;
        ragdoll.ConfigureCharacterCapsule(enabled, m_Settings.collisionMass, m_Settings.capsuleRadiusScale,
            m_Settings.contactSkin, m_PhysicsError, m_Settings.capsuleHeightScale);
        // 키네마틱 본체가 이동 쿼리의 겹침 해소로 밀리지 않도록 캐릭터 접촉은 몸통 캡슐에 맡긴다.
        character.GetCapsuleComponent().SetCharacterBlockingEnabled(!ragdoll.HasCharacterCapsule());
    }

    void CharacterImpactComponent::BeginFixedStep(float deltaTime)
    {
        (void)deltaTime;
        m_PreviousPushDirection = m_PushVelocity.getNormalized();
        m_PushVelocity = PxVec3(0);
        RefreshCollisionPolicy();
    }

    void CharacterImpactComponent::SyncCapsule()
    {
        Character& character = static_cast<Character&>(GetOwner());
        character.GetMesh().GetRagdoll().SyncCharacterCapsule(m_PushVelocity);
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
        const bool animated = !ragdoll.IsSimulating();
        const Vector3 locomotion = source.GetCharacterMovement().GetLocomotionVelocity();
        const PxVec3 drive(locomotion.GetX(), 0, locomotion.GetZ());
        if (animated && drive.dot(m_PreviousPushDirection) > 0)
        {
            // 직전 몸통 스윕에서 잘린 전진 입력은 이번 스윕에서 다시 시도한다.
            const float lost = (drive * deltaTime - remaining).dot(m_PreviousPushDirection);
            remaining += m_PreviousPushDirection * PxMax(0.0f, lost);
        }
        std::vector<PhysicsCapsuleHit> hits;
        // 실제 표면 접촉은 솔버에 남기고, 이동 목표가 상대 몸을 뚫고 누적되는 것만 막는다.
        const float sweepRadius = animated ? radius * .5f : radius + m_Settings.contactSkin;
        for (int iteration = 0; iteration < 3 && remaining.magnitudeSquared() > 1e-12f; ++iteration)
        {
            physics->SweepCharacterCapsule(capsule, sweepRadius, halfLength, remaining, &source, hits);
            const float distance = remaining.magnitude();
            const PxVec3 direction = remaining / distance;
            float allowed = distance;
            PxVec3 normal(0);
            for (const PhysicsCapsuleHit& hit : hits)
            {
                PxVec3 horizontalNormal(hit.normal.x, 0, hit.normal.z);
                const float closing = horizontalNormal.normalize() > 1e-4f ? -horizontalNormal.dot(direction) : 0;
                if (closing <= 1e-5f)
                {
                    continue;
                }
                float clearance = hit.distance;
                const PxRigidBody* receiver = hit.actor->is<PxRigidBody>();
                if (animated && receiver)
                {
                    // 같은 틱에 상대가 물러나는 만큼 따라가되, 두 몸통이 서로 지나치지 않게 한다.
                    const PxVec3 velocity = PxRigidBodyExt::getVelocityAtPos(*receiver, hit.position);
                    const float receding = PxMax(0.0f, -velocity.dot(horizontalNormal));
                    clearance = (clearance < 0 ? clearance / closing : clearance) + receding * deltaTime / closing;
                }
                if (clearance <= allowed)
                {
                    allowed = PxMax(0.0f, clearance - .001f);
                    normal = horizontalNormal;
                }
            }
            const PxVec3 travel = direction * allowed;
            if (animated && iteration == 0 && allowed < distance)
            {
                // 몸통끼리 막혀도 계속 누르는 입력은 독립 충돌 캡슐의 접촉 속도로 전달한다.
                m_PushVelocity = -normal * PxMax(0.0f, -drive.dot(normal));
            }
            moved += travel;
            capsule.p += travel;
            remaining -= travel;
            // 정면으로 밀 때는 굽은 몸통 표면이 전진 입력을 옆걸음으로 바꾸지 않게 한다.
            // 비스듬히 스친 경우에는 표면 접선 방향의 입력을 허용한다.
            if (normal.magnitudeSquared() < 1e-8f || (animated && normal.dot(direction) < -.70710678f))
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
