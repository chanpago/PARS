#include "SkeletalMeshComponent.h"

#include <cmath>

namespace PARS
{
    namespace
    {
        // 구현은 보존하고 상체 버둥거림의 실제 적용만 꺼 둔다.
        constexpr bool EnableRecoveryFlail = false;

        bool IsRecoveryFlailEnabled()
        {
            return EnableRecoveryFlail;
        }
    }

    bool SkeletalMeshComponent::ConfigureRecoveryFlail(uint32_t frontAnimationIndex, uint32_t backAnimationIndex)
    {
        if (frontAnimationIndex >= m_AnimInstance->GetAnimationCount() ||
            backAnimationIndex >= m_AnimInstance->GetAnimationCount())
        {
            return false;
        }
        PhysicsSkeleton skeleton;
        std::string error;
        return BuildPhysicsSkeleton(skeleton, error) &&
            m_RecoveryFlail.Configure(skeleton.names, skeleton.parents, frontAnimationIndex, backAnimationIndex);
    }

    void SkeletalMeshComponent::CaptureRecoveryFlailDirection(const std::vector<RagdollBodyImpulse>& impacts)
    {
        if (!IsRecoveryFlailEnabled())
        {
            return;
        }
        Math::Vector3 impulse(Math::kZero);
        for (const RagdollBodyImpulse& impact : impacts)
        {
            if (impact.impulse.isFinite())
            {
                impulse += Math::Vector3(impact.impulse.x, impact.impulse.y, impact.impulse.z);
            }
        }
        m_RecoveryFlail.RecordImpact(impulse, GetComponentTransform().GetZ());
    }

    bool SkeletalMeshComponent::ApplyRecoveryFlailTargets(std::vector<Math::Matrix4>& target,
        float deltaTime, float impactVelocity)
    {
        if (!IsRecoveryFlailEnabled() || !m_RecoveryFlail.IsConfigured() ||
            m_Recovery.GetSnapshot().state != RecoveryState::Recovery)
        {
            m_RecoveryFlail.Reset();
            return true;
        }
        if (impactVelocity >= m_RecoverySettings.impactVelocity)
        {
            std::vector<RagdollBodyImpulse> impacts;
            m_Ragdoll.CaptureExternalImpacts(impacts);
            CaptureRecoveryFlailDirection(impacts);
        }
        const FootSupportSnapshot& support = m_FootSupport.GetSnapshot();
        if (!m_RecoveryFlail.HasImpactDirection() && support.hasCenterOfMassVelocity)
        {
            // 접촉 전에 액티브 모드로 전환했거나 충격 없이 발딛기에 진입하면 실제 이동 방향을 사용한다.
            const physx::PxVec3& velocity = support.centerOfMassVelocity;
            if (std::hypot(velocity.x, velocity.z) >= m_RecoverySettings.impactVelocity)
            {
                m_RecoveryFlail.RecordImpact(Math::Vector3(velocity.x, velocity.y, velocity.z), GetComponentTransform().GetZ());
            }
        }
        const uint32_t front = m_RecoveryFlail.GetFrontAnimation();
        const uint32_t back = m_RecoveryFlail.GetBackAnimation();
        const float frontDuration = m_AnimInstance->GetAnimationDuration(front);
        const float backDuration = m_AnimInstance->GetAnimationDuration(back);
        m_RecoveryFlail.Update(true, deltaTime, frontDuration, backDuration);
        const float phase = m_RecoveryFlail.GetPhase();
        m_RecoveryFlailPose = m_AnimInstance->GetPose();
        if (!m_AnimInstance->SampleAnimationPose(front, phase * frontDuration, m_RecoveryFlailFrontPose) ||
            !m_AnimInstance->SampleAnimationPose(back, phase * backDuration, m_RecoveryFlailBackPose) ||
            !m_RecoveryFlail.Apply(m_RecoveryFlailFrontPose, m_RecoveryFlailBackPose, m_RecoveryFlailPose))
        {
            m_PhysicsError = "Cannot sample the recovery upper-body flail animations.";
            return false;
        }
        // 골반과 하체의 로컬 변환은 그대로이며, 뒤에서 기존 발딛기 IK를 적용한다.
        return EvaluatePhysicsPose(m_PhysicsSkeleton, m_RecoveryFlailPose,
            Math::Matrix4(GetComponentTransform()), target, m_PhysicsError);
    }
}
