#include "SkeletalMeshComponent.h"
#include "../GameFramework/Character.h"
#include "PhysicsWorld.h"
#include <algorithm>
#include <limits>

namespace PARS
{
    const char* CharacterBalanceStateLabel(CharacterBalanceState state)
    {
        switch (state)
        {
        case CharacterBalanceState::Grounded:
            return "Grounded";
        case CharacterBalanceState::Unbalanced:
            return "Unbalanced";
        case CharacterBalanceState::GroundFall:
            return "GroundFall";
        case CharacterBalanceState::Falling:
            return "Falling";
        case CharacterBalanceState::Landed:
            return "Landed";
        }
        return "--";
    }

    bool SkeletalMeshComponent::IsAirborneFallClip(uint32_t animationIndex) const
    {
        return animationIndex < m_AnimInstance->GetAnimationCount() &&
            std::find(m_AirborneFallAnimations.begin(), m_AirborneFallAnimations.end(), animationIndex) != m_AirborneFallAnimations.end();
    }

    bool SkeletalMeshComponent::SetAirborneFallAnimations(const std::array<uint32_t, 3>& clips)
    {
        for (size_t index = 0; index < clips.size(); ++index)
        {
            const uint32_t clip = clips[index];
            if (clip >= m_AnimInstance->GetAnimationCount() || IsFallingClip(clip) ||
                clip == m_StandingUpAnimationIndex || clip == m_StandingIdleAnimationIndex ||
                std::find(clips.begin(), clips.begin() + index, clip) != clips.begin() + index)
            {
                return false;
            }
        }
        m_AirborneFallAnimations = clips;
        return true;
    }

    void SkeletalMeshComponent::ResetAirborneFall()
    {
        if (IsFalling() && m_StandingIdleAnimationIndex < m_AnimInstance->GetAnimationCount())
        {
            m_AnimInstance->TransitionToAnimation(m_StandingIdleAnimationIndex, .15f);
        }
        m_AirborneFallTimer = {};
        m_BalanceState = CharacterBalanceState::Grounded;
    }

    bool SkeletalMeshComponent::UpdateBalanceState(float deltaTime)
    {
        const FootSupportSnapshot& sample = m_FootSupport.GetSnapshot();
        const Character* character = dynamic_cast<const Character*>(&GetOwner());
        const bool movingCapsule = character && character->GetCharacterMovement().IsTickEnabled() &&
            character->GetCapsuleComponent().HasPhysicsCollision();
        if (m_StandingUpState != StandingUpState::None || (!IsActiveRagdoll() && !movingCapsule))
        {
            ResetAirborneFall();
            return true;
        }

        bool supported = !IsActiveRagdoll() && movingCapsule && character->GetCharacterMovement().IsMovingOnGround();
        for (const FootSoleSample& foot : sample.feet)
        {
            supported = supported || (foot.contact.hasSample && foot.contact.grounded);
        }
        if (IsActiveRagdoll() && !supported)
        {
            // 손이나 무릎이 이미 바닥을 받치고 있으면 공중 낙하가 아니다.
            PhysicsGroundContact contact;
            for (size_t body = 0; body < m_Ragdoll.GetBodyCount(); ++body)
            {
                if (m_Ragdoll.GetBodyGroundContact(body, contact))
                {
                    supported = true;
                    break;
                }
            }
        }
        if (IsFalling())
        {
            // 뒤꿈치가 모서리를 스치더라도 몸이 계속 떨어지면 같은 공중 동작을 유지한다.
            const bool descentArrested = sample.hasCenterOfMassVelocity &&
                sample.centerOfMassVelocity.y >= -m_RecoverySettings.maximumReturnSpeed;
            if (supported && descentArrested)
            {
                m_BalanceState = CharacterBalanceState::Landed;
                m_AirborneFallTimer = {};
            }
            return true;
        }

        const bool released = IsActiveRagdoll() && m_AutomaticRootSupport &&
            m_RootSupportRelease != RootSupportRelease::None;
        const bool nearGround = sample.projectedPointCount != 0;
        const bool hasClips = m_AirborneFallAnimations.front() != UINT32_MAX;
        if (m_BalanceState != CharacterBalanceState::Landed)
        {
            if (released && (supported || nearGround || !hasClips || m_BalanceState == CharacterBalanceState::GroundFall))
            {
                m_BalanceState = CharacterBalanceState::GroundFall;
            }
            else if (released || sample.footstep.needsStep || sample.footstep.IsInProgress() ||
                m_Recovery.GetSnapshot().state == RecoveryState::Recovery)
            {
                m_BalanceState = CharacterBalanceState::Unbalanced;
            }
            else
            {
                m_BalanceState = CharacterBalanceState::Grounded;
            }
        }

        // 키네마틱 보행은 이동 캡슐의 하강 속도를, 동적 바디는 실제 전신 COM 속도를 쓴다.
        const bool descending = IsActiveRagdoll() ? sample.hasCenterOfMassVelocity && sample.centerOfMassVelocity.y < 0 :
            movingCapsule && float(character->GetCharacterMovement().GetVelocity().GetY()) < 0;
        const bool validFeet = sample.feet[0].contact.hasSample && sample.feet[1].contact.hasSample;
        const bool held = !movingCapsule && !m_AutomaticRootSupport && m_Ragdoll.IsRootSupported();
        // 모서리에 남은 발바닥 레이 투영은 실제 지지가 끊긴 시간을 연장하지 않는다.
        if (hasClips && m_AirborneFallTimer.Update(sample.contactRevision, sample.completedPhysicsStep,
            deltaTime, supported || !validFeet || held, descending))
        {
            return BeginAirborneFall();
        }
        return true;
    }

    bool SkeletalMeshComponent::BeginAirborneFall()
    {
        const std::vector<int>& nodes = m_Ragdoll.GetBoneNodes();
        const int pelvis = m_Ragdoll.GetImpactRootBody();
        if (pelvis < 0 || static_cast<size_t>(pelvis) >= nodes.size())
        {
            m_PhysicsError = "Airborne fall requires a pelvis body.";
            return false;
        }
        // 골반 기준 각 본의 높이를 비교하면 진행 방향에 관계없이 현재 자세와 가까운 루프를 고를 수 있다.
        const float pelvisY = float(m_Ragdoll.GetBoneWorld(pelvis).GetW().GetY());
        std::vector<float> heights;
        for (size_t body = 0; body < nodes.size(); ++body)
        {
            heights.push_back(float(m_Ragdoll.GetBoneWorld(body).GetW().GetY()) - pelvisY);
        }
        uint32_t bestClip = UINT32_MAX;
        float minimumError = std::numeric_limits<float>::infinity();
        std::vector<GraphNode> pose;
        std::vector<Math::Matrix4> worldPose;
        for (uint32_t clip : m_AirborneFallAnimations)
        {
            if (!m_AnimInstance->SampleAnimationPose(clip, 0, pose) ||
                !EvaluateAnimationPhysicsPose(m_PhysicsSkeleton, worldPose, m_PhysicsError, &pose))
            {
                return false;
            }
            const float clipPelvisY = float(worldPose[nodes[pelvis]].GetW().GetY());
            float error = 0;
            for (size_t body = 0; body < nodes.size(); ++body)
            {
                const float difference = float(worldPose[nodes[body]].GetW().GetY()) - clipPelvisY - heights[body];
                error += difference * difference;
            }
            if (error < minimumError)
            {
                minimumError = error;
                bestClip = clip;
            }
        }
        if (bestClip == UINT32_MAX)
        {
            m_PhysicsError = "Could not match an airborne fall pose.";
            return false;
        }
        if (!IsActiveRagdoll())
        {
            Character* character = dynamic_cast<Character*>(&GetOwner());
            if (!character || !character->SetPhysicsMode(PhysicsAnimationMode::ActiveRagdoll, m_PhysicsError))
            {
                return false;
            }
        }
        if (!m_Ragdoll.SetRootSupported(false, m_PhysicsError))
        {
            return false;
        }
        m_Ragdoll.SetFootDrivenSupport(false);
        if (m_AutomaticRootSupport)
        {
            m_RootSupportRelease = RootSupportRelease::NoSupport;
        }
        m_FootSupport.ResetFootstep();
        ResetRecovery();
        ResetFallArms();
        m_Ragdoll.ResetDriveTargetHistory();
        m_KinematicRecoveryTimer.Reset();
        m_StandingUpStableTime = 0;
        m_PendingRecoveryImpact = false;
        if (!m_AnimInstance->TransitionToAnimation(bestClip, .15f, true))
        {
            m_PhysicsError = "Could not start the airborne fall loop.";
            return false;
        }
        m_BalanceState = CharacterBalanceState::Falling;
        return true;
    }
}
