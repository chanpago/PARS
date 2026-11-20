#include "SkeletalMeshComponent.h"

using namespace Math;

namespace PARS
{
    void SkeletalMeshComponent::ResetWalkingPlayback()
    {
        m_WalkingPlayback = {};
        m_WalkingTransitionStep = 0;
        m_WalkingBlendSource.clear();
        m_WalkingPose.clear();
    }

    bool SkeletalMeshComponent::ApplyWalkingDrivePose(const FootstepSnapshot& step, float deltaTime,
        std::vector<Matrix4>& target)
    {
        const FootstepCommand* selected = nullptr;
        int movingFoot = -1;
        for (size_t side = 0; side < step.feet.size(); ++side)
        {
            const FootstepCommand& command = step.feet[side];
            // 매 스텝의 착지 후보가 잠시 비어도 이 명령에서 이미 선택한 포즈는 계속 진행한다.
            if (command.IsInProgress() &&
                command.motionMatch.clipIndex < m_AnimInstance->GetAnimationCount() &&
                command.motionSearchStep <= step.completedStep &&
                (!selected || command.motionSearchStep > selected->motionSearchStep))
            {
                selected = &command;
                movingFoot = static_cast<int>(side);
            }
        }

        const std::vector<GraphNode>& base = m_AnimInstance->GetPose();
        if (selected)
        {
            if (m_WalkingPlayback.searchStep != selected->motionSearchStep ||
                m_WalkingPlayback.movingFoot != movingFoot || m_WalkingPlayback.blendingOut)
            {
                // 연속 딛기에서도 직전 물리 목표에서 새 클립으로 이어진다.
                m_WalkingBlendSource = m_WalkingPose.empty() ? base : m_WalkingPose;
                m_WalkingTransitionStep = step.completedStep;
                m_WalkingPlayback.searchStep = selected->motionSearchStep;
                m_WalkingPlayback.clipIndex = selected->motionMatch.clipIndex;
                m_WalkingPlayback.movingFoot = movingFoot;
                m_WalkingPlayback.blendingOut = false;
            }
            const float elapsed = static_cast<float>(step.completedStep - selected->motionSearchStep) * deltaTime;
            m_WalkingPlayback.timeSeconds = m_AnimInstance->GetWalkingPlaybackTime(selected->motionMatch, elapsed);
            if (!m_AnimInstance->SampleAnimationPose(m_WalkingPlayback.clipIndex, m_WalkingPlayback.timeSeconds, m_WalkingPose))
            {
                ResetWalkingPlayback();
                return true;
            }
        }
        else
        {
            if (m_WalkingPose.empty())
            {
                return true;
            }
            if (!m_WalkingPlayback.blendingOut)
            {
                // 딛기가 끝나면 마지막으로 구동한 자세에서 현재 기본 자세로 복귀한다.
                m_WalkingBlendSource = m_WalkingPose;
                m_WalkingTransitionStep = step.completedStep;
                m_WalkingPlayback.blendingOut = true;
            }
            m_WalkingPose = base;
        }

        constexpr float blendDuration = 0.1f;
        const float blendTime = (static_cast<float>(step.completedStep - m_WalkingTransitionStep) + 1.0f) * deltaTime;
        const float t = Min(blendTime / blendDuration, 1.0f);
        m_WalkingPlayback.blendAlpha = t * t * (3.0f - 2.0f * t);
        if (t >= 1.0f && m_WalkingPlayback.blendingOut)
        {
            ResetWalkingPlayback();
            return true;
        }
        BlendAnimationPoses(m_WalkingBlendSource.data(), m_WalkingPose.data(),
            static_cast<uint32_t>(m_WalkingPose.size()), m_WalkingPlayback.blendAlpha);

        const int rootBody = m_Ragdoll.GetImpactRootBody();
        const std::vector<int>& nodes = m_Ragdoll.GetBoneNodes();
        if (rootBody < 0 || static_cast<size_t>(rootBody) >= nodes.size() ||
            nodes[rootBody] < 0 || static_cast<size_t>(nodes[rootBody]) >= target.size())
        {
            ResetWalkingPlayback();
            return true;
        }
        const int pelvis = nodes[rootBody];
        const Vector3 basePelvis(target[pelvis].GetW());
        if (!EvaluatePhysicsPose(m_PhysicsSkeleton, m_WalkingPose, Matrix4(GetComponentTransform()), target, m_PhysicsError))
        {
            return false;
        }

        // 골반의 높이와 회전은 모캡에서 가져오고, 클립의 수평 이동은 실제 캐릭터에 더하지 않는다.
        Vector3 shift = basePelvis - Vector3(target[pelvis].GetW());
        shift.SetY(0.0f);
        for (Matrix4& bone : target)
        {
            bone.SetW(Vector4(Vector3(bone.GetW()) + shift, 1.0f));
        }
        return true;
    }
}
