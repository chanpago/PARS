#include "SkeletalMeshComponent.h"

using namespace Math;

namespace PARS
{
    void SkeletalMeshComponent::ResetCrossStepPlayback()
    {
        m_CrossStepPlayback = {};
        m_CrossStepTransitionStep = 0;
        m_CrossStepBlendSource.clear();
        m_CrossStepPose.clear();
    }

    bool SkeletalMeshComponent::ApplyCrossStepDrivePose(const FootstepSnapshot& step, float deltaTime,
        std::vector<Matrix4>& target)
    {
        const FootstepCommand* selected = nullptr;
        int movingFoot = -1;
        for (size_t side = 0; side < step.feet.size(); ++side)
        {
            const FootstepCommand& command = step.feet[side];
            // 매 스텝의 착지 후보가 잠시 비어도 이 명령에서 이미 선택한 포즈는 계속 진행한다.
            if (command.IsInProgress() && command.target.recovery.mode != FootstepMode::Normal &&
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
            if (m_CrossStepPlayback.searchStep != selected->motionSearchStep ||
                m_CrossStepPlayback.movingFoot != movingFoot || m_CrossStepPlayback.blendingOut)
            {
                // 연속 딛기에서도 직전 물리 목표에서 새 클립으로 이어진다.
                m_CrossStepBlendSource = m_CrossStepPose.empty() ? base : m_CrossStepPose;
                m_CrossStepTransitionStep = step.completedStep;
                m_CrossStepPlayback.searchStep = selected->motionSearchStep;
                m_CrossStepPlayback.clipIndex = selected->motionMatch.clipIndex;
                m_CrossStepPlayback.movingFoot = movingFoot;
                m_CrossStepPlayback.blendingOut = false;
            }
            const float elapsed = static_cast<float>(step.completedStep - selected->motionSearchStep) * deltaTime;
            m_CrossStepPlayback.timeSeconds = m_AnimInstance->GetCrossStepPlaybackTime(selected->motionMatch, elapsed);
            if (!m_AnimInstance->SampleAnimationPose(m_CrossStepPlayback.clipIndex, m_CrossStepPlayback.timeSeconds, m_CrossStepPose))
            {
                ResetCrossStepPlayback();
                return true;
            }
        }
        else
        {
            if (m_CrossStepPose.empty())
            {
                return true;
            }
            if (!m_CrossStepPlayback.blendingOut)
            {
                // 착지하거나 일반딛기로 바뀌면 마지막으로 구동한 자세에서 현재 기본 자세로 복귀한다.
                m_CrossStepBlendSource = m_CrossStepPose;
                m_CrossStepTransitionStep = step.completedStep;
                m_CrossStepPlayback.blendingOut = true;
            }
            m_CrossStepPose = base;
        }

        constexpr float blendDuration = 0.1f;
        const float blendTime = (static_cast<float>(step.completedStep - m_CrossStepTransitionStep) + 1.0f) * deltaTime;
        const float t = Min(blendTime / blendDuration, 1.0f);
        m_CrossStepPlayback.blendAlpha = t * t * (3.0f - 2.0f * t);
        if (t >= 1.0f && m_CrossStepPlayback.blendingOut)
        {
            ResetCrossStepPlayback();
            return true;
        }
        BlendAnimationPoses(m_CrossStepBlendSource.data(), m_CrossStepPose.data(),
            static_cast<uint32_t>(m_CrossStepPose.size()), m_CrossStepPlayback.blendAlpha);

        const int rootBody = m_Ragdoll.GetImpactRootBody();
        const std::vector<int>& nodes = m_Ragdoll.GetBoneNodes();
        if (rootBody < 0 || static_cast<size_t>(rootBody) >= nodes.size() ||
            nodes[rootBody] < 0 || static_cast<size_t>(nodes[rootBody]) >= target.size())
        {
            ResetCrossStepPlayback();
            return true;
        }
        const int pelvis = nodes[rootBody];
        const Vector3 basePelvis(target[pelvis].GetW());
        if (!EvaluatePhysicsPose(m_PhysicsSkeleton, m_CrossStepPose, Matrix4(GetComponentTransform()), target, m_PhysicsError))
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
