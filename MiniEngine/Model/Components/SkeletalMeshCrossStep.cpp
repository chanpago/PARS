#include "SkeletalMeshComponent.h"

using namespace Math;

namespace PARS
{
    void SkeletalMeshComponent::ResetCrossStepPlayback()
    {
        m_CrossStepPlayback = {};
        m_CrossStepPose.clear();
    }

    void SkeletalMeshComponent::ApplyCrossStepDrivePose(const FootSupportSnapshot& support, float deltaTime,
        std::vector<Matrix4>& target)
    {
        const FootstepSnapshot& step = support.footstep;
        size_t side = 0;
        for (; side < step.feet.size(); ++side)
        {
            const FootstepCommand& command = step.feet[side];
            if (command.IsInProgress() && command.target.recovery.mode != FootstepMode::Normal)
            {
                break;
            }
        }
        if (side == step.feet.size())
        {
            ResetCrossStepPlayback();
            return;
        }

        const FootstepCommand& command = step.feet[side];
        CrossStepPlayback& playback = m_CrossStepPlayback;
        if (playback.startedStep != command.startedStep || playback.revision != step.revision || playback.movingFoot != side)
        {
            ResetCrossStepPlayback();
            playback.startedStep = command.startedStep;
            playback.revision = step.revision;
            playback.blendStartedStep = step.completedStep;
            playback.movingFoot = static_cast<uint32_t>(side);
            playback.match = m_AnimInstance->FindCrossStepMatch({playback.movingFoot, command.target.recovery.mode},
                command.motion.duration);
        }
        if (playback.match.clipIndex == UINT32_MAX)
        {
            return;
        }

        // 실제 이륙을 기준으로 선택 프레임에서 같은 발의 착지 프레임까지 진행한다.
        playback.timeSeconds = m_AnimInstance->GetWalkingPlaybackTime(playback.match,
            command.motion.progress * playback.match.remainingSwingTime);
        const std::vector<GraphNode>& base = m_AnimInstance->GetPose();
        if (!m_AnimInstance->SampleAnimationPose(playback.match.clipIndex, playback.timeSeconds, m_CrossStepPose) ||
            m_CrossStepPose.size() != base.size() || m_PhysicsSkeleton.parents.size() != base.size())
        {
            playback.match = {};
            m_CrossStepPose.clear();
            return;
        }
        const float elapsed = static_cast<float>(step.completedStep - playback.blendStartedStep + 1) * deltaTime;
        const float fadeTime = command.motion.duration * .25f;
        // 예정 시각이 지나도 실제 착지 전까지는 마지막 교차 자세를 유지한다.
        const float blend = Clamp(elapsed / fadeTime, 0.0f, 1.0f);
        playback.weight = blend * blend * (3.0f - 2.0f * blend);

        // 좌우 허벅지 아래의 로컬 포즈만 섞는다. 골반의 이동·회전과 상체는 기본 포즈에서 가져온다.
        const int leftHip = support.feet[0].leg.hipNode;
        const int rightHip = support.feet[1].leg.hipNode;
        for (size_t node = 0; node < base.size(); ++node)
        {
            int ancestor = static_cast<int>(node);
            while (ancestor >= 0 && ancestor != leftHip && ancestor != rightHip)
            {
                ancestor = m_PhysicsSkeleton.parents[ancestor];
            }
            if (ancestor < 0)
            {
                m_CrossStepPose[node] = base[node];
            }
            else
            {
                BlendAnimationPoses(&base[node], &m_CrossStepPose[node], 1, playback.weight);
            }
        }
        std::string error;
        if (!EvaluateAnimationPhysicsPose(m_PhysicsSkeleton, target, error, &m_CrossStepPose))
        {
            playback.match = {};
            m_CrossStepPose.clear();
        }
    }
}
