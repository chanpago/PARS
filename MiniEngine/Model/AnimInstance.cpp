#include "AnimInstance.h"
#include "Model.h"
#include <cmath>
#include <cstring>

namespace
{
    bool SameMatrix(const Math::Matrix4& a, const Math::Matrix4& b)
    {
        const float* lhs = reinterpret_cast<const float*>(&a);
        const float* rhs = reinterpret_cast<const float*>(&b);
        for (int i = 0; i < 16; ++i)
            if (!std::isfinite(lhs[i]) || !std::isfinite(rhs[i]) || std::abs(lhs[i] - rhs[i]) > 1e-4f)
                return false;
        return true;
    }

    // Imported clips must address the same node/joint layout and bind pose as the
    // mesh. Matching bone counts alone cannot establish this compatibility.
    bool CompatibleSkeletons(const Model& mesh, const Model& clip)
    {
        if (mesh.m_NumNodes != clip.m_NumNodes || mesh.m_NumJoints != clip.m_NumJoints)
            return false;
        for (uint32_t i = 0; i < mesh.m_NumNodes; ++i)
        {
            const GraphNode& a = mesh.m_SceneGraph[i];
            const GraphNode& b = clip.m_SceneGraph[i];
            if (a.matrixIdx != b.matrixIdx || a.hasChildren != b.hasChildren ||
                a.hasSibling != b.hasSibling || a.skeletonRoot != b.skeletonRoot || !SameMatrix(a.xform, b.xform))
                return false;
        }
        for (uint32_t i = 0; i < mesh.m_NumJoints; ++i)
            if (mesh.m_JointIndices[i] != clip.m_JointIndices[i] || !SameMatrix(mesh.m_JointIBMs[i], clip.m_JointIBMs[i]))
                return false;
        return true;
    }
}

namespace PARS
{
    void AnimInstance::Initialize(std::shared_ptr<const Model> skeleton)
    {
        m_Skeleton = std::move(skeleton);
        m_Clips.clear();
        m_CurrentClip = 0;
        m_Playing = false;
        m_Time = 0.0f;
        m_Pose.clear();
        m_BlendSource.clear();
        EvaluatePose();
    }

    bool AnimInstance::AddAnimation(const std::string& name, std::shared_ptr<const Model> source, uint32_t animationIndex)
    {
        if (!m_Skeleton || !source || animationIndex >= source->m_NumAnimations ||
            !CompatibleSkeletons(*m_Skeleton, *source))
            return false;
        const float duration = source->m_Animations[animationIndex].duration;
        if (!std::isfinite(duration) || duration <= 0.0f)
            return false;
        m_Clips.push_back({ name, std::move(source), animationIndex });
        return true;
    }

    bool AnimInstance::PlayAnimation(uint32_t index, bool loop)
    {
        if (index >= m_Clips.size())
            return false;
        m_CurrentClip = index;
        m_Loop = loop;
        m_Playing = true;
        Reset();
        return true;
    }

    void AnimInstance::Reset()
    {
        m_BlendSource.clear();
        m_Time = 0.0f;
        EvaluatePose();
    }

    bool AnimInstance::TransitionToAnimation(uint32_t index, float duration, bool loop)
    {
        if (index >= m_Clips.size() || !std::isfinite(duration))
            return false;
        if (index == m_CurrentClip && m_Playing)
            return true;
        if (duration <= 0.0f || m_Pose.empty())
            return PlayAnimation(index, loop);
        // Snapshot the displayed pose, including an interrupted blend. Starting
        // another transition therefore never snaps back to a previous clip.
        m_BlendSource = m_Pose;
        m_BlendElapsed = 0.0f;
        m_BlendDuration = duration;
        m_CurrentClip = index;
        m_Time = 0.0f;
        m_Loop = loop;
        m_Playing = true;
        EvaluatePose();
        return true;
    }

    void AnimInstance::Stop()
    {
        m_Playing = false;
        Reset();
    }

    float AnimInstance::GetAnimationDuration(uint32_t index) const
    {
        if (index >= m_Clips.size())
            return 0.0f;
        const auto& clip = m_Clips[index];
        return clip.source->m_Animations[clip.animationIndex].duration;
    }

    const std::string& AnimInstance::GetAnimationName(uint32_t index) const
    {
        static const std::string empty;
        return index < m_Clips.size() ? m_Clips[index].name : empty;
    }

    void AnimInstance::UpdateAnimation(float deltaTime)
    {
        if (!std::isfinite(deltaTime) || deltaTime < 0.0f)
            return;
        if (m_Playing && !m_Clips.empty())
        {
            NativeUpdateAnimation(deltaTime);
        }
        // A derived instance may stop playback or change its clips in the hook.
        if (m_Playing && !m_Clips.empty())
        {
            m_Time += deltaTime;
            m_BlendElapsed += deltaTime;
            if (m_BlendElapsed >= m_BlendDuration)
                m_BlendSource.clear();
            const float duration = GetAnimationDuration(m_CurrentClip);
            if (m_Loop)
                m_Time = std::fmod(m_Time, duration);
            else if (m_Time >= duration)
            {
                m_Time = duration;
                m_Playing = false;
            }
        }
        EvaluatePose();
    }

    void AnimInstance::EvaluatePose()
    {
        if (!m_Skeleton)
            return;
        m_Pose.resize(m_Skeleton->m_NumNodes);
        std::memcpy(m_Pose.data(), m_Skeleton->m_SceneGraph.get(), m_Pose.size() * sizeof(GraphNode));
        if (!m_Clips.empty())
        {
            const auto& clip = m_Clips[m_CurrentClip];
            EvaluateAnimationPose(clip.source->m_Animations[clip.animationIndex], clip.source->m_CurveData.get(),
                clip.source->m_KeyFrameData.get(), m_Pose.data(), static_cast<uint32_t>(m_Pose.size()), m_Time);
        }
        if (IsBlending())
            BlendAnimationPoses(m_BlendSource.data(), m_Pose.data(), static_cast<uint32_t>(m_Pose.size()), GetBlendAlpha());
    }
}
