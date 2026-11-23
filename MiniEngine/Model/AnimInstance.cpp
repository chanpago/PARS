#include "AnimInstance.h"
#include "Model.h"
#include <algorithm>
#include <cmath>
#include <cstring>

namespace
{
    bool SameMatrix(const Math::Matrix4& a, const Math::Matrix4& b)
    {
        const float* lhs = reinterpret_cast<const float*>(&a);
        const float* rhs = reinterpret_cast<const float*>(&b);
        for (int i = 0; i < 16; ++i)
        {
            if (!std::isfinite(lhs[i]) || !std::isfinite(rhs[i]) || std::abs(lhs[i] - rhs[i]) > 1e-4f)
            {
                return false;
            }
        }
        return true;
    }

    // 가져온 클립은 메시와 동일한 노드/조인트 구성 및 바인드 포즈를 사용해야 한다.
    // 본 개수가 같다는 것만으로는 이 호환성을 보장할 수 없다.
    bool CompatibleSkeletons(const Model& mesh, const Model& clip)
    {
        if (mesh.m_NumNodes != clip.m_NumNodes || mesh.m_NumJoints != clip.m_NumJoints)
        {
            return false;
        }
        for (uint32_t i = 0; i < mesh.m_NumNodes; ++i)
        {
            const GraphNode& a = mesh.m_SceneGraph[i];
            const GraphNode& b = clip.m_SceneGraph[i];
            if (a.matrixIdx != b.matrixIdx || a.hasChildren != b.hasChildren ||
                a.hasSibling != b.hasSibling || a.skeletonRoot != b.skeletonRoot || !SameMatrix(a.xform, b.xform))
            {
                return false;
            }
        }
        for (uint32_t i = 0; i < mesh.m_NumJoints; ++i)
        {
            if (mesh.m_JointIndices[i] != clip.m_JointIndices[i] || !SameMatrix(mesh.m_JointIBMs[i], clip.m_JointIBMs[i]))
            {
                return false;
            }
        }
        return true;
    }
}

namespace PARS
{
    void AnimInstance::Initialize(std::shared_ptr<const Model> skeleton)
    {
        m_Skeleton = std::move(skeleton);
        m_Clips.clear();
        m_MotionDatabase.reset();
        m_CurrentClip = 0;
        m_Playing = false;
        m_Time = 0.0f;
        m_Pose.clear();
        m_BlendSource.clear();
        m_AdvanceBlendSource = false;
        EvaluatePose();
    }

    bool AnimInstance::AddAnimation(const std::string& name, std::shared_ptr<const Model> source, uint32_t animationIndex)
    {
        if (!m_Skeleton || !source || animationIndex >= source->m_NumAnimations ||
            !CompatibleSkeletons(*m_Skeleton, *source))
        {
            return false;
        }
        const float duration = source->m_Animations[animationIndex].duration;
        if (!std::isfinite(duration) || duration <= 0.0f)
        {
            return false;
        }
        m_Clips.push_back({name, std::move(source), animationIndex});
        return true;
    }

    bool AnimInstance::PlayAnimation(uint32_t index, bool loop, float startTime)
    {
        if (index >= m_Clips.size() || !std::isfinite(startTime) || startTime < 0.0f)
        {
            return false;
        }
        m_CurrentClip = index;
        m_Loop = loop;
        m_Playing = true;
        m_Time = loop ? std::fmod(startTime, GetAnimationDuration(index)) : Math::Min(startTime, GetAnimationDuration(index));
        m_BlendSource.clear();
        m_AdvanceBlendSource = false;
        EvaluatePose();
        return true;
    }

    void AnimInstance::Reset()
    {
        m_BlendSource.clear();
        m_AdvanceBlendSource = false;
        m_Time = 0.0f;
        EvaluatePose();
    }

    bool AnimInstance::TransitionToAnimation(uint32_t index, float duration, bool loop, float startTime,
        const std::vector<GraphNode>* sourcePose)
    {
        if (index >= m_Clips.size() || !std::isfinite(duration) || !std::isfinite(startTime) || startTime < 0.0f ||
            (sourcePose && sourcePose->size() != m_Pose.size()))
        {
            return false;
        }
        if (index == m_CurrentClip && m_Playing)
        {
            return true;
        }
        if (duration <= 0.0f || m_Pose.empty())
        {
            return PlayAnimation(index, loop, startTime);
        }
        // 일반 전환은 이전 클립도 계속 재생한다. 전환 도중 재전환할 때는 현재 혼합 포즈를
        // 시작점으로 보존해, 아직 덜 섞인 이전 목표 클립으로 자세가 튀지 않게 한다.
        m_AdvanceBlendSource = !sourcePose && m_Playing && !IsBlending();
        m_BlendSourceClip = m_CurrentClip;
        m_BlendSourceTime = m_Time;
        m_BlendSourceLoop = m_Loop;
        m_BlendSource = sourcePose ? *sourcePose : m_Pose;
        m_BlendElapsed = 0.0f;
        m_BlendDuration = duration;
        m_CurrentClip = index;
        m_Time = loop ? std::fmod(startTime, GetAnimationDuration(index)) : Math::Min(startTime, GetAnimationDuration(index));
        m_Loop = loop;
        m_Playing = true;
        EvaluatePose();
        return true;
    }

    bool AnimInstance::SampleAnimationPose(uint32_t index, float time, std::vector<GraphNode>& pose) const
    {
        if (!m_Skeleton || index >= m_Clips.size() || !std::isfinite(time) || time < 0.0f)
        {
            return false;
        }
        return EvaluateClipPose(index, Math::Min(time, GetAnimationDuration(index)), pose);
    }

    bool AnimInstance::ShareMotionFramesFrom(const AnimInstance& source, std::string& error)
    {
        if (!source.m_MotionDatabase || m_Skeleton != source.m_Skeleton)
        {
            error = u8"모션 후보를 공유하려면 같은 기준 스켈레톤과 생성된 후보 배열이 필요합니다.";
            return false;
        }

        size_t requiredCount = 0;
        for (const MotionFrameGroup& group : source.m_MotionDatabase->groups)
        {
            for (const MotionClipSelection& selection : group.clips)
            {
                const uint32_t index = selection.clipIndex;
                requiredCount = std::max(requiredCount, static_cast<size_t>(index) + 1);
                if (index < m_Clips.size() && (m_Clips[index].source != source.m_Clips[index].source ||
                    m_Clips[index].animationIndex != source.m_Clips[index].animationIndex))
                {
                    error = u8"모션 후보의 클립 번호가 다른 애니메이션을 가리킵니다.";
                    return false;
                }
            }
        }
        // 후보가 가리키는 번호를 유지하면서 아직 등록되지 않은 클립만 가져온다.
        if (m_Clips.size() < requiredCount)
        {
            m_Clips.insert(m_Clips.end(), source.m_Clips.begin() + m_Clips.size(), source.m_Clips.begin() + requiredCount);
        }
        m_MotionDatabase = source.m_MotionDatabase;
        error.clear();
        return true;
    }

    const std::vector<MotionFrame>& AnimInstance::GetMotionFrames() const
    {
        static const std::vector<MotionFrame> empty;
        return m_MotionDatabase ? m_MotionDatabase->frames : empty;
    }

    MotionFrameRange AnimInstance::GetMotionFrameRange(MotionFrameUsage usage) const
    {
        const size_t index = static_cast<size_t>(usage);
        if (!m_MotionDatabase || index >= m_MotionDatabase->groups.size())
        {
            return {};
        }
        return m_MotionDatabase->groups[index].range;
    }

    void AnimInstance::Stop()
    {
        m_Playing = false;
        Reset();
    }

    float AnimInstance::GetAnimationDuration(uint32_t index) const
    {
        if (index >= m_Clips.size())
        {
            return 0.0f;
        }
        const PARS::AnimInstance::Clip& clip = m_Clips[index];
        return clip.source->m_Animations[clip.animationIndex].duration;
    }

    const std::string& AnimInstance::GetAnimationName(uint32_t index) const
    {
        static const std::string empty;
        return index < m_Clips.size() ? m_Clips[index].name : empty;
    }

    void AnimInstance::UpdateAnimation(float deltaTime, bool allowClipSelection)
    {
        if (!std::isfinite(deltaTime) || deltaTime < 0.0f)
        {
            return;
        }

        // 어떤 애니메이션을 재생할지 갱신
        // 캐릭터 이동 속도로 대기·걷기·달리기를 선택
        if (allowClipSelection && m_Playing && !m_Clips.empty())
        {
            NativeUpdateAnimation(deltaTime);
        }
        // 파생 인스턴스는 훅 안에서 재생을 멈추거나 클립을 변경할 수 있다.
        if (m_Playing && !m_Clips.empty())
        {
            m_Time += deltaTime;
            if (IsBlending() && m_AdvanceBlendSource)
            {
                const float sourceDuration = GetAnimationDuration(m_BlendSourceClip);
                m_BlendSourceTime += deltaTime;
                m_BlendSourceTime = m_BlendSourceLoop ? std::fmod(m_BlendSourceTime, sourceDuration)
                    : Math::Min(m_BlendSourceTime, sourceDuration);
            }
            m_BlendElapsed += deltaTime;
            if (m_BlendElapsed >= m_BlendDuration)
            {
                m_BlendSource.clear();
                m_AdvanceBlendSource = false;
            }
            const float duration = GetAnimationDuration(m_CurrentClip);
            if (m_Loop)
            {
                m_Time = std::fmod(m_Time, duration);
            }
            else if (m_Time >= duration)
            {
                m_Time = duration;
                m_Playing = false;
            }
        }

        // 실제 본 자세 계산
        // clip에서 키프레임 데이터와 시간 간격 정보를 읽어와서 두 프레임 사이의 비율을 계산하고 m_pose에 저장
        EvaluatePose();
    }

    bool AnimInstance::EvaluateClipPose(uint32_t index, float time, std::vector<GraphNode>& pose) const
    {
        if (!m_Skeleton)
        {
            return false;
        }

        // 각 클립을 독립된 기준 포즈에서 샘플링해 직전 프레임의 혼합 결과가 누적되지 않게 한다.
        pose.resize(m_Skeleton->m_NumNodes);
        std::memcpy(pose.data(), m_Skeleton->m_SceneGraph.get(), pose.size() * sizeof(GraphNode));
        if (index < m_Clips.size())
        {
            const PARS::AnimInstance::Clip& clip = m_Clips[index];
            return EvaluateAnimationPose(
                clip.source->m_Animations[clip.animationIndex], // 사용할 애니메이션
                clip.source->m_CurveData.get(),                 // 어느 노드의 어떤 값을 바꿀지
                clip.source->m_KeyFrameData.get(),              // 저장된 키프레임 값
                pose.data(),                                    // 해당 클립만 샘플링한 로컬 포즈
                static_cast<uint32_t>(pose.size()), time);       // 샘플링할 재생 시간
        }
        return false;
    }

    void AnimInstance::EvaluatePose()
    {
        if (!m_Skeleton)
        {
            return;
        }
        EvaluateClipPose(m_CurrentClip, m_Time, m_Pose);
        if (IsBlending())
        {
            if (m_AdvanceBlendSource)
            {
                EvaluateClipPose(m_BlendSourceClip, m_BlendSourceTime, m_BlendSource);
            }
            BlendAnimationPoses(m_BlendSource.data(), m_Pose.data(), static_cast<uint32_t>(m_Pose.size()), GetBlendAlpha());
        }
    }
}
