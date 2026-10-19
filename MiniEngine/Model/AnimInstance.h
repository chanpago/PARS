#pragma once

#include "Animation.h"
#include <memory>
#include <string>

class Model;

namespace PARS
{
    class SkeletalMeshComponent;

    // Actor의 컴포넌트 목록이 아니라 SkeletalMeshComponent가 소유한다.
    // CPU 포즈를 계산하며, GPU 스키닝과 업로드는 메시 컴포넌트가 담당한다.
    class AnimInstance
    {
    public:
        explicit AnimInstance(SkeletalMeshComponent& owner)
            : m_Owner(owner)
        {
        }
        virtual ~AnimInstance() = default;
        AnimInstance(const AnimInstance&) = delete;
        AnimInstance& operator=(const AnimInstance&) = delete;

        SkeletalMeshComponent& GetOwningComponent() const
        {
            return m_Owner;
        }
        void Initialize(std::shared_ptr<const Model> skeleton);
        bool AddAnimation(const std::string& name, std::shared_ptr<const Model> source, uint32_t animationIndex = 0);
        bool PlayAnimation(uint32_t index, bool loop = true);
        bool TransitionToAnimation(uint32_t index, float duration = 0.2f, bool loop = true);
        bool IsBlending() const
        {
            return !m_BlendSource.empty();
        }
        float GetBlendAlpha() const
        {
            return IsBlending() ? Math::Clamp(m_BlendElapsed / m_BlendDuration, 0.0f, 1.0f) : 1.0f;
        }
        void SetPlaying(bool playing)
        {
            m_Playing = playing && !m_Clips.empty();
        }
        bool IsPlaying() const
        {
            return m_Playing;
        }
        void Stop();
        void Reset();
        void UpdateAnimation(float deltaTime);

        size_t GetAnimationCount() const
        {
            return m_Clips.size();
        }
        uint32_t GetCurrentAnimationIndex() const
        {
            return m_CurrentClip;
        }
        float GetAnimationTime() const
        {
            return m_Time;
        }
        float GetAnimationDuration(uint32_t index) const;
        const std::string& GetAnimationName(uint32_t index) const;
        const std::vector<GraphNode>& GetPose() const
        {
            return m_Pose;
        }

    protected:
        virtual void NativeUpdateAnimation(float deltaTime)
        {
            (void)deltaTime;
        }

    private:
        struct Clip
        {
            std::string name;
            std::shared_ptr<const Model> source;
            uint32_t animationIndex;
        };
        void EvaluatePose();
        SkeletalMeshComponent& m_Owner;
        std::shared_ptr<const Model> m_Skeleton;
        std::vector<Clip> m_Clips;
        std::vector<GraphNode> m_Pose;
        std::vector<GraphNode> m_BlendSource;
        float m_BlendElapsed = 0.0f;
        float m_BlendDuration = 0.2f;
        uint32_t m_CurrentClip = 0;
        float m_Time = 0.0f;
        bool m_Playing = false;
        bool m_Loop = true;
    };
}
