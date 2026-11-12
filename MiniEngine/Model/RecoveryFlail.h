#pragma once

#include "Animation.h"
#include <string>

namespace PARS
{
    // 기본 클립을 바꾸지 않고 회복 중인 상체의 로컬 회전 목표만 혼합한다.
    class RecoveryFlail
    {
    public:
        bool Configure(const std::vector<std::string>& names, const std::vector<int32_t>& parents,
            uint32_t frontAnimation, uint32_t backAnimation);
        void Clear();
        void Reset();
        void RecordImpact(Math::Vector3 impulse, Math::Vector3 forward);
        void Update(bool recovering, float deltaTime, float frontDuration, float backDuration);
        bool Apply(const std::vector<GraphNode>& front, const std::vector<GraphNode>& back,
            std::vector<GraphNode>& pose) const;

    public:
        bool IsConfigured() const
        {
            return !m_BoneWeights.empty();
        }
        bool HasImpactDirection() const
        {
            return m_HasImpactDirection;
        }
        uint32_t GetFrontAnimation() const
        {
            return m_FrontAnimation;
        }
        uint32_t GetBackAnimation() const
        {
            return m_BackAnimation;
        }
        float GetWeight() const
        {
            return m_Weight;
        }
        float GetFrontWeight() const
        {
            return m_FrontWeight;
        }
        float GetTargetFrontWeight() const
        {
            return m_TargetFrontWeight;
        }
        float GetPhase() const
        {
            return m_Phase;
        }

    private:
        static constexpr float BlendInDuration = .15f;
        static constexpr float DirectionBlendDuration = .1f;

    private:
        std::vector<float> m_BoneWeights;
        uint32_t m_FrontAnimation = UINT32_MAX;
        uint32_t m_BackAnimation = UINT32_MAX;
        float m_Weight = 0;
        float m_FrontWeight = .5f;
        float m_TargetFrontWeight = .5f;
        float m_Phase = 0;
        bool m_HasImpactDirection = false;
    };
}
