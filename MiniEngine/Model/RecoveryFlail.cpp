#include "RecoveryFlail.h"
#include <cmath>
#include <utility>

namespace PARS
{
    namespace
    {
        std::string BoneName(const std::string& name)
        {
            const size_t separator = name.find_last_of(":|");
            return separator == std::string::npos ? name : name.substr(separator + 1);
        }

        float UpperBodyWeight(const std::string& name)
        {
            if (name == "Spine")
            {
                return .2f;
            }
            if (name == "Spine1")
            {
                return .35f;
            }
            if (name == "Spine2")
            {
                return .5f;
            }
            if (name == "Neck" || name == "Head" || name == "HeadTop_End")
            {
                return .25f;
            }
            return 1.0f;
        }
    }

    bool RecoveryFlail::Configure(const std::vector<std::string>& names, const std::vector<int32_t>& parents,
        uint32_t frontAnimation, uint32_t backAnimation)
    {
        if (names.empty() || names.size() != parents.size() || frontAnimation == backAnimation ||
            frontAnimation == UINT32_MAX || backAnimation == UINT32_MAX)
        {
            return false;
        }
        std::vector<float> weights(names.size(), 0);
        bool foundSpine = false;
        for (size_t node = 0; node < names.size(); ++node)
        {
            const int32_t parent = parents[node];
            if (parent < -1 || parent >= static_cast<int32_t>(node))
            {
                return false;
            }
            const std::string bone = BoneName(names[node]);
            if (bone == "Spine")
            {
                foundSpine = true;
                weights[node] = UpperBodyWeight(bone);
            }
            else if (parent >= 0 && weights[parent] > 0)
            {
                weights[node] = UpperBodyWeight(bone);
            }
        }
        if (!foundSpine)
        {
            return false;
        }
        m_BoneWeights = std::move(weights);
        m_FrontAnimation = frontAnimation;
        m_BackAnimation = backAnimation;
        Reset();
        return true;
    }

    void RecoveryFlail::Clear()
    {
        m_BoneWeights.clear();
        m_FrontAnimation = UINT32_MAX;
        m_BackAnimation = UINT32_MAX;
        Reset();
    }

    void RecoveryFlail::Reset()
    {
        m_Weight = 0;
        m_FrontWeight = .5f;
        m_TargetFrontWeight = .5f;
        m_Phase = 0;
        m_HasImpactDirection = false;
    }

    void RecoveryFlail::RecordImpact(Math::Vector3 impulse, Math::Vector3 forward)
    {
        const float impulseX = static_cast<float>(impulse.GetX());
        const float impulseZ = static_cast<float>(impulse.GetZ());
        const float forwardX = static_cast<float>(forward.GetX());
        const float forwardZ = static_cast<float>(forward.GetZ());
        const float impulseLength = std::hypot(impulseX, impulseZ);
        const float forwardLength = std::hypot(forwardX, forwardZ);
        if (!IsConfigured() || !std::isfinite(impulseLength) || !std::isfinite(forwardLength) ||
            impulseLength <= 1e-5f || forwardLength <= 1e-5f)
        {
            return;
        }
        // 충격이 온 위치가 아니라, 캐릭터가 실제로 밀리는 수평 방향을 사용한다.
        const float direction = (impulseX / impulseLength) * (forwardX / forwardLength) +
            (impulseZ / impulseLength) * (forwardZ / forwardLength);
        m_TargetFrontWeight = .5f * (Math::Clamp(direction, -1.0f, 1.0f) + 1.0f);
        if (!m_HasImpactDirection)
        {
            m_FrontWeight = m_TargetFrontWeight;
        }
        m_HasImpactDirection = true;
    }

    void RecoveryFlail::Update(bool recovering, float deltaTime, float frontDuration, float backDuration)
    {
        if (!recovering || !IsConfigured())
        {
            Reset();
            return;
        }
        if (!std::isfinite(deltaTime) || deltaTime <= 0 || !std::isfinite(frontDuration) ||
            !std::isfinite(backDuration) || frontDuration <= 0 || backDuration <= 0)
        {
            return;
        }
        const float directionAlpha = 1.0f - std::exp(-deltaTime / DirectionBlendDuration);
        m_FrontWeight = Math::Lerp(m_FrontWeight, m_TargetFrontWeight, directionAlpha);
        m_Weight = Math::Min(m_Weight + deltaTime / BlendInDuration, 1.0f);
        // 길이가 다른 두 클립을 같은 진행률로 샘플링해 앞뒤 혼합 중 위상이 벌어지지 않게 한다.
        const float duration = Math::Lerp(backDuration, frontDuration, m_FrontWeight);
        m_Phase = std::fmod(m_Phase + deltaTime / duration, 1.0f);
    }

    bool RecoveryFlail::Apply(const std::vector<GraphNode>& front, const std::vector<GraphNode>& back,
        std::vector<GraphNode>& pose) const
    {
        if (!IsConfigured() || m_Weight <= 0)
        {
            return true;
        }
        if (front.size() != m_BoneWeights.size() || back.size() != pose.size() || front.size() != pose.size())
        {
            return false;
        }
        for (size_t node = 0; node < pose.size(); ++node)
        {
            const float weight = m_BoneWeights[node] * m_Weight;
            if (weight <= 0)
            {
                continue;
            }
            const Math::Quaternion rotation = Math::Slerp(back[node].rotation, front[node].rotation, m_FrontWeight);
            pose[node].rotation = Math::Normalize(Math::Slerp(pose[node].rotation, rotation, weight));
            pose[node].staleMatrix = true;
        }
        return true;
    }
}
