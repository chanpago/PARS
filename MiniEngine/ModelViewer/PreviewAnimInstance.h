#pragma once

#include "AnimInstance.h"
#include <cmath>

class PreviewAnimInstance : public PARS::AnimInstance
{
public:
    using AnimInstance::AnimInstance;
    bool IsAutoCycleEnabled() const { return m_AutoCycle; }
    void SetAutoCycleEnabled(bool enabled) { m_AutoCycle = enabled; m_Elapsed = 0.0f; }
    bool SelectAnimation(uint32_t index)
    {
        const bool playing = IsPlaying();
        if (!PlayAnimation(index))
            return false;
        SetPlaying(playing);
        m_Elapsed = 0.0f;
        return true;
    }
    float GetTimeUntilNextAnimation() const { return GetDisplayDuration() - m_Elapsed; }

protected:
    void NativeUpdateAnimation(float deltaTime) override
    {
        if (!m_AutoCycle || GetAnimationCount() == 0)
            return;
        m_Elapsed += deltaTime;
        if (m_Elapsed >= GetDisplayDuration())
        {
            PlayAnimation((GetCurrentAnimationIndex() + 1) % static_cast<uint32_t>(GetAnimationCount()));
            m_Elapsed = 0.0f;
        }
    }

private:
    float GetDisplayDuration() const
    {
        const float duration = GetAnimationDuration(GetCurrentAnimationIndex());
        return duration > 0.0f ? duration * std::ceil(4.0f / duration) : 0.0f;
    }
    bool m_AutoCycle = true;
    float m_Elapsed = 0.0f;
};
