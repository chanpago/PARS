#pragma once

#include "LocomotionAnimInstance.h"
#include <algorithm>
#include <cmath>

class PreviewAnimInstance : public PARS::LocomotionAnimInstance
{
public:
    PreviewAnimInstance(PARS::SkeletalMeshComponent& owner, uint32_t previewClipCount)
        : LocomotionAnimInstance(owner), m_PreviewClipCount(previewClipCount)
    {
    }

public:
    bool SelectAnimation(uint32_t index)
    {
        const bool playing = IsPlaying();
        if (!TransitionToAnimation(index))
        {
            return false;
        }
        SetLocomotionEnabled(false);
        m_AutoCycle = false;
        SetPlaying(playing);
        m_Elapsed = 0.0f;
        return true;
    }

protected:
    void NativeUpdateAnimation(float deltaTime) override
    {
        if (IsLocomotionEnabled())
        {
            LocomotionAnimInstance::NativeUpdateAnimation(deltaTime);
            return;
        }
        const uint32_t count = std::min(m_PreviewClipCount, static_cast<uint32_t>(GetAnimationCount()));
        if (!m_AutoCycle || count == 0)
        {
            return;
        }
        m_Elapsed += deltaTime;
        if (m_Elapsed >= GetDisplayDuration())
        {
            TransitionToAnimation((GetCurrentAnimationIndex() + 1) % count);
            m_Elapsed = 0.0f;
        }
    }

public:
    bool IsAutoCycleEnabled() const
    {
        return m_AutoCycle;
    }
    void SetAutoCycleEnabled(bool enabled)
    {
        m_AutoCycle = enabled;
        m_Elapsed = 0.0f;
    }
    float GetTimeUntilNextAnimation() const
    {
        return GetDisplayDuration() - m_Elapsed;
    }

private:
    float GetDisplayDuration() const
    {
        const float duration = GetAnimationDuration(GetCurrentAnimationIndex());
        return duration > 0.0f ? duration * std::ceil(4.0f / duration) : 0.0f;
    }

private:
    uint32_t m_PreviewClipCount;
    float m_Elapsed = 0.0f;
    bool m_AutoCycle = true;
};
