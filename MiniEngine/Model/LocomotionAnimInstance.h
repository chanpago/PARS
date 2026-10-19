#pragma once

#include "AnimInstance.h"

namespace PARS
{
    // 클립 순서는 Idle, Walk, Run이다. 물리 처리 이후의 실제 캡슐 속도를 읽는다.
    class LocomotionAnimInstance : public AnimInstance
    {
    public:
        using AnimInstance::AnimInstance;
        bool IsLocomotionEnabled() const
        {
            return m_Locomotion;
        }
        void SetLocomotionEnabled(bool enabled)
        {
            m_Locomotion = enabled;
            if (enabled)
            {
                SetPlaying(true);
            }
        }

    protected:
        void NativeUpdateAnimation(float deltaTime) override;

    private:
        bool m_Locomotion = true;
    };
}
