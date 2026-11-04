#pragma once

#include "AnimInstance.h"

namespace PARS
{
    // 클립 순서는 Idle, Walk, Run이다. 이동 입력을 추종하는 모션 속도를 읽는다.
    class LocomotionAnimInstance : public AnimInstance
    {
    public:
        using AnimInstance::AnimInstance;

    protected:
        void NativeUpdateAnimation(float deltaTime) override;

    public:
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

    private:
        bool m_Locomotion = true;
    };
}
