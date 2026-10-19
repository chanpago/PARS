#pragma once

#include "AnimInstance.h"

namespace PARS
{
    // Clip order: Idle, Walk, Run. Reads actual capsule velocity after physics.
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
