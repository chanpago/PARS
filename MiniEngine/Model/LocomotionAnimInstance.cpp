#include "LocomotionAnimInstance.h"
#include "GameFramework/Character.h"

namespace PARS
{
    void LocomotionAnimInstance::NativeUpdateAnimation(float)
    {
        auto character = dynamic_cast<Character*>(&GetOwningComponent().GetOwner());
        if (!m_Locomotion || !character || GetAnimationCount() < 3)
        {
            return;
        }
        const float speed = character->GetCharacterMovement().GetGroundSpeed();
        const uint32_t current = GetCurrentAnimationIndex();
        // Hysteresis avoids repeated transitions around the thresholds.
        const uint32_t next = speed < (current == 0 ? 0.15f : 0.08f) ? 0 : speed > (current == 2 ? 2.1f : 2.4f) ? 2
                                                                                                                : 1;
        if (next != current)
        {
            TransitionToAnimation(next);
        }
    }
}
