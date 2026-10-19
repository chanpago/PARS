#pragma once
#include "PawnMovementComponent.h"

namespace PARS
{
    class CharacterMovementComponent : public PawnMovementComponent
    {
    public:
        using PawnMovementComponent::PawnMovementComponent;
        static constexpr float WalkSpeed = 1.6f;
        static constexpr float RunSpeed = 3.5f;
        bool IsMovingOnGround() const
        {
            return m_Grounded;
        }
        bool IsFalling() const
        {
            return !m_Grounded;
        }
        float GetGroundSpeed() const;
        void StopMovementImmediately();

    protected:
        void PreparePhysics(float deltaTime) override;
        void FixedTickComponent(float deltaTime) override;
        void EndPlay() override
        {
            StopMovementImmediately();
        }

    private:
        Math::Vector3 m_Input{Math::kZero};
        bool m_Grounded = false;
        float m_VerticalSpeed = 0;
    };
}
