#pragma once
#include "PawnMovementComponent.h"
#include <functional>

namespace PARS
{
    class CharacterMovementComponent : public PawnMovementComponent
    {
    public:
        using PawnMovementComponent::PawnMovementComponent;

    public:
        void StopMovementImmediately();

    protected:
        void PreparePhysics(float deltaTime) override;
        void FixedTickComponent(float deltaTime) override;
        void EndPlay() override
        {
            StopMovementImmediately();
        }

    public:
        bool IsMovingOnGround() const
        {
            return m_Grounded;
        }
        bool IsFalling() const
        {
            return !m_Grounded;
        }
        float GetGroundSpeed() const;
        float GetLocomotionSpeed() const;
        void SetHorizontalMoveConstraint(std::function<Math::Vector3(Math::Vector3)> constraint)
        {
            m_HorizontalMoveConstraint = std::move(constraint);
        }

    public:
        static constexpr float WalkSpeed = 1.6f;
        static constexpr float RunSpeed = 3.5f;

    private:
        Math::Vector3 m_Input{Math::kZero};
        Math::Vector3 m_LocomotionVelocity{Math::kZero};
        std::function<Math::Vector3(Math::Vector3)> m_HorizontalMoveConstraint;
        float m_VerticalSpeed = 0;
        bool m_Grounded = false;
    };
}
