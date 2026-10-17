#include "CharacterMovementComponent.h"
#include "CapsuleComponent.h"
#include <cmath>

using namespace Math;
namespace PARS
{
    float CharacterMovementComponent::GetGroundSpeed() const
    { return float(Length(Vector3(m_Velocity.GetX(), 0, m_Velocity.GetZ()))); }

    void CharacterMovementComponent::StopMovementImmediately()
    {
        m_Input = m_Velocity = Vector3(kZero);
        m_VerticalSpeed = 0;
        m_Grounded = false;
        if (auto pawn = GetPawnOwner()) pawn->ConsumeMovementInput();
    }

    void CharacterMovementComponent::PreparePhysics(float)
    {
        auto pawn = GetPawnOwner();
        m_Input = pawn ? pawn->ConsumeMovementInput() : Vector3(kZero);
        m_Input.SetY(0);
        const float length = Length(m_Input);
        if (!std::isfinite(length)) m_Input = Vector3(kZero);
        else if (length > 1) m_Input = m_Input / length;
    }

    void CharacterMovementComponent::FixedTickComponent(float dt)
    {
        auto capsule = dynamic_cast<CapsuleComponent*>(GetUpdatedComponent());
        auto pawn = GetPawnOwner();
        if (!capsule || !pawn || !capsule->HasPhysicsCollision() || dt <= 0) return;
        const Vector3 desired = m_Input * (pawn->IsRunRequested() ? RunSpeed : WalkSpeed);
        Vector3 horizontal(m_Velocity.GetX(), 0, m_Velocity.GetZ());
        const Vector3 difference = desired - horizontal;
        const float distance = Length(difference);
        const float acceleration = float(LengthSquare(m_Input)) > 0.0001f ? 10.0f : 14.0f;
        if (distance > 0.00001f)
            horizontal += difference * Min(1.0f, acceleration * dt / distance);
        m_VerticalSpeed = m_Grounded && m_VerticalSpeed <= 0 ? -0.5f : Max(m_VerticalSpeed - 9.81f * dt, -50.0f);
        const Vector3 before = capsule->GetComponentTransform().GetTranslation();
        const auto collision = capsule->Move((horizontal + Vector3(0, m_VerticalSpeed, 0)) * dt, dt);
        m_Grounded = collision.grounded;
        if (collision.grounded || (collision.ceiling && m_VerticalSpeed > 0)) m_VerticalSpeed = 0;
        m_Velocity = (capsule->GetComponentTransform().GetTranslation() - before) / dt;
        m_Velocity.SetY(m_VerticalSpeed);
        if (GetGroundSpeed() > 0.05f)
        {
            auto transform = capsule->GetComponentTransform();
            const Vector3 forward = transform.GetZ();
            const float yaw = std::atan2(float(forward.GetX()), float(forward.GetZ()));
            const float target = std::atan2(float(m_Velocity.GetX()), float(m_Velocity.GetZ()));
            const float differenceYaw = std::atan2(std::sin(target - yaw), std::cos(target - yaw));
            transform.SetBasis(Matrix3::MakeYRotation(yaw + Clamp(differenceYaw, -12.56637f * dt, 12.56637f * dt)));
            capsule->SetWorldTransform(transform);
        }
    }
}
