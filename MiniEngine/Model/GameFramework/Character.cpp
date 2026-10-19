#include "Character.h"

namespace PARS
{
    bool Character::StartRagdoll(std::string& error)
    {
        return SetPhysicsMode(PhysicsAnimationMode::PassiveRagdoll, error);
    }

    bool Character::SetPhysicsMode(PhysicsAnimationMode mode, std::string& error)
    {
        if (mode == PhysicsAnimationMode::Animation)
        {
            return StopRagdoll(error);
        }
        const bool wasSimulating = IsRagdoll();
        if (!m_Mesh->SetPhysicsMode(mode, m_Movement->GetVelocity(), error))
        {
            return false;
        }
        if (!wasSimulating)
        {
            m_MovementWasEnabled = m_Movement->IsTickEnabled();
            m_CapsuleWasEnabled = m_Capsule->IsPhysicsEnabled();
        }
        SyncRagdollMovement();
        return true;
    }

    bool Character::CanControlMovement() const
    {
        if (!IsRagdoll())
        {
            return m_Movement->IsTickEnabled() && m_Capsule->IsPhysicsEnabled();
        }
        return m_Mesh->IsActiveRagdoll() && m_Mesh->GetRagdoll().IsRootSupported() &&
            m_MovementWasEnabled && m_CapsuleWasEnabled;
    }

    void Character::SyncRagdollMovement()
    {
        if (!IsRagdoll())
        {
            return;
        }
        const bool enabled = CanControlMovement();
        if (!enabled)
        {
            m_Movement->StopMovementImmediately();
        }
        if (m_Capsule->IsPhysicsEnabled() != enabled)
        {
            m_Capsule->SetPhysicsEnabled(enabled);
        }
        m_Movement->SetTickEnabled(enabled && m_Capsule->HasPhysicsCollision());
    }

    void Character::PreparePhysics(float deltaTime)
    {
        // Support may have been toggled in the UI since the previous frame.
        SyncRagdollMovement();
        Pawn::PreparePhysics(deltaTime);
    }

    void Character::FixedTick(float deltaTime)
    {
        SyncRagdollMovement();
        Pawn::FixedTick(deltaTime); // Movement first, then animation and drive targets.
        // Target evaluation can fall back to Passive during this same substep.
        SyncRagdollMovement();
    }

    bool Character::StopRagdoll(std::string& error)
    {
        if (!IsRagdoll())
        {
            error.clear();
            return true;
        }
        // Return to animation at the latest movement target (no get-up blend).
        if (!m_Capsule->SetPhysicsEnabled(m_CapsuleWasEnabled))
        {
            error = "Could not restore the character capsule.";
            return false;
        }
        m_Mesh->StopRagdoll();
        m_Movement->StopMovementImmediately();
        m_Movement->SetTickEnabled(m_MovementWasEnabled);
        error.clear();
        return true;
    }

    Character::Character(const std::string& name)
        : Pawn(name)
    {
        m_Capsule = &CreateDefaultSubobject<CapsuleComponent>("CapsuleComponent");
        SetRootComponent(*m_Capsule);
        m_Movement = &CreateDefaultSubobject<CharacterMovementComponent>("CharacterMovementComponent");
        m_Movement->SetUpdatedComponent(*m_Capsule);
        m_Mesh = &CreateDefaultSubobject<SkeletalMeshComponent>("SkeletalMeshComponent");
        m_Mesh->AttachToComponent(*m_Capsule);
        // Actor origin is capsule centre; the imported mesh origin is at its feet.
        m_Mesh->SetRelativeLocation(Math::Vector3(0, -m_Capsule->GetHalfHeight(), 0));
    }
}
