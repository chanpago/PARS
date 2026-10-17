#include "Character.h"

namespace PARS
{
    bool Character::StartRagdoll(std::string& error)
    {
        if (!m_Mesh->StartRagdoll(m_Movement->GetVelocity(), error)) return false;
        m_MovementWasEnabled = m_Movement->IsTickEnabled();
        m_CapsuleWasEnabled = m_Capsule->IsPhysicsEnabled();
        m_Movement->StopMovementImmediately();
        m_Movement->SetTickEnabled(false);
        m_Capsule->SetPhysicsEnabled(false);
        return true;
    }

    bool Character::StopRagdoll(std::string& error)
    {
        if (!IsRagdoll()) { error.clear(); return true; }
        // The actor root stayed at the start position. Restore query collision there.
        if (!m_Capsule->SetPhysicsEnabled(m_CapsuleWasEnabled))
        { error = "Could not restore the character capsule."; return false; }
        m_Mesh->StopRagdoll();
        m_Movement->StopMovementImmediately();
        m_Movement->SetTickEnabled(m_MovementWasEnabled);
        error.clear(); return true;
    }

    Character::Character(const std::string& name) : Pawn(name)
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
