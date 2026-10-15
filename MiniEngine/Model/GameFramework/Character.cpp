#include "Character.h"

namespace PARS
{
    Character::Character(const std::string& name) : Pawn(name)
    {
        m_Capsule = &CreateDefaultSubobject<CapsuleComponent>("CapsuleComponent");
        SetRootComponent(*m_Capsule);
        m_Mesh = &CreateDefaultSubobject<SkeletalMeshComponent>("SkeletalMeshComponent");
        m_Mesh->AttachToComponent(*m_Capsule);
        // Actor origin is capsule centre; the imported mesh origin is at its feet.
        m_Mesh->SetRelativeLocation(Math::Vector3(0, -m_Capsule->GetHalfHeight(), 0));
    }
}
