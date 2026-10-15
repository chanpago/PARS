#pragma once

#include "../../Core/GameFramework/Pawn.h"
#include "../../Core/GameFramework/CapsuleComponent.h"
#include "../Components/SkeletalMeshComponent.h"

namespace PARS
{
    class Character : public Pawn
    {
    public:
        explicit Character(const std::string& name = "Character");
        CapsuleComponent& GetCapsuleComponent() const { return *m_Capsule; }
        SkeletalMeshComponent& GetMesh() const { return *m_Mesh; }

    private:
        CapsuleComponent* m_Capsule;
        SkeletalMeshComponent* m_Mesh;
    };
}
