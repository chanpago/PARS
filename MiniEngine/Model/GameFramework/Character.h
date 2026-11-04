#pragma once

#include "../../Core/GameFramework/Pawn.h"
#include "../../Core/GameFramework/CapsuleComponent.h"
#include "../../Core/GameFramework/CharacterMovementComponent.h"
#include "../Components/SkeletalMeshComponent.h"

namespace PARS
{
    class Character : public Pawn
    {
    public:
        explicit Character(const std::string& name = "Character");

    public:
        bool StartRagdoll(std::string& error);
        bool SetPhysicsMode(PhysicsAnimationMode mode, std::string& error);
        bool StopRagdoll(std::string& error);
        void PreparePhysics(float deltaTime) override;
        void FixedTick(float deltaTime) override;

    private:
        void SyncRagdollMovement();

    public:
        CapsuleComponent& GetCapsuleComponent() const
        {
            return *m_Capsule;
        }
        CharacterMovementComponent& GetCharacterMovement() const
        {
            return *m_Movement;
        }
        SkeletalMeshComponent& GetMesh() const
        {
            return *m_Mesh;
        }
        bool IsRagdoll() const
        {
            return m_Mesh->IsRagdoll();
        }
        bool CanControlMovement() const;

    private:
        CapsuleComponent* m_Capsule;
        CharacterMovementComponent* m_Movement;
        SkeletalMeshComponent* m_Mesh;
        bool m_MovementWasEnabled = true;
        bool m_CapsuleWasEnabled = true;
    };
}
