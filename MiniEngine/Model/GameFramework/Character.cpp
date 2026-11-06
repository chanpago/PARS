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
        m_Capsule->SetHorizontalBlockingEnabled(!m_Mesh->IsActiveRagdoll());
        m_Impact->RefreshCollisionPolicy();
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
        // 이전 프레임 이후 UI에서 골반 지지 설정이 바뀌었을 수 있다.
        SyncRagdollMovement();
        Pawn::PreparePhysics(deltaTime);
    }

    void Character::FixedTick(float deltaTime)
    {
        SyncRagdollMovement();
        m_Impact->BeginFixedStep(deltaTime);
        Pawn::FixedTick(deltaTime); // 이동을 먼저 처리한 다음 애니메이션과 드라이브 목표를 갱신한다.
        // 목표 계산 중 같은 서브스텝 안에서 Passive 모드로 전환될 수 있다.
        SyncRagdollMovement();
    }

    void Character::ApplyPhysicsInputs(float deltaTime)
    {
        Pawn::ApplyPhysicsInputs(deltaTime);
        // 모든 캐릭터의 이동·PD 목표 갱신 뒤, 독립 캡슐에 몸통의 현재 자세와 속도만 복사한다.
        m_Mesh->GetRagdoll().SyncCharacterCapsule();
    }

    bool Character::StopRagdoll(std::string& error)
    {
        if (!IsRagdoll())
        {
            error.clear();
            return true;
        }
        // 최신 이동 목표 위치에서 애니메이션으로 복귀한다. 일어서기 블렌딩은 수행하지 않는다.
        if (!m_Capsule->SetPhysicsEnabled(m_CapsuleWasEnabled))
        {
            error = "Could not restore the character capsule.";
            return false;
        }
        m_Mesh->StopRagdoll();
        m_Capsule->SetHorizontalBlockingEnabled(true);
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
        m_Impact = &CreateDefaultSubobject<CharacterImpactComponent>("CharacterImpactComponent");
        m_Movement->SetHorizontalMoveConstraint([this](Math::Vector3 displacement, float deltaTime) -> Math::Vector3
        {
            if (!m_Mesh->IsActiveRagdoll())
            {
                return displacement;
            }
            // 환경 장애물을 먼저 반영하고, 캐릭터 판정은 실제 몸의 속도로 수행한다.
            const Math::Vector3 environmentMove = m_Mesh->ConstrainHorizontalMovement(displacement);
            return m_Impact->ConstrainMovement(environmentMove, deltaTime);
        });
        // Actor 원점은 캡슐 중심이며, 가져온 메시의 원점은 발 위치에 있다.
        m_Mesh->SetRelativeLocation(Math::Vector3(0, -m_Capsule->GetHalfHeight(), 0));
    }
}
