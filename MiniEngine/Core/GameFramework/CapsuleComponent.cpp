#include "CapsuleComponent.h"
#include "Actor.h"
#include "World.h"
#if defined(_M_X64)
#include "../PhysicsWorld.h"
#endif

namespace PARS
{
    CapsuleComponent::~CapsuleComponent()
    {
        EndPlay();
    }
    bool CapsuleComponent::SetPhysicsEnabled(bool enabled)
    {
        m_PhysicsEnabled = enabled;
        if (!enabled)
        {
            EndPlay();
        }
        else if (HasBegunPlay())
        {
            BeginPlay();
        }
        return !enabled || !HasBegunPlay() || HasPhysicsCollision();
    }
    void CapsuleComponent::BeginPlay()
    {
#if defined(_M_X64)
        auto world = GetOwner().GetWorld();
        auto physics = world ? world->GetPhysicsWorld() : nullptr;
        if (!m_PhysicsEnabled || !physics || !physics->IsInitialized() || m_QueryCapsule)
        {
            return;
        }
        // 캐릭터 캡슐은 수직으로 서 있으며, 미터 단위와 단위 스케일을 사용한다.
        if (GetAttachParent() || GetOwner().GetRootComponent() != this)
        {
            return;
        }
        const auto transform = GetComponentTransform();
        if (std::abs(float(Math::LengthSquare(transform.GetX())) - 1.0f) > 0.001f ||
            std::abs(float(Math::LengthSquare(transform.GetZ())) - 1.0f) > 0.001f ||
            float(Math::LengthSquare(transform.GetY() - Math::Vector3(Math::kYUnitVector))) > 0.000001f ||
            std::abs(float(Math::Dot(transform.GetX(), transform.GetZ()))) > 0.001f)
        {
            return;
        }
        const auto position = transform.GetTranslation();
        m_QueryCapsule = physics->CreateQueryCapsule(
            physx::PxVec3(float(position.GetX()), float(position.GetY()), float(position.GetZ())), m_Radius, m_HalfHeight);
        if (m_QueryCapsule)
        {
            m_QueryCapsule->owner = &GetOwner();
        }
        m_PhysicsWorld = physics;
#endif
    }
    void CapsuleComponent::EndPlay()
    {
#if defined(_M_X64)
        if (m_QueryCapsule)
        {
            m_PhysicsWorld->DestroyQueryCapsule(m_QueryCapsule);
        }
#endif
        m_QueryCapsule = nullptr;
        m_PhysicsWorld = nullptr;
    }
    void CapsuleComponent::SyncQueryPosition()
    {
#if defined(_M_X64)
        if (!m_QueryCapsule)
        {
            return;
        }
        const auto position = GetComponentTransform().GetTranslation();
        const physx::PxVec3 centre(float(position.GetX()), float(position.GetY()), float(position.GetZ()));
        if (centre.isFinite())
        {
            m_QueryCapsule->centre = centre;
        }
#endif
    }
    CapsuleComponent::MoveResult CapsuleComponent::Move(Math::Vector3 displacement, float deltaTime)
    {
        MoveResult result;
#if defined(_M_X64)
        if (!m_QueryCapsule || deltaTime <= 0 || !std::isfinite(deltaTime))
        {
            return result;
        }
        auto transform = GetComponentTransform();
        const auto current = transform.GetTranslation();
        const physx::PxVec3 requested(float(current.GetX()), float(current.GetY()), float(current.GetZ()));
        if (!requested.isFinite())
        {
            return result;
        }
        const physx::PxVec3 delta(float(displacement.GetX()), float(displacement.GetY()), float(displacement.GetZ()));
        if (!delta.isFinite())
        {
            return result;
        }
        // Actor를 직접 수정하면 순간이동으로 처리한다. 일반 이동은 쿼리용 형상을 스윕한 뒤
        // 충돌을 해결한 중심 위치를 컴포넌트 계층에 다시 반영한다.
        m_QueryCapsule->centre = requested;
        const auto flags = m_PhysicsWorld->MoveQueryCapsule(*m_QueryCapsule, delta, m_HorizontalBlockingEnabled);
        result.grounded = flags.grounded;
        result.ceiling = flags.ceiling;
        result.wall = flags.wall;
        const auto position = m_QueryCapsule->centre;
        transform.SetTranslation(Math::Vector3(float(position.x), float(position.y), float(position.z)));
        SetWorldTransform(transform);
#else
        (void)displacement;
        (void)deltaTime;
#endif
        return result;
    }
}
