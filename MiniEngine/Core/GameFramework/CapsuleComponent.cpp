#include "CapsuleComponent.h"
#include "Actor.h"
#include "World.h"
#if defined(_M_X64)
#include "../PhysicsWorld.h"
#endif

namespace PARS
{
    CapsuleComponent::~CapsuleComponent() { EndPlay(); }
    void CapsuleComponent::BeginPlay()
    {
#if defined(_M_X64)
        auto world = GetOwner().GetWorld();
        auto physics = world ? world->GetPhysicsWorld() : nullptr;
        if (!physics || !physics->IsInitialized() || m_Controller) return;
        // Character capsules are upright, use metres, and have unit scale.
        if (GetAttachParent() || GetOwner().GetRootComponent() != this) return;
        const auto transform = GetComponentTransform();
        if (std::abs(float(Math::LengthSquare(transform.GetX())) - 1.0f) > 0.001f ||
            std::abs(float(Math::LengthSquare(transform.GetZ())) - 1.0f) > 0.001f ||
            float(Math::LengthSquare(transform.GetY() - Math::Vector3(Math::kYUnitVector))) > 0.000001f ||
            std::abs(float(Math::Dot(transform.GetX(), transform.GetZ()))) > 0.001f)
            return;
        const auto position = transform.GetTranslation();
        m_Controller = physics->CreateCapsuleController(
            physx::PxExtendedVec3(float(position.GetX()), float(position.GetY()), float(position.GetZ())), m_Radius, m_HalfHeight);
#endif
    }
    void CapsuleComponent::EndPlay()
    {
#if defined(_M_X64)
        if (m_Controller) m_Controller->release();
#endif
        m_Controller = nullptr;
    }
    CapsuleComponent::MoveResult CapsuleComponent::Move(Math::Vector3 displacement, float deltaTime)
    {
        MoveResult result;
#if defined(_M_X64)
        if (!m_Controller || deltaTime <= 0 || !std::isfinite(deltaTime)) return result;
        auto transform = GetComponentTransform();
        const auto current = transform.GetTranslation();
        const auto previous = m_Controller->getPosition();
        const physx::PxVec3 requested(float(current.GetX()), float(current.GetY()), float(current.GetZ()));
        if (!requested.isFinite()) return result;
        // External Actor position edits are explicit teleports; ordinary movement
        // always uses CCT sweeps and then copies the resolved position to the root.
        if ((physx::PxVec3(float(previous.x), float(previous.y), float(previous.z)) - requested).magnitudeSquared() > 1e-8f)
            m_Controller->setPosition(physx::PxExtendedVec3(requested.x, requested.y, requested.z));
        const physx::PxVec3 delta(float(displacement.GetX()), float(displacement.GetY()), float(displacement.GetZ()));
        if (!delta.isFinite()) return result;
        const auto flags = m_Controller->move(delta, 0.00001f, deltaTime, physx::PxControllerFilters());
        result.grounded = flags.isSet(physx::PxControllerCollisionFlag::eCOLLISION_DOWN);
        result.ceiling = flags.isSet(physx::PxControllerCollisionFlag::eCOLLISION_UP);
        result.wall = flags.isSet(physx::PxControllerCollisionFlag::eCOLLISION_SIDES);
        const auto position = m_Controller->getPosition();
        transform.SetTranslation(Math::Vector3(float(position.x), float(position.y), float(position.z)));
        SetWorldTransform(transform);
#else
        (void)displacement; (void)deltaTime;
#endif
        return result;
    }
}
