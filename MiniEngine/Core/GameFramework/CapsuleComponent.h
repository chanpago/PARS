#pragma once

#include "PrimitiveComponent.h"
#include <cmath>
namespace physx { class PxCapsuleController; }

namespace PARS
{
    // +Y-up capsule in metres. Half-height includes the hemispherical ends.
    // Owns its PhysX character controller when the owning World enables physics.
    class CapsuleComponent : public PrimitiveComponent
    {
    public:
        using PrimitiveComponent::PrimitiveComponent;
        ~CapsuleComponent() override;
        bool SetCapsuleSize(float radius, float halfHeight)
        {
            if (m_Controller || !std::isfinite(radius) || !std::isfinite(halfHeight) || radius <= 0 || halfHeight <= radius)
                return false;
            m_Radius = radius;
            m_HalfHeight = halfHeight;
            return true;
        }
        float GetRadius() const { return m_Radius; }
        float GetHalfHeight() const { return m_HalfHeight; }
        float GetCylinderHalfHeight() const { return m_HalfHeight - m_Radius; }
        bool HasPhysicsController() const { return m_Controller != nullptr; }
        struct MoveResult { bool grounded = false, ceiling = false, wall = false; };
        MoveResult Move(Math::Vector3 displacement, float deltaTime);
        Math::BoundingSphere GetLocalBounds() const override
        {
            return Math::BoundingSphere(Math::Vector3(Math::kZero), m_HalfHeight);
        }

    protected:
        void BeginPlay() override;
        void EndPlay() override;
    private:
        physx::PxCapsuleController* m_Controller = nullptr;
        float m_Radius = 0.3f;
        float m_HalfHeight = 0.9f;
    };
}
