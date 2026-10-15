#pragma once

#include "PrimitiveComponent.h"
#include <cmath>

namespace PARS
{
    // +Y-up capsule in metres. Half-height includes the hemispherical ends.
    // This component defines geometry/transform; it does not create a PhysX body yet.
    class CapsuleComponent : public PrimitiveComponent
    {
    public:
        using PrimitiveComponent::PrimitiveComponent;
        bool SetCapsuleSize(float radius, float halfHeight)
        {
            if (!std::isfinite(radius) || !std::isfinite(halfHeight) || radius <= 0 || halfHeight < radius)
                return false;
            m_Radius = radius;
            m_HalfHeight = halfHeight;
            return true;
        }
        float GetRadius() const { return m_Radius; }
        float GetHalfHeight() const { return m_HalfHeight; }
        float GetCylinderHalfHeight() const { return m_HalfHeight - m_Radius; }
        Math::BoundingSphere GetLocalBounds() const override
        {
            return Math::BoundingSphere(Math::Vector3(Math::kZero), m_HalfHeight);
        }

    private:
        float m_Radius = 0.3f;
        float m_HalfHeight = 0.9f;
    };
}
