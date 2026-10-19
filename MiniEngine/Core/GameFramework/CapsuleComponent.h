#pragma once

#include "PrimitiveComponent.h"
#include <cmath>
struct PhysicsQueryCapsule;
class PhysicsWorld;

namespace PARS
{
    // 미터 단위의 +Y 위쪽 캡슐이다. 반높이에는 양 끝의 반구도 포함된다.
    // 이동 쿼리 전용 형상을 소유하며, 디버그 화면에서는 와이어 캡슐로 표시한다.
    class CapsuleComponent : public PrimitiveComponent
    {
    public:
        using PrimitiveComponent::PrimitiveComponent;
        ~CapsuleComponent() override;
        bool SetCapsuleSize(float radius, float halfHeight)
        {
            if (m_QueryCapsule || !std::isfinite(radius) || !std::isfinite(halfHeight) || radius <= 0 || halfHeight <= radius)
            {
                return false;
            }
            m_Radius = radius;
            m_HalfHeight = halfHeight;
            return true;
        }
        float GetRadius() const
        {
            return m_Radius;
        }
        float GetHalfHeight() const
        {
            return m_HalfHeight;
        }
        float GetCylinderHalfHeight() const
        {
            return m_HalfHeight - m_Radius;
        }
        bool HasPhysicsCollision() const
        {
            return m_QueryCapsule != nullptr;
        }
        bool SetPhysicsEnabled(bool enabled);
        bool IsPhysicsEnabled() const
        {
            return m_PhysicsEnabled;
        }
        struct MoveResult
        {
            bool grounded = false, ceiling = false, wall = false;
        };
        MoveResult Move(Math::Vector3 displacement, float deltaTime);
        Math::BoundingSphere GetLocalBounds() const override
        {
            return Math::BoundingSphere(Math::Vector3(Math::kZero), m_HalfHeight);
        }

    protected:
        void BeginPlay() override;
        void EndPlay() override;
        void PreparePhysics(float) override
        {
            SyncQueryPosition();
        }
        void TickComponent(float) override
        {
            SyncQueryPosition();
        }

    private:
        void SyncQueryPosition();
        PhysicsQueryCapsule* m_QueryCapsule = nullptr;
        PhysicsWorld* m_PhysicsWorld = nullptr;
        float m_Radius = 0.3f;
        float m_HalfHeight = 0.9f;
        bool m_PhysicsEnabled = true;
    };
}
