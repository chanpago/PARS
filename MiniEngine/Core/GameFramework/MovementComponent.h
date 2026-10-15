#pragma once
#include "Actor.h"
#include "SceneComponent.h"

namespace PARS
{
    class MovementComponent : public ActorComponent
    {
    public:
        using ActorComponent::ActorComponent;
        bool SetUpdatedComponent(SceneComponent& component)
        {
            if (&component.GetOwner() != &GetOwner() || !GetOwner().OwnsComponent(component)) return false;
            m_UpdatedComponent = &component;
            return true;
        }
        SceneComponent* GetUpdatedComponent() const { return m_UpdatedComponent; }
        Math::Vector3 GetVelocity() const { return m_Velocity; }
    protected:
        SceneComponent* m_UpdatedComponent = nullptr;
        Math::Vector3 m_Velocity{ Math::kZero };
    };
}
