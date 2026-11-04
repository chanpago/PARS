#pragma once
#include "Actor.h"
#include "SceneComponent.h"

namespace PARS
{
    class MovementComponent : public ActorComponent
    {
    public:
        using ActorComponent::ActorComponent;

    public:
        bool SetUpdatedComponent(SceneComponent& component)
        {
            if (&component.GetOwner() != &GetOwner() || !GetOwner().OwnsComponent(component))
            {
                return false;
            }
            m_UpdatedComponent = &component;
            return true;
        }
        SceneComponent* GetUpdatedComponent() const
        {
            return m_UpdatedComponent;
        }
        Math::Vector3 GetVelocity() const
        {
            return m_Velocity;
        }

    protected:
        Math::Vector3 m_Velocity{Math::kZero};
        SceneComponent* m_UpdatedComponent = nullptr;
    };
}
