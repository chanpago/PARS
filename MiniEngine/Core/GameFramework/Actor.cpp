#include "Actor.h"
#include "SceneComponent.h"
#include "World.h"
#include <cmath>

namespace PARS
{
    Actor::~Actor()
    {
        EndPlay();
        m_RootComponent = nullptr;
        // Unlink while every sibling is still alive, independent of creation order.
        for (const auto& component : m_Components)
            if (auto scene = dynamic_cast<SceneComponent*>(component.get()))
                scene->DetachFromParent();
    }

    bool Actor::Destroy()
    {
        return m_World && m_World->DestroyActor(m_Handle);
    }

    bool Actor::OwnsComponent(const ActorComponent& component) const
    {
        for (const auto& owned : m_Components)
            if (owned.get() == &component)
                return true;
        return false;
    }

    bool Actor::SetRootComponent(SceneComponent& component)
    {
        if (!OwnsComponent(component) || component.GetAttachParent())
            return false;
        m_RootComponent = &component;
        return true;
    }

    Math::AffineTransform Actor::GetActorTransform() const
    {
        return m_RootComponent ? m_RootComponent->GetComponentTransform() : Math::AffineTransform(Math::kIdentity);
    }

    Math::Vector3 Actor::GetActorLocation() const { return GetActorTransform().GetTranslation(); }

    bool Actor::SetActorTransform(const Math::AffineTransform& transform)
    {
        return m_RootComponent && m_RootComponent->SetWorldTransform(transform);
    }

    bool Actor::SetActorLocation(Math::Vector3 location)
    {
        auto transform = GetActorTransform();
        transform.SetTranslation(location);
        return SetActorTransform(transform);
    }

    void Actor::BeginPlay()
    {
        if (m_BegunPlay || m_EndingPlay || m_PendingDestroy)
            return;
        m_BegunPlay = true;
        for (const auto& component : m_Components)
        {
            if (!m_BegunPlay || m_PendingDestroy)
                break;
            component->m_BegunPlay = true;
            component->BeginPlay();
        }
    }

    void Actor::EndPlay()
    {
        if (!m_BegunPlay)
            return;
        m_EndingPlay = true;
        m_BegunPlay = false;
        for (auto it = m_Components.rbegin(); it != m_Components.rend(); ++it)
        {
            if (!(*it)->m_BegunPlay)
                continue;
            (*it)->m_BegunPlay = false;
            (*it)->EndPlay();
        }
        m_EndingPlay = false;
    }

    void Actor::Tick(float deltaTime)
    {
        if (!m_BegunPlay || m_PendingDestroy || !std::isfinite(deltaTime) || deltaTime < 0.0f)
        {
			return;
        }
        
        for (const auto& component : m_Components)
        {
            if (!m_BegunPlay || m_PendingDestroy)
            {
                break;
            }

            if (component->IsTickEnabled())
            {
                component->TickComponent(deltaTime);
            }          
        }
    }

    void Actor::PreparePhysics(float deltaTime)
    {
        if (!m_BegunPlay || m_PendingDestroy)
        {
            return;
        }

        for (const auto& component : m_Components)
        {
            if (m_PendingDestroy || !m_BegunPlay)
            {
                break;
            }
            if (component->IsTickEnabled())
            {
                component->PreparePhysics(deltaTime);
            }
        }
    }

    void Actor::FixedTick(float deltaTime)
    {
        if (!m_BegunPlay || m_PendingDestroy)
        {
            return;
        }

        for (const auto& component : m_Components)
        {
            if (m_PendingDestroy || !m_BegunPlay)
            {
                break;
            }
            if (component->IsTickEnabled())
            {
                component->FixedTickComponent(deltaTime);
            }
        }
    }
}
