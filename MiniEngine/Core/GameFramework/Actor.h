#pragma once

#include "ActorComponent.h"
#include "../VectorMath.h"
#include <memory>
#include <cstdint>
#include <stdexcept>
#include <type_traits>
#include <vector>

namespace PARS
{
    class SceneComponent;
    class World;

    struct ActorHandle
    {
        uint64_t id = 0;
        bool operator==(ActorHandle other) const
        {
            return id == other.id;
        }
        bool operator!=(ActorHandle other) const
        {
            return !(*this == other);
        }
    };

    class Actor
    {
    public:
        explicit Actor(std::string name = "Actor")
            : m_Name(std::move(name))
        {
        }
        virtual ~Actor();
        Actor(const Actor&) = delete;
        Actor& operator=(const Actor&) = delete;

        // Construction-time subobjects have stable addresses and Actor-owned lifetimes.
        template<class T, class... Args>
        T& CreateDefaultSubobject(const std::string& name, Args&&... args)
        {
            static_assert(std::is_base_of<ActorComponent, T>::value, "T must be a component");
            if (m_BegunPlay || m_EndingPlay || m_PendingDestroy)
            {
                throw std::logic_error("Create default subobjects before BeginPlay");
            }
            for (const auto& component : m_Components)
            {
                if (component->GetName() == name)
                {
                    throw std::invalid_argument("Duplicate component name");
                }
            }
            auto component = std::make_unique<T>(*this, name, std::forward<Args>(args)...);
            T& result = *component;
            m_Components.push_back(std::move(component));
            return result;
        }

        const std::string& GetName() const
        {
            return m_Name;
        }
        World* GetWorld() const
        {
            return m_World;
        }
        ActorHandle GetHandle() const
        {
            return m_Handle;
        }
        bool Destroy();
        bool IsPendingDestroy() const
        {
            return m_PendingDestroy;
        }
        const std::vector<std::unique_ptr<ActorComponent>>& GetComponents() const
        {
            return m_Components;
        }
        bool OwnsComponent(const ActorComponent& component) const;
        SceneComponent* GetRootComponent() const
        {
            return m_RootComponent;
        }
        bool SetRootComponent(SceneComponent& component);

        Math::AffineTransform GetActorTransform() const;
        Math::Vector3 GetActorLocation() const;
        bool SetActorTransform(const Math::AffineTransform& transform);
        bool SetActorLocation(Math::Vector3 location);

        virtual void BeginPlay();
        virtual void EndPlay();
        virtual void Tick(float deltaTime);
        virtual void PreparePhysics(float deltaTime);
        virtual void FixedTick(float deltaTime);
        enum class TickGroup
        {
            PrePhysics,
            PostPhysics
        };
        TickGroup GetTickGroup() const
        {
            return m_TickGroup;
        }
        void SetTickGroup(TickGroup group)
        {
            m_TickGroup = group;
        }
        bool HasBegunPlay() const
        {
            return m_BegunPlay;
        }

    private:
        friend class World;
        World* m_World = nullptr; // World owns this Actor.
        ActorHandle m_Handle;
        bool m_PendingDestroy = false;
        std::string m_Name;
        std::vector<std::unique_ptr<ActorComponent>> m_Components;
        SceneComponent* m_RootComponent = nullptr;
        bool m_BegunPlay = false;
        bool m_EndingPlay = false;
        TickGroup m_TickGroup = TickGroup::PostPhysics;
    };
}
