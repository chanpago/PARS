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
    public:
        bool operator==(ActorHandle other) const
        {
            return id == other.id;
        }
        bool operator!=(ActorHandle other) const
        {
            return !(*this == other);
        }

    public:
        uint64_t id = 0;
    };

    class Actor
    {
    public:
        enum class TickGroup
        {
            PrePhysics,
            PostPhysics
        };

    private:
        friend class World;

    public:
        explicit Actor(std::string name = "Actor")
            : m_Name(std::move(name))
        {
        }
        virtual ~Actor();
        Actor(const Actor&) = delete;

    public:
        // 생성 시점에 만드는 하위 객체는 주소가 유지되며, 수명은 Actor가 관리한다.
        template <class T, class... Args>
        T& CreateDefaultSubobject(const std::string& name, Args&&... args)
        {
            static_assert(std::is_base_of<ActorComponent, T>::value, "T must be a component");
            if (m_BegunPlay || m_EndingPlay || m_PendingDestroy)
            {
                throw std::logic_error("Create default subobjects before BeginPlay");
            }
            for (const std::unique_ptr<ActorComponent>& component : m_Components)
            {
                if (component->GetName() == name)
                {
                    throw std::invalid_argument("Duplicate component name");
                }
            }
            std::unique_ptr<T> component = std::make_unique<T>(*this, name, std::forward<Args>(args)...);
            T& result = *component;
            m_Components.push_back(std::move(component));
            return result;
        }
        bool Destroy();
        virtual void BeginPlay();
        virtual void EndPlay();
        virtual void Tick(float deltaTime);
        virtual void PreparePhysics(float deltaTime);
        virtual void FixedTick(float deltaTime);

    public:
        Actor& operator=(const Actor&) = delete;

    public:
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
        World* m_World = nullptr; // 이 Actor의 소유자는 World다.
        ActorHandle m_Handle;
        std::string m_Name;
        std::vector<std::unique_ptr<ActorComponent>> m_Components;
        SceneComponent* m_RootComponent = nullptr;
        TickGroup m_TickGroup = TickGroup::PostPhysics;
        bool m_PendingDestroy = false;
        bool m_BegunPlay = false;
        bool m_EndingPlay = false;
    };
}
