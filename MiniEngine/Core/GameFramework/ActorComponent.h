#pragma once

#include <string>
#include <utility>

namespace PARS
{
    class Actor;

    // Actor가 컴포넌트를 소유한다. 컴포넌트는 자신의 Actor나 부착 부모를 소유하지 않는다.
    class ActorComponent
    {
    public:
        ActorComponent(Actor& owner, std::string name)
            : m_Owner(owner), m_Name(std::move(name))
        {
        }
        virtual ~ActorComponent() = default;
        ActorComponent(const ActorComponent&) = delete;
        ActorComponent& operator=(const ActorComponent&) = delete;

        Actor& GetOwner() const
        {
            return m_Owner;
        }
        const std::string& GetName() const
        {
            return m_Name;
        }
        bool HasBegunPlay() const
        {
            return m_BegunPlay;
        }
        bool IsTickEnabled() const
        {
            return m_TickEnabled;
        }
        void SetTickEnabled(bool enabled)
        {
            m_TickEnabled = enabled;
        }

    protected:
        virtual void BeginPlay()
        {
        }
        virtual void EndPlay()
        {
        }
        virtual void TickComponent(float deltaTime)
        {
            (void)deltaTime;
        }
        virtual void PreparePhysics(float deltaTime)
        {
            (void)deltaTime;
        }
        virtual void FixedTickComponent(float deltaTime)
        {
            (void)deltaTime;
        }

    private:
        friend class Actor;
        Actor& m_Owner;
        std::string m_Name;
        bool m_BegunPlay = false;
        bool m_TickEnabled = true;
    };
}
