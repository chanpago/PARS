#pragma once

#include "Actor.h"
#include <functional>

namespace PARS
{
    // Owns gameplay actors. PhysicsWorld remains the separate PhysX simulation.
    // Main-thread only; mutations during callbacks never invalidate a live Actor.
    class World
    {
    public:
        World() = default;
        ~World();
        World(const World&) = delete;
        World& operator=(const World&) = delete;

        template<class T, class... Args>
        T* SpawnActor(Args&&... args)
        {
            static_assert(std::is_base_of<Actor, T>::value, "T must be an Actor");
            if (m_Clearing)
                return nullptr;
            auto actor = std::make_unique<T>(std::forward<Args>(args)...);
            T* result = actor.get();
            RegisterActor(std::move(actor));
            return result;
        }

        // Handles can outlive an Actor. Resolving a destroyed actor returns null.
        Actor* FindActor(ActorHandle handle) const;
        template<class T> T* FindActor(ActorHandle handle) const
        {
            return dynamic_cast<T*>(FindActor(handle));
        }
        bool DestroyActor(ActorHandle handle);
        size_t GetActorCount() const;
        bool HasPendingDestroy() const;

        void BeginPlay();
        void Tick(float deltaTime);
        bool HasBegunPlay() const { return m_Playing; }

        // New actors join on the next pass; pending destruction is skipped.
        void ForEachActor(const std::function<void(Actor&)>& visitor);

        // Call at a safe frame boundary, after rendering has released references
        // and GPU work using the actors' resources has finished. Calls during
        // actor enumeration/callbacks defer reclamation until a later call.
        void CollectGarbage();
        void Clear(); // EndPlay and destroy all actors; same safe-boundary rule.

    private:
        void RegisterActor(std::unique_ptr<Actor> actor);
        std::vector<std::unique_ptr<Actor>> m_Actors;
        size_t m_IterationDepth = 0;
        bool m_Playing = false;
        bool m_Clearing = false;
    };
}
