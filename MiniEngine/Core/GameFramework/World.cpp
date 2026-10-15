#include "World.h"
#include <algorithm>
#include <atomic>
#include <cmath>

namespace
{
    // Process-wide identities prevent stale handles from aliasing a later spawn
    // or an actor belonging to another World.
    std::atomic<uint64_t> s_NextActorId{ 1 };

    struct IterationScope
    {
        explicit IterationScope(size_t& depth) : m_Depth(depth) { ++m_Depth; }
        ~IterationScope() { --m_Depth; }
        size_t& m_Depth;
    };
}

namespace PARS
{
    World::~World() { Clear(); }

    void World::RegisterActor(std::unique_ptr<Actor> actor)
    {
        actor->m_Handle.id = s_NextActorId.fetch_add(1, std::memory_order_relaxed);
        actor->m_World = this;
        m_Actors.push_back(std::move(actor));
    }

    Actor* World::FindActor(ActorHandle handle) const
    {
        if (handle.id == 0)
            return nullptr;
        for (const auto& actor : m_Actors)
            if (actor->GetHandle() == handle && !actor->IsPendingDestroy())
                return actor.get();
        return nullptr;
    }

    bool World::DestroyActor(ActorHandle handle)
    {
        Actor* actor = FindActor(handle);
        if (!actor)
            return false;
        actor->m_PendingDestroy = true;
        return true;
    }

    size_t World::GetActorCount() const
    {
        return static_cast<size_t>(std::count_if(m_Actors.begin(), m_Actors.end(),
            [](const std::unique_ptr<Actor>& actor) { return !actor->IsPendingDestroy(); }));
    }

    bool World::HasPendingDestroy() const
    {
        return std::any_of(m_Actors.begin(), m_Actors.end(),
            [](const std::unique_ptr<Actor>& actor) { return actor->IsPendingDestroy(); });
    }

    void World::ForEachActor(const std::function<void(Actor&)>& visitor)
    {
        IterationScope scope(m_IterationDepth);
        const size_t count = m_Actors.size();
        for (size_t i = 0; i < count; ++i)
        {
            // Indexing, rather than retaining a vector iterator, permits spawning
            // from a callback even if the ownership vector reallocates.
            Actor* actor = m_Actors[i].get();
            if (!actor->IsPendingDestroy())
                visitor(*actor);
        }
    }

    void World::BeginPlay()
    {
        if (m_Playing || m_Clearing || m_IterationDepth != 0)
            return;
        m_Playing = true;
        ForEachActor([](Actor& actor) { actor.BeginPlay(); });
    }

    void World::Tick(float deltaTime)
    {
        if (!m_Playing || m_IterationDepth != 0 || !std::isfinite(deltaTime) || deltaTime < 0.0f)
            return;
        ForEachActor([&](Actor& actor) {
            // SpawnActor leaves time to configure components before their first
            // BeginPlay. Spawns made during this pass start on the next Tick.
            actor.BeginPlay();
            if (!actor.IsPendingDestroy())
                actor.Tick(deltaTime);
        });
    }

    void World::CollectGarbage()
    {
        if (m_IterationDepth != 0)
            return;
        IterationScope scope(m_IterationDepth);
        std::vector<std::unique_ptr<Actor>> retired;
        auto end = std::remove_if(m_Actors.begin(), m_Actors.end(), [&](std::unique_ptr<Actor>& actor) {
            if (!actor->IsPendingDestroy())
                return false;
            retired.push_back(std::move(actor));
            return true;
        });
        m_Actors.erase(end, m_Actors.end());
        // Remove ownership entries before callbacks: EndPlay may request another
        // destruction or spawn a replacement, without modifying this retired list.
        for (auto it = retired.rbegin(); it != retired.rend(); ++it)
        {
            (*it)->EndPlay();
            (*it)->m_World = nullptr;
        }
        retired.clear();
        if (m_Clearing && m_Actors.empty())
            m_Clearing = false;
    }

    void World::Clear()
    {
        m_Playing = false;
        m_Clearing = true;
        for (const auto& actor : m_Actors)
            actor->m_PendingDestroy = true;
        CollectGarbage();
    }
}
