#include "World.h"
#include <algorithm>
#include <atomic>
#include <cmath>
#if defined(_M_X64)
#include "../PhysicsWorld.h"
#endif

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
    World::World() = default;
    World::~World() { Clear(); }

    bool World::InitializePhysics(bool connectPvd)
    {
#if defined(_M_X64)
        if (!m_Physics)
            m_Physics = std::make_unique<PhysicsWorld>();
        return m_Physics->Initialize(connectPvd);
#else
        (void)connectPvd;
        return false;
#endif
    }

    PhysicsWorld* World::GetPhysicsWorld() const
    {
#if defined(_M_X64)
        return m_Physics.get();
#else
        return nullptr;
#endif
    }

    void World::SetPhysicsPaused(bool paused)
    {
        m_PhysicsPaused = paused;
        m_StepRequested = false;
    }

    void World::SetPhysicsTimeScale(float scale)
    {
        if (std::isfinite(scale) && scale > 0 && scale <= 1.0f)
            m_PhysicsTimeScale = scale;
    }

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
        ForEachActor([](Actor& actor) { if (!actor.HasBegunPlay()) actor.BeginPlay(); });
    }

    void World::Tick(float deltaTime)
    {
        if (!m_Playing || m_IterationDepth != 0 || !std::isfinite(deltaTime) || deltaTime < 0.0f)
        {
			return;
        }
        // Inputs -> fixed movement/animation/physics -> final poses. A new actor is configured
        // before BeginPlay and never joins a tick pass halfway through spawning.
        {
            IterationScope scope(m_IterationDepth);
            const size_t count = m_Actors.size();
            for (size_t i = 0; i < count; ++i)
            {
                Actor* actor = m_Actors[i].get();
                if (!actor->IsPendingDestroy() && !actor->HasBegunPlay())
                {
                    actor->BeginPlay();
                }        
            }
        }

        if (!m_Playing)
        {
            return;
        }

        {
            IterationScope scope(m_IterationDepth);
            const size_t count = m_Actors.size();
            for (size_t i = 0; i < count; ++i)
            {
                Actor* actor = m_Actors[i].get();

				// 이 부분에서 Prephysics Tick을 호출한다. PrePhysics Tick은 물리 시뮬레이션이 시작되기 전에 호출되는 Tick이다. 이 Tick에서 Actor의 위치를 업데이트하고, 물리 시뮬레이션에 필요한 데이터를 준비한다.
				// 모든 액터들은 일반적으로 PostPhysics를 사용하지만, 이 프로젝트에서는 PlayerController가 PrePhysics를 사용한다. PlayerController는 입력을 받아서 Pawn의 위치를 업데이트하는 역할을 한다. 
                // 따라서 PlayerController는 PrePhysics Tick에서 입력을 처리하고, Pawn의 위치를 업데이트해야 한다.
                // PlayerController만 PrePhysics를 사용하는 이유는 그래야 이어지는 이동, 물리 계산이 바로 입력을 반영할 수 있기 때문이다.
				// PostPhysics Tick은 물리 시뮬레이션이 끝난 후에 호출되며, 이 Tick에서 Actor의 위치를 최종적으로 업데이트하고, 렌더링에 필요한 데이터를 준비한다.
                // 이 부분에서는 PlayerController의 Tick을 보통 수행한다. PlayerController가 조종하는 폰의 이동입력을 갱신해서 pawn에 이동을 누적한다.
                if (!actor->IsPendingDestroy() && actor->HasBegunPlay() && actor->GetTickGroup() == Actor::TickGroup::PrePhysics)
                {
					actor->Tick(deltaTime);
                }                  
            }
        }
        if (!m_Playing)
        {
            return;
        }

        {
            // 해당 PreparePhysics는 MovementController를 가진 폰에서만 상관이 있다. Pawn의 누적이동값을 가져와 MoveComponent의 m_Input에 저장한다. 그리고 Pawn의 누적입력은 0으로 초기화
            // 입력길이를 정규화해서 대각선에서 빨라지는걸 방지한다
            IterationScope scope(m_IterationDepth);
            const size_t count = m_Actors.size();
            for (size_t i = 0; i < count; ++i)
            {
                Actor* actor = m_Actors[i].get();
                if (!actor->IsPendingDestroy())
                {
					actor->PreparePhysics(deltaTime);
                }
            }
        }
        if (!m_Playing)
        {
            return;
        }

        float animationDelta = deltaTime;
#if defined(_M_X64)
        if (m_Physics && m_Physics->IsInitialized())
        {

            // 이 update에서는 실제로 물리를 실행하지않는다. 대신 m_LastSubsteps의 값을 0으로 초기화하고 끝낸다.
			// m_LastSubsteps는 물리 시뮬레이션에서 마지막으로 실행된 substep의 수를 나타낸다. 이 값을 0으로 초기화하면, 
            // 다음 프레임에서 물리 시뮬레이션이 시작될 때, 이전 프레임에서 남아있던 substep이 없음을 의미한다.
            m_Physics->Update(0.0); // Reset per-frame statistics, preserving the fractional step.

            const auto stepActors = [&](float dt) {
                IterationScope scope(m_IterationDepth);
                const size_t count = m_Actors.size();

                for (size_t i = 0; i < count; ++i)
                {
                    Actor* actor = m_Actors[i].get();
                    if (!actor->IsPendingDestroy())
                    {
                        // 이 부분에서 pawn을 상속한 character가 가진 컴포넌트들의 fixedtickcomponent가 호출된다. 
                        // 기본적으로 CharacterMovementComponent와 SkeletalMeshComponent가 있다.
						// CharacterMovementComponent는 이동을 갱신하고, SkeletalMeshComponent는 애니메이션, 래그돌 목표를 갱신한다.
                        actor->FixedTick(dt);
                    }                 
                }
            };

            animationDelta = m_PhysicsPaused ? 0.0f : deltaTime * m_PhysicsTimeScale;
            if (m_PhysicsPaused && m_StepRequested)
            {
                m_Physics->Step(stepActors);
                animationDelta = static_cast<float>(PhysicsWorld::FixedDeltaTime);
            }
            else if (!m_PhysicsPaused)
            {
				// 일반적으로 일시정지 되지않았다면 이 부분에서 물리 시뮬레이션을 진행한다.
                m_Physics->Update(animationDelta, stepActors);

                // Animation discards the same long stall as the physics clock.
                animationDelta = std::min(animationDelta, static_cast<float>(PhysicsWorld::FixedDeltaTime * PhysicsWorld::MaxSubsteps));
            }
            m_StepRequested = false;
        }
#endif
        if (!m_Playing)
        {
            return;
        }

        {
            IterationScope scope(m_IterationDepth);
            const size_t count = m_Actors.size();
            for (size_t i = 0; i < count; ++i)
            {
                Actor* actor = m_Actors[i].get();
                if (!actor->IsPendingDestroy() && actor->HasBegunPlay() && actor->GetTickGroup() == Actor::TickGroup::PostPhysics)
                {
                    actor->Tick(animationDelta);
                }          
            }
        }
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
