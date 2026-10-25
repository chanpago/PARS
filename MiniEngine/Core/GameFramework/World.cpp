#include "World.h"
#include <algorithm>
#include <atomic>
#include <cmath>
#if defined(_M_X64)
#include "../PhysicsWorld.h"
#endif

namespace
{
    // 프로세스 전체에서 고유한 ID를 사용해 오래된 핸들이 나중에 생성된 Actor나
    // 다른 World의 Actor를 잘못 가리키지 않도록 한다.
    std::atomic<uint64_t> s_NextActorId{1};

    struct IterationScope
    {
        explicit IterationScope(size_t& depth)
            : m_Depth(depth)
        {
            ++m_Depth;
        }
        ~IterationScope()
        {
            --m_Depth;
        }
        size_t& m_Depth;
    };
}

namespace PARS
{
    World::World() = default;
    World::~World()
    {
        Clear();
    }

    bool World::InitializePhysics(bool connectPvd, PhysicsWorld* sceneOwner)
    {
#if defined(_M_X64)
        if (!m_Physics)
        {
            m_Physics = std::make_unique<PhysicsWorld>();
        }
        return sceneOwner ? m_Physics->InitializeScene(*sceneOwner) : m_Physics->Initialize(connectPvd);
#else
        (void)connectPvd;
        (void)sceneOwner;
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
        {
            m_PhysicsTimeScale = scale;
        }
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
        {
            return nullptr;
        }
        for (const auto& actor : m_Actors)
        {
            if (actor->GetHandle() == handle && !actor->IsPendingDestroy())
            {
                return actor.get();
            }
        }
        return nullptr;
    }

    bool World::DestroyActor(ActorHandle handle)
    {
        Actor* actor = FindActor(handle);
        if (!actor)
        {
            return false;
        }
        actor->m_PendingDestroy = true;
        return true;
    }

    size_t World::GetActorCount() const
    {
        return static_cast<size_t>(std::count_if(m_Actors.begin(), m_Actors.end(),
            [](const std::unique_ptr<Actor>& actor) {
                return !actor->IsPendingDestroy();
            }));
    }

    bool World::HasPendingDestroy() const
    {
        return std::any_of(m_Actors.begin(), m_Actors.end(),
            [](const std::unique_ptr<Actor>& actor) {
                return actor->IsPendingDestroy();
            });
    }

    void World::ForEachActor(const std::function<void(Actor&)>& visitor)
    {
        IterationScope scope(m_IterationDepth);
        const size_t count = m_Actors.size();
        for (size_t i = 0; i < count; ++i)
        {
            // 벡터 반복자를 유지하지 않고 인덱스로 접근하므로, 콜백에서 Actor를 생성해
            // 소유 벡터가 재할당되어도 안전하게 순회할 수 있다.
            Actor* actor = m_Actors[i].get();
            if (!actor->IsPendingDestroy())
            {
                visitor(*actor);
            }
        }
    }

    void World::BeginPlay()
    {
        if (m_Playing || m_Clearing || m_IterationDepth != 0)
        {
            return;
        }
        m_Playing = true;
        InitializePendingActors();
    }

    void World::InitializePendingActors()
    {
        if (!m_Playing || m_Clearing || m_IterationDepth != 0)
        {
            return;
        }
        ForEachActor([](Actor& actor) {
            if (!actor.HasBegunPlay())
            {
                actor.BeginPlay();
            }
        });
    }

    void World::Tick(float deltaTime)
    {
        if (!m_Playing || m_IterationDepth != 0 || !std::isfinite(deltaTime) || deltaTime < 0.0f)
        {
            return;
        }
        // 입력 -> 고정 간격 이동/애니메이션/물리 -> 최종 포즈 순서다. 새 Actor는 BeginPlay 전에
        // 설정을 마치며, 생성 도중 진행 중인 틱 순회에 끼어들지 않는다.
        InitializePendingActors();

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

                // PrePhysics는 함수가 아니라 "물리 계산 전에 Tick할 Actor"를 구분하는 실행 그룹이다.
                // 이 그룹에 속한 Actor의 Tick(deltaTime)을 호출하며, 현재 PlayerController가 해당한다.
                // PlayerController::Tick()은 달리기 여부를 Pawn에 전달하고, 이동 방향 입력을 누적한다.
                // 실제 위치 변경은 이후 CharacterMovementComponent::FixedTickComponent()에서 처리한다.
                // 아래에서 별도로 호출하는 PreparePhysics()와는 다른 단계다.
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
            m_Physics->Update(0.0); // 고정 스텝 미만의 누적 시간은 유지하고, 프레임별 통계만 초기화한다.

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

                // 애니메이션도 물리 시간과 동일하게 장시간 정지 구간을 버린다.
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
        {
            return;
        }
        IterationScope scope(m_IterationDepth);
        std::vector<std::unique_ptr<Actor>> retired;
        auto end = std::remove_if(m_Actors.begin(), m_Actors.end(), [&](std::unique_ptr<Actor>& actor) {
            if (!actor->IsPendingDestroy())
            {
                return false;
            }
            retired.push_back(std::move(actor));
            return true;
        });
        m_Actors.erase(end, m_Actors.end());
        // 콜백 전에 소유 목록에서 제거한다. EndPlay에서 다른 객체를 파괴하거나 대체 객체를
        // 생성해도, 이미 분리한 삭제 대상 목록은 바뀌지 않는다.
        for (auto it = retired.rbegin(); it != retired.rend(); ++it)
        {
            (*it)->EndPlay();
            (*it)->m_World = nullptr;
        }
        retired.clear();
        if (m_Clearing && m_Actors.empty())
        {
            m_Clearing = false;
        }
    }

    void World::Clear()
    {
        m_Playing = false;
        m_Clearing = true;
        for (const auto& actor : m_Actors)
        {
            actor->m_PendingDestroy = true;
        }
        CollectGarbage();
    }
}
