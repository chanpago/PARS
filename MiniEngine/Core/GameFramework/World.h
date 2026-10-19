#pragma once

#include "Actor.h"
#include <functional>

class PhysicsWorld;

namespace PARS
{
    // 게임플레이 Actor와 이들이 공유하는 선택적 PhysX 시뮬레이션을 소유한다.
    // 메인 스레드에서만 사용한다. 콜백 도중 목록이 바뀌어도 살아 있는 Actor는 무효화되지 않는다.
    class World
    {
    public:
        World();
        ~World();
        World(const World&) = delete;
        World& operator=(const World&) = delete;

        template<class T, class... Args>
        T* SpawnActor(Args&&... args)
        {
            static_assert(std::is_base_of<Actor, T>::value, "T must be an Actor");
            if (m_Clearing)
            {
                return nullptr;
            }
            auto actor = std::make_unique<T>(std::forward<Args>(args)...);
            T* result = actor.get();
            RegisterActor(std::move(actor));
            return result;
        }

        // 핸들은 Actor보다 오래 유지될 수 있다. 파괴된 Actor를 조회하면 null을 반환한다.
        Actor* FindActor(ActorHandle handle) const;
        template<class T>
        T* FindActor(ActorHandle handle) const
        {
            return dynamic_cast<T*>(FindActor(handle));
        }
        bool DestroyActor(ActorHandle handle);
        size_t GetActorCount() const;
        bool HasPendingDestroy() const;

        void BeginPlay();
        void Tick(float deltaTime);
        bool HasBegunPlay() const
        {
            return m_Playing;
        }
        bool InitializePhysics(bool connectPvd = true);
        PhysicsWorld* GetPhysicsWorld() const;
        void SetPhysicsPaused(bool paused);
        bool IsPhysicsPaused() const
        {
            return m_PhysicsPaused;
        }
        void SetPhysicsTimeScale(float scale);
        float GetPhysicsTimeScale() const
        {
            return m_PhysicsTimeScale;
        }
        void RequestPhysicsStep()
        {
            if (m_PhysicsPaused)
            {
                m_StepRequested = true;
            }
        }

        // 새 Actor는 다음 순회부터 참여하며, 파괴 대기 중인 Actor는 건너뛴다.
        void ForEachActor(const std::function<void(Actor&)>& visitor);

        // 렌더러가 참조를 놓고 Actor 리소스를 사용하는 GPU 작업도 끝난 안전한 프레임 경계에서
        // 호출한다. Actor 순회나 콜백 중에 호출하면 실제 메모리 회수는
        // 나중에 다시 호출할 때까지 미룬다.
        void CollectGarbage();
        void Clear(); // 모든 Actor에 EndPlay를 호출하고 파괴한다. 동일한 안전한 프레임 경계 규칙을 따른다.

    private:
        void RegisterActor(std::unique_ptr<Actor> actor);
        std::vector<std::unique_ptr<Actor>> m_Actors;
        size_t m_IterationDepth = 0;
        bool m_Playing = false;
        bool m_Clearing = false;
#if defined(_M_X64)
        std::unique_ptr<PhysicsWorld> m_Physics;
#endif
        bool m_PhysicsPaused = false;
        bool m_StepRequested = false;
        float m_PhysicsTimeScale = 1.0f;
    };
}
