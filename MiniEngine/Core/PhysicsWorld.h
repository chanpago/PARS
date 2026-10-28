#pragma once

#include <PxPhysicsAPI.h>
#include "PhysicsMaterial.h"
#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>
#include <vector>

// 쿼리용 형상만 사용한다. PxActor/PxShape나 CCT 프록시 바디는 생성하지 않는다.
struct PhysicsQueryCapsule
{
    physx::PxVec3 centre;
    float radius, halfHeight;
    const void* owner = nullptr; // 이동 쿼리에서는 이 소유자로 표시된 강체 Actor만 제외한다.
};

// 미터, 킬로그램, 초 단위의 CPU 물리이며 +Y가 위쪽이다. 게임 스레드에서 사용한다.
// Create*가 반환한 Actor는 명시적으로 해제하거나 Shutdown()을 호출할 때까지 씬이 소유한다.
// Update()/Step()을 호출하기 전에 모든 읽기와 쓰기를 끝내야 한다.
class PhysicsWorld
{
public:
    static constexpr double FixedDeltaTime = 1.0 / 60.0;
    static constexpr uint32_t MaxSubsteps = 8;

    PhysicsWorld() = default;
    ~PhysicsWorld();
    PhysicsWorld(const PhysicsWorld&) = delete;
    PhysicsWorld& operator=(const PhysicsWorld&) = delete;

    bool Initialize(bool connectPvd = true);
    // SDK와 워커만 공유한다. 씬/시계/접촉은 독립적이며 owner보다 먼저 종료해야 한다.
    bool InitializeScene(PhysicsWorld& owner);
    void Shutdown();
    using BeforeStep = std::function<void(float)>;
    uint32_t Update(double elapsedSeconds, const BeforeStep& beforeStep = {});
    void Step(const BeforeStep& beforeStep = {});
    void ResetClock();

    physx::PxRigidStatic* CreateStaticBox(const physx::PxTransform& pose,
        const physx::PxVec3& halfExtents, const char* name);
    physx::PxRigidDynamic* CreateDynamicBox(const physx::PxTransform& pose,
        const physx::PxVec3& halfExtents, float density, const char* name);
    PhysicsQueryCapsule* CreateQueryCapsule(const physx::PxVec3& centre,
        float radius, float halfHeight);
    void DestroyQueryCapsule(PhysicsQueryCapsule* capsule);
    struct CapsuleMoveResult
    {
        bool grounded = false, ceiling = false, wall = false;
    };
    CapsuleMoveResult MoveQueryCapsule(PhysicsQueryCapsule& capsule, const physx::PxVec3& displacement,
        bool blockHorizontal = true);
    // 구형 프로브가 장애물에 닿기 전까지 이동할 수 있는 거리(m). 자기 Actor와 트리거는 제외한다.
    float SweepSphereDistance(const physx::PxVec3& origin, const physx::PxVec3& displacement,
        float radius, const void* ignoredOwner = nullptr) const;
    // 변위 길이 안의 가장 가까운 표면. 자기 Actor와 트리거는 제외한다.
    bool Raycast(const physx::PxVec3& origin, const physx::PxVec3& displacement,
        physx::PxRaycastHit& hit, const void* ignoredOwner = nullptr) const;
    uint32_t GetQueryCapsuleCount() const
    {
        return static_cast<uint32_t>(m_QueryCapsules.size());
    }
    void GetCapsuleDebugLines(std::vector<physx::PxDebugLine>& lines) const;
    void SetCapsuleDebugVisible(bool visible)
    {
        m_CapsuleDebugVisible = visible;
    }
    bool IsCapsuleDebugVisible() const
    {
        return m_CapsuleDebugVisible;
    }
    void SetPvdSceneQueries(bool enabled);
    uint32_t GetDynamicBodyCount() const;
    uint32_t GetConstraintCount() const;
    physx::PxRigidDynamic* CreateDynamicBody(const physx::PxTransform& pose,
        const physx::PxGeometry& geometry, float mass, const char* name,
        const PARS::PhysicsMaterialSettings& material = {});
    physx::PxArticulationReducedCoordinate* CreateArticulation();
    physx::PxArticulationLink* CreateArticulationLink(physx::PxArticulationReducedCoordinate& articulation,
        physx::PxArticulationLink* parent, const physx::PxTransform& pose,
        const physx::PxGeometry& geometry, float mass, const char* name,
        const PARS::PhysicsMaterialSettings& material = {});
    bool AddArticulation(physx::PxArticulationReducedCoordinate& articulation);
    // 씬 밖에 보관했던 바디를 다시 등록한다. 바디와 형상은 재생성하지 않는다.
    bool AddRigidBody(physx::PxRigidDynamic& body);
    physx::PxD6Joint* CreateJoint(physx::PxRigidActor& parent, const physx::PxTransform& parentFrame,
        physx::PxRigidActor& child, const physx::PxTransform& childFrame);

    bool ConnectPvd(bool reconnect = false);
    bool IsPvdConnected() const;
    static bool SupportsPvd();
    bool IsInitialized() const
    {
        return m_Scene != nullptr;
    }
    uint64_t GetStepCount() const
    {
        return m_StepCount;
    }
    uint32_t GetLastSubsteps() const
    {
        return m_LastSubsteps;
    }
    double GetDroppedTime() const
    {
        return m_DroppedTime;
    }
    uint32_t GetErrorCount() const
    {
        return m_ServiceOwner ? m_ServiceOwner->GetErrorCount() : m_ErrorCallback.errors.load();
    }

private:
    bool CreateScene();
    physx::PxMaterial* CreateBodyMaterial(const PARS::PhysicsMaterialSettings& settings);
    PhysicsWorld* m_ServiceOwner = nullptr;
    class ErrorCallback : public physx::PxDefaultErrorCallback
    {
    public:
        void reportError(physx::PxErrorCode::Enum code, const char* message,
            const char* file, int line) override;
        std::atomic<uint32_t> errors{0};
    };

    physx::PxDefaultAllocator m_Allocator;
    ErrorCallback m_ErrorCallback;
    physx::PxFoundation* m_Foundation = nullptr;
    physx::PxPvd* m_Pvd = nullptr;
    physx::PxPvdTransport* m_Transport = nullptr;
    physx::PxPhysics* m_Physics = nullptr;
    physx::PxDefaultCpuDispatcher* m_Dispatcher = nullptr;
    physx::PxScene* m_Scene = nullptr;
    std::vector<std::unique_ptr<PhysicsQueryCapsule>> m_QueryCapsules;
    bool m_CapsuleDebugVisible = true;
    physx::PxMaterial* m_Material = nullptr;
    bool m_ExtensionsInitialized = false;
    double m_Accumulator = 0.0;
    double m_DroppedTime = 0.0;
    uint64_t m_StepCount = 0;
    uint32_t m_LastSubsteps = 0;
};
