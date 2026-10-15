#pragma once

#include <PxPhysicsAPI.h>
#include <atomic>
#include <cstdint>
#include <functional>

// CPU physics in metres, kilograms and seconds; +Y is up. Use on the game thread.
// Actors returned by Create* are owned by the scene until explicitly released or
// Shutdown() is called. Finish all reads/writes before calling Update()/Step().
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
    void Shutdown();
    using BeforeStep = std::function<void(float)>;
    uint32_t Update(double elapsedSeconds, const BeforeStep& beforeStep = {});
    void Step(const BeforeStep& beforeStep = {});
    void ResetClock();

    physx::PxRigidStatic* CreateStaticBox(const physx::PxTransform& pose,
        const physx::PxVec3& halfExtents, const char* name);
    physx::PxRigidDynamic* CreateDynamicBox(const physx::PxTransform& pose,
        const physx::PxVec3& halfExtents, float density, const char* name);
    physx::PxCapsuleController* CreateCapsuleController(const physx::PxExtendedVec3& centre,
        float radius, float halfHeight);
    uint32_t GetControllerCount() const;

    bool ConnectPvd(bool reconnect = false);
    bool IsPvdConnected() const;
    static bool SupportsPvd();
    bool IsInitialized() const { return m_Scene != nullptr; }
    uint64_t GetStepCount() const { return m_StepCount; }
    uint32_t GetLastSubsteps() const { return m_LastSubsteps; }
    double GetDroppedTime() const { return m_DroppedTime; }
    uint32_t GetErrorCount() const { return m_ErrorCallback.errors.load(); }

private:
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
    physx::PxControllerManager* m_ControllerManager = nullptr;
    physx::PxMaterial* m_Material = nullptr;
    bool m_ExtensionsInitialized = false;
    double m_Accumulator = 0.0;
    double m_DroppedTime = 0.0;
    uint64_t m_StepCount = 0;
    uint32_t m_LastSubsteps = 0;
};
