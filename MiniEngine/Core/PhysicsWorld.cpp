#include "PhysicsWorld.h"

#include <cmath>

using namespace physx;

namespace
{
    template<class T> void Release(T*& object)
    {
        if (object)
        {
            object->release();
            object = nullptr;
        }
    }

    PxFilterFlags ContactFilter(PxFilterObjectAttributes attributes0, PxFilterData data0,
        PxFilterObjectAttributes attributes1, PxFilterData data1, PxPairFlags& pairFlags,
        const void* constantBlock, PxU32 constantBlockSize)
    {
        const PxFilterFlags flags = PxDefaultSimulationFilterShader(attributes0, data0,
            attributes1, data1, pairFlags, constantBlock, constantBlockSize);
        if (!PxFilterObjectIsTrigger(attributes0) && !PxFilterObjectIsTrigger(attributes1))
            pairFlags |= PxPairFlag::eNOTIFY_CONTACT_POINTS;
        return flags;
    }
}

void PhysicsWorld::ErrorCallback::reportError(PxErrorCode::Enum code, const char* message,
    const char* file, int line)
{
    if (code != PxErrorCode::eDEBUG_INFO && code != PxErrorCode::eDEBUG_WARNING)
        ++errors;
    PxDefaultErrorCallback::reportError(code, message, file, line);
}

PhysicsWorld::~PhysicsWorld() { Shutdown(); }

bool PhysicsWorld::SupportsPvd()
{
    // Matches the vendored SDK configurations: Release disables PVD instrumentation.
#if defined(_DEBUG) || defined(PROFILE)
    return true;
#else
    return false;
#endif
}

bool PhysicsWorld::Initialize(bool connectPvd)
{
    if (IsInitialized())
        return true;

    m_ErrorCallback.errors = 0;
    m_Foundation = PxCreateFoundation(PX_PHYSICS_VERSION, m_Allocator, m_ErrorCallback);
    if (!m_Foundation)
        return false;

    if (SupportsPvd())
    {
        m_Pvd = PxCreatePvd(*m_Foundation);
        m_Transport = PxDefaultPvdSocketTransportCreate("127.0.0.1", 5425, 100);
        if (connectPvd)
            ConnectPvd(); // A missing debugger must never prevent simulation.
    }

    m_Physics = PxCreatePhysics(PX_PHYSICS_VERSION, *m_Foundation,
        PxTolerancesScale(), true, m_Pvd);
    if (m_Physics)
        m_ExtensionsInitialized = PxInitExtensions(*m_Physics, m_Pvd);
    if (!m_Physics || !m_ExtensionsInitialized)
    {
        Shutdown();
        return false;
    }

    m_Dispatcher = PxDefaultCpuDispatcherCreate(2);
    m_Material = m_Physics->createMaterial(0.6f, 0.5f, 0.1f);
    if (!m_Dispatcher || !m_Material)
    {
        Shutdown();
        return false;
    }

    PxSceneDesc desc(m_Physics->getTolerancesScale());
    desc.gravity = PxVec3(0.0f, -9.81f, 0.0f);
    desc.cpuDispatcher = m_Dispatcher;
    desc.filterShader = ContactFilter;
    m_Scene = m_Physics->createScene(desc);
    if (!m_Scene)
    {
        Shutdown();
        return false;
    }
    m_ControllerManager = PxCreateControllerManager(*m_Scene);
    if (!m_ControllerManager)
    {
        Shutdown();
        return false;
    }
    m_ControllerManager->setOverlapRecoveryModule(true);
    if (PxPvdSceneClient* client = m_Scene->getScenePvdClient())
    {
        client->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_CONSTRAINTS, true);
        client->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_CONTACTS, true);
        client->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_SCENEQUERIES, true);
    }
    ResetClock();
    return true;
}

void PhysicsWorld::Shutdown()
{
    Release(m_ControllerManager);
    Release(m_Scene); // Also releases actors and their exclusive shapes.
    Release(m_Material);
    Release(m_Dispatcher);
    if (m_ExtensionsInitialized)
    {
        PxCloseExtensions();
        m_ExtensionsInitialized = false;
    }
    Release(m_Physics);
    Release(m_Pvd);
    Release(m_Transport);
    Release(m_Foundation);
    ResetClock();
}

bool PhysicsWorld::ConnectPvd(bool reconnect)
{
    if (!m_Pvd || !m_Transport)
        return false;
    if (!reconnect && IsPvdConnected())
        return true;
    m_Pvd->disconnect();
    if (!m_Pvd->connect(*m_Transport, PxPvdInstrumentationFlag::eALL))
        return false;
    // Deliver initial scene data even when the demo is waiting for Start drop.
    m_Transport->lock().flush();
    m_Transport->unlock();
    return IsPvdConnected();
}

bool PhysicsWorld::IsPvdConnected() const
{
    // Socket status is updated by I/O. Explicit reconnect also handles a debugger
    // closed during a pause, when there may have been no writes to detect it.
    return m_Pvd && m_Pvd->isConnected(false);
}

void PhysicsWorld::ResetClock()
{
    m_Accumulator = 0.0;
    m_DroppedTime = 0.0;
    m_StepCount = 0;
    m_LastSubsteps = 0;
}

void PhysicsWorld::Step(const BeforeStep& beforeStep)
{
    if (!m_Scene)
        return;
    m_ControllerManager->computeInteractions(static_cast<float>(FixedDeltaTime));
    if (beforeStep)
        beforeStep(static_cast<float>(FixedDeltaTime));
    m_Scene->simulate(static_cast<float>(FixedDeltaTime));
    m_Scene->fetchResults(true); // Actor poses are safe to read after this completes.
    ++m_StepCount;
    m_LastSubsteps = 1;
}

uint32_t PhysicsWorld::Update(double elapsedSeconds, const BeforeStep& beforeStep)
{
    m_LastSubsteps = 0;
    if (!m_Scene || !std::isfinite(elapsedSeconds) || elapsedSeconds <= 0.0)
        return 0;

    // Keep at most eight steps from this frame. Discard long debugger/window stalls
    // instead of creating a growing backlog; preserve the previous fractional step.
    const double maxElapsed = FixedDeltaTime * MaxSubsteps;
    if (elapsedSeconds > maxElapsed)
    {
        m_DroppedTime += elapsedSeconds - maxElapsed;
        elapsedSeconds = maxElapsed;
    }
    m_Accumulator += elapsedSeconds;
    uint32_t steps = 0;
    while (m_Accumulator + 1e-10 >= FixedDeltaTime && steps < MaxSubsteps)
    {
        Step(beforeStep);
        m_Accumulator -= FixedDeltaTime;
        ++steps;
    }
    if (m_Accumulator < 0.0)
        m_Accumulator = 0.0;
    m_LastSubsteps = steps;
    return steps;
}

PxRigidStatic* PhysicsWorld::CreateStaticBox(const PxTransform& pose,
    const PxVec3& halfExtents, const char* name)
{
    if (!m_Scene)
        return nullptr;
    PxRigidStatic* actor = PxCreateStatic(*m_Physics, pose, PxBoxGeometry(halfExtents), *m_Material);
    if (actor)
    {
        actor->setName(name); // PhysX keeps this pointer; names must outlive their actors.
        m_Scene->addActor(*actor);
    }
    return actor;
}

PxRigidDynamic* PhysicsWorld::CreateDynamicBox(const PxTransform& pose,
    const PxVec3& halfExtents, float density, const char* name)
{
    if (!m_Scene)
        return nullptr;
    PxRigidDynamic* actor = PxCreateDynamic(*m_Physics, pose,
        PxBoxGeometry(halfExtents), *m_Material, density);
    if (actor)
    {
        actor->setName(name);
        m_Scene->addActor(*actor);
    }
    return actor;
}

PxCapsuleController* PhysicsWorld::CreateCapsuleController(const PxExtendedVec3& centre,
    float radius, float halfHeight)
{
    if (!m_ControllerManager || !std::isfinite(radius) || !std::isfinite(halfHeight) ||
        radius <= 0 || halfHeight <= radius || !centre.isFinite())
        return nullptr;
    PxCapsuleControllerDesc desc;
    desc.position = centre;
    desc.radius = radius;
    desc.height = 2.0f * (halfHeight - radius);
    desc.upDirection = PxVec3(0, 1, 0);
    desc.material = m_Material;
    desc.contactOffset = 0.02f;
    desc.stepOffset = PxMin(0.25f, halfHeight);
    desc.slopeLimit = 0.70710678f;
    desc.climbingMode = PxCapsuleClimbingMode::eCONSTRAINED;
    desc.nonWalkableMode = PxControllerNonWalkableMode::ePREVENT_CLIMBING_AND_FORCE_SLIDING;
    if (!desc.isValid()) return nullptr;
    return static_cast<PxCapsuleController*>(m_ControllerManager->createController(desc));
}

uint32_t PhysicsWorld::GetControllerCount() const
{
    return m_ControllerManager ? m_ControllerManager->getNbControllers() : 0;
}
