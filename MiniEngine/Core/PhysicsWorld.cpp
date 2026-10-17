#include "PhysicsWorld.h"

#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <Windows.h>
#include <cmath>
#include <algorithm>

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

    // 물리 코어 수가 아니라, Windows가 보고하는 논리 프로세서 수를 조회한다.
    SYSTEM_INFO systemInfo{};
    GetSystemInfo(&systemInfo);
    const uint32_t logicalProcessorCount = systemInfo.dwNumberOfProcessors;

    // 레퍼런스처럼 메인 스레드의 작업 여유를 고려해 PhysX 워커를 하나 적게 만든다.
    // 예: 8코어 / 16논리 프로세서 CPU라면 워커 15개. 특정 코어를 전용으로 예약하는 것은 아니다.
    // 논리 프로세서가 1개이거나 조회값이 0이어도 최소 1개를 유지하고, unsigned 뺄셈 언더플로를 막는다.
    const uint32_t workerCount = logicalProcessorCount > 1 ? logicalProcessorCount - 1 : 1;

    // 이 디스패처가 PhysX 내부 작업을 워커에 분배한다. 아래에서 Scene의 cpuDispatcher로 연결한다.
    m_Dispatcher = PxDefaultCpuDispatcherCreate(workerCount);
    m_Material = m_Physics->createMaterial(0.6f, 0.5f, 0.1f);
    if (!m_Dispatcher || !m_Material)
    {
        Shutdown();
        return false;
    }

    PxSceneDesc desc(m_Physics->getTolerancesScale());
    // TGS handles the anatomical ragdoll's mass/inertia ratios and narrow joint
    // limits more accurately during contact. The external fixed step stays 1/60;
    // this selects PhysX's constraint solver, not a frame-rate-dependent clock.
    desc.solverType = PxSolverType::eTGS;
    desc.gravity = PxVec3(0.0f, -9.81f, 0.0f);
    desc.cpuDispatcher = m_Dispatcher;
    desc.filterShader = ContactFilter;
    m_Scene = m_Physics->createScene(desc);
    if (!m_Scene)
    {
        Shutdown();
        return false;
    }
    if (PxPvdSceneClient* client = m_Scene->getScenePvdClient())
    {
        client->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_CONSTRAINTS, true);
        client->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_CONTACTS, true);
        // Movement queries otherwise overlay the PA with swept capsule volumes.
        client->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_SCENEQUERIES, false);
    }
    ResetClock();
    return true;
}

void PhysicsWorld::Shutdown()
{
    m_QueryCapsules.clear();
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
    if (beforeStep)
        beforeStep(static_cast<float>(FixedDeltaTime));
    if (IsPvdConnected())
    {
        std::vector<PxDebugLine> lines;
        GetCapsuleDebugLines(lines);
        if (!lines.empty()) m_Scene->getScenePvdClient()->drawLines(lines.data(), static_cast<PxU32>(lines.size()));
    }
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

PhysicsQueryCapsule* PhysicsWorld::CreateQueryCapsule(const PxVec3& centre,
    float radius, float halfHeight)
{
    if (!m_Scene || !std::isfinite(radius) || !std::isfinite(halfHeight) ||
        radius <= 0 || halfHeight <= radius || !centre.isFinite())
        return nullptr;
    auto capsule = std::make_unique<PhysicsQueryCapsule>();
    capsule->centre = centre; capsule->radius = radius; capsule->halfHeight = halfHeight;
    auto* result = capsule.get();
    m_QueryCapsules.push_back(std::move(capsule));
    return result;
}

void PhysicsWorld::DestroyQueryCapsule(PhysicsQueryCapsule* capsule)
{
    m_QueryCapsules.erase(std::remove_if(m_QueryCapsules.begin(), m_QueryCapsules.end(),
        [capsule](const auto& entry) { return entry.get() == capsule; }), m_QueryCapsules.end());
}

namespace
{
    constexpr float CapsuleSkin = .02f, SweepEpsilon = .0002f, WalkableNormal = .70710678f;
    PxCapsuleGeometry CapsuleGeometry(const PhysicsQueryCapsule& capsule, float skin = 0)
    { return PxCapsuleGeometry(capsule.radius + skin, capsule.halfHeight - capsule.radius); }
    PxTransform CapsulePose(const PxVec3& centre)
    { return PxTransform(centre, PxQuat(PxHalfPi, PxVec3(0, 0, 1))); } // PhysX local X -> PARS +Y.

    class CapsuleQueryFilter : public PxQueryFilterCallback
    {
    public:
        PxVec3 direction{0};
        PxQueryHitType::Enum preFilter(const PxFilterData&, const PxShape* shape,
            const PxRigidActor*, PxHitFlags&) override
        { return shape->getFlags().isSet(PxShapeFlag::eTRIGGER_SHAPE) ? PxQueryHitType::eNONE : PxQueryHitType::eBLOCK; }
        PxQueryHitType::Enum postFilter(const PxFilterData&, const PxQueryHit& hit,
            const PxShape*, const PxRigidActor*) override
        {
            const auto& sweep = static_cast<const PxSweepHit&>(hit);
            // Touching the floor must not block tangential travel or moving away.
            return sweep.normal.dot(direction) < -1e-5f ? PxQueryHitType::eBLOCK : PxQueryHitType::eNONE;
        }
    };

    // Process all overlaps, including scenes with more hits than the local buffer.
    class CapsulePenetration : public PxOverlapCallback
    {
    public:
        CapsulePenetration(const PhysicsQueryCapsule& capsule) : PxOverlapCallback(hits, 32),
            geometry(CapsuleGeometry(capsule, CapsuleSkin)), pose(CapsulePose(capsule.centre)) {}
        void Consider(const PxGeometry& other, const PxTransform& otherPose)
        {
            PxVec3 normal; float depth;
            if (PxGeometryQuery::computePenetration(normal, depth, geometry, pose, other, otherPose) && depth > deepest)
            { deepest = depth; direction = normal; }
        }
        PxAgain processTouches(const PxOverlapHit* buffer, PxU32 count) override
        {
            for (PxU32 i = 0; i < count; ++i)
                Consider(buffer[i].shape->getGeometry(), PxShapeExt::getGlobalPose(*buffer[i].shape, *buffer[i].actor));
            return true;
        }
        void finalizeQuery() override { processTouches(touches, nbTouches); }
        PxCapsuleGeometry geometry;
        PxTransform pose;
        float deepest = 0;
        PxVec3 direction{0};
    private:
        PxOverlapHit hits[32];
    };
}

PhysicsWorld::CapsuleMoveResult PhysicsWorld::MoveQueryCapsule(PhysicsQueryCapsule& capsule, const PxVec3& displacement)
{
    CapsuleMoveResult result;
    if (!m_Scene || !capsule.centre.isFinite() || !displacement.isFinite()) return result;
    CapsuleQueryFilter filter;
    // Recover spawn/teleport penetration before attempting any sweep.
    for (int iteration = 0; iteration < 8; ++iteration)
    {
        CapsulePenetration overlaps(capsule);
        m_Scene->overlap(overlaps.geometry, overlaps.pose, overlaps,
            PxQueryFilterData(PxQueryFlag::eSTATIC | PxQueryFlag::eDYNAMIC | PxQueryFlag::ePREFILTER | PxQueryFlag::eNO_BLOCK), &filter);
        for (const auto& other : m_QueryCapsules)
            if (other.get() != &capsule) overlaps.Consider(CapsuleGeometry(*other), CapsulePose(other->centre));
        if (overlaps.deepest <= 0) break;
        capsule.centre += overlaps.direction * PxMin(overlaps.deepest + SweepEpsilon, 2.0f);
    }

    const auto sweep = [&](const PxVec3& origin, const PxVec3& delta, PxGeomSweepHit& hit) {
        const float distance = delta.magnitude();
        if (distance < 1e-6f) return false;
        filter.direction = delta / distance;
        PxSweepBuffer buffer;
        const auto geometry = CapsuleGeometry(capsule, CapsuleSkin);
        const auto pose = CapsulePose(origin);
        const PxHitFlags flags = PxHitFlag::eDEFAULT | PxHitFlag::eMTD;
        m_Scene->sweep(geometry, pose, filter.direction, distance, buffer, flags,
            PxQueryFilterData(PxQueryFlag::eSTATIC | PxQueryFlag::eDYNAMIC | PxQueryFlag::ePREFILTER | PxQueryFlag::ePOSTFILTER), &filter);
        bool found = buffer.hasBlock;
        if (found) hit = buffer.block;
        // Other characters also remain query geometry, without registering proxies.
        for (const auto& other : m_QueryCapsules)
        {
            if (other.get() == &capsule) continue;
            PxGeomSweepHit candidate;
            if (PxGeometryQuery::sweep(filter.direction, distance, geometry, pose,
                CapsuleGeometry(*other), CapsulePose(other->centre), candidate, flags) &&
                candidate.normal.dot(filter.direction) < -1e-5f && (!found || candidate.distance < hit.distance))
            { hit = candidate; found = true; }
        }
        return found;
    };
    const auto travel = [](const PxVec3& delta, const PxGeomSweepHit& hit) {
        // The sweep geometry already includes the contact skin. Stop exactly
        // there; adding another gap changes the established character foot height.
        return delta.getNormalized() * PxMax(0.0f, hit.distance);
    };
    PxGeomSweepHit floor;
    const bool floorContact = displacement.y <= 0 && sweep(capsule.centre, PxVec3(0, -.04f, 0), floor);
    const bool wasGrounded = floorContact && floor.normal.y >= WalkableNormal;
    const PxVec3 horizontal(displacement.x, 0, displacement.z);
    bool stepped = false;
    PxGeomSweepHit obstruction;
    if (floorContact && floor.normal.y > .01f && sweep(capsule.centre, horizontal, obstruction) && obstruction.normal.y < WalkableNormal)
    {
        // Keep the old 25 cm step allowance. All three legs must be clear and
        // the landing must be walkable; never lift through a low ceiling.
        const float stepHeight = PxMin(.25f, capsule.halfHeight);
        const PxVec3 up(0, stepHeight, 0), down(0, -stepHeight - .04f, 0);
        // Probe a full footprint ahead to find the tread rather than rejecting
        // the rounded capsule/step edge normal on very short fixed-step moves.
        const PxVec3 forward = horizontal + horizontal.getNormalized() * (capsule.radius + CapsuleSkin);
        PxGeomSweepHit hit;
        if (!sweep(capsule.centre, up, hit) && !sweep(capsule.centre + up, forward, hit) &&
            sweep(capsule.centre + up + forward, down, hit) && hit.normal.y >= WalkableNormal)
        {
            capsule.centre += up + horizontal + travel(down, hit);
            stepped = true;
            result.grounded = true;
        }
    }
    const auto slide = [&](PxVec3 remaining, bool vertical) {
        for (int iteration = 0; iteration < 8 && remaining.magnitudeSquared() > 1e-10f; ++iteration)
        {
            PxGeomSweepHit hit;
            if (!sweep(capsule.centre, remaining, hit)) { capsule.centre += remaining; break; }
            const auto moved = travel(remaining, hit);
            capsule.centre += moved; remaining -= moved;
            PxVec3 normal = hit.normal;
            if (normal.y >= WalkableNormal)
            {
                result.grounded = true;
                if (vertical) break; // Ground support absorbs gravity, without sliding downhill.
            }
            else if (normal.y < -.01f)
            {
                result.ceiling = true;
                if (vertical) break;
            }
            else
            {
                result.wall = true;
                // Steep slopes behave as walls instead of turning forward input
                // into upward velocity. Walkable slopes retain their true normal.
                if (!vertical) { normal.y = 0; normal.normalize(); }
            }
            remaining -= normal * PxMin(0.0f, remaining.dot(normal));
        }
    };
    if (!stepped)
    {
        slide(horizontal, false);
        slide(PxVec3(0, displacement.y, 0), true);
    }
    if (displacement.y <= 0)
    {
        const PxVec3 down(0, wasGrounded ? -.29f : -.002f, 0);
        if (sweep(capsule.centre, down, floor) && floor.normal.y >= WalkableNormal)
        {
            capsule.centre += travel(down, floor);
            result.grounded = true;
        }
    }
    return result;
}

void PhysicsWorld::GetCapsuleDebugLines(std::vector<PxDebugLine>& lines) const
{
    lines.clear();
    if (!m_CapsuleDebugVisible) return;
    constexpr PxU32 color = 0xffe8b84a;
    for (const auto& capsule : m_QueryCapsules)
    {
        const PxVec3 centre = capsule->centre;
        const float radius = capsule->radius, halfCylinder = capsule->halfHeight - radius;
        const auto line = [&](const PxVec3& a, const PxVec3& b) { lines.emplace_back(centre + a, centre + b, color); };
        for (int end : {-1, 1})
        {
            const float y = end * halfCylinder;
            for (int segment = 0; segment < 24; ++segment)
            {
                const float a = segment * PxTwoPi / 24, b = (segment + 1) * PxTwoPi / 24;
                line(PxVec3(radius * std::cos(a), y, radius * std::sin(a)), PxVec3(radius * std::cos(b), y, radius * std::sin(b)));
            }
            for (int plane = 0; plane < 2; ++plane)
                for (int segment = 0; segment < 12; ++segment)
                {
                    const float a = segment * PxPi / 12, b = (segment + 1) * PxPi / 12;
                    const PxVec3 p(radius * std::cos(a), y + end * radius * std::sin(a), 0);
                    const PxVec3 q(radius * std::cos(b), y + end * radius * std::sin(b), 0);
                    line(plane ? PxVec3(0, p.y, p.x) : p, plane ? PxVec3(0, q.y, q.x) : q);
                }
        }
        for (int side = 0; side < 4; ++side)
        {
            const float a = side * PxHalfPi, x = radius * std::cos(a), z = radius * std::sin(a);
            line(PxVec3(x, -halfCylinder, z), PxVec3(x, halfCylinder, z));
        }
    }
}

void PhysicsWorld::SetPvdSceneQueries(bool enabled)
{
    if (m_Scene)
        if (auto* client = m_Scene->getScenePvdClient())
            client->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_SCENEQUERIES, enabled);
}

PxRigidDynamic* PhysicsWorld::CreateDynamicBody(const PxTransform& pose, const PxGeometry& geometry, float mass, const char* name)
{
    if (!m_Scene || !pose.isValid() || !std::isfinite(mass) || mass <= 0) return nullptr;
    auto* body = PxCreateDynamic(*m_Physics, pose, geometry, *m_Material, 1.0f);
    if (!body) return nullptr;
    if (!PxRigidBodyExt::setMassAndUpdateInertia(*body, mass)) { body->release(); return nullptr; }
    body->setName(name);
    body->setSolverIterationCounts(32, 8);
    body->setAngularDamping(.3f);
    body->setLinearDamping(.05f);
    body->setMaxAngularVelocity(20);
    body->setMaxDepenetrationVelocity(3);
    m_Scene->addActor(*body);
    return body;
}

PxD6Joint* PhysicsWorld::CreateJoint(PxRigidActor& parent, const PxTransform& parentFrame,
    PxRigidActor& child, const PxTransform& childFrame)
{
    if (!m_Physics || parent.getScene() != m_Scene || child.getScene() != m_Scene) return nullptr;
    return PxD6JointCreate(*m_Physics, &parent, parentFrame, &child, childFrame);
}

uint32_t PhysicsWorld::GetDynamicBodyCount() const
{
    return m_Scene ? m_Scene->getNbActors(PxActorTypeFlag::eRIGID_DYNAMIC) : 0;
}

uint32_t PhysicsWorld::GetConstraintCount() const
{
    return m_Scene ? m_Scene->getNbConstraints() : 0;
}
