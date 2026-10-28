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
    template<class T>
    void Release(T*& object)
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
        {
            pairFlags |= PxPairFlag::eNOTIFY_CONTACT_POINTS;
        }
        return flags;
    }
}

void PhysicsWorld::ErrorCallback::reportError(PxErrorCode::Enum code, const char* message,
    const char* file, int line)
{
    if (code != PxErrorCode::eDEBUG_INFO && code != PxErrorCode::eDEBUG_WARNING)
    {
        ++errors;
    }
    PxDefaultErrorCallback::reportError(code, message, file, line);
}

PhysicsWorld::~PhysicsWorld()
{
    Shutdown();
}

bool PhysicsWorld::SupportsPvd()
{
    // 저장소에 포함된 SDK 설정에 맞춘다. Release에서는 PVD 계측을 사용하지 않는다.
#if defined(_DEBUG) || defined(PROFILE)
    return true;
#else
    return false;
#endif
}

bool PhysicsWorld::Initialize(bool connectPvd)
{
    if (IsInitialized())
    {
        return true;
    }

    m_ErrorCallback.errors = 0;
    m_Foundation = PxCreateFoundation(PX_PHYSICS_VERSION, m_Allocator, m_ErrorCallback);
    if (!m_Foundation)
    {
        return false;
    }

    if (SupportsPvd())
    {
        m_Pvd = PxCreatePvd(*m_Foundation);
        m_Transport = PxDefaultPvdSocketTransportCreate("127.0.0.1", 5425, 100);
        if (connectPvd)
        {
            ConnectPvd(); // 디버거가 없어도 시뮬레이션은 실행할 수 있어야 한다.
        }
    }

    m_Physics = PxCreatePhysics(PX_PHYSICS_VERSION, *m_Foundation,
        PxTolerancesScale(), true, m_Pvd);
    if (m_Physics)
    {
        m_ExtensionsInitialized = PxInitExtensions(*m_Physics, m_Pvd);
    }
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

    return CreateScene();
}

bool PhysicsWorld::InitializeScene(PhysicsWorld& owner)
{
    if (this == &owner || !owner.IsInitialized() || IsInitialized())
    {
        return false;
    }
    m_ServiceOwner = &owner;
    m_Physics = owner.m_Physics;
    m_Dispatcher = owner.m_Dispatcher;
    m_Material = owner.m_Material;
    return CreateScene();
}

bool PhysicsWorld::CreateScene()
{
    PxSceneDesc desc(m_Physics->getTolerancesScale());
    desc.solverType = PxSolverType::eTGS;
    // 중력/관절 보상력을 각 TGS 반복에 나눠 적용해 드라이브의 정상 상태 속도 편향을 줄인다.
    desc.flags |= PxSceneFlag::eENABLE_EXTERNAL_FORCES_EVERY_ITERATION_TGS;
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
        // 이동 쿼리를 전송하면 스윕한 캡슐 영역이 PA 위에 겹쳐 표시되므로 제외한다.
        client->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_SCENEQUERIES, false);
    }
    ResetClock();
    return true;
}

void PhysicsWorld::Shutdown()
{
    m_QueryCapsules.clear();
    Release(m_Scene); // Actor와 각 Actor가 독점 소유하는 형상도 함께 해제한다.
    if (m_ServiceOwner)
    {
        m_Physics = nullptr;
        m_Dispatcher = nullptr;
        m_Material = nullptr;
        m_ServiceOwner = nullptr;
        ResetClock();
        return;
    }
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
    {
        return false;
    }
    if (!reconnect && IsPvdConnected())
    {
        return true;
    }
    m_Pvd->disconnect();
    if (!m_Pvd->connect(*m_Transport, PxPvdInstrumentationFlag::eALL))
    {
        return false;
    }
    // 데모가 Start drop 입력을 기다리는 동안에도 초기 씬 데이터를 전달한다.
    m_Transport->lock().flush();
    m_Transport->unlock();
    return IsPvdConnected();
}

bool PhysicsWorld::IsPvdConnected() const
{
    // 소켓 상태는 입출력 시 갱신된다. 일시정지 중에는 쓰기가 없어 디버거 종료를
    // 감지하지 못할 수 있으므로, 명시적 재연결로 이 경우도 처리한다.
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
    {
        return;
    }

    // 콜백함수
    // 이 부분에서 물리 시뮬레이션 전에 전달 받은 콜백을 호출한다. 콜백은 물리 시뮬레이션 전에 Actor의 pose를 읽어서, 물리 시뮬레이션에 반영할 목표 자세를 갱신하는데 사용된다.
    // 콜백을 사용한 이유는 다음과같다.
    // world만 actor를 소유하고 불필요한 참조를 만들지 않기 위해서다. physicsworld는 그냥 단순하게 물리만 계산하고 actor의 소유는 world가 담당하게하기 위해서다.
    // World -> 어떤 액터를 어떻게 갱신할지 결정해서 콜백으로 전달
    // PhysicWorld -> 물리 스텝 직전에 콜백을 실행하고 PhysX 계산진행
    // 여기서 줄이는건 PhysicWorld가 World나 Actor의 내부 구조를 알아야 하는 의존성을 줄이기 위해서다. 콜백을 전달해도 액터의 소유권이나 관리책임은 그대로 World가 담당한다.
    if (beforeStep)
    {
        beforeStep(static_cast<float>(FixedDeltaTime));
    }

    if (IsPvdConnected())
    {
        std::vector<PxDebugLine> lines;
        GetCapsuleDebugLines(lines);
        if (!lines.empty())
        {
            m_Scene->getScenePvdClient()->drawLines(lines.data(), static_cast<PxU32>(lines.size()));
        }
    }

    // 이 부분에서 PhsyX를 생성할때 할당 처리한 워커스레드를 이용해서 물리 시뮬레이션을 진행한다. FixedDeltaTime은 1/60초로 고정되어 있다.
    m_Scene->simulate(static_cast<float>(FixedDeltaTime));

    // PhysX는 시뮬레이션이 끝나면 fetchResults()를 호출해야 한다. fetchResults()가 끝나야 Actor의 pose를 읽을 수 있다. true파라미터는 simulate의 결과를 다 받고 진행하겠다는 뜻.
    m_Scene->fetchResults(true);
    ++m_StepCount;
    m_LastSubsteps = 1;
}

/*
이번 프레임에 지난 시간을 받아서 물리 시뮬레이션을 몇 번 실행할지 결정한다

*/
uint32_t PhysicsWorld::Update(double elapsedSeconds, const BeforeStep& beforeStep)
{
    m_LastSubsteps = 0;
    if (!m_Scene || !std::isfinite(elapsedSeconds) || elapsedSeconds <= 0.0)
    {
        return 0;
    }

    // 현재 프레임에서는 최대 8스텝만 반영한다. 디버거나 창 조작으로 오래 멈춘 시간은 버려
    // 미처리 시간이 계속 쌓이지 않게 하되, 이전의 고정 스텝 미만 누적 시간은 보존한다.
    const double maxElapsed = FixedDeltaTime * MaxSubsteps;
    if (elapsedSeconds > maxElapsed)
    {
        m_DroppedTime += elapsedSeconds - maxElapsed;
        elapsedSeconds = maxElapsed;
    }

    // 이 프로젝트에서는 물리를 고정 시간 1/60으로 계산하므로 지난 시간을 누적해서 1/60보다 크면 Step()을 호출한다. Step()은 PhysX의 simulate()과 fetchResults()를 호출한다.
    m_Accumulator += elapsedSeconds;

    uint32_t steps = 0;

    // 1/60초가 쌓일 때마다 물리를 한번 진행한다. MaxSubsteps를 넘어가면 더 이상 진행하지 않고, 남은 시간은 다음 프레임으로 넘긴다.
    while (m_Accumulator + 1e-10 >= FixedDeltaTime && steps < MaxSubsteps)
    {
        Step(beforeStep);
        m_Accumulator -= FixedDeltaTime;
        ++steps;
    }

    if (m_Accumulator < 0.0)
    {
        m_Accumulator = 0.0;
    }

    m_LastSubsteps = steps;
    return steps;
}

PxRigidStatic* PhysicsWorld::CreateStaticBox(const PxTransform& pose,
    const PxVec3& halfExtents, const char* name)
{
    if (!m_Scene)
    {
        return nullptr;
    }
    PxRigidStatic* actor = PxCreateStatic(*m_Physics, pose, PxBoxGeometry(halfExtents), *m_Material);
    if (actor)
    {
        actor->setName(name); // PhysX가 이 포인터를 보관하므로 이름 문자열은 Actor보다 오래 유지되어야 한다.
        m_Scene->addActor(*actor);
    }
    return actor;
}

PxRigidDynamic* PhysicsWorld::CreateDynamicBox(const PxTransform& pose,
    const PxVec3& halfExtents, float density, const char* name)
{
    if (!m_Scene)
    {
        return nullptr;
    }
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
    {
        return nullptr;
    }
    auto capsule = std::make_unique<PhysicsQueryCapsule>();
    capsule->centre = centre;
    capsule->radius = radius;
    capsule->halfHeight = halfHeight;
    auto* result = capsule.get();
    m_QueryCapsules.push_back(std::move(capsule));
    return result;
}

void PhysicsWorld::DestroyQueryCapsule(PhysicsQueryCapsule* capsule)
{
    m_QueryCapsules.erase(std::remove_if(m_QueryCapsules.begin(), m_QueryCapsules.end(),
                              [capsule](const auto& entry) {
                                  return entry.get() == capsule;
                              }),
        m_QueryCapsules.end());
}

namespace
{
    constexpr float CapsuleSkin = .02f, SweepEpsilon = .0002f, WalkableNormal = .70710678f;
    PxCapsuleGeometry CapsuleGeometry(const PhysicsQueryCapsule& capsule, float skin = 0)
    {
        return PxCapsuleGeometry(capsule.radius + skin, capsule.halfHeight - capsule.radius);
    }
    PxTransform CapsulePose(const PxVec3& centre)
    {
        return PxTransform(centre, PxQuat(PxHalfPi, PxVec3(0, 0, 1)));
    } // PhysX의 로컬 X축을 PARS의 +Y축에 맞춘다.

    class OwnerQueryFilter : public PxQueryFilterCallback
    {
    public:
        explicit OwnerQueryFilter(const void* owner)
            : ignoredOwner(owner)
        {
        }
        const void* ignoredOwner;
        PxVec3 direction{0};
        bool verticalOnly = false;
        PxQueryHitType::Enum preFilter(const PxFilterData&, const PxShape* shape,
            const PxRigidActor* actor, PxHitFlags&) override
        {
            return shape->getFlags().isSet(PxShapeFlag::eTRIGGER_SHAPE) ||
                    (ignoredOwner && actor->userData == ignoredOwner)
                ? PxQueryHitType::eNONE
                : PxQueryHitType::eBLOCK;
        }
        PxQueryHitType::Enum postFilter(const PxFilterData&, const PxQueryHit& hit,
            const PxShape*, const PxRigidActor*) override
        {
            const auto& sweep = static_cast<const PxSweepHit&>(hit);
            // 바닥과 접촉해도 접선 방향 이동이나 바닥에서 멀어지는 이동은 막지 않는다.
            return (!verticalOnly || PxAbs(sweep.normal.y) >= WalkableNormal) && sweep.normal.dot(direction) < -1e-5f
                ? PxQueryHitType::eBLOCK : PxQueryHitType::eNONE;
        }
    };

    // 로컬 버퍼 용량보다 충돌 결과가 많은 씬에서도 모든 겹침을 처리한다.
    class CapsulePenetration : public PxOverlapCallback
    {
    public:
        CapsulePenetration(const PhysicsQueryCapsule& capsule, bool verticalOnly = false)
            : PxOverlapCallback(hits, 32),
            geometry(CapsuleGeometry(capsule, CapsuleSkin)), pose(CapsulePose(capsule.centre)), verticalOnly(verticalOnly)
        {
        }
        void Consider(const PxGeometry& other, const PxTransform& otherPose)
        {
            PxVec3 normal;
            float depth;
            if (PxGeometryQuery::computePenetration(normal, depth, geometry, pose, other, otherPose) && depth > deepest &&
                (!verticalOnly || PxAbs(normal.y) >= WalkableNormal))
            {
                deepest = depth;
                direction = normal;
            }
        }
        PxAgain processTouches(const PxOverlapHit* buffer, PxU32 count) override
        {
            for (PxU32 i = 0; i < count; ++i)
            {
                Consider(buffer[i].shape->getGeometry(), PxShapeExt::getGlobalPose(*buffer[i].shape, *buffer[i].actor));
            }
            return true;
        }
        void finalizeQuery() override
        {
            processTouches(touches, nbTouches);
        }
        PxCapsuleGeometry geometry;
        PxTransform pose;
        float deepest = 0;
        PxVec3 direction{0};

    private:
        bool verticalOnly;
        PxOverlapHit hits[32];
    };
}

float PhysicsWorld::SweepSphereDistance(const PxVec3& origin, const PxVec3& displacement,
    float radius, const void* ignoredOwner) const
{
    const float distance = displacement.magnitude();
    if (!origin.isFinite() || !displacement.isFinite() || !std::isfinite(distance) ||
        !std::isfinite(radius) || radius <= 0 || distance < 1e-6f)
    {
        return 0;
    }
    if (!m_Scene)
    {
        return distance;
    }
    const PxSphereGeometry geometry(radius);
    const PxTransform pose(origin);
    const PxVec3 direction = displacement / distance;
    OwnerQueryFilter filter(ignoredOwner);
    PxSweepBuffer hit;
    m_Scene->sweep(geometry, pose, direction, distance, hit, PxHitFlag::eDEFAULT,
        PxQueryFilterData(PxQueryFlag::eSTATIC | PxQueryFlag::eDYNAMIC | PxQueryFlag::ePREFILTER), &filter);
    float allowed = hit.hasBlock ? PxMax(0.0f, hit.block.distance) : distance;
    // 이동용 캡슐은 씬에 PxActor를 만들지 않으므로 별도로 검사한다.
    for (const auto& capsule : m_QueryCapsules)
    {
        if (ignoredOwner && capsule->owner == ignoredOwner)
        {
            continue;
        }
        PxGeomSweepHit candidate;
        if (PxGeometryQuery::sweep(direction, distance, geometry, pose,
                CapsuleGeometry(*capsule), CapsulePose(capsule->centre), candidate, PxHitFlag::eDEFAULT))
        {
            allowed = PxMin(allowed, PxMax(0.0f, candidate.distance));
        }
    }
    return PxMin(allowed, distance);
}

bool PhysicsWorld::Raycast(const PxVec3& origin, const PxVec3& displacement,
    PxRaycastHit& hit, const void* ignoredOwner) const
{
    const float distance = displacement.magnitude();
    if (!m_Scene || !origin.isFinite() || !displacement.isFinite() ||
        !std::isfinite(distance) || distance < 1e-6f)
    {
        return false;
    }
    OwnerQueryFilter filter(ignoredOwner);
    PxRaycastBuffer result;
    // OwnerQueryFilter의 postFilter는 스윕 전용이다. 레이는 preFilter만 사용한다.
    m_Scene->raycast(origin, displacement / distance, distance, result, PxHitFlag::eDEFAULT,
        PxQueryFilterData(PxQueryFlag::eSTATIC | PxQueryFlag::eDYNAMIC | PxQueryFlag::ePREFILTER), &filter);
    if (!result.hasBlock)
    {
        return false;
    }
    hit = result.block;
    return true;
}

PhysicsWorld::CapsuleMoveResult PhysicsWorld::MoveQueryCapsule(PhysicsQueryCapsule& capsule, const PxVec3& displacement,
    bool blockHorizontal)
{
    CapsuleMoveResult result;
    if (!m_Scene || !capsule.centre.isFinite() || !displacement.isFinite())
    {
        return result;
    }
    OwnerQueryFilter filter(capsule.owner);
    filter.verticalOnly = !blockHorizontal;
    if (!blockHorizontal)
    {
        // 수평 입력은 그대로 적용하고, 새 위치에서 지면/천장과의 수직 충돌만 해결한다.
        capsule.centre += PxVec3(displacement.x, 0, displacement.z);
    }
    // 스윕을 시도하기 전에 생성/순간이동으로 발생한 파고듦을 해결한다.
    for (int iteration = 0; iteration < 8; ++iteration)
    {
        CapsulePenetration overlaps(capsule, !blockHorizontal);
        m_Scene->overlap(overlaps.geometry, overlaps.pose, overlaps,
            PxQueryFilterData(PxQueryFlag::eSTATIC | PxQueryFlag::eDYNAMIC | PxQueryFlag::ePREFILTER | PxQueryFlag::eNO_BLOCK), &filter);
        for (const auto& other : m_QueryCapsules)
        {
            if (blockHorizontal && other.get() != &capsule)
            {
                overlaps.Consider(CapsuleGeometry(*other), CapsulePose(other->centre));
            }
        }
        if (overlaps.deepest <= 0)
        {
            break;
        }
        if (blockHorizontal)
        {
            capsule.centre += overlaps.direction * PxMin(overlaps.deepest + SweepEpsilon, 2.0f);
        }
        else
        {
            // 경사면에서도 수평 위치를 밀어내지 않고 필요한 높이만 복구한다.
            capsule.centre.y += PxClamp((overlaps.deepest + SweepEpsilon) / overlaps.direction.y, -2.0f, 2.0f);
        }
    }

    const auto sweep = [&](const PxVec3& origin, const PxVec3& delta, PxGeomSweepHit& hit) {
        const float distance = delta.magnitude();
        if (distance < 1e-6f)
        {
            return false;
        }
        filter.direction = delta / distance;
        PxSweepBuffer buffer;
        const auto geometry = CapsuleGeometry(capsule, CapsuleSkin);
        const auto pose = CapsulePose(origin);
        const PxHitFlags flags = PxHitFlag::eDEFAULT | PxHitFlag::eMTD;
        m_Scene->sweep(geometry, pose, filter.direction, distance, buffer, flags,
            PxQueryFilterData(PxQueryFlag::eSTATIC | PxQueryFlag::eDYNAMIC | PxQueryFlag::ePREFILTER | PxQueryFlag::ePOSTFILTER), &filter);
        bool found = buffer.hasBlock;
        if (found)
        {
            hit = buffer.block;
        }
        // 다른 캐릭터도 프록시 바디를 등록하지 않고 쿼리 형상으로만 처리한다.
        for (const auto& other : m_QueryCapsules)
        {
            if (!blockHorizontal || other.get() == &capsule)
            {
                continue;
            }
            PxGeomSweepHit candidate;
            if (PxGeometryQuery::sweep(filter.direction, distance, geometry, pose,
                    CapsuleGeometry(*other), CapsulePose(other->centre), candidate, flags) &&
                candidate.normal.dot(filter.direction) < -1e-5f && (!found || candidate.distance < hit.distance))
            {
                hit = candidate;
                found = true;
            }
        }
        return found;
    };
    const auto travel = [](const PxVec3& delta, const PxGeomSweepHit& hit) {
        // 스윕 형상에 이미 접촉 여유 두께가 포함되어 있으므로 충돌 지점에서 정확히 멈춘다.
        // 간격을 추가하면 기존 캐릭터의 발 높이가 달라진다.
        return delta.getNormalized() * PxMax(0.0f, hit.distance);
    };
    PxGeomSweepHit floor;
    const bool floorContact = displacement.y <= 0 && sweep(capsule.centre, PxVec3(0, -.04f, 0), floor);
    const bool wasGrounded = floorContact && floor.normal.y >= WalkableNormal;
    const PxVec3 horizontal(displacement.x, 0, displacement.z);
    bool stepped = false;
    PxGeomSweepHit obstruction;
    if (blockHorizontal && floorContact && floor.normal.y > .01f && sweep(capsule.centre, horizontal, obstruction) && obstruction.normal.y < WalkableNormal)
    {
        // 기존의 25cm 계단 허용 높이를 유지한다. 세 이동 구간이 모두 막히지 않아야 하고
        // 착지면도 걸을 수 있어야 한다. 낮은 천장을 뚫고 올라가지 않도록 한다.
        const float stepHeight = PxMin(.25f, capsule.halfHeight);
        const PxVec3 up(0, stepHeight, 0), down(0, -stepHeight - .04f, 0);
        // 매우 짧은 고정 스텝 이동에서 둥근 캡슐/계단 모서리의 법선 때문에 거부하지 않도록,
        // 발판에 캡슐 바닥 영역 전체가 걸리도록 더 앞쪽까지 탐색한다.
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
            if (!sweep(capsule.centre, remaining, hit))
            {
                capsule.centre += remaining;
                break;
            }
            const auto moved = travel(remaining, hit);
            capsule.centre += moved;
            remaining -= moved;
            PxVec3 normal = hit.normal;
            if (normal.y >= WalkableNormal)
            {
                result.grounded = true;
                if (vertical)
                {
                    break; // 지면 지지가 중력을 받아내므로 내리막으로 미끄러지지 않는다.
                }
            }
            else if (normal.y < -.01f)
            {
                result.ceiling = true;
                if (vertical)
                {
                    break;
                }
            }
            else
            {
                result.wall = true;
                // 가파른 경사는 벽처럼 처리해 전진 입력이 상승 속도로 바뀌지 않게 한다.
                // 걸을 수 있는 경사는 실제 법선을 유지한다.
                if (!vertical)
                {
                    normal.y = 0;
                    normal.normalize();
                }
            }
            remaining -= normal * PxMin(0.0f, remaining.dot(normal));
        }
    };
    if (!stepped)
    {
        if (blockHorizontal)
        {
            slide(horizontal, false);
        }
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
    if (!m_CapsuleDebugVisible)
    {
        return;
    }
    constexpr PxU32 color = 0xffe8b84a;
    for (const auto& capsule : m_QueryCapsules)
    {
        const PxVec3 centre = capsule->centre;
        const float radius = capsule->radius, halfCylinder = capsule->halfHeight - radius;
        const auto line = [&](const PxVec3& a, const PxVec3& b) {
            lines.emplace_back(centre + a, centre + b, color);
        };
        for (int end : {-1, 1})
        {
            const float y = end * halfCylinder;
            for (int segment = 0; segment < 24; ++segment)
            {
                const float a = segment * PxTwoPi / 24, b = (segment + 1) * PxTwoPi / 24;
                line(PxVec3(radius * std::cos(a), y, radius * std::sin(a)), PxVec3(radius * std::cos(b), y, radius * std::sin(b)));
            }
            for (int plane = 0; plane < 2; ++plane)
            {
                for (int segment = 0; segment < 12; ++segment)
                {
                    const float a = segment * PxPi / 12, b = (segment + 1) * PxPi / 12;
                    const PxVec3 p(radius * std::cos(a), y + end * radius * std::sin(a), 0);
                    const PxVec3 q(radius * std::cos(b), y + end * radius * std::sin(b), 0);
                    line(plane ? PxVec3(0, p.y, p.x) : p, plane ? PxVec3(0, q.y, q.x) : q);
                }
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
    {
        if (auto* client = m_Scene->getScenePvdClient())
        {
            client->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_SCENEQUERIES, enabled);
        }
    }
}

PxMaterial* PhysicsWorld::CreateBodyMaterial(const PARS::PhysicsMaterialSettings& settings)
{
    if (!m_Physics || !m_Material || !settings.IsValid())
    {
        return nullptr;
    }
    const PARS::PhysicsMaterialSettings defaults;
    if (settings.staticFriction == defaults.staticFriction && settings.dynamicFriction == defaults.dynamicFriction &&
        settings.restitution == defaults.restitution && settings.frictionCombine == defaults.frictionCombine)
    {
        m_Material->acquireReference();
        return m_Material;
    }
    auto* material = m_Physics->createMaterial(settings.staticFriction, settings.dynamicFriction, settings.restitution);
    if (material)
    {
        const PxCombineMode::Enum modes[] = {PxCombineMode::eAVERAGE, PxCombineMode::eMIN, PxCombineMode::eMULTIPLY, PxCombineMode::eMAX};
        material->setFrictionCombineMode(modes[static_cast<unsigned>(settings.frictionCombine)]);
    }
    return material;
}

PxRigidDynamic* PhysicsWorld::CreateDynamicBody(const PxTransform& pose, const PxGeometry& geometry, float mass, const char* name,
    const PARS::PhysicsMaterialSettings& settings)
{
    if (!m_Scene || !pose.isValid() || !std::isfinite(mass) || mass <= 0)
    {
        return nullptr;
    }
    auto* material = CreateBodyMaterial(settings);
    if (!material)
    {
        return nullptr;
    }
    auto* body = PxCreateDynamic(*m_Physics, pose, geometry, *material, 1.0f);
    material->release(); // The shape retains its own material reference.
    if (!body)
    {
        return nullptr;
    }
    if (!PxRigidBodyExt::setMassAndUpdateInertia(*body, mass))
    {
        body->release();
        return nullptr;
    }
    body->setName(name);
    body->setSolverIterationCounts(32, 8);
    body->setAngularDamping(.3f);
    body->setLinearDamping(.05f);
    body->setMaxAngularVelocity(20);
    body->setMaxDepenetrationVelocity(3);
    m_Scene->addActor(*body);
    return body;
}

PxArticulationReducedCoordinate* PhysicsWorld::CreateArticulation()
{
    return m_Scene ? m_Physics->createArticulationReducedCoordinate() : nullptr;
}

PxArticulationLink* PhysicsWorld::CreateArticulationLink(PxArticulationReducedCoordinate& articulation,
    PxArticulationLink* parent, const PxTransform& pose, const PxGeometry& geometry, float mass, const char* name,
    const PARS::PhysicsMaterialSettings& settings)
{
    if (!pose.isValid() || !std::isfinite(mass) || mass <= 0)
    {
        return nullptr;
    }
    auto* material = CreateBodyMaterial(settings);
    if (!material)
    {
        return nullptr;
    }
    auto* link = articulation.createLink(parent, pose);
    if (!link)
    {
        material->release();
        return nullptr;
    }
    auto* shape = PxRigidActorExt::createExclusiveShape(*link, geometry, *material);
    material->release();
    if (!shape || !PxRigidBodyExt::setMassAndUpdateInertia(*link, mass))
    {
        link->release();
        return nullptr;
    }
    link->setName(name);
    link->setLinearDamping(.05f);
    link->setAngularDamping(.3f);
    link->setMaxAngularVelocity(20);
    link->setMaxDepenetrationVelocity(3);
    return link;
}

bool PhysicsWorld::AddArticulation(PxArticulationReducedCoordinate& articulation)
{
    if (!m_Scene || articulation.getScene())
    {
        return false;
    }
    m_Scene->addArticulation(articulation);
    return articulation.getScene() == m_Scene;
}

bool PhysicsWorld::AddRigidBody(PxRigidDynamic& body)
{
    if (!m_Scene || body.getScene())
    {
        return false;
    }
    m_Scene->addActor(body);
    return body.getScene() == m_Scene;
}

PxD6Joint* PhysicsWorld::CreateJoint(PxRigidActor& parent, const PxTransform& parentFrame,
    PxRigidActor& child, const PxTransform& childFrame)
{
    if (!m_Physics || parent.getScene() != m_Scene || child.getScene() != m_Scene)
    {
        return nullptr;
    }
    return PxD6JointCreate(*m_Physics, &parent, parentFrame, &child, childFrame);
}

uint32_t PhysicsWorld::GetDynamicBodyCount() const
{
    if (!m_Scene)
    {
        return 0;
    }
    uint32_t count = m_Scene->getNbActors(PxActorTypeFlag::eRIGID_DYNAMIC);
    std::vector<PxArticulationReducedCoordinate*> articulations(m_Scene->getNbArticulations());
    m_Scene->getArticulations(articulations.data(), static_cast<PxU32>(articulations.size()));
    for (const auto* articulation : articulations)
    {
        count += articulation->getNbLinks();
    }
    return count;
}

uint32_t PhysicsWorld::GetConstraintCount() const
{
    return m_Scene ? m_Scene->getNbConstraints() : 0;
}
