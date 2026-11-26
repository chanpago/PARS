#pragma once

#include <PxPhysicsAPI.h>
#include "PhysicsMaterial.h"
#include "ImpactImpulseSettings.h"
#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

// 쿼리용 형상만 사용한다. PxActor/PxShape나 CCT 프록시 바디는 생성하지 않는다.
struct PhysicsQueryCapsule
{
public:
    const void* owner = nullptr; // 이동 쿼리에서는 이 소유자로 표시된 강체 Actor만 제외한다.
    physx::PxVec3 centre;
    float radius;
    float halfHeight;
};

// 캐릭터 밀침 쿼리 결과다. 이 포인터는 해당 쿼리의 처리 중에만 사용한다.
struct PhysicsCapsuleHit
{
public:
    const physx::PxRigidActor* actor = nullptr;
    const void* owner = nullptr;
    physx::PxVec3 position{0};
    physx::PxVec3 normal{0};
    float distance = 0;
};

// 실제 몸통 캡슐 접촉에서 PhysX 솔버가 상대 바디에 전달한 충격량이다.
struct PhysicsCapsuleContact
{
public:
    const physx::PxRigidActor* actor = nullptr;
    const void* owner = nullptr;
    physx::PxVec3 position{0};
    physx::PxVec3 impulse{0};
    physx::PxVec3 normal{0};
    float normalImpulse = 0;
};

// 마지막으로 완료한 물리 스텝의 실제 지면 접촉이다. 레이 히트와 구분한다.
struct PhysicsGroundContact
{
public:
    physx::PxVec3 position{0};
    physx::PxVec3 normal{0};
    uint32_t pointCount = 0;
    // 완료한 물리 스텝의 법선 충격량 합이며 접촉 판정 기준은 바꾸지 않는다.
    float normalImpulse = 0;
};

// 키네마틱 상태에서 받은 접촉 충격을 동적 전환 후 이어받는다.
struct PhysicsContactImpulse
{
public:
    uint64_t impactContactId = 0;
    physx::PxVec3 position{0};
    physx::PxVec3 impulse{0};
    physx::PxVec3 outwardNormal{0};
    physx::PxVec3 capsuleVelocity{0};
    uint32_t capsuleGroup = 0;
    bool fromProjectile = false;
    bool newProjectileHit = false;
    bool fromCharacterCapsule = false;
    bool fromAnimatedCapsule = false;
    bool newCapsuleHit = false;
};

// 최초 접촉점과 진입 방향을 상대 바디의 로컬 좌표로 보관한다.
struct PhysicsProjectileAttachment
{
public:
    std::string targetName;
    const void* targetOwner = nullptr;
    const physx::PxRigidActor* target = nullptr;
    physx::PxTransform localPose{physx::PxIdentity};
};

// 미터, 킬로그램, 초 단위의 CPU 물리이며 +Y가 위쪽이다. 게임 스레드에서 사용한다.
// Create*가 반환한 Actor는 명시적으로 해제하거나 Shutdown()을 호출할 때까지 씬이 소유한다.
// Update()/Step()을 호출하기 전에 모든 읽기와 쓰기를 끝내야 한다.
class PhysicsWorld : private physx::PxSimulationEventCallback, private physx::PxDeletionListener
{
public:
    using BeforeStep = std::function<void(float)>;
    struct CapsuleMoveResult
    {
    public:
        bool grounded = false;
        bool ceiling = false;
        bool wall = false;
    };

private:
    struct ContactPair
    {
    public:
        const physx::PxActor* actors[2]{};
        const physx::PxShape* shapes[2]{};
        std::vector<physx::PxContactPairPoint> points;
        uint64_t completedStep = 0;
        uint64_t clockRevision = 0;
        uint64_t firstImpulseStep = 0;
        uint64_t impactContactId = 0;
        physx::PxVec3 capsuleVelocity{0};
        bool beginsImpact = false;
    };
    struct ImpactSource
    {
    public:
        const physx::PxRigidDynamic* actor = nullptr;
        ImpactImpulseSettings settings;
    };
    struct ImpactContact
    {
    public:
        const void* source = nullptr;
        const void* receiver = nullptr;
        uint64_t id = 0;
        uint64_t firstImpulseStep = 0;
        physx::PxVec3 incomingDirection{0};
        physx::PxVec3 contactImpulse{0};
        physx::PxVec3 weightedPosition{0};
        float weight = 0;
        ImpactImpulseSettings settings;
        bool groundedArrowHit = false;
        bool groundedProjectileHit = false;
        bool consumed = false;
    };
    struct StoppedProjectile
    {
    public:
        PhysicsProjectileAttachment attachment;
        physx::PxRigidDynamic* actor = nullptr;
        uint64_t step = 0;
        float weight = -1;
    };
    class ErrorCallback : public physx::PxDefaultErrorCallback
    {
    public:
        void reportError(physx::PxErrorCode::Enum code, const char* message,
            const char* file, int line) override;

    public:
        std::atomic<uint32_t> errors{0};
    };

public:
    PhysicsWorld() = default;
    ~PhysicsWorld();
    PhysicsWorld(const PhysicsWorld&) = delete;

public:
    bool Initialize(bool connectPvd = true);
    // SDK와 워커만 공유한다. 씬/시계/접촉은 독립적이며 owner보다 먼저 종료해야 한다.
    bool InitializeScene(PhysicsWorld& owner);
    void Shutdown();
    uint32_t Update(double elapsedSeconds, const BeforeStep& beforeStep = {});
    void Step(const BeforeStep& beforeStep = {});
    void ResetClock();
    physx::PxRigidStatic* CreateStaticBox(const physx::PxTransform& pose,
        const physx::PxVec3& halfExtents, const char* name,
        const PARS::PhysicsMaterialSettings& material = {});
    physx::PxRigidDynamic* CreateDynamicBox(const physx::PxTransform& pose,
        const physx::PxVec3& halfExtents, float density, const char* name);
    PhysicsQueryCapsule* CreateQueryCapsule(const physx::PxVec3& centre,
        float radius, float halfHeight);
    void DestroyQueryCapsule(PhysicsQueryCapsule* capsule);
    CapsuleMoveResult MoveQueryCapsule(PhysicsQueryCapsule& capsule, const physx::PxVec3& displacement,
        bool blockHorizontal = true, bool blockCharacters = true);
    // 수신 정책이 켜진 캐릭터 PA만 수집한다. 여러 바디와 초기 겹침도 누락하지 않는다.
    void SweepCharacterCapsule(const physx::PxTransform& pose, float radius, float halfLength,
        const physx::PxVec3& displacement, const void* ignoredOwner, std::vector<PhysicsCapsuleHit>& hits) const;
    // 구형 프로브가 장애물에 닿기 전까지 이동할 수 있는 거리(m). 자기 Actor와 트리거는 제외한다.
    float SweepSphereDistance(const physx::PxVec3& origin, const physx::PxVec3& displacement,
        float radius, const void* ignoredOwner = nullptr) const;
    // 변위 길이 안의 가장 가까운 표면. 자기 Actor와 트리거는 제외한다.
    bool Raycast(const physx::PxVec3& origin, const physx::PxVec3& displacement,
        physx::PxRaycastHit& hit, const void* ignoredOwner = nullptr) const;
    // 순간이동하거나 씬에서 분리하기 전에 이전 접촉을 무효화한다.
    void InvalidateContacts(const physx::PxRigidActor& body);
    // 고정 스텝의 시뮬레이션 직전에 한 발 이상 지면을 딛고 있는 피격자를 기록한다.
    void RecordGroundedImpactReceiver(const void* owner);
    bool ConsumeImpactContact(uint64_t id);
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

private:
    void RecordImpactContact(ContactPair& pair, int sourceSide, bool capsule, const physx::PxContactPairVelocity* beforeContact);
    void RecordProjectileAttachment(physx::PxRigidDynamic& projectile, const physx::PxRigidActor& target, const ContactPair& pair,
        int side, const physx::PxContactPairVelocity* velocity, const physx::PxContactPairPose* pose);
    void onContact(const physx::PxContactPairHeader& header, const physx::PxContactPair* pairs,
        physx::PxU32 count) override;
    void onRelease(const physx::PxBase* object, void* userData, physx::PxDeletionEventFlag::Enum event) override;
    void onConstraintBreak(physx::PxConstraintInfo*, physx::PxU32) override;
    void onWake(physx::PxActor**, physx::PxU32) override;
    void onSleep(physx::PxActor**, physx::PxU32) override;
    void onTrigger(physx::PxTriggerPair*, physx::PxU32) override;
    void onAdvance(const physx::PxRigidBody* const*, const physx::PxTransform*, physx::PxU32) override;
    bool CreateScene();
    physx::PxMaterial* CreateBodyMaterial(const PARS::PhysicsMaterialSettings& settings);

public:
    PhysicsWorld& operator=(const PhysicsWorld&) = delete;

public:
    // 같은 소유자의 바디도 포함해 PhysX가 보고한 실제 접촉을 조회한다.
    bool HasContact(const physx::PxRigidActor& first, const physx::PxRigidActor& second) const;
    bool GetImpactResponse(uint64_t id, PhysicsImpactResponse& response) const;
    bool SetImpactImpulseSettings(const physx::PxRigidDynamic& source, const ImpactImpulseSettings& settings);
    bool SetProjectileBodyImpulse(const physx::PxRigidDynamic& projectile, float bodyMagnitude,
        float torqueMultiplier, float partMagnitude = 0);
    const PhysicsProjectileAttachment* GetProjectileAttachment(const physx::PxRigidDynamic& projectile) const;
    bool GetGroundContact(const physx::PxRigidActor& body, PhysicsGroundContact& contact,
        const void* ignoredOwner = nullptr) const;
    void GetCharacterCapsuleContacts(const physx::PxRigidActor& body,
        std::vector<PhysicsCapsuleContact>& contacts) const;
    // 마지막 완료 스텝의 외부 충격량 합(N s). 자기 충돌과 위쪽 지면 지지 접촉은 제외한다.
    float GetExternalImpactImpulse(const physx::PxRigidActor& body, const void* ignoredOwner = nullptr,
        std::vector<PhysicsContactImpulse>* contacts = nullptr) const;
    // 다음 고정 스텝에 접근 충돌이 예상되면 솔버 실행 전에 동적으로 전환할 수 있다.
    bool HasApproachingImpact(const physx::PxRigidBody& body, float totalMass, float minimumDeltaVelocity,
        const void* ignoredOwner, float deferCapsuleDeltaVelocity = PX_MAX_F32) const;
    uint64_t GetClockRevision() const
    {
        return m_ClockRevision;
    }
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

public:
    // 캐릭터 PA는 word3의 이 비트와 word2의 소유자 번호를 사용한다.
    static constexpr uint32_t CharacterImpactFilterFlag = 1u << 31;
    // 몸통 캡슐의 수신 대상인 루트와 상체 바디만 설정한다.
    static constexpr uint32_t CharacterImpactTargetFilterFlag = 1u << 30;
    // 질량·PD에서 분리된 독립 동적 몸통 캡슐을 기존 PA 형상과 구분한다.
    static constexpr uint32_t CharacterCapsuleFilterFlag = 1u << 29;
    // 이 PA의 소유자가 활성 몸통 캡슐을 가지고 있을 때만 설정한다.
    static constexpr uint32_t CharacterCapsuleOwnerFilterFlag = 1u << 28;
    // 발사체와 실제로 접촉한 본을 자세 구동기에서 식별한다.
    static constexpr uint32_t ProjectileFilterFlag = 1u << 27;
    // 조준 발사한 공은 첫 충돌 스텝부터 다시 중력을 받는다.
    static constexpr uint32_t AimedProjectileFilterFlag = 1u << 26;
    // 독립 캡슐의 운동을 애니메이션 플레이어가 구동하는지 접촉에 기록한다.
    static constexpr uint32_t AnimatedCapsuleFilterFlag = 1u << 25;
    static constexpr uint32_t AttachOnContactFilterFlag = 1u << 24;
    static constexpr double FixedDeltaTime = 1.0 / 60.0;
    static constexpr uint32_t MaxSubsteps = 8;

private:
    std::vector<ContactPair> m_Contacts;
    std::vector<ImpactSource> m_ImpactSources;
    std::vector<ImpactContact> m_ImpactContacts;
    std::vector<const void*> m_GroundedImpactReceivers;
    std::vector<StoppedProjectile> m_StoppedProjectiles;
    uint64_t m_NextImpactContactId = 1;
    std::vector<physx::PxRigidDynamic*> m_ImpactedAimedProjectiles;
    uint64_t m_ClockRevision = 0;
    PhysicsWorld* m_ServiceOwner = nullptr;
    physx::PxDefaultAllocator m_Allocator;
    ErrorCallback m_ErrorCallback;
    physx::PxFoundation* m_Foundation = nullptr;
    physx::PxPvd* m_Pvd = nullptr;
    physx::PxPvdTransport* m_Transport = nullptr;
    physx::PxPhysics* m_Physics = nullptr;
    physx::PxDefaultCpuDispatcher* m_Dispatcher = nullptr;
    physx::PxScene* m_Scene = nullptr;
    std::vector<std::unique_ptr<PhysicsQueryCapsule>> m_QueryCapsules;
    physx::PxMaterial* m_Material = nullptr;
    double m_Accumulator = 0.0;
    double m_DroppedTime = 0.0;
    uint64_t m_StepCount = 0;
    uint32_t m_LastSubsteps = 0;
    bool m_ListeningForDeletion = false;
    bool m_CapsuleDebugVisible = true;
    bool m_ExtensionsInitialized = false;
};
