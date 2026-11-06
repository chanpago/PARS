#pragma once
#include "PhysicsAsset.h"
#include <foundation/PxVec3.h>
#include <foundation/PxTransform.h>
#include <array>
#include <cstdint>
#include <memory>

class PhysicsWorld;
struct PhysicsGroundContact;
struct PhysicsCapsuleContact;
namespace physx
{
    class PxRigidBody;
    class PxRigidDynamic;
    class PxRigidActor;
    class PxD6Joint;
    class PxArticulationReducedCoordinate;
    class PxArticulationJointReducedCoordinate;
    class PxArticulationCache;
    class PxShape;
}

namespace PARS
{
    struct RagdollSupportContact
    {
    public:
        int body = -1;
        physx::PxVec3 position{0};
        physx::PxVec3 normal{0, 1, 0};
        bool supporting = false;
    };

    struct RagdollSupportLoads
    {
    public:
        std::array<float, 2> fractions{};
        std::array<physx::PxVec3, 2> forces{physx::PxVec3(0), physx::PxVec3(0)};
    };

    // 실행 중인 PhysX 객체를 소유한다. 소속 PhysicsWorld가 종료되기 전에 파괴해야 한다.
    // 에셋 복사본은 PhysX/PVD가 참조하는 이름 문자열도 소유한다.
    class Ragdoll
    {
    public:
        Ragdoll() = default;
        ~Ragdoll();
        Ragdoll(const Ragdoll&) = delete;

    public:
        bool Create(PhysicsWorld& world, const PhysicsAsset& asset, const std::vector<std::string>& names,
            const std::vector<Math::Matrix4>& boneWorld, Math::Vector3 velocity, std::string& error, bool simulate = true);
        void Destroy();
        void Swap(Ragdoll& other) noexcept;
        bool SetSimulating(bool simulate, Math::Vector3 velocity, std::string& error, bool inheritAnimationVelocity = false);
        // 애니메이션에서 새로 진입할 때와 달리, 드라이브 전환 시에는 동적 포즈와 속도를 유지한다.
        void SetDrivesEnabled(bool enabled);
        bool ApplyDriveSettings(const PhysicsAsset& asset, std::string& error);
        bool UpdateDriveTargets(const std::vector<Math::Matrix4>& boneWorld, float deltaTime, std::string& error);
        // 월드 공간의 힘/토크 드라이브로 동적 골반을 지지한다.
        // 인체 관절의 각도 드라이브와 독립적이며, simulate() 전에 갱신한다.
        bool SetRootSupported(bool supported, std::string& error);
        // 부분 지지는 회전과 선택한 위치 모터의 강도를 조절한다. 발 지지 모드의 Y 모터는 항상 꺼진다.
        // 가중치 0은 골반 보조 조인트를 제거하며 바디 위치를 순간 이동시키지 않는다.
        bool SetRootSupport(float weight, bool horizontal, std::string& error);
        // 발 지지에서는 골반 Y 모터를 끄고 지면 하중을 관절 보상 토크에 반영한다.
        void SetFootDrivenSupport(bool enabled);
        // 고정 스텝에서는 목표를 사용한다. 순간이동 시에는 일시정지/에디터 포즈도 즉시 갱신한다.
        bool UpdateKinematicPose(const std::vector<Math::Matrix4>& boneWorld, bool teleport, std::string& error);
        // 목표 생성 주체가 바뀔 때 불연속을 목표 속도/가속도로 미분하지 않는다. 실제 바디 속도는 유지한다.
        void ResetDriveTargetHistory();
        void AddVelocity(Math::Vector3 velocity);
        // 독립 동적 바디의 질량은 충돌 솔버에만 사용한다. PA의 질량과 PD 관절에는 연결하지 않는다.
        bool ConfigureCharacterCapsule(bool enabled, float mass, float radiusScale, float skin, std::string& error,
            float heightScale = 1.0f);
        // simulate() 직전에 몸통에서 캡슐로 자세·속도를 복사한다. 반작용은 몸통에 되돌리지 않는다.
        void SyncCharacterCapsule();
        void RemoveCharacterCapsule();
        // 비교용 키네마틱 기준 더미는 접촉을 만들지 않고 원본 포즈만 따른다.
        void SetCollisionEnabled(bool enabled);
        // 실제 PA 형상과 애니메이션 목표를 스윕해 벽 너머로 이동 목표가 누적되지 않게 한다.
        Math::Vector3 ConstrainHorizontalMovement(const std::vector<Math::Matrix4>& boneWorld,
            Math::Vector3 displacement) const;

    private:
        // 활성 구성과 대기 구성은 각각 PhysX 객체를 소유하며, 하나만 씬에 등록한다.
        bool CreateConfiguration(PhysicsWorld& world, const PhysicsAsset& asset, const std::vector<std::string>& names,
            const std::vector<Math::Matrix4>& boneWorld, Math::Vector3 velocity, std::string& error, bool simulate);
        bool PrepareStandby(std::string& error);
        bool SetSceneActive(bool active, std::string& error);
        bool ResetSimulationPose(const std::vector<Math::Matrix4>& boneWorld,
            const std::vector<Math::Vector3>& linear, const std::vector<Math::Vector3>& angular, std::string& error);
        void SwapConfiguration(Ragdoll& other) noexcept;
        bool CreateArticulation(PhysicsWorld& world, const std::vector<Math::Matrix4>& boneWorld, std::string& error);
        bool RebuildSimulation(bool simulate, Math::Vector3 velocity, std::string& error, bool inheritAnimationVelocity);
        void SetArticulationVelocities(const std::vector<Math::Vector3>& linear, const std::vector<Math::Vector3>& angular);
        void UpdateArticulationDrives(float deltaTime = 0);
        void SetArticulationTarget(size_t joint, Math::Quaternion rotation, Math::Vector3 angularVelocity);
        void WakeBodies();
        void ConfigureRootSupportDrives();
        std::vector<float> ComputeSupportGeneralizedForce();
        void InvalidateContactState();
        void RefreshCharacterImpactFilters();

    public:
        Ragdoll& operator=(const Ragdoll&) = delete;

    public:
        bool IsActive() const
        {
            return !m_Bodies.empty();
        }
        bool IsSimulating() const
        {
            return m_Simulating;
        }
        bool UsesArticulation() const
        {
            return m_Articulation != nullptr;
        }
        bool AreDrivesEnabled() const
        {
            return m_DrivesEnabled;
        }
        void SetSupportContacts(const std::array<RagdollSupportContact, 2>& contacts);
        bool UsesFootDrivenSupport() const
        {
            return m_FootDrivenSupport;
        }
        const RagdollSupportLoads& GetSupportLoads() const
        {
            return m_SupportLoads;
        }
        float GetRootSupportWeight() const
        {
            return m_RootSupportWeight;
        }
        bool IsRootAxisSupported(unsigned axis) const
        {
            return IsRootSupported() && axis < 6 && !(axis == 1 && m_FootDrivenSupport) &&
                (m_SupportHorizontal || (axis != 0 && axis != 2));
        }
        float GetRootTargetHeight() const;
        bool IsRootSupported() const
        {
            return m_SupportBody >= 0;
        }
        float GetRootPositionError() const;
        Math::Vector3 GetRootSupportForce() const;
        float GetRootRotationError() const;
        size_t GetDrivenJointCount() const;
        float GetMaxDriveError(size_t* worstJoint = nullptr) const;
        float GetMaxTargetClampAngle() const
        {
            return m_MaxTargetClampAngle;
        }
        size_t GetKinematicBodyCount() const;
        size_t GetBodyCount() const
        {
            return m_Bodies.size();
        }
        size_t GetJointCount() const
        {
            return m_Joints.size();
        }
        const std::vector<int>& GetBoneNodes() const
        {
            return m_BoneNodes;
        }
        Math::Matrix4 GetBoneWorld(size_t body) const;
        Math::Vector3 GetPosition() const;
        // 실제 바디의 월드 질량중심을 질량으로 가중 평균한다. 바디가 없으면 false다.
        bool GetCenterOfMass(Math::Vector3& center, float& totalMass) const;
        // 바디 질량중심의 실제 선속도를 질량으로 가중 평균한다. 단위는 m/s다.
        bool GetCenterOfMassVelocity(Math::Vector3& velocity) const;
        bool GetBodyGroundContact(size_t body, PhysicsGroundContact& contact) const;
        float GetExternalImpactImpulse() const;
        int GetImpactRootBody() const;
        bool IsCharacterImpactBody(size_t body) const;
        int FindPhysicsBody(const physx::PxRigidActor* actor) const;
        physx::PxVec3 GetBodyPointVelocity(size_t body, physx::PxVec3 point) const;
        float GetImpactRootMass() const;
        float GetCharacterCapsuleMass() const;
        void GetCharacterCapsuleContacts(std::vector<PhysicsCapsuleContact>& contacts) const;
        bool HasCharacterCapsule() const
        {
            return m_CharacterCapsuleShape != nullptr;
        }
        bool GetCharacterImpactCapsule(float radiusScale, physx::PxTransform& pose, float& radius,
            float& halfLength, physx::PxVec3& targetOffset, float heightScale = 1.0f) const;
        void SetCharacterImpactEnabled(bool enabled, uint32_t ownerGroup);
        bool UsesCharacterImpact() const
        {
            return m_CharacterImpactEnabled;
        }
        uint64_t GetContactRevision() const
        {
            return m_ContactRevision;
        }
        // PxActor::userData로 소유자를 식별해 이동 쿼리에서 자기 자신을 제외한다.
        void SetQueryOwner(void* owner);
        // 통합 테스트와 PVD 패널에서도 사용하는 진단 정보다.
        float GetMaxAnchorError() const;
        float GetMaxAngularError(size_t* worstJoint = nullptr) const;
        float GetLowestPoint() const;
        bool IsFinite() const;

    private:
        static constexpr float PelvisLinearFrequency = 30.0f;
        static constexpr float PelvisLinearDampingRatio = 1.0f;
        static constexpr float PelvisAngularFrequency = 16.0f;
        static constexpr float PelvisAngularDampingRatio = 1.0f;

    private:
        uint64_t m_ContactRevision = 0;
        PhysicsAsset m_Asset;
        PhysicsWorld* m_World = nullptr;
        std::vector<std::string> m_BoneNames;
        void* m_QueryOwner = nullptr;
        std::vector<int> m_BoneNodes;
        std::vector<physx::PxRigidBody*> m_Bodies;
        std::vector<physx::PxD6Joint*> m_Joints;
        physx::PxArticulationReducedCoordinate* m_Articulation = nullptr;
        physx::PxArticulationCache* m_ArticulationCache = nullptr;
        std::vector<physx::PxArticulationJointReducedCoordinate*> m_ArticulationJoints;
        std::vector<unsigned> m_ArticulationDofOffsets;
        std::vector<float> m_PreviousArticulationTargetVelocity;
        physx::PxD6Joint* m_SupportJoint = nullptr;
        std::vector<Math::Quaternion> m_DriveTargets;
        std::vector<Math::Matrix4> m_LastKinematicTargets;
        std::vector<Math::Vector3> m_AnimatedLinearVelocity;
        std::vector<Math::Vector3> m_AnimatedAngularVelocity;
        std::unique_ptr<Ragdoll> m_Standby;
        physx::PxRigidDynamic* m_CharacterCapsuleBody = nullptr;
        physx::PxShape* m_CharacterCapsuleShape = nullptr;
        int m_ArticulationRoot = -1;
        int m_SupportBody = -1;
        float m_RootSupportWeight = 0;
        std::array<RagdollSupportContact, 2> m_SupportContacts;
        RagdollSupportLoads m_SupportLoads;
        float m_MaxTargetClampAngle = 0;
        physx::PxTransform m_CapsuleRootPose{physx::PxIdentity};
        float m_CapsuleRadiusScale = 0;
        float m_CapsuleHeightScale = 0;
        float m_CapsuleSkin = 0;
        uint32_t m_CharacterImpactGroup = 0;
        bool m_CollisionEnabled = true;
        bool m_CharacterImpactEnabled = false;
        bool m_Simulating = false;
        bool m_DrivesEnabled = false;
        bool m_HasDriveTargets = false;
        bool m_HasSupportTarget = false;
        bool m_SupportHorizontal = true;
        bool m_FootDrivenSupport = false;
    };
}
