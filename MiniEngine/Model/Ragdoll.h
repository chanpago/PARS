#pragma once
#include "PhysicsAsset.h"
#include <cstdint>
#include <memory>

class PhysicsWorld;
struct PhysicsGroundContact;
namespace physx
{
    class PxRigidBody;
    class PxD6Joint;
    class PxArticulationReducedCoordinate;
    class PxArticulationJointReducedCoordinate;
    class PxArticulationCache;
}

namespace PARS
{
    // 실행 중인 PhysX 객체를 소유한다. 소속 PhysicsWorld가 종료되기 전에 파괴해야 한다.
    // 에셋 복사본은 PhysX/PVD가 참조하는 이름 문자열도 소유한다.
    class Ragdoll
    {
    public:
        Ragdoll() = default;
        ~Ragdoll();
        Ragdoll(const Ragdoll&) = delete;
        Ragdoll& operator=(const Ragdoll&) = delete;
        bool Create(PhysicsWorld& world, const PhysicsAsset& asset, const std::vector<std::string>& names,
            const std::vector<Math::Matrix4>& boneWorld, Math::Vector3 velocity, std::string& error, bool simulate = true);
        void Destroy();
        void Swap(Ragdoll& other) noexcept;
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
        bool SetSimulating(bool simulate, Math::Vector3 velocity, std::string& error, bool inheritAnimationVelocity = false);
        // 애니메이션에서 새로 진입할 때와 달리, 드라이브 전환 시에는 동적 포즈와 속도를 유지한다.
        void SetDrivesEnabled(bool enabled);
        bool AreDrivesEnabled() const
        {
            return m_DrivesEnabled;
        }
        bool ApplyDriveSettings(const PhysicsAsset& asset, std::string& error);
        bool UpdateDriveTargets(const std::vector<Math::Matrix4>& boneWorld, float deltaTime, std::string& error);
        // 월드 공간의 힘/토크 드라이브로 동적 골반을 지지한다.
        // 인체 관절의 각도 드라이브와 독립적이며, simulate() 전에 갱신한다.
        bool SetRootSupported(bool supported, std::string& error);
        // Partial support keeps Y/rotation motors, optionally releases X/Z, and never teleports.
        // A zero weight removes the support joint and its associated compensation.
        bool SetRootSupport(float weight, bool horizontal, std::string& error);
        float GetRootSupportWeight() const
        {
            return m_RootSupportWeight;
        }
        bool IsRootAxisSupported(unsigned axis) const
        {
            return IsRootSupported() && axis < 6 && (m_SupportHorizontal || (axis != 0 && axis != 2));
        }
        float GetRootTargetHeight() const;
        bool IsRootSupported() const
        {
            return m_SupportBody >= 0;
        }
        float GetRootPositionError() const;
        float GetRootRotationError() const;
        size_t GetDrivenJointCount() const;
        float GetMaxDriveError(size_t* worstJoint = nullptr) const;
        float GetMaxTargetClampAngle() const
        {
            return m_MaxTargetClampAngle;
        }
        // 고정 스텝에서는 목표를 사용한다. 순간이동 시에는 일시정지/에디터 포즈도 즉시 갱신한다.
        bool UpdateKinematicPose(const std::vector<Math::Matrix4>& boneWorld, bool teleport, std::string& error);
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
        uint64_t GetContactRevision() const
        {
            return m_ContactRevision;
        }
        void AddVelocity(Math::Vector3 velocity);
        // PxActor::userData로 소유자를 식별해 이동 쿼리에서 자기 자신을 제외한다.
        void SetQueryOwner(void* owner);
        // 비교용 키네마틱 기준 더미는 접촉을 만들지 않고 원본 포즈만 따른다.
        void SetCollisionEnabled(bool enabled);
        // 실제 PA 형상과 애니메이션 목표를 스윕해 벽 너머로 이동 목표가 누적되지 않게 한다.
        Math::Vector3 ConstrainHorizontalMovement(const std::vector<Math::Matrix4>& boneWorld,
            Math::Vector3 displacement) const;
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
        void InvalidateContactState();
        uint64_t m_ContactRevision = 0;
        PhysicsAsset m_Asset;
        PhysicsWorld* m_World = nullptr;
        std::vector<std::string> m_BoneNames;
        void* m_QueryOwner = nullptr;
        bool m_CollisionEnabled = true;
        std::vector<int> m_BoneNodes;
        std::vector<physx::PxRigidBody*> m_Bodies;
        std::vector<physx::PxD6Joint*> m_Joints;
        physx::PxArticulationReducedCoordinate* m_Articulation = nullptr;
        physx::PxArticulationCache* m_ArticulationCache = nullptr;
        std::vector<physx::PxArticulationJointReducedCoordinate*> m_ArticulationJoints;
        std::vector<unsigned> m_ArticulationDofOffsets;
        std::vector<float> m_PreviousArticulationTargetVelocity;
        int m_ArticulationRoot = -1;
        bool m_Simulating = false;
        bool m_DrivesEnabled = false, m_HasDriveTargets = false;
        int m_SupportBody = -1;
        physx::PxD6Joint* m_SupportJoint = nullptr;
        bool m_HasSupportTarget = false;
        bool m_SupportHorizontal = true;
        float m_RootSupportWeight = 0;
        float m_MaxTargetClampAngle = 0;
        std::vector<Math::Quaternion> m_DriveTargets;
        std::vector<Math::Matrix4> m_LastKinematicTargets;
        std::vector<Math::Vector3> m_AnimatedLinearVelocity, m_AnimatedAngularVelocity;
        std::unique_ptr<Ragdoll> m_Standby;
    };
}
