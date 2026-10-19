#pragma once
#include "PhysicsAsset.h"

class PhysicsWorld;
namespace physx
{
    class PxRigidDynamic;
    class PxD6Joint;
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
        void AddVelocity(Math::Vector3 velocity);
        // PxActor::userData로 소유자를 식별해 이동 쿼리에서 자기 자신을 제외한다.
        void SetQueryOwner(void* owner);
        // 통합 테스트와 PVD 패널에서도 사용하는 진단 정보다.
        float GetMaxAnchorError() const;
        float GetMaxAngularError(size_t* worstJoint = nullptr) const;
        float GetLowestPoint() const;
        bool IsFinite() const;

    private:
        PhysicsAsset m_Asset;
        std::vector<int> m_BoneNodes;
        std::vector<physx::PxRigidDynamic*> m_Bodies;
        std::vector<physx::PxD6Joint*> m_Joints;
        bool m_Simulating = false;
        bool m_DrivesEnabled = false, m_HasDriveTargets = false;
        int m_SupportBody = -1;
        physx::PxD6Joint* m_SupportJoint = nullptr;
        bool m_HasSupportTarget = false;
        float m_MaxTargetClampAngle = 0;
        std::vector<Math::Quaternion> m_DriveTargets;
        std::vector<Math::Matrix4> m_LastKinematicTargets;
        std::vector<Math::Vector3> m_AnimatedLinearVelocity, m_AnimatedAngularVelocity;
    };
}
