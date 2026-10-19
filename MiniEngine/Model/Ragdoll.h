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
    // Owns live PhysX objects. Destroy before the owning PhysicsWorld shuts down.
    // The asset snapshot also owns the names retained by PhysX/PVD.
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
        // Drive switches preserve dynamic poses/velocities, unlike entry from animation.
        void SetDrivesEnabled(bool enabled);
        bool AreDrivesEnabled() const
        {
            return m_DrivesEnabled;
        }
        bool ApplyDriveSettings(const PhysicsAsset& asset, std::string& error);
        bool UpdateDriveTargets(const std::vector<Math::Matrix4>& boneWorld, float deltaTime, std::string& error);
        // A world-space force/torque drive supports the DYNAMIC pelvis.
        // Independent of the anatomical angular drives; updates arrive before simulate().
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
        // Fixed steps use targets; teleport also refreshes paused/editor poses immediately.
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
        // PxActor::userData identifies the owner for self-excluding movement queries.
        void SetQueryOwner(void* owner);
        // Diagnostics also used by the integration test and PVD panel.
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
