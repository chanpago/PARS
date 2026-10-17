#pragma once
#include "PhysicsAsset.h"

class PhysicsWorld;
namespace physx { class PxRigidDynamic; class PxD6Joint; }

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
        bool IsActive() const { return !m_Bodies.empty(); }
        bool IsSimulating() const { return m_Simulating; }
        bool SetSimulating(bool simulate, Math::Vector3 velocity, std::string& error);
        // Fixed steps use targets; teleport also refreshes paused/editor poses immediately.
        bool UpdateKinematicPose(const std::vector<Math::Matrix4>& boneWorld, bool teleport, std::string& error);
        size_t GetKinematicBodyCount() const;
        size_t GetBodyCount() const { return m_Bodies.size(); }
        size_t GetJointCount() const { return m_Joints.size(); }
        const std::vector<int>& GetBoneNodes() const { return m_BoneNodes; }
        Math::Matrix4 GetBoneWorld(size_t body) const;
        Math::Vector3 GetPosition() const;
        void AddVelocity(Math::Vector3 velocity);
        // Diagnostics also used by the integration test and PVD panel.
        float GetMaxAnchorError() const;
        float GetMaxAngularError() const;
        float GetLowestPoint() const;
        bool IsFinite() const;
    private:
        PhysicsAsset m_Asset;
        std::vector<int> m_BoneNodes;
        std::vector<physx::PxRigidDynamic*> m_Bodies;
        std::vector<physx::PxD6Joint*> m_Joints;
        bool m_Simulating = false;
    };
}
