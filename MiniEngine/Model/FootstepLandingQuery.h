#pragma once

#include "FootstepCollision.h"
#include "FootstepRecovery.h"

namespace PARS
{
    class SkeletalMeshComponent;
    class Ragdoll;
    struct PhysicsAsset;
    struct FootSupportSnapshot;

    class FootstepLandingQuery
    {
    private:
        struct Leg
        {
        public:
            std::array<FootstepCollisionShape, 3> shapes;
            TwoBoneIKInput pose;
            physx::PxTransform anchor{physx::PxIdentity};
            std::array<int, 4> bodies{{-1, -1, -1, -1}};
            FootstepPoseInputIssue issue = FootstepPoseInputIssue::SoleSample;
        };

    public:
        void Prepare(const SkeletalMeshComponent& mesh, const FootSupportSnapshot& support,
            std::array<FootstepReachInput, 2>& reaches);
        FootstepPoseCheck CheckPose(size_t side, const physx::PxVec3& position, const physx::PxVec3& normal,
            const physx::PxVec3& displacement) const;

    private:
        const PhysicsAsset* m_Asset = nullptr;
        const Ragdoll* m_Ragdoll = nullptr;
        std::array<Leg, 2> m_Legs;
    };
}
