#pragma once

#include <foundation/PxVec3.h>
#include <vector>

namespace PARS
{
    struct TorsoBalanceLink
    {
    public:
        size_t body = 0;
        size_t joint = 0;
        int node = -1;
        physx::PxVec3 centerInBone{0};
        physx::PxVec3 anchorInBone{0};
        float mass = 0;
    };

    struct TorsoBalanceRig
    {
    public:
        std::vector<TorsoBalanceLink> links;
        size_t pelvis = 0;
        int pelvisNode = -1;
        physx::PxVec3 anchorInPelvis{0};
        physx::PxVec3 anchorInPelvisBone{0};
    };

    struct TorsoBalanceSample
    {
    public:
        physx::PxVec3 pivot{0};
        physx::PxVec3 pivotVelocity{0};
        physx::PxVec3 lean{0};
        physx::PxVec3 rate{0};
        float mass = 0;
        float height = 0;
        float inertia = 0;
        float gravity = 0;
    };

    // 수평 벡터의 방향은 몸통이 넘어가는 쪽이고 길이는 수직선에서 기운 각도(rad)다.
    struct TorsoBalance
    {
    public:
        void Begin(const TorsoBalanceSample& body, const physx::PxVec3& restLean);
        void AddAngularImpulse(const physx::PxVec3& angularImpulse);
        void Advance(float deltaTime, const physx::PxVec3& pivotVelocityChange);
        void Observe(const TorsoBalanceSample& body);
        bool CanFinish(float deltaTime, bool supported);

    public:
        static physx::PxVec3 GetLean(const physx::PxVec3& arm);
        physx::PxVec3 GetDirection() const;
        physx::PxVec3 GetRequestedAcceleration() const;
        physx::PxVec3 GetLandingOffset(float time) const;
        physx::PxVec3 GetHipTorque() const;
        float GetHipTorqueLimit() const;
        bool IsActive() const
        {
            return length > 0;
        }

    public:
        physx::PxVec3 neutral{0};
        physx::PxVec3 lean{0};
        physx::PxVec3 rate{0};
        float length = 0;
        float inertia = 0;
        float gravity = 0;
        float stableTime = 0;
    };
}
