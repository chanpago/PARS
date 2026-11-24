#pragma once

#include "FootstepIK.h"

namespace PARS
{
    struct FallArmSetup
    {
    public:
        physx::PxTransform palmInHand{physx::PxIdentity};
        physx::PxVec3 forwardInAnchor{0, 0, 1};
        physx::PxVec3 rightInAnchor{1, 0, 0};
        float upperLength = 0;
        float lowerLength = 0;
        int anchorNode = -1;
        int clavicleNode = -1;
        int shoulderNode = -1;
        int elbowNode = -1;
        int handNode = -1;
        int anchorBody = -1;
        int handBody = -1;
        bool available = false;
    };

    struct FallArmSample
    {
    public:
        FallArmSetup setup;
        physx::PxTransform actualAnchor{physx::PxIdentity};
        physx::PxTransform actualHand{physx::PxIdentity};
    };

    struct FallArmTarget
    {
    public:
        physx::PxTransform palm{physx::PxIdentity};
        physx::PxVec3 solvedPalm{0};
        physx::PxVec3 elbow{0};
        float weight = 0;
        float reachError = 0;
        bool hasTarget = false;
        bool applied = false;
        bool reachLimited = false;
    };

    struct FallArmSnapshot
    {
    public:
        std::vector<int> correctedNodes;
        std::array<FallArmTarget, 2> arms;
        bool active = false;
    };

    struct ImpactPalmGoal
    {
    public:
        bool SampleTiming(float hitAge, float approachAge, bool sustainedWound, float gripAge = -1.0f);

    public:
        static constexpr float GetWoundReleaseStart(float gripAge)
        {
            return (gripAge >= 0 ? gripAge : FullReachTime) + WoundHoldTime;
        }

    public:
        static constexpr float RecoilTime = .10f;
        static constexpr float ReachTime = .24f;
        static constexpr float FullReachTime = RecoilTime + ReachTime;
        static constexpr float WoundHoldTime = 5.0f;

    public:
        physx::PxVec3 point{0};
        physx::PxVec3 normal{0};
        float approach = 0;
        float weight = 0;
        int hitNode = -1;
        int hand = -1;
    };

    // 닿을 손을 처음 한 번 고르고, 이후에는 같은 손으로 움직이는 피격 표면을 따라간다.
    bool ApplyImpactPalmTarget(ImpactPalmGoal& goal, const std::array<FallArmSample, 2>& arms,
        const std::vector<int32_t>& parents, std::vector<physx::PxTransform>& pose, FallArmSnapshot& targets);
}
