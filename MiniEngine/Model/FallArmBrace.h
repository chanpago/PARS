#pragma once

#include "FootstepIK.h"
#include <functional>

namespace PARS
{
    struct FallArmJoint
    {
    public:
        physx::PxTransform inParentBone{physx::PxIdentity};
        physx::PxTransform inChildBone{physx::PxIdentity};
        int index = -1;
    };

    struct FallArmHandShape
    {
    public:
        float GetHalfExtent(const physx::PxVec3& axisInShape) const
        {
            return halfExtents.dot(axisInShape.abs()) + radius;
        }

    public:
        physx::PxTransform inBone{physx::PxIdentity};
        // 박스는 세 반폭, 캡슐은 X 반길이와 반지름, 구는 반지름만 사용한다.
        physx::PxVec3 halfExtents{0};
        float radius = 0;
    };

    struct FallArmSetup
    {
    public:
        physx::PxTransform palmInHand{physx::PxIdentity};
        FallArmHandShape handShape;
        physx::PxVec3 forwardInAnchor{0, 0, 1};
        physx::PxVec3 rightInAnchor{1, 0, 0};
        physx::PxVec3 upInChest{0, 1, 0};
        FallArmJoint clavicleJoint;
        // 어깨, 팔꿈치, 손목 순서의 물리 관절이다.
        std::array<FallArmJoint, 3> armJoints;
        float upperLength = 0;
        float lowerLength = 0;
        int anchorNode = -1;
        int clavicleNode = -1;
        int shoulderNode = -1;
        int elbowNode = -1;
        int handNode = -1;
        int anchorBody = -1;
        int handBody = -1;
        int chestNode = -1;
        int chestBody = -1;
        bool available = false;
    };

    struct FallArmSample
    {
    public:
        FallArmSetup setup;
        physx::PxTransform actualAnchor{physx::PxIdentity};
        physx::PxTransform actualHand{physx::PxIdentity};
        physx::PxTransform actualChest{physx::PxIdentity};
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
        bool canDriveWoundGrip = false;
    };

    struct FallArmSnapshot
    {
    public:
        std::vector<int> correctedNodes;
        std::array<FallArmTarget, 2> arms;
        bool active = false;
    };

    struct WoundHandSelection
    {
    public:
        int hand = -1;
        bool onBack = false;
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
        // 구동기와 같은 관절 좌표계에서 허용되는 회전을 구한다.
        std::function<physx::PxQuat(int, const physx::PxQuat&)> limitJointRotation;
        physx::PxVec3 point{0};
        physx::PxVec3 normal{0};
        float approach = 0;
        float weight = 0;
        // 목표 손목으로 가는 이동 중 쇄골이 분담할 비율이다.
        float clavicleReachFraction = .08f;
        int hitNode = -1;
        int hand = -1;
        WoundHandSelection selection;
    };

    inline bool PalmFitsWound(const physx::PxTransform& palm, const physx::PxTransform& target, float maxDistance)
    {
        // +Y는 손등 방향이다. 손가락이 상처 표면에서 어느 쪽을 향하는지는 접촉을 제한하지 않는다.
        return (palm.p - target.p).magnitudeSquared() <= maxDistance * maxDistance &&
            palm.q.getBasisVector1().dot(target.q.getBasisVector1()) >= physx::PxCos(physx::PxPi * 25.0f / 180.0f);
    }

    // 피격 순간의 양쪽 어깨와 골반을 기준으로 왼손(0) 또는 오른손(1)을 고른다. 중심선은 -1이다.
    WoundHandSelection SelectWoundHand(const physx::PxVec3& point, const physx::PxVec3& normal,
        const physx::PxVec3& leftShoulder, const physx::PxVec3& rightShoulder, const physx::PxVec3& pelvis);

    // 닿을 손을 처음 한 번 고르고, 이후에는 같은 손으로 움직이는 피격 표면을 따라간다.
    bool ApplyImpactPalmTarget(ImpactPalmGoal& goal, const std::array<FallArmSample, 2>& arms,
        const std::vector<int32_t>& parents, std::vector<physx::PxTransform>& pose, FallArmSnapshot& targets);
}
