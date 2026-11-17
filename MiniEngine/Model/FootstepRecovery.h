#pragma once

#include "FootSupportHull.h"
#include <cstdint>
#include <functional>

namespace PARS
{
    using FootstepPolygon = std::vector<physx::PxVec3>;

    enum class FootstepMode
    {
        Normal,
        FrontCross,
        BackCross
    };

    enum class FootstepRecoveryStatus
    {
        Unavailable,
        Preview,
        Feasible,
        BestEffort,
        NoReach,
        PoseRejected,
        Locked
    };

    struct FootstepRecoverySettings
    {
    public:
        float transferTime = .05f;
        float frontAngleDegrees = 20.0f;
        float backAngleDegrees = 30.0f;
        float clearance = .005f;
        float switchAreaRatio = .15f;
        uint32_t maximumSteps = 3;
        uint32_t maximumPoseChecks = 6;
        bool enabled = true;
    };

    struct FootstepReachInput
    {
    public:
        FootstepPolygon sole;
        physx::PxVec3 position{0};
        physx::PxVec3 hip{0};
        physx::PxVec3 ankleOffset{0};
        physx::PxVec3 outward{0};
        physx::PxVec3 stepOrigin{0};
        float legLength = 0;
        float minimumWidth = 0;
        float maximumDistance = 1;
        bool available = false;
    };

    struct FootstepRecoveryInput
    {
    public:
        std::function<bool(const physx::PxVec3&)> acceptsTarget;
        std::array<FootstepReachInput, 2> feet;
        size_t movingFoot = 0;
        physx::PxVec3 com{0};
        physx::PxVec3 velocity{0};
        physx::PxVec3 forward{0, 0, 1};
        physx::PxVec3 preferred{0};
        physx::PxVec3 normalTarget{0};
        physx::PxVec3 lockedTarget{0};
        physx::PxVec3 rejectedTarget{0};
        float groundHeight = 0;
        float firstStepTime = 0;
        float followingStepTime = 0;
        FootstepMode previousMode = FootstepMode::Normal;
        bool stepping = false;
        bool locked = false;
        bool hasRejectedTarget = false;
    };

    struct FootstepRecoverySnapshot
    {
    public:
        std::array<FootstepPolygon, 3> reachable;
        std::vector<FootstepPolygon> capture;
        std::array<std::vector<FootstepPolygon>, 3> intersections;
        double elapsedMilliseconds = 0;
        physx::PxVec3 icp{0};
        physx::PxVec3 target{0};
        physx::PxVec3 preferred{0};
        std::array<float, 3> overlapAreas{};
        FootstepMode mode = FootstepMode::Normal;
        FootstepRecoveryStatus status = FootstepRecoveryStatus::Unavailable;
        uint32_t steps = 0;
        uint32_t poseChecks = 0;
        float predictionTime = 0;
        uint32_t capturePieceCount = 0;
        bool hasTarget = false;
        bool modelFeasible = false;
        bool poseRejected = false;
        bool budgetExhausted = false;
        bool pathRejected = false;
        bool usesFutureReachApproximation = false;
    };

    // 볼록 조각을 별도로 보존한다. 조각 사이의 금지 공간을 볼록 껍질로 채우지 않는다.
    FootstepPolygon IntersectFootstepPolygons(const FootstepPolygon& first, const FootstepPolygon& second);
    float FootstepPolygonArea(const FootstepPolygon& polygon);
    physx::PxVec3 ClosestFootstepPoint(const FootstepPolygon& polygon, const physx::PxVec3& point);
    float FootstepUnionArea(const std::vector<FootstepPolygon>& polygons);
    FootstepRecoverySnapshot SelectFootstepRecovery(const FootstepRecoveryInput& input,
        const FootstepRecoverySettings& settings);
    const char* FootstepModeLabel(FootstepMode mode);
    const char* FootstepRecoveryStatusLabel(FootstepRecoveryStatus status);
}
