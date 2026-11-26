#pragma once

#include "FootSupportHull.h"
#include "FootstepIK.h"
#include "FootstepMode.h"
#include "foundation/PxFlags.h"
#include <cstdint>
#include <functional>

namespace PARS
{
    using FootstepPolygon = std::vector<physx::PxVec3>;

    // 실제 발바닥 사각형과 회복 영역의 발바닥 다각형에 같은 투영 폭을 사용한다.
    template <typename Corners>
    float FootstepSoleHalfExtent(const Corners& corners, const physx::PxVec3& center, const physx::PxVec3& axis)
    {
        float extent = 0;
        for (const physx::PxVec3& corner : corners)
        {
            extent = (std::max)(extent, std::abs((corner - center).dot(axis)));
        }
        return extent;
    }

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

    enum class FootstepPoseFailure : uint8_t
    {
        None = 0,
        Reach = 1 << 0,
        JointLimit = 1 << 1,
        OppositeLegOverlap = 1 << 2,
        InvalidInput = 1 << 3
    };

    using FootstepPoseFailures = physx::PxFlags<FootstepPoseFailure, uint8_t>;

    enum class FootstepPoseInputIssue : uint8_t
    {
        None,
        Validator,
        FootIndex,
        PhysicsAsset,
        Ragdoll,
        SoleSample,
        LegSetup,
        ThighBody,
        CalfBody,
        FootBody,
        HipParent,
        ThighShape,
        CalfShape,
        FootShape,
        ReferenceSole
    };

    enum class FootstepJointInputIssue : uint8_t
    {
        None,
        Body,
        Constraint,
        Rotation
    };

    struct FootstepJointCheck
    {
    public:
        float correctionRadians = 0;
        FootstepJointInputIssue inputIssue = FootstepJointInputIssue::None;
        // 개별 X, Y, Z 제한과 D6 스윙 원뿔 제한 순서다.
        std::array<bool, 4> limitedAxes{};
        bool usesArticulation = false;
    };

    struct FootstepPoseCheck
    {
    public:
        FootstepPoseCheck(FootstepPoseFailures failureFlags = {})
            : failures(failureFlags)
        {
        }

    public:
        static constexpr float SoleTolerance = .005f;
        static constexpr float JointToleranceRadians = .01f;

    public:
        // 이동하는 다리의 고관절, 무릎, 발목 순서다.
        std::array<FootstepJointCheck, 3> joints{};
        float soleError = 0;
        float requestedHipAnkleDistance = 0;
        float solvedHipAnkleDistance = 0;
        // 비트 번호는 기존 충돌 예측과 같이 왼쪽 부위 * 3 + 오른쪽 부위다.
        uint16_t overlapPairs = 0;
        FootstepPoseFailures failures;
        FootIKIssue ikIssue = FootIKIssue::None;
        FootstepPoseInputIssue inputIssue = FootstepPoseInputIssue::None;
        std::array<FootstepPoseInputIssue, 2> legInputIssues{};
        bool reachLimited = false;
    };

    struct FootstepRecoverySettings
    {
    public:
        static constexpr uint32_t MaximumPoseChecks = 6;

    public:
        float transferTime = .05f;
        float frontAngleDegrees = 20.0f;
        float backAngleDegrees = 30.0f;
        float clearance = .005f;
        float switchAreaRatio = .15f;
        float oneStepCaptureScale = 1.25f;
        uint32_t maximumSteps = 3;
        uint32_t maximumPoseChecks = MaximumPoseChecks;
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
        enum class ModeSelection : uint8_t
        {
            NewStep,
            GroundedNormal,
            KeepCurrent
        };

    public:
        std::function<FootstepPoseCheck(const physx::PxVec3&)> checkTargetPose;
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
        ModeSelection modeSelection = ModeSelection::NewStep;
        bool stepping = false;
        bool tryShorterFirstStep = false;
        bool locked = false;
        bool hasRejectedTarget = false;
        bool backwardFall = false;
    };

    struct FootstepNormalReach
    {
    public:
        FootstepPolygon BuildPolygon() const;

    public:
        physx::PxVec3 center{0};
        float radius = 0;
        physx::PxVec3 outward{0};
        float minimumProjection = 0;
        float minimumWidth = 0;
        float combinedHalfLength = 0;
    };

    struct FootstepRecoverySnapshot
    {
    public:
        std::array<FootstepPolygon, 3> reachable;
        std::vector<FootstepPolygon> capture;
        std::array<std::vector<FootstepPolygon>, 3> intersections;
        double elapsedMilliseconds = 0;
        FootstepNormalReach normalReach;
        physx::PxVec3 icp{0};
        physx::PxVec3 captureCenter{0};
        physx::PxVec3 target{0};
        physx::PxVec3 preferred{0};
        std::array<physx::PxVec3, FootstepRecoverySettings::MaximumPoseChecks> poseCheckPositions{
            physx::PxVec3(0), physx::PxVec3(0), physx::PxVec3(0),
            physx::PxVec3(0), physx::PxVec3(0), physx::PxVec3(0)};
        std::array<float, 3> overlapAreas{};
        FootstepMode mode = FootstepMode::Normal;
        FootstepRecoveryStatus status = FootstepRecoveryStatus::Unavailable;
        uint32_t steps = 0;
        uint32_t requiredSteps = 0;
        uint32_t poseChecks = 0;
        float predictionTime = 0;
        uint32_t capturePieceCount = 0;
        // 같은 번호의 빨간 X에서 확인된 실패 이유를 함께 보관한다.
        std::array<FootstepPoseCheck, FootstepRecoverySettings::MaximumPoseChecks> poseCheckResults{};
        uint8_t movingFoot = 0;
        bool hasTarget = false;
        bool modelFeasible = false;
        bool poseRejected = false;
        bool budgetExhausted = false;
        bool pathRejected = false;
        bool usesFutureReachApproximation = false;
        bool targetInCaptureMargin = false;
        bool captureBeyondSupport = false;
    };

    // 볼록 조각을 별도로 보존한다. 조각 사이의 금지 공간을 볼록 껍질로 채우지 않는다.
    FootstepPolygon IntersectFootstepPolygons(const FootstepPolygon& first, const FootstepPolygon& second);
    float FootstepPolygonArea(const FootstepPolygon& polygon);
    physx::PxVec3 ClosestFootstepPoint(const FootstepPolygon& polygon, const physx::PxVec3& point);
    float FootstepUnionArea(const std::vector<FootstepPolygon>& polygons, physx::PxVec3* centroid = nullptr);
    FootstepRecoverySnapshot SelectFootstepRecovery(const FootstepRecoveryInput& input,
        const FootstepRecoverySettings& settings);
    const char* FootstepModeLabel(FootstepMode mode);
    const char* FootstepRecoveryStatusLabel(FootstepRecoveryStatus status);
}
