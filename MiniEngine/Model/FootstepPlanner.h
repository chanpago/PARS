#pragma once

#include "FootSupportHull.h"
#include "FootstepRecovery.h"
#include <array>
#include <cstdint>
#include <limits>

namespace PARS
{
    constexpr float MinimumFootstepDuration = .13f;
    constexpr float MaximumFootstepDuration = .27f;

    struct ImpactWarmupStep
    {
    public:
        // 루트의 이동을 제외한 발의 착지 편위이며 방향은 월드 수평축 기준이다.
        physx::PxVec3 stride{0};
        float duration = 0;
    };

    ImpactWarmupStep PlanImpactWarmupStep(const FootstepPolygon& hull, const physx::PxVec3& com,
        const physx::PxVec3& velocity, float maximumStride, float fixedDeltaTime);
    // 두 발의 시작 위치는 루트 회전축 기준 월드 오프셋이다. 회전각은 월드 Y축 기준 라디안이다.
    float SelectImpactWarmupYaw(const physx::PxVec3& forward, const physx::PxVec3& velocity, float turnPreference,
        const physx::PxVec3& swingOffset, const physx::PxVec3& supportOffset, const physx::PxVec3& firstStride);

    struct FootstepSettings
    {
    public:
        FootstepRecoverySettings recovery;
        float startPredictionTime = .6f;
        float landingPredictionTime = .4f;
        float stanceScale = 1.0f;
        float minimumStanceWidthRatio = .0f;
        float maximumStepLength = 1.0f; // 각 발 명령의 출발점에서 허용하는 최대 이동 거리(m)
        float crossingOffset = .1f;
        float liftHeightScale = .20f;
        float backwardLiftTimeRatio = 1.25f; // 완전히 뒤로 디딜 때 상승 시간과 하강 시간의 비
        float stepDuration = MinimumFootstepDuration;
        float minimumLiftHeight = .03f;
        float landingLockProgress = .90f;
        float landingLoweringSpeed = .05f;
        float maximumLandingLowering = .03f;
        float supportLoadRatio = .25f;
        float supportConfirmationTime = .01f;
        float landingComClampHysteresisDegrees = 5.0f;
        bool preventCrossing = true;
        bool landingComClampEnabled = true;
        bool pathAvoidanceEnabled = false;
    };

    enum class FootstepPhase
    {
        Idle,
        AwaitingTakeoff,
        Airborne,
        Landed,
        Aborted
    };

    enum class FootstepReason
    {
        NotEvaluated,
        ValidTarget,
        Inactive,
        AutomaticSupportRequired,
        IdleRequired,
        InvalidSettings,
        InvalidCom,
        NoSupportArea,
        NoReference,
        NoCalfLength,
        NoStepDistance,
        NoContactSample,
        PredictedInside,
        NoFootRequest,
        NoGroundPlane,
        InvalidTarget,
        WaitingTakeoff,
        InFlight,
        Landed,
        OutsideBuffer,
        RootSupportReleased,
        WaitingSupportLoad
    };

    struct FootstepFootInput
    {
    public:
        FootstepReachInput reach;
        physx::PxVec3 position{0};
        physx::PxVec3 soleVelocity{0};
        physx::PxVec3 contactPosition{0};
        physx::PxVec3 normal{0};
        physx::PxVec3 stanceOffset{0};
        std::array<physx::PxVec3, 4> corners{physx::PxVec3(0), physx::PxVec3(0), physx::PxVec3(0), physx::PxVec3(0)};
        float calfLength = 0;
        float normalImpulse = 0;
        float timeToReachLimit = std::numeric_limits<float>::infinity();
        bool available = false;
        bool hasContactSample = false;
        bool grounded = false;
        bool landed = false;
        bool hasSoleRectangle = false;
    };

    struct FootstepInput
    {
    public:
        std::function<bool(size_t, const physx::PxVec3&)> acceptsLanding;
        std::vector<physx::PxVec3> hull;
        std::vector<physx::PxVec3> bufferHull;
        uint64_t completedStep = 0;
        uint64_t revision = 0;
        std::array<FootstepFootInput, 2> feet;
        physx::PxVec3 com{0};
        physx::PxVec3 velocity{0};
        physx::PxVec3 supportRight{1, 0, 0};
        physx::PxVec3 impactPosition{0};
        FootSupportZone zone = FootSupportZone::Invalid;
        float projectionHeight = 0;
        float fixedDeltaTime = 0;
        float bodyWeight = 0;
        int requestedFoot = -1; // 명령 대상만 나타내며 반대 발에 역할을 부여하지 않는다.
        bool active = false;
        bool automaticSupport = false;
        bool idleAnimation = false;
        bool rootReleased = false;
        bool fallingBackward = false;
        bool pelvisGrounded = false;
        bool hasFloorProjection = false;
        bool hasCom = false;
        bool hasVelocity = false;
        bool hasReference = false;
        bool hasImpact = false;
    };

    enum class FootstepLandingDirection
    {
        None,
        Front,
        Back
    };

    // 현재 COM에서 지지발 사각형의 양쪽 접선 꼭짓점으로 향하는 금지 부채꼴이다.
    struct FootstepLandingRegion
    {
    public:
        std::array<physx::PxVec3, 4> corners{physx::PxVec3(0), physx::PxVec3(0), physx::PxVec3(0), physx::PxVec3(0)};
        physx::PxVec3 origin{0};
        physx::PxVec3 frontRay{0};
        physx::PxVec3 backRay{0};
        physx::PxVec3 centerRay{0};
        float halfAngleRadians = 0;
        int supportFoot = -1;
        bool valid = false;
        bool containsCom = false;
    };

    FootstepLandingRegion BuildFootstepLandingRegion(const physx::PxVec3& origin,
        const std::array<physx::PxVec3, 4>& corners, const physx::PxVec3& forward);
    bool IsInsideFootstepLandingRegion(const FootstepLandingRegion& region, const physx::PxVec3& point);

    // 시작·착지 예측의 원래 값과 보정 결과를 실제 물리 상태와 구분해 저장한다.
    struct FootstepComPrediction
    {
    public:
        physx::PxVec3 origin{0};
        physx::PxVec3 raw{0};
        physx::PxVec3 clamped{0};
        FootstepLandingRegion region;
        FootstepLandingDirection direction = FootstepLandingDirection::None;
        bool enabled = false;
        bool limited = false;
        bool valid = false;
    };

    struct FootstepTarget
    {
    public:
        FootstepRecoverySnapshot recovery;
        FootstepReason reason = FootstepReason::NotEvaluated;
        physx::PxVec3 raw{0};
        physx::PxVec3 position{0};
        physx::PxVec3 normal{0};
        float travelDistance = 0;
        float separationDistance = 0;
        FootstepComPrediction landingCom;
        bool valid = false;
        bool lengthLimited = false;
        bool crossingLimited = false;
        bool stanceWidthLimited = false;
    };

    struct FootstepMotion
    {
    public:
        physx::PxVec3 position{0};
        physx::PxVec3 swingOutwardOffset{0};
        float initialHeight = 0;
        float liftTimeRatio = 1.0f;
        float heightLimit = 0;
        float height = 0;
        float takeoffElapsedTime = 0;
        float elapsedTime = 0;
        float duration = 0;
        float measuredProgress = 0;
        float progress = 0;
        float progressPerSecond = 0;
        float remainingDistance = 0;
        float landingError = 0;
        float landingTimeError = 0;
        float landingLockProgress = 0;
        float landingLoweringSpeed = 0;
        float maximumLandingLowering = 0;
        float landingLowering = 0;
        bool valid = false;
        bool hasLandingError = false;
        bool landingTargetLocked = false;
        bool hasSupportDeadline = false;
    };

    // 명령과 실제 접촉은 독립적이며 이륙 이력으로 공중 이동과 착지를 확인한다.
    struct FootstepCommand
    {
    public:
        bool IsInProgress() const
        {
            return phase == FootstepPhase::AwaitingTakeoff || phase == FootstepPhase::Airborne;
        }

        bool UsesSwingTrajectory() const
        {
            return hasTakenOff || motion.hasSupportDeadline;
        }

    public:
        uint64_t startedStep = 0;
        uint64_t takeoffStep = 0;
        uint64_t trajectoryStartStep = 0;
        FootstepTarget target;
        physx::PxVec3 startPosition{0};
        physx::PxVec3 initialTarget{0};
        physx::PxVec3 takeoffPosition{0};
        FootstepMotion motion;
        float initialDistance = 0;
        FootstepPhase phase = FootstepPhase::Idle;
        FootstepReason reason = FootstepReason::NotEvaluated;
        FootstepLandingDirection landingComDirection = FootstepLandingDirection::None;
        bool hasTarget = false;
        bool hasTakenOff = false;
        bool justStarted = false;
        bool justLanded = false;
    };

    // 지지 시한이 있으면 발을 드는 구간도 같은 궤적에 포함한다.
    float GetFootstepRemainingTime(const FootstepCommand& command);
    physx::PxVec3 SampleFootstepPosition(const FootstepCommand& command, float timeAhead);

    struct FootstepLoadTransfer
    {
    public:
        std::array<float, 2> loadRatios{};
        std::array<float, 2> confirmationTimes{};
        bool required = false;
        bool ready = false;
    };

    struct FootstepSnapshot
    {
    public:
        bool IsInProgress() const;
        bool HasTimedOutOutsideBuffer() const;
        float GetMaximumInitialDistance() const;

    public:
        uint64_t completedStep = 0;
        uint64_t revision = 0;
        uint64_t startCount = 0;
        uint64_t landingCount = 0;
        uint64_t abortCount = 0;
        double outsideBufferTime = 0.0;
        std::array<FootstepCommand, 2> feet;
        std::array<FootstepTarget, 2> candidates;
        FootstepComPrediction startCom;
        physx::PxVec3 predictedCom{0};
        physx::PxVec3 groundPosition{0};
        physx::PxVec3 groundNormal{0, 1, 0};
        physx::PxVec3 lastImpactPosition{0};
        FootstepLoadTransfer loadTransfer;
        FootstepReason reason = FootstepReason::Inactive;
        int requestedFoot = -1;
        int followupFoot = -1;
        uint32_t backwardFallStepsRemaining = 0;
        bool hasPrediction = false;
        bool actualComOutsideBuffer = false;
        bool needsStep = false;
        bool hasGroundPlane = false;
        bool hasImpact = false;
        bool aborted = false;
    };

    class FootstepPlanner
    {
    public:
        void Reset();
        void Update(const FootstepInput& input, const FootstepSettings& settings);
        void ReportRecoveryPath(size_t side, const physx::PxVec3& target, bool accepted);

    private:
        void Abort(FootstepReason reason);
        bool UpdateFallSteps(const FootstepInput& input, bool outsideBufferExpired);
        void UpdateMotion(FootstepCommand& command, const FootstepFootInput& foot, const FootstepInput& input);

    public:
        const FootstepSnapshot& GetSnapshot() const
        {
            return m_Snapshot;
        }

    public:
        static constexpr double OutsideBufferDuration = 0.3;
        static constexpr uint32_t MaximumBackwardFallSteps = 2;

    private:
        FootstepSnapshot m_Snapshot;
        uint64_t m_FallStepStart = 0;
        bool m_HasSample = false;
    };

    const char* FootstepPhaseLabel(FootstepPhase phase);
    const char* FootstepReasonLabel(FootstepReason reason);
}
