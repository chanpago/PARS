#pragma once

#include "FootSupportHull.h"
#include <cstdint>

namespace PARS
{
    struct FootstepSettings
    {
        float startPredictionTime = .3f, landingPredictionTime = .3f;
        float stanceScale = .8f, maximumStepLength = 2.0f;
        bool preventCrossing = false;
        float crossingOffset = .1f;
        float liftHeightScale = .25f;
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
        NoBlueFoot,
        NoStanceContact,
        InvalidTarget,
        WaitingTakeoff,
        InFlight,
        Landed,
        OutsideBuffer,
        RootSupportReleased
    };

    struct FootstepFootInput
    {
        bool available = false, hasContactSample = false, grounded = false;
        physx::PxVec3 position{0}, normal{0}, stanceOffset{0};
        float calfLength = 0;
    };

    struct FootstepInput
    {
        bool active = false, automaticSupport = false, idleAnimation = false;
        bool rootReleased = false, outsideBufferReleased = false;
        bool hasCom = false, hasVelocity = false, hasReference = false;
        physx::PxVec3 com{0}, velocity{0}, supportRight{1, 0, 0};
        std::array<FootstepFootInput, 2> feet;
        std::vector<physx::PxVec3> hull;
        FootSupportZone zone = FootSupportZone::Invalid;
        float projectionHeight = 0;
        int blueFoot = -1;
        uint64_t completedStep = 0, revision = 0;
    };

    struct FootstepTarget
    {
        bool valid = false, lengthLimited = false, crossingLimited = false;
        FootstepReason reason = FootstepReason::NotEvaluated;
        physx::PxVec3 raw{0}, position{0}, normal{0};
        float travelDistance = 0, stanceDistance = 0;
    };

    struct FootstepMotion
    {
        // valid일 때만 다음 단계의 IK 목표로 사용할 수 있다. 착지 목표와 별개인 발바닥 월드 목표다.
        bool valid = false, hasLandingError = false;
        physx::PxVec3 position{0};
        float initialHeight = 0, heightLimit = 0, height = 0;
        float measuredProgress = 0, progress = 0, remainingDistance = 0;
        float landingError = 0;
    };

    struct FootstepSnapshot
    {
        FootstepPhase phase = FootstepPhase::Idle;
        FootstepReason reason = FootstepReason::Inactive;
        bool hasPrediction = false, needsStep = false, hasTarget = false;
        bool justStarted = false, justLanded = false;
        int rawBlueFoot = -1, stepFoot = -1, supportFoot = -1;
        physx::PxVec3 predictedCom{0}, target{0}, targetNormal{0};
        physx::PxVec3 startPosition{0}, initialTarget{0};
        float initialDistance = 0;
        FootstepMotion motion;
        std::array<FootstepTarget, 2> candidates;
        uint64_t completedStep = 0, revision = 0, startedStep = 0;
        uint64_t startCount = 0, landingCount = 0, abortCount = 0;

        bool IsInProgress() const
        {
            return phase == FootstepPhase::AwaitingTakeoff || phase == FootstepPhase::Airborne;
        }
    };

    // 물리 상태를 읽어 판단과 목표만 만든다. 바디 이동, IK와 힘 적용은 하지 않는다.
    class FootstepPlanner
    {
    public:
        void Reset();
        void Update(const FootstepInput& input, const FootstepSettings& settings);
        const FootstepSnapshot& GetSnapshot() const
        {
            return m_Snapshot;
        }

    private:
        FootstepSnapshot m_Snapshot;
        bool m_HasSample = false;
        void Abort(FootstepReason reason);
        bool UpdateMotionProgress(const FootstepFootInput& foot);
        void UpdateMotionTarget(const FootstepFootInput& foot);
    };

    const char* FootstepPhaseLabel(FootstepPhase phase);
    const char* FootstepReasonLabel(FootstepReason reason);
}
