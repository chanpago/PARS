#pragma once

#include "FootSupportHull.h"
#include <cstdint>

namespace PARS
{
    // 발 이동에 관한 세팅 값
    struct FootstepSettings
    {
        float startPredictionTime = .6f;                                    // 발딛기가 필요한지 판단할 때, 몇 초뒤의 COM을 예측할지
        float landingPredictionTime = .6f;                                  // 착지 위치를 계산할 때, 몇 초 뒤의 COM을 기준으로 삼을지
        float stanceScale = .8f;                                            // Idle 기준 발 배치 오프셋을 적용하는 비율 
        float maximumStepLength = 2.0f;                                     // 지지발에서 착지 목표까지 허용하는 최대거리
        bool preventCrossing = false;                                       // 움직이는 발이 반대편으로 교차하는 것을 제한할지
        float crossingOffset = .1f;                                         // 교차 제한을 켰을 때 지지발 바깥 방향으로 확보할 최소거리

        // 착지 예측 시간과 별개인 발 이동 시간(s)과 궤적의 최소 최고 높이(m)다.
        float liftHeightScale = .35f;                                       // 발 이동 거리에 비례해 들어 올리는 높이를 정하는 비율 
        float stepDuration = .3f;                                           // 발 이동 목표 궤적의 진행시간
        float minimumLiftHeight = .08f;                                     // 발을 들어 올리는 최고 높이를 계산할 때 사용하는 최소값
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
        float fixedDeltaTime = 0;
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
        // progress는 시간 진행률이고, measuredProgress는 실제 발 위치의 진행률이다.
        float elapsedTime = 0, duration = 0;
        float measuredProgress = 0, progress = 0, remainingDistance = 0;
        float landingError = 0;
    };

    struct FootstepRoles
    {
        int supportFoot = -1;
        int movableFoot = -1;
        bool locked = false;

        bool IsValid() const
        {
            return supportFoot >= 0 && supportFoot < 2 && movableFoot == 1 - supportFoot;
        }
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

        // 제어에서 확정한 역할을 색상과 UI에서도 사용한다. 거리 후보와 진행 중인 역할을 섞지 않는다.
        FootstepRoles GetRoles() const;
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
