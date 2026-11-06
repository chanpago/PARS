#pragma once

#include "FootSupportHull.h"
#include <cstdint>

namespace PARS
{
    // 발 이동에 관한 세팅 값
    struct FootstepSettings
    {
    public:
        float startPredictionTime = .6f;   // 발딛기가 필요한지 판단할 때, 몇 초뒤의 COM을 예측할지
        float landingPredictionTime = .3f; // 착지 위치를 계산할 때, 몇 초 뒤의 COM을 기준으로 삼을지
        float stanceScale = .8f;           // Idle 기준 발 배치 오프셋을 적용하는 비율
        float maximumStepLength = 2.0f;    // 지지발에서 착지 목표까지 허용하는 최대거리
        float crossingOffset = .1f;        // 교차 제한을 켰을 때 지지발 바깥 방향으로 확보할 최소거리
        // 착지 예측 시간과 별개인 발 이동 시간(s)과 궤적의 최소 최고 높이(m)다.
        float liftHeightScale = .15f;   // 발 이동 거리에 비례해 들어 올리는 높이를 정하는 비율
        float stepDuration = .15f;       // 발 이동 목표 궤적의 진행시간
        float minimumLiftHeight = .08f; // 발을 들어 올리는 최고 높이를 계산할 때 사용하는 최소값
        bool preventCrossing = false;   // 움직이는 발이 반대편으로 교차하는 것을 제한할지
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
    public:
        physx::PxVec3 position{0};
        physx::PxVec3 normal{0};
        physx::PxVec3 stanceOffset{0};
        float calfLength = 0;
        bool available = false;
        bool hasContactSample = false;
        bool grounded = false;
    };

    struct FootstepInput
    {
    public:
        std::vector<physx::PxVec3> hull;
        uint64_t completedStep = 0;
        uint64_t revision = 0;
        physx::PxVec3 com{0};
        physx::PxVec3 velocity{0};
        physx::PxVec3 supportRight{1, 0, 0};
        std::array<FootstepFootInput, 2> feet;
        FootSupportZone zone = FootSupportZone::Invalid;
        float projectionHeight = 0;
        float fixedDeltaTime = 0;
        int blueFoot = -1;
        bool active = false;
        bool automaticSupport = false;
        bool idleAnimation = false;
        bool rootReleased = false;
        bool outsideBufferReleased = false;
        bool hasCom = false;
        bool hasVelocity = false;
        bool hasReference = false;
    };

    struct FootstepTarget
    {
    public:
        FootstepReason reason = FootstepReason::NotEvaluated;
        physx::PxVec3 raw{0};
        physx::PxVec3 position{0};
        physx::PxVec3 normal{0};
        float travelDistance = 0;
        float stanceDistance = 0;
        bool valid = false;
        bool lengthLimited = false;
        bool crossingLimited = false;
    };

    struct FootstepMotion
    {
    public:
        physx::PxVec3 position{0};
        float initialHeight = 0;
        float heightLimit = 0;
        float height = 0;
        // progress는 시간 진행률이고, measuredProgress는 실제 발 위치의 진행률이다.
        float elapsedTime = 0;
        float duration = 0;
        float measuredProgress = 0;
        float progress = 0;
        float remainingDistance = 0;
        float landingError = 0;
        // valid일 때만 다음 단계의 IK 목표로 사용할 수 있다. 착지 목표와 별개인 발바닥 월드 목표다.
        bool valid = false;
        bool hasLandingError = false;
    };

    struct FootstepRoles
    {
    public:
        bool IsValid() const
        {
            return supportFoot >= 0 && supportFoot < 2 && movableFoot == 1 - supportFoot;
        }

    public:
        int supportFoot = -1;
        int movableFoot = -1;
        bool locked = false;
    };

    // 발딛기 플래너의 현재 판단과 진행 상태
    struct FootstepSnapshot
    {
    public:
        bool IsInProgress() const
        {
            return phase == FootstepPhase::AwaitingTakeoff || phase == FootstepPhase::Airborne;
        }
        // 제어에서 확정한 역할을 색상과 UI에서도 사용한다. 거리 후보와 진행 중인 역할을 섞지 않는다.
        FootstepRoles GetRoles() const;

    public:
        uint64_t completedStep = 0;                       // 마지막으로 처리한 완료 물리 스텝 번호. 같은 결과를 중복 처리하지 않도록 사용
        uint64_t revision = 0;                            // 입력 접촉 이력의 버전 번호. 변경되면 발딛기 상태와 누적 횟수를 초기화
        uint64_t startedStep = 0;                         // 현재 또는 마지막 발딛기를 시작한 완료 물리 스텝 번호. 경과 시간 계산의 기준
        uint64_t startCount = 0;                          // 초기화 이후 발딛기를 시작한 누적 횟수
        uint64_t landingCount = 0;                        // 초기화 이후 실제 접지를 확인해 착지 처리한 누적 횟수
        uint64_t abortCount = 0;                          // 초기화 이후 발딛기가 중단 상태로 전환된 누적 횟수
        FootstepPhase phase = FootstepPhase::Idle;        // 발딛기 진행 단계. 대기, 이륙 대기, 공중, 착지, 중단 상태를 구분
        FootstepReason reason = FootstepReason::Inactive; // 현재 판단과 진행 상태의 사유. 목표 계산 실패나 발딛기 중단 원인 등을 기록
        int rawBlueFoot = -1;                             // 현재 입력에서 받은 움직일 발 후보. 0은 왼발, 1은 오른발, -1은 후보 없음
        int stepFoot = -1;                                // 발딛기 시작 시 확정한 움직일 발. 진행 중에는 유지하며 -1이면 미선택
        int supportFoot = -1;                             // 발딛기 시작 시 확정한 지지발. 움직일 발의 반대쪽이며 -1이면 미선택
        physx::PxVec3 predictedCom{0};                    // 발딛기 필요 여부를 판단할 미래 COM 위치. XZ는 현재 COM과 속도로 예측하고 Y는 지면 투영 높이
        physx::PxVec3 target{0};                          // 선택한 발의 최신 착지 목표 월드 위치(m). 진행 중 입력에 따라 갱신
        physx::PxVec3 targetNormal{0};                    // 최신 착지 목표 평면의 법선. 발바닥 목표 방향을 지면에 맞출 때 사용
        physx::PxVec3 startPosition{0};                   // 발딛기 시작 순간 움직일 발의 실제 발바닥 월드 위치(m). 이동 궤적의 시작점
        physx::PxVec3 initialTarget{0};                   // 발딛기 시작 순간의 최초 착지 목표 월드 위치(m). 이후 목표가 갱신돼도 유지
        float initialDistance = 0;                        // 시작 위치에서 최초 착지 목표까지의 수평 거리(m). 발 들기 높이와 골반 낮추기 계산에 사용
        FootstepMotion motion;                            // 현재 발 이동 궤적의 목표 위치, 경과 시간, 시간 및 실측 진행률과 착지 오차
        std::array<FootstepTarget, 2> candidates;         // 양발 각각의 착지 목표 후보. 위치, 유효성, 거리 및 교차 제한 여부와 실패 사유를 저장
        bool hasPrediction = false;                       // 미래 COM 위치를 유효하게 계산했는지. predictedCom을 사용할 때 확인
        bool needsStep = false;                           // 예측 COM이 현재 지지 영역 A 밖이어서 발딛기가 필요한지
        bool hasTarget = false;                           // 이번 완료 물리 스텝에서 선택한 발의 유효한 착지 목표를 출력했는지
        bool justStarted = false;                         // 이번 완료 물리 스텝에서 새 발딛기를 시작했는지
        bool justLanded = false;                          // 이번 완료 물리 스텝에서 계획 시간이 끝나고 실제 접지를 확인해 착지 처리했는지
    };

    // 물리 상태를 읽어 판단과 목표만 만든다. 바디 이동, IK와 힘 적용은 하지 않는다.
    class FootstepPlanner
    {
    public:
        void Reset();
        void Update(const FootstepInput& input, const FootstepSettings& settings);

    private:
        void Abort(FootstepReason reason);
        bool UpdateMotionProgress(const FootstepFootInput& foot);
        void UpdateMotionTarget(const FootstepFootInput& foot);

    public:
        const FootstepSnapshot& GetSnapshot() const
        {
            return m_Snapshot;
        }

    private:
        FootstepSnapshot m_Snapshot;
        bool m_HasSample = false;
    };

    const char* FootstepPhaseLabel(FootstepPhase phase);
    const char* FootstepReasonLabel(FootstepReason reason);
}
