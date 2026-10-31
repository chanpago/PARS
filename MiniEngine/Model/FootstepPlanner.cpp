#include "FootstepPlanner.h"

using namespace physx;
using namespace PARS;

namespace
{
    // 길이가 사실상 0인 이동에서 진행도를 나누지 않기 위한 수치 한계(m)다.
    constexpr float MinimumStepDistance = 1e-5f;

    bool ValidSettings(const FootstepSettings& settings)
    {
        return std::isfinite(settings.startPredictionTime) && settings.startPredictionTime > 0 &&
            std::isfinite(settings.landingPredictionTime) && settings.landingPredictionTime > 0 &&
            std::isfinite(settings.stanceScale) && settings.stanceScale >= 0 &&
            std::isfinite(settings.maximumStepLength) && settings.maximumStepLength > 0 &&
            std::isfinite(settings.crossingOffset) && settings.crossingOffset >= 0 &&
            settings.crossingOffset <= settings.maximumStepLength &&
            std::isfinite(settings.liftHeightScale) && settings.liftHeightScale > 0;
    }

    float HorizontalDistance(const PxVec3& a, const PxVec3& b)
    {
        const PxVec3 delta = a - b;
        return std::sqrt(delta.x * delta.x + delta.z * delta.z);
    }

    FootstepTarget MakeTarget(const FootstepInput& input, const FootstepSettings& settings, size_t side)
    {
        FootstepTarget result;
        result.reason = FootstepReason::InvalidTarget;
        const auto& foot = input.feet[side];
        const auto& stance = input.feet[1 - side];
        if (!input.hasReference)
        {
            result.reason = FootstepReason::NoReference;
            return result;
        }
        if (!foot.available || !stance.available || !foot.position.isFinite() ||
            !stance.position.isFinite() || !foot.stanceOffset.isFinite())
        {
            return result;
        }
        if (!stance.hasContactSample)
        {
            result.reason = FootstepReason::NoContactSample;
            return result;
        }
        if (!stance.grounded)
        {
            result.reason = FootstepReason::NoStanceContact;
            return result;
        }
        PxVec3 normal = stance.normal;
        if (!normal.isFinite() || normal.normalize() < 1e-6f || normal.y < .70710678f)
        {
            return result;
        }

        // 지지발 높이의 수평면을 거친 뒤 접촉 법선 평면에 투영한다.
        // COM의 높이를 발 목표 높이로 쓰거나 후보 위치에서 새 레이를 쏘지 않는다.
        result.raw = PxVec3(input.com.x + input.velocity.x * settings.landingPredictionTime,
            stance.position.y, input.com.z + input.velocity.z * settings.landingPredictionTime);
        result.raw += PxVec3(foot.stanceOffset.x, 0, foot.stanceOffset.z) * settings.stanceScale;
        PxVec3 delta = result.raw - stance.position;
        delta -= normal * delta.dot(normal);
        const float distance = delta.magnitude();
        if (!result.raw.isFinite() || !std::isfinite(distance))
        {
            return result;
        }
        if (distance > settings.maximumStepLength)
        {
            delta *= settings.maximumStepLength / distance;
            result.lengthLimited = true;
        }
        if (settings.preventCrossing)
        {
            PxVec3 outward = input.supportRight;
            if (!outward.isFinite() || !stance.stanceOffset.isFinite())
            {
                return result;
            }
            const float sideDirection = (foot.stanceOffset - stance.stanceOffset).dot(outward);
            if (std::abs(sideDirection) < 1e-6f)
            {
                return result;
            }
            outward *= sideDirection > 0 ? 1.0f : -1.0f;
            outward -= normal * outward.dot(normal);
            if (outward.normalize() < 1e-6f)
            {
                return result;
            }
            const float lateral = delta.dot(outward);
            if (lateral < settings.crossingOffset)
            {
                PxVec3 tangent = delta - outward * lateral;
                const float limit = std::sqrt((std::max)(0.0f,
                    settings.maximumStepLength * settings.maximumStepLength - settings.crossingOffset * settings.crossingOffset));
                const float length = tangent.magnitude();
                if (length > limit && length > 0)
                {
                    tangent *= limit / length;
                    result.lengthLimited = true;
                }
                // 교차 경계와 2m 반경을 동시에 지킨다. 마지막 반경 제한으로 경계를 다시 넘기지 않는다.
                delta = outward * settings.crossingOffset + tangent;
                result.crossingLimited = true;
            }
        }
        result.position = stance.position + delta;
        result.normal = normal;
        result.travelDistance = HorizontalDistance(foot.position, result.position);
        result.stanceDistance = delta.magnitude();
        result.valid = result.position.isFinite() && std::isfinite(result.travelDistance);
        if (result.valid)
        {
            result.reason = FootstepReason::ValidTarget;
        }
        return result;
    }
}

void FootstepPlanner::Reset()
{
    m_Snapshot = {};
    m_HasSample = false;
}

void FootstepPlanner::Abort(FootstepReason reason)
{
    if (m_Snapshot.phase != FootstepPhase::Aborted)
    {
        ++m_Snapshot.abortCount;
    }
    m_Snapshot.phase = FootstepPhase::Aborted;
    m_Snapshot.reason = reason;
    m_Snapshot.hasTarget = false;
    m_Snapshot.motion.valid = false;
    m_Snapshot.candidates = {};
    for (auto& candidate : m_Snapshot.candidates)
    {
        candidate.reason = reason;
    }
}

bool FootstepPlanner::UpdateMotionProgress(const FootstepFootInput& foot)
{
    auto& state = m_Snapshot;
    auto& motion = state.motion;
    if (!foot.position.isFinite() || !state.target.isFinite())
    {
        return false;
    }
    const double dx = double(state.target.x) - state.startPosition.x;
    const double dz = double(state.target.z) - state.startPosition.z;
    const double lengthSquared = dx * dx + dz * dz;
    motion.remainingDistance = HorizontalDistance(foot.position, state.target);
    if (!std::isfinite(motion.remainingDistance))
    {
        return false;
    }
    if (lengthSquared > double(MinimumStepDistance) * MinimumStepDistance)
    {
        // 시작점은 유지하되 최신 착지점 방향으로 실제 발의 이동을 투영한다.
        // 현재 목표까지의 거리로 정규화하므로 짧아지거나 방향이 바뀐 목표에도 도착을 표현한다.
        const double x = double(foot.position.x) - state.startPosition.x;
        const double z = double(foot.position.z) - state.startPosition.z;
        const double fraction = (x * dx + z * dz) / lengthSquared;
        motion.measuredProgress = float((std::max)(0.0, (std::min)(1.0, fraction)));
    }
    else
    {
        // 갱신된 목표가 시작점으로 돌아온 경우에도 0으로 나누지 않는다.
        motion.measuredProgress = motion.remainingDistance <= MinimumStepDistance ? 1.0f : 0.0f;
    }
    if (state.phase == FootstepPhase::Airborne)
    {
        // 이륙 전 미끄러짐만으로 높이가 사라지지 않게 한다.
        // 후퇴하거나 목표가 멀어져도 이미 진행한 내려놓기를 되돌리지 않는다.
        motion.progress = (std::max)(motion.progress, motion.measuredProgress);
    }
    return true;
}

void FootstepPlanner::UpdateMotionTarget(const FootstepFootInput& foot)
{
    auto& state = m_Snapshot;
    auto& motion = state.motion;
    if (!state.hasTarget || !state.IsInProgress() || !UpdateMotionProgress(foot))
    {
        return;
    }
    motion.height = motion.initialHeight * (1.0f - motion.progress);
    // XZ는 최신 착지 후보, Y는 그 지면 높이와 추가 높이를 합한다. 시간으로 진행시키지 않는다.
    motion.position = state.target + PxVec3(0, motion.height, 0);
    motion.valid = motion.position.isFinite() && std::isfinite(motion.height);
}

void FootstepPlanner::Update(const FootstepInput& input, const FootstepSettings& settings)
{
    if (m_HasSample && input.revision != m_Snapshot.revision)
    {
        Reset();
    }
    // 같은 완료 스텝의 재조회는 시작/착지/중단을 다시 집계하지 않는다.
    if (m_HasSample && input.completedStep == m_Snapshot.completedStep)
    {
        return;
    }

    // 이번 입력 번호를 저장하고, 이번 계산에서 새로 만들 결과를 비움 
    m_HasSample = true;
    FootstepSnapshot& state = m_Snapshot;
    state.completedStep = input.completedStep;
    state.revision = input.revision;
    state.rawBlueFoot = input.blueFoot;
    state.justStarted = state.justLanded = false;
    state.hasPrediction = state.needsStep = state.hasTarget = false;
    state.motion.valid = false;
    state.motion.position = PxVec3(0);
    state.motion.height = 0;
    state.candidates = {};

    // 실제 COM의 B 이탈 및 기존 해제 래치를 착지 처리보다 먼저 검사한다.
    // 액티브 래그돌·자동 지지 상태에서 현재 COM이 B 밖이거나, 이미 B 이탈로 지지가 해제됐다면 중단
    if (input.active && input.automaticSupport && (input.zone == FootSupportZone::Outside || input.outsideBufferReleased))
    {
        Abort(FootstepReason::OutsideBuffer);
        return;
    }


    if (state.phase == FootstepPhase::Aborted)
    {
        return;
    }

    if (input.active && input.automaticSupport && input.rootReleased)
    {
        Abort(FootstepReason::RootSupportReleased);
        return;
    }

    // 발딛기 기능을 사용할 실행 조건인지 확인
    if (!input.active || !input.automaticSupport || !input.idleAnimation)
    {
        state.phase = FootstepPhase::Idle;
        state.stepFoot = state.supportFoot = -1;
        state.motion = {};
        state.reason = !input.active ? FootstepReason::Inactive : !input.automaticSupport
            ? FootstepReason::AutomaticSupportRequired : FootstepReason::IdleRequired;
        return;
    }

    // 이미 발딛기 중이면 이륙, 착지를 먼저 확인
    if (state.IsInProgress())
    {
        // 이미 선택한 발의 상태를읽음 
        const FootstepFootInput& foot = input.feet[state.stepFoot];

        if (!foot.available || !foot.hasContactSample)
        {
            state.reason = FootstepReason::NoContactSample;
            return;
        }

        // 목표 계산 입력이 잠시 사라져도 실제 이륙/재접촉 이력은 놓치지 않는다.
        if (!foot.grounded)
        {
            state.phase = FootstepPhase::Airborne;
        }

        // 발이 바닥에 닿아 있지않으면 공중 상태로 바꿈
        else if (state.phase == FootstepPhase::Airborne)
        {
            UpdateMotionProgress(foot);
            state.phase = FootstepPhase::Landed;
            state.reason = FootstepReason::Landed;
            state.justLanded = true;
            // 재접촉만으로 목표 도달까지 성공했다고 판단하지 않도록 마지막 착지 목표와의 오차를 남긴다.
            state.motion.landingError = HorizontalDistance(foot.position, state.target);
            state.motion.hasLandingError = std::isfinite(state.motion.landingError);
            state.motion.remainingDistance = state.motion.hasLandingError ? state.motion.landingError : 0;
            ++state.landingCount;
            return;
        }
    }

    // 예측 계산에 필요한 설정과 입력이 유효한지 확인
    if (!ValidSettings(settings))
    {
        state.reason = FootstepReason::InvalidSettings;
        return;
    }
    if (!input.hasCom || !input.hasVelocity || !input.com.isFinite() || !input.velocity.isFinite())
    {
        state.reason = FootstepReason::InvalidCom;
        return;
    }
    if (input.zone == FootSupportZone::Invalid || input.hull.size() < 3 || !std::isfinite(input.projectionHeight))
    {
        state.reason = FootstepReason::NoSupportArea;
        return;
    }

    // 미래 COM을 계산하고 발딛기 필요 여부를 판단
    // 예측 위치 = 현재 COM 위치 + 현재 COM 속도 × 예측 시간
    state.predictedCom = PxVec3(input.com.x + input.velocity.x * settings.startPredictionTime,
        input.projectionHeight, input.com.z + input.velocity.z * settings.startPredictionTime);
    state.hasPrediction = state.predictedCom.isFinite();

    if (!state.hasPrediction)
    {
        state.reason = FootstepReason::InvalidCom;
        return;
    }

    // 예측점이 A 밖이면 needsStep = true, 안이면 false
    state.needsStep = !IsInsideFootSupportHull(input.hull, state.predictedCom);

    // 왼발·오른발 각각의 착지 후보를 계산
    for (size_t side = 0; side < state.candidates.size(); ++side)
    {
        // MakeTarget()은 반대 발이 실제로 지지 중인지 확인하고, COM 위치·속도와 Idle 발 간격을 이용해 후보를 만듦
        // 이후 지지발의 위치·접촉 법선으로 만든 평면에 맞추고 보폭 제한 등을 적용
        // 발딛기 필요 판정은 기본 2초 예측, 착지 후보 위치는 기본 0.3초 예측을 사용하는 구조
        state.candidates[side] = MakeTarget(input, settings, side);
    }

    // 진행 중인 발딛기가 없을 때만 새 발딛기의 시작 조건을 검사
    if (!state.IsInProgress())
    {
        state.phase = FootstepPhase::Idle;
        state.stepFoot = state.supportFoot = -1;
        if (!state.needsStep)
        {
            state.reason = FootstepReason::PredictedInside;
            return;
        }
        if (input.blueFoot < 0 || input.blueFoot > 1)
        {
            state.reason = FootstepReason::NoBlueFoot;
            return;
        }
        const FootstepTarget& candidate = state.candidates[input.blueFoot];
        if (!candidate.valid)
        {
            state.reason = candidate.reason;
            return;
        }
        const FootstepFootInput& foot = input.feet[input.blueFoot];
        if (!foot.hasContactSample)
        {
            state.reason = FootstepReason::NoContactSample;
            return;
        }
        if (!std::isfinite(foot.calfLength) || foot.calfLength <= 0)
        {
            state.reason = FootstepReason::NoCalfLength;
            return;
        }
        if (candidate.travelDistance <= MinimumStepDistance)
        {
            state.reason = FootstepReason::NoStepDistance;
            return;
        }

        // 검사를 통과하면 움직일 발과 시작 정보를 확정
        state.stepFoot = input.blueFoot;
        state.supportFoot = 1 - input.blueFoot;
        state.phase = foot.grounded ? FootstepPhase::AwaitingTakeoff : FootstepPhase::Airborne;
        state.startPosition = foot.position;
        state.initialTarget = candidate.position;
        state.initialDistance = candidate.travelDistance;

        // 발을 들어 올릴 초기 목표 높이를 계산하고 시작을 기록
        state.motion = {};
        state.motion.heightLimit = foot.calfLength;
        state.motion.initialHeight = (std::min)(state.initialDistance * settings.liftHeightScale, foot.calfLength);
        state.startedStep = input.completedStep;
        state.justStarted = true;
        ++state.startCount;
    }

    // 선택한 발의 최신 착지 목표를 갱신
    const FootstepTarget& candidate = state.candidates[state.stepFoot];
    if (!candidate.valid)
    {
        // 지면 입력을 잃어도 발을 다시 고르지 않는다. 유효해질 때까지 목표 출력을 보류한다.
        state.reason = candidate.reason;
        return;
    }
    state.target = candidate.position;
    state.targetNormal = candidate.normal;
    state.hasTarget = true;
    state.reason = state.phase == FootstepPhase::Airborne ? FootstepReason::InFlight : FootstepReason::WaitingTakeoff;

    // 현재 발 이동 목표를 만듦
    // 실제 발이 시작점에서 최신 착지 목표 방향으로 얼마나 이동했는지 계산해서 진행률을 구함
    // 공중 상태에서는 진행률이 이전 값보다 커질 때만 반영하고, 그 값으로 추가 높이를 줄임
    UpdateMotionTarget(input.feet[state.stepFoot]);
}

const char* PARS::FootstepPhaseLabel(FootstepPhase phase)
{
    switch (phase)
    {
    case FootstepPhase::Idle:
        return u8"대기";
    case FootstepPhase::AwaitingTakeoff:
        return u8"시작 / 이륙 대기";
    case FootstepPhase::Airborne:
        return u8"이륙 후 착지 대기";
    case FootstepPhase::Landed:
        return u8"착지 확인";
    case FootstepPhase::Aborted:
        return u8"중단 / 초기화 대기";
    default:
        return u8"알 수 없음";
    }
}

const char* PARS::FootstepReasonLabel(FootstepReason reason)
{
    switch (reason)
    {
    case FootstepReason::NotEvaluated:
        return u8"계산 대기";
    case FootstepReason::ValidTarget:
        return u8"유효한 착지 후보";
    case FootstepReason::Inactive:
        return u8"액티브 래그돌이 아님";
    case FootstepReason::AutomaticSupportRequired:
        return u8"COM 자동 골반 지지 모드에서 판단";
    case FootstepReason::IdleRequired:
        return u8"현재 단계는 혼합 중이 아닌 Idle만 지원";
    case FootstepReason::InvalidSettings:
        return u8"설정값 오류";
    case FootstepReason::InvalidCom:
        return u8"COM 위치 또는 실제 속도 없음";
    case FootstepReason::NoSupportArea:
        return u8"유효한 A/B 영역 없음";
    case FootstepReason::NoReference:
        return u8"기준 Idle 발 간격 없음";
    case FootstepReason::NoCalfLength:
        return u8"들기 높이 상한에 사용할 기준 종아리 길이 없음";
    case FootstepReason::NoStepDistance:
        return u8"발의 수평 이동 거리가 사실상 0";
    case FootstepReason::NoContactSample:
        return u8"실제 접촉 샘플 대기";
    case FootstepReason::PredictedInside:
        return u8"예측 COM이 A 안";
    case FootstepReason::NoBlueFoot:
        return u8"시작에 사용할 파란 발 후보 없음";
    case FootstepReason::NoStanceContact:
        return u8"반대 발의 실제 지면 접촉 없음";
    case FootstepReason::InvalidTarget:
        return u8"착지 평면 또는 목표 계산 불가";
    case FootstepReason::WaitingTakeoff:
        return u8"저장한 발의 이륙 대기";
    case FootstepReason::InFlight:
        return u8"저장한 발의 실제 재접촉 대기";
    case FootstepReason::Landed:
        return u8"이륙 후 재접촉 확인 / 다음 스텝에 재평가";
    case FootstepReason::OutsideBuffer:
        return u8"실제 COM의 B 이탈 / 재진입해도 재개하지 않음";
    case FootstepReason::RootSupportReleased:
        return u8"기존 골반 지지 해제 상태";
    default:
        return u8"알 수 없음";
    }
}
