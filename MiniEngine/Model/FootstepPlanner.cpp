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
            std::isfinite(settings.minimumStanceDistance) && settings.minimumStanceDistance >= 0 &&
            std::isfinite(settings.liftHeightScale) && settings.liftHeightScale > 0 &&
            std::isfinite(settings.stepDuration) && settings.stepDuration > 0 &&
            std::isfinite(settings.minimumLiftHeight) && settings.minimumLiftHeight > 0 &&
            std::isfinite(settings.landingLoweringSpeed) && settings.landingLoweringSpeed >= 0 &&
            std::isfinite(settings.maximumLandingLowering) && settings.maximumLandingLowering >= 0 &&
            std::isfinite(settings.supportLoadRatio) && settings.supportLoadRatio > 0 && settings.supportLoadRatio <= 1 &&
            std::isfinite(settings.supportConfirmationTime) && settings.supportConfirmationTime > 0 &&
            std::isfinite(settings.supportSwitchDistance) && settings.supportSwitchDistance >= 0 &&
            std::isfinite(settings.maximumJointSpeed) && settings.maximumJointSpeed > 0 &&
            std::isfinite(settings.maximumJointAcceleration) && settings.maximumJointAcceleration > 0 &&
            std::isfinite(settings.maximumPelvisYaw) && settings.maximumPelvisYaw >= 0 && settings.maximumPelvisYaw <= PxPi &&
            std::isfinite(settings.maximumPelvisTilt) && settings.maximumPelvisTilt >= 0 && settings.maximumPelvisTilt <= PxHalfPi &&
            std::isfinite(settings.maximumPelvisAngularSpeed) && settings.maximumPelvisAngularSpeed > 0 &&
            std::isfinite(settings.maximumPelvisAngularAcceleration) && settings.maximumPelvisAngularAcceleration > 0 &&
            std::isfinite(settings.reachPositionTolerance) && settings.reachPositionTolerance > 0 &&
            std::isfinite(settings.reachTiltTolerance) && settings.reachTiltTolerance > 0 &&
            settings.candidateSamples > 0 && settings.candidateSamples <= 128;
    }

    float HorizontalDistance(const PxVec3& a, const PxVec3& b)
    {
        const PxVec3 delta = a - b;
        return std::sqrt(delta.x * delta.x + delta.z * delta.z);
    }

    PxVec3 SelectionCom(const FootstepInput& input, const FootstepSettings& settings)
    {
        return PxVec3(input.com.x + input.velocity.x * settings.landingPredictionTime * .5f,
            input.projectionHeight, input.com.z + input.velocity.z * settings.landingPredictionTime * .5f);
    }

    int SelectSupportFoot(const FootstepInput& input, const FootstepSettings& settings, int previous)
    {
        const PxVec3 center = SelectionCom(input, settings);
        int selected = -1;
        float closest = PX_MAX_F32;
        std::array<float, 2> distances{PX_MAX_F32, PX_MAX_F32};
        for (size_t side = 0; side < input.feet.size(); ++side)
        {
            const FootstepFootInput& foot = input.feet[side];
            if (!foot.available || !foot.hasContactSample || !foot.grounded || !foot.position.isFinite())
            {
                continue;
            }
            distances[side] = HorizontalDistance(center, foot.position);
            if (distances[side] < closest)
            {
                closest = distances[side];
                selected = static_cast<int>(side);
            }
        }
        if (previous >= 0 && previous < 2 && distances[previous] < PX_MAX_F32 &&
            distances[previous] <= closest + settings.supportSwitchDistance)
        {
            return previous;
        }
        return selected;
    }

    FootstepTarget MakeTarget(const FootstepInput& input, const FootstepSettings& settings, size_t side)
    {
        FootstepTarget result;
        result.reason = FootstepReason::InvalidTarget;
        const FootstepFootInput& foot = input.feet[side];
        const FootstepFootInput& stance = input.feet[1 - side];
        if (!foot.available || !stance.available || !foot.position.isFinite() || !stance.position.isFinite() ||
            !foot.rotation.isSane())
        {
            return result;
        }
        if (!foot.hasContactSample || !stance.hasContactSample)
        {
            result.reason = FootstepReason::NoContactSample;
            return result;
        }
        if (!stance.grounded)
        {
            result.reason = FootstepReason::NoStanceContact;
            return result;
        }
        if (!std::isfinite(foot.calfLength) || foot.calfLength <= 0)
        {
            result.reason = FootstepReason::NoCalfLength;
            return result;
        }
        PxVec3 normal = stance.normal;
        if (!stance.contactPosition.isFinite() || !normal.isFinite() ||
            normal.normalize() < 1e-6f || normal.y < .70710678f)
        {
            return result;
        }
        // 예측 COM의 XZ를 보존하고 지지발의 실제 접촉 평면까지 수직 투영한다.
        result.raw = PxVec3(input.com.x + input.velocity.x * settings.landingPredictionTime,
            stance.contactPosition.y, input.com.z + input.velocity.z * settings.landingPredictionTime);
        result.raw.y -= normal.dot(result.raw - stance.contactPosition) / normal.y;
        result.normal = normal;
        PxVec3 start = foot.position;
        start.y -= normal.dot(start - stance.contactPosition) / normal.y;
        const PxVec3 path = result.raw - start;
        if (!result.raw.isFinite() || !path.isFinite())
        {
            return result;
        }
        if (foot.grounded && HorizontalDistance(start, result.raw) <= MinimumStepDistance)
        {
            result.reason = FootstepReason::NoStepDistance;
            return result;
        }
        result.minimumDistanceRequired = foot.grounded && settings.minimumStanceDistance > 0 &&
            HorizontalDistance(foot.position, stance.contactPosition) < settings.minimumStanceDistance;
        float minimumFraction = 0;
        if (result.minimumDistanceRequired)
        {
            // 출발점이 최소거리 원 안에 있으므로 선분이 원 밖으로 나가는 첫 위치를 구한다.
            const PxVec3 offset = start - stance.contactPosition;
            const float a = path.x * path.x + path.z * path.z;
            const float b = 2.0f * (offset.x * path.x + offset.z * path.z);
            const float c = offset.x * offset.x + offset.z * offset.z -
                settings.minimumStanceDistance * settings.minimumStanceDistance;
            const float discriminant = b * b - 4.0f * a * c;
            if (a <= MinimumStepDistance * MinimumStepDistance || discriminant < 0)
            {
                result.reason = FootstepReason::MinimumDistanceConflict;
                return result;
            }
            minimumFraction = (std::max)(0.0f, (-b + std::sqrt(discriminant)) / (2.0f * a));
            if (!std::isfinite(minimumFraction) || minimumFraction > 1.0f)
            {
                result.reason = FootstepReason::MinimumDistanceConflict;
                return result;
            }
        }
        if (!input.evaluateReachability)
        {
            result.reason = FootstepReason::NoReachability;
            return result;
        }
        float rejectedFraction = 1.0f;
        float acceptedFraction = -1.0f;
        FootstepReachability shortestRejected;
        bool timeRejected = false;
        const std::function<bool(float)> evaluate = [&](float fraction) -> bool
        {
            const PxVec3 position = start + path * fraction;
            const float distance = HorizontalDistance(foot.position, position);
            if (foot.grounded && distance <= MinimumStepDistance)
            {
                return false;
            }
            const float liftHeight = (std::min)(foot.calfLength,
                (std::max)(settings.minimumLiftHeight, distance * settings.liftHeightScale));
            const FootstepReachability reach = input.evaluateReachability(
                side, position, normal, settings.stepDuration, liftHeight);
            if (reach.timeLimited || reach.requiredDuration > settings.stepDuration + 1e-6f)
            {
                timeRejected = true;
                // 시작하지 못한 경우에도 가장 짧은 시간 추정치를 남겨 중단 원인을 확인한다.
                if (std::isfinite(reach.requiredDuration) && reach.requiredDuration > 0 &&
                    (shortestRejected.requiredDuration <= 0 || reach.requiredDuration < shortestRejected.requiredDuration))
                {
                    shortestRejected = reach;
                }
            }
            if (!reach.reachable || !reach.pelvisStart.isSane() || !reach.pelvisTarget.isSane() ||
                !std::isfinite(reach.requiredDuration) || reach.requiredDuration < 0)
            {
                return false;
            }
            if (reach.requiredDuration > settings.stepDuration + 1e-6f)
            {
                timeRejected = true;
                return false;
            }
            result.position = position;
            result.travelDistance = distance;
            result.stanceDistance = HorizontalDistance(position, stance.contactPosition);
            result.reachability = reach;
            return true;
        };
        // 선분을 목표 쪽부터 검사한다. 측면 오프셋과 고정 보폭 상한은 사용하지 않는다.
        for (uint32_t sample = 0; sample <= settings.candidateSamples; ++sample)
        {
            const float fraction = 1.0f - (1.0f - minimumFraction) *
                static_cast<float>(sample) / static_cast<float>(settings.candidateSamples);
            if (evaluate(fraction))
            {
                acceptedFraction = fraction;
                break;
            }
            rejectedFraction = fraction;
        }
        if (acceptedFraction < 0)
        {
            result.reason = timeRejected ? FootstepReason::InsufficientTime : FootstepReason::Unreachable;
            result.reachability = shortestRejected;
            return result;
        }
        if (acceptedFraction < 1.0f)
        {
            const float segmentLength = HorizontalDistance(start, result.raw);
            for (uint32_t refinement = 0; refinement < 6; ++refinement)
            {
                // 이미 허용 위치 오차의 절반 이내인 경계는 더 잘게 탐색하지 않는다.
                if ((rejectedFraction - acceptedFraction) * segmentLength <= settings.reachPositionTolerance * .5f)
                {
                    break;
                }
                const float fraction = (acceptedFraction + rejectedFraction) * .5f;
                if (evaluate(fraction))
                {
                    acceptedFraction = fraction;
                }
                else
                {
                    rejectedFraction = fraction;
                }
            }
        }
        result.reachLimited = acceptedFraction < 1.0f - 1e-5f;
        result.valid = true;
        result.reason = FootstepReason::ValidTarget;
        return result;
    }
}

FootstepRoles FootstepSnapshot::GetRoles() const
{
    if (IsInProgress() || justLanded)
    {
        // 목표 출력이나 지면 정보가 잠시 없어져도 착지를 처리한 틱까지 선택한 역할을 유지한다.
        const FootstepRoles roles{supportFoot, stepFoot, true};
        return roles.IsValid() ? roles : FootstepRoles{};
    }
    if ((phase == FootstepPhase::Landed || phase == FootstepPhase::Idle) && transfer.active)
    {
        const FootstepRoles roles{transfer.supportFoot, 1 - transfer.supportFoot, !transfer.ready};
        return roles.IsValid() ? roles : FootstepRoles{};
    }
    if (phase == FootstepPhase::Idle && hasPrediction && rawBlueFoot >= 0 && rawBlueFoot < 2 &&
        supportFoot == 1 - rawBlueFoot)
    {
        // 접지와 COM으로 고른 역할은 착지 후보의 성공 여부와 별개로 표시한다.
        return FootstepRoles{1 - rawBlueFoot, rawBlueFoot, false};
    }
    // 중단·비활성·판단 정보 없음 상태에서 이전 역할을 표시하지 않는다.
    return FootstepRoles{};
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
    m_Snapshot.transfer = {};
    for (PARS::FootstepTarget& candidate : m_Snapshot.candidates)
    {
        candidate.reason = reason;
    }
}

bool FootstepPlanner::UpdateMotionProgress(const FootstepFootInput& foot)
{
    FootstepSnapshot& state = m_Snapshot;
    FootstepMotion& motion = state.motion;
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
        // 출발 시 확정한 선분에 실제 발 위치를 투영한다.
        const double x = double(foot.position.x) - state.startPosition.x;
        const double z = double(foot.position.z) - state.startPosition.z;
        const double fraction = (x * dx + z * dz) / lengthSquared;
        motion.measuredProgress = float((std::max)(0.0, (std::min)(1.0, fraction)));
    }
    else
    {
        // 수평 거리가 사실상 0일 때 나눗셈을 생략한다.
        motion.measuredProgress = motion.remainingDistance <= MinimumStepDistance ? 1.0f : 0.0f;
    }
    // 실제 위치는 진단에만 사용한다. 목표의 이동 시간은 발의 미끄러짐과 별개로 진행한다.
    return true;
}

// 발바닥 목표를 출발점에서 착지점까지의 궤도 코드
void FootstepPlanner::UpdateMotionTarget(const FootstepInput& input)
{
    FootstepSnapshot& state = m_Snapshot;
    FootstepMotion& motion = state.motion;
    const FootstepFootInput& foot = input.feet[state.stepFoot];
    if (!state.hasTarget || !state.IsInProgress() || !UpdateMotionProgress(foot))
    {
        return;
    }

    // 다음 simulate()가 끝나는 시점의 목표를 준다. 완료된 샘플 시각을 그대로 쓰면 한 틱 늦게 도착한다.
    motion.commandProgress = (std::min)((motion.elapsedTime + input.fixedDeltaTime) / motion.duration, 1.0f);
    const float t = motion.commandProgress;

    // 일반적으로 시작지점 S에서 끝점E 로가는 보간은 위치 = S + (E - S) * t. 하지만 이렇게 하면 일정하게 움직이므로 시작할때 천천히 출발하고 끝날때 천천히 멈추도록 보간 비율 f(t)를 만든다
    // blend는 출발과 도착에서 속도가 0이 되도록 만든 보간 비율
    const float blend = t * t * (3.0f - 2.0f * t);

    // 코사인 한 주기를 높이 0 → 최고 높이 → 0으로 바꾸며, 출발과 도착의 수직 변화율은 0이다.
    // 발이 이미 공중에 있었다면 실제 시작 높이에서 착지면 높이로 함께 보간한다.
    motion.height = motion.initialHeight * .5f * (1.0f - std::cos(PxTwoPi * t));

    // 이번 프레임의 목표를 만듦
    motion.position = state.startPosition + (state.target - state.startPosition) * blend + PxVec3(0, motion.height, 0);

    // 착지 단계인데 공중에 있따면 얼마나 시간을 초과했나 계산
    if (motion.landingTargetLocked && !foot.grounded)
    {
        const uint64_t elapsedSteps = input.completedStep >= state.startedStep ? input.completedStep - state.startedStep : 0;
        const double landingTime = (std::max)(0.0, double(elapsedSteps) * input.fixedDeltaTime - double(motion.duration));
        const float requestedLowering = static_cast<float>((std::min)(landingTime * motion.landingLoweringSpeed, double(motion.maximumLandingLowering)));

        // 목표 출력이 잠시 멈췄어도 남은 깊이로 뛰지 않고 매번 정해진 하강 속도를 지킨다.
        const float maximumChange = motion.landingLoweringSpeed * input.fixedDeltaTime;
        motion.landingLowering = (std::min)(requestedLowering, motion.landingLowering + maximumChange);
        motion.position.y -= motion.landingLowering;
    }
    motion.valid = motion.position.isFinite() && std::isfinite(motion.height);
}

void FootstepPlanner::UpdateSupportTransfer(const FootstepInput& input, const FootstepSettings& settings)
{
    FootstepSupportTransfer& transfer = m_Snapshot.transfer;
    // 현재 COM과 착지 예측 COM의 중점으로 선택하고 실제 하중을 확인한다.
    transfer.predictedCom = PxVec3(input.com.x + input.velocity.x * settings.landingPredictionTime * .5f,
        input.projectionHeight, input.com.z + input.velocity.z * settings.landingPredictionTime * .5f);
    transfer.loadRatios = {};
    transfer.ready = false;
    const int previousCandidate = transfer.candidateFoot;
    transfer.candidateFoot = -1;
    float closestDistance = PX_MAX_F32;
    std::array<float, 2> distances{PX_MAX_F32, PX_MAX_F32};
    for (size_t side = 0; side < input.feet.size(); ++side)
    {
        const FootstepFootInput& foot = input.feet[side];
        if (foot.available && foot.hasContactSample && foot.grounded && foot.position.isFinite() && transfer.predictedCom.isFinite())
        {
            distances[side] = HorizontalDistance(transfer.predictedCom, foot.position);
            if (distances[side] < closestDistance)
            {
                closestDistance = distances[side];
                transfer.candidateFoot = static_cast<int>(side);
            }
        }
        PxVec3 normal = foot.normal;
        if (foot.available && foot.hasContactSample && foot.grounded &&
            normal.isFinite() && normal.normalize() > 1e-6f && normal.y >= .70710678f &&
            std::isfinite(foot.normalImpulse) && foot.normalImpulse >= 0 &&
            std::isfinite(input.bodyWeight) && input.bodyWeight > 0)
        {
            const float ratio = foot.normalImpulse * normal.y / (input.fixedDeltaTime * input.bodyWeight);
            transfer.loadRatios[side] = std::isfinite(ratio) ? ratio : 0;
        }
    }
    if (transfer.candidateFoot >= 0 && transfer.supportFoot >= 0 && transfer.supportFoot < 2 &&
        distances[transfer.supportFoot] <= closestDistance + settings.supportSwitchDistance)
    {
        transfer.candidateFoot = transfer.supportFoot;
    }
    if (transfer.candidateFoot != previousCandidate)
    {
        transfer.confirmationTime = 0;
        transfer.consecutiveSamples = 0;
    }
    if (transfer.candidateFoot < 0 || transfer.loadRatios[transfer.candidateFoot] < settings.supportLoadRatio)
    {
        transfer.confirmationTime = 0;
        transfer.consecutiveSamples = 0;
        return;
    }
    // 평균 하중만 사용하면 착지 피크의 잔상이 남는다. 매 샘플의 유효 접촉과 최소 하중을 연속 확인한다.
    transfer.confirmationTime = (std::min)(transfer.confirmationTime + input.fixedDeltaTime,
        settings.supportConfirmationTime);
    transfer.consecutiveSamples = (std::min)(transfer.consecutiveSamples + 1, uint32_t(2));
    transfer.ready = transfer.consecutiveSamples >= 2 &&
        transfer.confirmationTime + 1e-6f >= settings.supportConfirmationTime;
    if (transfer.ready)
    {
        transfer.supportFoot = transfer.candidateFoot;
    }
}

void FootstepPlanner::Update(const FootstepInput& input, const FootstepSettings& settings)
{
    if (m_HasSample && input.revision != m_Snapshot.revision)
    {
        Reset();
    }
    if (m_HasSample && input.completedStep == m_Snapshot.completedStep)
    {
        return;
    }
    FootstepSnapshot& state = m_Snapshot;
    const bool consecutive = m_HasSample && input.completedStep == state.completedStep + 1;
    m_HasSample = true;
    state.completedStep = input.completedStep;
    state.revision = input.revision;
    state.justStarted = false;
    state.justLanded = false;
    state.hasTarget = false;
    state.hasPrediction = false;
    state.needsStep = false;
    state.motion.valid = false;
    if (!consecutive)
    {
        state.transfer.confirmationTime = 0;
        state.transfer.consecutiveSamples = 0;
    }
    if (input.active && input.automaticSupport && input.outsideBufferReleased)
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
    // 재생 중인 애니메이션과 독립적으로 물리 제어 모드에서 활성화한다.
    if (!input.active || !input.automaticSupport)
    {
        state.phase = FootstepPhase::Idle;
        state.stepFoot = -1;
        state.supportFoot = -1;
        state.rawBlueFoot = -1;
        state.motion = {};
        state.transfer = {};
        state.candidates = {};
        state.reason = input.active ? FootstepReason::AutomaticSupportRequired : FootstepReason::Inactive;
        return;
    }
    if (!std::isfinite(input.fixedDeltaTime) || input.fixedDeltaTime <= 0)
    {
        state.reason = FootstepReason::InvalidSettings;
        return;
    }

    const bool validSettings = ValidSettings(settings);
    const bool validCom = input.hasCom && input.hasVelocity && input.com.isFinite() && input.velocity.isFinite();
    const bool validSupport = input.zone != FootSupportZone::Invalid && input.hull.size() >= 3 &&
        std::isfinite(input.projectionHeight);
    if (validSettings && validCom && validSupport)
    {
        state.predictedCom = PxVec3(input.com.x + input.velocity.x * settings.startPredictionTime,
            input.projectionHeight, input.com.z + input.velocity.z * settings.startPredictionTime);
        state.roleSelectionCom = SelectionCom(input, settings);
        state.hasPrediction = state.predictedCom.isFinite();
        state.needsStep = state.hasPrediction && !IsInsideFootSupportHull(input.hull, state.predictedCom);
    }

    // 시작 이후에는 COM과 접촉 평면이 바뀌어도 착지점과 골반 계획을 다시 고르지 않는다.
    if (state.IsInProgress())
    {
        const uint64_t elapsedSteps = input.completedStep >= state.startedStep ? input.completedStep - state.startedStep : 0;
        const double elapsed = static_cast<double>(elapsedSteps) * input.fixedDeltaTime;
        state.motion.elapsedTime = static_cast<float>((std::min)(elapsed, static_cast<double>(state.motion.duration)));
        state.motion.progress = (std::min)(state.motion.elapsedTime / state.motion.duration, 1.0f);
        const FootstepFootInput& foot = input.feet[state.stepFoot];
        if (!foot.available || !foot.hasContactSample)
        {
            state.reason = FootstepReason::NoContactSample;
            return;
        }
        if (!foot.grounded)
        {
            state.phase = FootstepPhase::Airborne;
            state.motion.deadlineMissed = state.motion.deadlineMissed || elapsed + 1e-6 >= state.motion.duration;
        }
        else if (elapsed + 1e-6 >= state.motion.duration)
        {
            UpdateMotionProgress(foot);
            state.phase = FootstepPhase::Landed;
            state.reason = FootstepReason::Landed;
            state.justLanded = true;
            state.transfer = {};
            state.transfer.active = true;
            state.transfer.supportFoot = state.supportFoot;
            state.motion.landingError = HorizontalDistance(foot.position, state.target);
            state.motion.hasLandingError = std::isfinite(state.motion.landingError);
            state.motion.deadlineMissed = state.motion.deadlineMissed ||
                !state.motion.hasLandingError || state.motion.landingError > state.motion.landingTolerance;
            state.motion.remainingDistance = state.motion.hasLandingError ? state.motion.landingError : 0;
            ++state.landingCount;
            return;
        }
        state.hasTarget = true;
        state.reason = state.motion.deadlineMissed ? FootstepReason::LandingDelayed :
            state.phase == FootstepPhase::Airborne ? FootstepReason::InFlight : FootstepReason::WaitingTakeoff;
        UpdateMotionTarget(input);
        return;
    }

    state.candidates = {};
    if (!validSettings || !validCom || !validSupport || !state.hasPrediction)
    {
        state.transfer.confirmationTime = 0;
        state.transfer.consecutiveSamples = 0;
        state.reason = !validSettings ? FootstepReason::InvalidSettings :
            !validCom ? FootstepReason::InvalidCom : FootstepReason::NoSupportArea;
        return;
    }
    const int previousSupport = state.transfer.active ? state.transfer.supportFoot : state.supportFoot;
    const int selectedSupport = SelectSupportFoot(input, settings, previousSupport);
    state.rawBlueFoot = selectedSupport >= 0 ? 1 - selectedSupport : -1;
    int movingFoot = state.rawBlueFoot;
    if (state.transfer.active)
    {
        UpdateSupportTransfer(input, settings);
        if (!state.transfer.ready)
        {
            state.reason = FootstepReason::WaitingSupportLoad;
            return;
        }
        movingFoot = 1 - state.transfer.supportFoot;
    }
    state.phase = FootstepPhase::Idle;
    state.stepFoot = -1;
    state.supportFoot = selectedSupport;
    if (!state.needsStep)
    {
        state.reason = FootstepReason::PredictedInside;
        return;
    }
    if (movingFoot < 0 || movingFoot > 1)
    {
        state.reason = FootstepReason::NoStanceContact;
        return;
    }

    // 역할을 먼저 고르고 해당 이동발의 출발 계획만 계산한다.
    state.candidates[movingFoot] = MakeTarget(input, settings, static_cast<size_t>(movingFoot));
    const FootstepTarget& candidate = state.candidates[movingFoot];
    if (!candidate.valid)
    {
        state.reason = candidate.reason;
        return;
    }
    const FootstepFootInput& foot = input.feet[movingFoot];
    state.stepFoot = movingFoot;
    state.supportFoot = 1 - movingFoot;
    state.phase = foot.grounded ? FootstepPhase::AwaitingTakeoff : FootstepPhase::Airborne;
    state.startPosition = foot.position;
    state.startRotation = foot.rotation;
    state.target = candidate.position;
    state.initialTarget = candidate.position;
    state.targetNormal = candidate.normal;
    state.initialDistance = candidate.travelDistance;
    state.reachability = candidate.reachability;
    state.motion = {};
    state.motion.heightLimit = foot.calfLength;
    state.motion.initialHeight = (std::min)(foot.calfLength,
        (std::max)(settings.minimumLiftHeight, state.initialDistance * settings.liftHeightScale));
    state.motion.duration = settings.stepDuration;
    state.motion.landingTolerance = settings.reachPositionTolerance;
    state.motion.landingTargetLocked = true;
    state.motion.landingLoweringSpeed = settings.landingLoweringSpeed;
    state.motion.maximumLandingLowering = (std::min)(settings.maximumLandingLowering, foot.calfLength);
    state.startedStep = input.completedStep;
    state.justStarted = true;
    state.hasTarget = true;
    state.reason = foot.grounded ? FootstepReason::WaitingTakeoff : FootstepReason::InFlight;
    ++state.startCount;
    UpdateMotionTarget(input);
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
    case FootstepReason::InvalidSettings:
        return u8"설정값 오류";
    case FootstepReason::InvalidCom:
        return u8"COM 위치 또는 실제 속도 없음";
    case FootstepReason::NoSupportArea:
        return u8"유효한 A/B 영역 없음";
    case FootstepReason::NoReachability:
        return u8"물리 다리의 도달 가능성 입력 없음";
    case FootstepReason::Unreachable:
        return u8"선분 안에 양쪽 다리 IK가 가능한 후보 없음";
    case FootstepReason::InsufficientTime:
        return u8"선분 안에 계획 시간 내 도달 가능한 후보 없음";
    case FootstepReason::MinimumDistanceConflict:
        return u8"선분 안에서 지지발 최소거리 확보 불가";
    case FootstepReason::LandingDelayed:
        return u8"착지 기한 초과 / 고정 목표의 접촉 대기";
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
        return u8"이동 시간 종료 및 접촉 확인 / 지지 하중 확인 시작";
    case FootstepReason::WaitingSupportLoad:
        return u8"착지 후 예측 지지발의 지속 접촉·하중 대기";
    case FootstepReason::OutsideBuffer:
        return u8"실제 COM의 B 이탈 / 재진입해도 재개하지 않음";
    case FootstepReason::RootSupportReleased:
        return u8"기존 골반 지지 해제 상태";
    default:
        return u8"알 수 없음";
    }
}
