#include "FootstepPlanner.h"

using namespace physx;
using namespace PARS;

namespace
{
    constexpr float MinimumStepDistance = 1e-5f;

    bool ValidSettings(const FootstepSettings& s)
    {
        return std::isfinite(s.startPredictionTime) && s.startPredictionTime > 0 &&
            std::isfinite(s.landingPredictionTime) && s.landingPredictionTime > 0 &&
            std::isfinite(s.stanceScale) && s.stanceScale >= 0 &&
            std::isfinite(s.minimumStanceWidthRatio) && s.minimumStanceWidthRatio >= 0 && s.minimumStanceWidthRatio <= 1 &&
            std::isfinite(s.maximumStepLength) && s.maximumStepLength > 0 &&
            std::isfinite(s.crossingOffset) && s.crossingOffset >= 0 &&
            std::isfinite(s.liftHeightScale) && s.liftHeightScale > 0 &&
            std::isfinite(s.stepDuration) && s.stepDuration > 0 &&
            std::isfinite(s.minimumLiftHeight) && s.minimumLiftHeight > 0 &&
            std::isfinite(s.landingLockProgress) && s.landingLockProgress >= .5f && s.landingLockProgress <= 1 &&
            std::isfinite(s.landingLoweringSpeed) && s.landingLoweringSpeed >= 0 &&
            std::isfinite(s.maximumLandingLowering) && s.maximumLandingLowering >= 0 &&
            std::isfinite(s.supportLoadRatio) && s.supportLoadRatio > 0 && s.supportLoadRatio <= 1 &&
            std::isfinite(s.supportConfirmationTime) && s.supportConfirmationTime > 0;
    }

    float HorizontalDistance(const PxVec3& a, const PxVec3& b)
    {
        const PxVec3 delta = a - b;
        return std::sqrt(delta.x * delta.x + delta.z * delta.z);
    }

    void UpdateGroundPlane(const FootstepInput& input, FootstepSnapshot& state)
    {
        PxVec3 position(0);
        PxVec3 normal(0);
        unsigned count = 0;
        for (const FootstepFootInput& foot : input.feet)
        {
            PxVec3 contactNormal = foot.normal;
            if (foot.available && foot.hasContactSample && foot.grounded && foot.contactPosition.isFinite() &&
                contactNormal.isFinite() && contactNormal.normalize() > 1e-6f && contactNormal.y >= .70710678f)
            {
                position += foot.contactPosition;
                normal += contactNormal;
                ++count;
            }
        }
        if (count > 0 && normal.normalize() > 1e-6f)
        {
            // 특정 발에 지면 기준 역할을 고정하지 않고 실제 접지 관측을 공유한다.
            state.groundPosition = position / static_cast<float>(count);
            state.groundNormal = normal;
            state.hasGroundPlane = true;
        }
    }

    FootstepTarget MakeTarget(const FootstepInput& input, const FootstepSettings& settings,
        const FootstepSnapshot& state, size_t side)
    {
        FootstepTarget result;
        result.reason = FootstepReason::InvalidTarget;
        const FootstepFootInput& foot = input.feet[side];
        if (!input.hasReference)
        {
            result.reason = FootstepReason::NoReference;
            return result;
        }
        if (!foot.available || !foot.position.isFinite() || !foot.stanceOffset.isFinite())
        {
            return result;
        }
        if (!state.hasGroundPlane)
        {
            result.reason = FootstepReason::NoGroundPlane;
            return result;
        }
        const PxVec3 normal = state.groundNormal;
        PxVec3 origin = state.feet[side].IsInProgress() ? state.feet[side].startPosition : foot.position;
        origin.y -= normal.dot(origin - state.groundPosition) / normal.y;
        result.raw = PxVec3(input.com.x + input.velocity.x * settings.landingPredictionTime,
            state.groundPosition.y, input.com.z + input.velocity.z * settings.landingPredictionTime);
        result.raw += PxVec3(foot.stanceOffset.x, 0, foot.stanceOffset.z) * settings.stanceScale;
        PxVec3 delta = result.raw - origin;
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
        const FootstepFootInput& other = input.feet[1 - side];
        if ((settings.preventCrossing || settings.minimumStanceWidthRatio > 0) &&
            other.available && other.position.isFinite() && other.stanceOffset.isFinite())
        {
            PxVec3 outward = input.supportRight;
            const float sideDirection = (foot.stanceOffset - other.stanceOffset).dot(outward);
            if (!outward.isFinite() || std::abs(sideDirection) < 1e-6f)
            {
                return result;
            }
            outward *= sideDirection > 0 ? 1.0f : -1.0f;
            outward -= normal * outward.dot(normal);
            if (outward.normalize() < 1e-6f)
            {
                return result;
            }
            // 반대 발은 간격 계산에만 사용하며 그 발의 접지 여부는 요구하지 않는다.
            const float idleWidth = std::abs((foot.stanceOffset - other.stanceOffset).dot(outward));
            const float minimumWidth = idleWidth * settings.minimumStanceWidthRatio;
            const float crossingWidth = settings.preventCrossing ? settings.crossingOffset : 0.0f;
            const float relativeOther = (other.position - origin).dot(outward);
            const float requiredLateral = relativeOther + (std::max)(minimumWidth, crossingWidth);
            const float minimumLateral = (std::min)(settings.maximumStepLength, requiredLateral);
            const float lateral = delta.dot(outward);
            if (lateral < minimumLateral)
            {
                PxVec3 tangent = delta - outward * lateral;
                const float limit = std::sqrt((std::max)(0.0f,
                    settings.maximumStepLength * settings.maximumStepLength - minimumLateral * minimumLateral));
                const float tangentLength = tangent.magnitude();
                if (tangentLength > limit && tangentLength > 0)
                {
                    tangent *= limit / tangentLength;
                    result.lengthLimited = true;
                }
                delta = outward * minimumLateral + tangent;
                result.stanceWidthLimited = lateral < relativeOther + minimumWidth;
                result.crossingLimited = settings.preventCrossing && lateral < relativeOther + crossingWidth;
                result.lengthLimited = result.lengthLimited || requiredLateral > settings.maximumStepLength;
            }
        }
        result.position = origin + delta;
        result.normal = normal;
        result.travelDistance = HorizontalDistance(foot.position, result.position);
        result.separationDistance = other.available && other.position.isFinite() ?
            HorizontalDistance(other.position, result.position) : 0;
        result.valid = result.position.isFinite() && std::isfinite(result.travelDistance);
        result.reason = result.valid ? FootstepReason::ValidTarget : FootstepReason::InvalidTarget;
        return result;
    }

    int SelectRequestedFoot(const FootstepInput& input, const FootstepSnapshot& state)
    {
        if (input.requestedFoot >= 0 && input.requestedFoot < 2)
        {
            return input.requestedFoot;
        }
        // 새 충격이 없으면 이동 자체로 거리 순서가 바뀌어도 추가 명령을 만들지 않는다.
        if (state.IsInProgress() && !input.hasImpact)
        {
            return state.requestedFoot;
        }
        if (state.hasImpact)
        {
            int selected = -1;
            float closest = PX_MAX_F32;
            for (size_t side = 0; side < input.feet.size(); ++side)
            {
                const FootstepFootInput& foot = input.feet[side];
                if (foot.available && foot.position.isFinite())
                {
                    const float distance = HorizontalDistance(state.lastImpactPosition, foot.position);
                    if (distance < closest)
                    {
                        closest = distance;
                        selected = static_cast<int>(side);
                    }
                }
            }
            return selected;
        }
        // 충격 관측이 없는 초기 자세 오차에는 목표 오차가 큰 발로 명령한다.
        int selected = -1;
        float greatestDistance = MinimumStepDistance;
        for (size_t side = 0; side < state.candidates.size(); ++side)
        {
            const FootstepTarget& candidate = state.candidates[side];
            if (candidate.valid && candidate.travelDistance > greatestDistance)
            {
                greatestDistance = candidate.travelDistance;
                selected = static_cast<int>(side);
            }
        }
        return selected;
    }

    void UpdateLoadTransfer(const FootstepInput& input, const FootstepSettings& settings,
        bool consecutiveSample, FootstepSnapshot& state)
    {
        FootstepLoadTransfer& transfer = state.loadTransfer;
        transfer.loadRatios = {};
        transfer.ready = false;
        std::array<bool, 2> valid{};
        if (std::isfinite(input.bodyWeight) && input.bodyWeight > 0)
        {
            const double denominator = static_cast<double>(input.bodyWeight) * input.fixedDeltaTime;
            for (size_t side = 0; side < input.feet.size(); ++side)
            {
                const FootstepFootInput& foot = input.feet[side];
                PxVec3 normal = foot.normal;
                if (foot.available && foot.hasContactSample && foot.grounded && normal.isFinite() &&
                    normal.normalize() > 1e-6f && normal.y >= .70710678f &&
                    std::isfinite(foot.normalImpulse) && foot.normalImpulse >= 0)
                {
                    // 접촉 충격량의 수직 성분을 힘으로 바꾸고 전신 체중으로 정규화한다.
                    const float ratio = static_cast<float>(static_cast<double>(foot.normalImpulse) * normal.y / denominator);
                    valid[side] = std::isfinite(ratio);
                    transfer.loadRatios[side] = valid[side] ? ratio : 0;
                }
            }
        }
        bool landedThisStep = false;
        for (const FootstepCommand& command : state.feet)
        {
            landedThisStep = landedThisStep || command.justLanded;
        }
        if (!transfer.required || state.IsInProgress() || landedThisStep)
        {
            // 착지 충격 자체와 아직 이동 중인 발의 접촉은 다음 명령의 확인 시간에 넣지 않는다.
            transfer.confirmationTimes = {};
            return;
        }
        if (!consecutiveSample)
        {
            transfer.confirmationTimes = {};
        }
        for (size_t side = 0; side < input.feet.size(); ++side)
        {
            if (valid[side] && transfer.loadRatios[side] >= settings.supportLoadRatio * (1.0f - 1e-6f))
            {
                transfer.confirmationTimes[side] = (std::min)(settings.supportConfirmationTime,
                    transfer.confirmationTimes[side] + input.fixedDeltaTime);
            }
            else
            {
                transfer.confirmationTimes[side] = 0;
            }
        }
        if (state.requestedFoot >= 0 && state.requestedFoot < 2)
        {
            // 발 선택은 유지하고, 그 발을 들었을 때 남을 접촉의 실측 하중만 검사한다.
            const size_t receivingFoot = static_cast<size_t>(1 - state.requestedFoot);
            transfer.ready = valid[receivingFoot] && transfer.confirmationTimes[receivingFoot] > 0 &&
                transfer.confirmationTimes[receivingFoot] + 1e-6f >= settings.supportConfirmationTime;
        }
    }
}

bool FootstepSnapshot::IsInProgress() const
{
    return feet[0].IsInProgress() || feet[1].IsInProgress();
}

float FootstepSnapshot::GetMaximumInitialDistance() const
{
    float distance = 0;
    for (const FootstepCommand& command : feet)
    {
        if (command.IsInProgress())
        {
            distance = (std::max)(distance, command.initialDistance);
        }
    }
    return distance;
}

void FootstepPlanner::Reset()
{
    m_Snapshot = {};
    m_HasSample = false;
}

void FootstepPlanner::Abort(FootstepReason reason)
{
    if (!m_Snapshot.aborted)
    {
        ++m_Snapshot.abortCount;
    }
    m_Snapshot.aborted = true;
    m_Snapshot.reason = reason;
    m_Snapshot.candidates = {};
    m_Snapshot.loadTransfer = {};
    for (FootstepCommand& command : m_Snapshot.feet)
    {
        command.phase = FootstepPhase::Aborted;
        command.reason = reason;
        command.hasTarget = false;
        command.motion.valid = false;
    }
}

void FootstepPlanner::UpdateMotion(FootstepCommand& command, const FootstepFootInput& foot, const FootstepInput& input)
{
    FootstepMotion& motion = command.motion;
    const PxVec3 horizontal = command.target.position - command.startPosition;
    const float lengthSquared = horizontal.x * horizontal.x + horizontal.z * horizontal.z;
    motion.remainingDistance = HorizontalDistance(foot.position, command.target.position);
    if (!foot.position.isFinite() || !std::isfinite(motion.remainingDistance))
    {
        return;
    }
    if (lengthSquared > MinimumStepDistance * MinimumStepDistance)
    {
        const PxVec3 actual = foot.position - command.startPosition;
        motion.measuredProgress = PxClamp((actual.x * horizontal.x + actual.z * horizontal.z) / lengthSquared, 0.0f, 1.0f);
    }
    else
    {
        motion.measuredProgress = motion.remainingDistance <= MinimumStepDistance ? 1.0f : 0.0f;
    }
    if (!command.hasTakenOff)
    {
        // 실제 이륙 전에는 수평으로 끌지 않고 위로 드는 목표를 유지한다.
        const float liftProgress = PxClamp(motion.takeoffElapsedTime / (.5f * motion.duration), 0.0f, 1.0f);
        const float liftBlend = liftProgress * liftProgress * (3.0f - 2.0f * liftProgress);
        motion.height = motion.initialHeight * liftBlend;
        motion.position = command.startPosition + PxVec3(0, motion.height, 0);
        motion.valid = motion.position.isFinite();
        return;
    }
    const float t = motion.progress;
    const float blend = t * t * (3.0f - 2.0f * t);
    motion.height = motion.initialHeight * .5f * (1.0f - std::cos(PxTwoPi * t));
    motion.position = command.takeoffPosition + (command.target.position - command.takeoffPosition) * blend + PxVec3(0, motion.height, 0);
    // 이륙 때 이미 확보한 높이를 중복으로 더해 최대 들기 높이를 넘지 않는다.
    const float maximumHeight = (std::max)(command.startPosition.y, command.target.position.y) + motion.initialHeight;
    motion.position.y = (std::min)(motion.position.y, maximumHeight);
    if (motion.landingTargetLocked && !foot.grounded)
    {
        const double elapsed = static_cast<double>(input.completedStep - command.takeoffStep) * input.fixedDeltaTime;
        const double overtime = (std::max)(0.0, elapsed - static_cast<double>(motion.duration));
        const float requested = static_cast<float>((std::min)(overtime * motion.landingLoweringSpeed,
            static_cast<double>(motion.maximumLandingLowering)));
        motion.landingLowering = (std::min)(requested, motion.landingLowering + motion.landingLoweringSpeed * input.fixedDeltaTime);
        motion.position.y -= motion.landingLowering;
    }
    motion.valid = motion.position.isFinite();
}

void FootstepPlanner::Update(const FootstepInput& input, const FootstepSettings& settings)
{
    if (m_HasSample && (input.revision != m_Snapshot.revision || input.completedStep < m_Snapshot.completedStep))
    {
        Reset();
    }
    if (m_HasSample && input.completedStep == m_Snapshot.completedStep)
    {
        return;
    }
    const bool consecutiveSample = m_HasSample && input.completedStep > m_Snapshot.completedStep &&
        input.completedStep - m_Snapshot.completedStep == 1;
    FootstepSnapshot& state = m_Snapshot;
    m_HasSample = true;
    state.completedStep = input.completedStep;
    state.revision = input.revision;
    state.hasPrediction = false;
    state.needsStep = false;
    state.candidates = {};
    state.loadTransfer.ready = false;
    for (FootstepCommand& command : state.feet)
    {
        command.justStarted = false;
        command.justLanded = false;
        command.hasTarget = false;
        command.motion.valid = false;
    }
    if (input.active && input.automaticSupport && input.outsideBufferReleased)
    {
        Abort(FootstepReason::OutsideBuffer);
        return;
    }
    if (state.aborted)
    {
        return;
    }
    if (input.active && input.automaticSupport && input.rootReleased)
    {
        Abort(FootstepReason::RootSupportReleased);
        return;
    }
    if (!input.active || !input.automaticSupport || !input.idleAnimation)
    {
        state.feet = {};
        state.loadTransfer = {};
        state.hasImpact = false;
        state.hasGroundPlane = false;
        state.requestedFoot = -1;
        state.reason = !input.active ? FootstepReason::Inactive : !input.automaticSupport ?
            FootstepReason::AutomaticSupportRequired : FootstepReason::IdleRequired;
        return;
    }
    if (!ValidSettings(settings) || !std::isfinite(input.fixedDeltaTime) || input.fixedDeltaTime <= 0)
    {
        state.reason = FootstepReason::InvalidSettings;
        state.loadTransfer.confirmationTimes = {};
        return;
    }
    if (!input.hasCom || !input.hasVelocity || !input.com.isFinite() || !input.velocity.isFinite())
    {
        state.reason = FootstepReason::InvalidCom;
        state.loadTransfer.confirmationTimes = {};
        return;
    }
    UpdateGroundPlane(input, state);
    if (input.hasImpact && input.impactPosition.isFinite())
    {
        state.lastImpactPosition = input.impactPosition;
        state.hasImpact = true;
    }
    state.predictedCom = PxVec3(input.com.x + input.velocity.x * settings.startPredictionTime,
        input.projectionHeight, input.com.z + input.velocity.z * settings.startPredictionTime);
    state.hasPrediction = state.predictedCom.isFinite();
    const bool validArea = input.zone != FootSupportZone::Invalid && input.hull.size() >= 3;
    state.needsStep = state.hasPrediction && validArea && !IsInsideFootSupportHull(input.hull, state.predictedCom);
    state.reason = !state.hasPrediction ? FootstepReason::InvalidCom : !validArea ? FootstepReason::NoSupportArea :
        state.needsStep ? FootstepReason::ValidTarget : FootstepReason::PredictedInside;

    for (size_t side = 0; side < state.feet.size(); ++side)
    {
        FootstepCommand& command = state.feet[side];
        const FootstepFootInput& foot = input.feet[side];
        if (command.IsInProgress())
        {
            if (!command.hasTakenOff)
            {
                command.motion.takeoffElapsedTime = static_cast<float>(
                    static_cast<double>(input.completedStep - command.startedStep) * input.fixedDeltaTime);
            }
            if (!foot.available || !foot.hasContactSample)
            {
                command.reason = FootstepReason::NoContactSample;
                continue;
            }
            if (!foot.grounded)
            {
                if (!command.hasTakenOff)
                {
                    // 실제 접촉이 끊긴 시점부터 해당 발의 공중 이동 시간을 센다.
                    command.hasTakenOff = true;
                    command.takeoffStep = input.completedStep;
                    command.takeoffPosition = command.motion.position;
                }
                command.phase = FootstepPhase::Airborne;
            }
            if (command.hasTakenOff)
            {
                const double elapsed = static_cast<double>(input.completedStep - command.takeoffStep) * input.fixedDeltaTime;
                command.motion.elapsedTime = static_cast<float>((std::min)(elapsed, static_cast<double>(command.motion.duration)));
                command.motion.progress = (std::min)(command.motion.elapsedTime / command.motion.duration, 1.0f);
                command.motion.landingTargetLocked = command.motion.landingTargetLocked ||
                    command.motion.progress >= command.motion.landingLockProgress;
            }
            if (command.hasTakenOff && foot.grounded && command.motion.elapsedTime + 1e-6f >= command.motion.duration)
            {
                command.phase = FootstepPhase::Landed;
                command.reason = FootstepReason::Landed;
                command.justLanded = true;
                command.motion.landingError = HorizontalDistance(foot.position, command.target.position);
                command.motion.hasLandingError = std::isfinite(command.motion.landingError);
                command.motion.remainingDistance = command.motion.landingError;
                ++state.landingCount;
            }
        }
        state.candidates[side] = MakeTarget(input, settings, state, side);
    }
    state.requestedFoot = SelectRequestedFoot(input, state);
    UpdateLoadTransfer(input, settings, consecutiveSample, state);
    for (size_t side = 0; side < state.feet.size(); ++side)
    {
        FootstepCommand& command = state.feet[side];
        const FootstepFootInput& foot = input.feet[side];
        const FootstepTarget& candidate = state.candidates[side];
        if (!command.IsInProgress())
        {
            if (command.justLanded || state.requestedFoot != static_cast<int>(side) || !state.needsStep)
            {
                continue;
            }
            command.reason = candidate.reason;
            if (!candidate.valid)
            {
                continue;
            }
            if (!foot.hasContactSample)
            {
                command.reason = FootstepReason::NoContactSample;
                continue;
            }
            if (!std::isfinite(foot.calfLength) || foot.calfLength <= 0)
            {
                command.reason = FootstepReason::NoCalfLength;
                continue;
            }
            if (candidate.travelDistance <= MinimumStepDistance)
            {
                command.reason = FootstepReason::NoStepDistance;
                continue;
            }
            if (state.loadTransfer.required && !state.loadTransfer.ready)
            {
                command.reason = FootstepReason::WaitingSupportLoad;
                continue;
            }
            command = {};
            command.startedStep = input.completedStep;
            command.takeoffStep = input.completedStep;
            command.startPosition = foot.position;
            command.takeoffPosition = foot.position;
            command.initialTarget = candidate.position;
            command.initialDistance = candidate.travelDistance;
            command.target = candidate;
            command.phase = foot.grounded ? FootstepPhase::AwaitingTakeoff : FootstepPhase::Airborne;
            command.hasTakenOff = !foot.grounded;
            command.justStarted = true;
            command.motion.heightLimit = foot.calfLength;
            command.motion.initialHeight = (std::min)((std::max)(command.initialDistance * settings.liftHeightScale,
                settings.minimumLiftHeight), foot.calfLength);
            command.motion.duration = settings.stepDuration;
            command.motion.landingLockProgress = settings.landingLockProgress;
            command.motion.landingLoweringSpeed = settings.landingLoweringSpeed;
            command.motion.maximumLandingLowering = (std::min)(settings.maximumLandingLowering, foot.calfLength);
            ++state.startCount;
            state.loadTransfer.required = true;
            state.loadTransfer.ready = false;
            state.loadTransfer.confirmationTimes = {};
        }
        if (!foot.available || !foot.hasContactSample)
        {
            continue;
        }
        if (!command.motion.landingTargetLocked && candidate.valid)
        {
            command.target = candidate;
        }
        else if (command.motion.landingTargetLocked && state.hasGroundPlane)
        {
            command.target.position.y -= state.groundNormal.dot(command.target.position - state.groundPosition) / state.groundNormal.y;
            command.target.normal = state.groundNormal;
        }
        if (!command.target.valid)
        {
            command.reason = candidate.reason;
            continue;
        }
        command.hasTarget = true;
        command.reason = command.phase == FootstepPhase::Airborne ? FootstepReason::InFlight : FootstepReason::WaitingTakeoff;
        UpdateMotion(command, foot, input);
    }
    if (state.needsStep && !state.IsInProgress())
    {
        state.reason = state.requestedFoot >= 0 ? state.feet[state.requestedFoot].reason :
            state.hasGroundPlane ? FootstepReason::NoFootRequest : FootstepReason::NoGroundPlane;
    }
}

const char* PARS::FootstepPhaseLabel(FootstepPhase phase)
{
    switch (phase)
    {
    case FootstepPhase::Idle:
        return u8"대기";
    case FootstepPhase::AwaitingTakeoff:
        return u8"이륙 대기";
    case FootstepPhase::Airborne:
        return u8"공중 이동";
    case FootstepPhase::Landed:
        return u8"착지";
    case FootstepPhase::Aborted:
        return u8"중단";
    default:
        return u8"알 수 없음";
    }
}

const char* PARS::FootstepReasonLabel(FootstepReason reason)
{
    switch (reason)
    {
    case FootstepReason::NotEvaluated:
        return u8"아직 평가하지 않음";
    case FootstepReason::ValidTarget:
        return u8"유효한 착지 후보";
    case FootstepReason::Inactive:
        return u8"액티브 레그돌 아님";
    case FootstepReason::AutomaticSupportRequired:
        return u8"자동 지지 기능 필요";
    case FootstepReason::IdleRequired:
        return u8"Idle 애니메이션 필요";
    case FootstepReason::InvalidSettings:
        return u8"잘못된 설정 또는 시간";
    case FootstepReason::InvalidCom:
        return u8"유효한 COM 정보 없음";
    case FootstepReason::NoSupportArea:
        return u8"유효한 지면 영역 없음";
    case FootstepReason::NoReference:
        return u8"Idle 발 기준 없음";
    case FootstepReason::NoCalfLength:
        return u8"유효한 종아리 길이 없음";
    case FootstepReason::NoStepDistance:
        return u8"이동 거리 없음";
    case FootstepReason::NoContactSample:
        return u8"해당 발의 접촉 관측 없음";
    case FootstepReason::PredictedInside:
        return u8"예측 COM이 영역 안";
    case FootstepReason::NoFootRequest:
        return u8"발 이동 명령 없음";
    case FootstepReason::NoGroundPlane:
        return u8"관측한 지면 평면 없음";
    case FootstepReason::InvalidTarget:
        return u8"유효하지 않은 목표";
    case FootstepReason::WaitingTakeoff:
        return u8"해당 발의 이륙 대기";
    case FootstepReason::InFlight:
        return u8"해당 발의 이동 진행";
    case FootstepReason::Landed:
        return u8"해당 발의 실제 접촉 확인";
    case FootstepReason::OutsideBuffer:
        return u8"B 이탈 유예 만료";
    case FootstepReason::RootSupportReleased:
        return u8"골반 보조 해제";
    case FootstepReason::WaitingSupportLoad:
        return u8"착지 후 지지 하중 확인 대기";
    default:
        return u8"알 수 없음";
    }
}
