#include "FootstepPlanner.h"

#include "FramePerformance.h"
#include <limits>

using namespace physx;
using namespace PARS;

namespace
{
    constexpr float MinimumStepDistance = 1e-5f;

    void AssignTarget(FootstepCommand& command, const FootstepTarget& target)
    {
        command.target = target;
        // 영역 도형은 이번 틱의 후보에만 보관한다. 경로 검사에서 명령을 복사할 때 힙 할당을 반복하지 않는다.
        command.target.recovery.reachable = {};
        command.target.recovery.capture.clear();
        command.target.recovery.intersections = {};
    }

    float LiftHeight(const FootstepMotion& motion, float elapsed)
    {
        const float t = PxClamp(elapsed / (.5f * motion.duration), 0.0f, 1.0f);
        const float blend = t * t * (3.0f - 2.0f * t);
        return motion.initialHeight * blend;
    }

    float SwingHeight(const FootstepMotion& motion, float progress)
    {
        return motion.initialHeight * .5f * (1.0f - std::cos(PxTwoPi * progress));
    }

    PxVec3 SwingPosition(const FootstepCommand& command, const PxVec3& takeoff, float progress)
    {
        const float blend = progress * progress * (3.0f - 2.0f * progress);
        PxVec3 position = takeoff + (command.target.position - takeoff) * blend +
            PxVec3(0, SwingHeight(command.motion, progress), 0);
        // 이륙 때 이미 확보한 높이를 중복으로 더하지 않도록 구동과 예측에 같은 상한을 쓴다.
        const float maximumHeight = (std::max)(command.startPosition.y, command.target.position.y) +
            command.motion.initialHeight;
        position.y = (std::min)(position.y, maximumHeight);
        return position;
    }

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
            std::isfinite(s.supportConfirmationTime) && s.supportConfirmationTime > 0 &&
            (!s.landingComClampEnabled ||
                (std::isfinite(s.landingComClampHysteresisDegrees) && s.landingComClampHysteresisDegrees >= 0 &&
                    s.landingComClampHysteresisDegrees < 90));
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

    FootstepComPrediction PredictCom(const FootstepInput& input, const FootstepSettings& settings,
        float predictionTime, float projectionHeight, int movingFoot, FootstepLandingDirection previousDirection)
    {
        FootstepComPrediction result;
        result.origin = PxVec3(input.com.x, projectionHeight, input.com.z);
        result.raw = PxVec3(input.com.x + input.velocity.x * predictionTime,
            projectionHeight, input.com.z + input.velocity.z * predictionTime);
        result.clamped = result.raw;
        result.enabled = settings.landingComClampEnabled;
        result.valid = result.raw.isFinite();
        if (!result.valid || movingFoot < 0 || movingFoot >= static_cast<int>(input.feet.size()))
        {
            return result;
        }
        const size_t supportFoot = static_cast<size_t>(1 - movingFoot);
        const FootstepFootInput& support = input.feet[supportFoot];
        if (!support.available || !support.hasContactSample || !support.grounded || !support.hasSoleRectangle)
        {
            return result;
        }
        // 실제 접지한 반대 발의 사각형으로만 금지 구간을 만들며 고정 각도를 사용하지 않는다.
        PxVec3 forward(-input.supportRight.z, 0, input.supportRight.x);
        if (!forward.isFinite() || forward.normalize() <= 1e-6f)
        {
            result.valid = !result.enabled;
            return result;
        }
        result.region = BuildFootstepLandingRegion(result.origin, support.corners, forward);
        result.region.supportFoot = static_cast<int>(supportFoot);
        if (!result.enabled || !IsInsideFootstepLandingRegion(result.region, result.raw))
        {
            return result;
        }
        const PxVec3 delta = result.raw - result.origin;
        const PxVec3 frontRay = result.region.frontRay;
        const PxVec3 backRay = result.region.backRay;
        const PxVec3 front = frontRay * (std::max)(0.0f, delta.dot(frontRay));
        const PxVec3 back = backRay * (std::max)(0.0f, delta.dot(backRay));
        const float frontError = (delta - front).magnitudeSquared();
        const float backError = (delta - back).magnitudeSquared();
        const float tieTolerance = 1e-6f * delta.magnitudeSquared();
        result.direction = frontError <= backError + tieTolerance ? FootstepLandingDirection::Front : FootstepLandingDirection::Back;
        const float hysteresis = std::tan(settings.landingComClampHysteresisDegrees * PxPi / 180.0f);
        const PxVec3 tangent(-result.region.centerRay.z, 0, result.region.centerRay.x);
        if (previousDirection != FootstepLandingDirection::None &&
            std::abs(delta.dot(tangent)) <= delta.dot(result.region.centerRay) * hysteresis)
        {
            // 발을 바라보는 부채꼴 이등분선 근처에서만 이전 경계 선택을 유지한다.
            result.direction = previousDirection;
        }
        result.clamped = result.origin + (result.direction == FootstepLandingDirection::Front ? front : back);
        result.limited = true;
        result.valid = result.clamped.isFinite();
        return result;
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
        const FootstepCommand& command = state.feet[side];
        result.landingCom = PredictCom(input, settings, settings.landingPredictionTime, state.groundPosition.y,
            static_cast<int>(side),
            command.IsInProgress() ? command.landingComDirection : FootstepLandingDirection::None);
        if (!result.landingCom.valid)
        {
            return result;
        }
        result.raw = result.landingCom.clamped;
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

    bool CanReconsiderNormalStep(const FootstepCommand& command, const FootstepFootInput& foot)
    {
        return command.phase == FootstepPhase::AwaitingTakeoff && !command.hasTakenOff &&
            command.target.recovery.mode == FootstepMode::Normal && foot.hasContactSample && foot.grounded;
    }

    FootstepTarget MakeRecoveryTarget(const FootstepInput& input, const FootstepSettings& settings,
        const FootstepSnapshot& state, size_t side, bool needsStep, FootstepTarget normal)
    {
        if (!settings.recovery.enabled || !normal.valid || !input.feet[0].reach.available ||
            !input.feet[1].reach.available || (input.active && !input.feet[1 - side].grounded) || state.groundNormal.y < .995f)
        {
            return normal;
        }
        const FootstepCommand& command = state.feet[side];
        const bool inProgress = command.IsInProgress();
        FootstepRecoveryInput request;
        request.movingFoot = side;
        request.com = input.com;
        request.velocity = input.velocity;
        request.groundHeight = state.groundPosition.y;
        request.forward = PxVec3(-input.supportRight.z, 0, input.supportRight.x).getNormalized();
        request.normalTarget = normal.position;
        request.preferred = PxVec3(input.com.x + input.velocity.x * settings.landingPredictionTime,
            request.groundHeight, input.com.z + input.velocity.z * settings.landingPredictionTime) +
            PxVec3(input.feet[side].stanceOffset.x, 0, input.feet[side].stanceOffset.z) * settings.stanceScale;
        if (inProgress)
        {
            request.modeSelection = CanReconsiderNormalStep(command, input.feet[side]) ?
                FootstepRecoveryInput::ModeSelection::GroundedNormal : FootstepRecoveryInput::ModeSelection::KeepCurrent;
        }
        request.stepping = needsStep || inProgress;
        request.locked = inProgress && command.motion.landingTargetLocked;
        request.lockedTarget = command.target.position;
        request.previousMode = inProgress ? command.target.recovery.mode : FootstepMode::Normal;
        request.hasRejectedTarget = inProgress && command.target.recovery.pathRejected && !request.locked;
        request.rejectedTarget = command.target.position;
        const float transfer = (std::max)(settings.recovery.transferTime, settings.supportConfirmationTime);
        request.firstStepTime = inProgress ?
            (std::max)(input.fixedDeltaTime, GetFootstepRemainingTime(command) +
                (command.hasTakenOff ? 0 : transfer)) : settings.stepDuration * 1.5f + transfer;
        request.followingStepTime = settings.stepDuration * 1.5f + transfer;
        for (size_t foot = 0; foot < 2; ++foot)
        {
            request.feet[foot] = input.feet[foot].reach;
            FootstepReachInput& reach = request.feet[foot];
            reach.position = input.feet[foot].position;
            reach.stepOrigin = state.feet[foot].IsInProgress() ? state.feet[foot].startPosition : reach.position;
            reach.maximumDistance = settings.maximumStepLength;
            const float sign = (input.feet[foot].stanceOffset - input.feet[1 - foot].stanceOffset).dot(input.supportRight);
            reach.outward = input.supportRight * (sign >= 0 ? 1.0f : -1.0f);
            const float idleWidth = std::abs(sign);
            reach.minimumWidth = (std::max)(idleWidth * settings.minimumStanceWidthRatio,
                settings.preventCrossing ? settings.crossingOffset : 0.0f);
        }
        request.acceptsTarget = [&input, side](const PxVec3& position) -> bool
        {
            return input.acceptsLanding && input.acceptsLanding(side, position);
        };
        normal.recovery = SelectFootstepRecovery(request, settings.recovery);
        normal.recovery.pathRejected = inProgress && command.target.recovery.pathRejected;
        if (!normal.recovery.hasTarget && inProgress)
        {
            // 이번 걸음의 모드에서 새 후보를 찾지 못하면 직전 착지점을 유지한다.
            normal.position = command.target.position;
            normal.raw = command.target.raw;
            normal.recovery.mode = command.target.recovery.mode;
            normal.travelDistance = HorizontalDistance(input.feet[side].position, normal.position);
            normal.separationDistance = HorizontalDistance(input.feet[1 - side].position, normal.position);
        }
        if (normal.recovery.hasTarget)
        {
            normal.recovery.pathRejected = request.locked ? command.target.recovery.pathRejected :
                (request.hasRejectedTarget && (normal.recovery.target - request.rejectedTarget).magnitudeSquared() < .0001f);
            normal.position = normal.recovery.target;
            normal.travelDistance = HorizontalDistance(input.feet[side].position, normal.position);
            normal.separationDistance = HorizontalDistance(input.feet[1 - side].position, normal.position);
        }
        if (normal.recovery.mode != FootstepMode::Normal)
        {
            if (normal.recovery.hasTarget)
            {
                // 교차 분기의 최종 목표에 일반 딛기의 COM·간격 제한을 다시 적용하지 않는다.
                normal.raw = request.preferred;
            }
            normal.crossingLimited = false;
            normal.stanceWidthLimited = false;
            normal.landingCom.clamped = normal.landingCom.raw;
            normal.landingCom.enabled = false;
            normal.landingCom.limited = false;
            normal.landingCom.direction = FootstepLandingDirection::None;
        }
        return normal;
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
        if (!input.hasImpact && state.followupFoot >= 0 && state.remainingRecoverySteps > 0)
        {
            return state.followupFoot;
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

FootstepLandingRegion PARS::BuildFootstepLandingRegion(const PxVec3& origin,
    const std::array<PxVec3, 4>& corners, const PxVec3& forward)
{
    FootstepLandingRegion result;
    result.origin = origin;
    if (!origin.isFinite())
    {
        return result;
    }
    PxVec3 horizontalForward(forward.x, 0, forward.z);
    if (!horizontalForward.isFinite() || horizontalForward.normalize() <= 1e-6f)
    {
        return result;
    }
    std::vector<PxVec3> points;
    points.reserve(corners.size());
    for (const PxVec3& corner : corners)
    {
        if (!corner.isFinite())
        {
            return result;
        }
        points.emplace_back(corner.x, origin.y, corner.z);
    }
    const std::vector<PxVec3> hull = BuildFootSupportHull(std::move(points));
    if (hull.size() != corners.size())
    {
        return result;
    }
    PxVec3 center(0);
    for (size_t index = 0; index < hull.size(); ++index)
    {
        result.corners[index] = hull[index];
        center += hull[index] - origin;
    }
    result.containsCom = IsInsideFootSupportHull(hull, origin);
    if (result.containsCom || center.normalize() <= 1e-6f)
    {
        // COM이 사각형 안이나 경계에 있으면 유일한 바깥 접선 부채꼴을 만들 수 없다.
        return result;
    }
    float minimumAngle = PX_MAX_F32;
    float maximumAngle = -PX_MAX_F32;
    PxVec3 minimumRay(0);
    PxVec3 maximumRay(0);
    for (const PxVec3& corner : hull)
    {
        PxVec3 ray = corner - origin;
        if (ray.normalize() <= 1e-6f)
        {
            return result;
        }
        // 발 중심 방향을 기준으로 비교해 월드 각도의 -180/180도 경계에서도 같은 끝점을 고른다.
        const float cross = center.x * ray.z - center.z * ray.x;
        const float angle = std::atan2(cross, center.dot(ray));
        if (angle < minimumAngle)
        {
            minimumAngle = angle;
            minimumRay = ray;
        }
        if (angle > maximumAngle)
        {
            maximumAngle = angle;
            maximumRay = ray;
        }
    }
    if (maximumAngle - minimumAngle <= 1e-6f || maximumAngle - minimumAngle >= PxPi)
    {
        return result;
    }
    result.centerRay = minimumRay + maximumRay;
    if (result.centerRay.normalize() <= 1e-6f)
    {
        return result;
    }
    const bool minimumIsFront = minimumRay.dot(horizontalForward) + 1e-6f >= maximumRay.dot(horizontalForward);
    result.frontRay = minimumIsFront ? minimumRay : maximumRay;
    result.backRay = minimumIsFront ? maximumRay : minimumRay;
    result.halfAngleRadians = (maximumAngle - minimumAngle) * .5f;
    result.valid = true;
    return result;
}

bool PARS::IsInsideFootstepLandingRegion(const FootstepLandingRegion& region, const PxVec3& point)
{
    if (!region.valid || !point.isFinite())
    {
        return false;
    }
    const PxVec3 delta(point.x - region.origin.x, 0, point.z - region.origin.z);
    if (!delta.isFinite())
    {
        return false;
    }
    const float cross = region.frontRay.x * region.backRay.z - region.frontRay.z * region.backRay.x;
    const float orientation = cross >= 0 ? 1.0f : -1.0f;
    // 월드 위치에 작은 변위를 다시 더할 때 생기는 오차를 미터 단위 경계 거리로 처리한다.
    const float tolerance = 1e-6f + std::numeric_limits<float>::epsilon() *
        (std::abs(region.origin.x) + std::abs(region.origin.z)) * 2.0f;
    const float frontDistance = orientation * (region.frontRay.x * delta.z - region.frontRay.z * delta.x);
    const float backDistance = orientation * (delta.x * region.backRay.z - delta.z * region.backRay.x);
    // COM 원점과 양쪽 경계는 허용하고 사각형을 향하는 열린 구간만 금지한다.
    return frontDistance > tolerance && backDistance > tolerance;
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
    m_Snapshot.followupFoot = -1;
    m_Snapshot.remainingRecoverySteps = 0;
    m_Snapshot.candidates = {};
    m_Snapshot.loadTransfer = {};
    for (FootstepCommand& command : m_Snapshot.feet)
    {
        command.phase = FootstepPhase::Aborted;
        command.reason = reason;
        command.hasTarget = false;
        command.motion.valid = false;
        command.landingComDirection = FootstepLandingDirection::None;
    }
}

float PARS::GetFootstepRemainingTime(const FootstepCommand& command)
{
    if (command.hasTakenOff)
    {
        return (std::max)(0.0f, command.motion.duration - command.motion.elapsedTime);
    }
    return (std::max)(0.0f, .5f * command.motion.duration - command.motion.takeoffElapsedTime) +
        command.motion.duration;
}

PxVec3 PARS::SampleFootstepPosition(const FootstepCommand& command, float timeAhead)
{
    const FootstepMotion& motion = command.motion;
    PxVec3 takeoff = command.takeoffPosition;
    float elapsed = motion.elapsedTime + timeAhead;
    if (!command.hasTakenOff)
    {
        const float remainingLift = (std::max)(0.0f, .5f * motion.duration - motion.takeoffElapsedTime);
        if (timeAhead <= remainingLift)
        {
            return command.startPosition + PxVec3(0, LiftHeight(motion, motion.takeoffElapsedTime + timeAhead), 0);
        }
        // 아직 확인되지 않은 실제 이륙 시점을 예언하지 않고, 들기 완료 후의 목표 경로를 검사한다.
        takeoff = command.startPosition + PxVec3(0, motion.initialHeight, 0);
        elapsed = timeAhead - remainingLift;
    }
    const float progress = PxClamp(elapsed / motion.duration, 0.0f, 1.0f);
    PxVec3 position = SwingPosition(command, takeoff, progress);
    const float overtime = (std::max)(0.0f, elapsed - motion.duration);
    position.y -= (std::min)(motion.maximumLandingLowering,
        motion.landingLowering + overtime * motion.landingLoweringSpeed);
    return position;
}

void FootstepPlanner::UpdateMotion(FootstepCommand& command, const FootstepFootInput& foot, const FootstepInput& input)
{
    FramePerformance::Scope swingTiming(FramePerformance::Category::SwingTrajectory);
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
        motion.height = LiftHeight(motion, motion.takeoffElapsedTime);
        motion.position = command.startPosition + PxVec3(0, motion.height, 0);
        motion.valid = motion.position.isFinite();
        return;
    }
    motion.height = SwingHeight(motion, motion.progress);
    motion.position = SwingPosition(command, command.takeoffPosition, motion.progress);
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
    state.startCom = {};
    state.candidates = {};
    state.loadTransfer.ready = false;
    if (!settings.recovery.enabled)
    {
        state.followupFoot = -1;
        state.remainingRecoverySteps = 0;
    }
    for (FootstepCommand& command : state.feet)
    {
        command.justStarted = false;
        command.justLanded = false;
        command.hasTarget = false;
        command.motion.valid = false;
        if (!settings.pathAvoidanceEnabled)
        {
            command.target.recovery.pathRejected = false;
        }
        if (!settings.landingComClampEnabled)
        {
            command.landingComDirection = FootstepLandingDirection::None;
        }
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
        state.followupFoot = -1;
        state.remainingRecoverySteps = 0;
        state.reason = !input.active ? FootstepReason::Inactive : !input.automaticSupport ?
            FootstepReason::AutomaticSupportRequired : FootstepReason::IdleRequired;
        if (!input.active && input.automaticSupport && input.idleAnimation && settings.recovery.enabled &&
            input.hasCom && input.hasVelocity && input.feet[0].reach.available &&
            input.feet[1].reach.available && ValidSettings(settings))
        {
            // 양발 대기 중에는 영역만 준비하며, 자세 탐색이나 이동 명령은 실행하지 않는다.
            UpdateGroundPlane(input, state);
            if (!state.hasGroundPlane && input.hull.size() >= 3 && std::isfinite(input.projectionHeight))
            {
                // 키네마틱 Idle의 미세한 바닥 간격은 레이 지면으로 미리 본다. 실제 접촉을 생성하지 않는다.
                state.groundPosition = PxVec3(input.com.x, input.projectionHeight, input.com.z);
                state.groundNormal = PxVec3(0, 1, 0);
                state.hasGroundPlane = true;
            }
            for (size_t side = 0; side < state.candidates.size(); ++side)
            {
                state.candidates[side] = MakeRecoveryTarget(input, settings, state, side, false,
                    MakeTarget(input, settings, state, side));
            }
        }
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
        state.followupFoot = -1;
        state.remainingRecoverySteps = 0;
        state.lastImpactPosition = input.impactPosition;
        state.hasImpact = true;
    }
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
                command.landingComDirection = FootstepLandingDirection::None;
                command.motion.landingError = HorizontalDistance(foot.position, command.target.position);
                command.motion.hasLandingError = std::isfinite(command.motion.landingError);
                command.motion.remainingDistance = command.motion.landingError;
                ++state.landingCount;
                if (command.target.recovery.hasTarget && command.target.recovery.modelFeasible &&
                    command.target.recovery.requiredSteps > 1)
                {
                    state.followupFoot = static_cast<int>(1 - side);
                    state.remainingRecoverySteps = command.target.recovery.requiredSteps - 1;
                }
                else
                {
                    state.followupFoot = -1;
                    state.remainingRecoverySteps = 0;
                }
            }
        }
        state.candidates[side] = MakeTarget(input, settings, state, side);
    }
    state.requestedFoot = SelectRequestedFoot(input, state);
    if (state.requestedFoot < 0 && settings.recovery.enabled &&
        input.feet[0].reach.available && input.feet[1].reach.available)
    {
        float greatestDistance = MinimumStepDistance;
        for (size_t side = 0; side < state.candidates.size(); ++side)
        {
            const PxVec3 preferred = state.candidates[side].landingCom.raw +
                input.feet[side].stanceOffset * settings.stanceScale;
            const float distance = HorizontalDistance(input.feet[side].position, preferred);
            if (state.candidates[side].valid && distance > greatestDistance)
            {
                greatestDistance = distance;
                state.requestedFoot = static_cast<int>(side);
            }
        }
    }
    FootstepLandingDirection previousDirection = FootstepLandingDirection::None;
    if (state.requestedFoot >= 0 && state.requestedFoot < static_cast<int>(state.feet.size()))
    {
        const FootstepCommand& command = state.feet[static_cast<size_t>(state.requestedFoot)];
        if (command.IsInProgress())
        {
            previousDirection = command.landingComDirection;
        }
    }
    // 발을 먼저 고른 뒤 같은 지지발 제한을 시작 예측과 실제 시작 판정에 함께 적용한다.
    state.startCom = PredictCom(input, settings, settings.startPredictionTime, input.projectionHeight,
        state.requestedFoot, previousDirection);
    state.predictedCom = state.startCom.clamped;
    state.hasPrediction = state.startCom.valid;
    const bool validArea = input.zone != FootSupportZone::Invalid && input.hull.size() >= 3;
    state.needsStep = state.hasPrediction && validArea && !IsInsideFootSupportHull(input.hull, state.predictedCom);
    state.reason = !state.hasPrediction ? FootstepReason::InvalidCom : !validArea ? FootstepReason::NoSupportArea :
        state.needsStep ? FootstepReason::ValidTarget : FootstepReason::PredictedInside;
    if (state.requestedFoot >= 0 && state.requestedFoot < 2)
    {
        const size_t side = static_cast<size_t>(state.requestedFoot);
        // 시작 COM 보정 결과를 판단 계층의 입력으로 사용하지 않는다.
        const bool rawNeedsStep = state.startCom.valid && validArea && !IsInsideFootSupportHull(input.hull, state.startCom.raw);
        state.candidates[side] = MakeRecoveryTarget(input, settings, state, side,
            rawNeedsStep || state.needsStep, state.candidates[side]);
        const FootstepRecoverySnapshot& recovery = state.candidates[side].recovery;
        if (rawNeedsStep && recovery.hasTarget)
        {
            state.needsStep = true;
            state.reason = FootstepReason::ValidTarget;
        }
        if (!rawNeedsStep && !state.IsInProgress())
        {
            state.followupFoot = -1;
            state.remainingRecoverySteps = 0;
        }
    }
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
            AssignTarget(command, candidate);
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
        const bool sameMode = candidate.recovery.mode == command.target.recovery.mode;
        const bool switchToCross = !sameMode && candidate.recovery.status == FootstepRecoveryStatus::Feasible &&
            CanReconsiderNormalStep(command, foot);
        if (!command.motion.landingTargetLocked && candidate.valid && (sameMode || switchToCross))
        {
            AssignTarget(command, candidate);
            if (switchToCross)
            {
                command.landingComDirection = FootstepLandingDirection::None;
            }
            else if (candidate.landingCom.enabled && candidate.landingCom.direction != FootstepLandingDirection::None)
            {
                command.landingComDirection = candidate.landingCom.direction;
            }
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

void FootstepPlanner::ReportRecoveryPath(size_t side, const PxVec3& target, bool accepted)
{
    if (side >= m_Snapshot.feet.size())
    {
        return;
    }
    FootstepCommand& command = m_Snapshot.feet[side];
    if (command.IsInProgress() && command.target.recovery.hasTarget &&
        (target - command.target.position).magnitudeSquared() < 1e-8f)
    {
        command.target.recovery.pathRejected = !accepted;
        if (m_Snapshot.candidates[side].recovery.hasTarget &&
            (m_Snapshot.candidates[side].position - target).magnitudeSquared() < 1e-8f)
        {
            m_Snapshot.candidates[side].recovery.pathRejected = !accepted;
        }
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
