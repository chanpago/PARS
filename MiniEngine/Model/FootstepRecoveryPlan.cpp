#include "FootstepRecoveryPlan.h"

using namespace PARS;
using namespace physx;

namespace
{
    constexpr float Gravity = 9.81f;
    constexpr float GeometryTolerance = 1e-4f;

    struct Pendulum
    {
    public:
        PxVec3 position{0};
        PxVec3 velocity{0};
    };

    PxVec3 Horizontal(PxVec3 value)
    {
        value.y = 0;
        return value;
    }

    Pendulum Predict(const Pendulum& state, const PxVec3& support, float omega, float time)
    {
        const float c = std::cosh(omega * time);
        const float s = std::sinh(omega * time);
        const PxVec3 offset = Horizontal(state.position - support);
        Pendulum result;
        result.position = state.position + offset * (c - 1) + Horizontal(state.velocity) * (s / omega);
        result.velocity = offset * (omega * s) + Horizontal(state.velocity) * c;
        return result;
    }

    float CaptureDistance(const PxVec3& capture, const PxVec3& a, const PxVec3& b, float radius)
    {
        const PxVec3 segment = Horizontal(b - a);
        const float fraction = segment.magnitudeSquared() > 1e-10f
            ? PxClamp(Horizontal(capture - a).dot(segment) / segment.magnitudeSquared(), 0.0f, 1.0f) : 0;
        return PxMax(0.0f, Horizontal(capture - a - segment * fraction).magnitude() - radius);
    }

    bool ValidLeg(const FootstepFootInput& foot)
    {
        for (const PxVec3& offset : foot.supportOffsets)
        {
            if (!offset.isFinite())
            {
                return false;
            }
        }
        return foot.available && foot.hasLegGeometry && foot.position.isFinite() &&
            foot.hipPosition.isFinite() && foot.soleToAnkle.isFinite() && foot.stanceOffset.isFinite() &&
            std::isfinite(foot.thighLength) && foot.thighLength > 0 &&
            std::isfinite(foot.calfLength) && foot.calfLength > 0 &&
            std::isfinite(foot.supportRadius) && foot.supportRadius > 0;
    }

    float CaptureFootDistance(const PxVec3& capture, const FootstepFootInput& first,
        const FootstepFootInput& second, const PxVec3& firstPosition, const PxVec3& secondPosition)
    {
        std::vector<PxVec3> points;
        points.reserve(8);
        for (const PxVec3& offset : first.supportOffsets)
        {
            points.push_back(firstPosition + offset);
        }
        for (const PxVec3& offset : second.supportOffsets)
        {
            points.push_back(secondPosition + offset);
        }
        const std::vector<PxVec3> hull = BuildFootSupportHull(std::move(points));
        if (hull.size() < 3)
        {
            return CaptureDistance(capture, firstPosition, secondPosition, PxMin(first.supportRadius, second.supportRadius));
        }
        if (IsInsideFootSupportHull(hull, capture))
        {
            return 0;
        }
        float distance = (std::numeric_limits<float>::max)();
        for (size_t index = 0; index < hull.size(); ++index)
        {
            distance = PxMin(distance, CaptureDistance(capture, hull[index], hull[(index + 1) % hull.size()], 0));
        }
        return distance;
    }

    PxVec3 SupportPivot(const Pendulum& state, const FootstepFootInput& foot, float omega)
    {
        const PxVec3 capture = state.position + state.velocity / omega;
        std::vector<PxVec3> points;
        for (const PxVec3& offset : foot.supportOffsets)
        {
            points.push_back(foot.position + offset);
        }
        const std::vector<PxVec3> hull = BuildFootSupportHull(std::move(points));
        if (hull.size() < 3)
        {
            return foot.contactPosition;
        }
        if (IsInsideFootSupportHull(hull, capture))
        {
            return PxVec3(capture.x, foot.contactPosition.y, capture.z);
        }
        PxVec3 closest = foot.contactPosition;
        float distance = (std::numeric_limits<float>::max)();
        for (size_t index = 0; index < hull.size(); ++index)
        {
            const PxVec3 a = hull[index];
            const PxVec3 edge = Horizontal(hull[(index + 1) % hull.size()] - a);
            if (edge.magnitudeSquared() <= 1e-10f)
            {
                continue;
            }
            const float fraction = PxClamp(Horizontal(capture - a).dot(edge) / edge.magnitudeSquared(), 0.0f, 1.0f);
            const PxVec3 point = a + edge * fraction;
            const float error = Horizontal(capture - point).magnitudeSquared();
            if (error < distance)
            {
                distance = error;
                closest = point;
            }
        }
        return closest;
    }

    float TravelLimit(const FootstepSettings& settings, float duration)
    {
        // 고정된 시작점·끝점의 삼차 smoothstep을 시간으로 미분한 최대값이다.
        return PxMin(settings.maximumFootSpeed * duration / 1.5f,
            settings.maximumFootAcceleration * duration * duration / 6.0f);
    }

    float Lift(const FootstepFootInput& foot, const FootstepSettings& settings, float distance)
    {
        return PxMin(PxMax(distance * settings.liftHeightScale, settings.minimumLiftHeight), foot.calfLength);
    }

    bool MotionFits(const FootstepFootInput& foot, const FootstepSettings& settings,
        const PxVec3& target, float duration)
    {
        const float distance = Horizontal(target - foot.position).magnitude();
        const float height = Lift(foot, settings, distance);
        const float vertical = std::abs(target.y - foot.position.y);
        // 높이 차이의 smoothstep과 4차 들기 곡선의 상한을 더한 보수적인 검사다.
        return distance <= TravelLimit(settings, duration) + GeometryTolerance &&
            height >= settings.minimumLiftHeight &&
            (1.5f * vertical + 3.079202f * height) / duration <= settings.maximumLiftSpeed &&
            (6.0f * vertical + 32.0f * height) / (duration * duration) <= settings.maximumLiftAcceleration;
    }

    PxVec3 OnPlane(PxVec3 point, const PxVec3& origin, const PxVec3& normal)
    {
        point.y -= normal.dot(point - origin) / normal.y;
        return point;
    }

    bool ProjectReach(PxVec3& point, const PxVec3& center, float radius,
        const PxVec3& origin, const PxVec3& normal)
    {
        const float planeDistance = normal.dot(center - origin);
        if (std::abs(planeDistance) >= radius)
        {
            return false;
        }
        const PxVec3 diskCenter = center - normal * planeDistance;
        const float diskRadius = std::sqrt(radius * radius - planeDistance * planeDistance);
        const PxVec3 delta = point - diskCenter;
        const float distance = delta.magnitude();
        if (distance > diskRadius)
        {
            point = diskCenter + delta * (diskRadius / distance);
        }
        return true;
    }

    bool FitTarget(PxVec3& point, const FootstepFootInput& foot, const FootstepFootInput& stance,
        const FootstepSettings& settings, const PxVec3& right, const PxVec3& normal,
        const PxVec3& hipDisplacement, float duration)
    {
        const PxVec3 origin = stance.contactPosition;
        const float length = foot.thighLength + foot.calfLength;
        const float maximumReach = length * .999f;
        const float travel = TravelLimit(settings, duration);
        PxVec3 outward = Horizontal(right);
        if (outward.normalize() < 1e-6f)
        {
            return false;
        }
        const float side = (foot.stanceOffset - stance.stanceOffset).dot(outward);
        outward *= side < 0 ? -1.0f : 1.0f;
        const PxVec3 ankleCenter = foot.hipPosition + hipDisplacement - foot.soleToAnkle;
        point = OnPlane(point, origin, normal);
        // 여러 제약의 교집합으로 반복 투영한다. 마지막에 모든 제약을 다시 검사한다.
        for (unsigned iteration = 0; iteration < 12; ++iteration)
        {
            const PxVec3 swing = Horizontal(point - foot.position);
            const float distance = swing.magnitude();
            if (distance > travel)
            {
                point = OnPlane(foot.position + swing * (travel / distance), origin, normal);
            }
            if (!ProjectReach(point, stance.position, settings.maximumStepLength, origin, normal) ||
                !ProjectReach(point, ankleCenter, maximumReach, origin, normal))
            {
                return false;
            }
            if (settings.preventCrossing)
            {
                const float lateral = Horizontal(point - stance.position).dot(outward);
                if (lateral < settings.crossingOffset)
                {
                    point = OnPlane(point + outward * (settings.crossingOffset - lateral), origin, normal);
                }
            }
        }
        const float ankleDistance = (point - ankleCenter).magnitude();
        return point.isFinite() && ankleDistance <= maximumReach + GeometryTolerance &&
            ankleDistance >= std::abs(foot.thighLength - foot.calfLength) + GeometryTolerance &&
            (point - stance.position).magnitude() <= settings.maximumStepLength + GeometryTolerance &&
            (!settings.preventCrossing || Horizontal(point - stance.position).dot(outward) >= settings.crossingOffset - GeometryTolerance) &&
            MotionFits(foot, settings, point, duration);
    }

    bool ReachAlongSwing(const FootstepFootInput& foot, const FootstepFootInput& stance,
        const FootstepSettings& settings, const Pendulum& initial, const PxVec3& pivot,
        const PxVec3& target, float omega, float duration)
    {
        const float height = Lift(foot, settings, Horizontal(target - foot.position).magnitude());
        // 궤적의 8개 시점에서 양쪽 다리의 기하학적 도달 범위를 검사한다.
        // 관절 각도 제한과 실제 토크 추종은 기존 IK·PD에서 별도로 적용한다.
        for (unsigned sample = 1; sample <= 8; ++sample)
        {
            const float t = static_cast<float>(sample) / 8;
            const float blend = t * t * (3 - 2 * t);
            const float arc = t * (1 - t);
            const PxVec3 sole = foot.position + (target - foot.position) * blend + PxVec3(0, height * 16 * arc * arc, 0);
            const PxVec3 displacement = Predict(initial, pivot, omega, duration * t).position - initial.position;
            const float movingReach = (sole + foot.soleToAnkle - foot.hipPosition - displacement).magnitude();
            const float supportReach = (stance.position + stance.soleToAnkle - stance.hipPosition - displacement).magnitude();
            if (movingReach > foot.thighLength + foot.calfLength + GeometryTolerance ||
                movingReach < std::abs(foot.thighLength - foot.calfLength) + GeometryTolerance ||
                supportReach > stance.thighLength + stance.calfLength + GeometryTolerance)
            {
                return false;
            }
        }
        return true;
    }
}

float PARS::FootstepCaptureError(const FootstepInput& input)
{
    const float height = input.com.y - input.projectionHeight;
    if (!input.com.isFinite() || !input.velocity.isFinite() || !std::isfinite(height) || height <= .05f)
    {
        return (std::numeric_limits<float>::infinity)();
    }
    const float omega = std::sqrt(Gravity / height);
    const PxVec3 capture = input.com + Horizontal(input.velocity) / omega;
    const FootstepFootInput& left = input.feet[0];
    const FootstepFootInput& right = input.feet[1];
    if (left.grounded && left.hasContactSample && right.grounded && right.hasContactSample)
    {
        // 현재 양발이 접지했다면 이미 측정한 실제 사각형의 지지 영역을 사용한다.
        // 좁은 내접 원만 쓰면 발뒤꿈치·발끝 위의 정지 상태도 불안정으로 오판한다.
        if (input.hull.size() >= 3)
        {
            if (IsInsideFootSupportHull(input.hull, capture))
            {
                return 0;
            }
            float distance = (std::numeric_limits<float>::max)();
            for (size_t index = 0; index < input.hull.size(); ++index)
            {
                distance = PxMin(distance, CaptureDistance(capture, input.hull[index],
                    input.hull[(index + 1) % input.hull.size()], 0));
            }
            return distance;
        }
        return CaptureDistance(capture, left.position, right.position, PxMin(left.supportRadius, right.supportRadius));
    }
    const FootstepFootInput& support = left.grounded && left.hasContactSample ? left : right;
    return CaptureDistance(capture, support.position, support.position, support.supportRadius);
}

FootstepTarget PARS::PlanFootstepRecovery(const FootstepInput& input, const FootstepSettings& settings, size_t side)
{
    FootstepTarget best;
    best.reason = FootstepReason::NoReachablePlan;
    if (side >= input.feet.size())
    {
        return best;
    }
    if (!std::isfinite(input.fixedDeltaTime) || input.fixedDeltaTime < .001f ||
        !std::isfinite(settings.minimumStepDuration) || settings.minimumStepDuration <= 0 ||
        !std::isfinite(settings.maximumStepDuration) || settings.maximumStepDuration < settings.minimumStepDuration ||
        settings.maximumStepDuration / input.fixedDeltaTime > 512)
    {
        best.reason = FootstepReason::InvalidSettings;
        return best;
    }
    const FootstepFootInput& foot = input.feet[side];
    const FootstepFootInput& stance = input.feet[1 - side];
    if (!input.hasReference)
    {
        best.reason = FootstepReason::NoReference;
        return best;
    }
    if (!ValidLeg(foot) || !ValidLeg(stance))
    {
        best.reason = FootstepReason::InvalidLegGeometry;
        return best;
    }
    if (!stance.hasContactSample || !foot.hasContactSample)
    {
        best.reason = FootstepReason::NoContactSample;
        return best;
    }
    PxVec3 normal = stance.normal;
    if (!stance.grounded || !stance.contactPosition.isFinite() || !normal.isFinite() ||
        normal.normalize() < 1e-6f || normal.y < .70710678f)
    {
        best.reason = FootstepReason::NoStanceContact;
        return best;
    }
    const float height = input.com.y - input.projectionHeight;
    if (!std::isfinite(height) || height <= .05f || !input.com.isFinite() || !input.velocity.isFinite())
    {
        best.reason = FootstepReason::InvalidCom;
        return best;
    }
    const float omega = std::sqrt(Gravity / height);
    const float noStepError = FootstepCaptureError(input);
    if (noStepError <= settings.minimumRecoveryImprovement)
    {
        best.reason = FootstepReason::CaptureStable;
        return best;
    }
    const float dt = input.fixedDeltaTime;
    const unsigned firstTick = static_cast<unsigned>(std::ceil(settings.minimumStepDuration / dt - 1e-5f));
    const unsigned lastTick = static_cast<unsigned>(std::floor(settings.maximumStepDuration / dt + 1e-5f));
    const Pendulum initial{input.com, Horizontal(input.velocity)};
    // 모델은 발바닥 안에서 압력중심을 옮길 수 있다고 가정한다. 실제 토크 제어의 보장은 아니다.
    const PxVec3 pivot = SupportPivot(initial, stance, omega);
    float bestPreference = (std::numeric_limits<float>::max)();
    float bestImprovementRate = 0;
    uint32_t feasible = 0;
    for (unsigned tick = firstTick; tick <= lastTick; ++tick)
    {
        const float duration = tick * dt;
        if (duration > input.remainingSupportTime + GeometryTolerance)
        {
            continue;
        }
        const Pendulum landing = Predict(initial, pivot, omega, duration);
        // 실제 하중 확인까지 기존 지지점이 유지된다고 가정해 즉시 하중 전환을 낙관하지 않는다.
        const float transferTime = duration + settings.supportConfirmationTime;
        const Pendulum transfer = Predict(initial, pivot, omega, transferTime);
        const PxVec3 capture = transfer.position + transfer.velocity / omega;
        const PxVec3 preferred = landing.position + foot.stanceOffset * settings.stanceScale;
        for (unsigned seed = 0; seed < 35; ++seed)
        {
            PxVec3 target = capture;
            if (seed == 1)
            {
                target = preferred;
            }
            else if (seed == 2)
            {
                target = foot.position;
            }
            else if (seed >= 3)
            {
                const float angle = static_cast<float>((seed - 3) % 16) * 6.28318530718f / 16;
                const float scale = seed < 19 ? 1.0f : .5f;
                target = foot.position + PxVec3(std::cos(angle), 0, std::sin(angle)) * (TravelLimit(settings, duration) * scale);
            }
            if (!FitTarget(target, foot, stance, settings, input.supportRight, normal,
                    landing.position - initial.position, duration) ||
                !ReachAlongSwing(foot, stance, settings, initial, pivot, target, omega, duration))
            {
                continue;
            }
            ++feasible;
            const float oneStepError = CaptureFootDistance(capture, stance, foot, stance.position, target) * std::exp(-omega * transferTime);
            float error = oneStepError;
            float nextDuration = 0;
            PxVec3 nextTarget(0);
            // 현재 발을 새 지지발로 삼고 반대 발을 한 번 더 옮겼을 때의 잔여 캡처 오차를 비교한다.
            FootstepFootInput nextFoot = stance;
            FootstepFootInput nextSupport = foot;
            nextFoot.hipPosition += transfer.position - initial.position;
            nextSupport.hipPosition += transfer.position - initial.position;
            nextSupport.position = target;
            nextSupport.contactPosition = target;
            const PxVec3 nextPivot = SupportPivot(transfer, nextSupport, omega);
            for (unsigned nextTick = firstTick; nextTick <= lastTick && error > 1e-5f; ++nextTick)
            {
                const float time = nextTick * dt;
                if (transferTime + time > input.remainingSupportTime + GeometryTolerance)
                {
                    continue;
                }
                const Pendulum nextLanding = Predict(transfer, nextPivot, omega, time);
                const float endTime = time + settings.supportConfirmationTime;
                const Pendulum end = Predict(transfer, nextPivot, omega, endTime);
                const PxVec3 nextCapture = end.position + end.velocity / omega;
                PxVec3 next = nextCapture;
                if (!FitTarget(next, nextFoot, nextSupport, settings, input.supportRight, normal,
                        nextLanding.position - transfer.position, time) ||
                    !ReachAlongSwing(nextFoot, nextSupport, settings, transfer, nextPivot, next, omega, time))
                {
                    continue;
                }
                // 서로 다른 착지 시각의 오차를 현재 시각으로 환산해 같은 기준으로 비교한다.
                const float remaining = CaptureFootDistance(nextCapture, nextSupport, nextFoot, target, next) * std::exp(-omega * (transferTime + endTime));
                if (remaining < error)
                {
                    error = remaining;
                    nextDuration = time;
                    nextTarget = next;
                }
            }
            const float preference = duration + nextDuration + .1f * Horizontal(target - preferred).magnitude();
            // 후속 발은 실제 착지 뒤 다시 계획한다. 회복 오차 감소량을 첫 지지 확보 시간으로 나눠
            // 먼 미래의 작은 오차만 선호하면서 첫 착지를 계속 늦추는 후보를 피한다.
            const float improvementRate = (noStepError - error) / transferTime;
            if (!best.valid || improvementRate > bestImprovementRate + 1e-5f ||
                (std::abs(improvementRate - bestImprovementRate) <= 1e-5f && preference < bestPreference))
            {
                best.valid = true;
                best.reason = FootstepReason::ValidTarget;
                best.raw = preferred;
                best.position = target;
                best.normal = normal;
                best.landingCom = landing.position;
                best.landingVelocity = landing.velocity;
                best.capturePoint = capture;
                best.duration = duration;
                best.travelDistance = Horizontal(target - foot.position).magnitude();
                best.stanceDistance = (target - stance.position).magnitude();
                best.liftHeight = Lift(foot, settings, best.travelDistance);
                best.noStepError = noStepError;
                best.oneStepError = oneStepError;
                best.recoveryRate = improvementRate;
                best.recoveryError = error;
                best.nextTarget = nextTarget;
                best.nextDuration = nextDuration;
                best.usesNextStep = nextDuration > 0;
                best.modelCapturable = error <= 1e-5f;
                best.lengthLimited = Horizontal(target - preferred).magnitude() > GeometryTolerance;
                bestPreference = preference;
                bestImprovementRate = improvementRate;
            }
        }
    }
    best.feasibleCandidates = feasible;
    if (best.valid && (best.travelDistance <= GeometryTolerance ||
        noStepError - best.recoveryError < settings.minimumRecoveryImprovement))
    {
        best.valid = false;
        best.reason = FootstepReason::NoRecoveryImprovement;
    }
    return best;
}
