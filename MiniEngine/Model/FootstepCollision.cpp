#include "FootstepCollisionBounds.h"
#include "geometry/PxGeometryQuery.h"
#include "FramePerformance.h"
#include <algorithm>
#include <cmath>

using namespace PARS;
using namespace physx;

namespace
{
    bool ValidShape(const FootstepCollisionShape& shape)
    {
        if (!shape.available || !shape.inBone.isSane() || !shape.actualPose.isSane())
        {
            return false;
        }
        switch (shape.geometry.getType())
        {
        case PxGeometryType::eCAPSULE:
            return shape.geometry.capsule().isValid();
        case PxGeometryType::eBOX:
            return shape.geometry.box().isValid();
        case PxGeometryType::eSPHERE:
            return shape.geometry.sphere().isValid();
        default:
            return false;
        }
    }

    bool ValidMotion(const FootstepCommand& command, const FootstepPath& path)
    {
        const FootstepMotion& motion = command.motion;
        if (path.enabled)
        {
            if (!std::isfinite(path.startTime) || path.startTime < 0 || !std::isfinite(path.duration) || path.duration <= 0)
            {
                return false;
            }
            for (const PxVec3& coefficient : path.coefficients)
            {
                if (!coefficient.isFinite())
                {
                    return false;
                }
            }
        }
        return command.hasTarget && command.target.valid && motion.valid &&
            command.startPosition.isFinite() && command.takeoffPosition.isFinite() &&
            command.target.position.isFinite() && command.target.normal.isFinite() &&
            command.target.normal.magnitudeSquared() > 1e-8f &&
            std::isfinite(motion.duration) && motion.duration > 0 &&
            (!motion.hasSupportDeadline || (std::isfinite(motion.progress) && motion.progress >= 0 && motion.progress <= 1 &&
                std::isfinite(motion.progressPerSecond) && motion.progressPerSecond >= 0)) &&
            std::isfinite(motion.initialHeight) && motion.initialHeight >= 0 &&
            std::isfinite(motion.liftTimeRatio) && motion.liftTimeRatio >= 1.0f &&
            std::isfinite(motion.elapsedTime) && motion.elapsedTime >= 0 &&
            std::isfinite(motion.takeoffElapsedTime) && motion.takeoffElapsedTime >= 0 &&
            std::isfinite(motion.landingLowering) && motion.landingLowering >= 0 &&
            std::isfinite(motion.landingLoweringSpeed) && motion.landingLoweringSpeed >= 0 &&
            std::isfinite(motion.maximumLandingLowering) && motion.maximumLandingLowering >= 0;
    }

    bool Sample(const FootstepCollisionInput& input, float time, uint32_t pairs,
        FootstepCollisionSample& sample, std::array<std::array<PxTransform, 3>, 2>& shapes,
        FootstepCollisionSnapshot& result)
    {
        sample.timeAhead = time;
        for (size_t side = 0; side < 2; ++side)
        {
            LegIKResult solved;
            if (!PredictFootstepLeg(input.legs[side], time, shapes[side], solved))
            {
                return false;
            }
            sample.solePositions[side] = solved.solvedSole.p;
            sample.reachLimited = sample.reachLimited || solved.reachLimited;
            if (input.legs[side].command.IsInProgress() && input.acceptsPose && !input.acceptsPose(side, solved))
            {
                result.jointLimited = true;
            }
        }
        for (uint32_t left = 0; left < 3; ++left)
        {
            for (uint32_t right = 0; right < 3; ++right)
            {
                const uint32_t pair = uint32_t(1) << (left * 3 + right);
                if ((pairs & pair) == 0)
                {
                    continue;
                }
                ++result.queryCount;
                if (PxGeometryQuery::overlap(input.legs[0].shapes[left].geometry.any(), shapes[0][left],
                    input.legs[1].shapes[right].geometry.any(), shapes[1][right]))
                {
                    sample.overlapPairs |= pair;
                }
            }
        }
        sample.valid = true;
        result.overlapPairs |= sample.overlapPairs;
        result.reachLimited = result.reachLimited || sample.reachLimited;
        return true;
    }

    struct Interval
    {
    public:
        float begin = 0;
        float end = 0;
        uint32_t pairs = 0x1ff;
        uint32_t depth = 0;
    };
}

FootstepCollisionSnapshot PARS::PredictFootstepCollisions(const FootstepCollisionInput& input,
    FootstepCollisionPass pass)
{
    FootstepCollisionSnapshot result;
    result.completedStep = input.completedStep;
    result.revision = input.revision;
    for (size_t side = 0; side < 2; ++side)
    {
        result.moving[side] = input.legs[side].command.IsInProgress();
    }
    if (!result.moving[0] && !result.moving[1])
    {
        return result;
    }
    const bool detour = pass == FootstepCollisionPass::DetourCandidate;
    FramePerformance::Scope samplesTiming(detour ?
        FramePerformance::Category::DetourSamples : FramePerformance::Category::CurrentPathSamples);
    result.status = FootstepCollisionStatus::Unavailable;
    if (!std::isfinite(input.fixedDeltaTime) || input.fixedDeltaTime <= 0 ||
        !std::isfinite(input.clearance) || input.clearance < 0)
    {
        return result;
    }
    float horizon = input.fixedDeltaTime;
    for (size_t side = 0; side < 2; ++side)
    {
        const FootstepCollisionLeg& leg = input.legs[side];
        for (const FootstepCollisionShape& shape : leg.shapes)
        {
            if (!ValidShape(shape))
            {
                return result;
            }
        }
        if (result.moving[side])
        {
            if (!ValidMotion(leg.command, leg.path))
            {
                return result;
            }
            result.assumesTakeoff = result.assumesTakeoff || !leg.command.hasTakenOff;
            horizon = (std::max)(horizon, GetFootstepRemainingTime(leg.command));
        }
    }
    std::array<std::array<PxTransform, 3>, 2> shapes;
    FootstepCollisionSample current;
    if (!Sample(input, 0, 0x1ff, current, shapes, result))
    {
        return result;
    }
    result.fixedPoseBlocked = current.overlapPairs != 0;
    std::array<std::array<float, 3>, 2> margins{};
    for (size_t side = 0; side < 2; ++side)
    {
        for (size_t part = 0; part < 3; ++part)
        {
            float tracking = 0;
            if (result.moving[side] && input.includeTrackingError)
            {
                const FootstepCollisionShape& shape = input.legs[side].shapes[part];
                const PxTransform& target = shapes[side][part];
                const float dot = PxClamp(PxAbs(target.q.getNormalized().dot(shape.actualPose.q.getNormalized())), 0.0f, 1.0f);
                tracking = (target.p - shape.actualPose.p).magnitude() +
                    2 * FootstepShapeRadius(shape) * std::sqrt(PxMax(0.0f, 1 - dot * dot));
            }
            result.maximumTrackingError = PxMax(result.maximumTrackingError, tracking);
            // 추종 오차는 현재 관측 범위다. 미래 외력의 상한으로 간주하지 않고 매 물리 틱 갱신한다.
            margins[side][part] = input.clearance * .5f + tracking + 1e-5f;
        }
    }
    // 여섯 자세는 표시와 빠른 거절에 사용한다. 이 샘플들만으로 통과시키지 않는다.
    for (uint32_t index = 0; index < FootstepCollisionSampleCount; ++index)
    {
        const float fraction = static_cast<float>(index) / (FootstepCollisionSampleCount - 1);
        const float time = input.fixedDeltaTime + (horizon - input.fixedDeltaTime) * fraction;
        if (!Sample(input, time, 0x1ff, result.samples[index], shapes, result))
        {
            return result;
        }
        ++result.sampleCount;
    }
    result.fixedPoseBlocked = result.fixedPoseBlocked || result.samples.back().overlapPairs != 0;
    if (result.overlapPairs != 0)
    {
        result.status = FootstepCollisionStatus::Collision;
        return result;
    }
    if (input.rejectInvalidPoses && (result.reachLimited || result.jointLimited))
    {
        result.status = FootstepCollisionStatus::Uncertain;
        result.uncertainPairs = 0x1ff;
        return result;
    }
    // 중간 자세 검사 시간을 빼면 경계 계산, 도형 분리 판정, 구간 분할에 쓴 시간이 남는다.
    FramePerformance::Scope intervalsTiming(detour ?
        FramePerformance::Category::DetourIntervals : FramePerformance::Category::CurrentPathIntervals);
    std::array<Interval, 20> stack;
    stack[0].end = horizon;
    uint32_t count = 1;
    while (count > 0)
    {
        const Interval interval = stack[--count];
        if (result.intervalCount >= input.maximumIntervals)
        {
            result.budgetExhausted = true;
            result.uncertainPairs |= interval.pairs;
            for (uint32_t pending = 0; pending < count; ++pending)
            {
                result.uncertainPairs |= stack[pending].pairs;
            }
            result.firstRiskTime = interval.begin;
            break;
        }
        ++result.intervalCount;
        const float middle = interval.begin + (interval.end - interval.begin) * .5f;
        FootstepCollisionSample sample;
        {
            // 중간 자세 하나의 IK, 관절 제한, 형상 겹침 질의를 묶어 측정한다.
            FramePerformance::Scope midpointTiming(detour ?
                FramePerformance::Category::DetourMidpoints : FramePerformance::Category::CurrentPathMidpoints);
            if (!Sample(input, middle, interval.pairs, sample, shapes, result))
            {
                return result;
            }
        }
        if (sample.overlapPairs != 0)
        {
            result.firstRiskTime = middle;
            result.status = FootstepCollisionStatus::Collision;
            return result;
        }
        if (input.rejectInvalidPoses && (result.reachLimited || result.jointLimited))
        {
            result.status = FootstepCollisionStatus::Uncertain;
            result.uncertainPairs = interval.pairs;
            return result;
        }
        std::array<std::array<FootstepInterval::Transform, 3>, 2> bounds;
        const bool bounded = BoundFootstepLeg(input.legs[0], interval.begin, interval.end, bounds[0]) &&
            BoundFootstepLeg(input.legs[1], interval.begin, interval.end, bounds[1]);
        uint32_t unresolved = interval.pairs;
        if (bounded)
        {
            for (uint32_t left = 0; left < 3; ++left)
            {
                for (uint32_t right = 0; right < 3; ++right)
                {
                    const uint32_t pair = uint32_t(1) << (left * 3 + right);
                    if ((unresolved & pair) != 0 && SeparateFootstepShapes(
                        input.legs[0].shapes[left], bounds[0][left], shapes[0][left], margins[0][left],
                        input.legs[1].shapes[right], bounds[1][right], shapes[1][right], margins[1][right]))
                    {
                        unresolved &= ~pair;
                    }
                }
            }
        }
        if (unresolved == 0)
        {
            continue;
        }
        if (interval.depth >= 16 || interval.end - interval.begin <= 1e-6f)
        {
            result.uncertainPairs |= unresolved;
            result.firstRiskTime = interval.begin;
            break;
        }
        Interval later;
        later.begin = middle;
        later.end = interval.end;
        later.pairs = unresolved;
        later.depth = interval.depth + 1;
        Interval earlier = later;
        earlier.begin = interval.begin;
        earlier.end = middle;
        stack[count++] = later;
        stack[count++] = earlier;
    }
    result.status = result.uncertainPairs == 0 ? FootstepCollisionStatus::Clear : FootstepCollisionStatus::Uncertain;
    return result;
}
