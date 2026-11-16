#include "FootstepCollision.h"
#include "geometry/PxGeometryQuery.h"
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

    bool ValidMotion(const FootstepCommand& command)
    {
        const FootstepMotion& motion = command.motion;
        return command.hasTarget && command.target.valid && motion.valid &&
            command.startPosition.isFinite() && command.takeoffPosition.isFinite() &&
            command.target.position.isFinite() && command.target.normal.isFinite() &&
            command.target.normal.magnitudeSquared() > 1e-8f &&
            std::isfinite(motion.duration) && motion.duration > 0 &&
            std::isfinite(motion.initialHeight) && motion.initialHeight >= 0 &&
            std::isfinite(motion.elapsedTime) && motion.elapsedTime >= 0 &&
            std::isfinite(motion.takeoffElapsedTime) && motion.takeoffElapsedTime >= 0 &&
            std::isfinite(motion.landingLowering) && motion.landingLowering >= 0 &&
            std::isfinite(motion.landingLoweringSpeed) && motion.landingLoweringSpeed >= 0 &&
            std::isfinite(motion.maximumLandingLowering) && motion.maximumLandingLowering >= 0;
    }

    bool PredictLeg(const FootstepCollisionLeg& leg, float timeAhead,
        std::array<PxTransform, 3>& shapes, PxVec3& solePosition, bool& reachLimited)
    {
        if (!leg.command.IsInProgress())
        {
            // 지지 다리는 현재 실제 충돌체 위치를 쓴다. 골반과 지지 다리의 미래 운동은 다음 틱에 갱신한다.
            for (size_t part = 0; part < shapes.size(); ++part)
            {
                shapes[part] = leg.shapes[part].actualPose;
            }
            return true;
        }

        TwoBoneIKInput pose = leg.pose;
        const PxVec3 targetPosition = SampleFootstepPosition(leg.command, timeAhead);
        if (!MakeFootSoleTarget(targetPosition, leg.command.target.normal, leg.pose.targetSole, pose.targetSole))
        {
            return false;
        }
        const LegIKResult solved = SolveTwoBoneIK(pose);
        if (!solved.valid)
        {
            return false;
        }
        const std::array<PxTransform, 3> bones{solved.hip, solved.knee, solved.ankle};
        for (size_t part = 0; part < shapes.size(); ++part)
        {
            shapes[part] = bones[part] * leg.shapes[part].inBone;
        }
        solePosition = solved.solvedSole.p;
        reachLimited = reachLimited || solved.reachLimited;
        return true;
    }
}

FootstepCollisionSnapshot PARS::PredictFootstepCollisions(const FootstepCollisionInput& input)
{
    FootstepCollisionSnapshot result;
    result.completedStep = input.completedStep;
    result.revision = input.revision;
    for (size_t side = 0; side < input.legs.size(); ++side)
    {
        result.moving[side] = input.legs[side].command.IsInProgress();
    }
    if (!result.moving[0] && !result.moving[1])
    {
        return result;
    }
    result.status = FootstepCollisionStatus::Unavailable;
    if (!std::isfinite(input.fixedDeltaTime) || input.fixedDeltaTime <= 0)
    {
        return result;
    }

    float horizon = input.fixedDeltaTime;
    for (size_t side = 0; side < input.legs.size(); ++side)
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
            if (!ValidMotion(leg.command))
            {
                return result;
            }
            result.assumesTakeoff = result.assumesTakeoff || !leg.command.hasTakenOff;
            horizon = (std::max)(horizon, GetFootstepRemainingTime(leg.command));
        }
    }

    for (uint32_t index = 0; index < FootstepCollisionSampleCount; ++index)
    {
        FootstepCollisionSample& sample = result.samples[index];
        const float fraction = static_cast<float>(index) / (FootstepCollisionSampleCount - 1);
        sample.timeAhead = input.fixedDeltaTime + (horizon - input.fixedDeltaTime) * fraction;
        std::array<std::array<PxTransform, 3>, 2> shapes;
        for (size_t side = 0; side < input.legs.size(); ++side)
        {
            if (!PredictLeg(input.legs[side], sample.timeAhead, shapes[side],
                sample.solePositions[side], sample.reachLimited))
            {
                return result;
            }
        }
        // 양발이 이동하는 경우도 같은 미래 시점끼리 비교하므로 자세당 최대 아홉 쌍이다.
        for (uint32_t left = 0; left < 3; ++left)
        {
            for (uint32_t right = 0; right < 3; ++right)
            {
                ++result.queryCount;
                if (PxGeometryQuery::overlap(input.legs[0].shapes[left].geometry.any(), shapes[0][left],
                    input.legs[1].shapes[right].geometry.any(), shapes[1][right]))
                {
                    sample.overlapPairs |= uint32_t(1) << (left * 3 + right);
                }
            }
        }
        sample.valid = true;
        ++result.sampleCount;
        result.overlapPairs |= sample.overlapPairs;
        result.reachLimited = result.reachLimited || sample.reachLimited;
    }
    result.status = FootstepCollisionStatus::Sampled;
    return result;
}
