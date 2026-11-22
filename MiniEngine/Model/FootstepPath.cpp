#include "FootstepPath.h"
#include <algorithm>
#include <cmath>

using namespace PARS;
using namespace physx;

FootstepPath PARS::MakeFootstepPath(const PxVec3& offset, const PxVec3& velocity,
    const PxVec3& detour, float startTime, float duration)
{
    FootstepPath path;
    if (!offset.isFinite() || !velocity.isFinite() || !detour.isFinite() ||
        !std::isfinite(startTime) || startTime < 0 || !std::isfinite(duration) || duration <= 1e-5f)
    {
        return path;
    }
    const PxVec3 tangent = velocity * duration;
    path.coefficients[0] = offset;
    path.coefficients[1] = tangent;
    path.coefficients[2] = -3.0f * offset - 2.0f * tangent + 16.0f * detour;
    path.coefficients[3] = 2.0f * offset + tangent - 32.0f * detour;
    path.coefficients[4] = 16.0f * detour;
    path.startTime = startTime;
    path.duration = duration;
    path.enabled = true;
    return path;
}

PxVec3 PARS::SampleFootstepPath(const FootstepPath& path, float flightTime)
{
    if (!path.enabled || path.duration <= 0 || flightTime < 0 || flightTime >= path.startTime + path.duration)
    {
        return PxVec3(0);
    }
    const float u = PxClamp((flightTime - path.startTime) / path.duration, 0.0f, 1.0f);
    PxVec3 offset = path.coefficients[4];
    for (int index = 3; index >= 0; --index)
    {
        offset = offset * u + path.coefficients[index];
    }
    return offset;
}

PxVec3 PARS::SampleFootstepPathVelocity(const FootstepPath& path, float flightTime)
{
    if (!path.enabled || path.duration <= 0 || flightTime < path.startTime ||
        flightTime >= path.startTime + path.duration)
    {
        return PxVec3(0);
    }
    const float u = PxClamp((flightTime - path.startTime) / path.duration, 0.0f, 1.0f);
    return (path.coefficients[1] + u * (2.0f * path.coefficients[2] +
        u * (3.0f * path.coefficients[3] + u * 4.0f * path.coefficients[4]))) / path.duration;
}

bool PARS::IsFootstepPathWithinLimits(const FootstepPath& path, float maximumSpeed, float maximumAcceleration)
{
    if (!std::isfinite(maximumSpeed) || maximumSpeed < 0 ||
        !std::isfinite(maximumAcceleration) || maximumAcceleration < 0)
    {
        return false;
    }
    if (!path.enabled || !std::isfinite(path.duration) || path.duration <= 0)
    {
        return !path.enabled;
    }
    // 도함수를 베지어 제어점으로 변환한다. 볼록 포락선 성질로 구간 전체의 속도·가속도를 제한한다.
    const std::array<PxVec3, 4> velocityControls
    {
        path.coefficients[1] / path.duration,
        (path.coefficients[1] + (2.0f / 3.0f) * path.coefficients[2]) / path.duration,
        (path.coefficients[1] + (4.0f / 3.0f) * path.coefficients[2] + path.coefficients[3]) / path.duration,
        (path.coefficients[1] + 2.0f * path.coefficients[2] + 3.0f * path.coefficients[3] +
            4.0f * path.coefficients[4]) / path.duration
    };
    for (const PxVec3& velocity : velocityControls)
    {
        if (!velocity.isFinite() || velocity.magnitude() > maximumSpeed)
        {
            return false;
        }
    }
    const float squareTime = path.duration * path.duration;
    const std::array<PxVec3, 3> accelerationControls
    {
        2.0f * path.coefficients[2] / squareTime,
        (2.0f * path.coefficients[2] + 3.0f * path.coefficients[3]) / squareTime,
        (2.0f * path.coefficients[2] + 6.0f * path.coefficients[3] + 12.0f * path.coefficients[4]) / squareTime
    };
    for (const PxVec3& acceleration : accelerationControls)
    {
        if (!acceleration.isFinite() || acceleration.magnitude() > maximumAcceleration)
        {
            return false;
        }
    }
    return true;
}

float PARS::GetPredictedFootstepFlightTime(const FootstepCommand& command, float timeAhead)
{
    if (command.UsesSwingTrajectory())
    {
        return command.motion.elapsedTime + timeAhead;
    }
    const float remainingLift = (std::max)(0.0f, .5f * command.motion.duration - command.motion.takeoffElapsedTime);
    return timeAhead - remainingLift;
}

PxVec3 PARS::SampleFootstepPosition(const FootstepCommand& command, const FootstepPath& path, float timeAhead)
{
    return SampleFootstepPosition(command, timeAhead) +
        SampleFootstepPath(path, GetPredictedFootstepFlightTime(command, timeAhead));
}
