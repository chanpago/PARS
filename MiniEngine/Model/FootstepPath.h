#pragma once

#include "FootstepPlanner.h"

namespace PARS
{
    // 기존 발 궤적에 더하는 보정이다. 시작 위치·속도를 이어받고 착지 시 보정과 보정 속도는 0이다.
    struct FootstepPath
    {
    public:
        std::array<physx::PxVec3, 5> coefficients{
            physx::PxVec3(0), physx::PxVec3(0), physx::PxVec3(0), physx::PxVec3(0), physx::PxVec3(0)};
        float startTime = 0;
        float duration = 0;
        bool enabled = false;
    };

    FootstepPath MakeFootstepPath(const physx::PxVec3& offset, const physx::PxVec3& velocity,
        const physx::PxVec3& detour, float startTime, float duration);
    physx::PxVec3 SampleFootstepPath(const FootstepPath& path, float flightTime);
    physx::PxVec3 SampleFootstepPathVelocity(const FootstepPath& path, float flightTime);
    bool IsFootstepPathWithinLimits(const FootstepPath& path, float maximumSpeed, float maximumAcceleration);
    float GetPredictedFootstepFlightTime(const FootstepCommand& command, float timeAhead);
    physx::PxVec3 SampleFootstepPosition(const FootstepCommand& command, const FootstepPath& path, float timeAhead);
}
