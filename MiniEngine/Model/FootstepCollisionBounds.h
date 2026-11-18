#pragma once

#include "FootstepCollision.h"
#include "FootstepInterval.h"

namespace PARS
{
    // 양 끝 자세 사이를 보간하지 않고, 실제 발 궤적과 IK 계산에 시간 구간을 대입한다.
    bool BoundFootstepLeg(const FootstepCollisionLeg& leg, float begin, float end,
        std::array<FootstepInterval::Transform, 3>& shapes);
    bool PredictFootstepLeg(const FootstepCollisionLeg& leg, float timeAhead,
        std::array<physx::PxTransform, 3>& shapes, LegIKResult& result);
    bool SeparateFootstepShapes(const FootstepCollisionShape& first, const FootstepInterval::Transform& firstBounds,
        const physx::PxTransform& firstMiddle, float firstMargin,
        const FootstepCollisionShape& second, const FootstepInterval::Transform& secondBounds,
        const physx::PxTransform& secondMiddle, float secondMargin);
    float FootstepShapeRadius(const FootstepCollisionShape& shape);
}
