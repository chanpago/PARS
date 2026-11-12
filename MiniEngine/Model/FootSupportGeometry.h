#pragma once

#include "PhysicsAsset.h"
#include <foundation/PxTransform.h>
#include <array>
#include <cmath>

namespace PARS
{
    // 바디 형상의 지지점을 발바닥 평면으로 투영한다. 이 윤곽은 실제 접촉 판정과 별개다.
    inline std::array<physx::PxVec3, 8> BuildFootBodyOutline(const PhysicsBody& body,
        const physx::PxTransform& bodyWorld, const physx::PxTransform& soleWorld)
    {
        std::array<physx::PxVec3, 8> outline{};
        const physx::PxVec3 up = soleWorld.q.rotate(physx::PxVec3(0, 1, 0));
        for (size_t point = 0; point < outline.size(); ++point)
        {
            const float angle = physx::PxTwoPi * static_cast<float>(point) / static_cast<float>(outline.size());
            const physx::PxVec3 direction = soleWorld.q.rotate(physx::PxVec3(std::cos(angle), 0, std::sin(angle)));
            const physx::PxVec3 localDirection = bodyWorld.q.rotateInv(direction);
            physx::PxVec3 localPoint(0);
            if (body.shape == PhysicsShapeType::Box)
            {
                localPoint = physx::PxVec3(localDirection.x >= 0 ? body.halfExtents.x : -body.halfExtents.x,
                    localDirection.y >= 0 ? body.halfExtents.y : -body.halfExtents.y,
                    localDirection.z >= 0 ? body.halfExtents.z : -body.halfExtents.z);
            }
            else
            {
                localPoint = localDirection * body.radius;
                if (body.shape == PhysicsShapeType::Capsule && std::abs(localDirection.x) > 1e-6f)
                {
                    // 축에 수직인 방향은 원통 중앙을 쓴다. 반올림 오차로 양 끝을 오가지 않게 한다.
                    localPoint.x += localDirection.x >= 0 ? body.halfLength : -body.halfLength;
                }
            }
            const physx::PxVec3 offset = bodyWorld.q.rotate(localPoint);
            outline[point] = soleWorld.p + offset - up * up.dot(offset);
        }
        return outline;
    }

    // 앞쪽 접촉 기준을 유지하며 현재 Foot/Toe 상대 굽힘으로 발목 목표를 복원한다.
    inline physx::PxTransform MakeToeSupportedFootTarget(const physx::PxTransform& foot,
        const physx::PxTransform& toe, const physx::PxTransform& supportedToe)
    {
        return supportedToe * toe.getInverse() * foot;
    }
}
