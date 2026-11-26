#pragma once

#include "FootstepRecovery.h"
#include "PhysicsPose.h"
#include "foundation/PxTransform.h"
#include <array>
#include <cmath>

namespace PARS
{
    inline Math::Matrix4 FootstepPoseMatrix(const physx::PxTransform& frame)
    {
        return Math::Matrix4(Math::OrthogonalTransform(
            Math::Quaternion(DirectX::XMVectorSet(frame.q.x, frame.q.y, frame.q.z, frame.q.w)),
            Math::Vector3(frame.p.x, frame.p.y, frame.p.z)));
    }

    // 착지 후보와 기존 연속 검사에서 실제 구동기의 동일한 관절 제한을 사용한다.
    inline FootstepPoseFailure CheckFootstepJointLimits(const PhysicsAsset& asset, bool articulation,
        const std::array<int, 4>& indices, const std::array<physx::PxTransform, 4>& frames)
    {
        std::array<Math::Matrix4, 4> bodies;
        for (size_t index = 0; index < 4; ++index)
        {
            if (indices[index] < 0 || static_cast<size_t>(indices[index]) >= asset.bodies.size())
            {
                return FootstepPoseFailure::InvalidInput;
            }
            bodies[index] = FootstepPoseMatrix(frames[index]) * asset.bodies[indices[index]].GetLocalTransform();
        }
        for (size_t index = 0; index < 3; ++index)
        {
            const int joint = asset.FindConstraint(asset.bodies[indices[index]].bone, asset.bodies[indices[index + 1]].bone);
            if (joint < 0)
            {
                return FootstepPoseFailure::InvalidInput;
            }
            const PhysicsConstraint& constraint = asset.constraints[joint];
            const Math::Quaternion raw = EvaluateJointDriveTarget(constraint, bodies[index], bodies[index + 1], false);
            const Math::Quaternion limited = articulation ?
                ClampArticulationDriveTarget(constraint, raw) : ClampD6DriveTarget(constraint, raw);
            const float dot = physx::PxClamp(physx::PxAbs(DirectX::XMVectorGetX(DirectX::XMVector4Dot(raw, limited))), 0.0f, 1.0f);
            if (!std::isfinite(dot))
            {
                return FootstepPoseFailure::InvalidInput;
            }
            if (2 * std::acos(dot) > .01f)
            {
                return FootstepPoseFailure::JointLimit;
            }
        }
        return FootstepPoseFailure::None;
    }
}
