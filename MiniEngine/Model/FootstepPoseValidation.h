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

    inline float FootstepJointCorrectionRadians(const PhysicsConstraint& constraint, bool articulation,
        const Math::Matrix4& parent, const Math::Matrix4& child, std::array<bool, 4>* limitedAxes = nullptr)
    {
        const Math::Quaternion raw = EvaluateJointDriveTarget(constraint, parent, child, false);
        const Math::Quaternion limited = articulation ?
            ClampArticulationDriveTarget(constraint, raw, limitedAxes) : ClampD6DriveTarget(constraint, raw, limitedAxes);
        const float dot = physx::PxAbs(DirectX::XMVectorGetX(DirectX::XMVector4Dot(raw, limited)));
        return std::isfinite(dot) ? 2 * std::acos(physx::PxClamp(dot, 0.0f, 1.0f)) : dot;
    }

    // 각도 보정은 구동 시 참고할 값이다. 다른 관절의 데이터 오류까지 확인한다.
    inline FootstepJointStatus CheckFootstepJointLimits(const PhysicsAsset& asset, bool articulation,
        const std::array<int, 4>& indices, const std::array<physx::PxTransform, 4>& frames,
        std::array<FootstepJointCheck, 3>* details = nullptr)
    {
        std::array<Math::Matrix4, 4> bodies;
        std::array<FootstepJointInputIssue, 4> bodyIssues{};
        for (size_t index = 0; index < bodies.size(); ++index)
        {
            if (indices[index] < 0 || static_cast<size_t>(indices[index]) >= asset.bodies.size())
            {
                bodyIssues[index] = FootstepJointInputIssue::Body;
            }
            else if (!frames[index].isSane())
            {
                bodyIssues[index] = FootstepJointInputIssue::Rotation;
            }
            else
            {
                bodies[index] = FootstepPoseMatrix(frames[index]) * asset.bodies[indices[index]].GetLocalTransform();
            }
        }
        FootstepJointStatus status = FootstepJointStatus::WithinLimits;
        for (size_t index = 0; index + 1 < bodies.size(); ++index)
        {
            FootstepJointCheck joint;
            joint.inputIssue = bodyIssues[index] != FootstepJointInputIssue::None ? bodyIssues[index] : bodyIssues[index + 1];
            if (joint.inputIssue == FootstepJointInputIssue::None)
            {
                const int constraint = asset.FindConstraint(asset.bodies[indices[index]].bone, asset.bodies[indices[index + 1]].bone);
                if (constraint < 0)
                {
                    joint.inputIssue = FootstepJointInputIssue::Constraint;
                }
                else
                {
                    std::array<bool, 4> limitedAxes{};
                    const float correction = FootstepJointCorrectionRadians(asset.constraints[constraint], articulation,
                        bodies[index], bodies[index + 1], details ? &limitedAxes : nullptr);
                    if (!std::isfinite(correction))
                    {
                        joint.inputIssue = FootstepJointInputIssue::Rotation;
                    }
                    else if (correction > FootstepPoseCheck::JointToleranceRadians)
                    {
                        joint.correctionRadians = correction;
                        joint.limitedAxes = limitedAxes;
                        joint.usesArticulation = articulation;
                        if (status == FootstepJointStatus::WithinLimits)
                        {
                            status = FootstepJointStatus::Clamped;
                        }
                    }
                }
            }
            if (joint.inputIssue != FootstepJointInputIssue::None)
            {
                status = FootstepJointStatus::InvalidInput;
            }
            if (details)
            {
                (*details)[index] = joint;
            }
            else if (status == FootstepJointStatus::InvalidInput)
            {
                break;
            }
        }
        return status;
    }
}
