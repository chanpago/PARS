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
        const Math::Matrix4& parent, const Math::Matrix4& child)
    {
        const Math::Quaternion raw = EvaluateJointDriveTarget(constraint, parent, child, false);
        const Math::Quaternion limited = articulation ?
            ClampArticulationDriveTarget(constraint, raw) : ClampD6DriveTarget(constraint, raw);
        const float dot = physx::PxAbs(DirectX::XMVectorGetX(DirectX::XMVector4Dot(raw, limited)));
        return std::isfinite(dot) ? 2 * std::acos(physx::PxClamp(dot, 0.0f, 1.0f)) : dot;
    }

    // 착지 후보는 위반한 관절을 모두 기록한다. 연속 경로 검사는 첫 위반에서 중단할 수 있다.
    inline FootstepPoseFailures CheckFootstepJointLimits(const PhysicsAsset& asset, bool articulation,
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
        FootstepPoseFailures failures;
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
                    const float correction = FootstepJointCorrectionRadians(asset.constraints[constraint], articulation,
                        bodies[index], bodies[index + 1]);
                    if (!std::isfinite(correction))
                    {
                        joint.inputIssue = FootstepJointInputIssue::Rotation;
                    }
                    else if (correction > FootstepPoseCheck::JointToleranceRadians)
                    {
                        joint.correctionRadians = correction;
                        failures |= FootstepPoseFailure::JointLimit;
                    }
                }
            }
            if (joint.inputIssue != FootstepJointInputIssue::None)
            {
                failures |= FootstepPoseFailure::InvalidInput;
            }
            if (details)
            {
                (*details)[index] = joint;
            }
            else if (failures)
            {
                break;
            }
        }
        return failures;
    }
}
