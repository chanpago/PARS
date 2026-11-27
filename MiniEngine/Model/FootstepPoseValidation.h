#pragma once

#include "FootstepRecovery.h"
#include "PhysicsPose.h"
#include "foundation/PxTransform.h"
#include <array>
#include <cmath>

namespace PARS
{
    inline physx::PxQuat FootstepRotation(const DirectX::XMFLOAT4& rotation)
    {
        return physx::PxQuat(rotation.x, rotation.y, rotation.z, rotation.w);
    }

    template <bool Articulation>
    inline physx::PxQuat ProjectFootstepRotation(const PhysicsConstraint& joint, const physx::PxQuat& rotation)
    {
        const Math::Quaternion raw(DirectX::XMVectorSet(rotation.x, rotation.y, rotation.z, rotation.w));
        const Math::Quaternion limited = Articulation ? ClampArticulationDriveTarget(joint, raw) : ClampD6DriveTarget(joint, raw);
        DirectX::XMFLOAT4 q;
        DirectX::XMStoreFloat4(&q, limited);
        return FootstepRotation(q);
    }

    template <bool Articulation>
    inline bool FootstepContainsRotations(const PhysicsConstraint& joint, physx::PxQuat q, float radius)
    {
        using namespace physx;
        if (joint.twist == PhysicsJointMotion::Free && joint.swingY == PhysicsJointMotion::Free &&
            joint.swingZ == PhysicsJointMotion::Free)
        {
            return true;
        }
        if (q.w < 0)
        {
            q = -q;
        }
        const float angle = 2 * std::atan2(q.getImaginaryPart().magnitude(), q.w);
        if (angle + radius >= PxPi)
        {
            return false;
        }
        PxVec3 coordinates(0);
        // 최단 회전 벡터의 변화율은 반회전까지 π/2 이하이다.
        float span = 2 * radius;
        if (Articulation)
        {
            const float length = q.getImaginaryPart().magnitude();
            coordinates = q.getImaginaryPart() * (length > 1e-8f ? angle / length : 2.0f);
        }
        else
        {
            // 트위스트 분해의 특이점에서 떨어진 구간만 증명한다. 스윙 좌표와 원뿔의 오차도 넉넉히 감싼다.
            const float twistLength = std::sqrt(q.x * q.x + q.w * q.w);
            if (twistLength <= 2 * radius)
            {
                return false;
            }
            const PxQuat swing = q * PxQuat(-q.x / twistLength, 0, 0, q.w / twistLength);
            coordinates = PxVec3(2 * std::atan2(q.x, q.w), 4 * std::atan2(swing.y, 1 + swing.w),
                4 * std::atan2(swing.z, 1 + swing.w));
            span = 8 * radius / twistLength;
            if (joint.swingY == PhysicsJointMotion::Limited && joint.swingZ == PhysicsJointMotion::Limited &&
                !joint.HasAsymmetricSwingZLimit())
            {
                const float y = (PxAbs(coordinates.y) + span) / joint.swingYLimit;
                const float z = (PxAbs(coordinates.z) + span) / joint.swingZMax;
                if (!(y * y + z * z <= 1))
                {
                    return false;
                }
            }
        }
        const PhysicsJointMotion motions[] = {joint.twist, joint.swingY, joint.swingZ};
        const PxVec3 lower(joint.twistMin, -joint.swingYLimit, joint.swingZMin);
        const PxVec3 upper(joint.twistMax, joint.swingYLimit, joint.swingZMax);
        for (PxU32 axis = 0; axis < 3; ++axis)
        {
            if (motions[axis] == PhysicsJointMotion::Locked ||
                (motions[axis] == PhysicsJointMotion::Limited &&
                (coordinates[axis] - span < lower[axis] || coordinates[axis] + span > upper[axis])))
            {
                return false;
            }
        }
        return true;
    }

    inline bool PrepareFootIKJointLimits(const PhysicsAsset& asset, bool articulation,
        const std::array<int, 4>& bodies, const physx::PxQuat& anchor, FootIKJointLimits& limits)
    {
        limits.project = nullptr;
        for (size_t i = 0; i < limits.joints.size(); ++i)
        {
            if (static_cast<size_t>(bodies[i]) >= asset.bodies.size() || static_cast<size_t>(bodies[i + 1]) >= asset.bodies.size())
            {
                return false;
            }
            const PhysicsBody& parent = asset.bodies[bodies[i]];
            const PhysicsBody& child = asset.bodies[bodies[i + 1]];
            const int index = asset.FindConstraint(parent.bone, child.bone);
            if (index < 0 || asset.constraints[index].parentBone != parent.bone)
            {
                return false;
            }
            const PhysicsConstraint& joint = asset.constraints[index];
            limits.joints[i] = &joint;
            limits.parentFrames[i] = (FootstepRotation(parent.rotation) * FootstepRotation(joint.parentFrame.rotation)).getNormalized();
            limits.childFrames[i] = (FootstepRotation(child.rotation) * FootstepRotation(joint.childFrame.rotation)).getNormalized();
        }
        limits.anchorRotation = anchor;
        limits.project = articulation ? ProjectFootstepRotation<true> : ProjectFootstepRotation<false>;
        limits.contains = articulation ? FootstepContainsRotations<true> : FootstepContainsRotations<false>;
        return true;
    }

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
