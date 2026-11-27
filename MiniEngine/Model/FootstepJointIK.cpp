#include "FootstepIK.h"
#include "foundation/PxMat33.h"
#include "foundation/PxMathUtils.h"
#include <cmath>

using namespace PARS;
using namespace physx;

namespace
{
    using JointErrors = std::array<PxVec3, 3>;

    struct ProjectedLeg
    {
    public:
        std::array<PxQuat, 3> local;
        JointErrors errors;
        float squaredError = 0;
    };

    float ErrorDot(const JointErrors& a, const JointErrors& b)
    {
        return a[0].dot(b[0]) + a[1].dot(b[1]) + a[2].dot(b[2]);
    }

    PxQuat ProjectLocal(const FootIKJointLimits& limits, size_t joint, const PxQuat& local, PxVec3* error)
    {
        const PxQuat raw = (limits.parentFrames[joint].getConjugate() * local * limits.childFrames[joint]).getNormalized();
        const PxQuat allowed = limits.project(*limits.joints[joint], raw);
        if (error)
        {
            const PxQuat difference = allowed * raw.getConjugate();
            *error = difference.getImaginaryPart() * (difference.w < 0 ? -2.0f : 2.0f);
        }
        return (limits.parentFrames[joint] * allowed * limits.childFrames[joint].getConjugate()).getNormalized();
    }

    ProjectedLeg ProjectLeg(const FootIKJointLimits& limits, const std::array<PxQuat, 3>& bones)
    {
        ProjectedLeg pose;
        PxQuat parent = limits.anchorRotation;
        for (size_t i = 0; i < bones.size(); ++i)
        {
            const PxQuat local = parent.getConjugate() * bones[i];
            pose.local[i] = ProjectLocal(limits, i, local, &pose.errors[i]);
            parent = bones[i];
        }
        pose.squaredError = ErrorDot(pose.errors, pose.errors);
        return pose;
    }

    struct LegFreedom
    {
    public:
        ProjectedLeg Evaluate(const PxVec3& angles) const
        {
            // 첫 회전은 무릎을 고관절-발목 축 주위로 옮긴다. 나머지는 각 뼈의 길이축 회전이다.
            // 세 회전 모두 발목 목표와 발바닥 방향을 보존한다.
            const PxQuat swivel(angles.x, ankleDirection);
            return ProjectLeg(limits, {{swivel * PxQuat(angles.y, thighDirection) * preferred.hip.q,
                swivel * PxQuat(angles.z, calfDirection) * preferred.knee.q, preferred.ankle.q}});
        }

    public:
        const FootIKJointLimits& limits;
        const LegIKResult& preferred;
        PxVec3 ankleDirection;
        PxVec3 thighDirection;
        PxVec3 calfDirection;
    };

    void SetBoundedPose(LegIKResult& result, const PxQuat& hip, const ProjectedLeg& pose,
        const PxVec3& upperInBone, const PxVec3& lowerInBone, const PxTransform& soleInBone)
    {
        result.hip.q = hip;
        result.knee = PxTransform(result.hip.transform(upperInBone), (hip * pose.local[1]).getNormalized());
        result.ankle = PxTransform(result.knee.transform(lowerInBone), (result.knee.q * pose.local[2]).getNormalized());
        result.solvedSole = result.ankle * soleInBone;
    }

    PxVec3 CorrectionStep(const LegFreedom& leg, const PxVec3& angles, const ProjectedLeg& pose)
    {
        // 세 자유도에서 축 제한 초과량의 변화율을 구해 감쇠 최소제곱으로 한 걸음 보정한다.
        constexpr float increment = .001f;
        std::array<JointErrors, 3> derivatives;
        for (PxU32 axis = 0; axis < 3; ++axis)
        {
            PxVec3 shifted = angles;
            shifted[axis] += increment;
            const ProjectedLeg probe = leg.Evaluate(shifted);
            for (size_t joint = 0; joint < pose.errors.size(); ++joint)
            {
                derivatives[axis][joint] = (probe.errors[joint] - pose.errors[joint]) / increment;
            }
        }
        PxMat33 normal(PxZero);
        PxVec3 gradient(0);
        for (PxU32 axis = 0; axis < 3; ++axis)
        {
            gradient[axis] = -ErrorDot(derivatives[axis], pose.errors);
            for (PxU32 other = 0; other < 3; ++other)
            {
                normal[axis][other] = ErrorDot(derivatives[axis], derivatives[other]);
            }
            normal[axis][axis] += .0001f;
        }
        const PxVec3 step = normal.getInverse() * gradient;
        return step * (.6f / PxMax(.6f, step.magnitude()));
    }
}

void PARS::ConstrainFootstepLeg(const FootIKJointLimits& limits, const PxTransform& soleInBone, LegIKResult& result)
{
    ProjectedLeg pose = ProjectLeg(limits, {{result.hip.q, result.knee.q, result.ankle.q}});
    constexpr float toleranceSquared = 1e-10f;
    if (pose.squaredError <= toleranceSquared)
    {
        return;
    }
    const PxVec3 upper = result.knee.p - result.hip.p;
    const PxVec3 lower = result.ankle.p - result.knee.p;
    const PxVec3 upperInBone = result.hip.q.rotateInv(upper);
    const PxVec3 lowerInBone = result.knee.q.rotateInv(lower);
    const PxVec3 ankleDirection = (result.ankle.p - result.hip.p).getNormalized();
    const LegFreedom freedom{limits, result, ankleDirection, upper.getNormalized(), lower.getNormalized()};
    PxVec3 angles(0);
    for (int iteration = 0; iteration < 12 && pose.squaredError > toleranceSquared; ++iteration)
    {
        PxVec3 step = CorrectionStep(freedom, angles, pose);
        const float initialError = pose.squaredError;
        for (int trial = 0; trial < 5 && step.magnitudeSquared() > 1e-12f; ++trial)
        {
            const ProjectedLeg next = freedom.Evaluate(angles + step);
            if (next.squaredError < initialError)
            {
                angles += step;
                pose = next;
                break;
            }
            step *= .5f;
        }
        if (pose.squaredError >= initialError)
        {
            break;
        }
    }

    // 실제 구동기에 허용된 로컬 회전만으로 다시 연결한다. 닿지 않은 발 목표에는 실제 잔차를 남긴다.
    result.jointAdjusted = true;
    SetBoundedPose(result, (limits.anchorRotation * pose.local[0]).getNormalized(), pose, upperInBone, lowerInBone, soleInBone);
    const PxVec3 fromHip = result.solvedSole.p - result.hip.p;
    const PxVec3 goalFromHip = result.requestedSole.p - result.hip.p;
    if (pose.squaredError > toleranceSquared && fromHip.magnitudeSquared() > 1e-10f &&
        goalFromHip.magnitudeSquared() > 1e-10f && (fromHip - goalFromHip).magnitudeSquared() > 1e-8f)
    {
        // 발 방향까지 만족하지 못하면 무릎·발목 상대각을 보존한 채 허용되는 고관절 회전으로 발 위치를 우선한다.
        const PxQuat toward = PxShortestRotation(fromHip.getNormalized(), goalFromHip.getNormalized());
        const PxQuat local = limits.anchorRotation.getConjugate() * toward * result.hip.q;
        const PxQuat hip = (limits.anchorRotation * ProjectLocal(limits, 0, local, nullptr)).getNormalized();
        const PxQuat turn = (hip * result.hip.q.getConjugate()).getNormalized();
        if ((turn.rotate(fromHip) - goalFromHip).magnitudeSquared() < (fromHip - goalFromHip).magnitudeSquared())
        {
            SetBoundedPose(result, hip, pose, upperInBone, lowerInBone, soleInBone);
        }
    }
    result.reachError = (result.solvedSole.p - result.requestedSole.p).magnitude();
    const PxQuat soleDifference = result.solvedSole.q * result.requestedSole.q.getConjugate();
    result.soleAngleError = 2 * std::atan2(soleDifference.getImaginaryPart().magnitude(), PxAbs(soleDifference.w));
    result.reachLimited = result.reachLimited || result.reachError > FootstepSoleTolerance ||
        result.soleAngleError > FootstepSoleAngleTolerance;
    const PxVec3 finalAxis = (result.ankle.p - result.hip.p).getNormalized();
    const PxVec3 finalThigh = result.knee.p - result.hip.p;
    const PxVec3 bend = finalThigh - finalAxis * finalThigh.dot(finalAxis);
    if (bend.magnitudeSquared() > 1e-10f)
    {
        result.bendDirection = bend.getNormalized();
    }
}
