#include "FootstepIK.h"
#include "foundation/PxMat33.h"
#include <algorithm>
#include <cmath>
#include <utility>

using namespace physx;
using namespace PARS;

namespace
{
    bool Unit(PxVec3& value)
    {
        const float length = value.magnitude();
        if (!value.isFinite() || !std::isfinite(length) || length <= 1e-6f)
        {
            return false;
        }
        value /= length;
        return true;
    }

    PxVec3 Perpendicular(const PxVec3& direction)
    {
        const PxVec3 basis = PxAbs(direction.x) < .7f ? PxVec3(1, 0, 0) : PxVec3(0, 1, 0);
        return (basis - direction * basis.dot(direction)).getNormalized();
    }

    bool ProjectHint(PxVec3& hint, const PxVec3& direction)
    {
        hint -= direction * hint.dot(direction);
        return Unit(hint);
    }

    PxQuat FromTo(PxVec3 from, PxVec3 to, PxVec3 planeNormal)
    {
        from.normalize();
        to.normalize();
        const float dot = PxClamp(from.dot(to), -1.0f, 1.0f);
        if (dot < -0.999999f)
        {
            if (!ProjectHint(planeNormal, from))
            {
                planeNormal = Perpendicular(from);
            }
            return PxQuat(PxPi, planeNormal);
        }
        const PxVec3 cross = from.cross(to);
        return PxQuat(cross.x, cross.y, cross.z, 1.0f + dot).getNormalized();
    }

    bool Ancestor(const std::vector<int32_t>& parents, int ancestor, int node)
    {
        if (ancestor < 0 || node < 0 || size_t(node) >= parents.size())
        {
            return false;
        }
        for (int parent = parents[node]; parent >= 0; parent = parents[parent])
        {
            if (parent == ancestor)
            {
                return true;
            }
        }
        return false;
    }

    constexpr size_t LegJointCount = 3;
    constexpr size_t LegVariableCount = LegJointCount * 3;
    constexpr size_t LegErrorCount = 7;
    constexpr float PositionTolerance = .001f;
    constexpr float TiltTolerance = PxPi / 360.0f;
    constexpr float TiltWeight = 2.0f;
    constexpr float HeadingWeight = .2f;

    struct LimitedLegPose
    {
    public:
        std::array<PxQuat, LegJointCount> rotations{PxQuat(PxIdentity), PxQuat(PxIdentity), PxQuat(PxIdentity)};
        std::array<PxTransform, LegJointCount> bones{PxTransform(PxIdentity), PxTransform(PxIdentity), PxTransform(PxIdentity)};
        PxTransform sole{PxIdentity};
        PxVec3 positionError{0};
        PxVec3 tiltError{0};
        float reachError = 0;
        float tiltAngle = 0;
        float yawError = 0;
        float primaryCost = 0;
    };

    PxVec3 RotationVector(PxQuat rotation)
    {
        rotation.normalize();
        if (rotation.w < 0)
        {
            rotation = -rotation;
        }
        const PxVec3 vector(rotation.x, rotation.y, rotation.z);
        const float length = vector.magnitude();
        return length > 1e-8f ? vector * (2.0f * std::atan2(length, rotation.w) / length) : PxVec3(0);
    }

    bool EvaluateLimitedLeg(const FootIKLegInput& leg, const std::array<PxQuat, LegJointCount>& rotations,
        float positionWeight, LimitedLegPose& result)
    {
        PxTransform parent = leg.actualAnchor;
        for (size_t joint = 0; joint < LegJointCount; ++joint)
        {
            const FootIKJointFrame& frame = leg.joints[joint];
            if (!frame.parentInBone.isSane() || !frame.childInBone.isSane() || !rotations[joint].isSane())
            {
                return false;
            }
            // 후보마다 실제 구동과 같은 제한을 적용한 다음 관절 연결점으로부터 본 자세를 다시 계산한다.
            result.rotations[joint] = leg.clampJointRotation(joint, rotations[joint]);
            if (!result.rotations[joint].isSane())
            {
                return false;
            }
            result.rotations[joint].normalize();
            result.bones[joint] = parent * frame.parentInBone * PxTransform(PxVec3(0), result.rotations[joint]) *
                frame.childInBone.getInverse();
            if (!result.bones[joint].isSane())
            {
                return false;
            }
            parent = result.bones[joint];
        }
        result.sole = result.bones[2] * leg.soleInBone;
        if (!result.sole.isSane())
        {
            return false;
        }
        result.positionError = leg.targetSole.p - result.sole.p;
        result.reachError = result.positionError.magnitude();
        const PxVec3 actualUp = result.sole.q.rotate(PxVec3(0, 1, 0));
        const PxVec3 targetUp = leg.targetSole.q.rotate(PxVec3(0, 1, 0));
        const PxQuat alignPlane = FromTo(actualUp, targetUp, leg.targetSole.q.rotate(PxVec3(1, 0, 0)));
        result.tiltError = RotationVector(alignPlane);
        result.tiltAngle = result.tiltError.magnitude();
        // 발바닥 법선을 맞춘 뒤 남는 회전만 발끝 방향 오차로 분리한다.
        const PxVec3 actualForward = alignPlane.rotate(result.sole.q.rotate(PxVec3(0, 0, 1)));
        const PxVec3 targetForward = leg.targetSole.q.rotate(PxVec3(0, 0, 1));
        result.yawError = std::atan2(targetUp.dot(actualForward.cross(targetForward)), actualForward.dot(targetForward));
        result.primaryCost = positionWeight * positionWeight * result.positionError.magnitudeSquared() +
            TiltWeight * TiltWeight * result.tiltError.magnitudeSquared();
        return std::isfinite(result.primaryCost) && std::isfinite(result.yawError);
    }

    std::array<float, LegErrorCount> LimitedLegErrors(const LimitedLegPose& pose, float positionWeight, float headingWeight)
    {
        return {pose.positionError.x * positionWeight, pose.positionError.y * positionWeight,
            pose.positionError.z * positionWeight, pose.tiltError.x * TiltWeight,
            pose.tiltError.y * TiltWeight, pose.tiltError.z * TiltWeight, pose.yawError * headingWeight};
    }

    float LimitedLegCost(const LimitedLegPose& pose, float headingWeight)
    {
        return pose.primaryCost + headingWeight * headingWeight * pose.yawError * pose.yawError;
    }

    bool SolveLegIncrement(const float jacobian[LegErrorCount][LegVariableCount],
        const std::array<float, LegErrorCount>& error, float damping, std::array<float, LegVariableCount>& increment)
    {
        // 감쇠 최소제곱으로 특이 자세와 잠긴 관절 축에서도 유한한 회전 증분을 구한다.
        double system[LegVariableCount][LegVariableCount + 1]{};
        for (size_t row = 0; row < LegVariableCount; ++row)
        {
            for (size_t column = 0; column < LegVariableCount; ++column)
            {
                for (size_t task = 0; task < LegErrorCount; ++task)
                {
                    system[row][column] += double(jacobian[task][row]) * jacobian[task][column];
                }
            }
            system[row][row] += damping;
            for (size_t task = 0; task < LegErrorCount; ++task)
            {
                system[row][LegVariableCount] -= double(jacobian[task][row]) * error[task];
            }
        }
        for (size_t column = 0; column < LegVariableCount; ++column)
        {
            size_t pivot = column;
            for (size_t row = column + 1; row < LegVariableCount; ++row)
            {
                if (std::abs(system[row][column]) > std::abs(system[pivot][column]))
                {
                    pivot = row;
                }
            }
            if (!std::isfinite(system[pivot][column]) || std::abs(system[pivot][column]) < 1e-12)
            {
                return false;
            }
            for (size_t entry = column; entry <= LegVariableCount; ++entry)
            {
                std::swap(system[column][entry], system[pivot][entry]);
            }
            const double divisor = system[column][column];
            for (size_t entry = column; entry <= LegVariableCount; ++entry)
            {
                system[column][entry] /= divisor;
            }
            for (size_t row = 0; row < LegVariableCount; ++row)
            {
                if (row == column)
                {
                    continue;
                }
                const double factor = system[row][column];
                for (size_t entry = column; entry <= LegVariableCount; ++entry)
                {
                    system[row][entry] -= factor * system[column][entry];
                }
            }
        }
        for (size_t variable = 0; variable < LegVariableCount; ++variable)
        {
            increment[variable] = static_cast<float>(system[variable][LegVariableCount]);
            if (!std::isfinite(increment[variable]))
            {
                return false;
            }
        }
        return true;
    }

    bool RefineLimitedLeg(const FootIKLegInput& leg, float positionWeight, bool refineHeading,
        float maximumReachError, float maximumTiltError, LimitedLegPose& state, uint32_t& iterations)
    {
        const float differenceStep = .002f;
        const float headingWeight = refineHeading ? HeadingWeight : 0.0f;
        const uint32_t maximumIterations = refineHeading ? 8u : 24u;
        float damping = .01f;
        for (uint32_t iteration = 0; iteration < maximumIterations; ++iteration)
        {
            if ((!refineHeading && state.reachError <= PositionTolerance && state.tiltAngle <= TiltTolerance) ||
                (refineHeading && std::abs(state.yawError) <= TiltTolerance))
            {
                break;
            }
            ++iterations;
            const std::array<float, LegErrorCount> error = LimitedLegErrors(state, positionWeight, headingWeight);
            float jacobian[LegErrorCount][LegVariableCount]{};
            for (size_t variable = 0; variable < LegVariableCount; ++variable)
            {
                const size_t joint = variable / 3;
                PxVec3 axis(0);
                axis[static_cast<unsigned>(variable % 3)] = 1;
                std::array<PxQuat, LegJointCount> positive = state.rotations;
                std::array<PxQuat, LegJointCount> negative = state.rotations;
                positive[joint] = (PxQuat(differenceStep, axis) * positive[joint]).getNormalized();
                negative[joint] = (PxQuat(-differenceStep, axis) * negative[joint]).getNormalized();
                LimitedLegPose plus;
                LimitedLegPose minus;
                if (!EvaluateLimitedLeg(leg, positive, positionWeight, plus) ||
                    !EvaluateLimitedLeg(leg, negative, positionWeight, minus))
                {
                    return false;
                }
                const std::array<float, LegErrorCount> plusError = LimitedLegErrors(plus, positionWeight, headingWeight);
                const std::array<float, LegErrorCount> minusError = LimitedLegErrors(minus, positionWeight, headingWeight);
                for (size_t task = 0; task < LegErrorCount - 1; ++task)
                {
                    jacobian[task][variable] = (plusError[task] - minusError[task]) / (2.0f * differenceStep);
                }
                // 뒤쪽을 향할 때 -180도와 +180도 사이의 불연속을 미분으로 오인하지 않는다.
                jacobian[LegErrorCount - 1][variable] = headingWeight *
                    std::remainder(plus.yawError - minus.yawError, PxTwoPi) / (2.0f * differenceStep);
            }
            std::array<float, LegVariableCount> increment{};
            if (!SolveLegIncrement(jacobian, error, damping, increment))
            {
                break;
            }
            float largestAngle = 0;
            for (size_t joint = 0; joint < LegJointCount; ++joint)
            {
                const PxVec3 rotation(increment[joint * 3], increment[joint * 3 + 1], increment[joint * 3 + 2]);
                largestAngle = PxMax(largestAngle, rotation.magnitude());
            }
            if (!std::isfinite(largestAngle) || largestAngle < 1e-6f)
            {
                break;
            }
            float scale = PxMin(1.0f, .2f / largestAngle);
            bool accepted = false;
            for (uint32_t attempt = 0; attempt < 8; ++attempt)
            {
                std::array<PxQuat, LegJointCount> rotations = state.rotations;
                for (size_t joint = 0; joint < LegJointCount; ++joint)
                {
                    const PxVec3 rotation(increment[joint * 3] * scale, increment[joint * 3 + 1] * scale,
                        increment[joint * 3 + 2] * scale);
                    const float angle = rotation.magnitude();
                    if (angle > 1e-8f)
                    {
                        rotations[joint] = (PxQuat(angle, rotation / angle) * rotations[joint]).getNormalized();
                    }
                }
                LimitedLegPose candidate;
                if (!EvaluateLimitedLeg(leg, rotations, positionWeight, candidate))
                {
                    return false;
                }
                if (candidate.reachError <= maximumReachError && candidate.tiltAngle <= maximumTiltError &&
                    LimitedLegCost(candidate, headingWeight) < LimitedLegCost(state, headingWeight))
                {
                    state = candidate;
                    damping = PxMax(.0001f, damping * .5f);
                    accepted = true;
                    break;
                }
                scale *= .5f;
            }
            if (!accepted)
            {
                damping *= 10.0f;
                if (damping > 100.0f)
                {
                    break;
                }
            }
        }
        return true;
    }

    bool ApplyLegJointLimits(const FootIKLegInput& leg, const LegIKResult& previous, LegIKResult& result)
    {
        const std::array<PxTransform, LegJointCount> seed{result.hip, result.knee, result.ankle};
        std::array<PxQuat, LegJointCount> rotations{PxQuat(PxIdentity), PxQuat(PxIdentity), PxQuat(PxIdentity)};
        PxTransform parent = leg.actualAnchor;
        for (size_t joint = 0; joint < LegJointCount; ++joint)
        {
            const FootIKJointFrame& frame = leg.joints[joint];
            if (!frame.parentInBone.isSane() || !frame.childInBone.isSane())
            {
                return false;
            }
            const PxQuat parentJoint = (parent.q * frame.parentInBone.q).getNormalized();
            const PxQuat childJoint = (seed[joint].q * frame.childInBone.q).getNormalized();
            rotations[joint] = (parentJoint.getConjugate() * childJoint).getNormalized();
            parent = seed[joint];
        }
        const float positionWeight = 4.0f / PxMax(leg.thighLength + leg.calfLength, .01f);
        LimitedLegPose state;
        if (!EvaluateLimitedLeg(leg, rotations, positionWeight, state))
        {
            return false;
        }
        result.initialLimitedReachError = state.reachError;
        result.initialLimitedTiltError = state.tiltAngle;
        // 이전 물리 틱의 해도 현재 제한으로 다시 검사해 가까운 해에서 이어 간다.
        if (previous.valid && previous.jointLimitsApplied)
        {
            LimitedLegPose warm;
            if (EvaluateLimitedLeg(leg, previous.jointRotations, positionWeight, warm) &&
                (warm.primaryCost < state.primaryCost ||
                    (warm.primaryCost == state.primaryCost && std::abs(warm.yawError) < std::abs(state.yawError))))
            {
                state = warm;
            }
        }
        uint32_t iterations = 0;
        if (!RefineLimitedLeg(leg, positionWeight, false, PX_MAX_F32, PX_MAX_F32, state, iterations))
        {
            return false;
        }
        // 위치와 발바닥 정렬을 먼저 풀고, 그 오차를 악화시키지 않는 범위에서만 발끝 방향을 맞춘다.
        const float maximumReachError = PxMax(state.reachError, PositionTolerance) + 1e-5f;
        const float maximumTiltError = PxMax(state.tiltAngle, TiltTolerance) + 1e-5f;
        if (!RefineLimitedLeg(leg, positionWeight, true, maximumReachError, maximumTiltError, state, iterations))
        {
            return false;
        }
        result.hip = state.bones[0];
        result.knee = state.bones[1];
        result.ankle = state.bones[2];
        result.solvedSole = state.sole;
        result.jointRotations = state.rotations;
        result.reachError = state.reachError;
        result.soleTiltError = state.tiltAngle;
        result.soleYawError = std::abs(state.yawError);
        result.limitIterations = iterations;
        result.jointLimitsApplied = true;
        result.limitConverged = state.reachError <= PositionTolerance && state.tiltAngle <= TiltTolerance;
        result.reachLimited = result.reachLimited || state.reachError > PositionTolerance;
        PxVec3 axis = result.ankle.p - result.hip.p;
        PxVec3 bend = result.knee.p - result.hip.p;
        if (Unit(axis) && ProjectHint(bend, axis))
        {
            result.bendDirection = bend;
        }
        return true;
    }
}

bool PARS::MakeFootSoleTarget(const PxVec3& position, const PxVec3& normal,
    const PxTransform& animatedSole, PxTransform& target)
{
    PxVec3 up = normal;
    if (!position.isFinite() || !animatedSole.isSane() || !Unit(up))
    {
        return false;
    }
    PxVec3 forward = animatedSole.q.rotate(PxVec3(0, 0, 1));
    if (!ProjectHint(forward, up))
    {
        PxVec3 right = animatedSole.q.rotate(PxVec3(1, 0, 0));
        if (!ProjectHint(right, up))
        {
            return false;
        }
        forward = right.cross(up);
    }
    const PxVec3 right = up.cross(forward).getNormalized();
    target = PxTransform(position, PxQuat(PxMat33(right, up, right.cross(up))).getNormalized());
    return target.isSane();
}

LegIKResult PARS::SolveTwoBoneIK(const TwoBoneIKInput& input)
{
    LegIKResult result;
    const float upper = input.thighLength, lower = input.calfLength;
    if (!input.hip.isSane() || !input.knee.isSane() || !input.ankle.isSane() ||
        !input.targetSole.isSane() || !input.soleInBone.isSane() ||
        !std::isfinite(upper) || !std::isfinite(lower) || upper <= 1e-5f || lower <= 1e-5f)
    {
        return result;
    }

    // input.targetSole: 발바닥의 목표 위치·회전
    // input.soleInBone: 발목본 기준으로 저장한 발바닥 로컬 변환
    // requested: 역산한 발목본의 목표 위치·회전
    const PxTransform requested = input.targetSole * input.soleInBone.getInverse();

    PxVec3 sourceUpper = input.knee.p - input.hip.p, sourceLower = input.ankle.p - input.knee.p;
    if (!requested.isSane() || !Unit(sourceUpper) || !Unit(sourceLower))
    {
        return result;
    }
    PxVec3 direction = requested.p - input.hip.p;
    const float requestedDistance = direction.magnitude();
    if (!std::isfinite(requestedDistance))
    {
        return result;
    }
    PxVec3 sourceAxis = input.ankle.p - input.hip.p;
    if (!Unit(sourceAxis))
    {
        sourceAxis = sourceUpper;
    }
    if (!Unit(direction))
    {
        direction = sourceAxis;
    }

    // 고관절에서 애니메이션 무릎으로 향하는 벡터
    const PxVec3 kneeOffset = input.knee.p - input.hip.p;

    // sourceAxis는 애니메이션 고관절에서 발목으로 향하는 단위벡터
    // 무릎 벡터에서 축과 나란한 성분을 제거해, 축에서 벗어난 굽힘 벡터를 구함
    PxVec3 sourceBend = kneeOffset - sourceAxis * kneeOffset.dot(sourceAxis);

    const bool animationBent = sourceBend.magnitude() > (upper + lower) * .001f;


    PxVec3 bend = sourceBend;
    // 거의 펴진 애니메이션의 작은 오차로 무릎 방향이 뒤집히는 것을 피한다.

    // bend는 b0
    // direction은 고관절에서 새 목표 발목으로 향하는 단위벡터
    if (animationBent && ProjectHint(bend, direction))
    {
        result.hintSource = KneeHintSource::Animation;
    }
    else
    {
        bend = input.previousBend;

        
        if (input.hasPreviousBend && ProjectHint(bend, direction))
        {
            result.hintSource = KneeHintSource::Previous;
        }
        else
        {
            bend = input.referenceBend;
            if (ProjectHint(bend, direction))
            {
                result.hintSource = KneeHintSource::Reference;
            }
            else
            {
                bend = Perpendicular(direction);
                result.hintSource = KneeHintSource::Fallback;
            }
        }
    }

    // 뼈 길이는 유지한다. 완전히 펴지거나 접히는 특이점에만 작은 계산 여유를 둔다.
    const float margin = PxMin((upper + lower) * 1e-4f, PxMin(upper, lower) * .1f);
    const float distance = PxClamp(requestedDistance, PxAbs(upper - lower) + margin, upper + lower - margin);

    // 두 직각삼각형의 관계로 고관절 H에서 기준점 Q까지의 거리 a를 구함
    const double along = (double(upper) * upper - double(lower) * lower + double(distance) * distance) / (2 * distance);

    // 피타고라스 정리로 기준선에서 무릎까지 떨어진 거리 h를 구함
    const float height = float(std::sqrt((std::max)(0.0, double(upper) * upper - along * along)));

    // Q = H + u * a에 무릎 굽힘 방향의 이동량 b * h를 더해 월드 위치 K를 구함
    const PxVec3 knee = input.hip.p + direction * float(along) + bend * height;


    const PxVec3 ankle = input.hip.p + direction * distance;
    const PxVec3 planeNormal = direction.cross(bend);
    if (!animationBent || !Unit(sourceBend))
    {
        sourceBend = bend;
        if (!ProjectHint(sourceBend, sourceAxis))
        {
            sourceBend = Perpendicular(sourceAxis);
        }
    }
    // 먼저 다리 전체의 굽힘 평면을 회전시킨다. 두 구간을 각각 최단 회전만 하면 무릎에 비틀림이 생긴다.
    const PxQuat sourcePlane(PxMat33(sourceAxis, sourceBend, sourceAxis.cross(sourceBend)));
    const PxQuat targetPlane(PxMat33(direction, bend, planeNormal));
    const PxQuat planeRotation = (targetPlane * sourcePlane.getConjugate()).getNormalized();
    result.hip = PxTransform(input.hip.p,
        (FromTo(planeRotation.rotate(sourceUpper), knee - input.hip.p, planeNormal) * planeRotation * input.hip.q).getNormalized());
    result.knee = PxTransform(knee,
        (FromTo(planeRotation.rotate(sourceLower), ankle - knee, planeNormal) * planeRotation * input.knee.q).getNormalized());
    result.ankle = PxTransform(ankle, requested.q);
    result.requestedSole = input.targetSole;
    result.solvedSole = result.ankle * input.soleInBone;
    result.requestedAnkle = requested.p;
    result.bendDirection = bend;
    result.reachError = (result.solvedSole.p - input.targetSole.p).magnitude();
    result.reachLimited = PxAbs(distance - requestedDistance) > 1e-6f;
    result.valid = result.hip.isSane() && result.knee.isSane() && result.ankle.isSane() &&
        result.solvedSole.isSane() && std::isfinite(result.reachError);
    return result;
}

void FootstepIK::Reset()
{
    m_Snapshot = {};
    m_HasPreviousBend = {};
    m_PreviousBendInAnchor = {PxVec3(0), PxVec3(0)};
}

bool FootstepIK::Apply(const FootstepIKInput& input, const std::vector<int32_t>& parents,
    std::vector<PxTransform>& pose)
{
    if (!input.enabled || input.revision != m_Snapshot.revision)
    {
        Reset();
    }
    const std::array<LegIKResult, 2> previousLegs = m_Snapshot.legs;
    m_Snapshot = {};
    m_Snapshot.revision = input.revision;
    m_Snapshot.stepFoot = input.stepFoot;
    if (!input.enabled)
    {
        return false;
    }
    m_Snapshot.status = FootstepIKStatus::InvalidPose;
    if (pose.empty() || pose.size() != parents.size())
    {
        return false;
    }
    for (size_t node = 0; node < pose.size(); ++node)
    {
        if (parents[node] < -1 || parents[node] >= int(node) || !pose[node].isSane())
        {
            return false;
        }
    }
    m_Snapshot.status = FootstepIKStatus::InvalidLeg;
    if (input.stepFoot < -1 || input.stepFoot > 1)
    {
        return false;
    }
    for (const PARS::FootIKLegInput& leg : input.legs)
    {
        if (!leg.available || !Ancestor(parents, leg.anchorNode, leg.hipNode) ||
            !Ancestor(parents, leg.hipNode, leg.kneeNode) || !Ancestor(parents, leg.kneeNode, leg.ankleNode) ||
            !leg.actualAnchor.isSane() || (leg.hasToeTarget &&
                (leg.toeNode == leg.ankleNode || !Ancestor(parents, leg.ankleNode, leg.toeNode) ||
                    !leg.toeTargetRotation.isSane())))
        {
            return false;
        }
    }
    const PARS::FootIKLegInput& left = input.legs[0];
    const PARS::FootIKLegInput& right = input.legs[1];
    if (left.hipNode == right.hipNode || Ancestor(parents, left.hipNode, right.hipNode) ||
        Ancestor(parents, right.hipNode, left.hipNode))
    {
        return false;
    }

    std::vector<physx::PxTransform> corrected = pose;
    std::vector<bool> changed(pose.size(), false);
    m_Snapshot.status = FootstepIKStatus::InvalidTarget;
    for (size_t side = 0; side < input.legs.size(); ++side)
    {
        const PARS::FootIKLegInput& leg = input.legs[side];
        // 현재 물리 부모를 기준으로 풀되, 골반의 애니메이션 목표 자체는 덮어쓰지 않는다.
        const PxTransform toPhysical = leg.actualAnchor * pose[leg.anchorNode].getInverse();
        TwoBoneIKInput solve;
        solve.hip = toPhysical * pose[leg.hipNode];
        solve.knee = toPhysical * pose[leg.kneeNode];
        solve.ankle = toPhysical * pose[leg.ankleNode];
        solve.soleInBone = leg.soleInBone;
        solve.targetSole = leg.targetSole;
        solve.thighLength = leg.thighLength;
        solve.calfLength = leg.calfLength;
        solve.referenceBend = solve.hip.q.rotate(leg.referenceBendInHip);
        solve.hasPreviousBend = m_HasPreviousBend[side];
        solve.previousBend = leg.actualAnchor.q.rotate(m_PreviousBendInAnchor[side]);
        PARS::LegIKResult& result = m_Snapshot.legs[side];
        result = SolveTwoBoneIK(solve);
        if (!result.valid)
        {
            return false;
        }
        if (leg.clampJointRotation && !ApplyLegJointLimits(leg, previousLegs[side], result))
        {
            result.valid = false;
            return false;
        }
        const PxTransform toAnimation = toPhysical.getInverse();
        corrected[leg.hipNode] = toAnimation * result.hip;
        corrected[leg.kneeNode] = toAnimation * result.knee;
        corrected[leg.ankleNode] = toAnimation * result.ankle;
        changed[leg.hipNode] = changed[leg.kneeNode] = changed[leg.ankleNode] = true;
    }
    for (size_t node = 0; node < pose.size(); ++node)
    {
        const int parent = parents[node];
        if (!changed[node] && parent >= 0 && changed[parent])
        {
            // 자식의 로컬 위치와 기본 애니메이션 회전을 먼저 보존한다.
            corrected[node] = corrected[parent] * pose[parent].getInverse() * pose[node];
            changed[node] = true;
        }
        for (const FootIKLegInput& leg : input.legs)
        {
            if (leg.hasToeTarget && static_cast<int>(node) == leg.toeNode)
            {
                const PxTransform toPhysical = leg.actualAnchor * pose[leg.anchorNode].getInverse();
                corrected[node].q = (toPhysical.q.getConjugate() * leg.toeTargetRotation).getNormalized();
                changed[node] = true;
            }
        }
        if (changed[node])
        {
            m_Snapshot.correctedNodes.push_back(int(node));
        }
    }
    pose = std::move(corrected);
    for (size_t side = 0; side < input.legs.size(); ++side)
    {
        m_PreviousBendInAnchor[side] = input.legs[side].actualAnchor.q.rotateInv(m_Snapshot.legs[side].bendDirection);
        m_HasPreviousBend[side] = true;
    }
    m_Snapshot.status = FootstepIKStatus::Applied;
    return true;
}

const char* PARS::FootstepIKStatusLabel(FootstepIKStatus status)
{
    switch (status)
    {
    case FootstepIKStatus::Applied:
        return u8"양쪽 다리 IK 목표 적용";
    case FootstepIKStatus::InvalidPose:
        return u8"본 계층 또는 목표 자세 오류";
    case FootstepIKStatus::InvalidLeg:
        return u8"다리 바디·관절·물리 부모 연결 확인 필요";
    case FootstepIKStatus::InvalidTarget:
        return u8"발 목표 또는 IK 계산 오류";
    default:
        return u8"발 보정 비활성";
    }
}

const char* PARS::KneeHintSourceLabel(KneeHintSource source)
{
    switch (source)
    {
    case KneeHintSource::Animation:
        return u8"애니메이션";
    case KneeHintSource::Previous:
        return u8"이전 방향";
    case KneeHintSource::Reference:
        return u8"기준 자세";
    default:
        return u8"수치 대체 방향";
    }
}
