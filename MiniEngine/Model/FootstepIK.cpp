#include "FootstepIK.h"
#include "foundation/PxMat33.h"
#include "FootstepPlanner.h"
#include "FramePerformance.h"
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
    constexpr size_t LegErrorCount = 6;
    constexpr float PositionTolerance = .001f;
    constexpr float TiltTolerance = PxPi / 360.0f;
    constexpr float TiltWeight = 2.0f;

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
        float cost = 0;
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

    bool EvaluateLimitedLegError(const FootIKLegInput& leg, float positionWeight, LimitedLegPose& result)
    {
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
        // 위치와 발바닥 법선만 맞춘다. 법선 축을 중심으로 한 발끝 방향 회전은 목표에 포함하지 않는다.
        result.cost = positionWeight * positionWeight * result.positionError.magnitudeSquared() +
            TiltWeight * TiltWeight * result.tiltError.magnitudeSquared();
        return std::isfinite(result.cost);
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
        return EvaluateLimitedLegError(leg, positionWeight, result);
    }

    bool EvaluateLimitedLegPerturbation(const FootIKLegInput& leg, const LimitedLegPose& base,
        size_t joint, const PxQuat& rotation, float positionWeight, LimitedLegPose& result)
    {
        result = base;
        result.rotations[joint] = leg.clampJointRotation(joint, rotation);
        if (!result.rotations[joint].isSane())
        {
            return false;
        }
        result.rotations[joint].normalize();
        // 수치 미분에서는 바뀐 관절만 다시 제한하고 그 아래의 본만 갱신한다.
        // 나머지 관절의 제한 결과와 부모 자세는 기준 해에 이미 계산되어 있다.
        PxTransform parent = joint == 0 ? leg.actualAnchor : base.bones[joint - 1];
        for (size_t child = joint; child < LegJointCount; ++child)
        {
            const FootIKJointFrame& frame = leg.joints[child];
            result.bones[child] = parent * frame.parentInBone * PxTransform(PxVec3(0), result.rotations[child]) *
                frame.childInBone.getInverse();
            if (!result.bones[child].isSane())
            {
                return false;
            }
            parent = result.bones[child];
        }
        return EvaluateLimitedLegError(leg, positionWeight, result);
    }

    std::array<float, LegErrorCount> LimitedLegErrors(const LimitedLegPose& pose, float positionWeight)
    {
        return {pose.positionError.x * positionWeight, pose.positionError.y * positionWeight,
            pose.positionError.z * positionWeight, pose.tiltError.x * TiltWeight,
            pose.tiltError.y * TiltWeight, pose.tiltError.z * TiltWeight};
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

    bool RefineLimitedLeg(const FootIKLegInput& leg, float positionWeight, LimitedLegPose& state,
        uint32_t& iterations, float positionTolerance, float tiltTolerance)
    {
        const float differenceStep = .002f;
        const uint32_t maximumIterations = 24;
        float damping = .01f;
        for (uint32_t iteration = 0; iteration < maximumIterations; ++iteration)
        {
            if (state.reachError <= positionTolerance && state.tiltAngle <= tiltTolerance)
            {
                break;
            }
            ++iterations;
            const std::array<float, LegErrorCount> error = LimitedLegErrors(state, positionWeight);
            float jacobian[LegErrorCount][LegVariableCount]{};
            for (size_t variable = 0; variable < LegVariableCount; ++variable)
            {
                const size_t joint = variable / 3;
                PxVec3 axis(0);
                axis[static_cast<unsigned>(variable % 3)] = 1;
                const PxQuat positive = (PxQuat(differenceStep, axis) * state.rotations[joint]).getNormalized();
                const PxQuat negative = (PxQuat(-differenceStep, axis) * state.rotations[joint]).getNormalized();
                LimitedLegPose plus;
                LimitedLegPose minus;
                if (!EvaluateLimitedLegPerturbation(leg, state, joint, positive, positionWeight, plus) ||
                    !EvaluateLimitedLegPerturbation(leg, state, joint, negative, positionWeight, minus))
                {
                    return false;
                }
                const std::array<float, LegErrorCount> plusError = LimitedLegErrors(plus, positionWeight);
                const std::array<float, LegErrorCount> minusError = LimitedLegErrors(minus, positionWeight);
                for (size_t task = 0; task < LegErrorCount; ++task)
                {
                    jacobian[task][variable] = (plusError[task] - minusError[task]) / (2.0f * differenceStep);
                }
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
                if (candidate.cost < state.cost)
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

    bool ApplyLegJointLimits(const FootIKLegInput& leg, const LegIKResult& previous, LegIKResult& result,
        float positionTolerance, float tiltTolerance)
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
            if (EvaluateLimitedLeg(leg, previous.jointRotations, positionWeight, warm) && warm.cost <= state.cost)
            {
                state = warm;
            }
        }
        uint32_t iterations = 0;
        if (!RefineLimitedLeg(leg, positionWeight, state, iterations, positionTolerance, tiltTolerance))
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
        result.limitIterations = iterations;
        result.jointLimitsApplied = true;
        result.limitConverged = state.reachError <= positionTolerance && state.tiltAngle <= tiltTolerance;
        result.reachLimited = result.reachLimited || state.reachError > positionTolerance;
        PxVec3 axis = result.ankle.p - result.hip.p;
        PxVec3 bend = result.knee.p - result.hip.p;
        if (Unit(axis) && ProjectHint(bend, axis))
        {
            result.bendDirection = bend;
        }
        return true;
    }
}

LegIKResult PARS::SolveFootIKLeg(const FootIKLegInput& leg,
    const std::array<PxTransform, 3>& bones, const LegIKResult& previous,
    float positionTolerance, float tiltTolerance)
{
    TwoBoneIKInput solve;
    solve.hip = bones[0];
    solve.knee = bones[1];
    solve.ankle = bones[2];
    solve.targetSole = leg.targetSole;
    solve.soleInBone = leg.soleInBone;
    solve.thighLength = leg.thighLength;
    solve.calfLength = leg.calfLength;
    solve.referenceBend = solve.hip.q.rotate(leg.referenceBendInHip);
    solve.previousBend = previous.bendDirection;
    solve.hasPreviousBend = previous.valid;
    LegIKResult result = SolveTwoBoneIK(solve);
    if (result.valid && leg.clampJointRotation &&
        !ApplyLegJointLimits(leg, previous, result, positionTolerance, tiltTolerance))
    {
        result.valid = false;
    }
    return result;
}

namespace
{
    PxQuat InterpolateRotation(const PxQuat& start, const PxQuat& target, float progress)
    {
        const PxVec3 rotation = RotationVector(target * start.getConjugate());
        const float angle = rotation.magnitude();
        if (angle <= 1e-6f)
        {
            return start;
        }
        return (PxQuat(angle * progress, rotation / angle) * start).getNormalized();
    }

    float AccelerationTimeScale(float acceleration, float limit)
    {
        if (acceleration <= 1e-4f)
        {
            return 0;
        }
        return limit > 1e-6f ? std::sqrt(acceleration / limit) : PX_MAX_F32;
    }

    float EstimateJointTravelDuration(float travel, float initialVelocity, float speedLimit, float accelerationLimit)
    {
        const float speed = PxAbs(initialVelocity);
        if (travel <= 1e-5f && speed <= 1e-4f)
        {
            return 0;
        }
        if (accelerationLimit <= 1e-6f)
        {
            return PX_MAX_F32;
        }
        // 현재 속도의 제동 여유와 전체 관절 이동량으로 시간을 추정한다.
        // 서로 다른 IK 해의 차분을 실제 가속도로 취급하면 출발 직후 모든 후보가 탈락할 수 있다.
        const float brakingTime = speed / accelerationLimit;
        const float distance = travel + .5f * speed * brakingTime;
        const float motionTime = (std::max)(1.5f * distance / speedLimit,
            std::sqrt(6.0f * distance / accelerationLimit));
        return brakingTime + motionTime;
    }

    FootstepReachability EvaluatePelvisPlan(const FootstepPlanningInput& input,
        const FootstepSettings& settings, size_t moving, const PxVec3& position,
        const PxVec3& normal, float duration, float liftHeight, const PxTransform& pelvisTarget)
    {
        FootstepReachability result;
        result.pelvisStart = input.legs[moving].actualAnchor;
        result.pelvisTarget = pelvisTarget;
        const PxVec3 rotation = RotationVector(pelvisTarget.q * result.pelvisStart.q.getConjugate());
        result.pelvisRotated = rotation.magnitude() > 1e-4f;
        float timeScale = (std::max)(1.5f * rotation.magnitude() /
            (duration * settings.maximumPelvisAngularSpeed),
            AccelerationTimeScale(6.0f * rotation.magnitude() / (duration * duration),
                settings.maximumPelvisAngularAcceleration));
        timeScale = (std::max)(timeScale, AccelerationTimeScale(
            2.0f * input.pelvisAngularVelocity.magnitude() / duration,
            settings.maximumPelvisAngularAcceleration));
        if (timeScale > 1.0f + 1e-6f)
        {
            result.timeLimited = true;
            result.requiredDuration = duration * timeScale;
            return result;
        }

        PxTransform landing(PxIdentity);
        if (!MakeFootSoleTarget(position, normal, input.soles[moving], landing))
        {
            return result;
        }
        // 길이만으로 불가능한 끝점은 수치 IK의 반복 계산 전에 제외한다.
        for (size_t side = 0; side < input.legs.size(); ++side)
        {
            const FootIKLegInput& leg = input.legs[side];
            const PxTransform sole = side == moving ? landing : input.supportSoles[side];
            const PxVec3 ankle = (sole * leg.soleInBone.getInverse()).p;
            const PxTransform delta = pelvisTarget * leg.actualAnchor.getInverse();
            const PxVec3 hip = (delta * input.bones[side][0]).p;
            if ((ankle - hip).magnitude() > leg.thighLength + leg.calfLength + settings.reachPositionTolerance)
            {
                return result;
            }
        }
        // 끝점뿐 아니라 들어 올리고 내려놓는 궤적 전체를 같은 제한 IK로 검사한다.
        constexpr size_t Samples = 8;
        std::array<FootIKLegInput, 2> legs = input.legs;
        std::array<LegIKResult, 2> previous;
        std::array<std::array<PxQuat, 3>, 2> rotations;
        std::array<std::array<PxVec3, 3>, 2> travels;
        for (size_t side = 0; side < input.legs.size(); ++side)
        {
            PxTransform parent = input.legs[side].actualAnchor;
            for (size_t joint = 0; joint < 3; ++joint)
            {
                rotations[side][joint] = ((parent.q * input.legs[side].joints[joint].parentInBone.q).getConjugate() *
                    input.bones[side][joint].q * input.legs[side].joints[joint].childInBone.q).getNormalized();
                travels[side][joint] = PxVec3(0);
                parent = input.bones[side][joint];
            }
            // 첫 샘플도 실제 관절 자세에서 이어 풀어 불필요한 해 전환을 줄인다.
            previous[side].jointRotations = rotations[side];
            previous[side].valid = true;
            previous[side].jointLimitsApplied = true;
            PxVec3 direction = input.bones[side][2].p - input.bones[side][0].p;
            PxVec3 bend = input.bones[side][1].p - input.bones[side][0].p;
            if (Unit(direction) && ProjectHint(bend, direction))
            {
                previous[side].bendDirection = bend;
            }
        }

        // 착지 자세가 관절 제한 안에서 불가능하면 앞선 일곱 궤적 지점을 풀 필요가 없다.
        // 후보에 이미 설정된 위치·기울기 허용 오차까지만 계산한다.
        for (size_t order = 0; order < legs.size(); ++order)
        {
            const size_t side = order == 0 ? moving : 1 - moving;
            FootIKLegInput& leg = legs[side];
            leg.actualAnchor = pelvisTarget;
            leg.targetSole = side == moving ? landing : input.supportSoles[side];
            const PxTransform delta = pelvisTarget * input.legs[side].actualAnchor.getInverse();
            const std::array<PxTransform, 3> seed{
                delta * input.bones[side][0], delta * input.bones[side][1], delta * input.bones[side][2]};
            const LegIKResult solved = SolveFootIKLeg(leg, seed, previous[side],
                settings.reachPositionTolerance, settings.reachTiltTolerance);
            result.reachError = (std::max)(result.reachError, solved.reachError);
            if (!solved.valid || !solved.jointLimitsApplied ||
                solved.reachError > settings.reachPositionTolerance ||
                solved.soleTiltError > settings.reachTiltTolerance)
            {
                return result;
            }
        }

        for (size_t sample = 1; sample <= Samples; ++sample)
        {
            const float progress = static_cast<float>(sample) / static_cast<float>(Samples);
            const float blend = progress * progress * (3.0f - 2.0f * progress);
            const PxTransform pelvis = InterpolateFootstepPelvis(result, progress);
            for (size_t side = 0; side < input.legs.size(); ++side)
            {
                FootIKLegInput& leg = legs[side];
                leg.actualAnchor = pelvis;
                leg.targetSole = input.supportSoles[side];
                if (side == moving)
                {
                    leg.targetSole.p = input.soles[side].p + (position - input.soles[side].p) * blend +
                        PxVec3(0, liftHeight * .5f * (1.0f - std::cos(PxTwoPi * progress)), 0);
                    leg.targetSole.q = InterpolateRotation(input.soles[side].q, landing.q, blend);
                }
                const PxTransform delta = pelvis * input.legs[side].actualAnchor.getInverse();
                const std::array<PxTransform, 3> seed{
                    delta * input.bones[side][0], delta * input.bones[side][1], delta * input.bones[side][2]};
                const LegIKResult solved = SolveFootIKLeg(leg, seed, previous[side],
                    settings.reachPositionTolerance, settings.reachTiltTolerance);
                result.reachError = (std::max)(result.reachError, solved.reachError);
                if (!solved.valid || !solved.jointLimitsApplied ||
                    solved.reachError > settings.reachPositionTolerance ||
                    solved.soleTiltError > settings.reachTiltTolerance)
                {
                    return result;
                }
                for (size_t joint = 0; joint < 3; ++joint)
                {
                    const PxVec3 rotationDelta = RotationVector(
                        solved.jointRotations[joint] * rotations[side][joint].getConjugate());
                    for (size_t axis = 0; axis < 3; ++axis)
                    {
                        // 잠긴 축의 작은 실제 오차는 물리 구속이 복원하며 구동 이동 시간에 포함하지 않는다.
                        if (input.lockedAxes[side][joint][axis])
                        {
                            continue;
                        }
                        travels[side][joint][axis] += PxAbs(rotationDelta[axis]);
                        const float limit = (std::min)(settings.maximumJointAcceleration,
                            input.jointAccelerations[side][joint][axis]);
                        const float requiredDuration = EstimateJointTravelDuration(travels[side][joint][axis],
                            input.jointVelocities[side][joint][axis], settings.maximumJointSpeed, limit);
                        timeScale = (std::max)(timeScale, requiredDuration < PX_MAX_F32 * (std::min)(duration, 1.0f) ?
                            requiredDuration / duration : PX_MAX_F32);
                    }
                    rotations[side][joint] = solved.jointRotations[joint];
                }
                previous[side] = solved;
            }
            // 이미 시간 예산을 넘은 궤적은 남은 수치 IK 계산을 생략한다.
            if (timeScale > 1.0f + 1e-6f)
            {
                result.timeLimited = true;
                result.requiredDuration = timeScale < PX_MAX_F32 / (std::max)(duration, 1.0f) ?
                    duration * timeScale : PX_MAX_F32;
                return result;
            }
        }
        // 접촉 충격까지 예측하는 물리 시뮬레이션은 아니다. 구동 한계와 샘플 궤적의 시간 예산 추정이다.
        result.requiredDuration = timeScale < PX_MAX_F32 / (std::max)(duration, 1.0f) ?
            duration * timeScale : PX_MAX_F32;
        result.reachable = true;
        return result;
    }
}

PxTransform PARS::InterpolateFootstepPelvis(const FootstepReachability& plan, float progress)
{
    const float t = PxClamp(progress, 0.0f, 1.0f);
    const float blend = t * t * (3.0f - 2.0f * t);
    // 수평 성분은 현재 속도의 예측을 유지하고, 낮추기와 회전만 부드럽게 보간한다.
    PxVec3 position = plan.pelvisStart.p + (plan.pelvisTarget.p - plan.pelvisStart.p) * t;
    position.y = plan.pelvisStart.p.y + (plan.pelvisTarget.p.y - plan.pelvisStart.p.y) * blend;
    return PxTransform(position, InterpolateRotation(plan.pelvisStart.q, plan.pelvisTarget.q, blend));
}

bool PARS::MakeFootstepSoleMotion(const PxTransform& start, const PxVec3& position,
    const PxVec3& normal, float progress, PxTransform& target)
{
    if (!std::isfinite(progress) || !MakeFootSoleTarget(position, normal, start, target))
    {
        return false;
    }
    const float t = PxClamp(progress, 0.0f, 1.0f);
    const float blend = t * t * (3.0f - 2.0f * t);
    target.q = InterpolateRotation(start.q, target.q, blend);
    return target.isSane();
}

FootstepReachability PARS::EvaluateFootstepReachability(const FootstepPlanningInput& input,
    const FootstepSettings& settings, size_t side, const PxVec3& position,
    const PxVec3& normal, float duration, float liftHeight)
{
    FootstepReachability best;
    if (side >= input.legs.size() || !input.legs[0].available || !input.legs[1].available ||
        !std::isfinite(duration) || duration <= 0 || !std::isfinite(liftHeight) || liftHeight < 0 ||
        !input.pelvisVelocity.isFinite() || !input.pelvisAngularVelocity.isFinite() ||
        !std::isfinite(input.maximumLowering) || input.maximumLowering < 0 ||
        !std::isfinite(input.loweringSpeed) || input.loweringSpeed < 0 ||
        !std::isfinite(input.loweringScale) || input.loweringScale < 0 ||
        !std::isfinite(settings.maximumJointSpeed) || settings.maximumJointSpeed <= 0 ||
        !std::isfinite(settings.maximumJointAcceleration) || settings.maximumJointAcceleration <= 0 ||
        !std::isfinite(settings.maximumPelvisAngularSpeed) || settings.maximumPelvisAngularSpeed <= 0 ||
        !std::isfinite(settings.maximumPelvisAngularAcceleration) || settings.maximumPelvisAngularAcceleration <= 0 ||
        !std::isfinite(settings.maximumPelvisYaw) || settings.maximumPelvisYaw < 0 ||
        !std::isfinite(settings.maximumPelvisTilt) || settings.maximumPelvisTilt < 0 ||
        !std::isfinite(settings.reachPositionTolerance) || settings.reachPositionTolerance <= 0 ||
        !std::isfinite(settings.reachTiltTolerance) || settings.reachTiltTolerance <= 0)
    {
        return best;
    }
    const PxTransform start = input.legs[side].actualAnchor;
    PxVec3 displacement = position - input.soles[side].p;
    displacement.y = 0;
    const float lowering = (std::min)(input.maximumLowering,
        (std::min)(input.loweringSpeed * duration / 1.5f, displacement.magnitude() * input.loweringScale));
    PxTransform target = start;
    // 착지까지의 실제 골반 수평 속도를 반영하고, 기존 지지 제어의 낮추기 범위 안에서 계획한다.
    target.p += PxVec3(input.pelvisVelocity.x * duration, -lowering, input.pelvisVelocity.z * duration);
    PxVec3 direction = position - start.p;
    direction.y = 0;
    if (!Unit(direction))
    {
        direction = input.forward;
    }
    PxVec3 forward = input.forward;
    forward.y = 0;
    if (!Unit(forward) || !Unit(direction))
    {
        return best;
    }
    const float heading = std::atan2(forward.cross(direction).y, forward.dot(direction));
    const float sign = PxAbs(heading) > 1e-4f ? (heading > 0 ? 1.0f : -1.0f) :
        (side == 0 ? 1.0f : -1.0f);
    const PxVec3 tiltAxis = PxVec3(0, 1, 0).cross(direction).getNormalized();
    const float rotationBudget = (std::min)(settings.maximumPelvisAngularSpeed * duration / 1.5f,
        settings.maximumPelvisAngularAcceleration * duration * duration / 6.0f);
    const float yawLimit = (std::min)(settings.maximumPelvisYaw, rotationBudget);
    const float tiltLimit = (std::min)(settings.maximumPelvisTilt, rotationBudget);
    const std::array<float, 9> yawScales{0, .5f, 1, -.5f, -1, 0, .5f, 1, -.5f};
    const std::array<float, 9> tiltScales{0, 0, 0, 0, 0, 1, 1, 1, 1};
    for (size_t attempt = 0; attempt < yawScales.size(); ++attempt)
    {
        const PxQuat yaw(yawScales[attempt] * sign * yawLimit, PxVec3(0, 1, 0));
        const PxQuat tilt(tiltScales[attempt] * tiltLimit, tiltAxis);
        target.q = (tilt * yaw * start.q).getNormalized();
        // 고정된 큰 각도만 시도하면 짧은 시간 예산에서 모든 회전 보조가 탈락한다.
        const float angle = RotationVector(target.q * start.q.getConjugate()).magnitude();
        if (angle > rotationBudget)
        {
            target.q = InterpolateRotation(start.q, target.q, rotationBudget / angle);
        }
        const FootstepReachability candidate = EvaluatePelvisPlan(
            input, settings, side, position, normal, duration, liftHeight, target);
        if (candidate.reachable && (!best.reachable || candidate.requiredDuration < best.requiredDuration))
        {
            best = candidate;
        }
        else if (!best.reachable && candidate.timeLimited &&
            (!best.timeLimited || candidate.requiredDuration < best.requiredDuration))
        {
            best = candidate;
        }
        // 회전 없이 가능하면 그대로 사용하고, 필요한 경우에만 골반 보조를 채택한다.
        if (candidate.reachable && candidate.requiredDuration <= duration + 1e-6f)
        {
            return candidate;
        }
    }
    return best;
}

bool PARS::MakeFootSoleTarget(const PxVec3& position, const PxVec3& normal,
    const PxTransform& referenceSole, PxTransform& target)
{
    PxVec3 up = normal;
    if (!position.isFinite() || !referenceSole.isSane() || !Unit(up))
    {
        return false;
    }
    PxVec3 forward = referenceSole.q.rotate(PxVec3(0, 0, 1));
    if (!ProjectHint(forward, up))
    {
        PxVec3 right = referenceSole.q.rotate(PxVec3(1, 0, 0));
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
    FramePerformance::Scope ikTiming(FramePerformance::Category::FootstepIK);
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
    if (pose.empty() || pose.size() != parents.size() || (input.hasPelvisPlan && !input.plannedPelvis.isSane()))
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
        FootIKLegInput leg = input.legs[side];
        // 골반 보조 중에는 양쪽 다리를 같은 계획 골반에서 풀어 착지점과 지지발을 함께 유지한다.
        if (input.hasPelvisPlan)
        {
            leg.actualAnchor = input.plannedPelvis;
        }
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
        if (leg.clampJointRotation &&
            !ApplyLegJointLimits(leg, previousLegs[side], result, PositionTolerance, TiltTolerance))
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
                const PxTransform anchor = input.hasPelvisPlan ? input.plannedPelvis : leg.actualAnchor;
                const PxTransform toPhysical = anchor * pose[leg.anchorNode].getInverse();
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
        const PxTransform anchor = input.hasPelvisPlan ? input.plannedPelvis : input.legs[side].actualAnchor;
        m_PreviousBendInAnchor[side] = anchor.q.rotateInv(m_Snapshot.legs[side].bendDirection);
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
        return u8"입력 다리 자세";
    case KneeHintSource::Previous:
        return u8"이전 방향";
    case KneeHintSource::Reference:
        return u8"기준 자세";
    default:
        return u8"수치 대체 방향";
    }
}
