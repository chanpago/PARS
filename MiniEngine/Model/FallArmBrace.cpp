#include "FallArmBrace.h"
#include "foundation/PxMat33.h"
#include "foundation/PxMathUtils.h"
#include <algorithm>
#include <cmath>

using namespace PARS;
using namespace physx;

namespace
{
    constexpr float ArmReachRatio = .98f;

    bool Normalize(PxVec3& value)
    {
        const float length = value.magnitude();
        if (!value.isFinite() || !std::isfinite(length) || length < 1e-6f)
        {
            return false;
        }
        value /= length;
        return true;
    }

    PxQuat BlendRotation(const PxQuat& from, PxQuat to, float weight)
    {
        if (from.dot(to) < 0)
        {
            to = -to;
        }
        return (from * (1 - weight) + to * weight).getNormalized();
    }

    bool Ancestor(const std::vector<int32_t>& parents, int ancestor, int node)
    {
        if (ancestor < 0 || node < 0 || static_cast<size_t>(node) >= parents.size())
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

    bool ValidArmPose(const std::vector<int32_t>& parents, const std::vector<PxTransform>& pose)
    {
        if (pose.empty() || parents.size() != pose.size())
        {
            return false;
        }
        for (size_t node = 0; node < pose.size(); ++node)
        {
            if (parents[node] < -1 || parents[node] >= static_cast<int>(node) || !pose[node].isSane())
            {
                return false;
            }
        }
        return true;
    }

    bool ValidArm(const FallArmSample& arm, const std::vector<int32_t>& parents)
    {
        const FallArmSetup& setup = arm.setup;
        return setup.available && Ancestor(parents, setup.anchorNode, setup.shoulderNode) &&
            Ancestor(parents, setup.shoulderNode, setup.elbowNode) &&
            Ancestor(parents, setup.elbowNode, setup.handNode) &&
            std::isfinite(setup.upperLength) && setup.upperLength > 1e-5f &&
            std::isfinite(setup.lowerLength) && setup.lowerLength > 1e-5f &&
            arm.actualAnchor.isSane() && arm.actualHand.isSane() && setup.palmInHand.isSane() &&
            setup.forwardInAnchor.isFinite() && setup.rightInAnchor.isFinite();
    }

    bool SolveClavicle(const FallArmSample& arm, const ImpactPalmGoal& goal, const PxVec3& wrist,
        const std::vector<int32_t>& parents, const std::vector<PxTransform>& source, PxTransform& clavicle)
    {
        const FallArmSetup& setup = arm.setup;
        if (!goal.limitClavicleRotation || !std::isfinite(goal.clavicleReachFraction) || goal.clavicleReachFraction <= 0 ||
            setup.clavicleJoint < 0 || setup.anchorNode != setup.clavicleNode || !Ancestor(parents, setup.chestNode, setup.clavicleNode) ||
            !arm.actualChest.isSane())
        {
            return false;
        }
        const PxTransform& chest = source[setup.chestNode];
        const PxTransform base = chest.transformInv(source[setup.clavicleNode]);
        const PxVec3 shoulder = chest.transformInv(source[setup.shoulderNode].p);
        const PxVec3 span = shoulder - base.p;
        PxVec3 axis = span;
        if (!Normalize(axis))
        {
            return false;
        }
        // 어깨를 쇄골 축에 수직인 방향으로만 보조해 본 길이와 원래의 축 비틀림을 보존한다.
        const PxVec3 toward = arm.actualChest.transformInv(wrist) - shoulder;
        PxVec3 tangent = toward - axis * axis.dot(toward);
        if (tangent.magnitudeSquared() < 1e-12f)
        {
            clavicle = source[setup.clavicleNode];
            return true;
        }
        // 가슴을 가로지를수록 낮은 손목이 어깨까지 끌어내리지 않게 앞뒤 이동의 일부를 올림에 나눈다.
        const PxVec3& up = setup.upInChest;
        PxVec3 outward = axis - up * axis.dot(up);
        if (Normalize(outward))
        {
            const float inward = PxClamp(-outward.dot(toward) / toward.magnitude(), 0.0f, 1.0f);
            const float sweep = PxAbs(tangent.dot(outward.cross(up)));
            const float lift = inward * PxMax(0.0f, sweep - tangent.dot(up));
            tangent += (up - axis * axis.dot(up)) * lift;
        }
        PxVec3 direction = span + tangent * PxMin(goal.clavicleReachFraction, 1.0f);
        if (!Normalize(direction))
        {
            return false;
        }
        const PxQuat swing = PxShortestRotation(axis, direction);
        const PxQuat limited = goal.limitClavicleRotation(setup, (swing * base.q).getNormalized());
        if (!limited.isUnit())
        {
            return false;
        }
        clavicle = chest * PxTransform(base.p, limited);
        return true;
    }

    bool ApplyArmPalmTarget(const FallArmSample& arm, int side, const ImpactPalmGoal& goal, const PxTransform& palm,
        const std::vector<int32_t>& parents, std::vector<PxTransform>& pose, FallArmSnapshot& targets)
    {
        const FallArmSetup& setup = arm.setup;
        const std::vector<PxTransform> source = pose;
        PxTransform toPhysical = arm.actualAnchor * source[setup.anchorNode].getInverse();
        PxTransform armToPhysical = toPhysical;
        PxTransform clavicle(PxIdentity);
        const PxVec3 wrist = (palm * setup.palmInHand.getInverse()).p;
        const bool useClavicle = SolveClavicle(arm, goal, wrist, parents, source, clavicle);
        if (useClavicle)
        {
            // 같은 가슴 기준에서 쇄골 보정과 팔 IK를 풀어 보정한 어깨 이동이 상쇄되지 않게 한다.
            toPhysical = arm.actualChest * source[setup.chestNode].getInverse();
            armToPhysical = toPhysical * clavicle * source[setup.clavicleNode].getInverse();
        }
        const PxQuat anchorRotation = useClavicle ? toPhysical.q * source[setup.anchorNode].q : arm.actualAnchor.q;
        const PxVec3 right = anchorRotation.rotate(setup.rightInAnchor);
        const PxVec3 forward = anchorRotation.rotate(setup.forwardInAnchor);
        const PxVec3 up = useClavicle ? arm.actualChest.q.rotate(setup.upInChest) : PxVec3(0, 1, 0);
        const float sign = side == 0 ? -1.0f : 1.0f;
        TwoBoneIKInput solve;
        solve.hip = armToPhysical * source[setup.shoulderNode];
        solve.knee = armToPhysical * source[setup.elbowNode];
        solve.ankle = armToPhysical * source[setup.handNode];
        solve.soleInBone = setup.palmInHand;
        solve.targetSole = palm;
        solve.thighLength = setup.upperLength;
        solve.calfLength = setup.lowerLength;
        solve.maximumReachRatio = ArmReachRatio;
        solve.preferReferenceBend = true;
        // 등 뒤에서 같은 손의 손바닥이 몸으로 향하도록 팔꿈치를 뒤쪽으로 굽힌다.
        solve.referenceBend = goal.selection.onBack && side == goal.selection.hand ?
            right * (sign * .65f) - forward * 1.2f - up * .3f : right * sign - forward * .25f - up * .4f;
        const LegIKResult result = SolveTwoBoneIK(solve);
        if (!result.valid)
        {
            return false;
        }

        const PxTransform toAnimation = toPhysical.getInverse();
        std::vector<PxTransform> desired = source;
        std::vector<bool> changed(pose.size(), false);
        if (useClavicle)
        {
            desired[setup.clavicleNode] = clavicle;
            changed[setup.clavicleNode] = true;
        }
        desired[setup.shoulderNode] = toAnimation * result.hip;
        desired[setup.elbowNode] = toAnimation * result.knee;
        desired[setup.handNode] = toAnimation * result.ankle;
        changed[setup.shoulderNode] = true;
        changed[setup.elbowNode] = true;
        changed[setup.handNode] = true;
        const float weight = goal.weight * goal.weight * (3 - 2 * goal.weight);
        // 손가락 등 자식의 로컬 자세를 보존한 다음 로컬 회전만 보간해 뼈 길이를 유지한다.
        for (size_t node = 0; node < pose.size(); ++node)
        {
            const int parent = parents[node];
            if (parent < 0)
            {
                continue;
            }
            const PxTransform local = source[parent].getInverse() * source[node];
            if (!changed[node] && changed[parent])
            {
                desired[node] = desired[parent] * local;
                changed[node] = true;
            }
            if (changed[node])
            {
                const PxTransform targetLocal = desired[parent].getInverse() * desired[node];
                pose[node] = pose[parent] * PxTransform(local.p, BlendRotation(local.q, targetLocal.q, weight));
                targets.correctedNodes.push_back(static_cast<int>(node));
            }
        }
        FallArmTarget& target = targets.arms[side];
        target.palm = palm;
        target.solvedPalm = (toPhysical * pose[setup.handNode] * setup.palmInHand).p;
        target.elbow = result.knee.p;
        target.weight = goal.weight;
        target.reachError = result.reachError;
        target.reachLimited = result.reachLimited;
        target.hasTarget = true;
        target.applied = true;
        return true;
    }
}

bool PARS::ImpactPalmGoal::SampleTiming(float hitAge, float approachAge, bool sustainedWound, float gripAge)
{
    constexpr float releaseTime = .25f;
    const float releaseStart = sustainedWound ? GetWoundReleaseStart(gripAge) : FullReachTime + .55f;
    approach = PxClamp((approachAge - RecoilTime) / ReachTime, 0.0f, 1.0f);
    weight = PxMin(approach, PxClamp(1.0f - (hitAge - releaseStart) / releaseTime, 0.0f, 1.0f));
    return hitAge < releaseStart + releaseTime;
}

WoundHandSelection PARS::SelectWoundHand(const PxVec3& point, const PxVec3& normal, const PxVec3& leftShoulder,
    const PxVec3& rightShoulder, const PxVec3& pelvis)
{
    const PxVec3 center = (leftShoulder + rightShoulder) * .5f;
    PxVec3 up = center - pelvis;
    if (!point.isFinite() || !normal.isFinite() || !Normalize(up))
    {
        return {};
    }
    PxVec3 right = rightShoulder - leftShoulder;
    right -= up * right.dot(up);
    if (!Normalize(right))
    {
        return {};
    }
    const PxVec3 forward = up.cross(right);
    const PxVec3 offset = point - center;
    const float lateral = offset.dot(right);
    if (PxAbs(lateral) <= 1e-5f)
    {
        return {};
    }
    // 평평한 가슴과 등에서도 좌우는 피격점으로, 등 쪽 면인지는 표면 법선으로 구분한다.
    float facing = normal.dot(forward);
    float sideways = normal.dot(right);
    if (facing * facing + sideways * sideways < 1e-6f)
    {
        // 윗면이나 아랫면처럼 수평 법선이 없으면 피격점이 위치한 방향을 쓴다.
        facing = offset.dot(forward);
        sideways = lateral;
    }
    const int hitSide = lateral > 0 ? 1 : 0;
    const bool onBack = -facing > PxAbs(sideways) + 1e-5f;
    return {onBack ? hitSide : 1 - hitSide, onBack};
}

bool PARS::ApplyImpactPalmTarget(ImpactPalmGoal& goal, const std::array<FallArmSample, 2>& arms,
    const std::vector<int32_t>& parents, std::vector<PxTransform>& pose, FallArmSnapshot& targets)
{
    targets = {};
    PxVec3 normal = goal.normal;
    if (!ValidArmPose(parents, pose) || goal.hitNode < 0 || static_cast<size_t>(goal.hitNode) >= parents.size() ||
        !goal.point.isFinite() || !Normalize(normal) || !std::isfinite(goal.weight) || goal.weight <= 0 || goal.weight > 1 ||
        !std::isfinite(goal.approach) || goal.approach < 0 || goal.approach > 1)
    {
        return false;
    }
    // 접근 중에는 몸 바깥을 거쳐 들어가고, 닿은 뒤에는 손바닥 면만 표면에 댄다.
    const float approachOffset = .10f * (1.0f - goal.approach);
    const PxVec3 palmPosition = goal.point + normal * (.006f + approachOffset);
    float leastExcess = PX_MAX_F32;
    float nearestPalmSquared = PX_MAX_F32;
    int selected = -1;
    PxTransform palm(PxIdentity);
    for (int side = 0; side < static_cast<int>(arms.size()); ++side)
    {
        const FallArmSample& arm = arms[side];
        const FallArmSetup& setup = arm.setup;
        if (!ValidArm(arm, parents) || (goal.hand >= 0 && side != goal.hand) ||
            goal.hitNode == setup.clavicleNode || goal.hitNode == setup.shoulderNode || Ancestor(parents, setup.shoulderNode, goal.hitNode))
        {
            continue;
        }
        const PxTransform mapping = arm.actualAnchor * pose[setup.anchorNode].getInverse();
        const PxTransform elbow = mapping * pose[setup.elbowNode];
        const PxTransform actualPalm = arm.actualHand * setup.palmInHand;
        PxVec3 fingers = goal.point - elbow.p;
        fingers -= normal * normal.dot(fingers);
        PxTransform candidate(PxIdentity);
        if (fingers.normalize() > 1e-4f)
        {
            candidate = PxTransform(palmPosition, PxQuat(PxMat33(normal.cross(fingers), normal, fingers)).getNormalized());
        }
        else if (!MakeFootSoleTarget(palmPosition, normal, actualPalm, candidate))
        {
            continue;
        }
        if (side == goal.selection.hand)
        {
            if (goal.selection.onBack && goal.approach < 1.0f)
            {
                // 접근 중에는 옆구리 바깥을 돌아가고, 접촉 시에는 원래 상처 위치로 수렴한다.
                const float outward = setup.upperLength * (1.0f - goal.approach * goal.approach);
                const float sign = side == 0 ? -1.0f : 1.0f;
                candidate.p += arm.actualAnchor.q.rotate(setup.rightInAnchor) * (sign * outward);
            }
            selected = side;
            palm = candidate;
            break;
        }
        const PxTransform shoulder = mapping * pose[setup.shoulderNode];
        const PxVec3 wristAtContact = (candidate * setup.palmInHand.getInverse()).p - normal * approachOffset;
        const float reach = (setup.upperLength + setup.lowerLength) * ArmReachRatio;
        const float excess = PxMax(0.0f, (wristAtContact - shoulder.p).magnitude() - reach);
        const float palmDistanceSquared = (goal.point - actualPalm.p).magnitudeSquared();
        if (excess < leastExcess || (excess == leastExcess && palmDistanceSquared < nearestPalmSquared))
        {
            leastExcess = excess;
            nearestPalmSquared = palmDistanceSquared;
            selected = side;
            palm = candidate;
        }
    }
    if (selected < 0)
    {
        return false;
    }
    targets.active = ApplyArmPalmTarget(arms[selected], selected, goal, palm, parents, pose, targets);
    if (!targets.active)
    {
        return false;
    }
    goal.hand = selected;
    return true;
}
