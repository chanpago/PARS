#include "FallArmBrace.h"
#include "foundation/PxMat33.h"
#include <algorithm>
#include <cmath>

using namespace PARS;
using namespace physx;

namespace
{
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

    bool ApplyArmPalmTargets(const std::array<FallArmSample, 2>& arms, const std::array<PxVec3, 2>& elbowHints,
        float reachRatio, const std::vector<int32_t>& parents, std::vector<PxTransform>& pose, FallArmSnapshot& targets)
    {
        targets.correctedNodes.clear();
        const std::vector<PxTransform> source = pose;
        std::vector<PxTransform> desired = pose;
        std::vector<float> weights(pose.size(), 0);
        std::vector<bool> changed(pose.size(), false);
        for (size_t side = 0; side < arms.size(); ++side)
        {
            const FallArmSample& arm = arms[side];
            const FallArmSetup& setup = arm.setup;
            FallArmTarget& target = targets.arms[side];
            target.applied = false;
            if (!target.hasTarget || target.weight <= 0)
            {
                continue;
            }
            const float smoothWeight = target.weight * target.weight * (3 - 2 * target.weight);
            const PxTransform toPhysical = arm.actualAnchor * source[setup.anchorNode].getInverse();
            TwoBoneIKInput solve;
            solve.hip = toPhysical * source[setup.shoulderNode];
            solve.knee = toPhysical * source[setup.elbowNode];
            solve.ankle = toPhysical * source[setup.handNode];
            solve.soleInBone = setup.palmInHand;
            solve.targetSole = target.palm;
            solve.thighLength = setup.upperLength;
            solve.calfLength = setup.lowerLength;
            solve.maximumReachRatio = reachRatio;
            solve.preferReferenceBend = true;
            solve.referenceBend = elbowHints[side];
            const LegIKResult result = SolveTwoBoneIK(solve);
            if (!result.valid)
            {
                target = {};
                continue;
            }
            const PxTransform toAnimation = toPhysical.getInverse();
            desired[setup.shoulderNode] = toAnimation * result.hip;
            desired[setup.elbowNode] = toAnimation * result.knee;
            desired[setup.handNode] = toAnimation * result.ankle;
            for (int node : {setup.shoulderNode, setup.elbowNode, setup.handNode})
            {
                changed[node] = true;
                weights[node] = smoothWeight;
            }
            target.elbow = result.knee.p;
            target.reachError = result.reachError;
            target.reachLimited = result.reachLimited;
            target.applied = true;
        }
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
                weights[node] = weights[parent];
            }
            if (changed[node])
            {
                const PxTransform targetLocal = desired[parent].getInverse() * desired[node];
                pose[node] = pose[parent] * PxTransform(local.p, BlendRotation(local.q, targetLocal.q, weights[node]));
                targets.correctedNodes.push_back(static_cast<int>(node));
            }
        }
        for (size_t side = 0; side < arms.size(); ++side)
        {
            FallArmTarget& target = targets.arms[side];
            if (target.applied)
            {
                const FallArmSample& arm = arms[side];
                const PxTransform toPhysical = arm.actualAnchor * source[arm.setup.anchorNode].getInverse();
                target.solvedPalm = (toPhysical * pose[arm.setup.handNode] * arm.setup.palmInHand).p;
            }
        }
        return !targets.correctedNodes.empty();
    }
}

bool PARS::ApplyImpactPalmTarget(ImpactPalmGoal& goal, const std::array<FallArmSample, 2>& arms,
    const std::vector<int32_t>& parents, std::vector<PxTransform>& pose, FallArmSnapshot& targets)
{
    constexpr float reachRatio = .98f;
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
        const PxTransform shoulder = mapping * pose[setup.shoulderNode];
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
        const PxVec3 wristAtContact = (candidate * setup.palmInHand.getInverse()).p - normal * approachOffset;
        const float reach = (setup.upperLength + setup.lowerLength) * reachRatio;
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
    const FallArmSample& arm = arms[selected];
    const float sign = selected == 0 ? -1.0f : 1.0f;
    const PxVec3 right = arm.actualAnchor.q.rotate(arm.setup.rightInAnchor);
    const PxVec3 forward = arm.actualAnchor.q.rotate(arm.setup.forwardInAnchor);
    std::array<PxVec3, 2> elbowHints{};
    elbowHints[selected] = right * sign - forward * .25f - PxVec3(0, .4f, 0);
    FallArmTarget& target = targets.arms[selected];
    target.hasTarget = true;
    target.palm = palm;
    target.weight = goal.weight;
    targets.active = ApplyArmPalmTargets(arms, elbowHints, reachRatio, parents, pose, targets);
    if (!targets.active)
    {
        return false;
    }
    goal.hand = selected;
    return true;
}
