#include "FallArmBrace.h"
#include "FootSupport.h"
#include "foundation/PxMat33.h"
#include "foundation/PxMathUtils.h"
#include <algorithm>
#include <cmath>

using namespace PARS;
using namespace physx;

namespace
{
    constexpr float ArmReachRatio = .98f;
    constexpr float PalmSurfaceOffset = .006f;
    constexpr float WoundPoseTolerance = .02f;
    constexpr float WoundClearance = .01f;

    struct ArmReachPose
    {
    public:
        PxTransform toPhysical{PxIdentity};
        PxTransform clavicle{PxIdentity};
        std::array<PxTransform, 3> bones;
        bool usesClavicle = false;
        bool reachLimited = false;
    };

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

    bool LimitJointPose(const ImpactPalmGoal& goal, const FallArmJoint& joint, const PxTransform& parent,
        const PxQuat& desired, PxTransform& child)
    {
        const PxTransform pivot = parent * joint.inParentBone;
        const PxQuat raw = (pivot.q.getConjugate() * desired * joint.inChildBone.q).getNormalized();
        const PxQuat limited = goal.limitJointRotation(joint.index, raw);
        if (!limited.isUnit())
        {
            return false;
        }
        // 실제 물리 관절의 피벗을 맞춰 제한 후 자식 본의 위치와 회전을 함께 복원한다.
        child = pivot * PxTransform(limited) * joint.inChildBone.getInverse();
        return child.isSane();
    }

    bool SolveClavicle(const FallArmSample& arm, const ImpactPalmGoal& goal, const PxVec3& wrist,
        const std::vector<int32_t>& parents, const std::vector<PxTransform>& source, PxTransform& clavicle)
    {
        const FallArmSetup& setup = arm.setup;
        if (!goal.limitJointRotation || !std::isfinite(goal.clavicleReachFraction) || goal.clavicleReachFraction <= 0 ||
            setup.clavicleJoint.index < 0 || setup.anchorNode != setup.clavicleNode || !Ancestor(parents, setup.chestNode, setup.clavicleNode) ||
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
            return LimitJointPose(goal, setup.clavicleJoint, chest, source[setup.clavicleNode].q, clavicle);
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
        return LimitJointPose(goal, setup.clavicleJoint, chest, chest.q * swing * base.q, clavicle);
    }

    bool SolveArmReach(const FallArmSample& arm, int side, const ImpactPalmGoal& goal, const PxTransform& palm,
        const std::vector<int32_t>& parents, const std::vector<PxTransform>& source, ArmReachPose& reach)
    {
        const FallArmSetup& setup = arm.setup;
        reach.toPhysical = arm.actualAnchor * source[setup.anchorNode].getInverse();
        PxTransform armToPhysical = reach.toPhysical;
        const PxVec3 wrist = (palm * setup.palmInHand.getInverse()).p;
        reach.usesClavicle = SolveClavicle(arm, goal, wrist, parents, source, reach.clavicle);
        if (reach.usesClavicle)
        {
            // 같은 가슴 기준에서 쇄골 보정과 팔 IK를 풀어 보정한 어깨 이동이 상쇄되지 않게 한다.
            reach.toPhysical = arm.actualChest * source[setup.chestNode].getInverse();
            armToPhysical = reach.toPhysical * reach.clavicle * source[setup.clavicleNode].getInverse();
        }
        const PxQuat anchorRotation = reach.usesClavicle ? reach.toPhysical.q * source[setup.anchorNode].q : arm.actualAnchor.q;
        const PxVec3 right = anchorRotation.rotate(setup.rightInAnchor);
        const PxVec3 forward = anchorRotation.rotate(setup.forwardInAnchor);
        const PxVec3 up = reach.usesClavicle ? arm.actualChest.q.rotate(setup.upInChest) : PxVec3(0, 1, 0);
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
        reach.reachLimited = result.reachLimited;
        reach.bones = {result.hip, result.knee, result.ankle};
        if (goal.limitJointRotation)
        {
            const PxTransform anchor = armToPhysical * source[setup.anchorNode];
            for (size_t link = 0; link < reach.bones.size(); ++link)
            {
                const FallArmJoint& joint = setup.armJoints[link];
                const PxTransform& parent = link == 0 ? anchor : reach.bones[link - 1];
                if (joint.index >= 0 && !LimitJointPose(goal, joint, parent, reach.bones[link].q, reach.bones[link]))
                {
                    return false;
                }
            }
        }
        return true;
    }

    float HandSurfaceGap(const FallArmHandShape& shape, const PxTransform& hand, const PxVec3& point, const PxVec3& normal)
    {
        const PxTransform body = hand * shape.inBone;
        return (body.p - point).dot(normal) - shape.GetHalfExtent(body.q.rotateInv(normal));
    }

    using ArmRotations = std::array<PxQuat, 3>;
    using PalmError = std::array<float, 6>;

    PxTransform EvaluateArm(const FallArmSetup& setup, const PxTransform& clavicle, const ArmRotations& rotations)
    {
        PxTransform bone = clavicle;
        for (size_t link = 0; link < rotations.size(); ++link)
        {
            const FallArmJoint& joint = setup.armJoints[link];
            bone = bone * joint.inParentBone * PxTransform(rotations[link]) * joint.inChildBone.getInverse();
        }
        return bone * setup.palmInHand;
    }

    PalmError MeasureError(const PxTransform& pose, const PxTransform& target, float normalWeight)
    {
        const PxVec3 delta = pose.p - target.p;
        const PxVec3 normalDelta = (pose.q.getBasisVector1() - target.q.getBasisVector1()) * normalWeight;
        return {delta.x, delta.y, delta.z, normalDelta.x, normalDelta.y, normalDelta.z};
    }

    float Cost(const PalmError& residual)
    {
        float value = 0;
        for (float component : residual)
        {
            value += component * component;
        }
        return value;
    }

    PxQuat UpdateRotation(const FallArmJoint& joint, const ImpactPalmGoal& goal, const PxQuat& current, const PxVec3& delta)
    {
        const float angle = delta.magnitude();
        if (angle < 1e-9f)
        {
            return current;
        }
        return goal.limitJointRotation(joint.index, (PxQuat(angle, delta / angle) * current).getNormalized());
    }

    // 댐핑된 6차 정규방정식을 부분 피벗으로 푼다.
    bool ReduceSystem(float (&matrix)[6][7])
    {
        for (int i = 0; i < 6; ++i)
        {
            int pivot = i;
            for (int k = i + 1; k < 6; ++k)
            {
                if (PxAbs(matrix[k][i]) > PxAbs(matrix[pivot][i]))
                {
                    pivot = k;
                }
            }
            if (PxAbs(matrix[pivot][i]) < 1e-12f)
            {
                return false;
            }
            if (pivot != i)
            {
                for (int j = i; j < 7; ++j)
                {
                    std::swap(matrix[i][j], matrix[pivot][j]);
                }
            }
            const float scale = matrix[i][i];
            for (int j = i; j < 7; ++j)
            {
                matrix[i][j] /= scale;
            }
            for (int row = 0; row < 6; ++row)
            {
                if (row != i)
                {
                    const float fraction = matrix[row][i];
                    for (int k = i; k < 7; ++k)
                    {
                        matrix[row][k] -= fraction * matrix[i][k];
                    }
                }
            }
        }
        return true;
    }

    void RefineContact(const FallArmSetup& setup, const ImpactPalmGoal& goal, const PxTransform& anchor,
        const PxTransform& target, float normalWeight, ArmRotations& angles)
    {
        PxTransform pose = EvaluateArm(setup, anchor, angles);
        float damping = .0002f;
        for (int iteration = 0; iteration < 4; ++iteration)
        {
            const PalmError error = MeasureError(pose, target, normalWeight);
            const float previousCost = Cost(error);
            if (PalmFitsWound(pose, target, WoundPoseTolerance))
            {
                break;
            }
            // PA 제한을 지난 양방향 미분으로 잠긴 축과 관절 끝 범위를 반영한다.
            float jacobian[6][9]{};
            constexpr float step = .004f;
            for (int column = 0; column < 9; ++column)
            {
                const int link = column / 3;
                const int axis = column % 3;
                PxVec3 delta(0);
                delta[axis] = step;
                ArmRotations plus = angles;
                ArmRotations minus = angles;
                plus[link] = UpdateRotation(setup.armJoints[link], goal, angles[link], delta);
                minus[link] = UpdateRotation(setup.armJoints[link], goal, angles[link], -delta);
                const PalmError raised = MeasureError(EvaluateArm(setup, anchor, plus), target, normalWeight);
                const PalmError lowered = MeasureError(EvaluateArm(setup, anchor, minus), target, normalWeight);
                for (int row = 0; row < 6; ++row)
                {
                    jacobian[row][column] = (raised[row] - lowered[row]) / (2.0f * step);
                }
            }
            float matrix[6][7]{};
            for (int row = 0; row < 6; ++row)
            {
                for (int col = 0; col < 6; ++col)
                {
                    for (int axis = 0; axis < 9; ++axis)
                    {
                        matrix[row][col] += jacobian[row][axis] * jacobian[col][axis];
                    }
                }
                matrix[row][row] += damping;
                matrix[row][6] = -error[row];
            }
            if (!ReduceSystem(matrix))
            {
                break;
            }
            std::array<PxVec3, 3> changes{PxVec3(0), PxVec3(0), PxVec3(0)};
            for (int axis = 0; axis < 9; ++axis)
            {
                for (int row = 0; row < 6; ++row)
                {
                    changes[axis / 3][axis % 3] += jacobian[row][axis] * matrix[row][6];
                }
            }
            float maximum = .35f;
            for (const PxVec3& change : changes)
            {
                maximum = PxMax(maximum, change.magnitude());
            }
            bool improved = false;
            for (float fraction = .35f / maximum; fraction >= .002f; fraction *= .5f)
            {
                ArmRotations trial = angles;
                for (size_t link = 0; link < trial.size(); ++link)
                {
                    trial[link] = UpdateRotation(setup.armJoints[link], goal, angles[link], fraction * changes[link]);
                }
                const PxTransform next = EvaluateArm(setup, anchor, trial);
                if (Cost(MeasureError(next, target, normalWeight)) < previousCost - 1e-12f)
                {
                    angles = trial;
                    pose = next;
                    improved = true;
                    break;
                }
            }
            if (improved)
            {
                damping = PxMax(.00001f, damping * .65f);
            }
            else
            {
                damping *= 5;
                if (damping > .8f)
                {
                    break;
                }
            }
        }
    }

    void RefineWoundArm(const FallArmSample& arm, int side, const ImpactPalmGoal& goal, const PxTransform& palm, ArmReachPose& reach)
    {
        const FallArmSetup& setup = arm.setup;
        const PxTransform anchor = reach.usesClavicle ? reach.toPhysical * reach.clavicle : arm.actualAnchor;
        ArmRotations rotations;
        PxTransform parent = anchor;
        for (size_t link = 0; link < rotations.size(); ++link)
        {
            const FallArmJoint& joint = setup.armJoints[link];
            rotations[link] = ((parent * joint.inParentBone).q.getConjugate() * reach.bones[link].q * joint.inChildBone.q).getNormalized();
            parent = reach.bones[link];
        }
        // 멀리서는 위치를 우선하고, 가까워지면 손바닥 면을 맞추는 오차의 비중을 높인다.
        const float distance = (arm.actualHand.transform(setup.palmInHand.p) - palm.p).magnitude();
        const float normalWeight = .025f + .085f * PxClamp((.25f - distance) / .20f, 0.0f, 1.0f);
        PxTransform candidate = reach.bones.back() * setup.palmInHand;
        if (goal.history && goal.history->hand == side)
        {
            ArmRotations previousRotations = goal.history->rotations;
            for (size_t link = 0; link < previousRotations.size(); ++link)
            {
                previousRotations[link] = goal.limitJointRotation(setup.armJoints[link].index, previousRotations[link]);
            }
            const PxTransform previous = EvaluateArm(setup, anchor, previousRotations);
            if (Cost(MeasureError(previous, palm, normalWeight)) <= Cost(MeasureError(candidate, palm, normalWeight)))
            {
                rotations = previousRotations;
                candidate = previous;
            }
        }
        if (!PalmFitsWound(candidate, palm, WoundPoseTolerance))
        {
            RefineContact(setup, goal, anchor, palm, normalWeight, rotations);
        }
        if (goal.history)
        {
            goal.history->rotations = rotations;
            goal.history->hand = side;
        }
        parent = anchor;
        for (size_t link = 0; link < rotations.size(); ++link)
        {
            const FallArmJoint& joint = setup.armJoints[link];
            reach.bones[link] = parent * joint.inParentBone * PxTransform(rotations[link]) * joint.inChildBone.getInverse();
            parent = reach.bones[link];
        }
    }

    void TransformBranch(int root, const PxTransform& delta, const std::vector<int32_t>& parents,
        const std::vector<PxTransform>& source, std::vector<PxTransform>& pose, std::vector<bool>& changed)
    {
        for (int node = root; node < static_cast<int>(pose.size()); ++node)
        {
            if (node == root || Ancestor(parents, root, node))
            {
                pose[node] = delta * source[node];
                changed[node] = true;
            }
        }
    }

    void TransformBranch(int root, const PxTransform& delta, const std::vector<int32_t>& parents,
        std::vector<PxTransform>& pose, std::vector<bool>& changed)
    {
        TransformBranch(root, delta, parents, pose, pose, changed);
    }

    void RotateBranch(int root, const PxVec3& pivot, const PxQuat& rotation, const std::vector<int32_t>& parents,
        std::vector<PxTransform>& pose, std::vector<bool>& changed)
    {
        TransformBranch(root, PxTransform(pivot - rotation.rotate(pivot), rotation), parents, pose, changed);
    }

    void LowerWoundPelvis(const ImpactPalmGoal& goal, const PxVec3& shift, const PxVec3& forward,
        const std::vector<int32_t>& parents, std::vector<PxTransform>& pose, std::vector<bool>& changed)
    {
        const std::array<const FootLegSetup*, 2> legs{goal.injuredLeg, goal.oppositeLeg};
        std::array<LegIKResult, 2> solutions;
        for (size_t side = 0; side < legs.size(); ++side)
        {
            const FootLegSetup* leg = legs[side];
            if (!leg || !leg->available || !Ancestor(parents, goal.pelvisNode, leg->hipNode) ||
                !Ancestor(parents, leg->hipNode, leg->kneeNode) || !Ancestor(parents, leg->kneeNode, leg->ankleNode))
            {
                return;
            }
            TwoBoneIKInput input;
            input.hip = pose[leg->hipNode];
            input.knee = pose[leg->kneeNode];
            input.ankle = pose[leg->ankleNode];
            input.targetSole = input.ankle;
            input.hip.p += shift;
            input.knee.p += shift;
            input.ankle.p += shift;
            input.thighLength = leg->thighLength;
            input.calfLength = leg->calfLength;
            input.referenceBend = forward;
            solutions[side] = SolveTwoBoneIK(input);
            if (!solutions[side].valid || solutions[side].reachError > .002f)
            {
                return;
            }
        }
        // 발딛기에서 정한 양 발목 위치와 방향을 지킬 수 있을 때만 골반을 뒤로 낮춘다.
        TransformBranch(goal.pelvisNode, PxTransform(shift), parents, pose, changed);
        for (size_t side = 0; side < legs.size(); ++side)
        {
            const FootLegSetup& leg = *legs[side];
            const LegIKResult& solved = solutions[side];
            TransformBranch(leg.hipNode, solved.hip * pose[leg.hipNode].getInverse(), parents, pose, changed);
            TransformBranch(leg.kneeNode, solved.knee * pose[leg.kneeNode].getInverse(), parents, pose, changed);
            TransformBranch(leg.ankleNode, solved.ankle * pose[leg.ankleNode].getInverse(), parents, pose, changed);
        }
    }

    void TurnInjuredLink(const ImpactPalmGoal& goal, const FallArmJoint& joint, int parent, int root,
        const PxVec3& from, const PxVec3& toward, float maximum, const std::vector<int32_t>& parents,
        std::vector<PxTransform>& pose, std::vector<bool>& changed)
    {
        if (joint.index < 0 || from.magnitudeSquared() < 1e-8f || toward.magnitudeSquared() < 1e-8f)
        {
            return;
        }
        const PxQuat swing = PxShortestRotation(from.getNormalized(), toward.getNormalized());
        const float fraction = PxMin(1.0f, maximum / PxMax(.001f, swing.getAngle())) * goal.weight;
        const PxQuat desired = BlendRotation(PxQuat(PxIdentity), swing, fraction) * pose[root].q;
        PxTransform limited(PxIdentity);
        if (LimitJointPose(goal, joint, pose[parent], desired, limited))
        {
            RotateBranch(root, pose[root].p, limited.q * pose[root].q.getConjugate(), parents, pose, changed);
        }
    }

    void ApplyWoundPosture(const ImpactPalmGoal& goal, const FallArmSetup& injured, const FallArmSetup& reaching,
        const std::vector<int32_t>& parents, const std::vector<PxTransform>& footstepPose,
        std::vector<PxTransform>& pose, std::vector<bool>& changed)
    {
        if (!goal.limitJointRotation || reaching.chestNode < 0 || !Ancestor(parents, reaching.chestNode, reaching.shoulderNode))
        {
            return;
        }
        const PxVec3 up = pose[reaching.chestNode].q.rotate(reaching.upInChest);
        const PxVec3 forward = pose[reaching.anchorNode].q.rotate(reaching.forwardInAnchor);
        if (goal.injuredLeg && Ancestor(parents, goal.pelvisNode, reaching.chestNode) &&
            Ancestor(parents, goal.injuredLeg->hipNode, goal.injuredLeg->kneeNode))
        {
            const FootLegSetup& leg = *goal.injuredLeg;
            std::vector<int> spine;
            for (int node = reaching.chestNode; node != goal.pelvisNode; node = parents[node])
            {
                spine.push_back(node);
            }
            PxVec3 bendAxis = up.cross(forward);
            if (!Normalize(bendAxis))
            {
                return;
            }
            if (goal.restingLegWeight > 0)
            {
                // 몸통이 지면에 안착하면 골반과 반대쪽 다리를 두고 다친 무릎을 손 쪽으로 접는다.
                const float weight = goal.restingLegWeight * goal.weight;
                const PxVec3 thigh = (pose[leg.kneeNode].p - pose[leg.hipNode].p).getNormalized();
                const PxQuat hip = PxShortestRotation(thigh, (forward + up * .75f).getNormalized());
                RotateBranch(leg.hipNode, pose[leg.hipNode].p, BlendRotation(PxQuat(PxIdentity), hip, weight), parents, pose, changed);
                if (Ancestor(parents, leg.kneeNode, leg.ankleNode))
                {
                    const PxVec3 calf = (pose[leg.ankleNode].p - pose[leg.kneeNode].p).getNormalized();
                    const PxQuat knee = PxShortestRotation(calf, (-up + forward * .2f).getNormalized());
                    RotateBranch(leg.kneeNode, pose[leg.kneeNode].p, BlendRotation(PxQuat(PxIdentity), knee, weight), parents, pose, changed);
                }
                const float curl = .66f * weight / static_cast<float>(spine.size());
                for (std::vector<int>::const_reverse_iterator node = spine.crbegin(); node != spine.crend(); ++node)
                {
                    RotateBranch(*node, pose[*node].p, PxQuat(curl, bendAxis), parents, pose, changed);
                }
                return;
            }
            if (goal.legBendWeight <= 0)
            {
                return;
            }
            const float bendWeight = goal.weight * goal.legBendWeight;
            const float pelvisAngle = (.98f - .20f * goal.freeLegWeight) * bendWeight;
            const PxVec3 side = pose[leg.hipNode].p - pose[goal.pelvisNode].p;
            const float lean = .16f * bendWeight * (side.dot(up.cross(forward)) >= 0 ? 1.0f : -1.0f);
            // 숙일 때 같은 쪽 어깨도 무릎 위로 내린다.
            const PxQuat fold = PxQuat(-lean, forward.getNormalized()) * PxQuat(pelvisAngle, bendAxis);
            const float trunkLength = (pose[reaching.shoulderNode].p - pose[goal.pelvisNode].p).magnitude();
            const float shiftLength = trunkLength * .06f * bendWeight * (1.0f - .5f * goal.freeLegWeight);
            LowerWoundPelvis(goal, -(forward * .6f + up * .8f) * shiftLength, forward, parents, pose, changed);
            RotateBranch(spine.back(), pose[goal.pelvisNode].p, fold, parents, pose, changed);
            pose[goal.pelvisNode].q = fold * pose[goal.pelvisNode].q;
            changed[goal.pelvisNode] = true;
            const float spineAngle = .52f * bendWeight / static_cast<float>(spine.size());
            for (std::vector<int>::const_reverse_iterator node = spine.crbegin(); node != spine.crend(); ++node)
            {
                RotateBranch(*node, pose[*node].p, PxQuat(spineAngle, bendAxis), parents, pose, changed);
            }
            if (goal.freeLegWeight > 0)
            {
                const float lift = goal.freeLegWeight * goal.weight;
                RotateBranch(leg.hipNode, pose[leg.hipNode].p, PxQuat(-.92f * lift, bendAxis), parents, pose, changed);
                RotateBranch(leg.kneeNode, pose[leg.kneeNode].p, PxQuat(1.05f * lift, bendAxis), parents, pose, changed);
            }
            const std::array<const FootLegSetup*, 2> legs{goal.injuredLeg, goal.oppositeLeg};
            for (size_t sideIndex = 0; sideIndex < legs.size(); ++sideIndex)
            {
                const FootLegSetup* moving = legs[sideIndex];
                const int anchor = goal.steppingLegAnchors[sideIndex];
                if (moving && moving->available && Ancestor(parents, anchor, moving->hipNode))
                {
                    // 같은 물리 틱에서 이미 푼 이동발 자세를 피격 보정 후의 부모 기준으로 옮긴다.
                    const PxTransform delta = pose[anchor] * footstepPose[anchor].getInverse();
                    TransformBranch(moving->hipNode, delta, parents, footstepPose, pose, changed);
                }
            }
            return;
        }
        if (!injured.available || !Ancestor(parents, injured.chestNode, injured.clavicleNode) ||
            !Ancestor(parents, injured.anchorNode, injured.shoulderNode) || !Ancestor(parents, injured.shoulderNode, injured.elbowNode) ||
            !Ancestor(parents, injured.elbowNode, injured.handNode) ||
            (goal.hitNode != injured.clavicleNode && goal.hitNode != injured.shoulderNode &&
                !Ancestor(parents, injured.shoulderNode, goal.hitNode)))
        {
            return;
        }
        const PxVec3 center = (pose[injured.shoulderNode].p + pose[reaching.shoulderNode].p) * .5f;
        const PxVec3 span = pose[injured.shoulderNode].p - pose[injured.clavicleNode].p;
        TurnInjuredLink(goal, injured.clavicleJoint, injured.chestNode, injured.clavicleNode, span,
            span + forward * span.magnitude(), .24f, parents, pose, changed);
        const PxVec3 elbowTarget = center + (forward * .68f - up * .65f) * injured.upperLength;
        TurnInjuredLink(goal, injured.armJoints[0], injured.anchorNode, injured.shoulderNode,
            pose[injured.elbowNode].p - pose[injured.shoulderNode].p, elbowTarget - pose[injured.shoulderNode].p,
            .82f, parents, pose, changed);
        if (goal.hitNode == injured.elbowNode || Ancestor(parents, injured.elbowNode, goal.hitNode))
        {
            const PxVec3 handTarget = center + (forward * .8f - up * .6f) * injured.lowerLength;
            TurnInjuredLink(goal, injured.armJoints[1], injured.shoulderNode, injured.elbowNode,
                pose[injured.handNode].p - pose[injured.elbowNode].p, handTarget - pose[injured.elbowNode].p,
                1.1f, parents, pose, changed);
        }
    }

    bool ApplyArmPalmTarget(const FallArmSample& arm, const FallArmSetup* injuredArm, int side, const ImpactPalmGoal& goal, const PxTransform& palm,
        const PxTransform& contact, const std::vector<int32_t>& parents, std::vector<PxTransform>& pose, FallArmSnapshot& targets)
    {
        const FallArmSetup& setup = arm.setup;
        std::vector<PxTransform> source = pose;
        std::vector<bool> assisted(pose.size(), false);
        if (injuredArm)
        {
            ApplyWoundPosture(goal, *injuredArm, setup, parents, pose, source, assisted);
        }
        ArmReachPose reach;
        if (!SolveArmReach(arm, side, goal, palm, parents, source, reach))
        {
            return false;
        }
        const bool hasLimits = goal.limitJointRotation &&
            std::all_of(setup.armJoints.begin(), setup.armJoints.end(), [](const FallArmJoint& joint) -> bool
            {
                return joint.index >= 0;
            });
        const ArmReachPose analyticReach = reach;
        if (hasLimits)
        {
            RefineWoundArm(arm, side, goal, palm, reach);
        }
        const PxTransform projectedPalm = reach.bones.back() * setup.palmInHand;
        const bool fitsContact = hasLimits && goal.approach >= 1.0f && PalmFitsWound(projectedPalm, contact, WoundPoseTolerance);
        if (hasLimits && !fitsContact)
        {
            const FallArmHandShape& shape = setup.handShape;
            const PxVec3 normal = contact.q.getBasisVector1();
            const float gap = HandSurfaceGap(shape, reach.bones.back(), goal.point, normal);
            const float nearWound = shape.halfExtents.magnitude() + shape.radius + WoundClearance;
            const PxVec3 center = reach.bones.back().transform(shape.inBone.p);
            if (gap < WoundClearance && (center - goal.point).magnitudeSquared() <= nearWound * nearWound)
            {
                // 손날이 상처에 눌리면 이미 가능한 손목 방향을 유지하며 손 형상 전체를 표면 밖으로 물린다.
                const float analyticGap = HandSurfaceGap(shape, analyticReach.bones.back(), goal.point, normal);
                const PxTransform permitted = analyticReach.bones.back() * setup.palmInHand;
                const PxTransform outside(permitted.p + normal * PxMax(0.0f, WoundClearance - analyticGap), permitted.q);
                ArmReachPose backedOff;
                if (analyticGap > gap)
                {
                    reach = analyticReach;
                }
                if (analyticGap < WoundClearance && SolveArmReach(arm, side, goal, outside, parents, source, backedOff) &&
                    HandSurfaceGap(shape, backedOff.bones.back(), goal.point, normal) > PxMax(gap, analyticGap))
                {
                    reach = backedOff;
                }
            }
        }

        const PxTransform attainable = reach.bones.back() * setup.palmInHand;
        const float reachError = (attainable.p - palm.p).magnitude();
        const bool canAssist = hasLimits && reachError <= .075f;
        // 보조손은 PA가 허용하는 최종 자세로 손등 부근에 닿을 수 있을 때만 참여한다.
        if (!injuredArm && (!canAssist || attainable.q.getBasisVector1().dot(contact.q.getBasisVector1()) < .5f))
        {
            return false;
        }
        const PxTransform toAnimation = reach.toPhysical.getInverse();
        std::vector<PxTransform> desired = source;
        std::vector<bool> changed(pose.size(), false);
        if (reach.usesClavicle)
        {
            desired[setup.clavicleNode] = reach.clavicle;
            changed[setup.clavicleNode] = true;
        }
        desired[setup.shoulderNode] = toAnimation * reach.bones[0];
        desired[setup.elbowNode] = toAnimation * reach.bones[1];
        desired[setup.handNode] = toAnimation * reach.bones[2];
        changed[setup.shoulderNode] = true;
        changed[setup.elbowNode] = true;
        changed[setup.handNode] = true;
        const float weight = goal.weight * goal.weight * (3 - 2 * goal.weight);
        // 손가락 등 자식의 로컬 자세를 보존한 다음 로컬 회전만 보간해 뼈 길이를 유지한다.
        for (size_t node = 0; node < pose.size(); ++node)
        {
            if (assisted[node])
            {
                pose[node] = source[node];
                targets.correctedNodes.push_back(static_cast<int>(node));
            }
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
                if (!assisted[node])
                {
                    targets.correctedNodes.push_back(static_cast<int>(node));
                }
            }
        }
        FallArmTarget& target = targets.arms[side];
        target.palm = palm;
        // 접근 구동기는 손날의 여유 공간을 포함한 도달 가능 자세까지만 당긴다.
        target.assistPalm = attainable;
        if (target.assistPalm.q.getBasisVector1().dot(palm.q.getBasisVector1()) > .5f)
        {
            target.assistPalm.q = palm.q;
        }
        target.solvedPalm = (reach.toPhysical * pose[setup.handNode] * setup.palmInHand).p;
        target.elbow = reach.bones[1].p;
        target.weight = goal.weight;
        target.reachError = reachError;
        target.canAssistWoundReach = canAssist;
        target.canDriveWoundGrip = injuredArm && fitsContact;
        target.reachLimited = reach.reachLimited;
        target.hasTarget = true;
        target.applied = true;
        return true;
    }

    bool CanReachHit(const FallArmSample& arm, int hitNode, const std::vector<int32_t>& parents)
    {
        const FallArmSetup& setup = arm.setup;
        return ValidArm(arm, parents) && hitNode != setup.clavicleNode && hitNode != setup.shoulderNode &&
            !Ancestor(parents, setup.shoulderNode, hitNode);
    }

    bool MakeWoundPalm(const PxVec3& point, const PxVec3& normal, float approachOffset, const PxTransform& actualPalm,
        const PxVec3& elbow, PxTransform& palm)
    {
        const PxVec3 position = point + normal * PalmSurfaceOffset + normal * approachOffset;
        PxVec3 fingers = point - elbow;
        fingers -= normal * normal.dot(fingers);
        if (fingers.normalize() > 1e-4f)
        {
            palm = PxTransform(position, PxQuat(PxMat33(normal.cross(fingers), normal, fingers)).getNormalized());
            return true;
        }
        return MakeFootSoleTarget(position, normal, actualPalm, palm);
    }

    void ApplySupportingPalm(const ImpactPalmGoal& primary, const std::array<FallArmSample, 2>& arms,
        const std::vector<int32_t>& parents, std::vector<PxTransform>& pose, FallArmSnapshot& targets)
    {
        const int side = 1 - primary.hand;
        const FallArmSample& arm = arms[side];
        if (!CanReachHit(arm, primary.hitNode, parents))
        {
            return;
        }
        // 이미 상처를 짚은 손의 실제 콜라이더 두께로 손등 표면을 구한다.
        const FallArmSample& leading = arms[primary.hand];
        const PxTransform hand = leading.actualHand * leading.setup.palmInHand;
        const FallArmHandShape& shape = leading.setup.handShape;
        const PxTransform collider = leading.actualHand * shape.inBone;
        const PxVec3 normal = hand.q.getBasisVector1();
        const float thickness = (collider.p - hand.p).dot(normal) + shape.GetHalfExtent(collider.q.rotateInv(normal));
        const PxVec3 point = hand.p + normal * PxMax(0.0f, thickness);

        const FallArmSetup& setup = arm.setup;
        const PxTransform mapping = arm.actualAnchor * pose[setup.anchorNode].getInverse();
        const float reach = setup.upperLength + setup.lowerLength + setup.palmInHand.p.magnitude() +
            (pose[setup.shoulderNode].p - pose[setup.anchorNode].p).magnitude();
        if ((point - arm.actualAnchor.p).magnitudeSquared() > reach * reach)
        {
            return;
        }
        ImpactPalmGoal goal;
        goal.point = point;
        goal.normal = normal;
        goal.approach = primary.supportApproach;
        goal.weight = PxMin(primary.weight, goal.approach);
        goal.clavicleReachFraction = primary.clavicleReachFraction;
        goal.history = primary.supportHistory;
        goal.limitJointRotation = primary.limitJointRotation;
        PxTransform palm(PxIdentity);
        if (!MakeWoundPalm(point, normal, .06f * (1.0f - goal.approach), arm.actualHand * setup.palmInHand,
            mapping.transform(pose[setup.elbowNode].p), palm))
        {
            return;
        }
        const PxTransform contact(point + normal * PalmSurfaceOffset, palm.q);
        if (ApplyArmPalmTarget(arm, nullptr, side, goal, palm, contact, parents, pose, targets))
        {
            // 무릎을 짚으며 몸통을 굽힌 경우에도 두 번 수정된 본을 한 번만 기록한다.
            std::sort(targets.correctedNodes.begin(), targets.correctedNodes.end());
            targets.correctedNodes.erase(std::unique(targets.correctedNodes.begin(), targets.correctedNodes.end()), targets.correctedNodes.end());
        }
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
    const PxVec3 contactPosition = goal.point + normal * PalmSurfaceOffset;
    float leastExcess = PX_MAX_F32;
    float nearestPalmSquared = PX_MAX_F32;
    int selected = -1;
    PxTransform palm(PxIdentity);
    for (int side = 0; side < static_cast<int>(arms.size()); ++side)
    {
        const FallArmSample& arm = arms[side];
        const FallArmSetup& setup = arm.setup;
        if ((goal.hand >= 0 && side != goal.hand) || !CanReachHit(arm, goal.hitNode, parents))
        {
            continue;
        }
        const PxTransform mapping = arm.actualAnchor * pose[setup.anchorNode].getInverse();
        const PxTransform elbow = mapping * pose[setup.elbowNode];
        const PxTransform actualPalm = arm.actualHand * setup.palmInHand;
        PxTransform candidate(PxIdentity);
        if (!MakeWoundPalm(goal.point, normal, approachOffset, actualPalm, elbow.p, candidate))
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
    const PxTransform contact(contactPosition, palm.q);
    targets.active = ApplyArmPalmTarget(arms[selected], &arms[1 - selected].setup, selected, goal, palm, contact, parents, pose, targets);
    if (!targets.active)
    {
        return false;
    }
    goal.hand = selected;
    if (goal.supportApproach > 0 && goal.supportApproach <= 1.0f && goal.limitJointRotation)
    {
        ApplySupportingPalm(goal, arms, parents, pose, targets);
    }
    return true;
}
