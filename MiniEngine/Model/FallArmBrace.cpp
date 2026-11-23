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

    PxVec3 Horizontal(PxVec3 value)
    {
        value.y = 0;
        return value;
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

    bool ValidSettings(const FallArmSettings& settings)
    {
        const float positive[] = {settings.blendInDuration, settings.blendOutDuration,
            settings.maximumDuration, settings.targetSpeed};
        for (float value : positive)
        {
            if (!std::isfinite(value) || value <= 0)
            {
                return false;
            }
        }
        const float nonnegative[] = {settings.contactGraceDuration, settings.forwardReachRatio,
            settings.backwardReachRatio, settings.outwardReachRatio, settings.surfaceOffset};
        for (float value : nonnegative)
        {
            if (!std::isfinite(value) || value < 0)
            {
                return false;
            }
        }
        return std::isfinite(settings.reachRatio) && settings.reachRatio > 0 && settings.reachRatio < 1 &&
            std::isfinite(settings.trackingStrength) && settings.trackingStrength >= 0 && settings.trackingStrength <= 1 &&
            std::isfinite(settings.braceDriveScale) && settings.braceDriveScale >= 0 && settings.braceDriveScale <= 1;
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
            arm.shoulderVelocity.isFinite() && setup.forwardInAnchor.isFinite() && setup.rightInAnchor.isFinite();
    }

    bool SurfaceFrame(const PxVec3& position, PxVec3 normal, PxVec3 forward, PxTransform& frame)
    {
        if (!position.isFinite() || !Normalize(normal) || normal.y < .5f)
        {
            return false;
        }
        forward -= normal * normal.dot(forward);
        if (!Normalize(forward))
        {
            forward = PxVec3(0, 0, 1) - normal * normal.z;
            if (!Normalize(forward))
            {
                return false;
            }
        }
        const PxVec3 right = normal.cross(forward);
        frame = PxTransform(position, PxQuat(PxMat33(right, normal, forward)).getNormalized());
        return frame.isSane();
    }

    PxVec3 Approach(const PxVec3& from, const PxVec3& to, float distance)
    {
        const PxVec3 delta = to - from;
        const float length = delta.magnitude();
        return length > distance && length > 1e-6f ? from + delta * (distance / length) : to;
    }

    void KeepHorizontalReach(const FallArmSetup& setup, const PxVec3& shoulder,
        float reachRatio, PxTransform& palm)
    {
        const PxTransform wrist = palm * setup.palmInHand.getInverse();
        const float length = setup.upperLength + setup.lowerLength;
        const float margin = PxMin(length * 1e-4f, PxMin(setup.upperLength, setup.lowerLength) * .1f);
        const float minimumReach = PxAbs(setup.upperLength - setup.lowerLength) + margin;
        const float maximumReach = PxMax(minimumReach, length * reachRatio - margin);
        PxVec3 horizontal = Horizontal(wrist.p - shoulder);
        const float horizontalDistance = horizontal.magnitude();
        const float maximumHorizontal = maximumReach * .98f;
        if (horizontalDistance > maximumHorizontal)
        {
            horizontal *= maximumHorizontal / horizontalDistance;
        }
        // 먼 바닥을 향해 팔 전체를 축소하지 않고, 앞쪽 거리를 확보한 채 손 목표를 올린다.
        // 어깨가 내려오면 필요한 높이 보정이 줄어들어 같은 목표가 지면으로 이어진다.
        const float downwardReach = std::sqrt(PxMax(0.0f, maximumReach * maximumReach - horizontal.magnitudeSquared()));
        const PxVec3 reachableWrist(shoulder.x + horizontal.x,
            PxMax(wrist.p.y, shoulder.y - downwardReach), shoulder.z + horizontal.z);
        palm.p += reachableWrist - wrist.p;
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

void FallArmBrace::Reset()
{
    m_Snapshot = {};
    m_HasSample = false;
    m_WasRequested = false;
}

bool FallArmBrace::Apply(const FallArmInput& input, const FallArmSettings& settings,
    const std::vector<int32_t>& parents, const FallArmGroundQuery& ground, std::vector<PxTransform>& pose)
{
    if (!settings.enabled || !ValidSettings(settings) || !std::isfinite(input.deltaTime) || input.deltaTime <= 0)
    {
        Reset();
        return false;
    }
    const bool protectionRequested = input.falling || input.preparing;
    if ((m_HasSample && input.revision != m_Snapshot.revision) || (protectionRequested && !m_WasRequested))
    {
        Reset();
    }
    m_WasRequested = protectionRequested;
    const float deltaTime = m_HasSample && input.completedStep == m_Snapshot.completedStep ? 0 : input.deltaTime;
    m_HasSample = true;
    m_Snapshot.revision = input.revision;
    m_Snapshot.completedStep = input.completedStep;
    m_Snapshot.correctedNodes.clear();
    m_Snapshot.active = false;
    if (!ValidArmPose(parents, pose))
    {
        Reset();
        return false;
    }
    if (input.falling)
    {
        m_Snapshot.elapsedTime += deltaTime;
        m_Snapshot.settledTime = input.bodySettled ? m_Snapshot.settledTime + deltaTime : 0;
        m_Snapshot.finished |= m_Snapshot.elapsedTime >= settings.maximumDuration || m_Snapshot.settledTime >= .2f;
    }
    const bool reaching = protectionRequested && !m_Snapshot.finished;
    std::array<bool, 2> valid{};
    PxVec3 direction(0);
    PxVec3 fallback(0);
    for (size_t side = 0; side < input.arms.size(); ++side)
    {
        const FallArmSample& arm = input.arms[side];
        valid[side] = ValidArm(arm, parents);
        if (!valid[side])
        {
            m_Snapshot.arms[side] = {};
            continue;
        }
        // 양쪽 어깨의 수평 속도를 먼저 더해 빠르게 움직이는 쪽의 기여를 보존한다.
        direction += Horizontal(arm.shoulderVelocity);
        fallback += Horizontal(arm.actualAnchor.q.rotate(arm.setup.forwardInAnchor));
    }
    if (valid[0] && valid[1] && (input.arms[0].setup.shoulderNode == input.arms[1].setup.shoulderNode ||
        Ancestor(parents, input.arms[0].setup.shoulderNode, input.arms[1].setup.shoulderNode) ||
        Ancestor(parents, input.arms[1].setup.shoulderNode, input.arms[0].setup.shoulderNode)))
    {
        Reset();
        return false;
    }
    if (!Normalize(direction))
    {
        direction = m_Snapshot.direction;
        if (!Normalize(direction))
        {
            direction = fallback;
            if (!Normalize(direction))
            {
                direction = PxVec3(0, 0, 1);
            }
        }
    }
    if (m_Snapshot.direction.magnitudeSquared() > .5f)
    {
        // 정반대 방향도 정규화 선형 보간의 영벡터에 걸리지 않도록 수평 각도로 보간한다.
        const float angle = std::atan2(m_Snapshot.direction.cross(direction).y, m_Snapshot.direction.dot(direction));
        direction = PxQuat(angle * (1 - std::exp(-10 * deltaTime)), PxVec3(0, 1, 0)).rotate(m_Snapshot.direction);
    }
    m_Snapshot.direction = direction;
    std::array<PxVec3, 2> elbowHints{};
    for (size_t side = 0; side < input.arms.size(); ++side)
    {
        if (!valid[side])
        {
            continue;
        }
        const FallArmSample& arm = input.arms[side];
        const FallArmSetup& setup = arm.setup;
        FallArmTarget& target = m_Snapshot.arms[side];
        const FallArmPhase previousPhase = target.phase;
        target.applied = false;
        target.touching = arm.touching;
        const PxTransform toPhysical = arm.actualAnchor * pose[setup.anchorNode].getInverse();
        const PxTransform shoulder = toPhysical * pose[setup.shoulderNode];
        target.shoulder = shoulder.p;
        target.actualPalm = (arm.actualHand * setup.palmInHand).p;
        const float length = setup.upperLength + setup.lowerLength;
        const float sign = side == 0 ? -1.0f : 1.0f;
        PxVec3 right = Horizontal(arm.actualAnchor.q.rotate(setup.rightInAnchor));
        if (!Normalize(right))
        {
            right = direction.cross(PxVec3(0, 1, 0));
        }
        PxVec3 forward = Horizontal(arm.actualAnchor.q.rotate(setup.forwardInAnchor));
        if (!Normalize(forward))
        {
            forward = PxVec3(0, 1, 0).cross(right);
        }
        const float opposite = PxMax(0.0f, -direction.dot(right) * sign);
        const float desiredWeight = reaching ? 1.0f - .65f * opposite : 0;
        PxTransform requested(PxIdentity);
        bool hasRequest = false;
        if (reaching && arm.touching)
        {
            hasRequest = SurfaceFrame(arm.contactPosition + arm.contactNormal * settings.surfaceOffset,
                arm.contactNormal, direction, requested);
            if (hasRequest)
            {
                target.phase = FallArmPhase::Bracing;
                target.contactLostTime = 0;
            }
        }
        else if (reaching && target.phase == FallArmPhase::Bracing)
        {
            target.contactLostTime += deltaTime;
            if (target.hasTarget && target.contactLostTime <= settings.contactGraceDuration)
            {
                requested = target.palm;
                hasRequest = true;
            }
        }
        if (reaching && !hasRequest && ground)
        {
            // 뒤쪽 목표는 어깨 신전 범위를 고려해 짧게 잡고, 반대쪽 팔의 몸통 가로지르기를 줄인다.
            const float backward = PxMax(0.0f, -direction.dot(forward));
            const float reach = settings.forwardReachRatio * (1 - backward) + settings.backwardReachRatio * backward;
            PxVec3 offset = direction * (length * reach);
            const float inward = offset.dot(right) * sign;
            if (inward < 0)
            {
                offset -= right * (inward * sign);
            }
            const PxVec3 candidate = shoulder.p + offset + right * (sign * length * settings.outwardReachRatio);
            PxVec3 position(0);
            PxVec3 normal(0);
            if (ground(candidate + PxVec3(0, .25f * length, 0), 3.5f * length, position, normal))
            {
                hasRequest = SurfaceFrame(position + normal * settings.surfaceOffset, normal, direction, requested);
            }
            if (hasRequest)
            {
                // 손이 지면에서 멀 때는 손바닥 면을 낙상 방향으로 향하게 한다.
                // 팔 길이의 절반 이내로 접근하면 지면을 짚는 방향으로 점차 회전한다.
                const PxVec3 groundNormal = requested.q.rotate(PxVec3(0, 1, 0));
                const float clearance = PxMax(0.0f, groundNormal.dot(target.actualPalm - position));
                const float facingWeight = PxClamp(clearance / (length * .5f), 0.0f, 1.0f);
                requested.q = (requested.q * PxQuat(-PxHalfPi * facingWeight, PxVec3(1, 0, 0))).getNormalized();
                KeepHorizontalReach(setup, shoulder.p, settings.reachRatio, requested);
                target.phase = FallArmPhase::Reaching;
                target.contactLostTime = 0;
            }
        }
        if (hasRequest)
        {
            if (target.hasTarget)
            {
                PxTransform previousPalm = target.palm;
                if (previousPhase == FallArmPhase::Reaching && target.phase == FallArmPhase::Reaching)
                {
                    // 몸통 이동과 회전은 즉시 반영하고, 몸통 기준으로 팔을 뻗는 변화만 제한한다.
                    previousPalm = arm.actualAnchor * target.previousAnchor.getInverse() * previousPalm;
                }
                // 접촉 중과 접촉 해제 유예 중에는 몸통에 따라 움직이지 않고 월드 접촉점을 유지한다.
                requested.p = Approach(previousPalm.p, requested.p, settings.targetSpeed * deltaTime);
                requested.q = BlendRotation(previousPalm.q, requested.q, 1 - std::exp(-15 * deltaTime));
            }
            target.palm = requested;
            target.hasTarget = true;
            target.weight = target.weight < desiredWeight ?
                PxMin(desiredWeight, target.weight + deltaTime / settings.blendInDuration) :
                PxMax(desiredWeight, target.weight - deltaTime / settings.blendOutDuration);
        }
        else
        {
            target.phase = FallArmPhase::Releasing;
            target.weight = PxMax(0.0f, target.weight - deltaTime / settings.blendOutDuration);
        }
        target.previousAnchor = arm.actualAnchor;
        if (!target.hasTarget || target.weight <= 0)
        {
            target = {};
            continue;
        }
        const float driveGoal = target.phase == FallArmPhase::Bracing ? settings.braceDriveScale : 1;
        target.driveScale += (driveGoal - target.driveScale) * (1 - std::exp(-12 * deltaTime));
        elbowHints[side] = right * sign - forward * .25f - PxVec3(0, .2f, 0);
    }
    m_Snapshot.active = ApplyArmPalmTargets(input.arms, elbowHints, settings.reachRatio, parents, pose, m_Snapshot);
    if (!protectionRequested && !m_Snapshot.active)
    {
        Reset();
    }
    return m_Snapshot.active;
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

const char* PARS::FallArmPhaseLabel(FallArmPhase phase)
{
    switch (phase)
    {
    case FallArmPhase::Reaching:
        return u8"손 뻗기";
    case FallArmPhase::Bracing:
        return u8"접촉 후 버티기";
    case FallArmPhase::Releasing:
        return u8"팔 보정 해제";
    default:
        return u8"비활성";
    }
}
