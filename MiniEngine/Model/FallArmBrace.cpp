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
        const float nonnegative[] = {settings.predictionTime, settings.contactGraceDuration,
            settings.forwardReachRatio, settings.backwardReachRatio, settings.outwardReachRatio,
            settings.surfaceOffset};
        for (float value : nonnegative)
        {
            if (!std::isfinite(value) || value < 0)
            {
                return false;
            }
        }
        return std::isfinite(settings.reachRatio) && settings.reachRatio > 0 && settings.reachRatio < 1 &&
            std::isfinite(settings.braceDriveScale) && settings.braceDriveScale >= 0 && settings.braceDriveScale <= 1;
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
            arm.shoulderVelocity.isFinite() && setup.forwardInAnchor.isFinite() &&
            setup.rightInAnchor.isFinite() && setup.upInAnchor.isFinite();
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
}

void FallArmBrace::Reset()
{
    m_Snapshot = {};
    m_HasSample = false;
    m_WasFalling = false;
}

bool FallArmBrace::Apply(const FallArmInput& input, const FallArmSettings& settings,
    const std::vector<int32_t>& parents, const FallArmGroundQuery& ground, std::vector<PxTransform>& pose)
{
    if (!settings.enabled || !ValidSettings(settings) || !std::isfinite(input.deltaTime) || input.deltaTime <= 0)
    {
        Reset();
        return false;
    }
    if ((m_HasSample && input.revision != m_Snapshot.revision) || (input.falling && !m_WasFalling))
    {
        Reset();
    }
    m_WasFalling = input.falling;
    const float deltaTime = m_HasSample && input.completedStep == m_Snapshot.completedStep ? 0 : input.deltaTime;
    m_HasSample = true;
    m_Snapshot.revision = input.revision;
    m_Snapshot.completedStep = input.completedStep;
    m_Snapshot.correctedNodes.clear();
    m_Snapshot.active = false;
    if (pose.empty() || pose.size() != parents.size())
    {
        Reset();
        return false;
    }
    for (size_t node = 0; node < pose.size(); ++node)
    {
        if (parents[node] < -1 || parents[node] >= static_cast<int>(node) || !pose[node].isSane())
        {
            Reset();
            return false;
        }
    }
    if (input.falling)
    {
        m_Snapshot.elapsedTime += deltaTime;
        m_Snapshot.settledTime = input.bodySettled ? m_Snapshot.settledTime + deltaTime : 0;
        m_Snapshot.finished |= m_Snapshot.elapsedTime >= settings.maximumDuration || m_Snapshot.settledTime >= .2f;
    }
    const bool reaching = input.falling && !m_Snapshot.finished;
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
        const float length = arm.setup.upperLength + arm.setup.lowerLength;
        // 상체 회전으로 생긴 어깨 속도와 실제 기울기를 함께 사용한다.
        direction += Horizontal(arm.shoulderVelocity) * settings.predictionTime +
            Horizontal(arm.actualAnchor.q.rotate(arm.setup.upInAnchor)) * length * .5f;
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
    const std::vector<PxTransform> source = pose;
    std::vector<PxTransform> desired = pose;
    std::vector<float> weights(pose.size(), 0);
    std::vector<bool> changed(pose.size(), false);
    for (size_t side = 0; side < input.arms.size(); ++side)
    {
        if (!valid[side])
        {
            continue;
        }
        const FallArmSample& arm = input.arms[side];
        const FallArmSetup& setup = arm.setup;
        FallArmTarget& target = m_Snapshot.arms[side];
        target.applied = false;
        target.touching = arm.touching;
        const PxTransform toPhysical = arm.actualAnchor * source[setup.anchorNode].getInverse();
        const PxTransform shoulder = toPhysical * source[setup.shoulderNode];
        target.shoulder = shoulder.p;
        target.actualPalm = (arm.actualHand * setup.palmInHand).p;
        const float length = setup.upperLength + setup.lowerLength;
        const float sign = side == 0 ? -1.0f : 1.0f;
        PxVec3 right = Horizontal(arm.actualAnchor.q.rotate(setup.rightInAnchor));
        if (!Normalize(right))
        {
            right = PxVec3(direction.z, 0, -direction.x);
        }
        PxVec3 forward = Horizontal(arm.actualAnchor.q.rotate(setup.forwardInAnchor));
        if (!Normalize(forward))
        {
            forward = right.cross(PxVec3(0, 1, 0));
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
                target.phase = FallArmPhase::Reaching;
                target.contactLostTime = 0;
            }
        }
        if (hasRequest)
        {
            if (target.hasTarget)
            {
                // 지면 후보가 바뀌어도 한 스텝의 목표 속도는 제한한다.
                requested.p = Approach(target.palm.p, requested.p, settings.targetSpeed * deltaTime);
                requested.q = BlendRotation(target.palm.q, requested.q, 1 - std::exp(-15 * deltaTime));
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
        if (!target.hasTarget || target.weight <= 0)
        {
            target = {};
            continue;
        }
        const float smoothWeight = target.weight * target.weight * (3 - 2 * target.weight);
        const float driveGoal = target.phase == FallArmPhase::Bracing ? settings.braceDriveScale : 1;
        target.driveScale += (driveGoal - target.driveScale) * (1 - std::exp(-12 * deltaTime));
        TwoBoneIKInput solve;
        solve.hip = shoulder;
        solve.knee = toPhysical * source[setup.elbowNode];
        solve.ankle = toPhysical * source[setup.handNode];
        solve.soleInBone = setup.palmInHand;
        solve.targetSole = target.palm;
        solve.thighLength = setup.upperLength;
        solve.calfLength = setup.lowerLength;
        solve.maximumReachRatio = settings.reachRatio;
        solve.preferReferenceBend = true;
        solve.referenceBend = right * sign - forward * .25f - PxVec3(0, .2f, 0);
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
        m_Snapshot.active = true;
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
            m_Snapshot.correctedNodes.push_back(static_cast<int>(node));
        }
    }
    for (size_t side = 0; side < input.arms.size(); ++side)
    {
        FallArmTarget& target = m_Snapshot.arms[side];
        if (target.applied)
        {
            const FallArmSample& arm = input.arms[side];
            const PxTransform toPhysical = arm.actualAnchor * source[arm.setup.anchorNode].getInverse();
            target.solvedPalm = (toPhysical * pose[arm.setup.handNode] * arm.setup.palmInHand).p;
        }
    }
    if (!input.falling && !m_Snapshot.active)
    {
        Reset();
    }
    return m_Snapshot.active;
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
