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

    const PxVec3 kneeOffset = input.knee.p - input.hip.p;
    PxVec3 sourceBend = kneeOffset - sourceAxis * kneeOffset.dot(sourceAxis);
    const bool animationBent = sourceBend.magnitude() > (upper + lower) * .001f;
    PxVec3 bend = sourceBend;
    // 거의 펴진 애니메이션의 작은 오차로 무릎 방향이 뒤집히는 것을 피한다.
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
    const double along = (double(upper) * upper - double(lower) * lower + double(distance) * distance) / (2 * distance);
    const float height = float(std::sqrt((std::max)(0.0, double(upper) * upper - along * along)));
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
    if (input.stepFoot < 0 || input.stepFoot > 1)
    {
        return false;
    }
    for (const auto& leg : input.legs)
    {
        if (!leg.available || !Ancestor(parents, leg.anchorNode, leg.hipNode) ||
            !Ancestor(parents, leg.hipNode, leg.kneeNode) || !Ancestor(parents, leg.kneeNode, leg.ankleNode) ||
            !leg.actualAnchor.isSane())
        {
            return false;
        }
    }
    const auto& left = input.legs[0];
    const auto& right = input.legs[1];
    if (left.hipNode == right.hipNode || Ancestor(parents, left.hipNode, right.hipNode) ||
        Ancestor(parents, right.hipNode, left.hipNode))
    {
        return false;
    }

    auto corrected = pose;
    std::vector<bool> changed(pose.size(), false);
    m_Snapshot.status = FootstepIKStatus::InvalidTarget;
    for (size_t side = 0; side < input.legs.size(); ++side)
    {
        const auto& leg = input.legs[side];
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
        auto& result = m_Snapshot.legs[side];
        result = SolveTwoBoneIK(solve);
        if (!result.valid)
        {
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
            // 발가락 등 별도 IK 대상이 아닌 자식은 기존 로컬 자세를 보존한다.
            corrected[node] = corrected[parent] * pose[parent].getInverse() * pose[node];
            changed[node] = true;
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
