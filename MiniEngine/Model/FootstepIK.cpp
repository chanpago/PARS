#include "FootstepIK.h"
#include "foundation/PxMat33.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

using namespace physx;
using namespace PARS;

namespace
{
    float ReachMargin(float upper, float lower)
    {
        return PxMin((upper + lower) * 1e-4f, PxMin(upper, lower) * .1f);
    }

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

PxVec3 PARS::GetFootstepTravelForward(const PxVec3& displacement, const PxVec3& referenceForward)
{
    PxVec3 travel(displacement.x, 0, displacement.z);
    if (!Unit(travel))
    {
        return PxVec3(0);
    }
    return travel.dot(referenceForward) < 0 ? -travel : travel;
}

FootIKIssue PARS::BuildFootSoleTarget(const PxVec3& position, const PxVec3& normal,
    const PxTransform& referenceSole, PxTransform& target, const PxVec3& displacement)
{
    PxVec3 up = normal;
    if (!position.isFinite())
    {
        return FootIKIssue::TargetPosition;
    }
    if (!referenceSole.isSane())
    {
        return FootIKIssue::ReferenceSole;
    }
    if (!Unit(up))
    {
        return FootIKIssue::GroundNormal;
    }
    PxVec3 forward = referenceSole.q.rotate(PxVec3(0, 0, 1));
    const PxVec3 travel = GetFootstepTravelForward(displacement, forward);
    if (!travel.isZero())
    {
        // 경사면에서도 발끝의 수평 방향이 보폭과 나란하도록 높이 성분만 지면에 맞춘다.
        forward = PxVec3(travel.z, 0, -travel.x).cross(up);
    }
    if (!ProjectHint(forward, up))
    {
        PxVec3 right = referenceSole.q.rotate(PxVec3(1, 0, 0));
        if (!ProjectHint(right, up))
        {
            return FootIKIssue::SoleOrientation;
        }
        forward = right.cross(up);
    }
    const PxVec3 right = up.cross(forward).getNormalized();
    target = PxTransform(position, PxQuat(PxMat33(right, up, right.cross(up))).getNormalized());
    return target.isSane() ? FootIKIssue::None : FootIKIssue::SoleOrientation;
}

bool PARS::MakeFootSoleTarget(const PxVec3& position, const PxVec3& normal,
    const PxTransform& referenceSole, PxTransform& target, const PxVec3& displacement)
{
    return BuildFootSoleTarget(position, normal, referenceSole, target, displacement) == FootIKIssue::None;
}

float PARS::PredictSupportReachTime(const PxVec3& hipFromAnkle, const PxVec3& hipVelocity,
    float thighLength, float calfLength)
{
    if (!hipFromAnkle.isFinite() || !hipVelocity.isFinite() || !std::isfinite(thighLength) ||
        !std::isfinite(calfLength) || thighLength <= 1e-5f || calfLength <= 1e-5f)
    {
        return std::numeric_limits<float>::infinity();
    }
    const double length = thighLength + calfLength - ReachMargin(thighLength, calfLength);
    const PxVec3d offset(hipFromAnkle.x, hipFromAnkle.y, hipFromAnkle.z);
    const PxVec3d velocity(hipVelocity.x, hipVelocity.y, hipVelocity.z);
    const double slack = length * length - offset.magnitudeSquared();
    const double radial = offset.dot(velocity);
    if (slack < 0 || (slack == 0 && radial >= 0))
    {
        return 0.0f;
    }
    const double speedSquared = velocity.magnitudeSquared();
    if (speedSquared <= 1e-12)
    {
        return std::numeric_limits<float>::infinity();
    }
    // 고정 발목을 중심으로 한 구에서 고관절이 빠져나가는 근을 구한다. 경계 근처의 뺄셈 오차를 피한다.
    const double root = std::sqrt(radial * radial + speedSquared * slack);
    const double time = radial >= 0 ? slack / (root + radial) : (root - radial) / speedSquared;
    return static_cast<float>(time);
}

LegIKResult PARS::SolveTwoBoneIK(const TwoBoneIKInput& input)
{
    LegIKResult result;
    const float upper = input.thighLength;
    const float lower = input.calfLength;
    if (!input.hip.isSane())
    {
        result.issue = FootIKIssue::HipPose;
        return result;
    }
    if (!input.knee.isSane())
    {
        result.issue = FootIKIssue::KneePose;
        return result;
    }
    if (!input.ankle.isSane())
    {
        result.issue = FootIKIssue::AnklePose;
        return result;
    }
    if (!input.targetSole.isSane() || !input.soleInBone.isSane())
    {
        result.issue = FootIKIssue::SoleTransform;
        return result;
    }
    if (!std::isfinite(upper) || upper <= 1e-5f)
    {
        result.issue = FootIKIssue::ThighLength;
        return result;
    }
    if (!std::isfinite(lower) || lower <= 1e-5f)
    {
        result.issue = FootIKIssue::CalfLength;
        return result;
    }
    if (!std::isfinite(input.maximumReachRatio) || input.maximumReachRatio <= 0 || input.maximumReachRatio > 1)
    {
        result.issue = FootIKIssue::ReachRatio;
        return result;
    }

    // input.targetSole: 발바닥의 목표 위치·회전
    // input.soleInBone: 발목본 기준으로 저장한 발바닥 로컬 변환
    // requested: 역산한 발목본의 목표 위치·회전
    const PxTransform requested = input.targetSole * input.soleInBone.getInverse();

    PxVec3 sourceUpper = input.knee.p - input.hip.p;
    PxVec3 sourceLower = input.ankle.p - input.knee.p;
    if (!requested.isSane())
    {
        result.issue = FootIKIssue::TargetAnkle;
        return result;
    }
    if (!Unit(sourceUpper))
    {
        result.issue = FootIKIssue::ThighDirection;
        return result;
    }
    if (!Unit(sourceLower))
    {
        result.issue = FootIKIssue::CalfDirection;
        return result;
    }
    PxVec3 direction = requested.p - input.hip.p;
    const float requestedDistance = direction.magnitude();
    if (!std::isfinite(requestedDistance))
    {
        result.issue = FootIKIssue::TargetDistance;
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


    PxVec3 bend = input.preferReferenceBend ? input.referenceBend : sourceBend;
    // 거의 펴진 애니메이션의 작은 오차로 무릎 방향이 뒤집히는 것을 피한다.

    // bend는 b0
    // direction은 고관절에서 새 목표 발목으로 향하는 단위벡터
    if (input.preferReferenceBend && ProjectHint(bend, direction))
    {
        result.hintSource = KneeHintSource::Reference;
    }
    else if (animationBent && ProjectHint(bend = sourceBend, direction))
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
    const float margin = ReachMargin(upper, lower);
    const float minimumReach = PxAbs(upper - lower) + margin;
    const float maximumReach = PxMax(minimumReach, (upper + lower) * input.maximumReachRatio - margin);
    const float distance = PxClamp(requestedDistance, minimumReach, maximumReach);

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
    if (input.jointLimits && input.jointLimits->project)
    {
        ConstrainFootstepLeg(*input.jointLimits, input.soleInBone, result);
    }
    result.valid = result.hip.isSane() && result.knee.isSane() && result.ankle.isSane() &&
        result.solvedSole.isSane() && std::isfinite(result.reachError);
    if (!result.valid)
    {
        result.issue = FootIKIssue::SolvedPose;
    }
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
    m_Snapshot.motionTargets = input.motionTargets;
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
    for (const PARS::FootIKLegInput& leg : input.legs)
    {
        if (!leg.available || !Ancestor(parents, leg.anchorNode, leg.hipNode) ||
            !Ancestor(parents, leg.hipNode, leg.kneeNode) || !Ancestor(parents, leg.kneeNode, leg.ankleNode) ||
            !leg.actualAnchor.isSane())
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
        if (!BuildLegInput(input, side, pose, solve))
        {
            return false;
        }
        PARS::LegIKResult& result = m_Snapshot.legs[side];
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

bool FootstepIK::BuildLegInput(const FootstepIKInput& input, size_t side,
    const std::vector<PxTransform>& pose, TwoBoneIKInput& result) const
{
    if (side >= input.legs.size())
    {
        return false;
    }
    const FootIKLegInput& leg = input.legs[side];
    if (!leg.available || leg.anchorNode < 0 || leg.hipNode < 0 || leg.kneeNode < 0 || leg.ankleNode < 0 ||
        size_t(leg.anchorNode) >= pose.size() || size_t(leg.hipNode) >= pose.size() ||
        size_t(leg.kneeNode) >= pose.size() || size_t(leg.ankleNode) >= pose.size())
    {
        return false;
    }
    const PxTransform toPhysical = leg.actualAnchor * pose[leg.anchorNode].getInverse();
    result.jointLimits = input.motionTargets[side] && leg.jointLimits.project ? &leg.jointLimits : nullptr;
    result.hip = toPhysical * pose[leg.hipNode];
    result.knee = toPhysical * pose[leg.kneeNode];
    result.ankle = toPhysical * pose[leg.ankleNode];
    result.soleInBone = leg.soleInBone;
    result.targetSole = leg.targetSole;
    result.thighLength = leg.thighLength;
    result.calfLength = leg.calfLength;
    result.referenceBend = result.hip.q.rotate(leg.referenceBendInHip);
    result.hasPreviousBend = input.revision == m_Snapshot.revision && m_HasPreviousBend[side];
    result.previousBend = leg.actualAnchor.q.rotate(m_PreviousBendInAnchor[side]);
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
