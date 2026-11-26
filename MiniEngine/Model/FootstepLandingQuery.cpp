#include "FootstepLandingQuery.h"
#include "FootstepPoseValidation.h"
#include "FootstepCollisionBounds.h"
#include "FootSupport.h"
#include "Components/SkeletalMeshComponent.h"
#include "geometry/PxGeometryQuery.h"

using namespace PARS;
using namespace physx;

namespace
{
    PxTransform Frame(const Math::Matrix4& matrix)
    {
        const PhysicsJointFrame frame = PhysicsJointFrame::FromTransform(PhysicsBoneFrame(matrix));
        return PxTransform(PxVec3(frame.offset.x, frame.offset.y, frame.offset.z),
            PxQuat(frame.rotation.x, frame.rotation.y, frame.rotation.z, frame.rotation.w));
    }
}

void FootstepLandingQuery::Prepare(const SkeletalMeshComponent& mesh, const FootSupportSnapshot& support,
    std::array<FootstepReachInput, 2>& reaches)
{
    m_Asset = mesh.GetPhysicsAsset();
    m_Ragdoll = &mesh.GetRagdoll();
    m_Legs = {};
    for (FootstepReachInput& reach : reaches)
    {
        reach.available = false;
    }
    if (!m_Asset)
    {
        return;
    }
    const std::vector<int>& nodes = m_Ragdoll->GetBoneNodes();
    for (size_t side = 0; side < 2; ++side)
    {
        const FootSoleSample& sample = support.feet[side];
        Leg& leg = m_Legs[side];
        const std::array<int, 3> chain{{sample.leg.hipNode, sample.leg.kneeNode, sample.leg.ankleNode}};
        if (!sample.available)
        {
            continue;
        }
        if (!sample.leg.available)
        {
            leg.issue = FootstepPoseInputIssue::LegSetup;
            continue;
        }
        constexpr FootstepPoseInputIssue bodyIssues[] = {FootstepPoseInputIssue::ThighBody,
            FootstepPoseInputIssue::CalfBody, FootstepPoseInputIssue::FootBody};
        for (size_t part = 0; part < 3; ++part)
        {
            const std::vector<int>::const_iterator node = std::find(nodes.begin(), nodes.end(), chain[part]);
            if (node == nodes.end() || static_cast<size_t>(node - nodes.begin()) >= m_Asset->bodies.size())
            {
                leg.issue = bodyIssues[part];
                break;
            }
            leg.bodies[part + 1] = static_cast<int>(node - nodes.begin());
            FootstepCollisionShape& shape = leg.shapes[part];
            shape.available = m_Ragdoll->GetBodyCollisionShape(leg.bodies[part + 1], shape.geometry, shape.inBone, shape.actualPose);
        }
        if (leg.bodies[3] < 0)
        {
            continue;
        }
        for (const PhysicsConstraint& joint : m_Asset->constraints)
        {
            if (joint.childBone == m_Asset->bodies[leg.bodies[1]].bone)
            {
                leg.bodies[0] = m_Asset->FindBody(joint.parentBone);
                break;
            }
        }
        if (leg.bodies[0] < 0)
        {
            leg.issue = FootstepPoseInputIssue::HipParent;
            continue;
        }
        leg.anchor = Frame(m_Ragdoll->GetBoneWorld(leg.bodies[0]));
        leg.pose.hip = Frame(m_Ragdoll->GetBoneWorld(leg.bodies[1]));
        leg.pose.knee = Frame(m_Ragdoll->GetBoneWorld(leg.bodies[2]));
        leg.pose.ankle = Frame(m_Ragdoll->GetBoneWorld(leg.bodies[3]));
        leg.pose.thighLength = sample.leg.thighLength;
        leg.pose.calfLength = sample.leg.calfLength;
        leg.pose.referenceBend = leg.pose.hip.q.rotate(sample.leg.referenceBendInHip);
        leg.pose.soleInBone = Frame(sample.soleInBone);
        leg.pose.targetSole = Frame(sample.soleWorld);
        PxTransform levelSole(PxIdentity);
        leg.issue = FootstepPoseInputIssue::None;
        if (!MakeFootSoleTarget(leg.pose.targetSole.p, PxVec3(0, 1, 0), leg.pose.targetSole, levelSole))
        {
            leg.issue = FootstepPoseInputIssue::ReferenceSole;
        }
        else
        {
            constexpr FootstepPoseInputIssue shapeIssues[] = {FootstepPoseInputIssue::ThighShape,
                FootstepPoseInputIssue::CalfShape, FootstepPoseInputIssue::FootShape};
            for (size_t part = 0; part < leg.shapes.size(); ++part)
            {
                if (!leg.shapes[part].available)
                {
                    leg.issue = shapeIssues[part];
                    break;
                }
            }
        }
        FootstepReachInput& reach = reaches[side];
        reach.hip = leg.pose.hip.p;
        reach.position = leg.pose.targetSole.p;
        reach.ankleOffset = (levelSole * leg.pose.soleInBone.getInverse()).p - levelSole.p;
        reach.legLength = sample.leg.thighLength + sample.leg.calfLength;
        reach.sole = BuildFootSupportHull(FootstepPolygon(sample.corners.begin(), sample.corners.end()));
        reach.available = leg.issue == FootstepPoseInputIssue::None;
    }
}

FootstepPoseCheck FootstepLandingQuery::CheckPose(size_t side, const PxVec3& position, const PxVec3& normal,
    const PxVec3& displacement) const
{
    FootstepPoseCheck check(FootstepPoseFailure::InvalidInput);
    if (side >= m_Legs.size())
    {
        check.inputIssue = FootstepPoseInputIssue::FootIndex;
        return check;
    }
    if (!m_Ragdoll || !m_Asset)
    {
        check.inputIssue = !m_Ragdoll ? FootstepPoseInputIssue::Ragdoll : FootstepPoseInputIssue::PhysicsAsset;
        return check;
    }
    if (m_Legs[0].issue != FootstepPoseInputIssue::None || m_Legs[1].issue != FootstepPoseInputIssue::None)
    {
        check.legInputIssues = {m_Legs[0].issue, m_Legs[1].issue};
        return check;
    }
    const Leg& leg = m_Legs[side];
    TwoBoneIKInput input = leg.pose;
    check.ikIssue = BuildFootSoleTarget(position, normal, leg.pose.targetSole, input.targetSole, displacement);
    if (check.ikIssue != FootIKIssue::None)
    {
        return check;
    }
    const LegIKResult solved = SolveTwoBoneIK(input);
    if (!solved.valid)
    {
        check.failures = FootstepPoseFailure::Reach;
        check.ikIssue = solved.issue;
        return check;
    }
    // 목표에 닿지 못해도 IK가 계산한 자세로 관절 제한과 반대쪽 다리 겹침을 검사할 수 있다.
    check.failures = FootstepPoseFailure::None;
    if (solved.reachError > FootstepPoseCheck::SoleTolerance)
    {
        check.failures |= FootstepPoseFailure::Reach;
        check.soleError = solved.reachError;
        check.requestedHipAnkleDistance = (solved.requestedAnkle - solved.hip.p).magnitude();
        check.solvedHipAnkleDistance = (solved.ankle.p - solved.hip.p).magnitude();
        check.reachLimited = solved.reachLimited;
    }
    const std::array<PxTransform, 4> frames{{leg.anchor, solved.hip, solved.knee, solved.ankle}};
    check.failures |= CheckFootstepJointLimits(*m_Asset, m_Ragdoll->UsesArticulation(), leg.bodies, frames, &check.joints);
    const std::array<FootstepCollisionShape, 3>& oppositeShapes = m_Legs[1 - side].shapes;
    for (size_t moving = 0; moving < leg.shapes.size(); ++moving)
    {
        const FootstepCollisionShape& shape = leg.shapes[moving];
        const PxTransform pose = frames[moving + 1] * shape.inBone;
        for (size_t opposite = 0; opposite < oppositeShapes.size(); ++opposite)
        {
            const FootstepCollisionShape& other = oppositeShapes[opposite];
            if (PxGeometryQuery::overlap(shape.geometry.any(), pose, other.geometry.any(), other.actualPose))
            {
                const size_t pair = side == 0 ? moving * 3 + opposite : opposite * 3 + moving;
                check.overlapPairs |= static_cast<uint16_t>(1u << pair);
                check.failures |= FootstepPoseFailure::OppositeLegOverlap;
            }
        }
    }
    return check;
}

TwoBoneIKInput FootstepLandingQuery::MakeSwingReference(size_t side) const
{
    const Leg& leg = m_Legs[side];
    TwoBoneIKInput pose = leg.pose;
    const PhysicsBody& thigh = m_Asset->bodies[leg.bodies[1]];
    const PhysicsBody& calf = m_Asset->bodies[leg.bodies[2]];
    const int index = m_Asset->FindConstraint(thigh.bone, calf.bone);
    if (index < 0)
    {
        return pose;
    }
    const PhysicsConstraint& knee = m_Asset->constraints[index];
    if (knee.twist != PhysicsJointMotion::Limited || knee.swingY == PhysicsJointMotion::Free ||
        knee.swingZ == PhysicsJointMotion::Free)
    {
        return pose;
    }
    const float flexLimit = PxAbs(knee.twistMin) > PxAbs(knee.twistMax) ? knee.twistMin : knee.twistMax;
    const float lateralY = knee.swingY == PhysicsJointMotion::Locked ? 0.0f : knee.swingYLimit;
    const float lateralZ = knee.swingZ == PhysicsJointMotion::Locked ? 0.0f : PxMax(PxAbs(knee.swingZMin), PxAbs(knee.swingZMax));
    if (PxAbs(flexLimit) <= PxMax(lateralY, lateralZ))
    {
        return pose;
    }
    // 거의 펴진 다리의 세 점은 무릎 굽힘 평면을 잘 정하지 못한다. PA의 주 굽힘축으로 IK 기준만 만든다.
    const PxQuat parent = pose.hip.q * Frame(thigh.GetLocalTransform() * knee.parentFrame.GetTransform()).q;
    const PxQuat child = pose.knee.q * Frame(calf.GetLocalTransform() * knee.childFrame.GetTransform()).q;
    const PxQuat relative = parent.getConjugate() * child;
    const float current = std::remainder(2.0f * std::atan2(relative.x, relative.w), PxTwoPi);
    const float reference = PxClamp(.5f * flexLimit, -PxPi / 4.0f, PxPi / 4.0f);
    if ((reference - current) * reference > 0)
    {
        const PxQuat bend(reference - current, parent.getBasisVector0());
        pose.knee.q = bend * pose.knee.q;
        pose.ankle = PxTransform(pose.knee.p + bend.rotate(pose.ankle.p - pose.knee.p), bend * pose.ankle.q);
    }
    return pose;
}

bool FootstepLandingQuery::CheckPelvisTurn(size_t side, const FootstepCommand& command, float angle,
    float fixedDeltaTime, FootstepPelvisTurn& selected) const
{
    const float duration = GetFootstepRemainingTime(command);
    // 부드러운 회전의 최고 각속도는 평균의 1.5배다. 남은 시간에 못 도는 각도는 제외한다.
    if (side >= 2 || !m_Asset || !m_Ragdoll || !std::isfinite(angle) ||
        !std::isfinite(fixedDeltaTime) || fixedDeltaTime <= 0 || m_Legs[0].issue != FootstepPoseInputIssue::None ||
        m_Legs[1].issue != FootstepPoseInputIssue::None || m_Legs[0].bodies[0] != m_Legs[1].bodies[0] ||
        m_Legs[0].bodies[0] != m_Ragdoll->GetImpactRootBody() || !std::isfinite(duration) ||
        duration <= 0 || 1.5f * PxAbs(angle) > FootstepPelvisTurn::MaximumAngularSpeed * duration)
    {
        return false;
    }
    FootstepPelvisTurn turn;
    turn.pelvis = m_Legs[side].anchor;
    turn.angle = angle;
    turn.duration = duration;
    turn.legs = {{m_Legs[0].pose, m_Legs[1].pose}};
    turn.legs[side] = MakeSwingReference(side);
    PxTransform landingSole(PxIdentity);
    if (!MakeFootSoleTarget(command.target.position, command.target.normal, m_Legs[side].pose.targetSole,
        landingSole, command.target.position - command.startPosition))
    {
        return false;
    }

    turn.SetLandingSole(side, landingSole);
    FootstepCollisionInput path;
    path.fixedDeltaTime = fixedDeltaTime;
    path.maximumIntervals = FootstepCollisionInput::PelvisTurnMaximumIntervals;
    path.rejectInvalidPoses = true;
    path.acceptsPose = [this, &turn](size_t foot, const LegIKResult& solved, float time) -> bool
    {
        const PxTransform anchor = turn.FrameAt(turn.pelvis.p, time) * turn.pelvis;
        const std::array<PxTransform, 4> frames{{anchor, solved.hip, solved.knee, solved.ankle}};
        return !CheckFootstepJointLimits(*m_Asset, m_Ragdoll->UsesArticulation(), m_Legs[foot].bodies, frames);
    };
    path.legs[side].command = command;
    for (size_t foot = 0; foot < path.legs.size(); ++foot)
    {
        FootstepCollisionLeg& leg = path.legs[foot];
        leg.pelvisTurn = &turn;
        leg.side = foot;
        leg.pelvisPivot = turn.pelvis.p;
        leg.shapes = m_Legs[foot].shapes;
        LegIKResult landing;
        std::array<PxTransform, 3> shapes;
        if (!PredictFootstepLeg(leg, duration, shapes, landing) ||
            landing.reachError > FootstepPoseCheck::SoleTolerance || !path.acceptsPose(foot, landing, duration))
        {
            return false;
        }
    }
    const FootstepCollisionSnapshot checked = PredictFootstepCollisions(path, FootstepCollisionPass::DetourCandidate);
    if (checked.status != FootstepCollisionStatus::Clear || checked.jointLimited || checked.reachLimited)
    {
        return false;
    }
    selected = turn;
    return true;
}
