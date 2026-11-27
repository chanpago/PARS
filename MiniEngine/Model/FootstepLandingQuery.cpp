#include "FootstepLandingQuery.h"
#include "FootstepPoseValidation.h"
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
        PrepareFootIKJointLimits(*m_Asset, m_Ragdoll->UsesArticulation(), leg.bodies, leg.anchor.q, leg.jointLimits);
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
    const PxVec3& soleForward) const
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
    input.jointLimits = &leg.jointLimits;
    check.ikIssue = BuildFootSoleTarget(position, normal, leg.pose.targetSole, input.targetSole, soleForward);
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
    // 도달 오차가 있어도 계산된 자세로 다른 탈락 사유와 관절 보정량을 함께 확인한다.
    check.failures = FootstepPoseFailure::None;
    if (solved.reachError > FootstepPoseCheck::SoleTolerance || solved.soleAngleError > FootstepSoleAngleTolerance)
    {
        check.failures |= FootstepPoseFailure::Reach;
        check.soleError = solved.reachError;
        check.soleAngleError = solved.soleAngleError;
        check.requestedHipAnkleDistance = (solved.requestedAnkle - solved.hip.p).magnitude();
        check.solvedHipAnkleDistance = (solved.ankle.p - solved.hip.p).magnitude();
        check.reachLimited = solved.reachLimited;
        check.jointAdjusted = solved.jointAdjusted;
    }
    const std::array<PxTransform, 4> frames{{leg.anchor, solved.hip, solved.knee, solved.ankle}};
    const FootstepJointStatus jointStatus = CheckFootstepJointLimits(*m_Asset, m_Ragdoll->UsesArticulation(),
        leg.bodies, frames, &check.joints);
    if (jointStatus == FootstepJointStatus::InvalidInput || !leg.jointLimits.project)
    {
        check.failures |= FootstepPoseFailure::InvalidInput;
    }
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
