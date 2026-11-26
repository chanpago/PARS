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
        if (!sample.available || !sample.leg.available)
        {
            continue;
        }
        for (size_t part = 0; part < 3; ++part)
        {
            const std::vector<int>::const_iterator node = std::find(nodes.begin(), nodes.end(), chain[part]);
            if (node == nodes.end())
            {
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
        leg.available = MakeFootSoleTarget(leg.pose.targetSole.p, PxVec3(0, 1, 0), leg.pose.targetSole, levelSole) &&
            leg.shapes[0].available && leg.shapes[1].available && leg.shapes[2].available;
        FootstepReachInput& reach = reaches[side];
        reach.hip = leg.pose.hip.p;
        reach.position = leg.pose.targetSole.p;
        reach.ankleOffset = (levelSole * leg.pose.soleInBone.getInverse()).p - levelSole.p;
        reach.legLength = sample.leg.thighLength + sample.leg.calfLength;
        reach.sole = BuildFootSupportHull(FootstepPolygon(sample.corners.begin(), sample.corners.end()));
        reach.available = leg.available;
    }
}

FootstepPoseFailures FootstepLandingQuery::CheckPose(size_t side, const PxVec3& position, const PxVec3& normal,
    const PxVec3& displacement) const
{
    if (side > 1 || !m_Legs[side].available || !m_Legs[1 - side].available || !m_Asset || !m_Ragdoll)
    {
        return FootstepPoseFailure::InvalidInput;
    }
    const Leg& leg = m_Legs[side];
    TwoBoneIKInput input = leg.pose;
    if (!MakeFootSoleTarget(position, normal, leg.pose.targetSole, input.targetSole, displacement))
    {
        return FootstepPoseFailure::InvalidInput;
    }
    const LegIKResult solved = SolveTwoBoneIK(input);
    if (!solved.valid)
    {
        return FootstepPoseFailure::Reach;
    }
    // 목표에 닿지 못해도 IK가 계산한 자세로 관절 제한과 반대쪽 다리 겹침을 검사할 수 있다.
    FootstepPoseFailures failures;
    if (solved.reachError > .0001f)
    {
        failures |= FootstepPoseFailure::Reach;
    }
    const std::array<PxTransform, 4> frames{{leg.anchor, solved.hip, solved.knee, solved.ankle}};
    failures |= CheckFootstepJointLimits(*m_Asset, m_Ragdoll->UsesArticulation(), leg.bodies, frames);
    for (size_t moving = 0; moving < 3; ++moving)
    {
        const FootstepCollisionShape& shape = leg.shapes[moving];
        const PxTransform pose = frames[moving + 1] * shape.inBone;
        for (const FootstepCollisionShape& other : m_Legs[1 - side].shapes)
        {
            if (PxGeometryQuery::overlap(shape.geometry.any(), pose, other.geometry.any(), other.actualPose))
            {
                failures |= FootstepPoseFailure::OppositeLegOverlap;
                return failures;
            }
        }
    }
    return failures;
}
