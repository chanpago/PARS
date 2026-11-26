#include "SkeletalMeshComponent.h"
#include "GameFramework/Actor.h"
#include "GameFramework/World.h"
#include "PhysicsWorld.h"
#include "foundation/PxMat33.h"
#include <algorithm>
#include <cmath>

using namespace PARS;
using namespace Math;
using namespace physx;

namespace
{
    PxTransform ToFrame(const Matrix4& matrix)
    {
        const PhysicsJointFrame frame = PhysicsJointFrame::FromTransform(PhysicsBoneFrame(matrix));
        return PxTransform(PxVec3(frame.offset.x, frame.offset.y, frame.offset.z),
            PxQuat(frame.rotation.x, frame.rotation.y, frame.rotation.z, frame.rotation.w));
    }

    Matrix4 ToMatrix(const PxTransform& frame)
    {
        return Matrix4(OrthogonalTransform(Quaternion(DirectX::XMVectorSet(frame.q.x, frame.q.y, frame.q.z, frame.q.w)),
            Vector3(frame.p.x, frame.p.y, frame.p.z)));
    }

    int FindNode(const PhysicsSkeleton& skeleton, const std::string& name)
    {
        for (size_t node = 0; node < skeleton.names.size(); ++node)
        {
            const std::string& candidate = skeleton.names[node];
            const size_t separator = candidate.find_last_of(':');
            if (candidate.substr(separator == std::string::npos ? 0 : separator + 1) == name)
            {
                return static_cast<int>(node);
            }
        }
        return -1;
    }

    int FindBody(const PhysicsSkeleton& skeleton, const Ragdoll& ragdoll, const std::string& name)
    {
        const std::vector<int>& nodes = ragdoll.GetBoneNodes();
        const int node = FindNode(skeleton, name);
        const std::vector<int>::const_iterator body = std::find(nodes.begin(), nodes.end(), node);
        return body != nodes.end() ? static_cast<int>(body - nodes.begin()) : -1;
    }

    FallArmJoint MakeArmJoint(const PhysicsAsset& asset, int index)
    {
        const PhysicsConstraint& constraint = asset.constraints[index];
        FallArmJoint joint;
        joint.index = index;
        joint.inParentBone = ToFrame(asset.bodies[asset.FindBody(constraint.parentBone)].GetLocalTransform() *
            constraint.parentFrame.GetTransform());
        joint.inChildBone = ToFrame(asset.bodies[asset.FindBody(constraint.childBone)].GetLocalTransform() *
            constraint.childFrame.GetTransform());
        return joint;
    }

    bool PhysicalArmFits(const Ragdoll& ragdoll, const FallArmSample& arm, const ImpactPalmGoal& goal, const PxTransform& contact)
    {
        const FallArmSetup& setup = arm.setup;
        if (goal.approach < 1.0f || !PalmFitsWound(arm.actualHand * setup.palmInHand, contact, .04f))
        {
            return false;
        }
        // 두 팔이 물리적으로 서로 다가간 경우에도 실제 관절 각도를 PA에 투영해 접촉을 확인한다.
        const std::array<PxTransform, 4> bones{arm.actualAnchor, ToFrame(ragdoll.GetBoneWorld(setup.shoulderBody)),
            ToFrame(ragdoll.GetBoneWorld(setup.elbowBody)), arm.actualHand};
        PxTransform projected = arm.actualAnchor;
        for (size_t link = 0; link < setup.armJoints.size(); ++link)
        {
            const FallArmJoint& joint = setup.armJoints[link];
            const PxQuat rotation = ((bones[link] * joint.inParentBone).q.getConjugate() *
                bones[link + 1].q * joint.inChildBone.q).getNormalized();
            const PxQuat limited = goal.limitJointRotation(joint.index, rotation);
            projected = projected * joint.inParentBone * PxTransform(limited) * joint.inChildBone.getInverse();
        }
        return PalmFitsWound(projected * setup.palmInHand, contact, .04f);
    }

    void WriteArmPose(const std::vector<PxTransform>& frames, const std::vector<int>& nodes, std::vector<Matrix4>& pose)
    {
        for (int node : nodes)
        {
            const Matrix4 stretch = Invert(PhysicsBoneFrame(pose[node])) * pose[node];
            pose[node] = ToMatrix(frames[node]) * stretch;
        }
    }
}

bool SkeletalMeshComponent::GetImpactPalmDistance(int& hitBody, float& meters) const
{
    hitBody = m_ImpactHandReach.body;
    const int hand = m_ImpactHandReach.hand;
    if (hitBody < 0 || static_cast<size_t>(hitBody) >= m_Ragdoll.GetBodyCount() || hand < 0)
    {
        return false;
    }
    const FallArmSetup& arm = m_FallArmSetups[hand];
    if (!arm.available || static_cast<size_t>(arm.handBody) >= m_Ragdoll.GetBodyCount())
    {
        return false;
    }
    const PxVec3 palm = ToFrame(m_Ragdoll.GetBoneWorld(arm.handBody)).transform(arm.palmInHand.p);
    const PxVec3 wound = ToFrame(m_Ragdoll.GetBoneWorld(hitBody)).transform(m_ImpactHandReach.pointInBone);
    meters = (palm - wound).magnitude();
    return true;
}

void SkeletalMeshComponent::ResetFallArms()
{
    m_Ragdoll.ReleaseWoundGrip();
    m_ImpactHandReach = {};
    m_Ragdoll.SetArmDriveScales({1.0f, 1.0f});
}

SkeletalMeshComponent::ImpactHandReach SkeletalMeshComponent::FindImpactHandReach(const std::vector<RagdollBodyImpulse>& impacts) const
{
    ImpactHandReach reach;
    const RagdollBodyImpulse* strongest = nullptr;
    float largestImpulse = 0;
    for (const RagdollBodyImpulse& hit : impacts)
    {
        if (!m_Ragdoll.IsPrimaryProjectileHit(hit, impacts) || !hit.outwardNormal.isFinite() ||
            hit.outwardNormal.magnitudeSquared() <= .5f)
        {
            continue;
        }
        const int woundJoint = m_Ragdoll.FindWoundJoint(hit.body);
        // 상처를 감싸는 동안 짧은 피격 반응이 같은 손의 목표를 가로채지 않게 한다.
        if (woundJoint < 0 && (reach.woundJoint >= 0 || (m_ImpactHandReach.woundJoint >= 0 && m_Ragdoll.HasWound()) || IsFalling() ||
            IsGroundFallActive() || m_StandingUpState != StandingUpState::None))
        {
            continue;
        }
        const float impulse = hit.impulse.magnitudeSquared();
        if ((woundJoint >= 0) == (reach.woundJoint >= 0) && impulse <= largestImpulse)
        {
            continue;
        }
        strongest = &hit;
        largestImpulse = impulse;
        reach.woundJoint = woundJoint;
    }
    if (strongest)
    {
        const PhysicsWorld* physics = GetOwner().GetWorld()->GetPhysicsWorld();
        const PxTransform bone = ToFrame(m_Ragdoll.GetBoneWorld(strongest->body));
        reach.body = static_cast<int>(strongest->body);
        reach.pointInBone = bone.transformInv(strongest->position);
        reach.normalInBone = bone.q.rotateInv(strongest->outwardNormal.getNormalized());
        const int leftShoulder = FindBody(m_PhysicsSkeleton, m_Ragdoll, "LeftArm");
        const int rightShoulder = FindBody(m_PhysicsSkeleton, m_Ragdoll, "RightArm");
        int pelvis = FindBody(m_PhysicsSkeleton, m_Ragdoll, "Hips");
        if (pelvis < 0)
        {
            pelvis = FindBody(m_PhysicsSkeleton, m_Ragdoll, "Pelvis");
        }
        if (leftShoulder >= 0 && rightShoulder >= 0 && pelvis >= 0)
        {
            reach.selection = SelectWoundHand(strongest->position, strongest->outwardNormal,
                ToFrame(m_Ragdoll.GetBoneWorld(leftShoulder)).p, ToFrame(m_Ragdoll.GetBoneWorld(rightShoulder)).p,
                ToFrame(m_Ragdoll.GetBoneWorld(pelvis)).p);
        }
        reach.step = physics->GetStepCount();
        reach.clock = physics->GetClockRevision();
        reach.approachStep = reach.step;
        const bool sameHand = reach.selection.hand < 0 || reach.selection.hand == m_ImpactHandReach.selection.hand ||
            reach.selection.hand == m_ImpactHandReach.hand;
        if (sameHand && reach.woundJoint >= 0 && m_ImpactHandReach.body == reach.body && m_ImpactHandReach.clock == reach.clock &&
            (m_ImpactHandReach.pointInBone - reach.pointInBone).magnitudeSquared() < .04f * .04f &&
            m_ImpactHandReach.normalInBone.dot(reach.normalInBone) > .5f)
        {
            // 같은 부위의 재피격은 유지 시간을 갱신하되 이미 다가간 손을 떼지 않는다.
            reach.approachStep = m_ImpactHandReach.approachStep;
            reach.hand = m_ImpactHandReach.hand;
            reach.armHistory = m_ImpactHandReach.armHistory;
            reach.supportStartStep = m_ImpactHandReach.supportStartStep;
            reach.supportHistory = m_ImpactHandReach.supportHistory;
            reach.freeLegWeight = m_ImpactHandReach.freeLegWeight;
            reach.restingLegWeight = m_ImpactHandReach.restingLegWeight;
            reach.legLoad = m_ImpactHandReach.legLoad;
            if (reach.selection.hand < 0)
            {
                reach.selection = m_ImpactHandReach.selection;
            }
        }
    }
    return reach;
}

void SkeletalMeshComponent::ApplyImpactHandReach(const PhysicsWorld& physics, const std::array<FallArmSample, 2>& arms,
    std::vector<PxTransform>& frames, FallArmSnapshot& corrected)
{
    const uint64_t step = physics.GetStepCount();
    if (m_ImpactHandReach.clock != physics.GetClockRevision() || step < m_ImpactHandReach.step ||
        static_cast<size_t>(m_ImpactHandReach.body) >= m_Ragdoll.GetBodyCount())
    {
        ResetFallArms();
        return;
    }
    const float age = static_cast<float>((step - m_ImpactHandReach.step) * PhysicsWorld::FixedDeltaTime);
    const float approachAge = static_cast<float>((step - m_ImpactHandReach.approachStep) * PhysicsWorld::FixedDeltaTime);
    ImpactPalmGoal goal;
    const int woundJoint = m_ImpactHandReach.woundJoint;
    const float gripAge = woundJoint >= 0 ? m_Ragdoll.GetWoundGripAge(woundJoint) : -1.0f;
    if (!goal.SampleTiming(age, approachAge, woundJoint >= 0, gripAge))
    {
        ResetFallArms();
        return;
    }
    if (goal.weight <= 0)
    {
        m_Ragdoll.ReleaseWoundGrip();
        m_Ragdoll.SetArmDriveScales({1.0f, 1.0f});
        return;
    }
    const PxTransform bone = ToFrame(m_Ragdoll.GetBoneWorld(m_ImpactHandReach.body));
    goal.point = bone.transform(m_ImpactHandReach.pointInBone);
    goal.normal = bone.q.rotate(m_ImpactHandReach.normalInBone);
    goal.hitNode = m_Ragdoll.GetBoneNodes()[m_ImpactHandReach.body];
    goal.selection = m_ImpactHandReach.selection;
    goal.hand = m_ImpactHandReach.hand;
    if (woundJoint >= 0)
    {
        goal.history = &m_ImpactHandReach.armHistory;
        goal.clavicleReachFraction = .27f;
        bool allowSupport = m_PhysicsAsset->bodies[m_ImpactHandReach.body].isCriticalWound;
        const FootSupportSnapshot& support = m_FootSupport.GetSnapshot();
        for (int side = 0; side < 2; ++side)
        {
            const FootLegSetup& leg = support.feet[side].leg;
            if (!leg.available || (goal.hitNode != leg.hipNode && goal.hitNode != leg.kneeNode))
            {
                continue;
            }
            goal.selection = {side, false};
            if (!IsFalling() && !IsGroundFallActive() && m_StandingUpState == StandingUpState::None)
            {
                const FootContactSample& injured = support.feet[side].contact;
                const FootContactSample& opposite = support.feet[1 - side].contact;
                const float totalImpulse = injured.normalImpulse + opposite.normalImpulse;
                const float load = totalImpulse > 1e-4f ? injured.normalImpulse / totalImpulse :
                    injured.grounded == opposite.grounded ? .5f : (injured.grounded ? 1.0f : 0.0f);
                m_ImpactHandReach.legLoad += .15f * (load - m_ImpactHandReach.legLoad);
                const bool freeLeg = injured.hasSample && !injured.grounded && opposite.grounded &&
                    !support.footstep.IsInProgress();
                m_ImpactHandReach.freeLegWeight = freeLeg ? PxMin(1.0f, m_ImpactHandReach.freeLegWeight + .08f) : 0;
                goal.freeLegWeight = m_ImpactHandReach.freeLegWeight;
                const PxVec3 thigh = frames[leg.kneeNode].p - frames[leg.hipNode].p;
                const PxVec3 offset = frames[goal.hitNode].transform(m_ImpactHandReach.pointInBone) - frames[leg.hipNode].p;
                const float thighDepth = PxClamp(offset.dot(thigh) / PxMax(1e-6f, thigh.magnitudeSquared()), 0.0f, 1.0f);
                // 허리를 손보다 천천히 굽혀 발 지지 제어가 체중 이동을 받아낼 시간을 준다.
                const float phase = PxClamp((approachAge - ImpactPalmGoal::RecoilTime) / .80f, 0.0f, 1.0f);
                goal.legBendWeight = (.35f + .65f * thighDepth) * (.94f + .12f * m_ImpactHandReach.legLoad) *
                    (phase * phase * (3.0f - 2.0f * phase));
                goal.pelvisNode = m_PhysicsSkeleton.parents[leg.hipNode];
                goal.injuredLeg = &leg;
                goal.oppositeLeg = &support.feet[1 - side].leg;
            }
            else if (IsGroundFallActive() && m_StandingUpState == StandingUpState::None && arms[side].setup.chestBody >= 0)
            {
                PhysicsGroundContact ground;
                const int root = m_Ragdoll.GetImpactRootBody();
                const bool onTorso = m_Ragdoll.GetBodyGroundContact(arms[side].setup.chestBody, ground) ||
                    (root >= 0 && m_Ragdoll.GetBodyGroundContact(root, ground));
                const PxVec3 up = arms[side].actualChest.q.rotate(arms[side].setup.upInChest);
                const bool resting = onTorso && PxAbs(up.dot(ground.normal)) < .65f && support.hasCenterOfMassVelocity &&
                    support.centerOfMassVelocity.magnitudeSquared() < .6f * .6f;
                m_ImpactHandReach.restingLegWeight = PxClamp(m_ImpactHandReach.restingLegWeight + (resting ? .04f : -.05f), 0.0f, 1.0f);
                goal.restingLegWeight = m_ImpactHandReach.restingLegWeight;
                goal.injuredLeg = &leg;
                goal.pelvisNode = m_PhysicsSkeleton.parents[leg.hipNode];
                goal.legBendWeight = 0;
            }
            // 다리를 짚을 때는 발을 옮기거나 넘어지는 동안 다른 팔을 균형 잡는 데 쓴다.
            allowSupport = allowSupport && goal.injuredLeg && !support.footstep.IsInProgress() &&
                (!IsGroundFallActive() || goal.restingLegWeight >= .5f);
            break;
        }
        uint64_t& supportStart = m_ImpactHandReach.supportStartStep;
        if (allowSupport && m_Ragdoll.IsHoldingWound())
        {
            supportStart = (std::min)(supportStart, step);
        }
        else if (!allowSupport || goal.weight >= 1.0f)
        {
            supportStart = UINT64_MAX;
        }
        if (supportStart != UINT64_MAX)
        {
            const float held = static_cast<float>((step - supportStart) * PhysicsWorld::FixedDeltaTime);
            goal.supportApproach = PxClamp((held - .10f) / ImpactPalmGoal::ReachTime, 0.0f, 1.0f);
            goal.supportHistory = &m_ImpactHandReach.supportHistory;
        }
        goal.limitJointRotation = [this](int index, const PxQuat& raw) -> PxQuat
        {
            const PhysicsConstraint& joint = m_PhysicsAsset->constraints[index];
            const Quaternion rotation(DirectX::XMVectorSet(raw.x, raw.y, raw.z, raw.w));
            const Quaternion limited = m_Ragdoll.UsesArticulation() ?
                ClampArticulationDriveTarget(joint, rotation) : ClampD6DriveTarget(joint, rotation);
            DirectX::XMFLOAT4 value;
            DirectX::XMStoreFloat4(&value, limited);
            return PxQuat(value.x, value.y, value.z, value.w);
        };
    }
    if (!ApplyImpactPalmTarget(goal, arms, m_PhysicsSkeleton.parents, frames, corrected))
    {
        if (woundJoint >= 0)
        {
            m_Ragdoll.ReleaseWoundGrip();
            m_Ragdoll.SetArmDriveScales({1.0f, 1.0f});
        }
        else
        {
            ResetFallArms();
        }
        return;
    }
    m_ImpactHandReach.hand = goal.hand;
    if (woundJoint >= 0)
    {
        const FallArmSample& arm = arms[goal.hand];
        const PxTransform palm = arm.actualHand * arm.setup.palmInHand;
        const FallArmTarget& target = corrected.arms[goal.hand];
        const bool canHold = target.canDriveWoundGrip || PhysicalArmFits(m_Ragdoll, arm, goal, target.palm);
        const PxTransform& driveTarget = m_Ragdoll.IsHoldingWound() || canHold ? target.palm : target.assistPalm;
        m_Ragdoll.DriveWoundGrip(woundJoint, arm.setup.handBody, m_ImpactHandReach.body, palm,
            driveTarget, goal.weight, canHold, target.canAssistWoundReach);
        const FallArmTarget& support = corrected.arms[1 - goal.hand];
        if (support.applied && m_Ragdoll.IsHoldingWound())
        {
            const FallArmSample& other = arms[1 - goal.hand];
            m_Ragdoll.DriveWoundSupport(arm.setup.handBody, other.setup.handBody,
                other.actualHand * other.setup.palmInHand, support.palm, support.weight);
        }
        else
        {
            m_Ragdoll.ReleaseWoundSupport();
        }
    }
    // 보조손이 참여하면 그 팔의 추종 강도만 주손보다 완만하게 올린다.
    const float base = m_Ragdoll.GetArmDriveStrength();
    const float tracking = PxMax(base, .8f);
    std::array<float, 2> scales{base / tracking, base / tracking};
    scales[goal.hand] += (1.0f - scales[goal.hand]) * goal.weight;
    const int otherSide = 1 - goal.hand;
    scales[otherSide] += (1.0f - scales[otherSide]) * .55f * corrected.arms[otherSide].weight;
    m_Ragdoll.SetArmDriveScales(scales, tracking);
}

void SkeletalMeshComponent::InitializeFallArms()
{
    m_FallArmsInitialized = true;
    m_FallArmSetups = {};
    if (!m_SkeletalMesh || !m_PhysicsAsset || m_PhysicsSkeleton.names.empty())
    {
        return;
    }
    const std::vector<GraphNode> reference(m_SkeletalMesh->m_SceneGraph.get(),
        m_SkeletalMesh->m_SceneGraph.get() + m_SkeletalMesh->m_NumNodes);
    std::vector<Matrix4> referenceWorld;
    std::string error;
    if (!EvaluatePhysicsPose(m_PhysicsSkeleton, reference, Matrix4(kIdentity), referenceWorld, error))
    {
        return;
    }
    const int leftArm = FindNode(m_PhysicsSkeleton, "LeftArm");
    const int rightArm = FindNode(m_PhysicsSkeleton, "RightArm");
    if (leftArm < 0 || rightArm < 0)
    {
        return;
    }
    PxVec3 right = ToFrame(referenceWorld[rightArm]).p - ToFrame(referenceWorld[leftArm]).p;
    right.y = 0;
    if (right.normalize() < 1e-6f)
    {
        return;
    }
    const PxVec3 up(0, 1, 0);
    // PA와 같은 정면 기준을 쓴다. 왼쪽과 위쪽의 외적은 위쪽과 오른쪽의 외적과 같다.
    const PxVec3 forward = up.cross(right);
    const std::vector<int>& bodies = m_Ragdoll.GetBoneNodes();
    for (size_t side = 0; side < m_FallArmSetups.size(); ++side)
    {
        FallArmSetup& setup = m_FallArmSetups[side];
        const std::string prefix = side == 0 ? "Left" : "Right";
        setup.clavicleNode = FindNode(m_PhysicsSkeleton, prefix + "Shoulder");
        setup.shoulderNode = FindNode(m_PhysicsSkeleton, prefix + "Arm");
        setup.elbowNode = FindNode(m_PhysicsSkeleton, prefix + "ForeArm");
        setup.handNode = FindNode(m_PhysicsSkeleton, prefix + "Hand");
        const int middle = FindNode(m_PhysicsSkeleton, prefix + "HandMiddle1");
        const int index = FindNode(m_PhysicsSkeleton, prefix + "HandIndex1");
        const int pinky = FindNode(m_PhysicsSkeleton, prefix + "HandPinky1");
        if (setup.shoulderNode < 0 || setup.elbowNode < 0 || setup.handNode < 0 || middle < 0 || index < 0 || pinky < 0)
        {
            continue;
        }
        for (int parent = m_PhysicsSkeleton.parents[setup.shoulderNode]; parent >= 0; parent = m_PhysicsSkeleton.parents[parent])
        {
            const std::vector<int>::const_iterator body = std::find(bodies.begin(), bodies.end(), parent);
            if (body != bodies.end())
            {
                setup.anchorNode = parent;
                setup.anchorBody = static_cast<int>(body - bodies.begin());
                break;
            }
        }
        const std::vector<int>::const_iterator hand = std::find(bodies.begin(), bodies.end(), setup.handNode);
        const std::vector<int>::const_iterator shoulderBody = std::find(bodies.begin(), bodies.end(), setup.shoulderNode);
        const std::vector<int>::const_iterator elbowBody = std::find(bodies.begin(), bodies.end(), setup.elbowNode);
        if (setup.anchorNode < 0 || hand == bodies.end() || shoulderBody == bodies.end() || elbowBody == bodies.end())
        {
            continue;
        }
        setup.handBody = static_cast<int>(hand - bodies.begin());
        setup.shoulderBody = static_cast<int>(shoulderBody - bodies.begin());
        setup.elbowBody = static_cast<int>(elbowBody - bodies.begin());
        const int chain[] = {setup.anchorNode, setup.shoulderNode, setup.elbowNode, setup.handNode};
        bool linked = true;
        for (size_t link = 0; link < setup.armJoints.size(); ++link)
        {
            const std::string& parent = m_PhysicsSkeleton.names[chain[link]];
            const int constraint = m_PhysicsAsset->FindConstraint(parent, m_PhysicsSkeleton.names[chain[link + 1]]);
            if (constraint < 0 || m_PhysicsAsset->constraints[constraint].parentBone != parent)
            {
                linked = false;
                break;
            }
            setup.armJoints[link] = MakeArmJoint(*m_PhysicsAsset, constraint);
        }
        if (!linked)
        {
            continue;
        }
        if (setup.anchorNode == setup.clavicleNode)
        {
            const std::vector<PhysicsConstraint>& joints = m_PhysicsAsset->constraints;
            const std::vector<PhysicsConstraint>::const_iterator joint = std::find_if(joints.begin(), joints.end(),
                [&](const PhysicsConstraint& entry) -> bool
                {
                    return entry.childBone == m_PhysicsSkeleton.names[setup.clavicleNode];
                });
            if (joint != joints.end())
            {
                setup.chestBody = m_PhysicsAsset->FindBody(joint->parentBone);
                if (setup.chestBody >= 0)
                {
                    setup.chestNode = bodies[setup.chestBody];
                    setup.clavicleJoint = MakeArmJoint(*m_PhysicsAsset, static_cast<int>(joint - joints.begin()));
                    setup.upInChest = ToFrame(referenceWorld[setup.chestNode]).q.rotateInv(up);
                }
            }
        }
        const PxTransform anchor = ToFrame(referenceWorld[setup.anchorNode]);
        const PxTransform shoulder = ToFrame(referenceWorld[setup.shoulderNode]);
        const PxTransform elbow = ToFrame(referenceWorld[setup.elbowNode]);
        const PxTransform wrist = ToFrame(referenceWorld[setup.handNode]);
        setup.upperLength = (elbow.p - shoulder.p).magnitude();
        setup.lowerLength = (wrist.p - elbow.p).magnitude();
        setup.forwardInAnchor = anchor.q.rotateInv(forward);
        setup.rightInAnchor = anchor.q.rotateInv(right);
        PxVec3 fingers = ToFrame(referenceWorld[middle]).p - wrist.p;
        PxVec3 radial = ToFrame(referenceWorld[index]).p - ToFrame(referenceWorld[pinky]).p;
        if (fingers.normalize() < 1e-6f || radial.normalize() < 1e-6f)
        {
            continue;
        }
        // 손바닥 프레임의 +Y는 손등, -Y는 손바닥이다.
        // PA 손 관절의 +Z가 손등을 향하는 것과 동일하게 좌우 부호를 맞춘다.
        PxVec3 palmUp = radial.cross(fingers) * (side == 0 ? 1.0f : -1.0f);
        if (palmUp.normalize() < 1e-6f)
        {
            continue;
        }
        const PhysicsBody& shape = m_PhysicsAsset->bodies[setup.handBody];
        setup.handShape.inBone = ToFrame(shape.GetLocalTransform());
        if (shape.shape == PhysicsShapeType::Box)
        {
            setup.handShape.halfExtents = PxVec3(shape.halfExtents.x, shape.halfExtents.y, shape.halfExtents.z);
        }
        else
        {
            setup.handShape.radius = shape.radius;
            setup.handShape.halfExtents.x = shape.shape == PhysicsShapeType::Capsule ? shape.halfLength : 0;
        }
        const PxTransform body = wrist * setup.handShape.inBone;
        const PxVec3 center = body.p - palmUp * setup.handShape.GetHalfExtent(body.q.rotateInv(palmUp));
        const PxVec3 palmRight = palmUp.cross(fingers).getNormalized();
        const PxTransform palm(center, PxQuat(PxMat33(palmRight, palmUp, fingers)).getNormalized());
        setup.palmInHand = wrist.getInverse() * palm;
        setup.available = setup.upperLength > 1e-5f && setup.lowerLength > 1e-5f && setup.palmInHand.isSane();
    }
}

void SkeletalMeshComponent::ApplyArmTargets(std::vector<Matrix4>& target)
{
    if (m_ImpactHandReach.body < 0)
    {
        return;
    }
    World* world = GetOwner().GetWorld();
    PhysicsWorld* physics = world ? world->GetPhysicsWorld() : nullptr;
    if (!physics || (m_ImpactHandReach.woundJoint < 0 && (IsGroundFallActive() || IsFalling() || m_StandingUpState != StandingUpState::None)))
    {
        ResetFallArms();
        return;
    }
    if (!m_FallArmsInitialized)
    {
        InitializeFallArms();
    }
    std::vector<PxTransform> frames;
    frames.reserve(target.size());
    for (const Matrix4& matrix : target)
    {
        frames.push_back(ToFrame(matrix));
    }
    std::array<FallArmSample, 2> arms;
    for (size_t side = 0; side < arms.size(); ++side)
    {
        FallArmSample& arm = arms[side];
        arm.setup = m_FallArmSetups[side];
        if (arm.setup.available)
        {
            arm.actualAnchor = ToFrame(m_Ragdoll.GetBoneWorld(arm.setup.anchorBody));
            arm.actualHand = ToFrame(m_Ragdoll.GetBoneWorld(arm.setup.handBody));
            if (m_ImpactHandReach.woundJoint >= 0 && arm.setup.chestBody >= 0)
            {
                arm.actualChest = ToFrame(m_Ragdoll.GetBoneWorld(arm.setup.chestBody));
            }
        }
    }
    FallArmSnapshot corrected;
    ApplyImpactHandReach(*physics, arms, frames, corrected);
    WriteArmPose(frames, corrected.correctedNodes, target);
}
