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

    float Extent(const PhysicsBody& body, const PxTransform& frame, const PxVec3& direction)
    {
        const PxVec3 axis = frame.q.rotateInv(direction);
        if (body.shape == PhysicsShapeType::Box)
        {
            return PxAbs(axis.x) * body.halfExtents.x + PxAbs(axis.y) * body.halfExtents.y +
                PxAbs(axis.z) * body.halfExtents.z;
        }
        return body.radius + (body.shape == PhysicsShapeType::Capsule ? PxAbs(axis.x) * body.halfLength : 0);
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

void SkeletalMeshComponent::ResetFallArms()
{
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
        const bool armWound = m_Ragdoll.FindArmWoundJoint(hit.body) >= 0;
        // 팔을 감싸는 도중에는 새 팔 상처만 손 목표를 교체한다.
        if (!armWound && (reach.armWound || (m_ImpactHandReach.armWound && m_Ragdoll.HasArmWound()) || IsFalling() ||
            IsGroundFallActive() || m_StandingUpState != StandingUpState::None))
        {
            continue;
        }
        const float impulse = hit.impulse.magnitudeSquared();
        if (armWound == reach.armWound && impulse <= largestImpulse)
        {
            continue;
        }
        strongest = &hit;
        largestImpulse = impulse;
        reach.armWound = armWound;
    }
    if (strongest)
    {
        const PhysicsWorld* physics = GetOwner().GetWorld()->GetPhysicsWorld();
        const PxTransform bone = ToFrame(m_Ragdoll.GetBoneWorld(strongest->body));
        reach.body = static_cast<int>(strongest->body);
        reach.pointInBone = bone.transformInv(strongest->position);
        reach.normalInBone = bone.q.rotateInv(strongest->outwardNormal.getNormalized());
        reach.step = physics->GetStepCount();
        reach.clock = physics->GetClockRevision();
        reach.approachStep = reach.step;
        if (reach.armWound && m_ImpactHandReach.body == reach.body && m_ImpactHandReach.clock == reach.clock)
        {
            // 같은 부위의 재피격은 유지 시간을 갱신하되 이미 다가간 손을 떼지 않는다.
            reach.approachStep = m_ImpactHandReach.approachStep;
            reach.hand = m_ImpactHandReach.hand;
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
    if (!goal.SampleTiming(age, approachAge, m_ImpactHandReach.armWound))
    {
        ResetFallArms();
        return;
    }
    if (goal.weight <= 0)
    {
        m_Ragdoll.SetArmDriveScales({1.0f, 1.0f});
        return;
    }
    const PxTransform bone = ToFrame(m_Ragdoll.GetBoneWorld(m_ImpactHandReach.body));
    goal.point = bone.transform(m_ImpactHandReach.pointInBone);
    goal.normal = bone.q.rotate(m_ImpactHandReach.normalInBone);
    goal.hitNode = m_Ragdoll.GetBoneNodes()[m_ImpactHandReach.body];
    goal.hand = m_ImpactHandReach.hand;
    if (!ApplyImpactPalmTarget(goal, arms, m_PhysicsSkeleton.parents, frames, corrected))
    {
        if (m_ImpactHandReach.armWound)
        {
            m_Ragdoll.SetArmDriveScales({1.0f, 1.0f});
        }
        else
        {
            ResetFallArms();
        }
        return;
    }
    m_ImpactHandReach.hand = goal.hand;
    // 선택한 팔만 손 목표를 추종하도록 힘을 올린다. 반대팔은 피격 반동을 유지한다.
    const float base = m_Ragdoll.GetArmDriveStrength();
    const float tracking = PxMax(base, .8f);
    std::array<float, 2> scales{base / tracking, base / tracking};
    scales[goal.hand] += (1.0f - scales[goal.hand]) * goal.weight;
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
        if (setup.anchorNode < 0 || hand == bodies.end() ||
            std::find(bodies.begin(), bodies.end(), setup.shoulderNode) == bodies.end() ||
            std::find(bodies.begin(), bodies.end(), setup.elbowNode) == bodies.end())
        {
            continue;
        }
        setup.handBody = static_cast<int>(hand - bodies.begin());
        const int chain[] = {setup.anchorNode, setup.shoulderNode, setup.elbowNode, setup.handNode};
        bool linked = true;
        for (size_t link = 1; link < 4; ++link)
        {
            linked &= std::any_of(m_PhysicsAsset->constraints.begin(), m_PhysicsAsset->constraints.end(),
                [&](const PhysicsConstraint& joint) -> bool
                {
                    return joint.parentBone == m_PhysicsSkeleton.names[chain[link - 1]] &&
                        joint.childBone == m_PhysicsSkeleton.names[chain[link]];
                });
        }
        if (!linked)
        {
            continue;
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
        const PxTransform body = wrist * ToFrame(shape.GetLocalTransform());
        const PxVec3 center = body.p - palmUp * Extent(shape, body, palmUp);
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
    if (!physics || (!m_ImpactHandReach.armWound && (IsGroundFallActive() || IsFalling() || m_StandingUpState != StandingUpState::None)))
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
        }
    }
    FallArmSnapshot corrected;
    ApplyImpactHandReach(*physics, arms, frames, corrected);
    WriteArmPose(frames, corrected.correctedNodes, target);
}
