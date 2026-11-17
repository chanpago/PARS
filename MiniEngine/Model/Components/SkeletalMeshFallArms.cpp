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
}

void SkeletalMeshComponent::ResetFallArms()
{
    m_FallArmBrace.Reset();
    m_Ragdoll.SetArmDriveScales({1.0f, 1.0f});
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
        setup.upInAnchor = anchor.q.rotateInv(up);
        PxVec3 fingers = ToFrame(referenceWorld[middle]).p - wrist.p;
        PxVec3 radial = ToFrame(referenceWorld[index]).p - ToFrame(referenceWorld[pinky]).p;
        if (fingers.normalize() < 1e-6f || radial.normalize() < 1e-6f)
        {
            continue;
        }
        // 손바닥의 +Y는 손등 쪽이다. 좌우 손의 해부학적 방향 차이를 반영한다.
        PxVec3 palmUp = radial.cross(fingers) * (side == 0 ? -1.0f : 1.0f);
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

void SkeletalMeshComponent::ApplyFallArmTargets(std::vector<Matrix4>& target, float deltaTime)
{
    World* world = GetOwner().GetWorld();
    PhysicsWorld* physics = world ? world->GetPhysicsWorld() : nullptr;
    if (!IsActiveRagdoll() || !physics || !m_FallArmSettings.enabled ||
        !std::isfinite(m_FallArmSettings.trackingStrength) || m_FallArmSettings.trackingStrength < 0 ||
        m_FallArmSettings.trackingStrength > 1)
    {
        ResetFallArms();
        return;
    }
    const FootSupportSnapshot& support = m_FootSupport.GetSnapshot();
    const bool canProtect = m_AutomaticRootSupport && m_StandingUpState == StandingUpState::None;
    const bool falling = canProtect && IsFollowingFallingAnimation() && m_RootSupportRelease != RootSupportRelease::None;
    // B 이탈 유예 중에도 팔은 즉시 준비한다. 골반 지지와 낙상 클립 전환은 기존 판정을 따른다.
    const bool preparing = canProtect && m_RootSupportRelease == RootSupportRelease::None &&
        support.balance.zone == FootSupportZone::Outside;
    const bool requested = falling || preparing;
    if (!requested && !m_FallArmBrace.GetSnapshot().active)
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
    FallArmInput input;
    // 발 샘플의 리비전은 물리 월드 시계 재설정과 래그돌 재생성을 함께 반영한다.
    input.revision = support.contactRevision;
    input.completedStep = physics->GetStepCount();
    input.deltaTime = deltaTime;
    input.falling = falling;
    input.preparing = preparing;
    for (size_t side = 0; side < input.arms.size(); ++side)
    {
        FallArmSample& arm = input.arms[side];
        arm.setup = m_FallArmSetups[side];
        if (!arm.setup.available)
        {
            continue;
        }
        arm.actualAnchor = ToFrame(m_Ragdoll.GetBoneWorld(arm.setup.anchorBody));
        arm.actualHand = ToFrame(m_Ragdoll.GetBoneWorld(arm.setup.handBody));
        const PxTransform mapping = arm.actualAnchor * frames[arm.setup.anchorNode].getInverse();
        const PxVec3 shoulder = (mapping * frames[arm.setup.shoulderNode]).p;
        arm.shoulderVelocity = m_Ragdoll.GetBodyPointVelocity(arm.setup.anchorBody, shoulder);
        PhysicsGroundContact contact;
        arm.touching = m_Ragdoll.GetBodyGroundContact(arm.setup.handBody, contact);
        arm.contactPosition = contact.position;
        arm.contactNormal = contact.normal;
    }
    const int root = m_Ragdoll.GetImpactRootBody();
    PhysicsGroundContact rootContact;
    Vector3 velocity(kZero);
    input.bodySettled = root >= 0 && m_Ragdoll.GetBodyGroundContact(root, rootContact) &&
        m_Ragdoll.GetCenterOfMassVelocity(velocity) && float(Length(velocity)) <= m_RecoverySettings.maximumReturnSpeed;
    const FallArmGroundQuery query = [&](const PxVec3& origin, float distance, PxVec3& position, PxVec3& normal) -> bool
    {
        PxRaycastHit hit;
        if (!physics->Raycast(origin, PxVec3(0, -distance, 0), hit, &GetOwner()))
        {
            return false;
        }
        position = hit.position;
        normal = hit.normal;
        return true;
    };
    m_FallArmBrace.Apply(input, m_FallArmSettings, m_PhysicsSkeleton.parents, query, frames);
    const FallArmSnapshot& snapshot = m_FallArmBrace.GetSnapshot();
    for (int node : snapshot.correctedNodes)
    {
        const Matrix4 stretch = Invert(PhysicsBoneFrame(target[node])) * target[node];
        target[node] = ToMatrix(frames[node]) * stretch;
    }
    if (requested)
    {
        m_Ragdoll.SetArmDriveScales({snapshot.arms[0].driveScale, snapshot.arms[1].driveScale}, m_FallArmSettings.trackingStrength);
    }
    else
    {
        // 회복하면 사용자 기본 강도를 복원하고, 남아 있는 팔 자세 보정만 부드럽게 해제한다.
        m_Ragdoll.SetArmDriveScales({1.0f, 1.0f});
    }
}
