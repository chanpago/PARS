#include "SkeletalMeshComponent.h"
#include "../FootstepPoseValidation.h"
#include "GameFramework/World.h"
#include "PhysicsWorld.h"
#include <algorithm>
#include <cmath>

using namespace Math;
using namespace PARS;
using namespace physx;

namespace
{
    PxVec3 BonePoint(const Matrix4& bone, const PxVec3& local)
    {
        const Vector3 point(PhysicsBoneFrame(bone) * Vector3(local.x, local.y, local.z));
        return PxVec3(point.GetX(), point.GetY(), point.GetZ());
    }

    PxVec3 TorsoArm(const TorsoBalanceRig& rig, const std::vector<Matrix4>& bones)
    {
        const PxVec3 pivot = BonePoint(bones[rig.pelvisNode], rig.anchorInPelvisBone);
        PxVec3 arm(0);
        float mass = 0;
        for (const TorsoBalanceLink& link : rig.links)
        {
            arm += (BonePoint(bones[link.node], link.centerInBone) - pivot) * link.mass;
            mass += link.mass;
        }
        return arm / mass;
    }
}

bool SkeletalMeshComponent::InitializeTorsoBalance(TorsoBalanceRig& rig, TorsoBalance& balance, TorsoBalanceSample& sample)
{
    std::vector<Matrix4> reference;
    if (!m_Ragdoll.BuildTorsoBalanceRig(rig) || !m_Ragdoll.GetTorsoBalanceSample(rig, sample) ||
        !EvaluateAnimationPhysicsPose(m_PhysicsSkeleton, reference, m_PhysicsError))
    {
        return false;
    }
    balance.Begin(sample, TorsoBalance::GetLean(TorsoArm(rig, reference)));
    return balance.IsActive();
}

bool SkeletalMeshComponent::IsAboveTorsoPivot(const TorsoBalanceRig& rig, size_t body) const
{
    const std::vector<int>& nodes = m_Ragdoll.GetBoneNodes();
    if (rig.links.empty() || body >= nodes.size())
    {
        return false;
    }
    const int spine = rig.links.front().node;
    int node = nodes[body];
    while (node > spine)
    {
        node = m_PhysicsSkeleton.parents[node];
    }
    return node == spine;
}

void SkeletalMeshComponent::StartDynamicTorsoBalance(const std::vector<RagdollBodyImpulse>& impacts)
{
    if (m_TorsoBalance.IsActive() || !IsActiveRagdoll() || IsFalling() || IsGroundFallActive() ||
        m_StandingUpState != StandingUpState::None ||
        !std::any_of(impacts.begin(), impacts.end(), [](const RagdollBodyImpulse& hit) -> bool
        {
            return hit.newCapsuleHit && hit.impulse.magnitudeSquared() > 1e-8f;
        }))
    {
        return;
    }
    TorsoBalanceSample sample;
    if (InitializeTorsoBalance(m_TorsoRig, m_TorsoBalance, sample))
    {
        m_PreviousPivotVelocity = sample.pivotVelocity;
        m_TorsoClock = GetOwner().GetWorld()->GetPhysicsWorld()->GetClockRevision();
        m_Ragdoll.SetTorsoTorqueLimits(m_TorsoRig, m_TorsoBalance.GetHipTorqueLimit());
    }
}

void SkeletalMeshComponent::ResetTorsoBalance()
{
    m_Ragdoll.SetTorsoTorqueLimits(m_TorsoRig, 0);
    m_TorsoBalance = {};
    m_TorsoRig = {};
    m_TorsoClock = UINT64_MAX;
}

void SkeletalMeshComponent::UpdateDynamicTorsoBalance(float deltaTime)
{
    if (!m_TorsoBalance.IsActive() || IsImpactWarmupActive())
    {
        return;
    }
    TorsoBalanceSample measured;
    if (!IsActiveRagdoll() || IsFalling() || IsGroundFallActive() || m_StandingUpState != StandingUpState::None ||
        GetOwner().GetWorld()->GetPhysicsWorld()->GetClockRevision() != m_TorsoClock ||
        !m_Ragdoll.GetTorsoBalanceSample(m_TorsoRig, measured))
    {
        ResetTorsoBalance();
        return;
    }
    // 동적 바디는 솔버가 만든 각속도를 사용한다. 진자는 다음 목표만 예측한다.
    m_TorsoBalance.Observe(measured);
    const FootSupportSnapshot& support = m_FootSupport.GetSnapshot();
    if (m_TorsoBalance.CanFinish(deltaTime, support.comInsideHull &&
        (support.feet[0].contact.grounded || support.feet[1].contact.grounded)))
    {
        ResetTorsoBalance();
        return;
    }
    m_TorsoBalance.Advance(deltaTime, measured.pivotVelocity - m_PreviousPivotVelocity);
    m_PreviousPivotVelocity = measured.pivotVelocity;
}

void SkeletalMeshComponent::ApplyTorsoBalancePose(std::vector<Matrix4>& worldPose) const
{
    const PxVec3 desired = m_TorsoBalance.GetDirection();
    // 가슴부터 굽힘을 나누고 허리에서 남은 오차를 맞춰 전체 몸통 질량중심이 목표 방향을 향하게 한다.
    for (size_t count = m_TorsoRig.links.size(); count > 0; --count)
    {
        const TorsoBalanceLink& link = m_TorsoRig.links[count - 1];
        const PxVec3 actual = TorsoArm(m_TorsoRig, worldPose).getNormalized();
        PxVec3 axis = actual.cross(desired);
        const float sine = axis.normalize();
        if (sine < 1e-6f)
        {
            continue;
        }
        const float angle = std::atan2(sine, actual.dot(desired)) / static_cast<float>(count);
        const size_t parentBody = count == 1 ? m_TorsoRig.pelvis : m_TorsoRig.links[count - 2].body;
        const int parentNode = count == 1 ? m_TorsoRig.pelvisNode : m_TorsoRig.links[count - 2].node;
        const PhysicsConstraint& joint = m_PhysicsAsset->constraints[link.joint];
        const Quaternion parent = Normalize(Quaternion(PhysicsBoneFrame(worldPose[parentNode]) *
            m_PhysicsAsset->bodies[parentBody].GetLocalTransform() * joint.parentFrame.GetTransform()));
        const Quaternion child = Normalize(Quaternion(PhysicsBoneFrame(worldPose[link.node]) *
            m_PhysicsAsset->bodies[link.body].GetLocalTransform() * joint.childFrame.GetTransform()));
        const Quaternion turn(Vector3(axis.x, axis.y, axis.z), angle);
        const Quaternion relative = Normalize(~parent * turn * child);
        // 예열 포즈도 인계받을 물리 관절의 가동 범위 안에 둔다.
        const Quaternion limited = IsActiveRagdoll() && !m_Ragdoll.UsesArticulation() ?
            ClampD6DriveTarget(joint, relative) : ClampArticulationDriveTarget(joint, relative);
        DirectX::XMFLOAT4 rotationValue;
        DirectX::XMStoreFloat4(&rotationValue, Normalize(parent * limited * ~child));
        const PxQuat bend(rotationValue.x, rotationValue.y, rotationValue.z, rotationValue.w);
        const PxVec3 pivot = BonePoint(worldPose[link.node], link.anchorInBone);
        const Matrix4 rotation = FootstepPoseMatrix(PxTransform(pivot - bend.rotate(pivot), bend));
        for (size_t node = static_cast<size_t>(link.node); node < worldPose.size(); ++node)
        {
            int ancestor = static_cast<int>(node);
            while (ancestor > link.node)
            {
                ancestor = m_PhysicsSkeleton.parents[ancestor];
            }
            if (ancestor == link.node)
            {
                worldPose[node] = rotation * worldPose[node];
            }
        }
    }
}

bool SkeletalMeshComponent::UpdateWarmupTorsoPose(float deltaTime)
{
    if (!m_TorsoBalance.IsActive())
    {
        return true;
    }
    const Matrix4 component(GetComponentTransform());
    std::vector<Matrix4> bones;
    if (!EvaluatePhysicsPose(m_PhysicsSkeleton, m_ImpactWarmup.pose, component, bones, m_PhysicsError))
    {
        return false;
    }
    const PxVec3 pivot = BonePoint(bones[m_TorsoRig.pelvisNode], m_TorsoRig.anchorInPelvisBone);
    const PxVec3 velocity = (pivot - m_ImpactWarmup.previousPivot) / deltaTime;
    m_TorsoBalance.Advance(deltaTime, velocity - m_PreviousPivotVelocity);
    m_ImpactWarmup.previousPivot = pivot;
    m_PreviousPivotVelocity = velocity;
    ApplyTorsoBalancePose(bones);
    const PxVec3 realized = TorsoBalance::GetLean(TorsoArm(m_TorsoRig, bones));
    PxVec3 outward = m_TorsoBalance.lean - realized;
    if (outward.normalize() > .001f)
    {
        // 관절 끝에 닿으면 굽힘 속도의 바깥 성분만 관절 반력으로 상쇄한다.
        m_TorsoBalance.rate -= outward * PxMax(0.0f, m_TorsoBalance.rate.dot(outward));
        m_TorsoBalance.lean = realized;
    }
    std::vector<int> nodes;
    std::vector<Matrix4> targets;
    for (const TorsoBalanceLink& link : m_TorsoRig.links)
    {
        nodes.push_back(link.node);
        targets.push_back(bones[link.node]);
    }
    return OverridePhysicsPose(m_PhysicsSkeleton, m_ImpactWarmup.pose, component, nodes, targets, m_ImpactWarmup.pose);
}
