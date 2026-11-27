#include "SkeletalMeshComponent.h"
#include "../FootstepPoseValidation.h"
#include "../GameFramework/Character.h"
#include "GameFramework/World.h"
#include "PhysicsWorld.h"
#include <algorithm>
#include <cmath>

using namespace Math;
using namespace PARS;
using namespace physx;

namespace
{
    constexpr float MaximumRestSpeed = .25f;
    constexpr float FirstLandingTurnFraction = .6f;
    constexpr float TorsoLeanPeakTime = .1f;
    constexpr float MaximumTorsoLean = PxPi / 8.0f;

    PxTransform RigidFrame(const Matrix4& matrix)
    {
        const PhysicsJointFrame frame = PhysicsJointFrame::FromTransform(PhysicsBoneFrame(matrix));
        return PxTransform(PxVec3(frame.offset.x, frame.offset.y, frame.offset.z),
            PxQuat(frame.rotation.x, frame.rotation.y, frame.rotation.z, frame.rotation.w));
    }
}

bool SkeletalMeshComponent::ImpactWarmup::ApplyCapsuleImpacts(const std::vector<RagdollBodyImpulse>& impacts)
{
    bool pushed = false;
    for (const RagdollBodyImpulse& contact : impacts)
    {
        if (!contact.fromCharacterCapsule)
        {
            continue;
        }
        PxVec3 direction(contact.impulse.x, 0, contact.impulse.z);
        const float impulse = direction.normalize();
        if (impulse <= 1e-5f)
        {
            continue;
        }
        pushed = true;
        // 애니메이션이 직접 미는 몸통은 최초 접근 속도까지 반응하고, 물리 플레이어는 실제 접촉 속도를 따른다.
        const float speed = contact.capsuleVelocity.dot(direction);
        const bool originalDriver = contact.fromAnimatedCapsule && contact.capsuleGroup == approachGroup;
        const float limit = originalDriver ? PxMax(speed, approachVelocity.dot(direction)) : speed;
        const float effectiveMass = contact.fromAnimatedCapsule ? torsoMass : mass;
        const float remaining = mass * limit - momentum.dot(direction);
        momentum += direction * PxClamp(remaining, 0.0f, impulse * (mass / effectiveMass));
    }
    return pushed;
}

bool SkeletalMeshComponent::ImpactWarmup::ApplyTorsoLean(const PhysicsSkeleton& skeleton, const PxQuat& currentRotation,
    float elapsed, std::string& error)
{
    PxVec3 axis = currentRotation.rotateInv(peakTorsoLean);
    const float peakAngle = axis.normalize();
    if (peakAngle <= 1e-6f || spineNodes.empty())
    {
        return true;
    }

    std::vector<Matrix4> bones;
    if (!EvaluatePhysicsPose(skeleton, pose, Matrix4(kIdentity), bones, error))
    {
        return false;
    }
    // 최초 각충격량의 임계감쇠 응답에 예열 시작부터의 경과 시간을 쓴다.
    const float t = elapsed / TorsoLeanPeakTime;
    const float jointAngle = peakAngle * t * std::exp(1.0f - t) / static_cast<float>(spineNodes.size());
    for (int node : spineNodes)
    {
        // 이번 틱의 부모 자세에서 회전축을 구해 각 척추 관절에 같은 방향으로 굽힘을 더한다.
        const PxVec3 parentAxis = RigidFrame(bones[skeleton.parents[node]]).q.rotateInv(axis);
        const Quaternion bend(Vector3(parentAxis.x, parentAxis.y, parentAxis.z), jointAngle);
        GraphNode& joint = pose[node];
        joint.rotation = Normalize(bend * joint.rotation);
        if (!joint.staleMatrix)
        {
            joint.xform.Set3x3(Matrix3(bend) * joint.xform.Get3x3());
        }
    }
    return true;
}

ImpactWarmupStep SkeletalMeshComponent::ImpactWarmup::PlanStep(const FootstepPolygon& hull, const PxVec3& com,
    const PxVec3& velocity, bool secondStep) const
{
    const float legLength = PxMin(legs[0].thighLength + legs[0].calfLength, legs[1].thighLength + legs[1].calfLength);
    ImpactWarmupStep step = PlanImpactWarmupStep(hull, com, velocity, legLength * .22f,
        static_cast<float>(PhysicsWorld::FixedDeltaTime));
    // COM이 밀리는 방향으로 디디되, 회전한 발 배치에서도 원래 좌우 간격의 20%는 남긴다.
    const PxQuat stance = secondStep ? PxQuat(turnYaw, PxVec3(0, 1, 0)) * componentRotation : componentRotation;
    const float lateral = PxAbs(stance.rotateInv(step.stride).x);
    const float width = PxAbs(legs[0].targetSole.p.x - legs[1].targetSole.p.x);
    if (lateral > .4f * width)
    {
        step.stride *= (.4f * width) / lateral;
    }
    return step;
}

float SkeletalMeshComponent::ImpactWarmup::GetLandingSoleYaw(size_t stepIndex) const
{
    const size_t side = stepIndex == 0 ? firstFoot : 1u - firstFoot;
    const PxTransform& sole = legs[side].targetSole;
    const PxQuat landingTurn(turnYaw, PxVec3(0, 1, 0));
    const PxVec3 startOffset = componentRotation.rotate(sole.p - turnPivotInComponent);
    // 양발에 공통인 슬라이드는 제외한다. 두 번째 이동발은 첫 보폭만큼 뒤에서 출발한다.
    PxVec3 displacement = landingTurn.rotate(startOffset) - startOffset + steps[0].stride;
    if (stepIndex == 1)
    {
        displacement += steps[1].stride;
    }
    const PxVec3 turnedForward = (landingTurn * componentRotation * sole.q).rotate(PxVec3(0, 0, 1));
    const PxVec3 forward = GetFootstepTravelForward(displacement, turnedForward);
    if (forward.isZero())
    {
        return turnYaw;
    }
    return turnYaw + std::atan2(turnedForward.cross(forward).y, turnedForward.dot(forward));
}

PxTransform SkeletalMeshComponent::ImpactWarmup::GetPelvisBrace(const PxQuat& rotation, float elapsed, float maximumLowering) const
{
    const PxVec3 up = rotation.rotateInv(PxVec3(0, 1, 0));
    const float legLength = PxMin(legs[0].thighLength + legs[0].calfLength, legs[1].thighLength + legs[1].calfLength);
    const float progress = PxMin(elapsed / TorsoLeanPeakTime, 1.0f);
    const float weight = progress * progress * (3.0f - 2.0f * progress);
    const float stride = steps[0].stride.magnitude();
    // 감속 중에도 첫 보폭만큼은 무릎을 굽힌 채 마지막 지지발에 하중을 넘긴다.
    const float travelDistance = PxMax(momentum.magnitude() * (TorsoLeanPeakTime / mass), stride);
    // 지지발을 둔 채 반응 시간만큼 골반이 밀려도 다리가 닿도록 수평 보폭에 필요한 높이 여유를 낸다.
    const float horizontalReach = PxMin(travelDistance, legLength);
    const float verticalReach = std::sqrt(PxMax(0.0f, legLength * legLength - horizontalReach * horizontalReach));
    const float lowering = PxMin(PxMax(legLength - verticalReach, stride), maximumLowering) * weight;
    const float angle = std::atan2(PxMin(travelDistance, maximumLowering), legLength) * weight;
    const PxQuat tilt(angle, rotation.rotateInv(impactDirection).cross(up).getNormalized());
    const PxVec3 pivot = RigidFrame(sourceBones[pelvisNode]).p;
    return PxTransform(pivot - tilt.rotate(pivot) - up * lowering, tilt);
}

void SkeletalMeshComponent::UpdateCapsuleImpactFacing(float yawSpeedChange, float deltaTime)
{
    if (!IsActiveRagdoll() || !m_Ragdoll.IsRootSupported() || !m_FootSupport.GetSnapshot().hasCenterOfMass ||
        IsFalling() || IsGroundFallActive())
    {
        m_CapsuleFacingAllowance = 0;
        return;
    }
    // 실제로 돈 골반만 정면에 반영하고, 남은 회전각은 발딛기 시간에 맞춰 감쇠한다.
    m_CapsuleFacingAllowance += PlanCapsuleImpactTurn(yawSpeedChange, MaximumFootstepDuration);
    m_CapsuleFacingAllowance = PxClamp(m_CapsuleFacingAllowance, -PxHalfPi, PxHalfPi);
    if (PxAbs(m_CapsuleFacingAllowance) < 1e-5f)
    {
        m_CapsuleFacingAllowance = 0;
        return;
    }
    const float actualStep = PxClamp(m_Ragdoll.GetImpactRootYawSpeed() * deltaTime,
        PxMin(0.0f, m_CapsuleFacingAllowance), PxMax(0.0f, m_CapsuleFacingAllowance));
    const float yaw = PxClamp(m_Ragdoll.GetSupportedPelvisYawOffset(), PxMin(0.0f, actualStep), PxMax(0.0f, actualStep));
    m_CapsuleFacingAllowance = (m_CapsuleFacingAllowance - yaw) * std::exp(-deltaTime / MaximumFootstepDuration);
    if (yaw != 0)
    {
        const PxVec3 center = m_FootSupport.GetSnapshot().centerOfMass;
        const Vector3 pivot(center.x, center.y, center.z);
        const Matrix3 turn = Matrix3::MakeYRotation(yaw);
        AffineTransform actor = GetOwner().GetActorTransform();
        actor.SetBasis(turn * actor.GetBasis());
        actor.SetTranslation(pivot + turn * (actor.GetTranslation() - pivot));
        if (GetOwner().SetActorTransform(actor))
        {
            StartShoveReaction(yaw);
        }
    }
}

bool SkeletalMeshComponent::UpdateImpactWarmup(float deltaTime, const std::vector<RagdollImpactResponse>& responses, float capsuleYawSpeed)
{
    if (!m_AutomaticRootSupport || (!m_AutomaticRagdoll && !IsActiveRagdoll() && !IsImpactWarmupActive()))
    {
        return false;
    }
    const FootSupportSnapshot& sample = m_FootSupport.GetSnapshot();
    const bool sliding = IsImpactWarmupActive();
    PxVec3 impulse(0);
    bool projectile = false;
    bool newCapsuleHit = false;
    const std::vector<RagdollBodyImpulse>& impacts = m_Ragdoll.GetExternalImpacts();
    for (const RagdollBodyImpulse& contact : impacts)
    {
        if (contact.fromCharacterCapsule)
        {
            impulse += contact.impulse;
            newCapsuleHit = newCapsuleHit || contact.newCapsuleHit;
        }
        projectile = projectile || contact.fromProjectile;
    }
    for (const RagdollImpactResponse& response : responses)
    {
        if (response.fromCharacterCapsule)
        {
            impulse += response.impulse + response.partImpulse;
        }
    }
    impulse.y = 0;
    if (sliding)
    {
        if (m_ImpactWarmup.landings == 2 || newCapsuleHit || !responses.empty())
        {
            // 예열 도중 별개의 충격이 들어오면 현재 속도를 인계하고 이번 접촉 전체를 동적 바디에 적용한다.
            return FinishImpactWarmup(false, true);
        }
        const PxVec3 previousMomentum = m_ImpactWarmup.momentum;
        const bool pushed = m_ImpactWarmup.ApplyCapsuleImpacts(impacts);
        if (projectile)
        {
            return FinishImpactWarmup(false);
        }
        m_Ragdoll.AdvanceImpactReaction(m_ImpactWarmup.reaction, impacts, deltaTime);
        if (!pushed)
        {
            // 미는 힘이 끊기면 바닥의 동마찰만큼 감속하고, 남은 속도를 실제 발딛기에 넘긴다.
            PxVec3 direction = m_ImpactWarmup.momentum;
            const float momentum = direction.normalize();
            // 골반 높이는 별도로 지지하므로 미끄러지는 몸통의 하중만 마찰에 반영한다.
            const float frictionImpulse = m_ImpactWarmup.torsoMass * m_ImpactWarmup.slideDeceleration * deltaTime;
            m_ImpactWarmup.momentum -= direction * PxMin(momentum, frictionImpulse);
        }
        if (m_ImpactWarmup.landings == 1 && m_ImpactWarmup.stepTime == 0 && sample.hasComProjection && sample.hull.size() >= 3)
        {
            // 착지 직후 실측 COM 속도에는 직전 루트 속도가 이미 들어 있다. 이번 틱의 속도 변화만 더한다.
            const PxVec3 velocity = sample.hasCenterOfMassVelocity ?
                sample.centerOfMassVelocity + (m_ImpactWarmup.momentum - previousMomentum) / m_ImpactWarmup.mass :
                m_ImpactWarmup.momentum / m_ImpactWarmup.mass;
            const ImpactWarmupStep corrected = m_ImpactWarmup.PlanStep(sample.hull, sample.projectedCenterOfMass, velocity, true);
            if (corrected.duration > 0)
            {
                m_ImpactWarmup.steps[1] = corrected;
            }
        }
        return AdvanceImpactWarmup(deltaTime);
    }

    const bool standing = m_StandingUpState == StandingUpState::None && !IsBlendingToKinematic() &&
        !IsFalling() && !IsGroundFallActive() && (!IsRagdoll() || (IsActiveRagdoll() && m_Ragdoll.IsRootSupported())) &&
        sample.hasComProjection && sample.hasCenterOfMassVelocity && sample.totalMass > 0 &&
        sample.feet[0].available && sample.feet[0].leg.available && sample.feet[1].available && sample.feet[1].leg.available;
    if (standing && m_ImpactWarmup.wasResting && !projectile && !m_Ragdoll.HasWound() &&
        impulse.magnitude() >= ImpactWarmup::MinimumImpactSpeedChange * sample.totalMass &&
        BeginImpactWarmup(responses, impulse, capsuleYawSpeed, deltaTime))
    {
        return AdvanceImpactWarmup(deltaTime);
    }
    // 접근 감지로 이미 동적으로 바뀌었더라도, 피격 전 완료 틱의 속도로 정지 여부를 판단한다.
    const PxVec3 velocity(sample.centerOfMassVelocity.x, 0, sample.centerOfMassVelocity.z);
    const bool idle = m_AnimInstance->GetAnimationName(m_AnimInstance->GetCurrentAnimationIndex()) == "Idle";
    const Character* character = dynamic_cast<const Character*>(&GetOwner());
    const bool quietFeet = std::all_of(sample.feet.begin(), sample.feet.end(), [](const FootSoleSample& foot) -> bool
    {
        return foot.contact.grounded || *std::min_element(foot.distances.begin(), foot.distances.end()) < .06f;
    });
    m_ImpactWarmup.wasResting = standing && character && !sample.footstep.IsInProgress() && (idle || quietFeet) &&
        character->GetCharacterMovement().GetLocomotionSpeed() <= MaximumRestSpeed &&
        sample.centerOfMassVelocity.magnitudeSquared() <= MaximumRestSpeed * MaximumRestSpeed;
    m_ImpactWarmup.restVelocity = velocity;
    return false;
}

bool SkeletalMeshComponent::BeginImpactWarmup(const std::vector<RagdollImpactResponse>& responses, const PxVec3& impulse,
    float capsuleYawSpeed, float deltaTime)
{
    Character* character = dynamic_cast<Character*>(&GetOwner());
    if (!character)
    {
        return false;
    }
    // 다른 서기 클립에서 멈춰 있었어도 현재 자세에서 출발하고, 인계 후에는 기존 Idle 회복기를 사용한다.
    const uint32_t currentClip = m_AnimInstance->GetCurrentAnimationIndex();
    const uint32_t idleClip = GetIdleAnimationIndex();
    if (idleClip == UINT32_MAX)
    {
        return false;
    }
    if (IsRagdoll())
    {
        UpdatePhysicsPose();
    }
    const FootSupportSnapshot& sample = m_FootSupport.GetSnapshot();
    ImpactWarmup next;
    next.sourcePose = GetFinalPose();
    if (!EvaluatePhysicsPose(m_PhysicsSkeleton, next.sourcePose, Matrix4(kIdentity), next.sourceBones, m_PhysicsError))
    {
        return false;
    }
    next.pose = next.sourcePose;
    const int pelvisBody = m_Ragdoll.GetImpactRootBody();
    if (pelvisBody < 0)
    {
        return false;
    }
    next.pelvisNode = m_Ragdoll.GetBoneNodes()[pelvisBody];
    const Vector3 pelvis(m_Ragdoll.GetBoneWorld(pelvisBody).GetW());
    const PxVec3 pelvisPosition(pelvis.GetX(), pelvis.GetY(), pelvis.GetZ());
    float torsoLengthSquared = 0;
    // 예열 중 다리는 위치로 따라간다. 캡슐이 직접 미는 골반과 척추의 질량으로 루트 반응을 구한다.
    for (size_t body = 0; body < m_PhysicsAsset->bodies.size(); ++body)
    {
        if (m_Ragdoll.IsCharacterImpactBody(body))
        {
            next.torsoMass += m_PhysicsAsset->bodies[body].mass;
            const int node = m_Ragdoll.GetBoneNodes()[body];
            if (static_cast<int>(body) != pelvisBody && m_PhysicsSkeleton.parents[node] >= 0)
            {
                next.spineNodes.push_back(node);
                const Vector3 spine(m_Ragdoll.GetBoneWorld(body).GetW());
                torsoLengthSquared = PxMax(torsoLengthSquared, float(LengthSquare(spine - pelvis)));
            }
        }
    }
    if (next.torsoMass <= 0)
    {
        return false;
    }
    next.mass = sample.totalMass;
    next.impactDirection = impulse.getNormalized();
    next.momentum = m_ImpactWarmup.restVelocity * next.mass;
    const std::vector<RagdollBodyImpulse>& impacts = m_Ragdoll.GetExternalImpacts();
    // 예열 전에 남아 있던 피격 반응은 전신 물리로 돌아갈 때까지 경과 시간만 갱신한다.
    next.reaction = m_Ragdoll.GetImpactReaction();
    m_Ragdoll.AdvanceImpactReaction(next.reaction, impacts, deltaTime);
    PxVec3 leanImpulse(0);
    for (const RagdollBodyImpulse& contact : impacts)
    {
        if (contact.fromCharacterCapsule)
        {
            leanImpulse += (contact.position - pelvisPosition).cross(contact.impulse);
        }
        const PxVec3 approach(contact.capsuleVelocity.x, 0, contact.capsuleVelocity.z);
        if (contact.fromAnimatedCapsule && approach.magnitudeSquared() > next.approachVelocity.magnitudeSquared())
        {
            next.approachVelocity = approach;
            next.approachGroup = contact.capsuleGroup;
        }
    }
    next.ApplyCapsuleImpacts(impacts);
    PxVec3 bodyAngularImpulse(0);
    for (const RagdollImpactResponse& response : responses)
    {
        // 솔버의 지속 밀기 속도 제한을 먼저 적용한 뒤, 첫 접촉에서만 설정한 추가 운동량을 더한다.
        next.momentum += PxVec3(response.impulse.x, 0, response.impulse.z);
        leanImpulse += (response.partPosition - pelvisPosition).cross(response.partImpulse);
        bodyAngularImpulse += response.GetAngularImpulse(sample.centerOfMass);
    }
    const PxVec3 bodySpin = bodyAngularImpulse.isZero() ? PxVec3(0) :
        m_Ragdoll.GetImpactAngularVelocity(sample.centerOfMass, bodyAngularImpulse);
    if (!next.momentum.isFinite() || !leanImpulse.isFinite() || !bodySpin.isFinite())
    {
        return false;
    }
    if (torsoLengthSquared > 1e-6f)
    {
        // 골반 기준의 수평 회전축만 써서 기존 좌우 회전과 분리한다. 관성은 몸통 질량과 골반~가슴 길이로 근사한다.
        leanImpulse.y = 0;
        PxVec3 angularVelocity = leanImpulse / (next.torsoMass * torsoLengthSquared) + PxVec3(bodySpin.x, 0, bodySpin.z);
        const float angularSpeed = angularVelocity.normalize();
        next.peakTorsoLean = angularVelocity * PxMin(MaximumTorsoLean, angularSpeed * TorsoLeanPeakTime * std::exp(-1.0f));
    }
    next.actorAtStart = character->GetActorTransform();
    const Matrix4 componentTransform(GetComponentTransform());
    next.componentRotation = RigidFrame(componentTransform).q;
    const Vector3 actorPosition = next.actorAtStart.GetTranslation();
    next.turnPivotWorld = PxVec3(sample.centerOfMass.x, actorPosition.GetY(), sample.centerOfMass.z);
    const Vector3 turnPivot(next.turnPivotWorld.x, next.turnPivotWorld.y, next.turnPivotWorld.z);
    const Vector3 localPivot(Invert(componentTransform) * turnPivot);
    next.turnPivotInComponent = PxVec3(localPivot.GetX(), localPivot.GetY(), localPivot.GetZ());
    std::array<PxVec3, 2> soleOffsets;
    for (size_t side = 0; side < next.legs.size(); ++side)
    {
        const FootSoleSample& foot = sample.feet[side];
        const FootLegSetup& setup = foot.leg;
        TwoBoneIKInput& leg = next.legs[side];
        leg.hip = RigidFrame(next.sourceBones[setup.hipNode]);
        leg.knee = RigidFrame(next.sourceBones[setup.kneeNode]);
        leg.ankle = RigidFrame(next.sourceBones[setup.ankleNode]);
        leg.soleInBone = RigidFrame(foot.soleInBone);
        leg.targetSole = leg.ankle * leg.soleInBone;
        const Vector3 soleOffset = Vector3(foot.soleWorld.GetW()) - turnPivot;
        soleOffsets[side] = PxVec3(soleOffset.GetX(), soleOffset.GetY(), soleOffset.GetZ());
        leg.thighLength = (leg.knee.p - leg.hip.p).magnitude();
        leg.calfLength = (leg.ankle.p - leg.knee.p).magnitude();
        leg.referenceBend = leg.hip.q.rotate(setup.referenceBendInHip);
    }
    const PxVec3 velocity = next.momentum / next.mass;
    next.steps[0] = next.PlanStep(sample.hull, sample.projectedCenterOfMass, velocity, false);
    if (next.steps[0].duration <= 0)
    {
        return false;
    }
    const PxVec3 spacing = next.componentRotation.rotate(next.legs[0].targetSole.p - next.legs[1].targetSole.p);
    next.firstFoot = spacing.dot(next.steps[0].stride) >= 0 ? 0u : 1u;
    // 두 발을 디딜 시간만큼 첫 충격의 회전을 인계한다. 발이 밀려나는 방향은 보폭에만 사용한다.
    next.turnYaw = PlanCapsuleImpactTurn(capsuleYawSpeed, 2.0f * next.steps[0].duration);
    const PxQuat landingTurn(next.turnYaw, PxVec3(0, 1, 0));
    const PxQuat landingSoleTurn(next.GetLandingSoleYaw(0), PxVec3(0, 1, 0));
    // 첫 착지를 미리 배치한다. 루트의 예상 평행이동은 COM과 양발에 공통이므로 함께 생략한다.
    FootstepPolygon nextCorners;
    nextCorners.reserve(8);
    for (size_t side = 0; side < sample.feet.size(); ++side)
    {
        const FootSoleSample& foot = sample.feet[side];
        const PxVec3 offset = side == next.firstFoot ? next.steps[0].stride : -next.steps[0].stride;
        const PxVec3 center = next.turnPivotWorld + soleOffsets[side];
        const PxVec3 turnedCenter = next.turnPivotWorld + landingTurn.rotate(soleOffsets[side]);
        for (size_t corner = 0; corner < foot.projected.size(); ++corner)
        {
            if (foot.projected[corner])
            {
                PxVec3 point = foot.projections[corner];
                if (side == next.firstFoot)
                {
                    point = turnedCenter + landingSoleTurn.rotate(point - center);
                }
                nextCorners.push_back(point + offset);
            }
        }
    }
    next.steps[1] = next.PlanStep(BuildFootSupportHull(std::move(nextCorners)), sample.projectedCenterOfMass, velocity, true);
    const CapsuleComponent& capsule = character->GetCapsuleComponent();
    PhysicsQueryCapsule sweep;
    sweep.owner = character;
    sweep.radius = capsule.GetRadius();
    sweep.halfHeight = capsule.GetHalfHeight();
    sweep.centre = PxVec3(sample.centerOfMass.x, sample.projectedCenterOfMass.y + sweep.halfHeight, sample.centerOfMass.z);
    // 실제 이동 캡슐을 끈 대기 더미도 같은 환경 스윕으로 벽과 바닥을 따른다.
    PhysicsWorld& physics = *character->GetWorld()->GetPhysicsWorld();
    physics.MoveQueryCapsule(sweep, PxVec3(0), true, false);
    PxRaycastHit ground;
    if (physics.Raycast(sample.projectedCenterOfMass + PxVec3(0, .06f, 0), PxVec3(0, -.12f, 0), ground, character))
    {
        const PxBaseMaterial* base = ground.shape->getMaterialFromInternalFaceIndex(ground.faceIndex);
        const PxMaterial* material = base ? base->is<PxMaterial>() : nullptr;
        if (material)
        {
            const float gravity = -ground.actor->getScene()->getGravity().dot(ground.normal);
            next.slideDeceleration = material->getDynamicFriction() * PxMax(0.0f, gravity);
        }
    }
    next.sweepStart = sweep.centre;
    next.sweepCenter = sweep.centre;
    if (IsRagdoll())
    {
        ImpactFlailPlayback previous = std::move(m_ImpactFlail);
        const bool stopped = character->StopRagdoll(m_PhysicsError);
        if (previous.playingShove || !stopped)
        {
            m_ImpactFlail = std::move(previous);
        }
        if (!stopped)
        {
            return false;
        }
    }
    for (const RagdollImpactResponse& response : responses)
    {
        physics.ConsumeImpactContact(response.contactId);
    }
    if (idleClip != currentClip || m_AnimInstance->IsBlending())
    {
        m_AnimInstance->PlayAnimation(idleClip);
    }
    m_ImpactWarmup = std::move(next);
    m_FootSupport.ResetFootstep();
    UpdateKinematicPose(true);
    return true;
}

bool SkeletalMeshComponent::AdvanceImpactWarmup(float deltaTime)
{
    ImpactWarmup& warmup = m_ImpactWarmup;
    Character& character = static_cast<Character&>(GetOwner());
    const CapsuleComponent& capsule = character.GetCapsuleComponent();
    PhysicsQueryCapsule sweep;
    sweep.owner = &character;
    sweep.radius = capsule.GetRadius();
    sweep.halfHeight = capsule.GetHalfHeight();
    sweep.centre = warmup.sweepCenter;
    const PxVec3 displacement = warmup.momentum * (deltaTime / warmup.mass);
    PhysicsWorld& physics = *character.GetWorld()->GetPhysicsWorld();
    const PhysicsWorld::CapsuleMoveResult movement = physics.MoveQueryCapsule(sweep, displacement, true, false);
    if (!movement.grounded)
    {
        return FinishImpactWarmup(true);
    }
    if (movement.wall)
    {
        const PxVec3 actual = (sweep.centre - warmup.sweepCenter) * (warmup.mass / deltaTime);
        warmup.momentum = PxVec3(actual.x, 0, actual.z);
    }
    warmup.sweepCenter = sweep.centre;
    const PxVec3 offset = sweep.centre - warmup.sweepStart;
    const bool first = warmup.landings == 0;
    const FootSupportSnapshot& support = m_FootSupport.GetSnapshot();
    const ImpactWarmupStep& step = warmup.steps[warmup.landings];
    warmup.stepTime += deltaTime;
    const float t = warmup.stepTime + 1e-6f >= step.duration ? 1.0f : warmup.stepTime / step.duration;
    const float smooth = t * t * (3.0f - 2.0f * t);
    const float turnProgress = first ? FirstLandingTurnFraction * smooth :
        FirstLandingTurnFraction + (1.0f - FirstLandingTurnFraction) * smooth;
    const float rootYaw = warmup.turnYaw * turnProgress;
    const PxVec3 worldUp(0, 1, 0);
    const PxVec3 localUp = warmup.componentRotation.rotateInv(worldUp);
    const PxQuat turn(rootYaw, worldUp);
    const PxQuat componentRotation = turn * warmup.componentRotation;
    AffineTransform actor = warmup.actorAtStart;
    actor.SetBasis(Matrix3::MakeYRotation(rootYaw) * actor.GetBasis());
    const Vector3 origin = actor.GetTranslation();
    const PxVec3 pivot = warmup.turnPivotWorld + offset;
    const PxVec3 location = pivot + turn.rotate(PxVec3(origin.GetX(), origin.GetY(), origin.GetZ()) - warmup.turnPivotWorld);
    actor.SetTranslation(Vector3(location.x, location.y, location.z));
    const float elapsed = first ? warmup.stepTime : warmup.steps[0].duration + warmup.stepTime;
    const PxTransform brace = warmup.GetPelvisBrace(componentRotation, elapsed, m_StandingSupportSettings.maximumLowering);
    const PxVec3 firstStride = componentRotation.rotateInv(warmup.steps[0].stride);
    const PxVec3 stride = componentRotation.rotateInv(step.stride);
    // 첫 착지 전후의 골반 기준 발 속도를 연결한다. 보폭이나 착지 시간이 바뀌어도 발이 튀지 않는다.
    const PxVec3 startTangent = first ? PxVec3(0) : firstStride * (-2.0f * step.duration / warmup.steps[0].duration);
    const uint32_t movingFoot = first ? warmup.firstFoot : 1u - warmup.firstFoot;
    // 발을 들어 올리는 동안 방향을 맞추고, 착지 후에는 같은 월드 방향을 지지발에 유지한다.
    const float liftProgress = PxMin(2.0f * t, 1.0f);
    const float swingYaw = warmup.GetLandingSoleYaw(warmup.landings) * liftProgress * liftProgress * (3.0f - 2.0f * liftProgress);
    const float supportYaw = first ? 0.0f : warmup.GetLandingSoleYaw(0);
    std::vector<int> nodes;
    std::vector<Matrix4> targets;
    nodes.reserve(7);
    targets.reserve(7);
    nodes.push_back(warmup.pelvisNode);
    targets.push_back(FootstepPoseMatrix(brace) * warmup.sourceBones[warmup.pelvisNode]);
    for (size_t side = 0; side < warmup.legs.size(); ++side)
    {
        const FootLegSetup& setup = support.feet[side].leg;
        TwoBoneIKInput leg = warmup.legs[side];
        const bool swing = side == movingFoot;
        const bool finalSwing = !first && swing;
        const bool planting = finalSwing && t >= 1.0f;
        // 마지막 지지발의 착지 속도는 지면을 기준으로 한다. 루트가 미끄러지는 만큼 상대 속도를 상쇄한다.
        const PxVec3 endTangent = finalSwing ? -componentRotation.rotateInv(warmup.momentum / warmup.mass) * step.duration :
            -2.0f * stride;
        const PxVec3 tangent = t * (1.0f - t) * ((1.0f - t) * startTangent - t * endTangent);
        if (warmup.turnYaw != 0)
        {
            // 이동발만 새 발자리로 돌린다. 지지발은 슬라이드와 기존 보폭만 따르고 골반을 따라 휘돌지 않는다.
            const float soleYaw = swing ? warmup.turnYaw * smooth : (first ? 0 : warmup.turnYaw);
            const PxQuat relativeTurn(soleYaw - rootYaw, localUp);
            leg.targetSole.p = warmup.turnPivotInComponent + relativeTurn.rotate(leg.targetSole.p - warmup.turnPivotInComponent);
        }
        const float footYaw = swing ? swingYaw : supportYaw;
        leg.targetSole.q = PxQuat(footYaw - rootYaw, localUp) * leg.targetSole.q;
        const PxVec3 start = first ? PxVec3(0) : (swing ? -firstStride : firstStride);
        const PxVec3 end = swing ? stride : -stride;
        leg.targetSole.p += start * (1.0f - smooth) + end * smooth + tangent;
        if (swing)
        {
            const float lift = 4.0f * t * (1.0f - t);
            leg.targetSole.p += localUp * (PxMin(.085f, leg.calfLength * .2f) * lift * lift);
        }
        if (planting)
        {
            // 마지막 구간은 발밑으로 내려딛는다. 다음 물리 틱은 지면에 닿은 자세와 수평으로 멎은 발을 물려받는다.
            const PxTransform previous = RigidFrame(support.feet[side].soleWorld);
            const PxTransform component(pivot - componentRotation.rotate(warmup.turnPivotInComponent), componentRotation);
            const float probe = leg.calfLength * .5f;
            PxRaycastHit ground;
            PxTransform planted(PxIdentity);
            if (!physics.Raycast(previous.p + worldUp * probe, -worldUp * (2.0f * probe), ground, &character) ||
                ground.normal.y < std::cos(PxPi * .25f) ||
                !MakeFootSoleTarget(ground.position, ground.normal, component * leg.targetSole, planted))
            {
                return FinishImpactWarmup(false);
            }
            leg.targetSole = component.getInverse() * planted;
        }
        leg.hip = brace * leg.hip;
        leg.knee = brace * leg.knee;
        leg.ankle = brace * leg.ankle;
        leg.referenceBend = brace.q.rotate(leg.referenceBend);
        const LegIKResult solved = SolveTwoBoneIK(leg);
        if (!solved.valid || (planting && solved.reachError > .0001f))
        {
            return FinishImpactWarmup(false);
        }
        const std::array<int, 3> chain{setup.hipNode, setup.kneeNode, setup.ankleNode};
        const std::array<PxTransform, 3> frames{solved.hip, solved.knee, solved.ankle};
        for (size_t joint = 0; joint < chain.size(); ++joint)
        {
            const int node = chain[joint];
            const Matrix4& original = warmup.sourceBones[node];
            nodes.push_back(node);
            targets.push_back(FootstepPoseMatrix(frames[joint]) * Invert(PhysicsBoneFrame(original)) * original);
        }
    }
    StartShoveReaction(warmup.turnYaw);
    UpdateImpactFlail(true, deltaTime);
    const std::vector<GraphNode>& basePose = m_ImpactFlail.pose.empty() ? warmup.sourcePose : m_ImpactFlail.pose;
    if (!OverridePhysicsPose(m_PhysicsSkeleton, basePose, Matrix4(kIdentity), nodes, targets, warmup.pose) ||
        !warmup.ApplyTorsoLean(m_PhysicsSkeleton, componentRotation, elapsed, m_PhysicsError))
    {
        return FinishImpactWarmup(false);
    }
    // 착지 자세가 성립한 뒤에만 이번 틱의 루트와 다리를 함께 옮긴다.
    character.SetActorTransform(actor);
    if (!UpdateKinematicPose(false))
    {
        return FinishImpactWarmup(false);
    }
    if (t >= 1.0f)
    {
        ++warmup.landings;
        warmup.stepTime = 0;
    }
    return true;
}

bool SkeletalMeshComponent::FinishImpactWarmup(bool lostFloor, bool replayCapsuleImpacts)
{
    const PxVec3 targetVelocity = m_ImpactWarmup.momentum / m_ImpactWarmup.mass;
    const uint32_t landings = m_ImpactWarmup.landings;
    const uint32_t supportFoot = landings == 1 ? m_ImpactWarmup.firstFoot : 1u - m_ImpactWarmup.firstFoot;
    // 두 번째 스윙 중 다시 부딪혀도 이미 디딘 첫발에서 인계한다. 실제 접지는 물리 스텝에서 다시 확인한다.
    const bool hasSupportFoot = landings == 2 || (landings == 1 && replayCapsuleImpacts);
    // 예열 루트에 아직 반영되지 않은 캡슐 접촉만 첫 물리 스텝으로 넘긴다.
    std::vector<RagdollBodyImpulse> pendingImpacts;
    for (const RagdollBodyImpulse& impact : m_Ragdoll.GetExternalImpacts())
    {
        if (!impact.fromCharacterCapsule || replayCapsuleImpacts)
        {
            pendingImpacts.push_back(impact);
        }
    }
    Character& character = static_cast<Character&>(GetOwner());
    if (!character.SetPhysicsMode(PhysicsAnimationMode::ActiveRagdoll, m_PhysicsError))
    {
        m_ImpactWarmup = {};
        return true;
    }
    // 각 다리의 상대 속도는 유지하면서 계승된 전신 수평 운동량의 차이만 보충한다.
    Vector3 inherited(kZero);
    if (m_Ragdoll.GetCenterOfMassVelocity(inherited))
    {
        m_Ragdoll.AddVelocity(Vector3(targetVelocity.x - inherited.GetX(), 0, targetVelocity.z - inherited.GetZ()));
    }
    m_RootSupportRelease = lostFloor ? RootSupportRelease::NoSupport : RootSupportRelease::None;
    if (!m_Ragdoll.SetRootSupported(!lostFloor, m_PhysicsError))
    {
        m_RootSupportRelease = RootSupportRelease::Error;
    }
    if (hasSupportFoot && m_RootSupportRelease == RootSupportRelease::None)
    {
        m_FootSupport.ResumeAfterWarmup(supportFoot);
    }
    else
    {
        m_FootSupport.ResetFootstep();
    }
    ResetRecovery();
    m_KinematicRecoveryTimer.Reset();
    m_PendingRecoveryImpact = m_RootSupportRelease == RootSupportRelease::None;
    if (!pendingImpacts.empty())
    {
        ReplayKinematicImpacts(pendingImpacts, static_cast<float>(PhysicsWorld::FixedDeltaTime));
    }
    UpdateFootSupport();
    return false;
}
