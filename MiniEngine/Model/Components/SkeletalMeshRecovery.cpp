#include "SkeletalMeshComponent.h"
#include <cmath>
#include <limits>
#include "../GameFramework/Character.h"
#include "PhysicsWorld.h"

using namespace Math;
using namespace physx;

namespace PARS
{
    bool SkeletalMeshComponent::SetFallingAnimation(uint32_t animationIndex)
    {
        if (animationIndex >= m_AnimInstance->GetAnimationCount())
        {
            return false;
        }
        m_FallingAnimationIndex = animationIndex;
        return true;
    }

    bool SkeletalMeshComponent::IsFollowingFallingAnimation() const
    {
        return m_FallingAnimationIndex < m_AnimInstance->GetAnimationCount() &&
            m_AnimInstance->GetCurrentAnimationIndex() == m_FallingAnimationIndex;
    }

    bool SkeletalMeshComponent::SetStandingUpAnimation(uint32_t animationIndex, uint32_t idleAnimationIndex)
    {
        if (animationIndex >= m_AnimInstance->GetAnimationCount() ||
            idleAnimationIndex >= m_AnimInstance->GetAnimationCount() || animationIndex == idleAnimationIndex ||
            animationIndex == m_FallingAnimationIndex || idleAnimationIndex == m_FallingAnimationIndex ||
            m_AnimInstance->GetAnimationName(idleAnimationIndex) != "Idle")
        {
            return false;
        }
        m_StandingUpAnimationIndex = animationIndex;
        m_StandingIdleAnimationIndex = idleAnimationIndex;
        m_StandingUpStableTime = 0;
        m_StandingUpState = StandingUpState::None;
        return true;
    }

    bool SkeletalMeshComponent::IsFollowingStandingUpAnimation() const
    {
        return m_StandingUpAnimationIndex < m_AnimInstance->GetAnimationCount() &&
            m_AnimInstance->GetCurrentAnimationIndex() == m_StandingUpAnimationIndex;
    }

    bool SkeletalMeshComponent::UpdateFallingAnimation(float deltaTime)
    {
        // 기상과 Idle 보간은 물리 시뮬레이션을 꺼 둔 상태에서도 계속 진행한다.
        if (m_StandingUpState == StandingUpState::Kinematic || m_StandingUpState == StandingUpState::IdleBlend)
        {
            return UpdateStandingUpAnimation(deltaTime);
        }
        if (!IsActiveRagdoll() || !m_AutomaticRootSupport)
        {
            m_StandingUpStableTime = 0;
            m_StandingUpState = StandingUpState::None;
            return true;
        }
        if (IsFollowingFallingAnimation())
        {
            return UpdateStandingUpAnimation(deltaTime);
        }
        if (m_RootSupportRelease == RootSupportRelease::None || m_FallingAnimationIndex >= m_AnimInstance->GetAnimationCount())
        {
            return true;
        }

        // 낙상은 한 번 재생하고 착지 후 기상을 기다린다. 위치와 회전 지지는 이미 해제된 상태다.
        if (!m_AnimInstance->TransitionToAnimation(m_FallingAnimationIndex, .2f, false))
        {
            m_PhysicsError = "Cannot transition to the falling animation.";
            return false;
        }
        m_KinematicRecoveryTimer.Reset();
        m_StandingUpStableTime = 0;
        m_StandingUpState = StandingUpState::None;
        m_Ragdoll.ResetDriveTargetHistory();
        ResetRecovery();
        return true;
    }

    bool SkeletalMeshComponent::UpdateStandingUpAnimation(float deltaTime)
    {
        if (m_StandingUpAnimationIndex >= m_AnimInstance->GetAnimationCount() ||
            m_StandingIdleAnimationIndex >= m_AnimInstance->GetAnimationCount())
        {
            return true;
        }
        if (m_StandingUpState == StandingUpState::IdleBlend)
        {
            if (m_AnimInstance->IsBlending() || IsBlendingToKinematic())
            {
                return true;
            }
            // 최초 스폰처럼 키네마틱 Idle을 유지하고, 다음 충격에서만 물리 시뮬레이션을 켠다.
            m_StandingUpState = StandingUpState::None;
            m_StandingUpStableTime = 0;
            m_RootSupportRelease = RootSupportRelease::None;
            m_PendingRecoveryImpact = false;
            m_FootSupport.ResetFootstep();
            ResetRecovery();
            m_Ragdoll.ResetDriveTargetHistory();
            m_KinematicRecoveryTimer.Reset();
            return true;
        }

        if (m_AnimInstance->IsPlaying() || m_AnimInstance->IsBlending() || IsBlendingToKinematic())
        {
            m_StandingUpStableTime = 0;
            return true;
        }
        if (m_StandingUpState == StandingUpState::Kinematic)
        {
            // 키네마틱 기상은 양발의 물리 접촉을 기다리지 않고 클립이 끝나면 Idle로 이어진다.
            if (!m_AnimInstance->TransitionToAnimation(m_StandingIdleAnimationIndex, .2f, true))
            {
                m_PhysicsError = "Cannot transition from the standing-up animation to idle.";
                return false;
            }
            m_StandingUpState = StandingUpState::IdleBlend;
            return true;
        }

        Vector3 velocity(kZero);
        const bool settled = m_Ragdoll.GetCenterOfMassVelocity(velocity) &&
            float(Length(velocity)) <= m_RecoverySettings.maximumReturnSpeed;
        bool grounded = false;
        float groundHeight = -std::numeric_limits<float>::infinity();
        for (size_t body = 0; body < m_Ragdoll.GetBodyCount(); ++body)
        {
            PhysicsGroundContact contact;
            if (m_Ragdoll.GetBodyGroundContact(body, contact))
            {
                grounded = true;
                groundHeight = Max(groundHeight, contact.position.y);
            }
        }
        const bool ready = settled && grounded;
        m_StandingUpStableTime = ready ? m_StandingUpStableTime + deltaTime : 0;
        if (m_StandingUpStableTime < m_RecoverySettings.stableDuration)
        {
            return true;
        }

        return BeginKinematicStandingUp(groundHeight);
    }

    bool SkeletalMeshComponent::BeginKinematicStandingUp(float groundHeight)
    {
        const int rootBody = m_Ragdoll.GetImpactRootBody();
        const std::vector<int>& nodes = m_Ragdoll.GetBoneNodes();
        if (!m_PhysicsAsset || !std::isfinite(groundHeight) || rootBody < 0 ||
            static_cast<size_t>(rootBody) >= nodes.size())
        {
            m_PhysicsError = "Kinematic standing-up requires a pelvis body and ground contact.";
            return false;
        }
        std::vector<GraphNode> standingPose;
        std::vector<GraphNode> idlePose;
        std::vector<Matrix4> standingWorld;
        std::vector<Matrix4> idleWorld;
        const Matrix4 component(GetComponentTransform());
        if (!m_AnimInstance->SampleAnimationPose(m_StandingUpAnimationIndex, 0, standingPose) ||
            !m_AnimInstance->SampleAnimationPose(m_StandingIdleAnimationIndex, 0, idlePose) ||
            !EvaluatePhysicsPose(m_PhysicsSkeleton, standingPose, component, standingWorld, m_PhysicsError) ||
            !EvaluatePhysicsPose(m_PhysicsSkeleton, idlePose, component, idleWorld, m_PhysicsError))
        {
            m_PhysicsError = "Cannot sample the standing-up and idle alignment poses.";
            return false;
        }
        const int rootNode = nodes[rootBody];
        if (rootNode < 0 || static_cast<size_t>(rootNode) >= standingWorld.size() ||
            nodes.size() != m_PhysicsAsset->bodies.size())
        {
            m_PhysicsError = "Invalid pelvis or body mapping for kinematic standing-up.";
            return false;
        }

        // 기상 뒤 Idle의 PA 최하단이 실제 착지한 바닥에 놓이도록 기준 높이를 잡는다.
        float idleLowest = std::numeric_limits<float>::infinity();
        for (size_t body = 0; body < nodes.size(); ++body)
        {
            const int node = nodes[body];
            if (node < 0 || static_cast<size_t>(node) >= idleWorld.size())
            {
                m_PhysicsError = "Invalid idle body mapping for kinematic standing-up.";
                return false;
            }
            const PARS::PhysicsBody& shape = m_PhysicsAsset->bodies[body];
            const Matrix4 frame = PhysicsBoneFrame(idleWorld[node]) * shape.GetLocalTransform();
            float extent = shape.radius;
            if (shape.shape == PhysicsShapeType::Capsule)
            {
                extent += shape.halfLength * std::abs(float(frame.GetX().GetY()));
            }
            else if (shape.shape == PhysicsShapeType::Box)
            {
                extent = shape.halfExtents.x * std::abs(float(frame.GetX().GetY())) +
                    shape.halfExtents.y * std::abs(float(frame.GetY().GetY())) +
                    shape.halfExtents.z * std::abs(float(frame.GetZ().GetY()));
            }
            idleLowest = Min(idleLowest, float(frame.GetW().GetY()) - extent);
        }

        // 누운 골반의 몸통 방향을 수평으로 맞춘다. 수직에 가까우면 다른 축을 사용한다.
        const Matrix4 actualPelvis = m_Ragdoll.GetBoneWorld(static_cast<size_t>(rootBody));
        const Matrix4 animatedPelvis = PhysicsBoneFrame(standingWorld[rootNode]);
        const Vector3 actualAxes[] = {Vector3(actualPelvis.GetY()), Vector3(actualPelvis.GetZ()), Vector3(actualPelvis.GetX())};
        const Vector3 animatedAxes[] = {Vector3(animatedPelvis.GetY()), Vector3(animatedPelvis.GetZ()), Vector3(animatedPelvis.GetX())};
        float yaw = 0;
        for (size_t axis = 0; axis < 3; ++axis)
        {
            Vector3 actual = actualAxes[axis];
            Vector3 animated = animatedAxes[axis];
            actual.SetY(0);
            animated.SetY(0);
            if (float(LengthSquare(actual)) > .01f && float(LengthSquare(animated)) > .01f)
            {
                yaw = std::atan2(float(Cross(animated, actual).GetY()), float(Dot(animated, actual)));
                break;
            }
        }
        AffineTransform alignment = AffineTransform::MakeYRotation(yaw);
        const Vector3 rotatedPelvis = alignment * Vector3(animatedPelvis.GetW());
        Vector3 offset = Vector3(actualPelvis.GetW()) - rotatedPelvis;
        offset.SetY(groundHeight - idleLowest);
        if (!std::isfinite(float(Length(offset))))
        {
            m_PhysicsError = "Invalid kinematic standing-up alignment.";
            return false;
        }
        alignment.SetTranslation(offset);
        const AffineTransform previousTransform = GetOwner().GetActorTransform();
        if (!GetOwner().SetActorTransform(alignment * previousTransform))
        {
            m_PhysicsError = "Cannot align the character for kinematic standing-up.";
            return false;
        }
        // 정렬된 액터 좌표계에서 실제 물리 자세를 다시 표현해 전환 순간의 월드 자세를 보존한다.
        if (!CaptureRecoveryBlendSource())
        {
            GetOwner().SetActorTransform(previousTransform);
            m_PhysicsError = "Cannot capture the fallen pose for kinematic standing-up.";
            return false;
        }
        m_KinematicBlendSource = m_RecoveryBlendSource;
        m_KinematicBlendPose = m_KinematicBlendSource;
        m_KinematicBlendTime = 0;
        Character* character = dynamic_cast<Character*>(&GetOwner());
        if (character)
        {
            if (!character->StopRagdoll(m_PhysicsError))
            {
                GetOwner().SetActorTransform(previousTransform);
                m_KinematicBlendSource.clear();
                m_KinematicBlendPose.clear();
                return false;
            }
        }
        else
        {
            StopRagdoll();
        }
        if (IsRagdoll() || !m_AnimInstance->PlayAnimation(m_StandingUpAnimationIndex, false))
        {
            m_PhysicsError = "Cannot start kinematic standing-up.";
            return false;
        }
        m_StandingUpState = StandingUpState::Kinematic;
        m_StandingUpStableTime = 0;
        m_KinematicRecoveryTimer.Reset();
        m_Ragdoll.ResetDriveTargetHistory();
        ResetRecovery();
        return true;
    }

    void SkeletalMeshComponent::SetAutomaticRagdoll(bool enabled)
    {
        m_AutomaticRagdoll = enabled;
        m_KinematicRecoveryTimer.Reset();
        m_KinematicBlendSource.clear();
        m_KinematicBlendPose.clear();
        m_KinematicBlendTime = 0;
        m_PendingRecoveryImpact = false;
        m_StandingUpStableTime = 0;
        m_StandingUpState = StandingUpState::None;
    }

    void SkeletalMeshComponent::SetKinematicStableDuration(float duration)
    {
        if (std::isfinite(duration) && duration > 0)
        {
            m_KinematicStableDuration = duration;
            m_KinematicRecoveryTimer.Reset();
        }
    }

    bool SkeletalMeshComponent::UpdateAutomaticRagdoll(float deltaTime)
    {
        if (!m_AutomaticRagdoll)
        {
            return true;
        }
        const FootSupportSnapshot& sample = m_FootSupport.GetSnapshot();
        const float impactVelocity = sample.totalMass > 0 ? m_Ragdoll.GetExternalImpactImpulse() / sample.totalMass : 0;
        const bool impact = impactVelocity >= m_RecoverySettings.impactVelocity;
        if (!IsRagdoll())
        {
            const bool approaching = m_Ragdoll.HasApproachingImpact(m_RecoverySettings.impactVelocity);
            if (!impact && !approaching)
            {
                return true;
            }
            const bool interruptedStandingUp = m_StandingUpState == StandingUpState::Kinematic ||
                m_StandingUpState == StandingUpState::IdleBlend;
            // 모드 전환은 접촉 캐시를 비우므로 충격점과 바디별 충격량을 먼저 복사한다.
            std::vector<RagdollBodyImpulse> impacts;
            m_Ragdoll.CaptureExternalImpacts(impacts);
            Character* character = dynamic_cast<Character*>(&GetOwner());
            const bool activated = character ? character->SetPhysicsMode(PhysicsAnimationMode::ActiveRagdoll, m_PhysicsError) :
                SetPhysicsMode(PhysicsAnimationMode::ActiveRagdoll, Vector3(kZero), m_PhysicsError);
            if (!activated)
            {
                return false;
            }
            if (interruptedStandingUp)
            {
                // 현재 보간 자세와 속도를 물리 바디에 넘긴 뒤 기상을 취소한다.
                m_StandingUpState = StandingUpState::None;
                m_StandingUpStableTime = 0;
            }
            // 일어나는 도중에는 골반을 서기 목표에 붙잡지 않고 낙상 경로로 돌아간다.
            m_RootSupportRelease = interruptedStandingUp ? RootSupportRelease::NoSupport : RootSupportRelease::None;
            m_FootSupport.ResetFootstep();
            ResetRecovery();
            m_KinematicRecoveryTimer.Reset();
            m_PendingRecoveryImpact = !interruptedStandingUp;
            m_Ragdoll.ApplyExternalImpacts(impacts);
            if (!interruptedStandingUp)
            {
                // 모드 전환으로 접촉 캐시가 지워지기 전에 복사한 충격 방향을 상체에 전달한다.
                CaptureRecoveryFlailDirection(impacts);
            }
            return m_Ragdoll.SetRootSupported(!interruptedStandingUp, m_PhysicsError);
        }
        // 낙상은 전용 기상 경로에서 처리하므로 기존 서기 복귀 타이머에서 제외한다.
        if (!IsActiveRagdoll() || IsFollowingFallingAnimation())
        {
            m_KinematicRecoveryTimer.Reset();
            return true;
        }
        const bool insideA = sample.hasComProjection && sample.balance.zone == FootSupportZone::Inside;
        const bool bothGrounded = sample.feet[0].available && sample.feet[0].contact.hasSample &&
            sample.feet[0].contact.grounded && sample.feet[1].available &&
            sample.feet[1].contact.hasSample && sample.feet[1].contact.grounded;
        // A영역 유지 시간은 그대로 누적하되, 실제 양발 접지가 확인될 때만 전환한다.
        if (m_KinematicRecoveryTimer.Update(sample.contactRevision, sample.completedPhysicsStep,
            deltaTime, insideA, bothGrounded, impact, m_KinematicStableDuration))
        {
            return BeginKinematicRecovery();
        }
        return true;
    }

    bool SkeletalMeshComponent::BeginKinematicRecovery()
    {
        std::vector<Matrix4> targets;
        if (!EvaluateAnimationPhysicsPose(m_PhysicsSkeleton, targets, m_PhysicsError))
        {
            return false;
        }
        Vector3 actualMidpoint(kZero);
        Vector3 animationMidpoint(kZero);
        for (const FootSoleSample& foot : m_FootSupport.GetSnapshot().feet)
        {
            if (!foot.available || foot.boneNode < 0 || static_cast<size_t>(foot.boneNode) >= targets.size())
            {
                m_PhysicsError = "Kinematic recovery requires both foot sole frames.";
                return false;
            }
            // PA와 같은 강체 본 좌표계를 사용해 발목이 아닌 발바닥 중심을 비교한다.
            const Matrix4 animatedSole = PhysicsBoneFrame(targets[foot.boneNode]) * foot.soleInBone;
            actualMidpoint += Vector3(foot.soleWorld.GetW()) * .5f;
            animationMidpoint += Vector3(animatedSole.GetW()) * .5f;
        }
        const Vector3 offset = actualMidpoint - animationMidpoint;
        if (!std::isfinite(float(Length(offset))))
        {
            m_PhysicsError = "Invalid foot sole position in kinematic recovery.";
            return false;
        }
        // 현재 양발 중점에 애니메이션 양발 중점을 맞추며 높이 차이도 루트에 반영한다.
        const Vector3 previousLocation = GetOwner().GetActorLocation();
        GetOwner().SetActorLocation(previousLocation + offset);
        if (!CaptureRecoveryBlendSource())
        {
            GetOwner().SetActorLocation(previousLocation);
            m_PhysicsError = "Cannot capture the physical pose for kinematic recovery.";
            return false;
        }
        m_KinematicBlendSource = m_RecoveryBlendSource;
        m_KinematicBlendPose = m_KinematicBlendSource;
        m_KinematicBlendTime = 0;
        // 첫 키네마틱 포즈도 캡처한 실제 자세를 사용해 모드 전환 순간의 튐을 막는다.
        Character* character = dynamic_cast<Character*>(&GetOwner());
        if (character)
        {
            if (!character->StopRagdoll(m_PhysicsError))
            {
                GetOwner().SetActorLocation(previousLocation);
                m_KinematicBlendSource.clear();
                m_KinematicBlendPose.clear();
                return false;
            }
        }
        else
        {
            StopRagdoll();
        }
        return !IsRagdoll();
    }

    bool SkeletalMeshComponent::UpdateKinematicRecoveryPose(float deltaTime)
    {
        if (m_KinematicBlendPose.empty())
        {
            return true;
        }
        m_KinematicBlendTime += deltaTime;
        if (m_KinematicBlendTime >= m_RecoverySettings.blendDuration)
        {
            m_KinematicBlendSource.clear();
            m_KinematicBlendPose.clear();
            return true;
        }
        const float t = m_KinematicBlendTime / m_RecoverySettings.blendDuration;
        m_KinematicBlendPose = m_AnimInstance->GetPose();
        if (!BlendRecoveryPose(m_KinematicBlendSource, m_KinematicBlendPose, t * t * (3 - 2 * t)))
        {
            m_PhysicsError = "Invalid kinematic recovery blend pose.";
            return false;
        }
        return true;
    }

    namespace
    {
        PxTransform ToRecoveryFrame(const Matrix4& matrix)
        {
            const PhysicsJointFrame frame = PhysicsJointFrame::FromTransform(PhysicsBoneFrame(matrix));
            return PxTransform(PxVec3(frame.offset.x, frame.offset.y, frame.offset.z),
                PxQuat(frame.rotation.x, frame.rotation.y, frame.rotation.z, frame.rotation.w));
        }

        PxTransform CaptureSupportSole(const FootSoleSample& foot)
        {
            return ToRecoveryFrame(foot.supportSoleWorld);
        }
    }

    void SkeletalMeshComponent::ResetRecovery()
    {
        m_Recovery.Reset();
        m_RecoveryFlail.Reset();
        m_RecoveryFlailPose.clear();
        m_RecoveryFlailFrontPose.clear();
        m_RecoveryFlailBackPose.clear();
        m_HasRecoverySoles = false;
        m_RecoveryReferenceHull.clear();
        m_RecoveryBlendSource.clear();
        m_FootstepIK.Reset();
        m_StandingSupport.Reset();
        m_Ragdoll.SetSupportContacts({});
    }

    bool SkeletalMeshComponent::CaptureRecoveryBlendSource()
    {
        std::vector<Matrix4> bodies;
        const std::vector<int>& nodes = m_Ragdoll.GetBoneNodes();
        if (m_BoneStretch.size() != nodes.size())
        {
            return false;
        }
        for (size_t body = 0; body < nodes.size(); ++body)
        {
            bodies.push_back(m_Ragdoll.GetBoneWorld(body) * m_BoneStretch[body]);
        }
        // 직전 렌더 포즈가 아니라 마지막 물리 스텝의 실제 바디를 로컬 본 자세로 변환한다.
        return OverridePhysicsPose(m_PhysicsSkeleton, m_AnimInstance->GetPose(), Matrix4(GetComponentTransform()),
            nodes, bodies, m_RecoveryBlendSource);
    }

    bool SkeletalMeshComponent::ApplyRecoveryTargets(std::vector<Matrix4>& target, float deltaTime)
    {
        // UpdateFootSupport() 에서 계산한 발 지지 정보와 발딛기 결과를 가져옴
        const FootSupportSnapshot& sample = m_FootSupport.GetSnapshot();
        const FootstepSnapshot& step = sample.footstep;

        // 회복 제어는 재생 클립과 독립적으로 실제 COM과 접촉 상태에서 시작한다.
        RecoveryInput input;
        input.enabled = IsActiveRagdoll() && m_AutomaticRootSupport;

        if (!input.enabled)
        {
            // 회복상태, 저장한 발 목표, 복귀 보간 포즈, IK상태 등을 초기화
            ResetRecovery();
            return true;
        }

        // 지지 해제 여부와 발딛기 상태를 전달 
        // fallen : 골반 지지가 해제됐거나 발딛기가 중단
        // needsStep : 플래너가 예측 COM을 보고 발딛기가 필요하다고 판단
        // stepping : 발딛기가 이미 시작돼, 이륙 대기 또는 공중 단계인지
        input.fallen = m_RootSupportRelease != RootSupportRelease::None || step.phase == FootstepPhase::Aborted;
        input.needsStep = step.needsStep;
        input.stepping = step.IsInProgress();

        // 시간과 입력의 버전 정보를 전달. 예를 들어 같은 물리 결과로 회복 판단을 두번 호출해도, 안정화 시간이 두 배로 누적되지 않도록 스텝 번호를 사용
        // deltatime : 안정 상태를 유지한 시간과 복귀 보간 시간 계산
        // revision : 접촉 이력이 초기화됐는지 감지하고 이전 회복 상태 초기화
        // completedStep : 같은 완료 물리 스텝을 중복 처리하지 않도록 확인
        input.deltaTime = deltaTime;
        input.revision = sample.contactRevision;
        input.completedStep = sample.completedPhysicsStep;

        // 현재 COM이 지지 영역 안에 있는지 전달
        input.insideSupport = sample.hasCenterOfMass && sample.comInsideHull;

        // 예측 COM도 지지 영역 안에 있는지 전달
        // 플래너가 이미 계산한 결과를 이용
        input.predictionInsideSupport = step.hasPrediction && !step.needsStep;

        // 양발 모두 실제로 접지했는지 전달. 접촉샘플 데이터와 지면접촉정보로 판단
        input.bothGrounded = 
            sample.feet[0].contact.hasSample && 
            sample.feet[0].contact.grounded &&
            sample.feet[1].contact.hasSample && 
            sample.feet[1].contact.grounded;

        // 외부 충격을 속도 변화 단위로 환산. GetExternalImpactImpulse()은 물리 엔진에 기록된 전신의 외부 충돌 충격량을 가져옴. 이를 전신 질량으로 나눔
        if (sample.totalMass > 0)
        {
            // 충격량을 전신 질량으로 나눠 모델 크기에 덜 의존하는 속도 변화 단위로 판정한다.
            // 상대 캡슐이 PA에 가한 충격은 포함하고, 본인 독립 캡슐에 생긴 반작용은 제외한다.
            input.impactVelocity = m_Ragdoll.GetExternalImpactImpulse() / sample.totalMass;
        }
        input.impactTriggered = m_PendingRecoveryImpact;
        m_PendingRecoveryImpact = false;

        // 현재 COM의 수평 속력 크기를 계산
        if (sample.hasCenterOfMassVelocity)
        {
            const PxVec3& v = sample.centerOfMassVelocity;
            input.horizontalSpeed = std::sqrt(v.x * v.x + v.z * v.z);
        }

        // 복귀 판단도 실제 발 지지 영역을 사용한다. Idle 자세의 발 배치를 기준으로 만들지 않는다.
        m_RecoveryReferenceHull = sample.hull;
        input.hasReference = m_RecoveryReferenceHull.size() >= 3;
        input.insideReference = input.insideSupport;
        input.predictionInsideReference = input.predictionInsideSupport;

        // m_RecoverySoles : 회복,지지에 사용할 양발 발바닥의 목표 위치,회전
		// m_RecoveryBlendSource : 애니메이션 복귀를 시작할 때 캡처한 물리 본 자세

        // 앞서 정의한 RecoveryInput input;으로 회복 상태를 갱신. 충격, COM, 발딛기, 접지 상태 등을 보고 회복 제어가 현재 상태를 결정
        m_Recovery.Update(input, m_RecoverySettings);
        const RecoverySnapshot& recovery = m_Recovery.GetSnapshot();

        // 지지가 해제되거나 발딛기가 중단됐다면 발 보정 정보를 정리
        if (input.fallen)
        {
            m_RecoveryFlail.Reset();
            m_HasRecoverySoles = false;
            m_RecoveryBlendSource.clear();
            m_FootstepIK.Reset();
            m_StandingSupport.Reset();
            m_Ragdoll.SetSupportContacts({});
            return true;
        }

        // 새로 회복에 진입했다면, 이전 목표의 속도 계산 이력을 끊음
        // 목표 생성방식이 바뀌면, 이전 목표와  목표 사이에 차이가 생길 수 있음
        if (recovery.enteredRecovery)
        {
            m_Ragdoll.ResetDriveTargetHistory();
            m_RecoveryBlendSource.clear();
        }

        // 상체 반응은 발딛기 계획과 분리하고, 회복 조건에는 실제 COM과 접촉만 사용한다.
        // 회복 종료 시에는 아래의 실제 물리 자세 캡처와 기존 복귀 보간이 상체도 함께 처리한다.
        if (!ApplyRecoveryFlailTargets(target, deltaTime, input.impactVelocity))
        {
            return false;
        }

        
        if ((m_Ragdoll.UsesFootDrivenSupport() && !m_HasRecoverySoles) || (recovery.enteredRecovery && !m_Ragdoll.UsesFootDrivenSupport()))
        {
            m_HasRecoverySoles = sample.feet[0].available && sample.feet[1].available;


            for (size_t side = 0; side < sample.feet.size(); ++side)
            {
                m_RecoverySoles[side] = m_Ragdoll.UsesFootDrivenSupport() ? CaptureSupportSole(sample.feet[side]) : ToRecoveryFrame(sample.feet[side].soleWorld);
                m_HasRecoverySoles = m_HasRecoverySoles && m_RecoverySoles[side].isSane();
            }
        }

        // 애니메이션 복귀를 시작하면 현재 물리 자세를 캡처. 복귀 보간 포즈의 출발점으로 사용
        if (recovery.state == RecoveryState::Animation && recovery.startedBlend)
        {
            if (!CaptureRecoveryBlendSource())
            {
                m_PhysicsError = "Cannot capture the current physical pose for animation recovery.";
                return false;
            }
            m_Ragdoll.ResetDriveTargetHistory();
        }

        // 복귀 중에는 캡처한 물리 자세에서 현재 애니메이션 자세로 보간
        if (recovery.state == RecoveryState::Animation && recovery.blending)
        {
            std::vector<GraphNode> pose = m_AnimInstance->GetPose();
            if (m_RecoveryBlendSource.size() != pose.size() || !BlendRecoveryPose(m_RecoveryBlendSource, pose, recovery.blendAlpha))
            {
                m_PhysicsError = "Invalid physical pose in animation recovery.";
                return false;
            }
            if (!EvaluatePhysicsPose(m_PhysicsSkeleton, pose, Matrix4(GetComponentTransform()), target, m_PhysicsError))
            {
                return false;
            }
        }
        else if (recovery.state == RecoveryState::Animation)
        {
            m_RecoveryBlendSource.clear();
        }

        // 회복 중이거나 발 기반 지지를 사용하면, 발 목표를 계속 갱신
        // 유효한 양발 목표가 있는지 검사하고, 없으면 오류를 반환 
        if (recovery.state == RecoveryState::Recovery || m_Ragdoll.UsesFootDrivenSupport())
        {
            if (!m_HasRecoverySoles)
            {
                m_PhysicsError = "Recovery requires both physical foot sole frames.";
                return false;
            }

            // 발딛기를 시작하면 지지발의 기준을 현재 상태로 다시 잡음
            // 이번 발딛기를 지탱할 왼발의 기본 목표
            if (step.justStarted && step.supportFoot >= 0 && step.supportFoot < 2)
            {
                m_RecoverySoles[step.supportFoot] = m_Ragdoll.UsesFootDrivenSupport() ? CaptureSupportSole(sample.feet[step.supportFoot]) :
                    ToRecoveryFrame(sample.feet[step.supportFoot].soleWorld);
            }

            // 움직이던 발이 착지하면 실제 착지 상태를 새 기준으로 저장
			// 예를들어 계획한 착지 X위치가 0.4m인데 실제로 0.35m에 착지했다면, 평평한 지면에서는 실제 착지한 0.35를 바탕으로 새 기본 목표를 잡음
            if (step.justLanded && step.stepFoot >= 0 && step.stepFoot < 2)
            {
                // 착지는 회복의 종료가 아니다. 착지한 발을 유지하며 COM 안정 조건을 계속 확인한다.
                m_RecoverySoles[step.stepFoot] = m_Ragdoll.UsesFootDrivenSupport() ? CaptureSupportSole(sample.feet[step.stepFoot]) :
                    ToRecoveryFrame(sample.feet[step.stepFoot].soleWorld);
            }

            // 착지 전에는 움직이는 발의 목표를 궤적에 따라 갱신
            else if (step.motion.valid && step.stepFoot >= 0 && step.stepFoot < 2)
            {
                PxTransform sole(PxIdentity);
                // 후보 검사와 같은 출발 회전에서 착지면까지 기울기를 보간한다.
                const PxTransform startSole(step.startPosition, step.startRotation);
                if (MakeFootstepSoleMotion(startSole, step.motion.position, step.targetNormal,
                    step.motion.commandProgress, sole))
                {
                    // 성공하면 이동할 발의 목표를 갱신
                    m_RecoverySoles[step.stepFoot] = sole;
                }
            }

            // 시작,착지 때 목표 변화율 계산 이력을 초기화
            if (m_Ragdoll.UsesFootDrivenSupport() && (step.justStarted || step.justLanded))
            {
                // 이륙/착지 때 실제 발로 기준을 다시 잡는 차이를 목표 속도·가속도로 오인하지 않는다.
                // 현재 물리 속도는 유지하므로 관절 PD의 제동은 계속 작동한다.
                m_Ragdoll.ResetDriveTargetHistory();
            }

            // 저장한 발바닥의 목표를 다리의 본 목표로 변환함
            // 다리 IK를 적용해서 target을 수정함
            ApplyFootstepIKTargets(target, deltaTime);

            // IK목표를 정상적으로 계산했는지 확인
            if (m_FootstepIK.GetSnapshot().status != FootstepIKStatus::Applied)
            {
                m_PhysicsError = "Recovery leg IK could not produce valid drive targets.";
                return false;
            }
            return true;
        }

        m_FootstepIK.Reset();
        m_HasRecoverySoles = false;
        return true;
    }
}
