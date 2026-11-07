#include "SkeletalMeshComponent.h"
#include <cmath>
#include "../GameFramework/Character.h"

using namespace Math;
using namespace physx;

namespace PARS
{
    void SkeletalMeshComponent::SetAutomaticRagdoll(bool enabled)
    {
        m_AutomaticRagdoll = enabled;
        m_KinematicRecoveryTimer.Reset();
        m_KinematicBlendSource.clear();
        m_KinematicBlendPose.clear();
        m_KinematicBlendTime = 0;
        m_PendingRecoveryImpact = false;
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
            m_RootSupportRelease = RootSupportRelease::None;
            m_FootSupport.ResetFootstep();
            ResetRecovery();
            m_KinematicRecoveryTimer.Reset();
            m_PendingRecoveryImpact = true;
            m_Ragdoll.ApplyExternalImpacts(impacts);
            return m_Ragdoll.SetRootSupported(true, m_PhysicsError);
        }
        if (!IsActiveRagdoll())
        {
            m_KinematicRecoveryTimer.Reset();
            return true;
        }
        const bool insideA = sample.hasComProjection && sample.balance.zone == FootSupportZone::Inside;
        if (m_KinematicRecoveryTimer.Update(sample.contactRevision, sample.completedPhysicsStep,
            deltaTime, insideA, impact, m_KinematicStableDuration))
        {
            return BeginKinematicRecovery();
        }
        return true;
    }

    bool SkeletalMeshComponent::BeginKinematicRecovery()
    {
        std::vector<Matrix4> targets;
        const int root = m_Ragdoll.GetImpactRootBody();
        if (root < 0 || !EvaluateAnimationPhysicsPose(m_PhysicsSkeleton, targets, m_PhysicsError))
        {
            return false;
        }
        // 충격 전 원점으로 되돌리지 않고 현재 골반의 수평 위치에서 애니메이션을 시작한다.
        const Vector3 actual(m_Ragdoll.GetBoneWorld(static_cast<size_t>(root)).GetW());
        const Vector3 target(targets[m_Ragdoll.GetBoneNodes()[root]].GetW());
        const Vector3 previousLocation = GetOwner().GetActorLocation();
        GetOwner().SetActorLocation(previousLocation + Vector3(actual.GetX() - target.GetX(), 0, actual.GetZ() - target.GetZ()));
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
            const PxTransform actual = ToRecoveryFrame(foot.soleWorld);
            PxVec3 points[3];
            unsigned count = 0;
            for (size_t corner = 0; corner < foot.projected.size() && count < 3; ++corner)
            {
                if (foot.projected[corner])
                {
                    points[count++] = foot.projections[corner];
                }
            }
            if (count < 3)
            {
                return actual;
            }
            PxVec3 normal = (points[1] - points[0]).cross(points[2] - points[0]);
            if (normal.normalize() < 1e-6f)
            {
                return actual;
            }
            if (normal.y < 0)
            {
                normal = -normal;
            }
            if (normal.y < .70710678f)
            {
                return actual;
            }
            // 초기 공중 위치를 영구 고정점으로 저장하지 않는다. 실제 레이로 얻은 바닥에 발 목표를 둔다.
            // 목표만 바꾸며 물리 접촉 판정이나 실제 바디 위치는 수정하지 않는다.
            const PxVec3 position = actual.p - normal * normal.dot(actual.p - points[0]);
            PxTransform planted(PxIdentity);
            return MakeFootSoleTarget(position, normal, actual, planted) ? planted : actual;
        }
    }

    void SkeletalMeshComponent::ResetRecovery()
    {
        m_Recovery.Reset();
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

        // 현재 애니메이션이 안정적으로 idle을 재생 중인지 확인
        // 애니메이션 클립사이의 전환 보간 중이 아니여야함. 현재 재생 중인 애니메이션 이름이 idle
        // 예를들어 Idle->Walk 전환 중일 때 걷기 애니메이션을 재생 중이면 거짓
        const bool idle = !m_AnimInstance->IsBlending() && m_AnimInstance->GetAnimationName(m_AnimInstance->GetCurrentAnimationIndex()) == "Idle";

        // 회복 제어의 실행 조건을 검사
        // 회복 판단에 사용할 입력 객체를 만듦
        RecoveryInput input;
        input.enabled = IsActiveRagdoll() && m_AutomaticRootSupport && idle;

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

        // 이전 기준 영역 삭제
        m_RecoveryReferenceHull.clear();

        // 애니메이션 목표 발바닥의 위치,회전을 담을 배열을 준비
        std::array<PxTransform, 2> animatedSoles{PxTransform(PxIdentity), PxTransform(PxIdentity)};

        // 기준 계산에 필요한 COM정보가 있는지 확인
        bool referenceValid = sample.hasCenterOfMass && sample.hasCenterOfMassVelocity;

        // 양발 애니메이션,AR 중심 준비
        PxVec3 actualMidpoint(0), animationMidpoint(0);
        std::vector<PxVec3> corners;

        // 애니메이션 발바닥 목표, 실제, 애니메이션 양발의 중점, 발바닥 모서리 좌표를 계산하는 과정
        for (size_t side = 0; side < sample.feet.size(); ++side)
        {
            const FootSoleSample& foot = sample.feet[side];
            if (!foot.available || foot.boneNode < 0 || size_t(foot.boneNode) >= target.size())
            {
                referenceValid = false;
                break;
            }

            // 애니메이션이 요구하는 발바닥의 월드 위치, 회전을 계산
            // 발바닥의 월드변환 = 발 본의 월드변환 * 발바닥의 본기준 로컬 변환
            animatedSoles[side] = ToRecoveryFrame(target[foot.boneNode]) * ToRecoveryFrame(foot.soleInBone);
            actualMidpoint += ToRecoveryFrame(foot.soleWorld).p * .5f;
            animationMidpoint += animatedSoles[side].p * .5f;

            // 발바닥 사각형의 크기를 가져옴
            const FootSoleSettings& sole = m_FootSupport.GetSoleSettings(side);
            const float w = sole.halfWidth, l = sole.halfLength;

            // 
            for (const PxVec3& corner : {PxVec3(-w, 0, -l), PxVec3(w, 0, -l), PxVec3(w, 0, l), PxVec3(-w, 0, l)})
            {
                // 발바닥 네 모서리를 월드 좌표로 바꿈
                corners.push_back(animatedSoles[side].transform(corner));
            }
        }

        // COM과 양발 정보등을 확인하여 참일때만 실행
        if (referenceValid)
        {
            // 애니메이션 양발 중심을 실제 양발 중심으로 옮길 이동량을 구함
            const PxVec3 shift = actualMidpoint - animationMidpoint;

            // 애니메이션 목표와 실제발의 위치 차만큼 모서리를 이동시킴
            for (PxVec3& corner : corners)
            {
                corner += shift;
                corner.y = sample.projectedCenterOfMass.y;
            }
            // 발 모서리 4개를 이용해서 다각형을 구함
            m_RecoveryReferenceHull = BuildFootSupportHull(std::move(corners));

            // 최소 3개로 다각형을 이루었는지
            input.hasReference = m_RecoveryReferenceHull.size() >= 3;
			// 현재 COM이 기준 다각형 안에 있는지
            input.insideReference = IsInsideFootSupportHull(m_RecoveryReferenceHull, sample.centerOfMass);
			// 예측 COM이 기준 다각형 안에 있는지
            input.predictionInsideReference = step.hasPrediction && IsInsideFootSupportHull(m_RecoveryReferenceHull, step.predictedCom);
        }

        // m_RecoverySoles : 회복,지지에 사용할 양발 발바닥의 목표 위치,회전
		// m_RecoveryBlendSource : 애니메이션 복귀를 시작할 때 캡처한 물리 본 자세

        // 앞서 정의한 RecoveryInput input;으로 회복 상태를 갱신. 충격, COM, 발딛기, 접지 상태 등을 보고 회복 제어가 현재 상태를 결정
        m_Recovery.Update(input, m_RecoverySettings);
        const RecoverySnapshot& recovery = m_Recovery.GetSnapshot();

        // 지지가 해제되거나 발딛기가 중단됐다면 발 보정 정보를 정리
        if (input.fallen)
        {
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

                /*
				step.motion.position : 발딛기 궤적에 따라 계산된 발바닥 위치
				step.targetNormal : 목표 발바닥의 위 방향을 정할 지면 법선
				animatedSoles[step.stepFoot] : 발의 전방 방향을 얻을 애니메이션 기준 자세
                sole : 계산 결과를 받을 위치,회전 변환
                */
                if (MakeFootSoleTarget(step.motion.position, step.targetNormal, animatedSoles[step.stepFoot], sole))
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
