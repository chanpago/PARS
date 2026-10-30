#include "SkeletalMeshComponent.h"
#include <cmath>

using namespace Math;
using namespace physx;

namespace PARS
{
    namespace
    {
        PxTransform ToRecoveryFrame(const Matrix4& matrix)
        {
            const auto frame = PhysicsJointFrame::FromTransform(PhysicsBoneFrame(matrix));
            return PxTransform(PxVec3(frame.offset.x, frame.offset.y, frame.offset.z),
                PxQuat(frame.rotation.x, frame.rotation.y, frame.rotation.z, frame.rotation.w));
        }
    }

    void SkeletalMeshComponent::ResetRecovery()
    {
        m_Recovery.Reset();
        m_HasRecoverySoles = false;
        m_RecoveryReferenceHull.clear();
        m_RecoveryBlendSource.clear();
        m_FootstepIK.Reset();
    }

    bool SkeletalMeshComponent::CaptureRecoveryBlendSource()
    {
        std::vector<Matrix4> bodies;
        const auto& nodes = m_Ragdoll.GetBoneNodes();
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
        const auto& sample = m_FootSupport.GetSnapshot();
        const auto& step = sample.footstep;
        const bool idle = !m_AnimInstance->IsBlending() &&
            m_AnimInstance->GetAnimationName(m_AnimInstance->GetCurrentAnimationIndex()) == "Idle";
        RecoveryInput input;
        input.enabled = IsActiveRagdoll() && m_AutomaticRootSupport && idle;
        if (!input.enabled)
        {
            ResetRecovery();
            return true;
        }
        input.fallen = m_RootSupportRelease != RootSupportRelease::None || step.phase == FootstepPhase::Aborted;
        input.needsStep = step.needsStep;
        input.stepping = step.IsInProgress();
        input.deltaTime = deltaTime;
        input.revision = sample.contactRevision;
        input.completedStep = sample.completedPhysicsStep;
        input.insideSupport = sample.hasCenterOfMass && sample.comInsideHull;
        input.predictionInsideSupport = step.hasPrediction && !step.needsStep;
        input.bothGrounded = sample.feet[0].contact.hasSample && sample.feet[0].contact.grounded &&
            sample.feet[1].contact.hasSample && sample.feet[1].contact.grounded;
        if (sample.totalMass > 0)
        {
            // 충격량을 전신 질량으로 나눠 모델 크기에 덜 의존하는 속도 변화 단위로 판정한다.
            input.impactVelocity = m_Ragdoll.GetExternalImpactImpulse() / sample.totalMass;
        }
        if (sample.hasCenterOfMassVelocity)
        {
            const auto& v = sample.centerOfMassVelocity;
            input.horizontalSpeed = std::sqrt(v.x * v.x + v.z * v.z);
        }

        m_RecoveryReferenceHull.clear();
        std::array<PxTransform, 2> animatedSoles{PxTransform(PxIdentity), PxTransform(PxIdentity)};
        bool referenceValid = sample.hasCenterOfMass && sample.hasCenterOfMassVelocity;
        PxVec3 actualMidpoint(0), animationMidpoint(0);
        std::vector<PxVec3> corners;
        for (size_t side = 0; side < sample.feet.size(); ++side)
        {
            const auto& foot = sample.feet[side];
            if (!foot.available || foot.boneNode < 0 || size_t(foot.boneNode) >= target.size())
            {
                referenceValid = false;
                break;
            }
            animatedSoles[side] = ToRecoveryFrame(target[foot.boneNode]) * ToRecoveryFrame(foot.soleInBone);
            actualMidpoint += ToRecoveryFrame(foot.soleWorld).p * .5f;
            animationMidpoint += animatedSoles[side].p * .5f;
            const auto& sole = m_FootSupport.GetSoleSettings(side);
            const float w = sole.halfWidth, l = sole.halfLength;
            for (const auto& corner : {PxVec3(-w, 0, -l), PxVec3(w, 0, -l), PxVec3(w, 0, l), PxVec3(-w, 0, l)})
            {
                corners.push_back(animatedSoles[side].transform(corner));
            }
        }
        if (referenceValid)
        {
            // 밀리기 전 월드 위치로 돌아가라고 요구하지 않는다. 기준 발 영역을 현재 양발 중심에 맞춘다.
            const PxVec3 shift = actualMidpoint - animationMidpoint;
            for (auto& corner : corners)
            {
                corner += shift;
                corner.y = sample.projectedCenterOfMass.y;
            }
            m_RecoveryReferenceHull = BuildFootSupportHull(std::move(corners));
            input.hasReference = m_RecoveryReferenceHull.size() >= 3;
            input.insideReference = IsInsideFootSupportHull(m_RecoveryReferenceHull, sample.centerOfMass);
            input.predictionInsideReference = step.hasPrediction &&
                IsInsideFootSupportHull(m_RecoveryReferenceHull, step.predictedCom);
        }

        m_Recovery.Update(input, m_RecoverySettings);
        const auto& recovery = m_Recovery.GetSnapshot();
        if (input.fallen)
        {
            m_HasRecoverySoles = false;
            m_RecoveryBlendSource.clear();
            m_FootstepIK.Reset();
            return true;
        }
        if (recovery.enteredRecovery)
        {
            m_Ragdoll.ResetDriveTargetHistory();
            m_RecoveryBlendSource.clear();
            m_HasRecoverySoles = sample.feet[0].available && sample.feet[1].available;
            for (size_t side = 0; side < sample.feet.size(); ++side)
            {
                m_RecoverySoles[side] = ToRecoveryFrame(sample.feet[side].soleWorld);
                m_HasRecoverySoles = m_HasRecoverySoles && m_RecoverySoles[side].isSane();
            }
        }
        if (recovery.state == RecoveryState::Recovery)
        {
            if (!m_HasRecoverySoles)
            {
                m_PhysicsError = "Recovery requires both physical foot sole frames.";
                return false;
            }
            if (step.justStarted && step.supportFoot >= 0 && step.supportFoot < 2)
            {
                m_RecoverySoles[step.supportFoot] = ToRecoveryFrame(sample.feet[step.supportFoot].soleWorld);
            }
            if (step.justLanded && step.stepFoot >= 0 && step.stepFoot < 2)
            {
                // 착지는 회복의 종료가 아니다. 착지한 발을 유지하며 COM 안정 조건을 계속 확인한다.
                m_RecoverySoles[step.stepFoot] = ToRecoveryFrame(sample.feet[step.stepFoot].soleWorld);
            }
            else if (step.motion.valid && step.stepFoot >= 0 && step.stepFoot < 2)
            {
                PxTransform sole(PxIdentity);
                if (MakeFootSoleTarget(step.motion.position, step.targetNormal, animatedSoles[step.stepFoot], sole))
                {
                    m_RecoverySoles[step.stepFoot] = sole;
                }
            }
            // 접촉/후보가 잠시 사라져도 마지막 유효한 양발 목표로 IK를 계속 푼다.
            ApplyFootstepIKTargets(target);
            if (m_FootstepIK.GetSnapshot().status != FootstepIKStatus::Applied)
            {
                m_PhysicsError = "Recovery leg IK could not produce valid drive targets.";
                return false;
            }
            return true;
        }

        m_FootstepIK.Reset();
        if (recovery.startedBlend)
        {
            if (!CaptureRecoveryBlendSource())
            {
                m_PhysicsError = "Cannot capture the current physical pose for animation recovery.";
                return false;
            }
            m_Ragdoll.ResetDriveTargetHistory();
        }
        if (recovery.blending)
        {
            auto pose = m_AnimInstance->GetPose();
            if (m_RecoveryBlendSource.size() != pose.size())
            {
                m_PhysicsError = "Recovery blend source does not match the skeleton.";
                return false;
            }
            // 애니메이션 시간은 계속 진행한다. 로컬 본 자세를 섞어 길이와 본 계층을 보존한다.
            if (!BlendRecoveryPose(m_RecoveryBlendSource, pose, recovery.blendAlpha))
            {
                m_PhysicsError = "Invalid physical pose in the recovery blend.";
                return false;
            }
            return EvaluatePhysicsPose(m_PhysicsSkeleton, pose, Matrix4(GetComponentTransform()), target, m_PhysicsError);
        }
        m_RecoveryBlendSource.clear();
        m_HasRecoverySoles = false;
        return true;
    }
}
