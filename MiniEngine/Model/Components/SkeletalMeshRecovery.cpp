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
        const FootSupportSnapshot& sample = m_FootSupport.GetSnapshot();
        const FootstepSnapshot& step = sample.footstep;
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
            const PxVec3& v = sample.centerOfMassVelocity;
            input.horizontalSpeed = std::sqrt(v.x * v.x + v.z * v.z);
        }

        m_RecoveryReferenceHull.clear();
        std::array<PxTransform, 2> animatedSoles{PxTransform(PxIdentity), PxTransform(PxIdentity)};
        bool referenceValid = sample.hasCenterOfMass && sample.hasCenterOfMassVelocity;
        PxVec3 actualMidpoint(0), animationMidpoint(0);
        std::vector<PxVec3> corners;
        for (size_t side = 0; side < sample.feet.size(); ++side)
        {
            const FootSoleSample& foot = sample.feet[side];
            if (!foot.available || foot.boneNode < 0 || size_t(foot.boneNode) >= target.size())
            {
                referenceValid = false;
                break;
            }
            animatedSoles[side] = ToRecoveryFrame(target[foot.boneNode]) * ToRecoveryFrame(foot.soleInBone);
            actualMidpoint += ToRecoveryFrame(foot.soleWorld).p * .5f;
            animationMidpoint += animatedSoles[side].p * .5f;
            const FootSoleSettings& sole = m_FootSupport.GetSoleSettings(side);
            const float w = sole.halfWidth, l = sole.halfLength;
            for (const PxVec3& corner : {PxVec3(-w, 0, -l), PxVec3(w, 0, -l), PxVec3(w, 0, l), PxVec3(-w, 0, l)})
            {
                corners.push_back(animatedSoles[side].transform(corner));
            }
        }
        if (referenceValid)
        {
            // 밀리기 전 월드 위치로 돌아가라고 요구하지 않는다. 기준 발 영역을 현재 양발 중심에 맞춘다.
            const PxVec3 shift = actualMidpoint - animationMidpoint;
            for (PxVec3& corner : corners)
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
        const RecoverySnapshot& recovery = m_Recovery.GetSnapshot();
        if (input.fallen)
        {
            m_HasRecoverySoles = false;
            m_RecoveryBlendSource.clear();
            m_FootstepIK.Reset();
            m_StandingSupport.Reset();
            m_Ragdoll.SetSupportContacts({});
            return true;
        }
        if (recovery.enteredRecovery)
        {
            m_Ragdoll.ResetDriveTargetHistory();
            m_RecoveryBlendSource.clear();
        }
        if ((m_Ragdoll.UsesFootDrivenSupport() && !m_HasRecoverySoles) ||
            (recovery.enteredRecovery && !m_Ragdoll.UsesFootDrivenSupport()))
        {
            m_HasRecoverySoles = sample.feet[0].available && sample.feet[1].available;
            for (size_t side = 0; side < sample.feet.size(); ++side)
            {
                m_RecoverySoles[side] = m_Ragdoll.UsesFootDrivenSupport() ? CaptureSupportSole(sample.feet[side]) :
                    ToRecoveryFrame(sample.feet[side].soleWorld);
                m_HasRecoverySoles = m_HasRecoverySoles && m_RecoverySoles[side].isSane();
            }
        }
        if (recovery.state == RecoveryState::Animation && recovery.startedBlend)
        {
            if (!CaptureRecoveryBlendSource())
            {
                m_PhysicsError = "Cannot capture the current physical pose for animation recovery.";
                return false;
            }
            m_Ragdoll.ResetDriveTargetHistory();
        }
        if (recovery.state == RecoveryState::Animation && recovery.blending)
        {
            std::vector<GraphNode> pose = m_AnimInstance->GetPose();
            if (m_RecoveryBlendSource.size() != pose.size() ||
                !BlendRecoveryPose(m_RecoveryBlendSource, pose, recovery.blendAlpha))
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
        if (recovery.state == RecoveryState::Recovery || m_Ragdoll.UsesFootDrivenSupport())
        {
            if (!m_HasRecoverySoles)
            {
                m_PhysicsError = "Recovery requires both physical foot sole frames.";
                return false;
            }
            if (step.justStarted && step.supportFoot >= 0 && step.supportFoot < 2)
            {
                m_RecoverySoles[step.supportFoot] = m_Ragdoll.UsesFootDrivenSupport() ? CaptureSupportSole(sample.feet[step.supportFoot]) :
                    ToRecoveryFrame(sample.feet[step.supportFoot].soleWorld);
            }
            if (step.justLanded && step.stepFoot >= 0 && step.stepFoot < 2)
            {
                // 착지는 회복의 종료가 아니다. 착지한 발을 유지하며 COM 안정 조건을 계속 확인한다.
                m_RecoverySoles[step.stepFoot] = m_Ragdoll.UsesFootDrivenSupport() ? CaptureSupportSole(sample.feet[step.stepFoot]) :
                    ToRecoveryFrame(sample.feet[step.stepFoot].soleWorld);
            }
            else if (step.motion.valid && step.stepFoot >= 0 && step.stepFoot < 2)
            {
                PxTransform sole(PxIdentity);
                if (MakeFootSoleTarget(step.motion.position, step.targetNormal, animatedSoles[step.stepFoot], sole))
                {
                    m_RecoverySoles[step.stepFoot] = sole;
                }
            }
            if (m_Ragdoll.UsesFootDrivenSupport() && (step.justStarted || step.justLanded))
            {
                // 이륙/착지 때 실제 발로 기준을 다시 잡는 차이를 목표 속도·가속도로 오인하지 않는다.
                // 현재 물리 속도는 유지하므로 관절 PD의 제동은 계속 작동한다.
                m_Ragdoll.ResetDriveTargetHistory();
            }
            // 접촉/후보가 잠시 사라져도 마지막 유효한 양발 목표로 IK를 계속 푼다.
            // 발딛기 목표 출력이 잠시 멈춰도 지지 다리의 높이 복구는 계속 계산한다.
            ApplyFootstepIKTargets(target, deltaTime);
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
