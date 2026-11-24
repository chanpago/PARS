#include "SkeletalMeshComponent.h"
#include "FramePerformance.h"
#include <algorithm>

namespace PARS
{
    SkeletalMeshComponent::SkeletalMeshComponent(Actor& owner, const std::string& name)
        : SkinnedMeshComponent(owner, name), m_AnimInstance(std::make_unique<AnimInstance>(*this))
    {
        m_DirectionalFallingAnimations.fill(UINT32_MAX);
    }

    bool SkeletalMeshComponent::SetSkeletalMesh(std::shared_ptr<const Model> mesh)
    {
        if (IsRagdoll())
        {
            return false;
        }
        if (mesh && (mesh->m_NumNodes == 0 || mesh->m_NumJoints == 0))
        {
            return false;
        }
        DestroyPhysicsState();
        m_SkeletalMesh = std::move(mesh);
        m_ModelInstance = m_SkeletalMesh;
        m_AnimInstance->Initialize(m_SkeletalMesh);
        m_FallingAnimationIndex = UINT32_MAX;
        m_DirectionalFallingAnimations.fill(UINT32_MAX);
        m_AirborneFallAnimations.fill(UINT32_MAX);
        m_StandingUpAnimationIndex = UINT32_MAX;
        m_StandingIdleAnimationIndex = UINT32_MAX;
        m_StandingUpStableTime = 0;
        m_StandingUpState = StandingUpState::None;
        m_PhysicsAsset.reset();
        return true;
    }

    bool SkeletalMeshComponent::SetPhysicsAsset(const PhysicsAsset& asset, std::string& error, bool restartSimulation)
    {
        if (IsRagdoll() && !restartSimulation)
        {
            error = "Stop the ragdoll before replacing its PA.";
            return false;
        }
        if (!m_SkeletalMesh)
        {
            error = "Assign a skeletal mesh first.";
            return false;
        }
        std::vector<std::string> joints;
        for (uint32_t i = 0; i < m_SkeletalMesh->m_NumJoints; ++i)
        {
            const uint16_t node = m_SkeletalMesh->m_JointIndices[i];
            if (node >= m_SkeletalMesh->m_NodeNames.size())
            {
                error = "Skeletal mesh has an invalid joint index.";
                return false;
            }
            joints.push_back(m_SkeletalMesh->m_NodeNames[node]);
        }
        if (!asset.ValidateSkeleton(joints, error))
        {
            return false;
        }
        Ragdoll candidate;
        PhysicsSkeleton skeleton;
        if (HasBegunPlay() && !CreatePhysicsState(asset, candidate, skeleton, error))
        {
            return false;
        }
        std::vector<Math::Matrix4> stretch;
        if (IsRagdoll())
        {
            bool hasDrive = false;
            for (const PARS::PhysicsConstraint& joint : asset.constraints)
            {
                hasDrive |= HasEnabledAngularDrive(joint);
            }
            if (asset.bodies.empty() || (IsActiveRagdoll() && !hasDrive))
            {
                error = "A simulated PA needs bodies and active ragdolls need enabled drives.";
                return false;
            }
            std::vector<Math::Matrix4> pose, physicalPose;
            if (!EvaluatePhysicsPose(skeleton, GetFinalPose(), Math::Matrix4(GetComponentTransform()), physicalPose, error) ||
                !candidate.UpdateKinematicPose(physicalPose, true, error) ||
                !EvaluateAnimationPhysicsPose(skeleton, pose, error) ||
                (IsActiveRagdoll() && !candidate.UpdateDriveTargets(pose, 0, error)) ||
                !candidate.SetSimulating(true, Math::Vector3(Math::kZero), error))
            {
                return false;
            }
            candidate.SetDrivesEnabled(IsActiveRagdoll());
            candidate.SetFootDrivenSupport(m_Ragdoll.UsesFootDrivenSupport());
            if (!candidate.SetRootSupport(m_Ragdoll.GetRootSupportWeight(), m_Ragdoll.IsRootAxisSupported(0), error))
            {
                return false;
            }
            for (int node : candidate.GetBoneNodes())
            {
                stretch.push_back(Invert(PhysicsBoneFrame(pose[node])) * pose[node]);
            }
        }
        ResetTorsoBalance();
        m_Ragdoll.Swap(candidate);
        m_ImpactWarmup = {};
        m_ImpactFlail = {};
        ResetCrossStepPlayback();
        ResetFallArms();
        m_FallArmsInitialized = false;
        m_PhysicsSkeleton = std::move(skeleton);
        m_PhysicsAsset = std::make_unique<const PhysicsAsset>(asset);
        m_FootSupport.OnPhysicsAssetChanged(*m_PhysicsAsset);
        m_PendingDriveSettings.reset();
        if (IsRagdoll())
        {
            m_BoneStretch = std::move(stretch);
            m_RagdollSourcePose = m_AnimInstance->GetPose();
            UpdatePhysicsPose();
        }
        UpdateFootSupport();
        m_PhysicsError.clear();
        error.clear();
        return true;
    }

    bool SkeletalMeshComponent::SetAnimInstance(std::unique_ptr<AnimInstance> instance)
    {
        if (IsRagdoll())
        {
            return false;
        }
        if (!instance || &instance->GetOwningComponent() != this)
        {
            return false;
        }
        instance->Initialize(m_SkeletalMesh);
        ResetTorsoBalance();
        m_ImpactWarmup = {};
        m_AnimInstance = std::move(instance);
        ResetFallArms();
        ResetAirborneFall();
        m_FallingAnimationIndex = UINT32_MAX;
        m_DirectionalFallingAnimations.fill(UINT32_MAX);
        m_AirborneFallAnimations.fill(UINT32_MAX);
        m_StandingUpAnimationIndex = UINT32_MAX;
        m_StandingIdleAnimationIndex = UINT32_MAX;
        m_StandingUpStableTime = 0;
        m_StandingUpState = StandingUpState::None;
        UpdateKinematicPose(true);
        return true;
    }

    void SkeletalMeshComponent::TickComponent(float deltaTime)
    {
        FramePerformance::Scope poseTiming(FramePerformance::Category::ParsPose);
        if (IsRagdoll())
        {
            UpdatePhysicsPose();
        }
        else if (m_Ragdoll.IsActive())
        {
            UpdateKinematicPose(true);
        }
        else
        {
            m_AnimInstance->UpdateAnimation(deltaTime);
        }
    }

    /*
    * 애니메이션을 진행하고, 래그돌이 따라갈 목표 자세를 갱신한다
    */
    void SkeletalMeshComponent::FixedTickComponent(float deltaTime)
    {
        FramePerformance::Scope controlTiming(FramePerformance::Category::ParsControl);
        m_FootstepCollision = {};
        if (!m_Ragdoll.IsActive())
        {
            ResetFallArms();
            return;
        }

        if (m_PendingDriveSettings)
        {
            if (m_Ragdoll.ApplyDriveSettings(*m_PendingDriveSettings, m_PhysicsError))
            {
                m_PhysicsAsset = std::move(m_PendingDriveSettings);
            }
            else
            {
                m_PendingDriveSettings.reset();
            }
        }

        // 여기서 발 움직임을 계산하고 지지영역을 조사해 가중치를 구한다
        m_Ragdoll.SetHorizontalSupportForce(physx::PxVec3(0));
        ImpactHandReach hit;
        std::vector<RagdollImpactResponse> responses;
        if (m_Ragdoll.IsSimulating() || m_AutomaticRagdoll || IsImpactWarmupActive())
        {
            const std::vector<RagdollBodyImpulse>& impacts = m_Ragdoll.GetExternalImpacts();
            if (IsActiveRagdoll() || (m_AutomaticRagdoll && !IsRagdoll()) || IsImpactWarmupActive())
            {
                // 예열이나 물리 모드 전환이 접촉을 비우기 전에 같은 스텝으로 상처와 손 목표를 기록한다.
                m_Ragdoll.UpdateArmWounds(impacts);
                hit = FindImpactHandReach(impacts);
            }
            responses = m_Ragdoll.GetImpactResponses(impacts);
        }
        // 예열이 시작되면 같은 충격을 키네마틱 자세가 이어받으므로 동적 바디 적용을 그 결정 뒤로 미룬다.
        const bool capsuleWarmup = (m_ImpactWarmup.wasResting || IsImpactWarmupActive()) &&
            std::any_of(responses.begin(), responses.end(), [](const RagdollImpactResponse& response) -> bool
            {
                return response.fromCharacterCapsule;
            });
        if (!capsuleWarmup)
        {
            m_Ragdoll.ApplyImpactResponses(responses);
        }
        if (IsActiveRagdoll())
        {
            StartDynamicTorsoBalance(m_Ragdoll.GetExternalImpacts());
            UpdateDynamicTorsoBalance(deltaTime);
        }
        UpdateFootSupport();
        if (UpdateImpactWarmup(deltaTime, responses))
        {
            return;
        }
        if (capsuleWarmup)
        {
            m_Ragdoll.ApplyImpactResponses(responses);
        }
        bool longitudinalImpact = false;
        if (IsActiveRagdoll() || (m_AutomaticRagdoll && !IsRagdoll()))
        {
            const std::vector<RagdollBodyImpulse>& impacts = m_Ragdoll.GetExternalImpacts();
            if (!IsFalling() && !IsGroundFallActive() && m_StandingUpState == StandingUpState::None)
            {
                physx::PxVec3 impulse(0);
                for (const RagdollBodyImpulse& contact : impacts)
                {
                    impulse += contact.impulse;
                }
                const Math::Vector3 facing(GetComponentTransform().GetZ());
                const float axial = float(facing.GetX()) * impulse.x + float(facing.GetZ()) * impulse.z;
                const float lateral = float(facing.GetZ()) * impulse.x - float(facing.GetX()) * impulse.z;
                longitudinalImpact = std::abs(axial) > 1e-5f && std::abs(axial) >= std::abs(lateral);
            }
            if (IsActiveRagdoll())
            {
                m_Ragdoll.UpdateProjectileReaction(impacts, deltaTime);
            }
        }

        const std::vector<RagdollBodyImpulse>& contacts = m_Ragdoll.GetExternalImpacts();
        const bool capsuleImpact = m_AutomaticRootSupport && std::any_of(contacts.begin(), contacts.end(), [](const RagdollBodyImpulse& impact) -> bool
        {
            return impact.newCapsuleHit && impact.impulse.magnitudeSquared() > 1e-8f;
        });
        const bool woundNeedsPhysics = !IsRagdoll() && m_Ragdoll.HasArmWound();
        if (!UpdateAutomaticRagdoll(deltaTime, hit.body >= 0 || !responses.empty() || capsuleImpact || woundNeedsPhysics))
        {
            return;
        }
        if (hit.body >= 0 && IsActiveRagdoll())
        {
            m_ImpactHandReach = hit;
        }
        if (!UpdateAutomaticRootSupport(m_PhysicsError))
        {
            ResetCrossStepPlayback();
            return;
        }

        if (m_PhysicsMode == PhysicsAnimationMode::PassiveRagdoll)
        {
            ResetFallArms();
            return;
        }

        // 지지 해제 판정이 끝난 같은 물리 스텝에서 낙상 클립을 선택한다.
        const bool fallingTargetReady = UpdateBalanceState(deltaTime) && UpdateFallingAnimation(deltaTime);
        if (!fallingTargetReady && !IsActiveRagdoll())
        {
            return;
        }

        // 캐릭터 이동이 컴포넌트 루트를 이미 갱신한 상태다. simulate() 전에 애니메이션을
        // 샘플링해 PVD와 렌더링이 모두 이 포즈를 사용하도록 한다.
        // 이버 스텝에서의 애니메이션 자세를 구한다.
        // 애니메이션 재생 시간을 진행시키고, 그 시간에 해당하는 본들의 자세를 계산
        // clip에서 키프레임 데이터와 시간 간격 정보를 읽어와서 두 프레임 사이의 비율을 계산하고 m_pose에 저장
        // 낙상·기상 중에는 해당 클립의 시간을 진행하고 일반 이동 클립 선택은 복귀 후 재개한다.
        m_AnimInstance->UpdateAnimation(deltaTime, !IsFalling() && !IsGroundFallActive() &&
            m_StandingUpState == StandingUpState::None);

        if (IsActiveRagdoll())
        {
            std::vector<Math::Matrix4> target;
            UpdateImpactFlail(longitudinalImpact, deltaTime);

            // 이 부분에서 액티브 래그돌 물리처리를한다(pd구동 및 추가보상토크계산)
            // EvaluateAnimationPhysicsPose부분에서 이번 프레임의 애니메이션의 목표 본 자세를 월드 기준으로 계산하고 target에 저장한다.
            // 
            // UpdateDriveTargets부분은 좀 길다 순서대로 설명하면,
            // 1. 앞서 계산한 target(애니메이션 목표 본 자세)을 기반으로 이번 프레임의 리지드바디의 목표 자세를 계산한다.
            // 2. 1에서 계산한 리지드 바디의 목표 자세를 기반으로 각 조인트의 목표 자세를 계산
            // 3. 골반 보정을 실행한다. 위치, 속도
            // 4. targets에 있는 부모기준 상대각 차이 쿼터니언을 physx가 읽을 수 있도록 vec3기반 각속도벡터로 만든다.
            // 5. Articulation Drive update <- 이 부분이 중요하다
            //      - 골반보정
            //      - 타겟을 목표로 유효관성을 계산하고 중력보상, 코리올리-원심력 보상을 토크값에 더함
            //      - 각 관절의 각 축에 얼마만큼의 PD를 적용시킬지 계산
            bool evaluated = fallingTargetReady && EvaluateAnimationPhysicsPose(m_PhysicsSkeleton, target, m_PhysicsError, &GetImpactAnimationPose());
            if (evaluated && !IsFalling())
            {
                evaluated = ApplyRecoveryTargets(target, deltaTime);
                if (evaluated)
                {
                    UpdateHorizontalBalance(deltaTime);
                    if (m_TorsoBalance.IsActive())
                    {
                        ApplyTorsoBalancePose(target);
                        m_Ragdoll.ApplyTorsoHipTorque(m_TorsoRig, m_TorsoBalance.GetHipTorque());
                    }
                }
            }
            if (evaluated && (!IsFalling() || m_ImpactHandReach.armWound))
            {
                ApplyArmTargets(target);
            }
            if (!evaluated || !m_Ragdoll.UpdateDriveTargets(target, deltaTime, m_PhysicsError))
            {
                // 목표 갱신에 실패하면 전신 패시브 물리 시뮬레이션으로 전환한다.
                // 골반의 힘/토크 지지를 제거하면서 목표 갱신 오류 메시지는 보존한다.
                m_FootstepIK.Reset();
                m_FootstepCollision = {};
                ResetCrossStepPlayback();
                ResetFallArms();
                m_Ragdoll.SetDrivesEnabled(false);
                std::string supportError;
                m_Ragdoll.SetRootSupported(false, supportError);
                m_PhysicsMode = PhysicsAnimationMode::PassiveRagdoll;
                ResetAirborneFall();
            }
        }
        else
        {
            ResetFallArms();
            if (!UpdateKinematicRecoveryPose(deltaTime))
            {
                return;
            }
            UpdateKinematicPose(false);
        }
    }

    bool SkeletalMeshComponent::UpdateRenderData(GraphicsContext& context, const std::vector<GraphNode>* poseOverride)
    {
        FramePerformance::Scope renderTiming(FramePerformance::Category::RenderPreparation);
        if (!m_SkeletalMesh)
        {
            return false;
        }
        UpdateFootCandidateDebugColors();
        m_ModelInstance.SetTransform(GetComponentTransform());
        return m_ModelInstance.UpdatePose(context, IsRagdoll() || IsImpactWarmupActive() || IsBlendingToKinematic() ? GetFinalPose() :
            poseOverride ? *poseOverride : m_AnimInstance->GetPose());
    }

    void SkeletalMeshComponent::Render(Renderer::MeshSorter& sorter) const
    {
        if (IsVisible())
        {
            m_ModelInstance.Render(sorter);
        }
    }
}
