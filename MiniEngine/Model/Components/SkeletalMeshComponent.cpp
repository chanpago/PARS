#include "SkeletalMeshComponent.h"

namespace PARS
{
    SkeletalMeshComponent::SkeletalMeshComponent(Actor& owner, const std::string& name)
        : SkinnedMeshComponent(owner, name), m_AnimInstance(std::make_unique<AnimInstance>(*this))
    {
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
            const auto node = m_SkeletalMesh->m_JointIndices[i];
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
            for (const auto& joint : asset.constraints)
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
            if (!candidate.SetRootSupport(m_Ragdoll.GetRootSupportWeight(), m_Ragdoll.IsRootAxisSupported(0), error))
            {
                return false;
            }
            for (int node : candidate.GetBoneNodes())
            {
                stretch.push_back(Invert(PhysicsBoneFrame(pose[node])) * pose[node]);
            }
        }
        m_Ragdoll.Swap(candidate);
        m_PhysicsSkeleton = std::move(skeleton);
        m_PhysicsAsset = std::make_unique<const PhysicsAsset>(asset);
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
        m_AnimInstance = std::move(instance);
        UpdateKinematicPose(true);
        return true;
    }

    void SkeletalMeshComponent::TickComponent(float deltaTime)
    {
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
        if (!m_Ragdoll.IsActive())
        {
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
        UpdateFootSupport();

        //
        if (!UpdateAutomaticRootSupport(m_PhysicsError))
        {
            return;
        }

        if (m_PhysicsMode == PhysicsAnimationMode::PassiveRagdoll)
        {
            return;
        }

        // 캐릭터 이동이 컴포넌트 루트를 이미 갱신한 상태다. simulate() 전에 애니메이션을
        // 샘플링해 PVD와 렌더링이 모두 이 포즈를 사용하도록 한다.
        // 이버 스텝에서의 애니메이션 자세를 구한다.
        // 애니메이션 재생 시간을 진행시키고, 그 시간에 해당하는 본들의 자세를 계산
        // clip에서 키프레임 데이터와 시간 간격 정보를 읽어와서 두 프레임 사이의 비율을 계산하고 m_pose에 저장
        m_AnimInstance->UpdateAnimation(deltaTime);

        if (IsActiveRagdoll())
        {
            std::vector<Math::Matrix4> target;

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
            if (!EvaluateAnimationPhysicsPose(m_PhysicsSkeleton, target, m_PhysicsError) || !m_Ragdoll.UpdateDriveTargets(target, deltaTime, m_PhysicsError))
            {
                // 목표 갱신에 실패하면 전신 패시브 물리 시뮬레이션으로 전환한다.
                // 골반의 힘/토크 지지를 제거하면서 목표 갱신 오류 메시지는 보존한다.
                m_Ragdoll.SetDrivesEnabled(false);
                std::string supportError;
                m_Ragdoll.SetRootSupported(false, supportError);
                m_PhysicsMode = PhysicsAnimationMode::PassiveRagdoll;
            }
        }
        else
        {
            UpdateKinematicPose(false);
        }
    }

    bool SkeletalMeshComponent::UpdateRenderData(GraphicsContext& context, const std::vector<GraphNode>* poseOverride)
    {
        if (!m_SkeletalMesh)
        {
            return false;
        }
        UpdateFootCandidateDebugColors();
        m_ModelInstance.SetTransform(GetComponentTransform());
        return m_ModelInstance.UpdatePose(context, IsRagdoll() ? m_PhysicsPose : poseOverride ? *poseOverride
                                                                                              : m_AnimInstance->GetPose());
    }

    void SkeletalMeshComponent::Render(Renderer::MeshSorter& sorter) const
    {
        if (IsVisible())
        {
            m_ModelInstance.Render(sorter);
        }
    }
}
