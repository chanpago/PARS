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

    bool SkeletalMeshComponent::SetPhysicsAsset(const PhysicsAsset& asset, std::string& error)
    {
        if (IsRagdoll())
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
        m_Ragdoll.Swap(candidate);
        m_PhysicsSkeleton = std::move(skeleton);
        m_PhysicsAsset = std::make_unique<const PhysicsAsset>(asset);
        m_PendingDriveSettings.reset();
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
