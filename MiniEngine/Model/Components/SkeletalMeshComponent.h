#pragma once

#include "SkinnedMeshComponent.h"
#include "../AnimInstance.h"
#include "../PhysicsAsset.h"
#include "../PhysicsPose.h"
#include "../Ragdoll.h"

namespace PARS
{
    enum class PhysicsAnimationMode
    {
        Animation,
        PassiveRagdoll,
        ActiveRagdoll
    };
    class SkeletalMeshComponent : public SkinnedMeshComponent
    {
    public:
        SkeletalMeshComponent(Actor& owner, const std::string& name);
        // 렌더링 시작 전이나 GPU가 유휴 상태일 때 에셋을 지정한다.
        bool SetSkeletalMesh(std::shared_ptr<const Model> mesh) override;
        bool SetAnimInstance(std::unique_ptr<AnimInstance> instance);
        AnimInstance& GetAnimInstance() const
        {
            return *m_AnimInstance;
        }
        // 검증된 복사본을 보관한다. 실행 중인 물리 월드에서는 키네마틱 바디를 즉시 등록한다.
        bool SetPhysicsAsset(const PhysicsAsset& asset, std::string& error);
        const PhysicsAsset* GetPhysicsAsset() const
        {
            return m_PhysicsAsset.get();
        }
        bool ClearPhysicsAsset();
        bool BuildPhysicsSkeleton(PhysicsSkeleton& skeleton, std::string& error) const;
        bool StartRagdoll(Math::Vector3 velocity, std::string& error);
        bool SetPhysicsMode(PhysicsAnimationMode mode, Math::Vector3 velocity, std::string& error);
        PhysicsAnimationMode GetPhysicsMode() const
        {
            return m_PhysicsMode;
        }
        bool IsActiveRagdoll() const
        {
            return m_PhysicsMode == PhysicsAnimationMode::ActiveRagdoll;
        }
        // 조인트 이름을 기준으로 드라이브 값만 복사한다. 다음 고정 스텝 전에 적용된다.
        bool QueueDriveSettings(const PhysicsAsset& source, std::string& error);
        bool HasPendingDriveSettings() const
        {
            return m_PendingDriveSettings != nullptr;
        }
        void StopRagdoll();
        bool IsRagdoll() const
        {
            return m_Ragdoll.IsSimulating();
        }
        Ragdoll& GetRagdoll()
        {
            return m_Ragdoll;
        }
        const std::string& GetPhysicsError() const
        {
            return m_PhysicsError;
        }
        const std::vector<GraphNode>& GetFinalPose() const
        {
            return IsRagdoll() ? m_PhysicsPose : m_AnimInstance->GetPose();
        }

        // 선택적으로 최종 포즈를 덮어쓴다. 해당 포즈는 호출자(예: 에디터 미리보기)가 소유한다.
        bool UpdateRenderData(GraphicsContext& context, const std::vector<GraphNode>* poseOverride = nullptr);
        void Render(Renderer::MeshSorter& sorter) const;
        const ModelInstance& GetModelInstance() const
        {
            return m_ModelInstance;
        }

    protected:
        void BeginPlay() override;
        void TickComponent(float deltaTime) override;
        void FixedTickComponent(float deltaTime) override;
        void EndPlay() override
        {
            DestroyPhysicsState();
        }

    private:
        ModelInstance m_ModelInstance;
        std::unique_ptr<AnimInstance> m_AnimInstance;
        std::unique_ptr<const PhysicsAsset> m_PhysicsAsset;
        bool CreatePhysicsState(const PhysicsAsset& asset, Ragdoll& candidate, PhysicsSkeleton& skeleton, std::string& error);
        bool EvaluateAnimationPhysicsPose(const PhysicsSkeleton& skeleton, std::vector<Math::Matrix4>& worldPose, std::string& error) const;
        void DestroyPhysicsState();
        void UpdateKinematicPose(bool teleport);
        void UpdatePhysicsPose();
        Ragdoll m_Ragdoll;
        PhysicsSkeleton m_PhysicsSkeleton;
        std::vector<GraphNode> m_RagdollSourcePose, m_PhysicsPose;
        std::vector<Math::Matrix4> m_BoneStretch;
        std::string m_PhysicsError;
        PhysicsAnimationMode m_PhysicsMode = PhysicsAnimationMode::Animation;
        std::unique_ptr<const PhysicsAsset> m_PendingDriveSettings;
    };
}
