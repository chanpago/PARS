#pragma once

#include "SkinnedMeshComponent.h"
#include "../AnimInstance.h"
#include "../PhysicsAsset.h"
#include "../PhysicsPose.h"
#include "../Ragdoll.h"
#include "../FootSupport.h"
#include "../FootstepIK.h"
#include "../RagdollRecovery.h"
#include "../StandingSupport.h"

namespace PARS
{
    enum class PhysicsAnimationMode
    {
        Animation,
        PassiveRagdoll,
        ActiveRagdoll
    };
    enum class RootSupportRelease
    {
        None,
        OutsideBuffer,
        NoSupport,
        Inactive,
        Error
    };
    class SkeletalMeshComponent : public SkinnedMeshComponent
    {
    public:
        SkeletalMeshComponent(Actor& owner, const std::string& name);

    public:
        // 렌더링 시작 전이나 GPU가 유휴 상태일 때 에셋을 지정한다.
        bool SetSkeletalMesh(std::shared_ptr<const Model> mesh) override;
        bool SetAnimInstance(std::unique_ptr<AnimInstance> instance);
        // 검증된 복사본을 보관한다. 실행 중인 물리 월드에서는 키네마틱 바디를 즉시 등록한다.
        // 재생성을 허용하면 현재 본 자세와 물리 모드를 유지한다. 물리 스텝 밖에서 호출한다.
        bool SetPhysicsAsset(const PhysicsAsset& asset, std::string& error, bool restartSimulation = false);
        bool ClearPhysicsAsset();
        bool BuildPhysicsSkeleton(PhysicsSkeleton& skeleton, std::string& error) const;
        bool StartRagdoll(Math::Vector3 velocity, std::string& error);
        bool SetPhysicsMode(PhysicsAnimationMode mode, Math::Vector3 velocity, std::string& error);
        // 조인트 이름을 기준으로 드라이브 값만 복사한다. 다음 고정 스텝 전에 적용된다.
        bool QueueDriveSettings(const PhysicsAsset& source, std::string& error);
        void StopRagdoll();
        // Explicit mode changes rearm the controller. The dummy changes mode only through Reset().
        bool SetAutomaticRootSupport(bool enabled, std::string& error);
        Math::Vector3 ConstrainHorizontalMovement(Math::Vector3 displacement) const;
        // 선택적으로 최종 포즈를 덮어쓴다. 해당 포즈는 호출자(예: 에디터 미리보기)가 소유한다.
        bool UpdateRenderData(GraphicsContext& context, const std::vector<GraphNode>* poseOverride = nullptr);
        void Render(Renderer::MeshSorter& sorter) const;

    protected:
        void BeginPlay() override;
        void TickComponent(float deltaTime) override;
        void FixedTickComponent(float deltaTime) override;
        void EndPlay() override
        {
            DestroyPhysicsState();
        }

    private:
        bool CreatePhysicsState(const PhysicsAsset& asset, Ragdoll& candidate, PhysicsSkeleton& skeleton, std::string& error);
        bool EvaluateAnimationPhysicsPose(const PhysicsSkeleton& skeleton, std::vector<Math::Matrix4>& worldPose, std::string& error) const;
        void DestroyPhysicsState();
        void UpdateKinematicPose(bool teleport);
        void UpdatePhysicsPose();
        void UpdateFootSupport();
        void ApplyFootstepIKTargets(std::vector<Math::Matrix4>& target, float deltaTime);
        bool ApplyRecoveryTargets(std::vector<Math::Matrix4>& target, float deltaTime);
        void ResetRecovery();
        bool CaptureRecoveryBlendSource();
        void UpdateFootCandidateDebugColors();
        bool UpdateAutomaticRootSupport(std::string& error);

    public:
        AnimInstance& GetAnimInstance() const
        {
            return *m_AnimInstance;
        }
        const PhysicsAsset* GetPhysicsAsset() const
        {
            return m_PhysicsAsset.get();
        }
        PhysicsAnimationMode GetPhysicsMode() const
        {
            return m_PhysicsMode;
        }
        bool IsActiveRagdoll() const
        {
            return m_PhysicsMode == PhysicsAnimationMode::ActiveRagdoll;
        }
        bool HasPendingDriveSettings() const
        {
            return m_PendingDriveSettings != nullptr;
        }
        bool IsRagdoll() const
        {
            return m_Ragdoll.IsSimulating();
        }
        Ragdoll& GetRagdoll()
        {
            return m_Ragdoll;
        }
        FootSupport& GetFootSupport()
        {
            return m_FootSupport;
        }
        const FootSupport& GetFootSupport() const
        {
            return m_FootSupport;
        }
        const FootstepIKSnapshot& GetFootstepIKSnapshot() const
        {
            return m_FootstepIK.GetSnapshot();
        }
        const RecoverySnapshot& GetRecoverySnapshot() const
        {
            return m_Recovery.GetSnapshot();
        }
        const RecoverySettings& GetRecoverySettings() const
        {
            return m_RecoverySettings;
        }
        const StandingSupportSnapshot& GetStandingSupportSnapshot() const
        {
            return m_StandingSupport.GetSnapshot();
        }
        const std::vector<physx::PxVec3>& GetRecoveryReferenceHull() const
        {
            return m_RecoveryReferenceHull;
        }
        void SetFootCandidateDebugVisible(bool visible)
        {
            m_FootCandidateDebugVisible = visible;
        }
        bool IsFootCandidateDebugVisible() const
        {
            return m_FootCandidateDebugVisible;
        }
        bool UsesAutomaticRootSupport() const
        {
            return m_AutomaticRootSupport;
        }
        RootSupportRelease GetRootSupportRelease() const
        {
            return m_RootSupportRelease;
        }
        const std::string& GetPhysicsError() const
        {
            return m_PhysicsError;
        }
        const std::vector<GraphNode>& GetFinalPose() const
        {
            return IsRagdoll() ? m_PhysicsPose : m_AnimInstance->GetPose();
        }
        const ModelInstance& GetModelInstance() const
        {
            return m_ModelInstance;
        }

    private:
        ModelInstance m_ModelInstance;
        FootSupport m_FootSupport;
        std::unique_ptr<AnimInstance> m_AnimInstance;
        std::unique_ptr<const PhysicsAsset> m_PhysicsAsset;
        Ragdoll m_Ragdoll;
        FootstepIK m_FootstepIK;
        RagdollRecovery m_Recovery;
        StandingSupport m_StandingSupport;
        std::vector<physx::PxVec3> m_RecoveryReferenceHull;
        std::vector<GraphNode> m_RecoveryBlendSource;
        PhysicsSkeleton m_PhysicsSkeleton;
        std::vector<GraphNode> m_RagdollSourcePose;
        std::vector<GraphNode> m_PhysicsPose;
        std::vector<Math::Matrix4> m_BoneStretch;
        std::string m_PhysicsError;
        std::unique_ptr<const PhysicsAsset> m_PendingDriveSettings;
        RecoverySettings m_RecoverySettings;
        StandingSupportSettings m_StandingSupportSettings;
        std::array<physx::PxTransform, 2> m_RecoverySoles{
            physx::PxTransform(physx::PxIdentity), physx::PxTransform(physx::PxIdentity)};
        RootSupportRelease m_RootSupportRelease = RootSupportRelease::None;
        PhysicsAnimationMode m_PhysicsMode = PhysicsAnimationMode::Animation;
        bool m_HasRecoverySoles = false;
        bool m_FootCandidateDebugVisible = false;
        bool m_AutomaticRootSupport = false;
    };
}
