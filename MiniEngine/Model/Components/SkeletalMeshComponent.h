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
        // Assign assets before rendering starts (or while the GPU is idle).
        bool SetSkeletalMesh(std::shared_ptr<const Model> mesh) override;
        bool SetAnimInstance(std::unique_ptr<AnimInstance> instance);
        AnimInstance& GetAnimInstance() const
        {
            return *m_AnimInstance;
        }
        // Validated snapshot. A playing physics world registers kinematic bodies immediately.
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
        // Copy only drive values by joint names. Applied before the next fixed step.
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

        // Optional final pose override, owned by the caller (e.g. an editor preview).
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
