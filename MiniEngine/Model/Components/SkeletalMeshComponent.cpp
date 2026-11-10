#include "SkeletalMeshComponent.h"
#include "FramePerformance.h"

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
        m_FallingAnimationIndex = UINT32_MAX;
        m_StandingUpAnimationIndex = UINT32_MAX;
        m_StandingIdleAnimationIndex = UINT32_MAX;
        m_StandingUpStableTime = 0;
        m_ReturningFromStandingUp = false;
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
        m_Ragdoll.Swap(candidate);
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
        m_AnimInstance = std::move(instance);
        m_FallingAnimationIndex = UINT32_MAX;
        m_StandingUpAnimationIndex = UINT32_MAX;
        m_StandingIdleAnimationIndex = UINT32_MAX;
        m_StandingUpStableTime = 0;
        m_ReturningFromStandingUp = false;
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
    * æ÷¥œ∏ﬁ¿Ãº«¿ª ¡¯«‡«œ∞Ì, ∑°±◊µπ¿Ã µ˚∂Û∞• ∏Ò«• ¿⁄ºº∏¶ ∞ªΩ≈«—¥Ÿ
    */
    void SkeletalMeshComponent::FixedTickComponent(float deltaTime)
    {
        FramePerformance::Scope controlTiming(FramePerformance::Category::ParsControl);
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

        // ø©±‚º≠ πﬂ øÚ¡˜¿”¿ª ∞ËªÍ«œ∞Ì ¡ˆ¡ˆøµø™¿ª ¡∂ªÁ«ÿ ∞°¡ﬂƒ°∏¶ ±∏«—¥Ÿ
        UpdateFootSupport();

        //
        if (!UpdateAutomaticRagdoll(deltaTime))
        {
            return;
        }
        if (!UpdateAutomaticRootSupport(m_PhysicsError))
        {
            return;
        }

        if (m_PhysicsMode == PhysicsAnimationMode::PassiveRagdoll)
        {
            return;
        }

        // ÏßÄÏßÄ Ìï¥Ï†ú ÌåêÏ†ïÏù¥ ÎÅùÎÇú Í∞ôÏùÄ Î¨ºÎ¶¨ Ïä§ÌÖùÏóêÏÑú ÎÇôÏÉÅ ÌÅ¥Î¶ΩÏùÑ ÏÑ†ÌÉùÌïúÎã§.
        const bool fallingTargetReady = UpdateFallingAnimation(deltaTime);

        // ƒ≥∏Ø≈Õ ¿Ãµø¿Ã ƒƒ∆˜≥Õ∆Æ ∑Á∆Æ∏¶ ¿ÃπÃ ∞ªΩ≈«— ªÛ≈¬¥Ÿ. simulate() ¿¸ø° æ÷¥œ∏ﬁ¿Ãº«¿ª
        // ª˘«√∏µ«ÿ PVDøÕ ∑ª¥ı∏µ¿Ã ∏µŒ ¿Ã ∆˜¡Ó∏¶ ªÁøÎ«œµµ∑œ «—¥Ÿ.
        // ¿Ãπˆ Ω∫≈‹ø°º≠¿« æ÷¥œ∏ﬁ¿Ãº« ¿⁄ºº∏¶ ±∏«—¥Ÿ.
        // æ÷¥œ∏ﬁ¿Ãº« ¿Áª˝ Ω√∞£¿ª ¡¯«‡Ω√≈∞∞Ì, ±◊ Ω√∞£ø° «ÿ¥Á«œ¥¬ ∫ªµÈ¿« ¿⁄ºº∏¶ ∞ËªÍ
        // clipø°º≠ ≈∞«¡∑π¿” µ•¿Ã≈ÕøÕ Ω√∞£ ∞£∞› ¡§∫∏∏¶ ¿–æÓøÕº≠ µŒ «¡∑π¿” ªÁ¿Ã¿« ∫Ò¿≤¿ª ∞ËªÍ«œ∞Ì m_poseø° ¿˙¿Â
        m_AnimInstance->UpdateAnimation(deltaTime);

        if (IsActiveRagdoll())
        {
            std::vector<Math::Matrix4> target;

            // ¿Ã ∫Œ∫–ø°º≠ æ◊∆º∫Í ∑°±◊µπ π∞∏Æ√≥∏Æ∏¶«—¥Ÿ(pd±∏µø π◊ √ﬂ∞°∫∏ªÛ≈‰≈©∞ËªÍ)
			// EvaluateAnimationPhysicsPose∫Œ∫–ø°º≠ ¿Ãπ¯ «¡∑π¿”¿« æ÷¥œ∏ﬁ¿Ãº«¿« ∏Ò«• ∫ª ¿⁄ºº∏¶ ø˘µÂ ±‚¡ÿ¿∏∑Œ ∞ËªÍ«œ∞Ì targetø° ¿˙¿Â«—¥Ÿ.
            // 
            // UpdateDriveTargets∫Œ∫–¿∫ ¡ª ±Ê¥Ÿ º¯º≠¥Î∑Œ º≥∏Ì«œ∏È,
            // 1. æ’º≠ ∞ËªÍ«— target(æ÷¥œ∏ﬁ¿Ãº« ∏Ò«• ∫ª ¿⁄ºº)¿ª ±‚π›¿∏∑Œ ¿Ãπ¯ «¡∑π¿”¿« ∏Æ¡ˆµÂπŸµ¿« ∏Ò«• ¿⁄ºº∏¶ ∞ËªÍ«—¥Ÿ.
            // 2. 1ø°º≠ ∞ËªÍ«— ∏Æ¡ˆµÂ πŸµ¿« ∏Ò«• ¿⁄ºº∏¶ ±‚π›¿∏∑Œ ∞¢ ¡∂¿Œ∆Æ¿« ∏Ò«• ¿⁄ºº∏¶ ∞ËªÍ
            // 3. ∞Òπ› ∫∏¡§¿ª Ω««‡«—¥Ÿ. ¿ßƒ°, º”µµ
            // 4. targetsø° ¿÷¥¬ ∫Œ∏±‚¡ÿ ªÛ¥Î∞¢ ¬˜¿Ã ƒı≈Õ¥œæ¿ª physx∞° ¿–¿ª ºˆ ¿÷µµ∑œ vec3±‚π› ∞¢º”µµ∫§≈Õ∑Œ ∏∏µÁ¥Ÿ.
            // 5. Articulation Drive update <- ¿Ã ∫Œ∫–¿Ã ¡ﬂø‰«œ¥Ÿ
            //      - ∞Òπ›∫∏¡§
            //      - ≈∏∞Ÿ¿ª ∏Ò«•∑Œ ¿Ø»ø∞¸º∫¿ª ∞ËªÍ«œ∞Ì ¡ﬂ∑¬∫∏ªÛ, ƒ⁄∏Æø√∏Æ-ø¯Ω…∑¬ ∫∏ªÛ¿ª ≈‰≈©∞™ø° ¥ı«‘
            //      - ∞¢ ∞¸¿˝¿« ∞¢ √‡ø° æÛ∏∂∏∏≈≠¿« PD∏¶ ¿˚øÎΩ√≈≥¡ˆ ∞ËªÍ
            bool evaluated = fallingTargetReady && EvaluateAnimationPhysicsPose(m_PhysicsSkeleton, target, m_PhysicsError);
            if (evaluated)
            {
                evaluated = ApplyRecoveryTargets(target, deltaTime);
            }
            if (!evaluated || !m_Ragdoll.UpdateDriveTargets(target, deltaTime, m_PhysicsError))
            {
                // ∏Ò«• ∞ªΩ≈ø° Ω«∆–«œ∏È ¿¸Ω≈ ∆–Ω√∫Í π∞∏Æ Ω√πƒ∑π¿Ãº«¿∏∑Œ ¿¸»Ø«—¥Ÿ.
                // ∞Òπ›¿« »˚/≈‰≈© ¡ˆ¡ˆ∏¶ ¡¶∞≈«œ∏Èº≠ ∏Ò«• ∞ªΩ≈ ø¿∑˘ ∏ﬁΩ√¡ˆ¥¬ ∫∏¡∏«—¥Ÿ.
                m_FootstepIK.Reset();
                m_Ragdoll.SetDrivesEnabled(false);
                std::string supportError;
                m_Ragdoll.SetRootSupported(false, supportError);
                m_PhysicsMode = PhysicsAnimationMode::PassiveRagdoll;
            }
        }
        else
        {
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
        return m_ModelInstance.UpdatePose(context, IsRagdoll() || IsBlendingToKinematic() ? GetFinalPose() :
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
