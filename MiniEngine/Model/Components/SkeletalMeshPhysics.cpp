#include "SkeletalMeshComponent.h"
#include "GameFramework/Actor.h"
#include "GameFramework/World.h"
#include <cmath>

using namespace Math;
namespace PARS
{
    bool SkeletalMeshComponent::ClearPhysicsAsset()
    {
        if (IsRagdoll())
        {
            return false;
        }
        DestroyPhysicsState();
        m_PhysicsAsset.reset();
        return true;
    }

    bool SkeletalMeshComponent::CreatePhysicsState(const PhysicsAsset& asset, Ragdoll& candidate,
        PhysicsSkeleton& skeleton, std::string& error)
    {
        auto* world = GetOwner().GetWorld();
        auto* physics = world ? world->GetPhysicsWorld() : nullptr;
        // Asset editing also uses this component in a world without physics.
        if (!physics || asset.bodies.empty())
        {
            error.clear();
            return true;
        }
        if (!BuildPhysicsSkeleton(skeleton, error))
        {
            return false;
        }
        std::vector<Matrix4> worldPose;
        if (!EvaluateAnimationPhysicsPose(skeleton, worldPose, error))
        {
            return false;
        }
        if (!candidate.Create(*physics, asset, skeleton.names, worldPose, Vector3(kZero), error, false))
        {
            return false;
        }
        candidate.SetQueryOwner(&GetOwner());
        return true;
    }

    bool SkeletalMeshComponent::EvaluateAnimationPhysicsPose(const PhysicsSkeleton& skeleton,
        std::vector<Matrix4>& worldPose, std::string& error) const
    {
        const Matrix4 component(GetComponentTransform()), rigid = PhysicsBoneFrame(component);
        const float differences[] = {float(Length(Vector3(component.GetX()) - Vector3(rigid.GetX()))),
            float(Length(Vector3(component.GetY()) - Vector3(rigid.GetY()))),
            float(Length(Vector3(component.GetZ()) - Vector3(rigid.GetZ())))};

        for (float difference : differences)
        {
            if (!std::isfinite(difference) || difference > .001f)
            {
                error = "PA bodies require a rigid, unit-scale component transform.";
                return false;
            }
        }
        if (!std::isfinite(float(Length(Vector3(component.GetW())))))
        {
            error = "Invalid PA component position.";
            return false;
        }

        return EvaluatePhysicsPose(skeleton, m_AnimInstance->GetPose(), component, worldPose, error);
    }

    void SkeletalMeshComponent::BeginPlay()
    {
        if (m_PhysicsAsset && !m_Ragdoll.IsActive())
        {
            CreatePhysicsState(*m_PhysicsAsset, m_Ragdoll, m_PhysicsSkeleton, m_PhysicsError);
        }
    }

    void SkeletalMeshComponent::DestroyPhysicsState()
    {
        m_Ragdoll.Destroy();
        m_PhysicsSkeleton = {};
        m_PhysicsPose.clear();
        m_RagdollSourcePose.clear();
        m_BoneStretch.clear();
        m_PhysicsError.clear();
        m_ModelInstance.ClearSkinnedWorldBounds();
        m_PhysicsMode = PhysicsAnimationMode::Animation;
        m_PendingDriveSettings.reset();
    }

    void SkeletalMeshComponent::UpdateKinematicPose(bool teleport)
    {
        if (!m_Ragdoll.IsActive() || IsRagdoll())
        {
            return;
        }
        std::vector<Matrix4> worldPose;
        if (EvaluateAnimationPhysicsPose(m_PhysicsSkeleton, worldPose, m_PhysicsError))
        {
            m_Ragdoll.UpdateKinematicPose(worldPose, teleport, m_PhysicsError);
        }
    }

    bool SkeletalMeshComponent::BuildPhysicsSkeleton(PhysicsSkeleton& skeleton, std::string& error) const
    {
        if (!m_SkeletalMesh)
        {
            error = "Assign a skeletal mesh first.";
            return false;
        }
        const auto& model = *m_SkeletalMesh;
        skeleton.names = model.m_NodeNames;
        skeleton.parents = model.m_NodeParents;
        skeleton.meshNodes.assign(model.m_NumNodes, -1);

        if (skeleton.names.size() != model.m_NumNodes || skeleton.parents.size() != model.m_NumNodes)
        {
            error = "Skeletal mesh is missing its named hierarchy.";
            return false;
        }

        std::vector<int> matrixNode(model.m_NumNodes, -1);

        for (uint32_t i = 0; i < model.m_NumNodes; ++i)
        {
            if (model.m_SceneGraph[i].matrixIdx < model.m_NumNodes)
            {
                matrixNode[model.m_SceneGraph[i].matrixIdx] = static_cast<int>(i);
            }
        }

        const uint8_t* meshData = model.m_MeshData.get();
        for (uint32_t i = 0; i < model.m_NumMeshes; ++i)
        {
            const Mesh& mesh = *reinterpret_cast<const Mesh*>(meshData);
            if (mesh.numJoints)
            {
                if (mesh.meshCBV >= matrixNode.size() || matrixNode[mesh.meshCBV] < 0 || uint32_t(mesh.startJoint) + mesh.numJoints > model.m_NumJoints)
                {
                    error = "Invalid skin mesh binding.";
                    return false;
                }
                for (uint32_t j = mesh.startJoint; j < uint32_t(mesh.startJoint) + mesh.numJoints; ++j)
                {
                    const auto node = model.m_JointIndices[j];
                    if (node >= model.m_NumNodes)
                    {
                        error = "Invalid skin joint index.";
                        return false;
                    }
                    auto& owner = skeleton.meshNodes[node];
                    if (owner >= 0 && owner != matrixNode[mesh.meshCBV])
                    {
                        error = "Ragdoll currently requires each joint to use one skin transform.";
                        return false;
                    }
                    owner = matrixNode[mesh.meshCBV];
                }
            }
            meshData += sizeof(Mesh) + (mesh.numDraws - 1) * sizeof(Mesh::Draw);
        }
        error.clear();
        return true;
    }

    bool SkeletalMeshComponent::StartRagdoll(Vector3 velocity, std::string& error)
    {
        return SetPhysicsMode(PhysicsAnimationMode::PassiveRagdoll, velocity, error);
    }

    bool SkeletalMeshComponent::SetPhysicsMode(PhysicsAnimationMode mode, Vector3 velocity, std::string& error)
    {
        if (mode == PhysicsAnimationMode::Animation)
        {
            StopRagdoll();
            error.clear();
            return true;
        }
        if (mode != PhysicsAnimationMode::PassiveRagdoll && mode != PhysicsAnimationMode::ActiveRagdoll)
        {
            error = "Unknown physics animation mode.";
            return false;
        }
        if (mode == m_PhysicsMode)
        {
            error.clear();
            return true;
        }
        if (!m_PhysicsAsset)
        {
            error = "Assign a PhysicsAsset first.";
            return false;
        }
        if (m_PhysicsAsset->bodies.empty())
        {
            error = "The PhysicsAsset has no bodies.";
            return false;
        }
        auto* world = GetOwner().GetWorld();
        auto* physics = world ? world->GetPhysicsWorld() : nullptr;
        if (!physics)
        {
            error = "The world has no physics scene.";
            return false;
        }
        if (!m_Ragdoll.IsActive() && !CreatePhysicsState(*m_PhysicsAsset, m_Ragdoll, m_PhysicsSkeleton, error))
        {
            return false;
        }
        const bool active = mode == PhysicsAnimationMode::ActiveRagdoll;
        if (active)
        {
            bool hasDrive = false;
            for (const auto& joint : m_PhysicsAsset->constraints)
            {
                hasDrive |= HasEnabledAngularDrive(joint);
            }
            if (!hasDrive)
            {
                error = "No enabled angular drives. Configure PA drives and apply them first.";
                return false;
            }
        }
        std::vector<Matrix4> worldPose;
        if (!EvaluateAnimationPhysicsPose(m_PhysicsSkeleton, worldPose, error))
        {
            return false;
        }
        if (active && !m_Ragdoll.UpdateDriveTargets(worldPose, 0, error))
        {
            return false;
        }
        if (!IsRagdoll())
        {
            if (!m_Ragdoll.UpdateKinematicPose(worldPose, true, error) || !m_Ragdoll.SetSimulating(true, velocity, error, active))
            {
                return false;
            }
            m_BoneStretch.clear();
            for (int node : m_Ragdoll.GetBoneNodes())
            {
                m_BoneStretch.push_back(Invert(PhysicsBoneFrame(worldPose[node])) * worldPose[node]);
            }
        }
        // Passive simulation releases the supported pelvis as well as the motors.
        // Keep the existing bodies and poses instead of re-entering from animation.
        if (!active && !m_Ragdoll.SetRootSupported(false, error))
        {
            return false;
        }
        m_RagdollSourcePose = m_AnimInstance->GetPose();
        m_PhysicsMode = mode;
        m_Ragdoll.SetDrivesEnabled(active);
        UpdatePhysicsPose();
        error.clear();
        return true;
    }

    bool SkeletalMeshComponent::QueueDriveSettings(const PhysicsAsset& source, std::string& error)
    {
        if (!m_PhysicsAsset)
        {
            error = "Assign a PhysicsAsset first.";
            return false;
        }
        if (!source.Validate(error))
        {
            return false;
        }
        auto candidate = std::make_unique<PhysicsAsset>(*m_PhysicsAsset);
        if (source.constraints.size() != candidate->constraints.size())
        {
            error = "Editor and character PA joints differ.";
            return false;
        }
        for (auto& target : candidate->constraints)
        {
            const int index = source.FindConstraint(target.parentBone, target.childBone);
            if (index < 0 || source.constraints[index].parentBone != target.parentBone)
            {
                error = "Editor and character joint parent/child mappings differ.";
                return false;
            }
            for (int axis = 0; axis < 3; ++axis)
            {
                target.drives[axis] = source.constraints[index].drives[axis];
            }
            target.useSlerpDrive = source.constraints[index].useSlerpDrive;
        }
        if (!candidate->Validate(error))
        {
            return false;
        }
        m_PendingDriveSettings = std::move(candidate);
        error.clear();
        return true;
    }

    void SkeletalMeshComponent::UpdatePhysicsPose()
    {
        const auto& source = IsActiveRagdoll() ? m_AnimInstance->GetPose() : m_RagdollSourcePose;
        if (IsActiveRagdoll())
        {
            std::vector<Matrix4> animated;
            if (EvaluateAnimationPhysicsPose(m_PhysicsSkeleton, animated, m_PhysicsError))
            {
                for (size_t i = 0; i < m_Ragdoll.GetBodyCount(); ++i)
                {
                    const int node = m_Ragdoll.GetBoneNodes()[i];
                    m_BoneStretch[i] = Invert(PhysicsBoneFrame(animated[node])) * animated[node];
                }
            }
        }
        std::vector<Matrix4> targets;
        for (size_t i = 0; i < m_Ragdoll.GetBodyCount(); ++i)
        {
            targets.push_back(m_Ragdoll.GetBoneWorld(i) * m_BoneStretch[i]);
        }
        OverridePhysicsPose(m_PhysicsSkeleton, source, Matrix4(GetComponentTransform()),
            m_Ragdoll.GetBoneNodes(), targets, m_PhysicsPose);
        // Culling must follow the falling mesh, rather than the frozen actor root.
        const Vector3 center = m_Ragdoll.GetPosition();
        float radius = 0;
        for (const auto& target : targets)
        {
            radius = Max(radius, float(Length(Vector3(target.GetW()) - center)));
        }
        m_ModelInstance.SetSkinnedWorldBounds(BoundingSphere(center, radius + float(GetLocalBounds().GetRadius())));
    }

    void SkeletalMeshComponent::StopRagdoll()
    {
        if (!IsRagdoll())
        {
            return;
        }
        m_Ragdoll.SetSimulating(false, Vector3(kZero), m_PhysicsError);
        m_PhysicsMode = PhysicsAnimationMode::Animation;
        m_PhysicsPose.clear();
        m_RagdollSourcePose.clear();
        m_BoneStretch.clear();
        m_ModelInstance.ClearSkinnedWorldBounds();
        UpdateKinematicPose(true);
    }
}
