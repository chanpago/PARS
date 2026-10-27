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
        // 에셋 편집 시에도 물리가 없는 월드에서 이 컴포넌트를 사용한다.
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

        // 본의 계층과 연결 정보, m_Pose: 애니메이션으로 계산한 로컬 자세, 메시 컴포넌트의 월드 변환, 결과: worldPose - 본별 월드 공간 목표 자세
        // 본의 로컬 자세에 부모 계층의 변환과 컴포넌트의 월드 변환을 반영해서, 각 본이 월드의 어디에 어떤 방향으로 있어야 하는지 계산
        return EvaluatePhysicsPose(skeleton, m_AnimInstance->GetPose(), component, worldPose, error);
    }

    Vector3 SkeletalMeshComponent::ConstrainHorizontalMovement(Vector3 displacement) const
    {
        std::vector<Matrix4> target;
        std::string error;
        if (!EvaluateAnimationPhysicsPose(m_PhysicsSkeleton, target, error))
        {
            return Vector3(kZero);
        }
        return m_Ragdoll.ConstrainHorizontalMovement(target, displacement);
    }

    void SkeletalMeshComponent::BeginPlay()
    {
        if (m_PhysicsAsset && !m_Ragdoll.IsActive())
        {
            CreatePhysicsState(*m_PhysicsAsset, m_Ragdoll, m_PhysicsSkeleton, m_PhysicsError);
        }
        UpdateFootSupport();
    }

    void SkeletalMeshComponent::UpdateFootSupport()
    {
        auto* world = GetOwner().GetWorld();
        auto* physics = world ? world->GetPhysicsWorld() : nullptr;
        if (physics) m_FootSupport.Update(*this, *physics);
        else m_FootSupport.Reset();
    }

    void SkeletalMeshComponent::DestroyPhysicsState()
    {
        m_Ragdoll.Destroy();
        m_FootSupport.Reset();
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
        // 패시브 시뮬레이션에서는 모터뿐 아니라 골반 지지도 해제한다.
        // 애니메이션에서 다시 진입하지 않고 기존 바디와 포즈를 유지한다.
        if (!active && !m_Ragdoll.SetRootSupported(false, error))
        {
            return false;
        }
        m_RagdollSourcePose = m_AnimInstance->GetPose();
        m_PhysicsMode = mode;
        m_Ragdoll.SetDrivesEnabled(active);
        UpdatePhysicsPose();
        UpdateFootSupport();
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
        // 컬링 영역은 멈춰 있는 Actor 루트가 아니라 떨어지는 메시를 따라가야 한다.
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
        UpdateFootSupport();
    }
}
