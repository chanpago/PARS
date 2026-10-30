#include "SkeletalMeshComponent.h"
#include "GameFramework/Actor.h"
#include "GameFramework/World.h"
#include <algorithm>
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
        if (physics)
        {
            m_FootSupport.Update(*this, *physics);
        }
        else
        {
            m_FootSupport.Reset();
        }
        const auto& sample = m_FootSupport.GetSnapshot();
        if (!IsActiveRagdoll() || sample.contactRevision != m_Recovery.GetSnapshot().revision)
        {
            ResetRecovery();
        }
    }

    void SkeletalMeshComponent::ApplyFootstepIKTargets(std::vector<Matrix4>& target)
    {
        const auto& sample = m_FootSupport.GetSnapshot();
        const auto& step = sample.footstep;
        if (!IsActiveRagdoll() || !m_AutomaticRootSupport || m_RootSupportRelease != RootSupportRelease::None ||
            m_Recovery.GetSnapshot().state != RecoveryState::Recovery || !m_HasRecoverySoles)
        {
            m_FootstepIK.Reset();
            return;
        }
        const auto toPx = [](const Matrix4& matrix)
        {
            const auto frame = PhysicsJointFrame::FromTransform(PhysicsBoneFrame(matrix));
            return physx::PxTransform(physx::PxVec3(frame.offset.x, frame.offset.y, frame.offset.z),
                physx::PxQuat(frame.rotation.x, frame.rotation.y, frame.rotation.z, frame.rotation.w));
        };
        std::vector<physx::PxTransform> rigidTargets;
        rigidTargets.reserve(target.size());
        for (const auto& matrix : target)
        {
            rigidTargets.push_back(toPx(matrix));
        }
        FootstepIKInput input;
        input.enabled = true;
        input.stepFoot = step.IsInProgress() ? step.stepFoot : -1;
        input.revision = sample.contactRevision;
        const auto& nodes = m_Ragdoll.GetBoneNodes();
        for (size_t side = 0; side < input.legs.size(); ++side)
        {
            const auto& foot = sample.feet[side];
            const auto& setup = foot.leg;
            auto& leg = input.legs[side];
            leg.hipNode = setup.hipNode;
            leg.kneeNode = setup.kneeNode;
            leg.ankleNode = setup.ankleNode;
            leg.thighLength = setup.thighLength;
            leg.calfLength = setup.calfLength;
            leg.referenceBendInHip = setup.referenceBendInHip;
            if (!setup.available || !foot.available || setup.hipNode < 0 ||
                setup.kneeNode < 0 || setup.ankleNode < 0 ||
                size_t(setup.hipNode) >= target.size() || size_t(setup.kneeNode) >= target.size() ||
                size_t(setup.ankleNode) >= target.size() || m_PhysicsSkeleton.parents.size() != target.size())
            {
                continue;
            }
            // 세 본의 실제 바디와 연결된 가장 가까운 물리 부모가 있어야 관절 PD로 구동할 수 있다.
            if (std::find(nodes.begin(), nodes.end(), setup.hipNode) == nodes.end() ||
                std::find(nodes.begin(), nodes.end(), setup.kneeNode) == nodes.end() ||
                std::find(nodes.begin(), nodes.end(), setup.ankleNode) == nodes.end())
            {
                continue;
            }
            for (int parent = m_PhysicsSkeleton.parents[setup.hipNode]; parent >= 0; parent = m_PhysicsSkeleton.parents[parent])
            {
                const auto body = std::find(nodes.begin(), nodes.end(), parent);
                if (body != nodes.end())
                {
                    leg.anchorNode = parent;
                    leg.actualAnchor = toPx(m_Ragdoll.GetBoneWorld(size_t(body - nodes.begin())));
                    break;
                }
            }
            if (leg.anchorNode < 0 || !m_PhysicsAsset)
            {
                continue;
            }
            const auto linked = [&](int parent, int child)
            {
                return std::any_of(m_PhysicsAsset->constraints.begin(), m_PhysicsAsset->constraints.end(),
                    [&](const PhysicsConstraint& joint)
                    {
                        return joint.parentBone == m_PhysicsSkeleton.names[parent] &&
                            joint.childBone == m_PhysicsSkeleton.names[child];
                    });
            };
            if (!linked(leg.anchorNode, setup.hipNode) || !linked(setup.hipNode, setup.kneeNode) ||
                !linked(setup.kneeNode, setup.ankleNode))
            {
                continue;
            }
            leg.soleInBone = toPx(foot.soleInBone);
            // 지지발은 현재 접촉한 발바닥 자세를 목표로 삼는다. 바디를 월드에 고정하지 않는다.
            // Recovery 진입/착지 시 저장한 유효 자세를 접촉이 잠시 끊겨도 유지한다.
            leg.targetSole = m_RecoverySoles[side];
            leg.available = leg.targetSole.isSane();
        }
        if (!m_FootstepIK.Apply(input, m_PhysicsSkeleton.parents, rigidTargets))
        {
            // IK 입력이 부족하면 기본 애니메이션 목표를 유지하고 실패 사유를 디버그에 남긴다.
            // Recovery 호출자는 이 실패를 감지해 패시브 물리로 전환하므로 Idle 목표를 구동하지 않는다.
            return;
        }
        for (int node : m_FootstepIK.GetSnapshot().correctedNodes)
        {
            const auto& frame = rigidTargets[node];
            const Matrix4 corrected(OrthogonalTransform(
                Quaternion(DirectX::XMVectorSet(frame.q.x, frame.q.y, frame.q.z, frame.q.w)),
                Vector3(frame.p.x, frame.p.y, frame.p.z)));
            // 원래 본의 스케일·전단은 보존하고 강체 목표 프레임만 교체한다.
            target[node] = corrected * Invert(PhysicsBoneFrame(target[node])) * target[node];
        }
    }

    void SkeletalMeshComponent::UpdateFootCandidateDebugColors()
    {
        m_ModelInstance.ClearJointDebugColors();
        if (!m_FootCandidateDebugVisible)
        {
            return;
        }
        const auto& sample = m_FootSupport.GetSnapshot();
        const auto& candidates = sample.candidates;
        if (candidates.support < 0 || candidates.step < 0)
        {
            return;
        }
        const int supportNode = sample.feet[candidates.support].boneNode;
        const int stepNode = sample.feet[candidates.step].boneNode;
        if (supportNode >= 0 && stepNode >= 0)
        {
            m_ModelInstance.SetNodeDebugColor(uint32_t(supportNode), DirectX::XMFLOAT4(1, 0, 0, 1));
            m_ModelInstance.SetNodeDebugColor(uint32_t(stepNode), DirectX::XMFLOAT4(0, 0, 1, 1));
        }
    }

    bool SkeletalMeshComponent::SetAutomaticRootSupport(bool enabled, std::string& error)
    {
        if (enabled && !IsActiveRagdoll())
        {
            error = "Automatic root support requires active ragdoll mode.";
            return false;
        }
        if (enabled != m_AutomaticRootSupport)
        {
            m_RootSupportRelease = RootSupportRelease::None;
            m_FootSupport.ResetFootstep();
            ResetRecovery();
        }
        m_AutomaticRootSupport = enabled;
        if (enabled)
        {
            UpdateFootSupport();
            return UpdateAutomaticRootSupport(error);
        }
        error.clear();
        return true;
    }

    bool SkeletalMeshComponent::UpdateAutomaticRootSupport(std::string& error)
    {
        if (!m_AutomaticRootSupport)
        {
            return true;
        }
        const auto& balance = m_FootSupport.GetSnapshot().balance;
        if (m_RootSupportRelease == RootSupportRelease::None)
        {
            if (!IsActiveRagdoll())
            {
                m_RootSupportRelease = RootSupportRelease::Inactive;
            }
            else if (balance.zone == FootSupportZone::Invalid)
            {
                m_RootSupportRelease = RootSupportRelease::NoSupport;
            }
            else if (balance.weight <= 0)
            {
                m_RootSupportRelease = RootSupportRelease::OutsideBuffer;
            }
        }
        const float weight = m_RootSupportRelease == RootSupportRelease::None ? balance.weight : 0;
        if (m_Ragdoll.SetRootSupport(weight, false, error))
        {
            return true;
        }
        m_RootSupportRelease = RootSupportRelease::Error;
        std::string ignored;
        m_Ragdoll.SetRootSupported(false, ignored);
        return false;
    }

    void SkeletalMeshComponent::DestroyPhysicsState()
    {
        m_Ragdoll.Destroy();
        m_FootSupport.Reset();
        ResetRecovery();
        m_AutomaticRootSupport = false;
        m_RootSupportRelease = RootSupportRelease::None;
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
