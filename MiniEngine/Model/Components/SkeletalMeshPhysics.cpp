#include "SkeletalMeshComponent.h"
#include "GameFramework/Actor.h"
#include "GameFramework/World.h"
#include <algorithm>
#include <cmath>
#include <functional>

using namespace Math;
using namespace physx;
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
        PARS::World* world = GetOwner().GetWorld();
        PhysicsWorld* physics = world ? world->GetPhysicsWorld() : nullptr;
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
        PARS::World* world = GetOwner().GetWorld();
        PhysicsWorld* physics = world ? world->GetPhysicsWorld() : nullptr;
        if (physics)
        {
            /*
                - COM 정보: 현재 질량중심, 전체 질량, COM 속도.
                - 발 정보: 실제 접촉 상태, 발바닥 위치, 레이로 얻은 지면 투영점.
                - 균형 판단: A 영역, COM의 지면 투영과 A 내부 여부, A/B 기준 골반 지지계수 weight.
                - 발딛기 판단: 지지발·움직일 발 후보, 예측 COM, 발딛기 필요 여부, 착지 목표와 발딛기 진행 상태.
            */
            m_FootSupport.Update(*this, *physics);
        }
        else
        {
            m_FootSupport.Reset();
        }
        const PARS::FootSupportSnapshot& sample = m_FootSupport.GetSnapshot();
        if (!IsActiveRagdoll() || sample.contactRevision != m_Recovery.GetSnapshot().revision)
        {
            ResetRecovery();
        }
    }


    // target은 본 노드별 월드 공간 목표 행렬 배열 
    // 참조로 받으르모 함수 마지막에서 이 배열의 다리관련 행렬을 수정
    void SkeletalMeshComponent::ApplyFootstepIKTargets(std::vector<Matrix4>& target, float deltaTime)
    {
        // IK를 실행할 수 있는 상태인지 확인. 발 지지 정보와 발딛기 상태를 가져옴 
        const FootSupportSnapshot& sample = m_FootSupport.GetSnapshot();
        const FootstepSnapshot& step = sample.footstep;

        if (!IsActiveRagdoll() || !m_AutomaticRootSupport || m_RootSupportRelease != RootSupportRelease::None ||
            (!m_Ragdoll.UsesFootDrivenSupport() && m_Recovery.GetSnapshot().state != RecoveryState::Recovery) ||
            !m_HasRecoverySoles)
        {
            m_FootstepIK.Reset();
            return;
        }

        // toPx라는 람다함수를 정의. 위치와 쿼터니언을 분리해서 PxTransform으로 변환.
        const std::function<PxTransform(const Matrix4&)> toPx = [](const Matrix4& matrix) -> PxTransform
        {
            const PhysicsJointFrame frame = PhysicsJointFrame::FromTransform(PhysicsBoneFrame(matrix));
            return physx::PxTransform(physx::PxVec3(frame.offset.x, frame.offset.y, frame.offset.z),
                physx::PxQuat(frame.rotation.x, frame.rotation.y, frame.rotation.z, frame.rotation.w));
        };
        std::vector<physx::PxTransform> rigidTargets;
        rigidTargets.reserve(target.size());
        for (const Matrix4& matrix : target)
        {
            rigidTargets.push_back(toPx(matrix));
        }

        // IK와 서있기 보정의 공통입력을 준비함
        FootstepIKInput input;
        input.enabled = true;
        input.stepFoot = step.IsInProgress() ? step.stepFoot : -1;
        input.revision = sample.contactRevision;


        const std::vector<int>& nodes = m_Ragdoll.GetBoneNodes();
        StandingSupportInput standing;
        standing.enabled = m_Ragdoll.UsesFootDrivenSupport();
        standing.stepping = step.IsInProgress();
        standing.stepDistance = step.initialDistance;
        standing.deltaTime = deltaTime;
        standing.revision = sample.contactRevision;
        standing.completedStep = sample.completedPhysicsStep;
        standing.velocity = sample.hasCenterOfMassVelocity ? sample.centerOfMassVelocity : PxVec3(0);
        std::array<RagdollSupportContact, 2> contacts;

		// 다리별 IK 입력을 준비하고, 서있기 보정 입력과 접촉 정보를 준비함
        for (size_t side = 0; side < input.legs.size(); ++side)
        {
            /*
            hipsNode : 허벅지가 시작하는 본
			kneeNode : 무릎이 시작하는 본
			ankleNode : 발목이 시작하는 본
			thighLength : 허벅지 길이
			caalfLength : 종아리 길이
			referenceBendInHip : 허벅지 본 기준의 기준 무릎 굽힘 방향
            */
            const FootSoleSample& foot = sample.feet[side];
            const FootLegSetup& setup = foot.leg;
            FootIKLegInput& leg = input.legs[side];
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
            // 다리를 지탱하는 실제 물리 부모를 찾음
            for (int parent = m_PhysicsSkeleton.parents[setup.hipNode]; parent >= 0; parent = m_PhysicsSkeleton.parents[parent])
            {
                const std::vector<int>::const_iterator body = std::find(nodes.begin(), nodes.end(), parent);
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
            const std::function<bool(int, int)> linked = [&](int parent, int child) -> bool
            {
                return std::any_of(m_PhysicsAsset->constraints.begin(), m_PhysicsAsset->constraints.end(),
                    [&](const PhysicsConstraint& joint) -> bool
                    {
                    return joint.parentBone == m_PhysicsSkeleton.names[parent] &&
                        joint.childBone == m_PhysicsSkeleton.names[child];
                });
            };
            // PA에 힙, 허벅지, 무릎, 발목이 연결되어 있는지 확인
            if (!linked(leg.anchorNode, setup.hipNode) || !linked(setup.hipNode, setup.kneeNode) ||
                !linked(setup.kneeNode, setup.ankleNode))
            {
                continue;
            }

            // 다리별로 기본 발바닥목표를 설정
            leg.soleInBone = toPx(foot.soleInBone);
            // 지지발은 현재 접촉한 발바닥 자세를 목표로 삼는다. 바디를 월드에 고정하지 않는다.
            // Recovery 진입/착지 시 저장한 유효 자세를 접촉이 잠시 끊겨도 유지한다.
            leg.targetSole = m_RecoverySoles[side];
            leg.available = leg.targetSole.isSane();

            // 서있기 보정에 필요한 발,골반 관계를 계산
            StandingSupportFoot& support = standing.feet[side];
            support.available = leg.available;

            // 지지역할을 하는발과 실제 접지를 구분함
            support.supporting = input.stepFoot != static_cast<int>(side);
            support.grounded = foot.contact.hasSample && foot.contact.grounded;


            support.sole = leg.targetSole;
            support.soleInBone = leg.soleInBone;

            // 입력 목표 포즈의 발바닥을 계산
            support.animatedSole = rigidTargets[leg.ankleNode] * leg.soleInBone;

            // IK좌표 변환의 핵심
            /*
			일단 기본적으로 목표(애니메이션 트랙데이터) 를 실제 물리 바디의 부모 좌표계로 쉽게 변환하기 위해 쓰는 toPhysical 변환이다.
            목표 고관절의 월드변환은 목표 루트로부터 쭉 곱해지는데 여기서 행렬의 결합법칙이 작용하므로 
			목표 고관절의 월드변환 = 목표 루트 * ... * 목표 고관절 부모 * 목표 고관절
			물리 고관절의 월드변환 = toPhysical * 목표 루트 * ... * 목표 고관절
								= 물리 루트 * ... * 물리 고관절 부모 * 물리 고관절
            그냥 목표로 적용되어있던 관절을 루트만 바꿔서 물리로 바꿔주기위한 도움 변수 행렬이다 
            */
            const PxTransform toPhysical = leg.actualAnchor * rigidTargets[leg.anchorNode].getInverse();

            // 물리 엉덩이본과 물리 골반이랑 위치차이를 구한다.
            support.hipOffset = (toPhysical * rigidTargets[leg.hipNode]).p - leg.actualAnchor.p;
            support.length = leg.thighLength + leg.calfLength;
            standing.pelvis = leg.actualAnchor;
            standing.animatedPelvis = rigidTargets[leg.anchorNode];
            const std::vector<int>::const_iterator body = std::find(nodes.begin(), nodes.end(), foot.boneNode);

            // 실제 지지 접촉을 래그돌에 전달
            // 각 발에 대해 물리 바디 인덱스, 접촉 위치 법선, 지지여부를 구성. 
            contacts[side].body = body != nodes.end() ? static_cast<int>(body - nodes.begin()) : -1;
            contacts[side].supporting = support.supporting && support.grounded;
            contacts[side].position = foot.contact.position;
            contacts[side].normal = foot.contact.normal;
        }

        // 지지 역할 이면서 실제 접지한 발만 지지 접촉으로 전달
        m_Ragdoll.SetSupportContacts(contacts);

        // 발 기반 지지에서는 기본 발 목표에 서 있기 보정을 더함
        if (standing.enabled)
        {
            /*
            m_StandingSupport.Update는 다음을 고려한다.
            - 애니메이션이 요구하는 골반의 발 위 높이
            - 발딛기 중 필요한 골반 낮추기
            - 지지할 발의 접촉이 끊겼을때 재접촉을 위한 낮추기
            - 다리 기링와 수평 거리로 결정되는 도달 범위
            - 현재 골반의 수평 위치와 COM 속도
            */

            // 골반이 유지할 목표 높이와 수평 보정량을 계산하고 이를 지지발의 IK목표에 반영하는함수
            if (!m_StandingSupport.Update(standing, m_StandingSupportSettings))
            {
                m_FootstepIK.Reset();
                return;
            }

            // 이번 IK입력의 발 목표를 교체
            const StandingSupportSnapshot& support = m_StandingSupport.GetSnapshot();
            for (size_t side = 0; side < input.legs.size(); ++side)
            {
                input.legs[side].targetSole = support.soleTargets[side];
            }
        }

        // 실제 양쪽 다리 IK를 계산
        if (!m_FootstepIK.Apply(input, m_PhysicsSkeleton.parents, rigidTargets))
        {
            // IK 입력이 부족하면 기본 애니메이션 목표를 유지하고 실패 사유를 디버그에 남긴다.
            // Recovery 호출자는 이 실패를 감지해 패시브 물리로 전환하므로 Idle 목표를 구동하지 않는다.
            return;
        }

		// IK 결과를 원래 target에 반영. correctedNodesd 에 있는 본만 갱신
        for (int node : m_FootstepIK.GetSnapshot().correctedNodes)
        {
            const PxTransform& frame = rigidTargets[node];
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
        const FootSupportSnapshot& sample = m_FootSupport.GetSnapshot();
        const FootstepRoles roles = sample.GetDisplayRoles();
        if (!roles.IsValid())
        {
            return;
        }
        const int supportNode = sample.feet[roles.supportFoot].boneNode;
        const int stepNode = sample.feet[roles.movableFoot].boneNode;
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
        if (!enabled)
        {
            m_Ragdoll.SetFootDrivenSupport(false);
        }
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
        const FootSupportBalance& balance = m_FootSupport.GetSnapshot().balance;
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
        const bool idle = !m_AnimInstance->IsBlending() &&
            m_AnimInstance->GetAnimationName(m_AnimInstance->GetCurrentAnimationIndex()) == "Idle";
        m_Ragdoll.SetFootDrivenSupport(IsActiveRagdoll() && idle && m_RootSupportRelease == RootSupportRelease::None);
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
        const Model& model = *m_SkeletalMesh;
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
                    const uint16_t node = model.m_JointIndices[j];
                    if (node >= model.m_NumNodes)
                    {
                        error = "Invalid skin joint index.";
                        return false;
                    }
                    int32_t& owner = skeleton.meshNodes[node];
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
        PARS::World* world = GetOwner().GetWorld();
        PhysicsWorld* physics = world ? world->GetPhysicsWorld() : nullptr;
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
            for (const PARS::PhysicsConstraint& joint : m_PhysicsAsset->constraints)
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
        std::unique_ptr<PARS::PhysicsAsset> candidate = std::make_unique<PhysicsAsset>(*m_PhysicsAsset);
        if (source.constraints.size() != candidate->constraints.size())
        {
            error = "Editor and character PA joints differ.";
            return false;
        }
        for (PARS::PhysicsConstraint& target : candidate->constraints)
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
        const std::vector<GraphNode>& source = IsActiveRagdoll() ? m_AnimInstance->GetPose() : m_RagdollSourcePose;
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
        for (const Math::Matrix4& target : targets)
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
