#include "SkeletalMeshComponent.h"
#include "FramePerformance.h"
#include "GameFramework/Actor.h"
#include "GameFramework/World.h"
#include "PhysicsWorld.h"
#include "../FootstepPoseValidation.h"
#include <algorithm>
#include <chrono>
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
        candidate.SetArmDriveStrength(m_ArmDriveStrength);
        return true;
    }

    bool SkeletalMeshComponent::SetArmDriveStrength(float strength)
    {
        if (!m_Ragdoll.SetArmDriveStrength(strength))
        {
            return false;
        }
        m_ArmDriveStrength = strength;
        return true;
    }

    bool SkeletalMeshComponent::EvaluateAnimationPhysicsPose(const PhysicsSkeleton& skeleton,
        std::vector<Matrix4>& worldPose, std::string& error, const std::vector<GraphNode>* poseOverride) const
    {
        const Matrix4 component(GetComponentTransform());
        const Matrix4 rigid = PhysicsBoneFrame(component);
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
        return EvaluatePhysicsPose(skeleton, poseOverride ? *poseOverride : m_AnimInstance->GetPose(), component, worldPose, error);
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
                - 발딛기 판단: 예측 COM, 발딛기 필요 여부, 발별 명령 대상·착지 목표·진행 상태.
            */
            // 현재 몸과 발의 상태를 읽고, 균형 판단과 발딛기 계획을 m_Snapshot에 갱신하는함수
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
        const bool fallingSteps = step.backwardFallStepsRemaining > 0;
        const bool hasRecoveryTargets = m_RootSupportRelease == RootSupportRelease::None && m_HasRecoverySoles &&
            (m_Ragdoll.UsesFootDrivenSupport() || m_Recovery.GetSnapshot().state == RecoveryState::Recovery);

        if (!IsActiveRagdoll() || !m_AutomaticRootSupport || (!fallingSteps && !hasRecoveryTargets))
        {
            m_FootstepIK.Reset();
            m_FootstepAvoidance.Reset();
            return;
        }

        FramePerformance::Scope ikTiming(FramePerformance::Category::FootstepIK);
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
        input.revision = sample.contactRevision;


        const std::vector<int>& nodes = m_Ragdoll.GetBoneNodes();
        StandingSupportInput standing;
        standing.enabled = m_Ragdoll.UsesFootDrivenSupport();
        standing.stepping = step.IsInProgress();
        standing.stepDistance = step.GetMaximumInitialDistance();
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
            const FootstepCommand& command = step.feet[side];
            const PxTransform actualSole = toPx(foot.soleWorld);
            leg.targetSole = fallingSteps ? actualSole : m_RecoverySoles[side];
            if (fallingSteps && command.motion.valid)
            {
                const PxVec3 position = command.motion.hasSupportDeadline ?
                    SampleFootstepPosition(command, deltaTime) : command.motion.position;
                if (!MakeFootSoleTarget(position, command.target.normal, actualSole, leg.targetSole,
                    command.target.position - command.startPosition))
                {
                    continue;
                }
            }
            leg.available = leg.targetSole.isSane();

            // 서있기 보정에 필요한 발,골반 관계를 계산
            StandingSupportFoot& support = standing.feet[side];
            support.available = leg.available;

            // 명령 여부와 접촉 측정은 독립적이며 제어에 요구할 하중은 아래에서 별도로 정한다.
            support.hasMotionTarget = step.feet[side].IsInProgress();
            input.motionTargets[side] = support.hasMotionTarget;
            support.grounded = foot.contact.hasSample && foot.contact.grounded;
            support.contactGraceActive = foot.contact.hasSample && !support.grounded && foot.contact.support.IsValid();


            support.sole = leg.targetSole;
            support.soleInBone = leg.soleInBone;
            support.actualSole = actualSole;
            support.supportNormal = foot.contact.support.GetNormal();

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
            if (fallingSteps && !support.hasMotionTarget && !support.grounded)
            {
                leg.targetSole = toPhysical * support.animatedSole;
            }

            // 물리 엉덩이본과 물리 골반이랑 위치차이를 구한다.
            support.hipOffset = (toPhysical * rigidTargets[leg.hipNode]).p - leg.actualAnchor.p;
            support.length = leg.thighLength + leg.calfLength;
            standing.pelvis = leg.actualAnchor;
            standing.animatedPelvis = rigidTargets[leg.anchorNode];
            const std::vector<int>::const_iterator body = std::find(nodes.begin(), nodes.end(), foot.boneNode);

            // 하중 보상에는 실제 접지한 발과 그 지면을 전달한다.
            contacts[side].body = body != nodes.end() ? static_cast<int>(body - nodes.begin()) : -1;
            contacts[side].grounded = support.grounded;
            // 접촉 측정은 유지하되 이륙 명령에 몸을 지탱할 하중 목표를 함께 부여하지 않는다.
            contacts[side].loadWeight = support.hasMotionTarget ? 0.0f : 1.0f;
            contacts[side].position = foot.contact.support.GetPosition();
            contacts[side].normal = foot.contact.support.GetNormal();
            contacts[side].ankleDriveBoost = m_StandingSupportSettings.ankleDriveBoost;
            contacts[side].ankleDriveBlendTime = m_StandingSupportSettings.ankleDriveBlendTime;
            if (support.hasMotionTarget && command.motion.hasSupportDeadline && deltaTime > 0)
            {
                contacts[side].swingNodes = {{setup.hipNode, setup.kneeNode, setup.ankleNode}};
                // 위치와 속도를 함께 제어할 수 있는 두 물리 틱을 게인 계산의 최소 시간으로 둔다.
                contacts[side].inverseLandingTime = 1.0f / (std::max)(GetFootstepRemainingTime(command), 2.0f * deltaTime);
            }
        }

        // 실제 접촉만 전달한다. 이동 명령과 접촉 유예로 하중을 만들어 내지 않는다.
        if (!fallingSteps)
        {
            m_Ragdoll.SetSupportContacts(contacts);
        }

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
                m_FootstepAvoidance.Reset();
                return;
            }

            // 이번 IK입력의 발 목표를 교체
            const StandingSupportSnapshot& support = m_StandingSupport.GetSnapshot();
            for (size_t side = 0; side < input.legs.size(); ++side)
            {
                input.legs[side].targetSole = support.soleTargets[side];
            }
        }

        UpdateFootstepCollision(input, rigidTargets, deltaTime);

        // 선택한 경로의 현재 발 목표로 실제 양쪽 다리 IK를 계산한다.
        if (!m_FootstepIK.Apply(input, m_PhysicsSkeleton.parents, rigidTargets))
        {
            m_FootstepAvoidance.Reset();
            m_FootstepCollision.status = FootstepCollisionStatus::Unavailable;
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

    void SkeletalMeshComponent::UpdateFootstepCollision(FootstepIKInput& input,
        const std::vector<PxTransform>& sourcePose, float deltaTime)
    {
        const FootstepSnapshot& step = m_FootSupport.GetSnapshot().footstep;
        if (!m_FootSupport.GetFootstepSettings().pathAvoidanceEnabled || !step.IsInProgress())
        {
            m_FootstepAvoidance.Reset();
            return;
        }
        FramePerformance::Scope pathTiming(FramePerformance::Category::TrajectorySetup);
        const std::chrono::steady_clock::time_point started = std::chrono::steady_clock::now();
        FootstepCollisionInput prediction;
        prediction.completedStep = step.completedStep;
        prediction.revision = step.revision;
        prediction.fixedDeltaTime = deltaTime;
        const std::vector<int>& nodes = m_Ragdoll.GetBoneNodes();
        for (size_t side = 0; side < prediction.legs.size(); ++side)
        {
            FootstepCollisionLeg& leg = prediction.legs[side];
            leg.command = step.feet[side];
            const FootIKLegInput& source = input.legs[side];
            // 실제 IK와 동일한 원본 자세·부모 좌표·이전 무릎 힌트로 후보를 평가한다.
            if (!m_FootstepIK.BuildLegInput(input, side, sourcePose, leg.pose))
            {
                m_FootstepAvoidance.Reset();
                m_FootstepCollision.status = FootstepCollisionStatus::Unavailable;
                return;
            }
            const std::array<int, 3> legNodes{source.hipNode, source.kneeNode, source.ankleNode};
            for (size_t part = 0; part < legNodes.size(); ++part)
            {
                const std::vector<int>::const_iterator body = std::find(nodes.begin(), nodes.end(), legNodes[part]);
                if (body == nodes.end())
                {
                    continue;
                }
                FootstepCollisionShape& shape = leg.shapes[part];
                shape.available = m_Ragdoll.GetBodyCollisionShape(static_cast<size_t>(body - nodes.begin()),
                    shape.geometry, shape.inBone, shape.actualPose);
            }
        }
        prediction.acceptsPose = [this, &input](size_t side, const LegIKResult& solved) -> bool
        {
            const FootIKLegInput& leg = input.legs[side];
            const std::array<int, 4> chain{leg.anchorNode, leg.hipNode, leg.kneeNode, leg.ankleNode};
            const std::array<PxTransform, 4> frames{leg.actualAnchor, solved.hip, solved.knee, solved.ankle};
            std::array<int, 4> bodies;
            for (size_t index = 0; index < chain.size(); ++index)
            {
                const int body = m_PhysicsAsset->FindBody(m_PhysicsSkeleton.names[chain[index]]);
                if (body < 0)
                {
                    return false;
                }
                bodies[index] = body;
            }
            return CheckFootstepJointLimits(*m_PhysicsAsset, m_Ragdoll.UsesArticulation(), bodies, frames) == FootstepPoseFailure::None;
        };
        const Vector3 actorForward = GetOwner().GetActorTransform().GetBasis() * Vector3(0, 0, 1);
        const PxVec3 forward(float(actorForward.GetX()), float(actorForward.GetY()), float(actorForward.GetZ()));
        m_FootstepAvoidance.Update(prediction, forward);
        const FootstepAvoidanceSnapshot& avoidance = m_FootstepAvoidance.GetSnapshot();
        m_FootstepCollision = avoidance.collision;
        for (size_t side = 0; side < input.legs.size(); ++side)
        {
            if (step.feet[side].IsInProgress())
            {
                const FootstepCommand& command = step.feet[side];
                const float driveTimeAhead = command.motion.hasSupportDeadline ? deltaTime : 0.0f;
                input.legs[side].targetSole.p += SampleFootstepPath(avoidance.paths[side],
                    GetPredictedFootstepFlightTime(command, driveTimeAhead));
            }
        }
        m_FootstepCollision.elapsedMilliseconds = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - started).count();
        for (size_t side = 0; side < step.feet.size(); ++side)
        {
            if (step.feet[side].IsInProgress())
            {
                m_FootSupport.ReportRecoveryPath(side, step.feet[side].target.position,
                    m_FootstepCollision.status == FootstepCollisionStatus::Clear &&
                    !m_FootstepCollision.reachLimited && !m_FootstepCollision.jointLimited);
            }
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
        for (size_t side = 0; side < sample.feet.size(); ++side)
        {
            const FootSoleSample& foot = sample.feet[side];
            if (foot.boneNode < 0)
            {
                continue;
            }
            const bool commanded = sample.footstep.feet[side].IsInProgress();
            const bool grounded = foot.contact.hasSample && foot.contact.grounded;
            const DirectX::XMFLOAT4 color(commanded ? 0.0f : .4f, grounded ? 1.0f : .4f,
                commanded ? 1.0f : .4f, 1.0f);
            m_ModelInstance.SetNodeDebugColor(static_cast<uint32_t>(foot.boneNode), color);
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
            m_ImpactWarmup = {};
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
        if (!m_AutomaticRootSupport || (m_AutomaticRagdoll && !IsRagdoll()))
        {
            return true;
        }
        const FootSupportSnapshot& sample = m_FootSupport.GetSnapshot();
        const FootSupportBalance& balance = sample.balance;
        const bool idle = !m_AnimInstance->IsBlending() &&
            m_AnimInstance->GetAnimationName(m_AnimInstance->GetCurrentAnimationIndex()) == "Idle";
        if (IsFalling())
        {
            m_RootSupportRelease = RootSupportRelease::NoSupport;
        }
        else if (m_RootSupportRelease == RootSupportRelease::None)
        {
            if (!IsActiveRagdoll())
            {
                m_RootSupportRelease = RootSupportRelease::Inactive;
            }
            else if (balance.zone == FootSupportZone::Invalid)
            {
                m_RootSupportRelease = RootSupportRelease::NoSupport;
            }
            else if (sample.footstep.HasTimedOutOutsideBuffer())
            {
                m_RootSupportRelease = RootSupportRelease::OutsideBuffer;
            }
            else if ((sample.footstep.needsStep || balance.zone != FootSupportZone::Inside) && m_Ragdoll.IsTopplingFromProjectile())
            {
                m_RootSupportRelease = RootSupportRelease::ProjectileImpact;
            }
        }
        float weight = 0;
        if (m_RootSupportRelease == RootSupportRelease::None)
        {
            weight = balance.weight;
            if (balance.zone == FootSupportZone::Outside)
            {
                // B 밖에서도 직전 보조 강도로 회복 스텝을 이어간다. 첫 관측부터 밖이면 기본 강도로 시작한다.
                weight = m_Ragdoll.IsRootSupported() ? m_Ragdoll.GetRootSupportWeight() : 1.0f;
            }
        }
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
        ResetAirborneFall();
        ResetFallArms();
        m_FallArmsInitialized = false;
        m_FallArmSetups = {};
        m_Ragdoll.Destroy();
        m_FootSupport.Reset();
        ResetRecovery();
        m_AutomaticRootSupport = false;
        SetAutomaticRagdoll(false);
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

    bool SkeletalMeshComponent::UpdateKinematicPose(bool teleport)
    {
        if (!m_Ragdoll.IsActive() || IsRagdoll())
        {
            return false;
        }
        std::vector<Matrix4> worldPose;
        const bool evaluated = IsImpactWarmupActive() || IsBlendingToKinematic() ?
            EvaluatePhysicsPose(m_PhysicsSkeleton, GetFinalPose(), Matrix4(GetComponentTransform()), worldPose, m_PhysicsError) :
            EvaluateAnimationPhysicsPose(m_PhysicsSkeleton, worldPose, m_PhysicsError);
        return evaluated && m_Ragdoll.UpdateKinematicPose(worldPose, teleport, m_PhysicsError);
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
            std::vector<Matrix4> physicalPose;
            if (!EvaluatePhysicsPose(m_PhysicsSkeleton, GetFinalPose(), Matrix4(GetComponentTransform()), physicalPose, error) ||
                !m_Ragdoll.UpdateKinematicPose(physicalPose, true, error) || !m_Ragdoll.SetSimulating(true, velocity, error, active))
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
        m_RagdollSourcePose = GetRagdollAnimationPose();
        ResetCrossStepPlayback();
        m_KinematicBlendSource.clear();
        m_KinematicBlendPose.clear();
        m_PhysicsMode = mode;
        if (!active)
        {
            ResetAirborneFall();
        }
        ResetFallArms();
        const bool wasInWarmup = IsImpactWarmupActive();
        m_Ragdoll.SetDrivesEnabled(active, wasInWarmup ? &m_ImpactWarmup.reaction : nullptr);
        if (wasInWarmup)
        {
            m_ImpactWarmup = {};
        }
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
        const std::vector<GraphNode>& source = IsActiveRagdoll() ? GetRagdollAnimationPose() : m_RagdollSourcePose;
        if (IsActiveRagdoll())
        {
            std::vector<Matrix4> animated;
            if (EvaluateAnimationPhysicsPose(m_PhysicsSkeleton, animated, m_PhysicsError, &source))
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
        m_ImpactWarmup = {};
        ResetFallArms();
        if (!IsRagdoll())
        {
            return;
        }
        m_Ragdoll.SetSimulating(false, Vector3(kZero), m_PhysicsError);
        ResetAirborneFall();
        m_PhysicsMode = PhysicsAnimationMode::Animation;
        m_PhysicsPose.clear();
        m_RagdollSourcePose.clear();
        m_BoneStretch.clear();
        m_ModelInstance.ClearSkinnedWorldBounds();
        UpdateKinematicPose(true);
        UpdateFootSupport();
    }
}
