#include "SkeletalMeshComponent.h"
#include "FramePerformance.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>
#include "../GameFramework/Character.h"
#include "PhysicsWorld.h"

using namespace Math;
using namespace physx;

namespace PARS
{
    bool SkeletalMeshComponent::SetFallingAnimation(uint32_t animationIndex)
    {
        if (animationIndex >= m_AnimInstance->GetAnimationCount() || IsAirborneFallClip(animationIndex) ||
            animationIndex == m_StandingUpAnimationIndex || animationIndex == m_StandingIdleAnimationIndex)
        {
            return false;
        }
        m_FallingAnimationIndex = animationIndex;
        return true;
    }

    bool SkeletalMeshComponent::SetDirectionalFallingAnimation(FallDirection direction, uint32_t animationIndex)
    {
        const size_t slot = static_cast<size_t>(direction);
        if (slot >= m_DirectionalFallingAnimations.size() || animationIndex >= m_AnimInstance->GetAnimationCount() ||
            IsAirborneFallClip(animationIndex) ||
            animationIndex == m_StandingUpAnimationIndex || animationIndex == m_StandingIdleAnimationIndex)
        {
            return false;
        }
        m_DirectionalFallingAnimations[slot] = animationIndex;
        return true;
    }

    bool SkeletalMeshComponent::SetShoveReactionAnimations(uint32_t left, uint32_t right)
    {
        if (left == right || left >= m_AnimInstance->GetAnimationCount() || right >= m_AnimInstance->GetAnimationCount())
        {
            return false;
        }
        m_ShoveReactionAnimations = {left, right};
        return true;
    }

    bool SkeletalMeshComponent::IsFallingClip(uint32_t animationIndex) const
    {
        return animationIndex < m_AnimInstance->GetAnimationCount() && (animationIndex == m_FallingAnimationIndex ||
            std::find(m_DirectionalFallingAnimations.begin(), m_DirectionalFallingAnimations.end(), animationIndex) !=
                m_DirectionalFallingAnimations.end());
    }

    bool SkeletalMeshComponent::IsFollowingFallingAnimation() const
    {
        const uint32_t current = m_AnimInstance->GetCurrentAnimationIndex();
        return IsFallingClip(current) || (IsFalling() && IsAirborneFallClip(current));
    }

    FallDirection SkeletalMeshComponent::GetGroundFallDirection() const
    {
        const FootSupportSnapshot& support = m_FootSupport.GetSnapshot();
        PxVec3 travel = support.centerOfMassVelocity;
        if (!support.hasCenterOfMassVelocity || !travel.isFinite() || std::hypot(travel.x, travel.z) < 0.05f)
        {
            // 천천히 쓰러질 때는 지지 영역 밖으로 기운 쪽을 사용한다.
            if (!support.hasCenterOfMass || !support.balance.hasDirection || support.balance.zone == FootSupportZone::Inside)
            {
                return FallDirection::Count;
            }
            travel = support.centerOfMass - support.balance.center;
            if (!travel.isFinite() || std::hypot(travel.x, travel.z) < 0.02f)
            {
                return FallDirection::Count;
            }
        }

        // 보행과 같은 캐릭터 정면을 사용한다. 이 Mixamo 기준 자세에서 로컬 +X는 캐릭터의 왼쪽이다.
        const Vector3 facing(GetComponentTransform().GetZ());
        const float forward = float(facing.GetX()) * travel.x + float(facing.GetZ()) * travel.z;
        const float left = float(facing.GetZ()) * travel.x - float(facing.GetX()) * travel.z;
        if (!std::isfinite(forward) || !std::isfinite(left) || (forward == 0.0f && left == 0.0f))
        {
            return FallDirection::Count;
        }
        if (std::abs(forward) >= std::abs(left))
        {
            return forward >= 0.0f ? FallDirection::Forward : FallDirection::Backward;
        }
        return left >= 0.0f ? FallDirection::Left : FallDirection::Right;
    }

    uint32_t SkeletalMeshComponent::GetIdleAnimationIndex() const
    {
        const uint32_t current = m_AnimInstance->GetCurrentAnimationIndex();
        if (m_AnimInstance->GetAnimationName(current) == "Idle")
        {
            return current;
        }
        if (m_StandingIdleAnimationIndex < m_AnimInstance->GetAnimationCount())
        {
            return m_StandingIdleAnimationIndex;
        }
        for (uint32_t clip = 0; clip < m_AnimInstance->GetAnimationCount(); ++clip)
        {
            if (m_AnimInstance->GetAnimationName(clip) == "Idle")
            {
                return clip;
            }
        }
        return UINT32_MAX;
    }

    void SkeletalMeshComponent::TrackShoveContact(const std::vector<RagdollBodyImpulse>& impacts, const PxVec3& centerOfMass)
    {
        if (m_ShoveReactionAnimations[0] == UINT32_MAX)
        {
            return;
        }
        // 같은 틱에 여러 캡슐이 닿으면 수평 충격이 가장 큰 접촉을 선택한다.
        const RagdollBodyImpulse* hit = nullptr;
        float largest = 1e-8f;
        for (const RagdollBodyImpulse& impact : impacts)
        {
            const float horizontal = impact.impulse.x * impact.impulse.x + impact.impulse.z * impact.impulse.z;
            if (impact.newCapsuleHit && impact.impactContactId != 0 && horizontal > largest)
            {
                hit = &impact;
                largest = horizontal;
            }
        }
        if (!hit || hit->impactContactId == m_ShoveContact.id)
        {
            return;
        }
        m_ShoveContact = {};
        m_ShoveContact.id = hit->impactContactId;
        // 여러 몸통 본에 걸친 접촉은 충격량으로 가중한다. 방향 성분의 비율만 쓰므로 가중치 합은 생략한다.
        PxVec3 offset(0);
        for (const RagdollBodyImpulse& impact : impacts)
        {
            if (impact.impactContactId == hit->impactContactId)
            {
                offset += (impact.position - centerOfMass) * impact.impulse.magnitude();
            }
        }
        const Vector3 forward = GetComponentTransform().GetZ();
        const float axial = float(forward.GetX()) * offset.x + float(forward.GetZ()) * offset.z;
        const float lateral = float(forward.GetZ()) * offset.x - float(forward.GetX()) * offset.z;
        // 정면 중앙은 기존 반응을 쓴다. 앞에서 들어와도 접촉점이 어깨 쪽으로 벗어나면 밀침 반응을 쓴다.
        constexpr float centerHalfWidthToDepth = .5f;
        const bool shoulderHit = std::abs(lateral) > centerHalfWidthToDepth * std::abs(axial);
        m_ShoveContact.pending = shoulderHit && hit->capsuleTorqueMultiplier > 0;
        if (!shoulderHit && m_ImpactFlail.playingShove)
        {
            // 별개의 정면 충돌은 진행 중인 어깨 반응에서 기존 균형 반응으로 전환한다.
            std::vector<GraphNode> source = std::move(m_ImpactFlail.pose);
            m_ImpactFlail = {};
            m_ImpactFlail.blendSource = std::move(source);
        }
    }

    void SkeletalMeshComponent::StartShoveReaction(float yaw)
    {
        if (!m_ShoveContact.pending)
        {
            return;
        }
        // 수치 오차를 넘긴 첫 회전 방향을 접촉당 한 번만 사용한다.
        m_ShoveContact.yaw += yaw;
        if (std::abs(m_ShoveContact.yaw) < 1e-5f)
        {
            return;
        }
        m_ShoveContact.pending = false;
        // 이 스켈레톤의 정면 +Z가 +X로 도는 양의 yaw는 캐릭터의 왼쪽 회전이다.
        const uint32_t clip = m_ShoveReactionAnimations[m_ShoveContact.yaw > 0 ? 0 : 1];
        if (clip >= m_AnimInstance->GetAnimationCount())
        {
            return;
        }
        m_ImpactFlail.blendSource = std::move(m_ImpactFlail.pose);
        m_ImpactFlail.clip = clip;
        m_ImpactFlail.time = 0;
        m_ImpactFlail.weight = 0;
        m_ImpactFlail.playingShove = true;
    }

    void SkeletalMeshComponent::UpdateImpactFlail(bool triggerReaction, float deltaTime)
    {
        if (IsFalling() || IsGroundFallActive() || m_StandingUpState != StandingUpState::None)
        {
            m_ImpactFlail = {};
            return;
        }
        ImpactFlailPlayback& flail = m_ImpactFlail;
        constexpr float blendDuration = .1f;
        if (flail.playingShove)
        {
            const float duration = m_AnimInstance->GetAnimationDuration(flail.clip);
            flail.time = (std::min)(flail.time + deltaTime, duration);
            flail.weight = Clamp((std::min)(flail.time, duration - flail.time) / blendDuration, 0.0f, 1.0f);
        }
        else
        {
            // 충격 직후 균형 상태가 갱신될 때까지 상체 반응을 이어 간다.
            flail.impactTimeRemaining = triggerReaction ? MotionFeatures::PredictionTimeSeconds :
                (std::max)(0.0f, flail.impactTimeRemaining - deltaTime);
            const bool recovering = flail.impactTimeRemaining > 0 || m_BalanceState == CharacterBalanceState::Unbalanced;
            if (recovering && flail.clip == UINT32_MAX)
            {
                const MotionMatchResult match = m_AnimInstance->FindImpactPoseMatch(MotionFrameUsage::BalanceUpperBody);
                flail.clip = match.clipIndex;
                flail.time = match.clipTimeSeconds;
            }
            if (flail.clip == UINT32_MAX)
            {
                return;
            }
            flail.weight = Clamp(flail.weight + (recovering ? deltaTime : -deltaTime) / blendDuration, 0.0f, 1.0f);
        }
        const std::vector<GraphNode>* basePose = IsImpactWarmupActive() ? &m_ImpactWarmup.sourcePose : nullptr;
        const std::vector<GraphNode>* source = flail.blendSource.empty() ? nullptr : &flail.blendSource;
        if (flail.weight <= 0 || !m_AnimInstance->SampleUpperBodyPose(flail.clip, flail.time, flail.weight, flail.pose, basePose, source))
        {
            flail = {};
            return;
        }
        if (flail.weight >= 1.0f)
        {
            flail.blendSource.clear();
        }
        if (!flail.playingShove)
        {
            constexpr float playbackRate = 0.5f;
            flail.time = std::fmod(flail.time + deltaTime * playbackRate, m_AnimInstance->GetAnimationDuration(flail.clip));
        }
    }

    MotionMatchResult SkeletalMeshComponent::SelectFallingAnimation() const
    {
        const FallDirection direction = GetGroundFallDirection();
        const uint32_t directionalClip = direction == FallDirection::Count ? UINT32_MAX : m_DirectionalFallingAnimations[static_cast<size_t>(direction)];
        MotionMatchResult match = m_AnimInstance->FindImpactPoseMatch(MotionFrameUsage::ShoveFall, directionalClip);
        if (match.clipIndex == UINT32_MAX)
        {
            match.clipIndex = directionalClip < m_AnimInstance->GetAnimationCount() ? directionalClip : m_FallingAnimationIndex;
        }
        return match;
    }

    bool SkeletalMeshComponent::SetStandingUpAnimation(uint32_t animationIndex, uint32_t idleAnimationIndex)
    {
        if (animationIndex >= m_AnimInstance->GetAnimationCount() ||
            idleAnimationIndex >= m_AnimInstance->GetAnimationCount() || animationIndex == idleAnimationIndex ||
            IsFallingClip(animationIndex) || IsFallingClip(idleAnimationIndex) ||
            IsAirborneFallClip(animationIndex) || IsAirborneFallClip(idleAnimationIndex) ||
            m_AnimInstance->GetAnimationName(idleAnimationIndex) != "Idle")
        {
            return false;
        }
        m_StandingUpAnimationIndex = animationIndex;
        m_StandingIdleAnimationIndex = idleAnimationIndex;
        m_StandingUpStableTime = 0;
        m_StandingUpState = StandingUpState::None;
        return true;
    }

    bool SkeletalMeshComponent::IsFollowingStandingUpAnimation() const
    {
        return m_StandingUpAnimationIndex < m_AnimInstance->GetAnimationCount() &&
            m_AnimInstance->GetCurrentAnimationIndex() == m_StandingUpAnimationIndex;
    }

    bool SkeletalMeshComponent::UpdateFallingAnimation(float deltaTime)
    {
        if (IsFalling())
        {
            // 공중에서 선택한 루프는 실제 지면 접촉까지 같은 재생 시계로 진행한다.
            return true;
        }
        // 기상과 Idle 보간은 물리 시뮬레이션을 꺼 둔 상태에서도 계속 진행한다.
        if (m_StandingUpState == StandingUpState::Kinematic || m_StandingUpState == StandingUpState::IdleBlend)
        {
            return UpdateStandingUpAnimation(deltaTime);
        }
        if (!IsGroundFallActive())
        {
            m_StandingUpStableTime = 0;
            m_StandingUpState = StandingUpState::None;
            return true;
        }
        // 한 번 고른 클립은 착지와 감속으로 COM 방향이 바뀌어도 끝까지 유지한다.
        if (m_FallingAnimationEnabled && IsFollowingFallingAnimation())
        {
            return UpdateStandingUpAnimation(deltaTime);
        }
        const MotionMatchResult fall = m_FallingAnimationEnabled ? SelectFallingAnimation() : MotionMatchResult{};
        if (fall.clipIndex >= m_AnimInstance->GetAnimationCount())
        {
            // 낙상 클립을 꺼도 물리 낙상과 착지 후 기상은 계속 처리한다.
            if (IsFollowingFallingAnimation() || IsFollowingStandingUpAnimation() ||
                IsAirborneFallClip(m_AnimInstance->GetCurrentAnimationIndex()))
            {
                // 재생 도중 끄거나 기상 중 다시 넘어지면 기본 Idle 목표로 돌아간다.
                if (m_StandingIdleAnimationIndex < m_AnimInstance->GetAnimationCount())
                {
                    if (!m_AnimInstance->TransitionToAnimation(m_StandingIdleAnimationIndex, .05f, true))
                    {
                        m_PhysicsError = "Cannot return to idle after disabling the falling animation.";
                        return false;
                    }
                    m_Ragdoll.ResetDriveTargetHistory();
                }
                else
                {
                    m_AnimInstance->SetPlaying(false);
                }
            }
            return UpdateStandingUpAnimation(deltaTime);
        }
        // 낙상은 한 번 재생하고 착지 후 기상을 기다린다. 위치와 회전 지지는 이미 해제된 상태다.
        const std::vector<GraphNode>* previous = m_ImpactFlail.pose.empty() && m_CrossStepPose.empty() ? nullptr : &GetRagdollAnimationPose();
        if (!m_AnimInstance->TransitionToAnimation(fall.clipIndex, .2f, false, fall.clipTimeSeconds, previous))
        {
            m_PhysicsError = "Cannot transition to the falling animation.";
            return false;
        }
        m_KinematicRecoveryTimer.Reset();
        m_StandingUpStableTime = 0;
        m_StandingUpState = StandingUpState::None;
        m_Ragdoll.ResetDriveTargetHistory();
        ResetRecovery();
        return true;
    }

    bool SkeletalMeshComponent::UpdateStandingUpAnimation(float deltaTime)
    {
        if (m_StandingUpAnimationIndex >= m_AnimInstance->GetAnimationCount() ||
            m_StandingIdleAnimationIndex >= m_AnimInstance->GetAnimationCount())
        {
            return true;
        }
        if (m_StandingUpState == StandingUpState::IdleBlend)
        {
            if (m_AnimInstance->IsBlending() || IsBlendingToKinematic())
            {
                return true;
            }
            // 최초 스폰처럼 키네마틱 Idle을 유지하고, 다음 충격에서만 물리 시뮬레이션을 켠다.
            m_StandingUpState = StandingUpState::None;
            m_StandingUpStableTime = 0;
            m_RootSupportRelease = RootSupportRelease::None;
            m_PendingRecoveryImpact = false;
            m_FootSupport.ResetFootstep();
            ResetRecovery();
            m_Ragdoll.ResetDriveTargetHistory();
            m_KinematicRecoveryTimer.Reset();
            return true;
        }

        // 낙상 시퀀스를 생략한 경우 반복 Idle의 종료를 기다리지 않는다.
        const bool waitForClip = m_StandingUpState == StandingUpState::Kinematic ||
            (m_FallingAnimationEnabled && IsFollowingFallingAnimation());
        if ((waitForClip && m_AnimInstance->IsPlaying()) || m_AnimInstance->IsBlending() || IsBlendingToKinematic())
        {
            m_StandingUpStableTime = 0;
            return true;
        }
        if (m_StandingUpState == StandingUpState::Kinematic)
        {
            // 키네마틱 기상은 양발의 물리 접촉을 기다리지 않고 클립이 끝나면 Idle로 이어진다.
            if (!m_AnimInstance->TransitionToAnimation(m_StandingIdleAnimationIndex, .2f, true))
            {
                m_PhysicsError = "Cannot transition from the standing-up animation to idle.";
                return false;
            }
            m_StandingUpState = StandingUpState::IdleBlend;
            return true;
        }

        Vector3 velocity(kZero);
        const bool settled = m_Ragdoll.GetCenterOfMassVelocity(velocity) &&
            float(Length(velocity)) <= m_RecoverySettings.maximumReturnSpeed;
        bool grounded = false;
        float groundHeight = -std::numeric_limits<float>::infinity();
        const bool requireBodyContact = !m_FallingAnimationEnabled || !IsFollowingFallingAnimation();
        const std::vector<int>& nodes = m_Ragdoll.GetBoneNodes();
        const FootSupportSnapshot& support = m_FootSupport.GetSnapshot();
        for (size_t body = 0; body < m_Ragdoll.GetBodyCount(); ++body)
        {
            PhysicsGroundContact contact;
            if (m_Ragdoll.GetBodyGroundContact(body, contact))
            {
                // 클립의 종료 대기가 없을 때는 발만 닿은 채 기상 동작을 시작하지 않는다.
                const bool footContact = body < nodes.size() &&
                    (nodes[body] == support.feet[0].boneNode || nodes[body] == support.feet[1].boneNode);
                grounded = grounded || !requireBodyContact || !footContact;
                groundHeight = Max(groundHeight, contact.position.y);
            }
        }
        const bool ready = settled && grounded;
        m_StandingUpStableTime = ready ? m_StandingUpStableTime + deltaTime : 0;
        if (m_StandingUpStableTime < m_RecoverySettings.stableDuration || m_Ragdoll.HasWound())
        {
            return true;
        }

        return BeginKinematicStandingUp(groundHeight);
    }

    bool SkeletalMeshComponent::BeginKinematicStandingUp(float groundHeight)
    {
        const int rootBody = m_Ragdoll.GetImpactRootBody();
        const std::vector<int>& nodes = m_Ragdoll.GetBoneNodes();
        if (!m_PhysicsAsset || !std::isfinite(groundHeight) || rootBody < 0 ||
            static_cast<size_t>(rootBody) >= nodes.size())
        {
            m_PhysicsError = "Kinematic standing-up requires a pelvis body and ground contact.";
            return false;
        }
        std::vector<GraphNode> standingPose;
        std::vector<GraphNode> idlePose;
        std::vector<Matrix4> standingWorld;
        std::vector<Matrix4> idleWorld;
        const Matrix4 component(GetComponentTransform());
        if (!m_AnimInstance->SampleAnimationPose(m_StandingUpAnimationIndex, 0, standingPose) ||
            !m_AnimInstance->SampleAnimationPose(m_StandingIdleAnimationIndex, 0, idlePose) ||
            !EvaluatePhysicsPose(m_PhysicsSkeleton, standingPose, component, standingWorld, m_PhysicsError) ||
            !EvaluatePhysicsPose(m_PhysicsSkeleton, idlePose, component, idleWorld, m_PhysicsError))
        {
            m_PhysicsError = "Cannot sample the standing-up and idle alignment poses.";
            return false;
        }
        const int rootNode = nodes[rootBody];
        if (rootNode < 0 || static_cast<size_t>(rootNode) >= standingWorld.size() ||
            nodes.size() != m_PhysicsAsset->bodies.size())
        {
            m_PhysicsError = "Invalid pelvis or body mapping for kinematic standing-up.";
            return false;
        }

        // 기상 뒤 Idle의 PA 최하단이 실제 착지한 바닥에 놓이도록 기준 높이를 잡는다.
        float idleLowest = std::numeric_limits<float>::infinity();
        for (size_t body = 0; body < nodes.size(); ++body)
        {
            const int node = nodes[body];
            if (node < 0 || static_cast<size_t>(node) >= idleWorld.size())
            {
                m_PhysicsError = "Invalid idle body mapping for kinematic standing-up.";
                return false;
            }
            const PARS::PhysicsBody& shape = m_PhysicsAsset->bodies[body];
            const Matrix4 frame = PhysicsBoneFrame(idleWorld[node]) * shape.GetLocalTransform();
            float extent = shape.radius;
            if (shape.shape == PhysicsShapeType::Capsule)
            {
                extent += shape.halfLength * std::abs(float(frame.GetX().GetY()));
            }
            else if (shape.shape == PhysicsShapeType::Box)
            {
                extent = shape.halfExtents.x * std::abs(float(frame.GetX().GetY())) +
                    shape.halfExtents.y * std::abs(float(frame.GetY().GetY())) +
                    shape.halfExtents.z * std::abs(float(frame.GetZ().GetY()));
            }
            idleLowest = Min(idleLowest, float(frame.GetW().GetY()) - extent);
        }

        // 누운 골반의 몸통 방향을 수평으로 맞춘다. 수직에 가까우면 다른 축을 사용한다.
        const Matrix4 actualPelvis = m_Ragdoll.GetBoneWorld(static_cast<size_t>(rootBody));
        const Matrix4 animatedPelvis = PhysicsBoneFrame(standingWorld[rootNode]);
        const Vector3 actualAxes[] = {Vector3(actualPelvis.GetY()), Vector3(actualPelvis.GetZ()), Vector3(actualPelvis.GetX())};
        const Vector3 animatedAxes[] = {Vector3(animatedPelvis.GetY()), Vector3(animatedPelvis.GetZ()), Vector3(animatedPelvis.GetX())};
        float yaw = 0;
        for (size_t axis = 0; axis < 3; ++axis)
        {
            Vector3 actual = actualAxes[axis];
            Vector3 animated = animatedAxes[axis];
            actual.SetY(0);
            animated.SetY(0);
            if (float(LengthSquare(actual)) > .01f && float(LengthSquare(animated)) > .01f)
            {
                yaw = std::atan2(float(Cross(animated, actual).GetY()), float(Dot(animated, actual)));
                break;
            }
        }
        AffineTransform alignment = AffineTransform::MakeYRotation(yaw);
        const Vector3 rotatedPelvis = alignment * Vector3(animatedPelvis.GetW());
        Vector3 offset = Vector3(actualPelvis.GetW()) - rotatedPelvis;
        offset.SetY(groundHeight - idleLowest);
        if (!std::isfinite(float(Length(offset))))
        {
            m_PhysicsError = "Invalid kinematic standing-up alignment.";
            return false;
        }
        alignment.SetTranslation(offset);
        const AffineTransform previousTransform = GetOwner().GetActorTransform();
        if (!GetOwner().SetActorTransform(alignment * previousTransform))
        {
            m_PhysicsError = "Cannot align the character for kinematic standing-up.";
            return false;
        }
        // 정렬된 액터 좌표계에서 실제 물리 자세를 다시 표현해 전환 순간의 월드 자세를 보존한다.
        if (!CaptureRecoveryBlendSource())
        {
            GetOwner().SetActorTransform(previousTransform);
            m_PhysicsError = "Cannot capture the fallen pose for kinematic standing-up.";
            return false;
        }
        m_KinematicBlendSource = m_RecoveryBlendSource;
        m_KinematicBlendPose = m_KinematicBlendSource;
        m_KinematicBlendTime = 0;
        Character* character = dynamic_cast<Character*>(&GetOwner());
        if (character)
        {
            if (!character->StopRagdoll(m_PhysicsError))
            {
                GetOwner().SetActorTransform(previousTransform);
                m_KinematicBlendSource.clear();
                m_KinematicBlendPose.clear();
                return false;
            }
        }
        else
        {
            StopRagdoll();
        }
        if (IsRagdoll() || !m_AnimInstance->PlayAnimation(m_StandingUpAnimationIndex, false))
        {
            m_PhysicsError = "Cannot start kinematic standing-up.";
            return false;
        }
        m_StandingUpState = StandingUpState::Kinematic;
        m_StandingUpStableTime = 0;
        m_KinematicRecoveryTimer.Reset();
        m_Ragdoll.ResetDriveTargetHistory();
        ResetRecovery();
        return true;
    }

    void SkeletalMeshComponent::SetAutomaticRagdoll(bool enabled)
    {
        if (!enabled)
        {
            m_ImpactWarmup = {};
        }
        m_AutomaticRagdoll = enabled;
        m_KinematicRecoveryTimer.Reset();
        m_KinematicBlendSource.clear();
        m_KinematicBlendPose.clear();
        m_KinematicBlendTime = 0;
        m_PendingRecoveryImpact = false;
        m_StandingUpStableTime = 0;
        m_StandingUpState = StandingUpState::None;
    }

    void SkeletalMeshComponent::SetKinematicStableDuration(float duration)
    {
        if (std::isfinite(duration) && duration > 0)
        {
            m_KinematicStableDuration = duration;
            m_KinematicRecoveryTimer.Reset();
        }
    }

    void SkeletalMeshComponent::ReplayKinematicImpacts(const std::vector<RagdollBodyImpulse>& impacts, float deltaTime)
    {
        m_Ragdoll.ApplyExternalImpacts(impacts);
        m_Ragdoll.ApplyImpactResponses(m_Ragdoll.GetImpactResponses(impacts));
        // 모드 교체로 소멸한 접촉도 이번 틱의 관절 반응에는 그대로 넘긴다.
        m_Ragdoll.UpdateImpactReaction(impacts, deltaTime);
    }

    bool SkeletalMeshComponent::UpdateAutomaticRagdoll(float deltaTime, bool firstHit)
    {
        if (!m_AutomaticRagdoll)
        {
            return true;
        }
        const FootSupportSnapshot& sample = m_FootSupport.GetSnapshot();
        const float impactVelocity = sample.totalMass > 0 ? m_Ragdoll.GetExternalImpactImpulse() / sample.totalMass : 0;
        const bool impact = impactVelocity >= m_RecoverySettings.impactVelocity;
        if (!IsRagdoll())
        {
            // 큰 몸 충돌은 정지 상태에서 실제 수평 충격량을 받은 뒤 예열 여부를 결정한다.
            const float deferCapsule = m_ImpactWarmup.wasResting ? ImpactWarmup::MinimumImpactSpeedChange : PX_MAX_F32;
            const bool approaching = m_Ragdoll.HasApproachingImpact(m_RecoverySettings.impactVelocity, deferCapsule);
            Character* character = dynamic_cast<Character*>(&GetOwner());
            const bool movingCapsule = character && character->GetCharacterMovement().IsTickEnabled() &&
                character->GetCapsuleComponent().HasPhysicsCollision();
            // 이동 캡슐이 없는 대기 더미는 받침이 사라져도 키네마틱인 동안에는 하강 속도가 생기지 않는다.
            const bool lostFloor = !movingCapsule && m_AutomaticRootSupport && !IsBlendingToKinematic() &&
                m_StandingUpState == StandingUpState::None && m_AirborneFallAnimations.front() != UINT32_MAX &&
                sample.projectedPointCount == 0 && sample.feet[0].contact.hasSample && sample.feet[1].contact.hasSample &&
                !sample.feet[0].contact.grounded && !sample.feet[1].contact.grounded;
            if (!impact && !approaching && !lostFloor && !firstHit)
            {
                return true;
            }
            const bool interruptedStandingUp = m_StandingUpState == StandingUpState::Kinematic ||
                m_StandingUpState == StandingUpState::IdleBlend;
            // 모드 전환은 접촉 캐시를 비우므로 충격점과 바디별 충격량을 먼저 복사한다.
            std::vector<RagdollBodyImpulse> impacts;
            m_Ragdoll.CaptureExternalImpacts(impacts);
            const int impactRoot = m_Ragdoll.GetImpactRootBody();
            PxVec3 impactPosition(0);
            float impactWeight = 0;
            for (const RagdollBodyImpulse& bodyImpact : impacts)
            {
                const float weight = bodyImpact.impulse.magnitude();
                if ((bodyImpact.fromProjectile || (impactRoot >= 0 && bodyImpact.body == static_cast<size_t>(impactRoot))) &&
                    bodyImpact.position.isFinite() && std::isfinite(weight) && weight > 1e-6f)
                {
                    impactPosition += bodyImpact.position * weight;
                    impactWeight += weight;
                }
            }
            const bool activated = character ? character->SetPhysicsMode(PhysicsAnimationMode::ActiveRagdoll, m_PhysicsError) :
                SetPhysicsMode(PhysicsAnimationMode::ActiveRagdoll, Vector3(kZero), m_PhysicsError);
            if (!activated)
            {
                return false;
            }
            if (interruptedStandingUp)
            {
                // 현재 보간 자세와 속도를 물리 바디에 넘긴 뒤 기상을 취소한다.
                m_StandingUpState = StandingUpState::None;
                m_StandingUpStableTime = 0;
            }
            // 일어나는 도중에는 골반을 서기 목표에 붙잡지 않고 낙상 경로로 돌아간다.
            const bool releaseRoot = interruptedStandingUp || lostFloor;
            m_RootSupportRelease = releaseRoot ? RootSupportRelease::NoSupport : RootSupportRelease::None;
            m_FootSupport.ResetFootstep();
            ResetRecovery();
            m_KinematicRecoveryTimer.Reset();
            m_PendingRecoveryImpact = !interruptedStandingUp && (impact || approaching);
            ReplayKinematicImpacts(impacts, deltaTime);
            if (impactWeight > 0)
            {
                m_FootSupport.RecordImpact(impactPosition / impactWeight);
            }
            return m_Ragdoll.SetRootSupported(!releaseRoot, m_PhysicsError);
        }
        // 이번 틱에 발딛기를 포기한 경우도 낙상 경로에 맡겨, 서기 복귀가 먼저 시작되지 않게 한다.
        if (!IsActiveRagdoll() || IsFalling() || IsGroundFallActive() ||
            sample.footstep.aborted || sample.footstep.backwardFallStepsRemaining > 0)
        {
            m_KinematicRecoveryTimer.Reset();
            return true;
        }
        const bool insideA = sample.hasComProjection && sample.balance.zone == FootSupportZone::Inside;
        const bool bothGrounded = sample.feet[0].available && sample.feet[0].contact.hasSample &&
            sample.feet[0].contact.grounded && sample.feet[1].available &&
            sample.feet[1].contact.hasSample && sample.feet[1].contact.grounded;
        // A영역 유지 시간은 그대로 누적하되, 실제 양발 접지가 확인될 때만 전환한다.
        if (m_KinematicRecoveryTimer.Update(sample.contactRevision, sample.completedPhysicsStep,
            deltaTime, insideA, bothGrounded, impact, m_KinematicStableDuration) && !firstHit && m_ImpactHandReach.body < 0 &&
            !m_Ragdoll.HasWound())
        {
            return BeginKinematicRecovery();
        }
        return true;
    }

    bool SkeletalMeshComponent::BeginKinematicRecovery()
    {
        std::vector<Matrix4> targets;
        if (!EvaluateAnimationPhysicsPose(m_PhysicsSkeleton, targets, m_PhysicsError))
        {
            return false;
        }
        Vector3 actualMidpoint(kZero);
        Vector3 animationMidpoint(kZero);
        for (const FootSoleSample& foot : m_FootSupport.GetSnapshot().feet)
        {
            if (!foot.available || foot.boneNode < 0 || static_cast<size_t>(foot.boneNode) >= targets.size())
            {
                m_PhysicsError = "Kinematic recovery requires both foot sole frames.";
                return false;
            }
            // PA와 같은 강체 본 좌표계를 사용해 발목이 아닌 발바닥 중심을 비교한다.
            const Matrix4 animatedSole = PhysicsBoneFrame(targets[foot.boneNode]) * foot.soleInBone;
            actualMidpoint += Vector3(foot.soleWorld.GetW()) * .5f;
            animationMidpoint += Vector3(animatedSole.GetW()) * .5f;
        }
        const Vector3 offset = actualMidpoint - animationMidpoint;
        if (!std::isfinite(float(Length(offset))))
        {
            m_PhysicsError = "Invalid foot sole position in kinematic recovery.";
            return false;
        }
        // 현재 양발 중점에 애니메이션 양발 중점을 맞추며 높이 차이도 루트에 반영한다.
        const Vector3 previousLocation = GetOwner().GetActorLocation();
        GetOwner().SetActorLocation(previousLocation + offset);
        if (!CaptureRecoveryBlendSource())
        {
            GetOwner().SetActorLocation(previousLocation);
            m_PhysicsError = "Cannot capture the physical pose for kinematic recovery.";
            return false;
        }
        m_KinematicBlendSource = m_RecoveryBlendSource;
        m_KinematicBlendPose = m_KinematicBlendSource;
        m_KinematicBlendTime = 0;
        // 첫 키네마틱 포즈도 캡처한 실제 자세를 사용해 모드 전환 순간의 튐을 막는다.
        Character* character = dynamic_cast<Character*>(&GetOwner());
        if (character)
        {
            if (!character->StopRagdoll(m_PhysicsError))
            {
                GetOwner().SetActorLocation(previousLocation);
                m_KinematicBlendSource.clear();
                m_KinematicBlendPose.clear();
                return false;
            }
        }
        else
        {
            StopRagdoll();
        }
        return !IsRagdoll();
    }

    bool SkeletalMeshComponent::UpdateKinematicRecoveryPose(float deltaTime)
    {
        if (m_KinematicBlendPose.empty())
        {
            return true;
        }
        m_KinematicBlendTime += deltaTime;
        if (m_KinematicBlendTime >= m_RecoverySettings.blendDuration)
        {
            m_KinematicBlendSource.clear();
            m_KinematicBlendPose.clear();
            return true;
        }
        const float t = m_KinematicBlendTime / m_RecoverySettings.blendDuration;
        m_KinematicBlendPose = m_AnimInstance->GetPose();
        if (!BlendRecoveryPose(m_KinematicBlendSource, m_KinematicBlendPose, t * t * (3 - 2 * t)))
        {
            m_PhysicsError = "Invalid kinematic recovery blend pose.";
            return false;
        }
        return true;
    }

    namespace
    {
        PxTransform ToRecoveryFrame(const Matrix4& matrix)
        {
            const PhysicsJointFrame frame = PhysicsJointFrame::FromTransform(PhysicsBoneFrame(matrix));
            return PxTransform(PxVec3(frame.offset.x, frame.offset.y, frame.offset.z),
                PxQuat(frame.rotation.x, frame.rotation.y, frame.rotation.z, frame.rotation.w));
        }

        PxTransform CaptureSupportSole(const FootSoleSample& foot)
        {
            const PxTransform actual = ToRecoveryFrame(foot.soleWorld);
            PxVec3 points[3];
            unsigned count = 0;
            for (size_t corner = 0; corner < foot.projected.size() && count < 3; ++corner)
            {
                if (foot.projected[corner])
                {
                    points[count++] = foot.projections[corner];
                }
            }
            if (count < 3)
            {
                return actual;
            }
            PxVec3 normal = (points[1] - points[0]).cross(points[2] - points[0]);
            if (normal.normalize() < 1e-6f)
            {
                return actual;
            }
            if (normal.y < 0)
            {
                normal = -normal;
            }
            if (normal.y < .70710678f)
            {
                return actual;
            }
            // 초기 공중 위치를 영구 고정점으로 저장하지 않는다. 실제 레이로 얻은 바닥에 발 목표를 둔다.
            // 목표만 바꾸며 물리 접촉 판정이나 실제 바디 위치는 수정하지 않는다.
            const PxVec3 position = actual.p - normal * normal.dot(actual.p - points[0]);
            PxTransform planted(PxIdentity);
            return MakeFootSoleTarget(position, normal, actual, planted) ? planted : actual;
        }
    }

    void SkeletalMeshComponent::ResetRecovery()
    {
        ResetCrossStepPlayback();
        m_FootstepAvoidance.Reset();
        m_FootstepCollision = {};
        m_Recovery.Reset();
        m_HasRecoverySoles = false;
        m_RecoveryReferenceHull.clear();
        m_RecoveryBlendSource.clear();
        m_FootstepIK.Reset();
        m_StandingSupport.Reset();
        m_HorizontalBalance.Reset();
        m_Ragdoll.SetHorizontalSupportForce(PxVec3(0));
        m_Ragdoll.SetSupportContacts({});
    }

    bool SkeletalMeshComponent::CaptureRecoveryBlendSource()
    {
        std::vector<Matrix4> bodies;
        const std::vector<int>& nodes = m_Ragdoll.GetBoneNodes();
        if (m_BoneStretch.size() != nodes.size())
        {
            return false;
        }
        for (size_t body = 0; body < nodes.size(); ++body)
        {
            bodies.push_back(m_Ragdoll.GetBoneWorld(body) * m_BoneStretch[body]);
        }
        // 직전 렌더 포즈가 아니라 마지막 물리 스텝의 실제 바디를 로컬 본 자세로 변환한다.
        return OverridePhysicsPose(m_PhysicsSkeleton, GetRagdollAnimationPose(), Matrix4(GetComponentTransform()),
            nodes, bodies, m_RecoveryBlendSource);
    }

    bool SkeletalMeshComponent::ApplyRecoveryTargets(std::vector<Matrix4>& target, float deltaTime)
    {
        FramePerformance::Scope recoveryTiming(FramePerformance::Category::RecoveryTargets);
        // UpdateFootSupport() 에서 계산한 발 지지 정보와 발딛기 결과를 가져옴
        const FootSupportSnapshot& sample = m_FootSupport.GetSnapshot();
        const FootstepSnapshot& step = sample.footstep;

        if (IsGroundFallActive() && IsFollowingFallingAnimation())
        {
            // 지상 낙상 클립이 시작된 뒤에는 팔과 다리도 선택된 전신 자세를 따른다.
            return true;
        }
        if (step.backwardFallStepsRemaining > 0)
        {
            // 낙상 클립의 상체를 유지하고 지면을 짚는 다리에만 목표를 적용한다.
            ApplyFootstepIKTargets(target, deltaTime);
            return true;
        }

        // 현재 애니메이션이 안정적으로 idle을 재생 중인지 확인
        // 애니메이션 클립사이의 전환 보간 중이 아니여야함. 현재 재생 중인 애니메이션 이름이 idle
        // 예를들어 Idle->Walk 전환 중일 때 걷기 애니메이션을 재생 중이면 거짓
        const bool idle = !m_AnimInstance->IsBlending() && m_AnimInstance->GetAnimationName(m_AnimInstance->GetCurrentAnimationIndex()) == "Idle";

        // 회복 제어의 실행 조건을 검사
        // 회복 판단에 사용할 입력 객체를 만듦
        RecoveryInput input;
        input.enabled = IsActiveRagdoll() && m_AutomaticRootSupport && idle;

        if (!input.enabled)
        {
            // 회복상태, 저장한 발 목표, 복귀 보간 포즈, IK상태 등을 초기화
            ResetRecovery();
            return true;
        }

        // 지지 해제 여부와 발딛기 상태를 전달
        // fallen : 골반 지지가 해제됐거나 발딛기가 중단
        // needsStep : 플래너가 예측 COM을 보고 발딛기가 필요하다고 판단
        // stepping : 발딛기가 이미 시작돼, 이륙 대기 또는 공중 단계인지
        input.fallen = m_RootSupportRelease != RootSupportRelease::None || step.aborted;
        input.needsStep = step.needsStep;
        input.stepping = step.IsInProgress();

        // 시간과 입력의 버전 정보를 전달. 예를 들어 같은 물리 결과로 회복 판단을 두번 호출해도, 안정화 시간이 두 배로 누적되지 않도록 스텝 번호를 사용
        // deltatime : 안정 상태를 유지한 시간과 복귀 보간 시간 계산
        // revision : 접촉 이력이 초기화됐는지 감지하고 이전 회복 상태 초기화
        // completedStep : 같은 완료 물리 스텝을 중복 처리하지 않도록 확인
        input.deltaTime = deltaTime;
        input.revision = sample.contactRevision;
        input.completedStep = sample.completedPhysicsStep;

        // 현재 COM이 지지 영역 안에 있는지 전달
        input.insideSupport = sample.hasCenterOfMass && sample.comInsideHull;

        // 예측 COM도 지지 영역 안에 있는지 전달
        // 플래너가 이미 계산한 결과를 이용
        input.predictionInsideSupport = step.hasPrediction && !step.needsStep;

        // 양발 모두 실제로 접지했는지 전달. 접촉샘플 데이터와 지면접촉정보로 판단
        input.bothGrounded =
            sample.feet[0].contact.hasSample &&
            sample.feet[0].contact.grounded &&
            sample.feet[1].contact.hasSample &&
            sample.feet[1].contact.grounded;

        // 외부 충격을 속도 변화 단위로 환산. GetExternalImpactImpulse()은 물리 엔진에 기록된 전신의 외부 충돌 충격량을 가져옴. 이를 전신 질량으로 나눔
        if (sample.totalMass > 0)
        {
            // 충격량을 전신 질량으로 나눠 모델 크기에 덜 의존하는 속도 변화 단위로 판정한다.
            // 상대 캡슐이 PA에 가한 충격은 포함하고, 본인 독립 캡슐에 생긴 반작용은 제외한다.
            input.impactVelocity = m_Ragdoll.GetExternalImpactImpulse() / sample.totalMass;
        }
        input.impactTriggered = m_PendingRecoveryImpact;
        m_PendingRecoveryImpact = false;

        // 현재 COM의 수평 속력 크기를 계산
        if (sample.hasCenterOfMassVelocity)
        {
            const PxVec3& v = sample.centerOfMassVelocity;
            input.horizontalSpeed = std::sqrt(v.x * v.x + v.z * v.z);
        }

        // 이전 기준 영역 삭제
        m_RecoveryReferenceHull.clear();

        // 애니메이션 목표 발바닥의 위치,회전을 담을 배열을 준비
        std::array<PxTransform, 2> animatedSoles{PxTransform(PxIdentity), PxTransform(PxIdentity)};

        // 기준 계산에 필요한 COM정보가 있는지 확인
        bool referenceValid = sample.hasCenterOfMass && sample.hasCenterOfMassVelocity;

        // 양발 애니메이션,AR 중심 준비
        PxVec3 actualMidpoint(0), animationMidpoint(0);
        std::vector<PxVec3> corners;

        // 애니메이션 발바닥 목표, 실제, 애니메이션 양발의 중점, 발바닥 모서리 좌표를 계산하는 과정
        for (size_t side = 0; side < sample.feet.size(); ++side)
        {
            const FootSoleSample& foot = sample.feet[side];
            if (!foot.available || foot.boneNode < 0 || size_t(foot.boneNode) >= target.size())
            {
                referenceValid = false;
                break;
            }

            // 애니메이션이 요구하는 발바닥의 월드 위치, 회전을 계산
            // 발바닥의 월드변환 = 발 본의 월드변환 * 발바닥의 본기준 로컬 변환
            animatedSoles[side] = ToRecoveryFrame(target[foot.boneNode]) * ToRecoveryFrame(foot.soleInBone);
            actualMidpoint += ToRecoveryFrame(foot.soleWorld).p * .5f;
            animationMidpoint += animatedSoles[side].p * .5f;

            // 발바닥 사각형의 크기를 가져옴
            const FootSoleSettings& sole = m_FootSupport.GetSoleSettings(side);
            const float w = sole.halfWidth, l = sole.halfLength;

            //
            for (const PxVec3& corner : {PxVec3(-w, 0, -l), PxVec3(w, 0, -l), PxVec3(w, 0, l), PxVec3(-w, 0, l)})
            {
                // 발바닥 네 모서리를 월드 좌표로 바꿈
                corners.push_back(animatedSoles[side].transform(corner));
            }
        }

        // COM과 양발 정보등을 확인하여 참일때만 실행
        if (referenceValid)
        {
            // 애니메이션 양발 중심을 실제 양발 중심으로 옮길 이동량을 구함
            const PxVec3 shift = actualMidpoint - animationMidpoint;

            // 애니메이션 목표와 실제발의 위치 차만큼 모서리를 이동시킴
            for (PxVec3& corner : corners)
            {
                corner += shift;
                corner.y = sample.projectedCenterOfMass.y;
            }
            // 발 모서리 4개를 이용해서 다각형을 구함
            m_RecoveryReferenceHull = BuildFootSupportHull(std::move(corners));

            // 최소 3개로 다각형을 이루었는지
            input.hasReference = m_RecoveryReferenceHull.size() >= 3;
            // 현재 COM이 기준 다각형 안에 있는지
            input.insideReference = IsInsideFootSupportHull(m_RecoveryReferenceHull, sample.centerOfMass);
            // 예측 COM이 기준 다각형 안에 있는지
            input.predictionInsideReference = step.hasPrediction && IsInsideFootSupportHull(m_RecoveryReferenceHull, step.predictedCom);
        }

        // m_RecoverySoles : 회복,지지에 사용할 양발 발바닥의 목표 위치,회전
        // m_RecoveryBlendSource : 애니메이션 복귀를 시작할 때 캡처한 물리 본 자세

        // 앞서 정의한 RecoveryInput input;으로 회복 상태를 갱신. 충격, COM, 발딛기, 접지 상태 등을 보고 회복 제어가 현재 상태를 결정
        m_Recovery.Update(input, m_RecoverySettings);
        const RecoverySnapshot& recovery = m_Recovery.GetSnapshot();

        // 지지가 해제되거나 발딛기가 중단됐다면 발 보정 정보를 정리
        if (input.fallen)
        {
            ResetCrossStepPlayback();
            m_HasRecoverySoles = false;
            m_RecoveryBlendSource.clear();
            m_FootstepIK.Reset();
            m_StandingSupport.Reset();
            m_Ragdoll.SetSupportContacts({});
            return true;
        }

        // 새로 회복에 진입했다면, 이전 목표의 속도 계산 이력을 끊음
        // 목표 생성방식이 바뀌면, 이전 목표와  목표 사이에 차이가 생길 수 있음
        if (recovery.enteredRecovery)
        {
            m_Ragdoll.ResetDriveTargetHistory();
            m_RecoveryBlendSource.clear();
        }


        if ((m_Ragdoll.UsesFootDrivenSupport() && !m_HasRecoverySoles) || (recovery.enteredRecovery && !m_Ragdoll.UsesFootDrivenSupport()))
        {
            m_HasRecoverySoles = sample.feet[0].available && sample.feet[1].available;


            for (size_t side = 0; side < sample.feet.size(); ++side)
            {
                m_RecoverySoles[side] = m_Ragdoll.UsesFootDrivenSupport() ? CaptureSupportSole(sample.feet[side]) : ToRecoveryFrame(sample.feet[side].soleWorld);
                m_HasRecoverySoles = m_HasRecoverySoles && m_RecoverySoles[side].isSane();
            }
        }

        // 애니메이션 복귀를 시작하면 현재 물리 자세를 캡처. 복귀 보간 포즈의 출발점으로 사용
        if (recovery.state == RecoveryState::Animation && recovery.startedBlend)
        {
            if (!CaptureRecoveryBlendSource())
            {
                m_PhysicsError = "Cannot capture the current physical pose for animation recovery.";
                return false;
            }
            m_Ragdoll.ResetDriveTargetHistory();
        }

        // 복귀 중에는 캡처한 물리 자세에서 현재 애니메이션 자세로 보간
        if (recovery.state == RecoveryState::Animation && recovery.blending)
        {
            std::vector<GraphNode> pose = GetImpactAnimationPose();
            if (m_RecoveryBlendSource.size() != pose.size() || !BlendRecoveryPose(m_RecoveryBlendSource, pose, recovery.blendAlpha))
            {
                m_PhysicsError = "Invalid physical pose in animation recovery.";
                return false;
            }
            if (!EvaluatePhysicsPose(m_PhysicsSkeleton, pose, Matrix4(GetComponentTransform()), target, m_PhysicsError))
            {
                return false;
            }
        }
        else if (recovery.state == RecoveryState::Animation)
        {
            m_RecoveryBlendSource.clear();
        }

        // 회복 중이거나 발 기반 지지를 사용하면, 발 목표를 계속 갱신
        // 유효한 양발 목표가 있는지 검사하고, 없으면 오류를 반환
        if (recovery.state == RecoveryState::Recovery || m_Ragdoll.UsesFootDrivenSupport())
        {
            if (!m_HasRecoverySoles)
            {
                m_PhysicsError = "Recovery requires both physical foot sole frames.";
                return false;
            }

            ApplyCrossStepDrivePose(sample, deltaTime, target);
            bool changedTargets = false;
            for (size_t side = 0; side < step.feet.size(); ++side)
            {
                const FootstepCommand& command = step.feet[side];
                changedTargets = changedTargets || command.justStarted || command.justLanded;
                if (command.justLanded || (!command.IsInProgress() && sample.feet[side].contact.landed))
                {
                    // 해당 발의 실제 착지점만 갱신하며 반대 발의 목표나 역할은 바꾸지 않는다.
                    m_RecoverySoles[side] = m_Ragdoll.UsesFootDrivenSupport() ? CaptureSupportSole(sample.feet[side]) :
                        ToRecoveryFrame(sample.feet[side].soleWorld);
                }
                else if (command.motion.valid)
                {
                    PxTransform sole(PxIdentity);
                    const FootSoleSample& foot = sample.feet[side];
                    const PxTransform actualSole = ToRecoveryFrame(foot.soleWorld);
                    // 다음 물리 틱 끝에 도달할 자세를 관절 PD에 전달한다.
                    const PxVec3 drivePosition = command.motion.hasSupportDeadline ?
                        SampleFootstepPosition(command, deltaTime) : command.motion.position;
                    if (MakeFootSoleTarget(drivePosition, command.target.normal, actualSole, sole,
                        command.target.position - command.startPosition))
                    {
                        m_RecoverySoles[side] = sole;
                    }
                }
            }

            // 시작,착지 때 목표 변화율 계산 이력을 초기화
            if (m_Ragdoll.UsesFootDrivenSupport() && changedTargets)
            {
                // 이륙/착지 때 실제 발로 기준을 다시 잡는 차이를 목표 속도·가속도로 오인하지 않는다.
                // 현재 물리 속도는 유지하므로 관절 PD의 제동은 계속 작동한다.
                m_Ragdoll.ResetDriveTargetHistory();
            }

            // 저장한 발바닥의 목표를 다리의 본 목표로 변환함
            // 다리 IK를 적용해서 target을 수정함
            ApplyFootstepIKTargets(target, deltaTime);

            // IK목표를 정상적으로 계산했는지 확인
            if (m_FootstepIK.GetSnapshot().status != FootstepIKStatus::Applied)
            {
                m_PhysicsError = "Recovery leg IK could not produce valid drive targets.";
                return false;
            }
            return true;
        }

        ResetCrossStepPlayback();
        m_FootstepIK.Reset();
        m_HasRecoverySoles = false;
        return true;
    }
}
