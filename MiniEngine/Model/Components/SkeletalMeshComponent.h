#pragma once

#include "SkinnedMeshComponent.h"
#include "../AnimInstance.h"
#include "../PhysicsAsset.h"
#include "../PhysicsPose.h"
#include "../Ragdoll.h"
#include "../FootSupport.h"
#include "../FootstepIK.h"
#include "../FootstepAvoidance.h"
#include "../RagdollRecovery.h"
#include "../StandingSupport.h"
#include "../HorizontalBalance.h"
#include "../FallArmBrace.h"

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
        Error,
        ProjectileImpact
    };

    enum class FallDirection : uint8_t
    {
        Forward,
        Backward,
        Left,
        Right,
        Count
    };

    enum class CharacterBalanceState : uint8_t
    {
        Grounded,
        Unbalanced,
        GroundFall,
        Falling,
        Landed
    };

    const char* CharacterBalanceStateLabel(CharacterBalanceState state);

    struct CrossStepPlayback
    {
    public:
        uint64_t startedStep = UINT64_MAX;
        uint64_t revision = UINT64_MAX;
        uint64_t blendStartedStep = UINT64_MAX;
        MotionMatchResult match;
        uint32_t movingFoot = UINT32_MAX;
        float timeSeconds = 0.0f;
        float weight = 0.0f;
    };

    class SkeletalMeshComponent : public SkinnedMeshComponent
    {
    private:
        enum class StandingUpState : uint8_t
        {
            None,
            Kinematic,
            IdleBlend
        };

        struct ImpactFlailPlayback
        {
        public:
            std::vector<GraphNode> pose;
            uint32_t clip = UINT32_MAX;
            float time = 0;
            float weight = 0;
            float impactTimeRemaining = 0;
        };

        struct ImpactHandReach
        {
        public:
            uint64_t step = 0;
            uint64_t approachStep = 0;
            uint64_t clock = 0;
            physx::PxVec3 pointInBone{0};
            physx::PxVec3 normalInBone{0};
            int body = -1;
            int hand = -1;
            int woundJoint = -1;
        };

        struct ImpactWarmup
        {
        public:
            bool ApplyCapsuleImpacts(const std::vector<RagdollBodyImpulse>& impacts);
            bool ApplyTorsoLean(const PhysicsSkeleton& skeleton, const physx::PxQuat& currentRotation,
                float elapsed, std::string& error);
            ImpactWarmupStep PlanStep(const FootstepPolygon& hull, const physx::PxVec3& com, const physx::PxVec3& velocity,
                bool secondStep) const;

        public:
            float GetLandingSoleYaw(size_t stepIndex) const;

        public:
            static constexpr float MinimumImpactSpeedChange = .25f;

        public:
            Math::AffineTransform actorAtStart{Math::kIdentity};
            std::vector<GraphNode> sourcePose;
            std::vector<GraphNode> pose;
            std::vector<Math::Matrix4> sourceBones;
            std::vector<int> spineNodes;
            RagdollImpactReaction reaction;
            std::array<TwoBoneIKInput, 2> legs;
            std::array<ImpactWarmupStep, 2> steps;
            physx::PxQuat componentRotation{physx::PxIdentity};
            physx::PxVec3 restVelocity{0};
            physx::PxVec3 momentum{0};
            physx::PxVec3 approachVelocity{0};
            physx::PxVec3 sweepStart{0};
            physx::PxVec3 sweepCenter{0};
            physx::PxVec3 turnPivotInComponent{0};
            physx::PxVec3 peakTorsoLean{0};
            float mass = 0;
            float torsoMass = 0;
            float slideDeceleration = 0;
            float stepTime = 0;
            float turnYaw = 0;
            uint32_t approachGroup = 0;
            uint32_t firstFoot = 0;
            uint32_t landings = 0;
            bool wasResting = false;
        };

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
        // 명시적인 지지 모드 변경은 제어기를 다시 준비한다.
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
        bool EvaluateAnimationPhysicsPose(const PhysicsSkeleton& skeleton, std::vector<Math::Matrix4>& worldPose,
            std::string& error, const std::vector<GraphNode>* poseOverride = nullptr) const;
        void DestroyPhysicsState();
        bool UpdateKinematicPose(bool teleport);
        void UpdatePhysicsPose();
        void UpdateFootSupport();
        void UpdateHorizontalBalance(float deltaTime);
        void ApplyCrossStepDrivePose(const FootSupportSnapshot& support, float deltaTime, std::vector<Math::Matrix4>& target);
        void ResetCrossStepPlayback();
        void UpdateImpactFlail(bool triggerReaction, float deltaTime);
        void ApplyFootstepIKTargets(std::vector<Math::Matrix4>& target, float deltaTime);
        void UpdateFootstepCollision(FootstepIKInput& input,
            const std::vector<physx::PxTransform>& sourcePose, float deltaTime);
        bool ApplyRecoveryTargets(std::vector<Math::Matrix4>& target, float deltaTime);
        void ResetRecovery();
        bool CaptureRecoveryBlendSource();
        void UpdateFootCandidateDebugColors();
        bool UpdateAutomaticRootSupport(std::string& error);
        bool UpdateAutomaticRagdoll(float deltaTime, bool firstHit);
        void ReplayKinematicImpacts(const std::vector<RagdollBodyImpulse>& impacts, float deltaTime);
        bool UpdateImpactWarmup(float deltaTime, const std::vector<RagdollImpactResponse>& responses);
        bool BeginImpactWarmup(const std::vector<RagdollImpactResponse>& responses, float deltaTime);
        bool AdvanceImpactWarmup(float deltaTime);
        bool FinishImpactWarmup(bool lostFloor, bool replayCapsuleImpacts = false);
        bool UpdateBalanceState(float deltaTime);
        bool BeginAirborneFall();
        void ResetAirborneFall();
        bool UpdateFallingAnimation(float deltaTime);
        bool UpdateStandingUpAnimation(float deltaTime);
        bool BeginKinematicStandingUp(float groundHeight);
        bool BeginKinematicRecovery();
        bool UpdateKinematicRecoveryPose(float deltaTime);
        void InitializeFallArms();
        ImpactHandReach FindImpactHandReach(const std::vector<RagdollBodyImpulse>& impacts) const;
        void ApplyImpactHandReach(const PhysicsWorld& physics, const std::array<FallArmSample, 2>& arms,
            std::vector<physx::PxTransform>& frames, FallArmSnapshot& corrected);
        void ApplyArmTargets(std::vector<Math::Matrix4>& target);
        void ResetFallArms();

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
        bool IsImpactWarmupActive() const
        {
            return !m_ImpactWarmup.pose.empty();
        }
        uint32_t GetImpactWarmupLandings() const
        {
            return m_ImpactWarmup.landings;
        }
        const std::array<ImpactWarmupStep, 2>& GetImpactWarmupSteps() const
        {
            return m_ImpactWarmup.steps;
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
        const Ragdoll& GetRagdoll() const
        {
            return m_Ragdoll;
        }
        bool GetImpactPalmDistance(int& hitBody, float& meters) const;
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
        const FootstepCollisionSnapshot& GetFootstepCollisionSnapshot() const
        {
            return m_FootstepCollision;
        }
        const FootstepAvoidanceSnapshot& GetFootstepAvoidanceSnapshot() const
        {
            return m_FootstepAvoidance.GetSnapshot();
        }
        const RecoverySnapshot& GetRecoverySnapshot() const
        {
            return m_Recovery.GetSnapshot();
        }
        const RecoverySettings& GetRecoverySettings() const
        {
            return m_RecoverySettings;
        }
        bool SetFallingAnimation(uint32_t animationIndex);
        bool SetDirectionalFallingAnimation(FallDirection direction, uint32_t animationIndex);
        bool SetAirborneFallAnimations(const std::array<uint32_t, 3>& clips);
        void SetFallingAnimationEnabled(bool enabled)
        {
            m_FallingAnimationEnabled = enabled;
        }
        bool IsFallingAnimationEnabled() const
        {
            return m_FallingAnimationEnabled;
        }
        bool IsFalling() const
        {
            return m_BalanceState == CharacterBalanceState::Falling;
        }
        CharacterBalanceState GetBalanceState() const
        {
            return m_BalanceState;
        }
        FallDirection GetGroundFallDirection() const;
        bool IsFollowingFallingAnimation() const;
        bool SetStandingUpAnimation(uint32_t animationIndex, uint32_t idleAnimationIndex);
        bool IsFollowingStandingUpAnimation() const;
        bool UsesAutomaticRagdoll() const
        {
            return m_AutomaticRagdoll;
        }
        void SetAutomaticRagdoll(bool enabled);
        float GetKinematicStableTime() const
        {
            return m_KinematicRecoveryTimer.GetStableTime();
        }
        float GetKinematicStableDuration() const
        {
            return m_KinematicStableDuration;
        }
        void SetKinematicStableDuration(float duration);
        bool IsBlendingToKinematic() const
        {
            return !m_KinematicBlendPose.empty();
        }
        const StandingSupportSnapshot& GetStandingSupportSnapshot() const
        {
            return m_StandingSupport.GetSnapshot();
        }
        bool SetArmDriveStrength(float strength);
        float GetArmDriveStrength() const
        {
            return m_ArmDriveStrength;
        }
        uint32_t GetImpactFlailClip() const
        {
            return m_ImpactFlail.clip;
        }
        float GetImpactFlailTime() const
        {
            return m_ImpactFlail.time;
        }
        const HorizontalBalanceSnapshot& GetHorizontalBalanceSnapshot() const
        {
            return m_HorizontalBalance.GetSnapshot();
        }
        HorizontalBalanceSettings& GetHorizontalBalanceSettings()
        {
            return m_HorizontalBalanceSettings;
        }
        const HorizontalBalanceSettings& GetHorizontalBalanceSettings() const
        {
            return m_HorizontalBalanceSettings;
        }
        StandingSupportSettings& GetStandingSupportSettings()
        {
            return m_StandingSupportSettings;
        }
        const StandingSupportSettings& GetStandingSupportSettings() const
        {
            return m_StandingSupportSettings;
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
            if (IsImpactWarmupActive() && !IsRagdoll())
            {
                return m_ImpactWarmup.pose;
            }
            return IsRagdoll() ? m_PhysicsPose : IsBlendingToKinematic() ? m_KinematicBlendPose : m_AnimInstance->GetPose();
        }
        const ModelInstance& GetModelInstance() const
        {
            return m_ModelInstance;
        }

    private:
        bool IsGroundFallActive() const
        {
            return m_BalanceState == CharacterBalanceState::GroundFall || m_BalanceState == CharacterBalanceState::Landed;
        }
        bool IsFallingClip(uint32_t animationIndex) const;
        bool IsAirborneFallClip(uint32_t animationIndex) const;
        uint32_t GetIdleAnimationIndex() const;
        MotionMatchResult SelectFallingAnimation() const;
        const std::vector<GraphNode>& GetImpactAnimationPose() const
        {
            return m_ImpactFlail.pose.empty() ? m_AnimInstance->GetPose() : m_ImpactFlail.pose;
        }
        const std::vector<GraphNode>& GetRagdollAnimationPose() const
        {
            return m_CrossStepPose.empty() ? GetImpactAnimationPose() : m_CrossStepPose;
        }

    private:
        ModelInstance m_ModelInstance;
        FootSupport m_FootSupport;
        ImpactWarmup m_ImpactWarmup;
        std::unique_ptr<AnimInstance> m_AnimInstance;
        std::unique_ptr<const PhysicsAsset> m_PhysicsAsset;
        Ragdoll m_Ragdoll;
        FootstepIK m_FootstepIK;
        FootstepCollisionSnapshot m_FootstepCollision;
        FootstepAvoidance m_FootstepAvoidance;
        RagdollRecovery m_Recovery;
        KinematicRecoveryTimer m_KinematicRecoveryTimer;
        AirborneFallTimer m_AirborneFallTimer;
        StandingSupport m_StandingSupport;
        HorizontalBalance m_HorizontalBalance;
        ImpactFlailPlayback m_ImpactFlail;
        ImpactHandReach m_ImpactHandReach;
        std::vector<physx::PxVec3> m_RecoveryReferenceHull;
        std::vector<GraphNode> m_RecoveryBlendSource;
        std::vector<GraphNode> m_CrossStepPose;
        CrossStepPlayback m_CrossStepPlayback;
        std::vector<GraphNode> m_KinematicBlendSource;
        std::vector<GraphNode> m_KinematicBlendPose;
        PhysicsSkeleton m_PhysicsSkeleton;
        std::vector<GraphNode> m_RagdollSourcePose;
        std::vector<GraphNode> m_PhysicsPose;
        std::vector<Math::Matrix4> m_BoneStretch;
        std::string m_PhysicsError;
        std::unique_ptr<const PhysicsAsset> m_PendingDriveSettings;
        RecoverySettings m_RecoverySettings;
        StandingSupportSettings m_StandingSupportSettings;
        HorizontalBalanceSettings m_HorizontalBalanceSettings;
        std::array<FallArmSetup, 2> m_FallArmSetups;
        std::array<physx::PxTransform, 2> m_RecoverySoles{
            physx::PxTransform(physx::PxIdentity), physx::PxTransform(physx::PxIdentity)};
        RootSupportRelease m_RootSupportRelease = RootSupportRelease::None;
        PhysicsAnimationMode m_PhysicsMode = PhysicsAnimationMode::Animation;
        float m_KinematicStableDuration = 2.0f;
        float m_ArmDriveStrength = 1.0f;
        float m_KinematicBlendTime = 0;
        float m_StandingUpStableTime = 0;
        uint32_t m_FallingAnimationIndex = UINT32_MAX;
        std::array<uint32_t, static_cast<size_t>(FallDirection::Count)> m_DirectionalFallingAnimations;
        std::array<uint32_t, 3> m_AirborneFallAnimations{UINT32_MAX, UINT32_MAX, UINT32_MAX};
        uint32_t m_StandingUpAnimationIndex = UINT32_MAX;
        uint32_t m_StandingIdleAnimationIndex = UINT32_MAX;
        bool m_HasRecoverySoles = false;
        bool m_FootCandidateDebugVisible = false;
        bool m_AutomaticRootSupport = false;
        bool m_AutomaticRagdoll = false;
        bool m_PendingRecoveryImpact = false;
        bool m_FallArmsInitialized = false;
        bool m_FallingAnimationEnabled = true;
        StandingUpState m_StandingUpState = StandingUpState::None;
        CharacterBalanceState m_BalanceState = CharacterBalanceState::Grounded;
    };
}
