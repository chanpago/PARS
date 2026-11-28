#pragma once

#include "foundation/PxTransform.h"
#include <array>
#include <cstdint>
#include <vector>

namespace PARS
{
    struct PhysicsConstraint;

    constexpr float FootstepSoleTolerance = .005f;
    constexpr float FootstepSoleAngleTolerance = .01f;

    struct FootIKJointLimits
    {
    public:
        using ProjectRotation = physx::PxQuat (*)(const PhysicsConstraint&, const physx::PxQuat&);
        using ContainsRotations = bool (*)(const PhysicsConstraint&, physx::PxQuat, float);

    public:
        ProjectRotation project = nullptr;
        ContainsRotations contains = nullptr;
        std::array<const PhysicsConstraint*, 3> joints{};
        // 고관절, 무릎, 발목 순서의 PA 관절축을 부모 본과 자식 본 기준으로 저장한다.
        std::array<physx::PxQuat, 3> parentFrames{{physx::PxQuat(physx::PxIdentity),
            physx::PxQuat(physx::PxIdentity), physx::PxQuat(physx::PxIdentity)}};
        std::array<physx::PxQuat, 3> childFrames{{physx::PxQuat(physx::PxIdentity),
            physx::PxQuat(physx::PxIdentity), physx::PxQuat(physx::PxIdentity)}};
        physx::PxQuat anchorRotation{physx::PxIdentity};
    };

    enum class KneeHintSource
    {
        Animation,
        Previous,
        Reference,
        Fallback
    };

    enum class FootIKIssue : uint8_t
    {
        None,
        TargetPosition,
        GroundNormal,
        ReferenceSole,
        SoleOrientation,
        HipPose,
        KneePose,
        AnklePose,
        SoleTransform,
        ThighLength,
        CalfLength,
        ReachRatio,
        TargetAnkle,
        ThighDirection,
        CalfDirection,
        TargetDistance,
        SolvedPose
    };

    struct TwoBoneIKInput
    {
    public:
        const FootIKJointLimits* jointLimits = nullptr;
        physx::PxTransform hip{physx::PxIdentity};
        physx::PxTransform knee{physx::PxIdentity};
        physx::PxTransform ankle{physx::PxIdentity};
        physx::PxTransform targetSole{physx::PxIdentity};
        physx::PxTransform soleInBone{physx::PxIdentity};
        physx::PxVec3 referenceBend{0};
        physx::PxVec3 previousBend{0};
        float thighLength = 0;
        float calfLength = 0;
        // 팔 보호 동작은 팔꿈치 방향을 지정하고 완전 신전 전에 여유를 남긴다.
        float maximumReachRatio = 1.0f;
        bool hasPreviousBend = false;
        bool preferReferenceBend = false;
    };

    struct LegIKResult
    {
    public:
        KneeHintSource hintSource = KneeHintSource::Fallback;
        physx::PxTransform hip{physx::PxIdentity};
        physx::PxTransform knee{physx::PxIdentity};
        physx::PxTransform ankle{physx::PxIdentity};
        physx::PxTransform requestedSole{physx::PxIdentity};
        physx::PxTransform solvedSole{physx::PxIdentity};
        physx::PxVec3 requestedAnkle{0};
        physx::PxVec3 bendDirection{0};
        float reachError = 0;
        float soleAngleError = 0;
        FootIKIssue issue = FootIKIssue::None;
        bool valid = false;
        bool reachLimited = false;
        bool jointAdjusted = false;
    };

    // 캐릭터 정면과 보정 전 COM 예측선 사이의 수평 각도로 보폭 방향을 따를 비율을 정한다.
    float GetFootstepStrideWeight(const physx::PxVec3& characterForward, const physx::PxVec3& predictedComDelta);
    // 원래 발끝 방향에서 보폭 축의 가까운 쪽까지 수평 회전각을 보간한다.
    physx::PxVec3 GetFootstepSoleForward(const physx::PxVec3& displacement, const physx::PxVec3& stanceForward,
        float strideWeight);
    // 지정한 발끝 방향을 착지면에 맞춘다. 방향을 지정하지 않으면 기준 발바닥 자세를 사용한다.
    FootIKIssue BuildFootSoleTarget(const physx::PxVec3& position, const physx::PxVec3& normal,
        const physx::PxTransform& referenceSole, physx::PxTransform& target, const physx::PxVec3& soleForward);
    bool MakeFootSoleTarget(const physx::PxVec3& position, const physx::PxVec3& normal,
        const physx::PxTransform& referenceSole, physx::PxTransform& target,
        const physx::PxVec3& soleForward = physx::PxVec3(0));
    LegIKResult SolveTwoBoneIK(const TwoBoneIKInput& input);
    void ConstrainFootstepLeg(const FootIKJointLimits& limits, const physx::PxTransform& soleInBone, LegIKResult& result);
    float PredictSupportReachTime(const physx::PxVec3& hipFromAnkle,
        const physx::PxVec3& hipVelocity, float thighLength, float calfLength);

    struct FootIKLegInput
    {
    public:
        FootIKJointLimits jointLimits;
        int anchorNode = -1;
        int hipNode = -1;
        int kneeNode = -1;
        int ankleNode = -1;
        float thighLength = 0;
        float calfLength = 0;
        physx::PxVec3 referenceBendInHip{0};
        physx::PxTransform actualAnchor{physx::PxIdentity};
        physx::PxTransform soleInBone{physx::PxIdentity};
        physx::PxTransform targetSole{physx::PxIdentity};
        bool available = false;
    };

    struct FootstepIKInput
    {
    public:
        uint64_t revision = 0;
        std::array<FootIKLegInput, 2> legs;
        std::array<bool, 2> motionTargets{};
        bool enabled = false;
    };

    enum class FootstepIKStatus
    {
        Inactive,
        Applied,
        InvalidPose,
        InvalidLeg,
        InvalidTarget
    };

    struct FootstepIKSnapshot
    {
    public:
        uint64_t revision = 0;
        std::vector<int> correctedNodes;
        FootstepIKStatus status = FootstepIKStatus::Inactive;
        std::array<LegIKResult, 2> legs;
        // 다리별 IK 해를 구할 때 기준으로 삼은 물리 부모 본이다.
        std::array<int, 2> anchorNodes{-1, -1};
        std::array<bool, 2> motionTargets{};
    };

    // 실제 부모 좌표에서 IK를 푼 뒤 애니메이션 부모 기준 목표로 되돌린다. 물리 바디는 옮기지 않는다.
    class FootstepIK
    {
    public:
        void Reset();
        bool BuildLegInput(const FootstepIKInput& input, size_t side,
            const std::vector<physx::PxTransform>& pose, TwoBoneIKInput& result) const;
        bool Apply(const FootstepIKInput& input, const std::vector<int32_t>& parents,
            std::vector<physx::PxTransform>& pose);

    public:
        const FootstepIKSnapshot& GetSnapshot() const
        {
            return m_Snapshot;
        }

    private:
        FootstepIKSnapshot m_Snapshot;
        std::array<physx::PxVec3, 2> m_PreviousBendInAnchor{physx::PxVec3(0), physx::PxVec3(0)};
        std::array<bool, 2> m_HasPreviousBend{};
    };

    const char* FootstepIKStatusLabel(FootstepIKStatus status);
    const char* KneeHintSourceLabel(KneeHintSource source);
}
