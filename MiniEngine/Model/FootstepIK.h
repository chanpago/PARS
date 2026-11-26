#pragma once

#include "foundation/PxTransform.h"
#include <array>
#include <cstdint>
#include <vector>

namespace PARS
{
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
        FootIKIssue issue = FootIKIssue::None;
        bool valid = false;
        bool reachLimited = false;
    };

    // 발끝과 가까운 쪽을 택해 보폭의 수평축을 정한다. 수평 이동이 없으면 영벡터를 반환한다.
    physx::PxVec3 GetFootstepTravelForward(const physx::PxVec3& displacement, const physx::PxVec3& referenceForward);
    // 이동발은 보폭 방향을, 제자리에서는 기준 발 방향을 지면에 맞춘다.
    FootIKIssue BuildFootSoleTarget(const physx::PxVec3& position, const physx::PxVec3& normal,
        const physx::PxTransform& referenceSole, physx::PxTransform& target, const physx::PxVec3& displacement);
    bool MakeFootSoleTarget(const physx::PxVec3& position, const physx::PxVec3& normal,
        const physx::PxTransform& referenceSole, physx::PxTransform& target,
        const physx::PxVec3& displacement = physx::PxVec3(0));
    LegIKResult SolveTwoBoneIK(const TwoBoneIKInput& input);
    float PredictSupportReachTime(const physx::PxVec3& hipFromAnkle,
        const physx::PxVec3& hipVelocity, float thighLength, float calfLength);

    struct FootIKLegInput
    {
    public:
        const TwoBoneIKInput* turnReference = nullptr;
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
