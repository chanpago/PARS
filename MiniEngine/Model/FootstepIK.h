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

    struct TwoBoneIKInput
    {
        physx::PxTransform hip{physx::PxIdentity}, knee{physx::PxIdentity}, ankle{physx::PxIdentity};
        physx::PxTransform targetSole{physx::PxIdentity}, soleInBone{physx::PxIdentity};
        physx::PxVec3 referenceBend{0}, previousBend{0};
        float thighLength = 0, calfLength = 0;
        bool hasPreviousBend = false;
    };

    struct LegIKResult
    {
        bool valid = false, reachLimited = false;
        KneeHintSource hintSource = KneeHintSource::Fallback;
        physx::PxTransform hip{physx::PxIdentity}, knee{physx::PxIdentity}, ankle{physx::PxIdentity};
        physx::PxTransform requestedSole{physx::PxIdentity}, solvedSole{physx::PxIdentity};
        physx::PxVec3 requestedAnkle{0}, bendDirection{0};
        float reachError = 0;
    };

    // 발바닥의 앞 방향은 애니메이션에서, 위 방향은 지지 평면에서 얻는다.
    bool MakeFootSoleTarget(const physx::PxVec3& position, const physx::PxVec3& normal,
        const physx::PxTransform& animatedSole, physx::PxTransform& target);
    LegIKResult SolveTwoBoneIK(const TwoBoneIKInput& input);

    struct FootIKLegInput
    {
        bool available = false;
        int anchorNode = -1, hipNode = -1, kneeNode = -1, ankleNode = -1;
        float thighLength = 0, calfLength = 0;
        physx::PxVec3 referenceBendInHip{0};
        physx::PxTransform actualAnchor{physx::PxIdentity};
        physx::PxTransform soleInBone{physx::PxIdentity}, targetSole{physx::PxIdentity};
    };

    struct FootstepIKInput
    {
        bool enabled = false;
        int stepFoot = -1;
        uint64_t revision = 0;
        std::array<FootIKLegInput, 2> legs;
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
        FootstepIKStatus status = FootstepIKStatus::Inactive;
        uint64_t revision = 0;
        int stepFoot = -1;
        std::array<LegIKResult, 2> legs;
        std::vector<int> correctedNodes;
    };

    // 실제 부모 좌표에서 IK를 푼 뒤 애니메이션 부모 기준 목표로 되돌린다. 물리 바디는 옮기지 않는다.
    class FootstepIK
    {
    public:
        void Reset();
        bool Apply(const FootstepIKInput& input, const std::vector<int32_t>& parents,
            std::vector<physx::PxTransform>& pose);
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
