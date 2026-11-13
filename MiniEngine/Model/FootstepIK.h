#pragma once

#include "foundation/PxTransform.h"
#include <array>
#include <cstdint>
#include <functional>
#include <vector>

namespace PARS
{
    struct FootstepReachability;
    struct FootstepSettings;
    enum class KneeHintSource
    {
        Animation,
        Previous,
        Reference,
        Fallback
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
        bool hasPreviousBend = false;
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
        // 물리 구동과 동일한 관절 좌표계의 제한 적용 후 회전이다.
        std::array<physx::PxQuat, 3> jointRotations{
            physx::PxQuat(physx::PxIdentity), physx::PxQuat(physx::PxIdentity), physx::PxQuat(physx::PxIdentity)};
        float reachError = 0;
        float initialLimitedReachError = 0;
        float initialLimitedTiltError = 0;
        float soleTiltError = 0; // 목표 발바닥 법선과의 각도 오차(rad)
        uint32_t limitIterations = 0;
        bool valid = false;
        bool reachLimited = false;
        bool jointLimitsApplied = false;
        bool limitConverged = false;
    };

    // 발바닥의 앞 방향은 주어진 발 자세에서, 위 방향은 지지 평면에서 얻는다.
    bool MakeFootSoleTarget(const physx::PxVec3& position, const physx::PxVec3& normal,
        const physx::PxTransform& referenceSole, physx::PxTransform& target);
    bool MakeFootstepSoleMotion(const physx::PxTransform& start, const physx::PxVec3& position,
        const physx::PxVec3& normal, float progress, physx::PxTransform& target);
    LegIKResult SolveTwoBoneIK(const TwoBoneIKInput& input);

    struct FootIKJointFrame
    {
    public:
        // PA 바디 오프셋까지 포함한 본 기준 관절 프레임이다.
        physx::PxTransform parentInBone{physx::PxIdentity};
        physx::PxTransform childInBone{physx::PxIdentity};
    };

    struct FootIKLegInput
    {
    public:
        // 고관절·무릎·발목 순서이며, 실제 물리 구동의 제한 함수를 호출한다.
        std::function<physx::PxQuat(size_t, const physx::PxQuat&)> clampJointRotation;
        int anchorNode = -1;
        int hipNode = -1;
        int kneeNode = -1;
        int ankleNode = -1;
        int toeNode = -1;
        float thighLength = 0;
        float calfLength = 0;
        physx::PxVec3 referenceBendInHip{0};
        physx::PxTransform actualAnchor{physx::PxIdentity};
        physx::PxTransform soleInBone{physx::PxIdentity};
        physx::PxTransform targetSole{physx::PxIdentity};
        std::array<FootIKJointFrame, 3> joints;
        // 지지 중인 Toe의 월드 회전 목표다. 본의 로컬 위치와 길이는 IK 후에도 보존한다.
        physx::PxQuat toeTargetRotation{physx::PxIdentity};
        bool available = false;
        bool hasToeTarget = false;
    };

    struct FootstepIKInput
    {
    public:
        uint64_t revision = 0;
        std::array<FootIKLegInput, 2> legs;
        physx::PxTransform plannedPelvis{physx::PxIdentity};
        // -1은 새 발딛기 없이 회복 중 양발 목표를 유지하는 경우다.
        int stepFoot = -1;
        bool enabled = false;
        bool hasPelvisPlan = false;
    };

    // 후보 검사와 실행 IK가 같은 실제 관절 프레임을 사용한다.
    struct FootstepPlanningInput
    {
    public:
        std::array<FootIKLegInput, 2> legs;
        std::array<std::array<physx::PxTransform, 3>, 2> bones;
        std::array<std::array<physx::PxVec3, 3>, 2> jointVelocities;
        std::array<std::array<physx::PxVec3, 3>, 2> jointAccelerations;
        std::array<physx::PxTransform, 2> soles;
        std::array<physx::PxTransform, 2> supportSoles;
        physx::PxVec3 pelvisVelocity{0};
        physx::PxVec3 pelvisAngularVelocity{0};
        physx::PxVec3 forward{0, 0, 1};
        float maximumLowering = 0;
        float loweringSpeed = 0;
        float loweringScale = 0;
        std::array<std::array<std::array<bool, 3>, 3>, 2> lockedAxes{};
    };

    LegIKResult SolveFootIKLeg(const FootIKLegInput& leg,
        const std::array<physx::PxTransform, 3>& bones, const LegIKResult& previous);
    FootstepReachability EvaluateFootstepReachability(const FootstepPlanningInput& input,
        const FootstepSettings& settings, size_t side, const physx::PxVec3& position,
        const physx::PxVec3& normal, float duration, float liftHeight);
    physx::PxTransform InterpolateFootstepPelvis(const FootstepReachability& plan, float progress);

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
        int stepFoot = -1;
        std::array<LegIKResult, 2> legs;
    };

    // 실제 또는 계획 골반에서 IK를 풀어 관절 목표를 만든다. 물리 바디는 직접 옮기지 않는다.
    class FootstepIK
    {
    public:
        void Reset();
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
