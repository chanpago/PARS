#pragma once

#include "FootstepIK.h"
#include "FootstepPath.h"
#include "geometry/PxGeometryHelpers.h"
#include <functional>

namespace PARS
{
    constexpr uint32_t FootstepCollisionSampleCount = 6;

    struct FootstepCollisionShape
    {
    public:
        physx::PxGeometryHolder geometry;
        physx::PxTransform inBone{physx::PxIdentity};
        physx::PxTransform actualPose{physx::PxIdentity};
        bool available = false;
    };

    struct FootstepCollisionLeg
    {
    public:
        FootstepCommand command;
        // 허벅지, 종아리, 발 순서이며 현재 PA의 실제 충돌 형상을 복사한다.
        std::array<FootstepCollisionShape, 3> shapes;
        TwoBoneIKInput pose;
        FootstepPath path;
    };

    struct FootstepCollisionInput
    {
    public:
        // 물리 구동 시 필요한 관절 보정과 계산할 수 없는 관절 자세를 구분한다.
        std::function<FootstepJointStatus(size_t, const LegIKResult&)> checkJoints;
        uint64_t completedStep = 0;
        uint64_t revision = 0;
        std::array<FootstepCollisionLeg, 2> legs;
        float fixedDeltaTime = 0;
        float clearance = .005f;
        uint32_t maximumIntervals = 128;
        bool includeTrackingError = true;
        bool rejectInvalidPoses = false;
    };

    enum class FootstepCollisionStatus
    {
        Inactive,
        Unavailable,
        Clear,
        Collision,
        Uncertain
    };

    struct FootstepCollisionSample
    {
    public:
        std::array<physx::PxVec3, 2> solePositions{physx::PxVec3(0), physx::PxVec3(0)};
        float timeAhead = 0;
        // 비트 번호는 왼쪽 부위 * 3 + 오른쪽 부위다. 실제 접촉 마스크와 같은 순서다.
        uint32_t overlapPairs = 0;
        bool reachLimited = false;
        bool valid = false;
    };

    struct FootstepCollisionSnapshot
    {
    public:
        bool IsReachableAndClear() const
        {
            return status == FootstepCollisionStatus::Clear && !reachLimited;
        }

    public:
        uint64_t completedStep = 0;
        uint64_t revision = 0;
        double elapsedMilliseconds = 0;
        std::array<FootstepCollisionSample, FootstepCollisionSampleCount> samples;
        uint32_t sampleCount = 0;
        uint32_t queryCount = 0;
        uint32_t overlapPairs = 0;
        uint32_t uncertainPairs = 0;
        uint32_t intervalCount = 0;
        float maximumTrackingError = 0;
        float firstRiskTime = 0;
        FootstepCollisionStatus status = FootstepCollisionStatus::Inactive;
        std::array<bool, 2> moving{};
        bool assumesTakeoff = false;
        bool reachLimited = false;
        bool jointLimited = false;
        bool budgetExhausted = false;
        bool fixedPoseBlocked = false;
    };

    enum class FootstepCollisionPass
    {
        CurrentPath,
        DetourCandidate
    };

    // 한 호출 안에서는 골반·지지 다리를 고정한 계획 궤적과 관측한 추종 오차 범위를 검사한다.
    // 이 모델의 시간 구간 전체가 분리되어야 통과하며, 예산·수치 한계는 미확인으로 반환한다.
    // 미래 외력이나 지지 다리의 운동을 보장하지 않으므로 매 물리 틱 다시 검사해야 한다.
    // 관절 보정 후에도 남은 발 목표 오차는 도달 판정에 반영하고 보정 발생 여부를 기록한다.
    FootstepCollisionSnapshot PredictFootstepCollisions(const FootstepCollisionInput& input,
        FootstepCollisionPass pass = FootstepCollisionPass::CurrentPath);
}
