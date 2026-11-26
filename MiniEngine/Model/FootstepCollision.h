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
        const FootstepPelvisTurn* pelvisTurn = nullptr;
        size_t side = 0;
        FootstepCommand command;
        // 허벅지, 종아리, 발 순서이며 현재 PA의 실제 충돌 형상을 복사한다.
        std::array<FootstepCollisionShape, 3> shapes;
        TwoBoneIKInput pose;
        FootstepPath path;
        physx::PxVec3 pelvisPivot{0};
        float turnElapsed = 0;
        float turnRate = 1;
    };

    struct FootstepCollisionInput
    {
    public:
        // 양쪽 다리가 동시에 회전하는 궤적은 고정 지지발 경로보다 더 잘게 나눠야 한다.
        static constexpr uint32_t PelvisTurnMaximumIntervals = 512;

    public:
        // 런타임 관절 제한에 의해 목표가 잘리는지 확인한다. 후보 검사에서는 상태를 바꾸지 않는다.
        std::function<bool(size_t, const LegIKResult&, float)> acceptsPose;
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

    // 골반 회전이 있으면 지지 발바닥을 고정하고 양쪽 다리의 궤적을 검사한다. 기본 궤적은 지지 다리를 고정한다.
    // 이 모델의 시간 구간 전체가 분리되어야 통과하며, 예산·수치 한계는 미확인으로 반환한다.
    // 관측한 추종 오차와 외력에 의한 자세 변화를 반영하기 위해 매 물리 틱 다시 검사한다.
    // 관절 제한 콜백과 도달 제한은 평가한 자세에서 확인하는 별도 후보 거절 조건이다.
    FootstepCollisionSnapshot PredictFootstepCollisions(const FootstepCollisionInput& input,
        FootstepCollisionPass pass = FootstepCollisionPass::CurrentPath);
}
