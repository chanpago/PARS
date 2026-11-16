#pragma once

#include "FootstepIK.h"
#include "FootstepPlanner.h"
#include "geometry/PxGeometryHelpers.h"

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
    };

    struct FootstepCollisionInput
    {
    public:
        uint64_t completedStep = 0;
        uint64_t revision = 0;
        std::array<FootstepCollisionLeg, 2> legs;
        float fixedDeltaTime = 0;
    };

    enum class FootstepCollisionStatus
    {
        Inactive,
        Unavailable,
        Sampled
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
        FootstepCollisionStatus status = FootstepCollisionStatus::Inactive;
        std::array<bool, 2> moving{};
        bool assumesTakeoff = false;
        bool reachLimited = false;
    };

    // 물리 바디와 제어기의 상태는 바꾸지 않는다. 샘플 사이의 연속 충돌을 보장하는 검사는 아니다.
    FootstepCollisionSnapshot PredictFootstepCollisions(const FootstepCollisionInput& input);
}
