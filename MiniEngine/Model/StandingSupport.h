#pragma once

#include <foundation/PxTransform.h>
#include <array>
#include <cstdint>

namespace PARS
{
    struct StandingSupportSettings
    {
    public:
        float stepLoweringScale = .25f;
        float contactLowering = .08f;
        float contactLoweringTime = .15f;
        float maximumLowering = .18f;
        float loweringSpeed = .5f;
        float raisingSpeed = .25f;
        float reachReserve = .01f;
        float horizontalDampingTime = .1f;
        float maximumHorizontalCorrection = .06f;
        float horizontalFrequency = 4.0f;
        // 기존 관성 보상에 더하는 최대 배율 증가량이다. 0이면 하중별 증폭을 끈다.
        float ankleDriveBoost = .5f;
        float ankleDriveBlendTime = .08f;
        float footTiltLimitStartDegrees = 2.0f;
        float footTiltLimitDegrees = 5.0f;
        bool limitFootTilt = true;
    };

    struct StandingSupportFoot
    {
    public:
        physx::PxTransform sole{physx::PxIdentity};
        physx::PxTransform soleInBone{physx::PxIdentity};
        physx::PxVec3 hipOffset{0};
        physx::PxTransform actualSole{physx::PxIdentity};
        physx::PxVec3 supportNormal{0, 1, 0};
        float length = 0;
        bool available = false;
        bool grounded = false;
        bool supporting = true;
        bool contactGraceActive = false;
        bool allowHeelLift = false;
    };

    struct StandingSupportInput
    {
    public:
        uint64_t revision = 0;
        uint64_t completedStep = 0;
        float stepDistance = 0;
        float deltaTime = 0;
        physx::PxTransform pelvis{physx::PxIdentity};
        physx::PxTransform plannedPelvis{physx::PxIdentity};
        physx::PxVec3 velocity{0};
        std::array<StandingSupportFoot, 2> feet;
        bool enabled = false;
        bool stepping = false;
        bool hasPelvisPlan = false;
    };

    struct StandingSupportSnapshot
    {
    public:
        float nominalHeight = 0;
        float targetHeight = 0;
        float actualHeight = 0;
        float stepLowering = 0;
        float contactLowering = 0;
        float reachLowering = 0;
        float lowering = 0;
        physx::PxVec3 horizontalCorrection{0};
        std::array<physx::PxTransform, 2> soleTargets{
            physx::PxTransform(physx::PxIdentity), physx::PxTransform(physx::PxIdentity)};
        bool valid = false;
        bool reachLimited = false;
        std::array<bool, 2> footTiltLimited{};
    };

    // 골반 높이 오차를 지지 다리의 상대 자세로 바꾼다. 바디 위치와 외력은 직접 바꾸지 않는다.
    class StandingSupport
    {
    public:
        void Reset();
        bool Update(const StandingSupportInput& input, const StandingSupportSettings& settings);

    public:
        const StandingSupportSnapshot& GetSnapshot() const
        {
            return m_Snapshot;
        }

    private:
        uint64_t m_Revision = 0;
        uint64_t m_CompletedStep = 0;
        StandingSupportSnapshot m_Snapshot;
        std::array<float, 2> m_MissingContactTime{};
        physx::PxVec3 m_HorizontalReference{0};
        physx::PxVec3 m_HorizontalTarget{0};
        physx::PxVec3 m_HorizontalVelocity{0};
        bool m_Initialized = false;
    };
}
