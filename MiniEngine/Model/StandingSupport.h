#pragma once

#include <foundation/PxTransform.h>
#include <array>
#include <cstdint>

namespace PARS
{
    struct StandingSupportSettings
    {
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
    };

    struct StandingSupportFoot
    {
        bool available = false;
        bool grounded = false;
        bool supporting = true;
        physx::PxTransform sole{physx::PxIdentity};
        physx::PxTransform animatedSole{physx::PxIdentity};
        physx::PxTransform soleInBone{physx::PxIdentity};
        physx::PxVec3 hipOffset{0};
        float length = 0;
    };

    struct StandingSupportInput
    {
        bool enabled = false;
        bool stepping = false;
        float stepDistance = 0;
        float deltaTime = 0;
        uint64_t revision = 0;
        uint64_t completedStep = 0;
        physx::PxTransform pelvis{physx::PxIdentity};
        physx::PxTransform animatedPelvis{physx::PxIdentity};
        physx::PxVec3 velocity{0};
        std::array<StandingSupportFoot, 2> feet;
    };

    struct StandingSupportSnapshot
    {
        bool valid = false;
        bool reachLimited = false;
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
    };

    // 골반 높이 오차를 지지 다리의 상대 자세로 바꾼다. 바디 위치와 외력은 직접 바꾸지 않는다.
    class StandingSupport
    {
    public:
        void Reset();
        bool Update(const StandingSupportInput& input, const StandingSupportSettings& settings);
        const StandingSupportSnapshot& GetSnapshot() const
        {
            return m_Snapshot;
        }

    private:
        StandingSupportSnapshot m_Snapshot;
        std::array<float, 2> m_MissingContactTime{};
        physx::PxVec3 m_HorizontalReference{0};
        physx::PxVec3 m_HorizontalTarget{0};
        physx::PxVec3 m_HorizontalVelocity{0};
        uint64_t m_Revision = 0;
        uint64_t m_CompletedStep = 0;
        bool m_Initialized = false;
    };
}
