#pragma once

#include <foundation/PxVec3.h>
#include <array>
#include <cstdint>

namespace PARS
{
    struct HorizontalBalanceSettings
    {
    public:
        float frequency = 3.0f;
        float dampingRatio = 1.0f;
        float maximumAcceleration = 2.0f;
        float maximumAccelerationRate = 20.0f;
        float targetResponseTime = .15f;
        float contactBlendTime = .08f;
        float minimumLoadRatio = .02f;
        float steppingPositionScale = .1f;
        bool enabled = true;
    };

    struct HorizontalBalanceFoot
    {
    public:
        physx::PxVec3 position{0};
        physx::PxVec3 normal{0, 1, 0};
        float normalImpulse = 0;
        bool available = false;
        bool grounded = false;
        bool moving = false;
    };

    struct HorizontalBalanceInput
    {
    public:
        uint64_t revision = 0;
        uint64_t completedStep = 0;
        std::array<HorizontalBalanceFoot, 2> feet;
        physx::PxVec3 com{0};
        physx::PxVec3 velocity{0};
        physx::PxVec3 torsoAcceleration{0};
        float mass = 0;
        float deltaTime = 0;
        float supportWeight = 0;
        bool enabled = false;
        bool stepping = false;
    };

    struct HorizontalBalanceSnapshot
    {
    public:
        uint64_t revision = 0;
        uint64_t completedStep = 0;
        std::array<float, 2> loadRatios{};
        std::array<float, 2> contactWeights{};
        physx::PxVec3 targetCom{0};
        physx::PxVec3 positionError{0};
        physx::PxVec3 requestedAcceleration{0};
        physx::PxVec3 acceleration{0};
        physx::PxVec3 force{0};
        float supportWeight = 0;
        bool valid = false;
        bool active = false;
        bool limited = false;
    };

    // 실제 지지 접촉이 있는 동안 COM 오차를 골반에 가할 수평 외력으로 바꾼다.
    class HorizontalBalance
    {
    public:
        void Reset();
        bool Update(const HorizontalBalanceInput& input, const HorizontalBalanceSettings& settings);

    public:
        const HorizontalBalanceSnapshot& GetSnapshot() const
        {
            return m_Snapshot;
        }

    private:
        HorizontalBalanceSnapshot m_Snapshot;
        physx::PxVec3 m_ReferenceOffset{0};
        physx::PxVec3 m_TargetCom{0};
        physx::PxVec3 m_Acceleration{0};
        std::array<float, 2> m_ContactWeights{};
        bool m_HasReference = false;
        bool m_HasSample = false;
    };
}
