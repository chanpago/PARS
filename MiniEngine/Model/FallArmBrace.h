#pragma once

#include "FootstepIK.h"
#include <functional>

namespace PARS
{
    enum class FallArmPhase : uint8_t
    {
        Inactive,
        Reaching,
        Bracing,
        Releasing
    };

    struct FallArmSettings
    {
    public:
        float predictionTime = .18f;
        float blendInDuration = .14f;
        float blendOutDuration = .22f;
        float maximumDuration = 1.6f;
        float contactGraceDuration = .06f;
        float reachRatio = .94f;
        float forwardReachRatio = .65f;
        float backwardReachRatio = .30f;
        float outwardReachRatio = .18f;
        float targetSpeed = 3.0f;
        float braceDriveScale = .65f;
        float surfaceOffset = .005f;
        bool enabled = true;
    };

    // 왼팔은 0, 오른팔은 1이다. 길이와 손바닥 프레임은 모델의 기준 자세에서 구한다.
    struct FallArmSetup
    {
    public:
        physx::PxTransform palmInHand{physx::PxIdentity};
        physx::PxVec3 forwardInAnchor{0, 0, 1};
        physx::PxVec3 rightInAnchor{1, 0, 0};
        physx::PxVec3 upInAnchor{0, 1, 0};
        float upperLength = 0;
        float lowerLength = 0;
        int anchorNode = -1;
        int shoulderNode = -1;
        int elbowNode = -1;
        int handNode = -1;
        int anchorBody = -1;
        int handBody = -1;
        bool available = false;
    };

    struct FallArmSample
    {
    public:
        FallArmSetup setup;
        physx::PxTransform actualAnchor{physx::PxIdentity};
        physx::PxTransform actualHand{physx::PxIdentity};
        physx::PxVec3 shoulderVelocity{0};
        physx::PxVec3 contactPosition{0};
        physx::PxVec3 contactNormal{0, 1, 0};
        bool touching = false;
    };

    struct FallArmInput
    {
    public:
        uint64_t revision = 0;
        uint64_t completedStep = 0;
        std::array<FallArmSample, 2> arms;
        float deltaTime = 0;
        bool falling = false;
        bool bodySettled = false;
    };

    struct FallArmTarget
    {
    public:
        physx::PxTransform palm{physx::PxIdentity};
        physx::PxVec3 shoulder{0};
        physx::PxVec3 actualPalm{0};
        physx::PxVec3 solvedPalm{0};
        physx::PxVec3 elbow{0};
        float weight = 0;
        float driveScale = 1;
        float contactLostTime = 0;
        float reachError = 0;
        FallArmPhase phase = FallArmPhase::Inactive;
        bool hasTarget = false;
        bool touching = false;
        bool applied = false;
        bool reachLimited = false;
    };

    struct FallArmSnapshot
    {
    public:
        uint64_t revision = 0;
        uint64_t completedStep = 0;
        std::vector<int> correctedNodes;
        std::array<FallArmTarget, 2> arms;
        physx::PxVec3 direction{0};
        float elapsedTime = 0;
        float settledTime = 0;
        bool active = false;
        bool finished = false;
    };

    // 후보 위에서 아래로 탐색한다. 표면 탐색과 실제 손의 접촉은 서로 다른 입력이다.
    using FallArmGroundQuery = std::function<bool(const physx::PxVec3& origin, float distance,
        physx::PxVec3& position, physx::PxVec3& normal)>;

    class FallArmBrace
    {
    public:
        void Reset();
        bool Apply(const FallArmInput& input, const FallArmSettings& settings,
            const std::vector<int32_t>& parents, const FallArmGroundQuery& ground,
            std::vector<physx::PxTransform>& pose);

    public:
        const FallArmSnapshot& GetSnapshot() const
        {
            return m_Snapshot;
        }

    private:
        FallArmSnapshot m_Snapshot;
        bool m_HasSample = false;
        bool m_WasFalling = false;
    };

    const char* FallArmPhaseLabel(FallArmPhase phase);
}
