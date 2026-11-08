#pragma once

#include <cstdint>
#include <vector>

struct GraphNode;

namespace PARS
{
    // A영역 연속 유지 시간을 누적하고, 양발이 실제 접지한 스텝에서 복귀를 허용한다.
    class KinematicRecoveryTimer
    {
    public:
        void Reset();
        bool Update(uint64_t revision, uint64_t step, float deltaTime, bool insideA,
            bool bothGrounded, bool impact, float duration);

    public:
        float GetStableTime() const
        {
            return static_cast<float>(m_StableTime);
        }

    private:
        uint64_t m_Revision = 0;
        uint64_t m_Step = 0;
        double m_StableTime = 0;
        bool m_HasSample = false;
    };

    // 물리 모드와 별개인 목표 제어 상태다. 두 상태 모두 액티브 래그돌을 유지한다.
    enum class RecoveryState
    {
        Animation,
        Recovery
    };

    enum class RecoveryReason
    {
        Disabled,
        FollowingAnimation,
        Impact,
        BalanceLost,
        Stepping,
        WaitingContact,
        OutsideReference,
        Moving,
        Settling,
        Blending,
        Fallen,
        InvalidSample
    };

    struct RecoverySettings
    {
    public:
        float impactVelocity = .05f;
        float maximumReturnSpeed = .15f;
        float stableDuration = .2f;
        float blendDuration = .5f;
    };

    struct RecoveryInput
    {
    public:
        uint64_t revision = 0;
        uint64_t completedStep = 0;
        float impactVelocity = 0;
        float horizontalSpeed = 0;
        float deltaTime = 0;
        bool enabled = false;
        bool impactTriggered = false;
        bool fallen = false;
        bool needsStep = false;
        bool stepping = false;
        bool bothGrounded = false;
        bool hasReference = false;
        bool insideReference = false;
        bool predictionInsideReference = false;
        bool insideSupport = false;
        bool predictionInsideSupport = false;
    };

    struct RecoverySnapshot
    {
    public:
        uint64_t revision = 0;
        uint64_t completedStep = 0;
        uint64_t recoveryCount = 0;
        uint64_t returnCount = 0;
        RecoveryState state = RecoveryState::Animation;
        RecoveryReason reason = RecoveryReason::Disabled;
        float stableTime = 0;
        float blendAlpha = 1;
        float impactVelocity = 0;
        float horizontalSpeed = 0;
        bool blending = false;
        bool enteredRecovery = false;
        bool startedBlend = false;
        bool hasReference = false;
        bool insideReference = false;
        bool predictionInsideReference = false;
        bool bothGrounded = false;
        bool holdingTargets = false;
    };

    class RagdollRecovery
    {
    public:
        void Reset();
        void Update(const RecoveryInput& input, const RecoverySettings& settings);

    public:
        const RecoverySnapshot& GetSnapshot() const
        {
            return m_Snapshot;
        }

    private:
        RecoverySnapshot m_Snapshot;
        float m_BlendTime = 0;
        bool m_HasSample = false;
    };

    const char* RecoveryReasonLabel(RecoveryReason reason);
    // 물리 결과의 행렬을 직접 읽는다. 애니메이션용 회전 캐시는 실제 물리 자세와 다를 수 있다.
    bool BlendRecoveryPose(const std::vector<GraphNode>& source, std::vector<GraphNode>& target, float alpha);
}
