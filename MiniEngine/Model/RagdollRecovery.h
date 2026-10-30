#pragma once

#include <cstdint>
#include <vector>

struct GraphNode;

namespace PARS
{
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
        float impactVelocity = .05f;
        float maximumReturnSpeed = .15f;
        float stableDuration = .2f;
        float blendDuration = .5f;
    };

    struct RecoveryInput
    {
        bool enabled = false, fallen = false;
        bool needsStep = false, stepping = false, bothGrounded = false;
        bool hasReference = false, insideReference = false, predictionInsideReference = false;
        bool insideSupport = false, predictionInsideSupport = false;
        float impactVelocity = 0, horizontalSpeed = 0, deltaTime = 0;
        uint64_t revision = 0, completedStep = 0;
    };

    struct RecoverySnapshot
    {
        RecoveryState state = RecoveryState::Animation;
        RecoveryReason reason = RecoveryReason::Disabled;
        bool blending = false, enteredRecovery = false, startedBlend = false;
        bool hasReference = false, insideReference = false, predictionInsideReference = false;
        bool bothGrounded = false, holdingTargets = false;
        float stableTime = 0, blendAlpha = 1, impactVelocity = 0, horizontalSpeed = 0;
        uint64_t revision = 0, completedStep = 0, recoveryCount = 0, returnCount = 0;
    };

    class RagdollRecovery
    {
    public:
        void Reset();
        void Update(const RecoveryInput& input, const RecoverySettings& settings);
        const RecoverySnapshot& GetSnapshot() const
        {
            return m_Snapshot;
        }

    private:
        RecoverySnapshot m_Snapshot;
        bool m_HasSample = false;
        float m_BlendTime = 0;
    };

    const char* RecoveryReasonLabel(RecoveryReason reason);
    // 물리 결과의 행렬을 직접 읽는다. 애니메이션용 회전 캐시는 실제 물리 자세와 다를 수 있다.
    bool BlendRecoveryPose(const std::vector<GraphNode>& source, std::vector<GraphNode>& target, float alpha);
}
