#include "RagdollRecovery.h"
#include <algorithm>
#include <cmath>

using namespace PARS;

void RagdollRecovery::Reset()
{
    m_Snapshot = {};
    m_HasSample = false;
    m_BlendTime = 0;
}

void RagdollRecovery::Update(const RecoveryInput& input, const RecoverySettings& settings)
{
    if (!input.enabled || (m_HasSample && input.revision != m_Snapshot.revision))
    {
        Reset();
    }
    if (!input.enabled || (m_HasSample && input.completedStep == m_Snapshot.completedStep))
    {
        return;
    }
    m_HasSample = true;
    auto& state = m_Snapshot;
    state.revision = input.revision;
    state.completedStep = input.completedStep;
    state.enteredRecovery = state.startedBlend = false;
    state.hasReference = input.hasReference;
    state.insideReference = input.insideReference;
    state.predictionInsideReference = input.predictionInsideReference;
    state.bothGrounded = input.bothGrounded;
    state.impactVelocity = input.impactVelocity;
    state.horizontalSpeed = input.horizontalSpeed;
    const bool valid = std::isfinite(input.deltaTime) && input.deltaTime > 0 &&
        std::isfinite(input.impactVelocity) && input.impactVelocity >= 0 &&
        std::isfinite(input.horizontalSpeed) && input.horizontalSpeed >= 0 &&
        std::isfinite(settings.impactVelocity) && settings.impactVelocity > 0 &&
        std::isfinite(settings.maximumReturnSpeed) && settings.maximumReturnSpeed >= 0 &&
        std::isfinite(settings.stableDuration) && settings.stableDuration > 0 &&
        std::isfinite(settings.blendDuration) && settings.blendDuration > 0;
    const bool impact = valid && input.impactVelocity >= settings.impactVelocity;
    const bool recoveryNeeded = input.fallen || impact || input.needsStep || input.stepping;
    if (state.state == RecoveryState::Animation && recoveryNeeded)
    {
        state.state = RecoveryState::Recovery;
        state.enteredRecovery = true;
        state.blending = false;
        state.blendAlpha = 0;
        state.stableTime = 0;
        m_BlendTime = 0;
        ++state.recoveryCount;
    }
    state.holdingTargets = state.state == RecoveryState::Recovery && !input.fallen;
    if (!valid || input.fallen)
    {
        state.reason = input.fallen ? RecoveryReason::Fallen : RecoveryReason::InvalidSample;
        state.stableTime = 0;
        return;
    }
    if (state.state == RecoveryState::Animation)
    {
        if (state.blending)
        {
            m_BlendTime += input.deltaTime;
            const float t = (std::min)(m_BlendTime / settings.blendDuration, 1.0f);
            // 양 끝의 목표 속도가 갑자기 바뀌지 않도록 부드러운 보간 비율을 사용한다.
            state.blendAlpha = t * t * (3 - 2 * t);
            state.blending = t < 1;
        }
        state.reason = state.blending ? RecoveryReason::Blending : RecoveryReason::FollowingAnimation;
        return;
    }

    RecoveryReason reason = RecoveryReason::Settling;
    if (impact)
    {
        reason = RecoveryReason::Impact;
    }
    else if (input.stepping)
    {
        reason = RecoveryReason::Stepping;
    }
    else if (!input.bothGrounded)
    {
        reason = RecoveryReason::WaitingContact;
    }
    else if (!input.insideSupport || !input.predictionInsideSupport || input.needsStep)
    {
        reason = RecoveryReason::BalanceLost;
    }
    else if (!input.hasReference || !input.insideReference || !input.predictionInsideReference)
    {
        reason = RecoveryReason::OutsideReference;
    }
    else if (input.horizontalSpeed > settings.maximumReturnSpeed)
    {
        reason = RecoveryReason::Moving;
    }
    state.reason = reason;
    if (reason != RecoveryReason::Settling)
    {
        state.stableTime = 0;
        return;
    }
    state.stableTime += input.deltaTime;
    if (state.stableTime + 1e-6f >= settings.stableDuration)
    {
        state.state = RecoveryState::Animation;
        state.reason = RecoveryReason::Blending;
        state.blending = state.startedBlend = true;
        state.holdingTargets = false;
        state.blendAlpha = 0;
        m_BlendTime = 0;
        ++state.returnCount;
    }
}

const char* PARS::RecoveryReasonLabel(RecoveryReason reason)
{
    switch (reason)
    {
    case RecoveryReason::FollowingAnimation:
        return u8"애니메이션 목표 추종";
    case RecoveryReason::Impact:
        return u8"외부 충격 감지";
    case RecoveryReason::BalanceLost:
        return u8"현재 또는 예측 COM이 실제 지지 영역 밖";
    case RecoveryReason::Stepping:
        return u8"발딛기 진행 중";
    case RecoveryReason::WaitingContact:
        return u8"양발의 실제 접촉 대기";
    case RecoveryReason::OutsideReference:
        return u8"현재 또는 예측 COM이 기준 애니메이션 발 영역 밖";
    case RecoveryReason::Moving:
        return u8"COM 속도가 아직 큼";
    case RecoveryReason::Settling:
        return u8"복귀 조건의 연속 유지 확인";
    case RecoveryReason::Blending:
        return u8"현재 물리 자세에서 애니메이션으로 복귀 중";
    case RecoveryReason::Fallen:
        return u8"골반 지지 해제 / 발 보정 중단";
    case RecoveryReason::InvalidSample:
        return u8"회복 입력 또는 설정 오류";
    default:
        return u8"혼합 중이 아닌 Idle + COM 자동 지지에서 사용";
    }
}
