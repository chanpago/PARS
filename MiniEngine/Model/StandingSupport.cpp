#include "StandingSupport.h"
#include <algorithm>
#include <cmath>

using namespace PARS;
using namespace physx;

void StandingSupport::Reset()
{
    m_Snapshot = {};
    m_MissingContactTime = {};
    m_HorizontalReference = PxVec3(0);
    m_HorizontalTarget = PxVec3(0);
    m_HorizontalVelocity = PxVec3(0);
    m_Revision = 0;
    m_CompletedStep = 0;
    m_Initialized = false;
}

bool StandingSupport::Update(const StandingSupportInput& input, const StandingSupportSettings& settings)
{
    if (!input.enabled || (m_Initialized && input.revision != m_Revision))
    {
        Reset();
    }
    if (!input.enabled)
    {
        return false;
    }
    if (m_Initialized && input.completedStep == m_CompletedStep)
    {
        return m_Snapshot.valid;
    }
    if (!input.pelvis.isSane() || !input.animatedPelvis.isSane() || !input.velocity.isFinite() ||
        !std::isfinite(input.deltaTime) || input.deltaTime <= 0 ||
        !std::isfinite(input.stepDistance) || input.stepDistance < 0 ||
        !std::isfinite(settings.stepLoweringScale) || settings.stepLoweringScale < 0 ||
        !std::isfinite(settings.contactLowering) || settings.contactLowering < 0 ||
        !std::isfinite(settings.contactLoweringTime) || settings.contactLoweringTime <= 0 ||
        !std::isfinite(settings.maximumLowering) || settings.maximumLowering < 0 ||
        !std::isfinite(settings.loweringSpeed) || settings.loweringSpeed <= 0 ||
        !std::isfinite(settings.raisingSpeed) || settings.raisingSpeed <= 0 ||
        !std::isfinite(settings.reachReserve) || settings.reachReserve < 0 ||
        !std::isfinite(settings.horizontalDampingTime) || settings.horizontalDampingTime < 0 ||
        !std::isfinite(settings.maximumHorizontalCorrection) || settings.maximumHorizontalCorrection < 0 ||
        !std::isfinite(settings.horizontalFrequency) || settings.horizontalFrequency <= 0)
    {
        m_Snapshot.valid = false;
        return false;
    }

    float nominalHeight = 0;
    float supportCount = 0;
    float maximumHeight = PX_MAX_F32;
    float missingAmount = 0;
    bool reachable = true;

    // 현재 발 목표 배치에 맞는 골반 기준 위치를 계산
    // 처음에는 입력 목표 포즈의 골반 위치를 계산한다. 여기에 양발의 다음 차이를 절반씩 더한다
    PxVec3 desiredPelvis = input.animatedPelvis.p;

    for (size_t side = 0; side < input.feet.size(); ++side)
    {
        const StandingSupportFoot& foot = input.feet[side];
        if (!foot.available || !foot.sole.isSane() || !foot.animatedSole.isSane() ||
            !foot.soleInBone.isSane() || !foot.hipOffset.isFinite() ||
            !std::isfinite(foot.length) || foot.length <= settings.reachReserve)
        {
            m_Snapshot.valid = false;
            return false;
        }
        // 발바닥 목표가 애니메이션 기준 위치에서 옮겨진 평균만큼, 골반 기준 위치도 옮김
        // 이렇게 desiredPelvis(애니메이션 목표 골반위치)에 물리발의 오프셋만큼 더해줌
        // 발 목표 배치에 맞춘 골반 위치를 구해서, 뒤에서 수평 위치를 유지할 기준으로 사용
        desiredPelvis += (foot.sole.p - foot.animatedSole.p) * .5f;
        if (!foot.supporting)
        {
            m_MissingContactTime[side] = 0;
            continue;
        }

        // 지지 역할을 맡은 발에 대해 실행
        // 지지발을 기준으로 정상 골반 높이를 계산하는부분
        // 실제 접촉 판정은 바꾸지 않고 지지 예정인 발만 재접촉용 높이 목표를 낮춘다.

        // 지지하는 발은 바닥에 붙어있어야하는데, 불가피한 경우에 지지한느 발이 뜰 경우를 상정한다.시간을 누적해서 더해준다
        m_MissingContactTime[side] = foot.grounded ? 0 : (std::min)(m_MissingContactTime[side] + input.deltaTime, settings.contactLoweringTime);

        // 
        missingAmount = (std::max)(missingAmount, m_MissingContactTime[side] / settings.contactLoweringTime);
        nominalHeight += foot.sole.p.y + input.animatedPelvis.p.y - foot.animatedSole.p.y;
        supportCount += 1;

        const PxVec3 ankle = (foot.sole * foot.soleInBone.getInverse()).p;
        const PxVec3 hip = input.pelvis.p + foot.hipOffset;
        const float dx = ankle.x - hip.x;
        const float dz = ankle.z - hip.z;
        const float length = foot.length - settings.reachReserve;
        const float verticalSquared = length * length - dx * dx - dz * dz;
        reachable = reachable && verticalSquared > 0;
        const float allowedHeight = ankle.y + std::sqrt((std::max)(0.0f, verticalSquared)) - foot.hipOffset.y;
        maximumHeight = (std::min)(maximumHeight, allowedHeight);
    }
    if (supportCount == 0)
    {
        m_Snapshot.valid = false;
        return false;
    }
    nominalHeight /= supportCount;
    const float stepLowering = input.stepping ? input.stepDistance * settings.stepLoweringScale : 0;
    const float contactLowering = missingAmount * settings.contactLowering;
    const float reachLowering = (std::max)(0.0f, nominalHeight - maximumHeight);
    const float requestedLowering = (std::min)(settings.maximumLowering,
        (std::max)(stepLowering, (std::max)(contactLowering, reachLowering)));
    const float desiredHeight = nominalHeight - requestedLowering;
    const float previousHeight = m_Initialized ? m_Snapshot.targetHeight : input.pelvis.p.y;
    const float rate = desiredHeight < previousHeight ? settings.loweringSpeed : settings.raisingSpeed;
    const float change = PxClamp(desiredHeight - previousHeight, -rate * input.deltaTime, rate * input.deltaTime);
    // 공중 발의 이동 궤적을 골반 목표에 다시 넣으면 몸이 그 궤적을 쫓아 추가 발딛기를 유발한다.
    // 양발이 지지 역할일 때 기준을 갱신하고 발딛기 중에는 직전 지지 기준을 유지한다.
    if (!m_Initialized)
    {
        m_HorizontalReference = desiredPelvis;
        m_HorizontalTarget = desiredPelvis;
        m_HorizontalVelocity = PxVec3(0);
    }
    if (!input.stepping)
    {
        m_HorizontalTarget = desiredPelvis;
    }
    // 착지로 기준이 바뀌어도 위치를 즉시 교체하지 않는다. 임계 감쇠 응답으로 속도를 이어 간다.
    const float frequency = settings.horizontalFrequency;
    const float dt = input.deltaTime;
    const PxVec3 horizontalAcceleration = (frequency * frequency *
        (m_HorizontalTarget - m_HorizontalReference - m_HorizontalVelocity * dt) -
        2.0f * frequency * m_HorizontalVelocity) / (1.0f + 2.0f * frequency * dt + frequency * frequency * dt * dt);
    m_HorizontalVelocity += horizontalAcceleration * dt;
    m_HorizontalReference += m_HorizontalVelocity * dt;
    // 양발 지지 기준으로 골반 자세를 유지하고 수평 속도를 감쇠한다.
    // 월드 골반에 힘을 더하지 않고 지지 다리의 상대 자세에만 제한된 오차를 전달한다.
    PxVec3 horizontalCorrection = input.pelvis.p - m_HorizontalReference + input.velocity * settings.horizontalDampingTime;
    horizontalCorrection.y = 0;
    const float horizontalLength = horizontalCorrection.magnitude();
    if (horizontalLength > settings.maximumHorizontalCorrection && horizontalLength > 0)
    {
        horizontalCorrection *= settings.maximumHorizontalCorrection / horizontalLength;
    }

    m_Snapshot = {};
    m_Snapshot.valid = true;
    m_Snapshot.reachLimited = !reachable || reachLowering > settings.maximumLowering;
    m_Snapshot.nominalHeight = nominalHeight;
    m_Snapshot.targetHeight = previousHeight + change;
    m_Snapshot.actualHeight = input.pelvis.p.y;
    m_Snapshot.stepLowering = stepLowering;
    m_Snapshot.contactLowering = contactLowering;
    m_Snapshot.reachLowering = reachLowering;
    m_Snapshot.lowering = nominalHeight - m_Snapshot.targetHeight;
    m_Snapshot.horizontalCorrection = horizontalCorrection;
    for (size_t side = 0; side < input.feet.size(); ++side)
    {
        m_Snapshot.soleTargets[side] = input.feet[side].sole;
        // 접촉을 잃은 다리는 골반 낮추기에 맞춰 발을 위로 빼지 않고 바닥을 계속 찾는다.
        if (input.feet[side].supporting && input.feet[side].grounded)
        {
            // 실제 골반이 낮으면 제어용 발 목표를 아래로 보내 다리가 바닥을 밀도록 한다.
            m_Snapshot.soleTargets[side].p.y += input.pelvis.p.y - m_Snapshot.targetHeight;
            m_Snapshot.soleTargets[side].p += horizontalCorrection;
        }
    }
    m_Initialized = true;
    m_Revision = input.revision;
    m_CompletedStep = input.completedStep;
    return true;
}
