#include "StandingSupport.h"
#include <algorithm>
#include <cmath>

using namespace PARS;
using namespace physx;

namespace
{
    PxVec3 SoleTiltVector(const PxQuat& orientation, const PxVec3& normal)
    {
        const PxVec3 up = orientation.rotate(PxVec3(0, 1, 0));
        PxVec3 axis = normal.cross(up);
        const float sine = axis.normalize();
        const float cosine = PxClamp(normal.dot(up), -1.0f, 1.0f);
        if (sine < 1e-6f)
        {
            if (cosine >= 0.0f)
            {
                return PxVec3(0);
            }
            axis = normal.cross(PxVec3(1, 0, 0));
            if (axis.normalize() < 1e-6f)
            {
                axis = normal.cross(PxVec3(0, 0, 1)).getNormalized();
            }
        }
        return axis * std::atan2(sine, cosine);
    }

    PxQuat TiltRotation(const PxVec3& rotation)
    {
        const float angle = rotation.magnitude();
        return angle > 1e-6f ? PxQuat(angle, rotation / angle) : PxQuat(PxIdentity);
    }

    bool LimitSupportTilt(const StandingSupportFoot& foot, const StandingSupportSettings& settings,
        PxQuat& target)
    {
        PxVec3 normal = foot.supportNormal;
        if (!settings.limitFootTilt || !foot.actualSole.isSane() || !normal.isFinite() ||
            normal.normalize() < 1e-6f || normal.y < .70710678f)
        {
            return false;
        }
        PxVec3 tiltAxis = SoleTiltVector(foot.actualSole.q, normal);
        if (foot.allowHeelLift)
        {
            // 앞쪽 지지 중에는 발 폭 방향의 굽힘을 허용하고, 옆으로 무너지는 기울기는 계속 제한한다.
            PxVec3 hinge = foot.actualSole.q.rotate(PxVec3(1, 0, 0));
            hinge -= normal * normal.dot(hinge);
            if (hinge.normalize() > 1e-6f)
            {
                tiltAxis -= hinge * hinge.dot(tiltAxis);
            }
        }
        const float actualAngle = tiltAxis.normalize();
        const float start = settings.footTiltLimitStartDegrees * PxPi / 180.0f;
        const float maximum = settings.footTiltLimitDegrees * PxPi / 180.0f;
        if (actualAngle <= start)
        {
            return false;
        }
        const PxVec3 targetTilt = SoleTiltVector(target, normal);
        const float alongTilt = targetTilt.dot(tiltAxis);
        const float blend = PxClamp((actualAngle - start) / (maximum - start), 0.0f, 1.0f);
        const float excess = (std::max)(0.0f, alongTilt - maximum) * blend;
        if (excess <= 1e-6f)
        {
            return false;
        }
        // 반대 방향의 복원과 선회는 남기고, 이미 들린 방향으로 과도한 목표만 줄인다.
        const PxVec3 limitedTilt = targetTilt - tiltAxis * excess;
        target = (TiltRotation(limitedTilt) * TiltRotation(targetTilt).getConjugate() * target).getNormalized();
        return true;
    }
}

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
    if (!input.pelvis.isSane() || (input.hasPelvisPlan && !input.plannedPelvis.isSane()) || !input.velocity.isFinite() ||
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
        !std::isfinite(settings.horizontalFrequency) || settings.horizontalFrequency <= 0 ||
        !std::isfinite(settings.footTiltLimitStartDegrees) || settings.footTiltLimitStartDegrees < 0 ||
        !std::isfinite(settings.footTiltLimitDegrees) ||
        settings.footTiltLimitDegrees <= settings.footTiltLimitStartDegrees || settings.footTiltLimitDegrees > 90.0f)
    {
        m_Snapshot.valid = false;
        return false;
    }

    float nominalHeight = 0;
    float supportCount = 0;
    float maximumHeight = PX_MAX_F32;
    float missingAmount = 0;
    bool reachable = true;

    // 실제 발 배치와 고관절 오프셋으로 골반의 수평 지지 기준을 계산한다.
    PxVec3 desiredPelvis(0);

    for (size_t side = 0; side < input.feet.size(); ++side)
    {
        const StandingSupportFoot& foot = input.feet[side];
        if (!foot.available || !foot.sole.isSane() ||
            !foot.soleInBone.isSane() || !foot.hipOffset.isFinite() ||
            !std::isfinite(foot.length) || foot.length <= settings.reachReserve)
        {
            m_Snapshot.valid = false;
            return false;
        }
        desiredPelvis += (foot.sole.p - foot.hipOffset) * .5f;
        if (!foot.supporting)
        {
            m_MissingContactTime[side] = 0;
            continue;
        }

        // 지지 역할을 맡은 발에 대해 실행
        // 지지발을 기준으로 정상 골반 높이를 계산하는부분
        // 실제 접촉 판정은 바꾸지 않고 지지 예정인 발만 재접촉용 높이 목표를 낮춘다.

        // 지지 접촉 유예가 끝난 뒤부터 재접촉을 위한 낮추기를 누적한다.
        const bool hasSupportContact = foot.grounded || foot.contactGraceActive;
        m_MissingContactTime[side] = hasSupportContact ? 0 :
            (std::min)(m_MissingContactTime[side] + input.deltaTime, settings.contactLoweringTime);

        // 접촉 상실 때문에 골반을 얼마나 낮출지 정하는 비율 0~1로 설정된다.
        missingAmount = (std::max)(missingAmount, m_MissingContactTime[side] / settings.contactLoweringTime);

        // 발목 목표에 다리가 닿을 수 있도록 골반이 올라갈 수 있는 최대 높이를 계산하는부분
        // 발목 목표와 고관절 위치를 구함
        const PxVec3 ankle = (foot.sole * foot.soleInBone.getInverse()).p;
        const PxVec3 hip = input.pelvis.p + foot.hipOffset;


        const float dx = ankle.x - hip.x;
        const float dz = ankle.z - hip.z;
        const float length = foot.length - settings.reachReserve;
        // 기준 높이는 실제 다리 길이와 발목·골반 연결점으로 정한다.
        nominalHeight += ankle.y + length - foot.hipOffset.y;
        supportCount += 1;

        // 수직거리의 제곱을 구함
        const float verticalSquared = length * length - dx * dx - dz * dz;

        // 수직거리 여유가있는지 판단
        reachable = reachable && verticalSquared > 0;

        // 가능한 수직거리를 골반의 최대 높이로 바꿈
        /*
        ankle.y + std::sqrt((std::max)(0.0f, verticalSquared)) 
        이렇게만 적으면 발목부터 고관절(다리가시작하는부분) 까지의거리. 여기에 hip offset을 빼줘서 골반까지의 높이를 구한다
        일반적으로 골반보다 다리가 시작하는 부분이 더 밑에 있기 때문에 foot.hipOffset.y는 음수
        */
        const float allowedHeight = ankle.y + std::sqrt((std::max)(0.0f, verticalSquared)) - foot.hipOffset.y;

        // 지지하는 모든 다리의 조건을 만족하는 높이를 선택
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
    m_Snapshot.targetHeight = input.hasPelvisPlan ? input.plannedPelvis.p.y : previousHeight + change;
    m_Snapshot.actualHeight = input.pelvis.p.y;
    m_Snapshot.stepLowering = stepLowering;
    m_Snapshot.contactLowering = contactLowering;
    m_Snapshot.reachLowering = reachLowering;
    m_Snapshot.lowering = nominalHeight - m_Snapshot.targetHeight;
    m_Snapshot.horizontalCorrection = horizontalCorrection;
    for (size_t side = 0; side < input.feet.size(); ++side)
    {
        m_Snapshot.soleTargets[side] = input.feet[side].sole;
        // 짧은 접촉 상실에는 보정을 이어가고, 유예가 끝난 다리는 바닥을 다시 찾는다.
        if (input.feet[side].supporting && (input.feet[side].grounded || input.feet[side].contactGraceActive))
        {
            if (input.hasPelvisPlan)
            {
                // 양쪽 다리 IK가 계획 골반을 공통 부모로 사용하므로 발 월드 목표는 유지한다.
                // 별도의 골반 오차를 다시 더하면 계획 보정이 중복된다.
                continue;
            }
            // 실제 골반이 낮으면 제어용 발 목표를 아래로 보내 다리가 바닥을 밀도록 한다.
            m_Snapshot.soleTargets[side].p.y += input.pelvis.p.y - m_Snapshot.targetHeight;
            m_Snapshot.soleTargets[side].p += horizontalCorrection;
            m_Snapshot.footTiltLimited[side] = LimitSupportTilt(input.feet[side], settings,
                m_Snapshot.soleTargets[side].q);
        }
    }
    m_Initialized = true;
    m_Revision = input.revision;
    m_CompletedStep = input.completedStep;
    return true;
}
