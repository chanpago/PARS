#include "HorizontalBalance.h"
#include <algorithm>
#include <cmath>

using namespace PARS;
using namespace physx;

namespace
{
    bool ValidSettings(const HorizontalBalanceSettings& settings)
    {
        return std::isfinite(settings.frequency) && settings.frequency > 0 &&
            std::isfinite(settings.dampingRatio) && settings.dampingRatio >= 0 &&
            std::isfinite(settings.maximumAcceleration) && settings.maximumAcceleration >= 0 &&
            std::isfinite(settings.maximumAccelerationRate) && settings.maximumAccelerationRate > 0 &&
            std::isfinite(settings.targetResponseTime) && settings.targetResponseTime >= 0 &&
            std::isfinite(settings.contactBlendTime) && settings.contactBlendTime >= 0 &&
            std::isfinite(settings.minimumLoadRatio) && settings.minimumLoadRatio > 0 &&
            std::isfinite(settings.steppingPositionScale) && settings.steppingPositionScale >= 0 &&
            settings.steppingPositionScale <= 1;
    }

    bool LimitLength(PxVec3& value, float maximum)
    {
        const float length = value.magnitude();
        if (length > maximum && length > 0)
        {
            value *= maximum / length;
            return true;
        }
        return false;
    }
}

void HorizontalBalance::Reset()
{
    m_Snapshot = {};
    m_ReferenceOffset = PxVec3(0);
    m_TargetCom = PxVec3(0);
    m_Acceleration = PxVec3(0);
    m_ContactWeights = {};
    m_HasReference = false;
    m_HasSample = false;
}

bool HorizontalBalance::Update(const HorizontalBalanceInput& input, const HorizontalBalanceSettings& settings)
{
    if (!input.enabled || !settings.enabled || (m_HasSample &&
        (input.revision != m_Snapshot.revision || input.completedStep < m_Snapshot.completedStep)))
    {
        Reset();
    }
    if (!input.enabled || !settings.enabled)
    {
        return false;
    }
    if (m_HasSample && input.completedStep == m_Snapshot.completedStep)
    {
        return m_Snapshot.valid;
    }
    const bool consecutive = m_HasSample && input.completedStep > m_Snapshot.completedStep &&
        input.completedStep - m_Snapshot.completedStep == 1;
    m_Snapshot = {};
    m_Snapshot.revision = input.revision;
    m_Snapshot.completedStep = input.completedStep;
    m_HasSample = true;
    if (!ValidSettings(settings) || !input.com.isFinite() || !input.velocity.isFinite() || !input.targetVelocity.isFinite() ||
        !std::isfinite(input.mass) || input.mass <= 0 || !std::isfinite(input.deltaTime) || input.deltaTime <= 0 ||
        !std::isfinite(input.supportWeight) || input.supportWeight < 0 || input.supportWeight > 1)
    {
        m_Acceleration = PxVec3(0);
        m_ContactWeights = {};
        return false;
    }
    if (!consecutive)
    {
        m_ContactWeights = {};
        m_Acceleration = PxVec3(0);
    }
    PxVec3 footCenter(0);
    unsigned availableFeet = 0;
    unsigned supportingFeet = 0;
    float measuredSupport = 0;
    for (size_t side = 0; side < input.feet.size(); ++side)
    {
        const HorizontalBalanceFoot& foot = input.feet[side];
        if (foot.available && foot.position.isFinite())
        {
            footCenter += foot.position;
            ++availableFeet;
        }
        PxVec3 normal = foot.normal;
        float ratio = 0;
        if (foot.available && foot.position.isFinite() && foot.grounded && !foot.moving &&
            normal.isFinite() && normal.normalize() > 1e-6f && normal.y >= .70710678f &&
            std::isfinite(foot.normalImpulse) && foot.normalImpulse > 0)
        {
            ratio = foot.normalImpulse * normal.y / (input.mass * 9.81f * input.deltaTime);
        }
        const bool loaded = std::isfinite(ratio) && ratio >= settings.minimumLoadRatio;
        m_Snapshot.loadRatios[side] = std::isfinite(ratio) ? ratio : 0;
        if (loaded)
        {
            ++supportingFeet;
            m_ContactWeights[side] = settings.contactBlendTime > 0 ?
                (std::min)(1.0f, m_ContactWeights[side] + input.deltaTime / settings.contactBlendTime) : 1.0f;
            measuredSupport += PxClamp(ratio, 0.0f, 1.0f) * m_ContactWeights[side];
        }
        else
        {
            // 접촉 유예는 외력에 사용하지 않는다. 실제 하중이 사라지면 즉시 해당 발을 제외한다.
            m_ContactWeights[side] = 0;
        }
    }
    m_Snapshot.valid = true;
    m_Snapshot.contactWeights = m_ContactWeights;
    if (supportingFeet == 0 || availableFeet != input.feet.size() || input.supportWeight == 0)
    {
        // 공중에서는 이전 스텝의 힘이나 변화율 제한의 잔여 힘을 이어 가지 않는다.
        m_Acceleration = PxVec3(0);
        m_Snapshot.targetCom = m_HasReference ? m_TargetCom : input.com;
        return true;
    }
    footCenter /= static_cast<float>(availableFeet);
    if (!m_HasReference)
    {
        // 최초 서기 자세의 자연스러운 COM 오프셋을 보존한다.
        m_ReferenceOffset = input.com - footCenter;
        m_ReferenceOffset.y = 0;
        m_TargetCom = input.com;
        m_TargetCom.y = 0;
        m_HasReference = true;
    }
    if (!input.stepping && supportingFeet == input.feet.size())
    {
        // 실제 양발 지지가 확인된 뒤에만 새 발 배치를 따른다. 공중 발의 궤적은 따라가지 않는다.
        PxVec3 desired = footCenter + m_ReferenceOffset;
        desired.y = 0;
        const float blend = input.deltaTime / (settings.targetResponseTime + input.deltaTime);
        m_TargetCom += (desired - m_TargetCom) * blend;
    }
    m_Snapshot.targetCom = m_TargetCom;
    m_Snapshot.positionError = m_TargetCom - input.com;
    m_Snapshot.positionError.y = 0;
    PxVec3 velocity = input.velocity - input.targetVelocity;
    velocity.y = 0;
    const float positionScale = input.stepping || !input.targetVelocity.isZero() ? settings.steppingPositionScale : 1.0f;
    const float stiffness = settings.frequency * settings.frequency;
    const float damping = 2.0f * settings.dampingRatio * settings.frequency;
    const PxVec3 requested = m_Snapshot.positionError * (stiffness * positionScale) - velocity * damping;
    if (!requested.isFinite())
    {
        m_Acceleration = PxVec3(0);
        m_Snapshot.valid = false;
        return false;
    }
    m_Snapshot.requestedAcceleration = requested;
    m_Snapshot.supportWeight = PxClamp(measuredSupport, 0.0f, 1.0f) * input.supportWeight;
    PxVec3 acceleration = requested;
    m_Snapshot.limited = LimitLength(acceleration, settings.maximumAcceleration);
    acceleration *= m_Snapshot.supportWeight;
    PxVec3 change = acceleration - m_Acceleration;
    m_Snapshot.limited = LimitLength(change, settings.maximumAccelerationRate * input.deltaTime) || m_Snapshot.limited;
    m_Acceleration += change;
    m_Snapshot.limited = LimitLength(m_Acceleration,
        settings.maximumAcceleration * m_Snapshot.supportWeight) || m_Snapshot.limited;
    m_Snapshot.acceleration = m_Acceleration;
    m_Snapshot.force = m_Acceleration * input.mass;
    if (!m_Snapshot.force.isFinite())
    {
        m_Acceleration = PxVec3(0);
        m_Snapshot.force = PxVec3(0);
        m_Snapshot.valid = false;
        return false;
    }
    m_Snapshot.active = !m_Snapshot.force.isZero();
    return true;
}
