#include "TorsoBalance.h"
#include <cmath>

using namespace PARS;
using namespace physx;

namespace
{
    constexpr float MaxBend = 1.0f;
    constexpr float HipGravityFraction = .20f;

    PxVec3 LimitLength(PxVec3 value, float maximum)
    {
        const float magnitude = value.magnitude();
        return magnitude > maximum ? value * (maximum / magnitude) : value;
    }

    PxVec3 HipAcceleration(const TorsoBalance& torso, const PxVec3& horizontalGravity)
    {
        // 작은 흔들림은 허리가 일부 받아내고, 큰 충격은 실제 골반 이동이 필요하다.
        const PxVec3 desired = (torso.lean - torso.neutral) * 20.0f + torso.rate * 7.0f;
        return LimitLength(horizontalGravity / torso.length + desired * .8f,
            torso.GetHipTorqueLimit() / torso.inertia);
    }
}

PxVec3 TorsoBalance::GetLean(const PxVec3& arm)
{
    PxVec3 direction(arm.x, 0, arm.z);
    const float horizontal = direction.normalize();
    return direction * std::atan2(horizontal, arm.y);
}

PxVec3 TorsoBalance::GetDirection() const
{
    PxVec3 direction = lean;
    const float angle = direction.normalize();
    return direction * std::sin(angle) + PxVec3(0, std::cos(angle), 0);
}

void TorsoBalance::Begin(const TorsoBalanceSample& body, const PxVec3& restLean)
{
    *this = {};
    if (body.height <= .02f || body.mass <= 0 || body.inertia <= 0 || body.gravity <= 0)
    {
        return;
    }
    length = body.inertia / (body.mass * body.height);
    inertia = body.inertia;
    gravity = body.gravity;
    neutral = restLean;
    Observe(body);
}

void TorsoBalance::Observe(const TorsoBalanceSample& body)
{
    lean = body.lean;
    rate = body.rate;
}

void TorsoBalance::AddAngularImpulse(const PxVec3& angularImpulse)
{
    if (IsActive())
    {
        rate += angularImpulse.cross(PxVec3(0, 1, 0)) / inertia;
    }
}

float TorsoBalance::GetHipTorqueLimit() const
{
    return IsActive() ? HipGravityFraction * gravity * inertia / length : 0;
}

PxVec3 TorsoBalance::GetHipTorque() const
{
    if (!IsActive())
    {
        return PxVec3(0);
    }
    const PxVec3 direction = GetDirection();
    const PxVec3 restoring = HipAcceleration(*this, PxVec3(direction.x, 0, direction.z) * gravity);
    return restoring.cross(PxVec3(0, 1, 0)) * inertia;
}

PxVec3 TorsoBalance::GetRequestedAcceleration() const
{
    if (!IsActive())
    {
        return PxVec3(0);
    }
    // 같은 각도라도 이미 돌아오는 몸통은 덜 쫓는다. 허리가 홀로 버틸 수 있는 토크를 먼저 뺀다.
    const PxVec3 error = lean - neutral;
    const PxVec3 restoring = error * 20.0f + rate * 7.0f;
    const PxVec3 gravityAcceleration = GetDirection() * gravity;
    const PxVec3 horizontalGravity(gravityAcceleration.x, 0, gravityAcceleration.z);
    const PxVec3 hip = HipAcceleration(*this, horizontalGravity);
    return LimitLength((horizontalGravity + (restoring - hip) * length) / PxMax(.45f, std::cos(lean.magnitude())), gravity);
}

PxVec3 TorsoBalance::GetLandingOffset(float time) const
{
    return GetRequestedAcceleration() * (.5f * time * time);
}

void TorsoBalance::Advance(float deltaTime, const PxVec3& pivotVelocityChange)
{
    if (!IsActive() || deltaTime <= 0)
    {
        return;
    }
    // 실제 받침점 속도 변화만 진자에 전달한다. 허리 토크의 상한 밖에서는 중력이 기울기를 키운다.
    const PxVec3 direction = GetDirection();
    const PxVec3 horizontalGravity(direction.x * gravity, 0, direction.z * gravity);
    const PxVec3 hip = HipAcceleration(*this, horizontalGravity);
    const PxVec3 deltaVelocity(pivotVelocityChange.x, 0, pivotVelocityChange.z);
    rate += (horizontalGravity / length - hip) * deltaTime - deltaVelocity * (direction.y / length);
    lean += rate * deltaTime;
    if (lean.magnitudeSquared() > MaxBend * MaxBend)
    {
        const PxVec3 outward = lean.getNormalized();
        lean = outward * MaxBend;
        rate -= outward * PxMax(0.0f, rate.dot(outward));
    }
}

bool TorsoBalance::CanFinish(float deltaTime, bool supported)
{
    const bool quiet = (lean - neutral).magnitudeSquared() < .03f * .03f && rate.magnitudeSquared() < .12f * .12f;
    stableTime = supported && quiet ? stableTime + deltaTime : 0;
    return stableTime >= .12f;
}
