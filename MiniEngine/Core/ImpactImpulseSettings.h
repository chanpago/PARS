#pragma once

#include <cmath>
#include <foundation/PxVec3.h>

struct ImpactImpulseSettings
{
public:
    bool IsValid() const
    {
        return std::isfinite(partImpulse) && partImpulse >= 0 && std::isfinite(bodyImpulse) && bodyImpulse >= 0 &&
            std::isfinite(torqueMultiplier) && torqueMultiplier >= 0;
    }

public:
    float partImpulse = 0;
    float bodyImpulse = 0;
    float torqueMultiplier = 0;
};

struct PhysicsImpactResponse
{
public:
    physx::PxVec3 GetLinearImpulse() const
    {
        if (groundedArrowHit && impulse.y > 0.0f)
        {
            return physx::PxVec3(impulse.x, 0.0f, impulse.z);
        }
        return impulse;
    }

    physx::PxVec3 GetAngularImpulse(const physx::PxVec3& center, bool contactImpulseApplied = false) const
    {
        if (fromCharacterCapsule)
        {
            // 예열은 실측 각충격량을 그대로 인계하고, 동적 바디는 솔버가 이미 전달한 1배를 제외한다.
            // 배율 0은 기존처럼 별도 회전 보정을 끈다.
            const float scale = contactImpulseApplied && torqueMultiplier > 0 ? torqueMultiplier - 1.0f : torqueMultiplier;
            return ((position - center).cross(contactImpulse) + contactMomentAtPosition) * scale;
        }
        return (position - center).cross(impulse) * torqueMultiplier;
    }

public:
    physx::PxVec3 position{0};
    physx::PxVec3 impulse{0};
    physx::PxVec3 partImpulse{0};
    physx::PxVec3 contactImpulse{0};
    physx::PxVec3 contactMomentAtPosition{0};
    float torqueMultiplier = 0;
    bool groundedArrowHit = false;
    bool fromCharacterCapsule = false;
};
