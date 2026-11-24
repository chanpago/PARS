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
    physx::PxVec3 GetAngularImpulse(const physx::PxVec3& center) const
    {
        return (position - center).cross(impulse) * torqueMultiplier;
    }

public:
    physx::PxVec3 position{0};
    physx::PxVec3 impulse{0};
    physx::PxVec3 partImpulse{0};
    float torqueMultiplier = 0;
};
