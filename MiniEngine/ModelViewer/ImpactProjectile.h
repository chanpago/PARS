#pragma once

#include <PxPhysicsAPI.h>
#include <cstdint>

enum class ImpactProjectileType : uint8_t
{
    Sphere,
    Arrow
};

struct ImpactProjectileSettings
{
public:
    static ImpactProjectileSettings ArrowDefaults()
    {
        ImpactProjectileSettings settings;
        settings.mass = .5f;
        settings.speed = 30.0f;
        settings.radius = .01f;
        return settings;
    }

public:
    float mass = 2.0f;
    float speed = 7.5f;
    float radius = .15f;
    float length = .6f;
    float partImpulse = 0;
    float bodyImpulse = 0;
    float torqueMultiplier = 0;
};

// 메쉬 중심의 월드 자세다. 화살 원기둥의 앞쪽은 로컬 +X축이다.
struct ImpactProjectileVisual
{
public:
    physx::PxTransform pose{physx::PxIdentity};
    float radius = 0;
    float length = 0;
    ImpactProjectileType type = ImpactProjectileType::Sphere;
};
