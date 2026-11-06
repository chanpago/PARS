#pragma once

#include "GameFramework/Actor.h"
#include <foundation/PxVec3.h>
#include <string>
#include <vector>

namespace PARS
{
    // 충돌용 동적 캡슐의 질량은 캐릭터 PA의 질량·관성·PD와 분리한다.
    struct CharacterImpactSettings
    {
    public:
        bool Validate(std::string& error) const;
        bool Save(const std::string& path, std::string& error) const;
        bool Load(const std::string& path, std::string& error);

    public:
        float collisionMass = 8.154099464416504f;
        float capsuleRadiusScale = .95f;
        float capsuleHeightScale = 1.0f;
        float contactSkin = .02f;
        bool enabled = true;
    };

    struct BodyImpactShare
    {
    public:
        std::string bone;
        physx::PxVec3 impulse{0};
    };

    enum class CharacterImpactReason
    {
        NoContact,
        Applied,
        Disabled
    };

    struct CharacterImpactSnapshot
    {
    public:
        ActorHandle target;
        std::string hitBone;
        std::vector<BodyImpactShare> shares;
        uint64_t eventCount = 0;
        uint64_t step = 0;
        physx::PxVec3 point{0};
        physx::PxVec3 direction{0};
        physx::PxVec3 actualVelocity{0};
        physx::PxVec3 requestedVelocity{0};
        float closingSpeed = 0;
        float normalImpulse = 0;
        CharacterImpactReason reason = CharacterImpactReason::NoContact;
    };
}
