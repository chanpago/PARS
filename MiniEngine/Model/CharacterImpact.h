#pragma once

#include "GameFramework/Actor.h"
#include <foundation/PxVec3.h>
#include <string>
#include <vector>

namespace PARS
{
    // 게임플레이 밀침의 입력 단위는 N s다. 관절 PD와 PA 설정은 변경하지 않는다.
    struct CharacterImpactSettings
    {
    public:
        bool Validate(std::string& error) const;
        bool Save(const std::string& path, std::string& error) const;
        bool Load(const std::string& path, std::string& error);
        float ComputeImpulse(float closingSpeed, float& requested) const;

    public:
        float referenceImpulse = 180.0f;
        float referenceSpeed = 3.5f;
        float strengthMultiplier = 1.0f;
        float maximumImpulse = 500.0f;
        float minimumClosingSpeed = .35f;
        float pelvisShare = .6f;
        float separationTime = .1f;
        float separationDistance = .08f;
        float cooldown = .35f;
        float capsuleRadiusScale = .95f;
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
        BelowMinimumSpeed,
        HeldContact,
        Cooldown,
        Queued,
        Applied,
        StaleTarget,
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
        float requestedImpulse = 0;
        float commandedImpulse = 0;
        CharacterImpactReason reason = CharacterImpactReason::NoContact;
        bool capped = false;
    };
}
