#pragma once

#include "GameFramework/ActorComponent.h"
#include "../CharacterImpact.h"

namespace PARS
{
    class Character;

    class CharacterImpactComponent : public ActorComponent
    {
    private:
        struct Contact
        {
        public:
            ActorHandle target;
            uint64_t revision = 0;
            uint64_t lastSeenStep = 0;
            double lastSeen = 0;
            double lastEvent = -1000;
            bool latched = false;
        };

        struct PendingImpact
        {
        public:
            ActorHandle target;
            uint64_t targetRevision = 0;
            uint64_t sourceRevision = 0;
            uint64_t clockRevision = 0;
            physx::PxVec3 point{0};
            physx::PxVec3 impulse{0};
        };

    public:
        using ActorComponent::ActorComponent;

    public:
        void BeginFixedStep(float deltaTime);
        Math::Vector3 ConstrainMovement(Math::Vector3 displacement, float deltaTime);
        void ApplyPhysicsInputs(float deltaTime);
        void RefreshCollisionPolicy();

    protected:
        void EndPlay() override;

    public:
        const CharacterImpactSettings& GetSettings() const
        {
            return m_Settings;
        }
        bool SetSettings(const CharacterImpactSettings& settings, std::string& error);
        const CharacterImpactSnapshot& GetSnapshot() const
        {
            return m_Snapshot;
        }
        const CharacterImpactSnapshot& GetLastImpact() const
        {
            return m_LastImpact;
        }

    private:
        std::vector<Contact> m_Contacts;
        std::vector<PendingImpact> m_Pending;
        std::vector<ActorHandle> m_AppliedPartners;
        CharacterImpactSnapshot m_Snapshot;
        CharacterImpactSnapshot m_LastImpact;
        uint64_t m_ClockRevision = 0;
        uint64_t m_SourceRevision = 0;
        double m_Time = 0;
        CharacterImpactSettings m_Settings;
    };
}
