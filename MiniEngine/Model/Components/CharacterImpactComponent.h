#pragma once

#include "GameFramework/ActorComponent.h"
#include "../CharacterImpact.h"

namespace PARS
{
    class CharacterImpactComponent : public ActorComponent
    {
    public:
        using ActorComponent::ActorComponent;

    public:
        void BeginFixedStep(float deltaTime);
        Math::Vector3 ConstrainMovement(Math::Vector3 displacement, float deltaTime);
        void RefreshCollisionPolicy();

    protected:
        void EndPlay() override;

    private:
        void RecordContacts();

    public:
        const CharacterImpactSettings& GetSettings() const
        {
            return m_Settings;
        }
        bool SetSettings(const CharacterImpactSettings& settings, std::string& error);
        const CharacterImpactSnapshot& GetSnapshot();
        const CharacterImpactSnapshot& GetLastImpact();
        const std::vector<CharacterImpactSnapshot>& GetAppliedImpacts();
        const std::string& GetPhysicsError() const
        {
            return m_PhysicsError;
        }

    private:
        std::vector<CharacterImpactSnapshot> m_AppliedImpacts;
        CharacterImpactSnapshot m_Snapshot;
        CharacterImpactSnapshot m_LastImpact;
        std::string m_PhysicsError;
        uint64_t m_ClockRevision = 0;
        uint64_t m_RecordedStep = UINT64_MAX;
        uint64_t m_SourceRevision = 0;
        CharacterImpactSettings m_Settings;
    };
}
