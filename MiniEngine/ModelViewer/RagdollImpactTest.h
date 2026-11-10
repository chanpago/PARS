#pragma once

#include "FootSoleDebug.h"
#include "PhysicsAsset.h"
#include "GameFramework/Actor.h"

class Model;
namespace PARS
{
    class World;
    class Character;
}

class RagdollImpactTest
{
private:
    struct Shot
    {
    public:
        physx::PxRigidDynamic* body;
        uint64_t expiresAtStep;
    };

public:
    RagdollImpactTest(PARS::World& world, FootSoleDebug& footSoles)
        : m_World(world), m_FootSoles(footSoles)
    {
    }
    ~RagdollImpactTest();

public:
    bool Initialize(std::shared_ptr<const Model> idle, std::shared_ptr<const Model> falling, const PARS::PhysicsAsset& asset);
    bool Reset();
    bool SetAutomaticSupport(bool enabled);
    bool FireFrom(const PARS::Character& player);
    bool FireAimed(const PARS::Character& player, Math::Vector3 cameraPosition, Math::Vector3 cameraDirection);
    void ClearProjectiles();
    void Update();
    void DrawUI(PARS::Character* player);
    bool LoadProjectileSettings(const std::string& path);
    bool SaveProjectileSettings();
    void LoadBodyImpactSettings(PARS::Character& player, const std::string& path);
    bool LoadDummySupportSettings(const std::string& path);
    bool SaveDummySupportSettings();
    void BuildLines(std::vector<physx::PxDebugLine>& lines);

private:
    void DrawBodyImpactUI(PARS::Character* player);
    void DrawDummySupportSettingsUI();
    void BuildBodyImpactLines(std::vector<physx::PxDebugLine>& lines);
    bool FireAt(const PARS::Character& player, Math::Vector3 target, bool checkMuzzle = false);

public:
    void SetSpawnOrigin(Math::Vector3 origin)
    {
        m_SpawnOrigin = origin;
    }
    void RememberPhysicsAsset(const PARS::PhysicsAsset& asset)
    {
        m_Asset = asset;
    }
    bool UsesAutomaticSupport() const
    {
        return m_AutomaticSupport;
    }
    bool SetProjectileSettings(float mass, float speed);
    PARS::Character* GetDummy() const;
    bool IsRulerVisible() const
    {
        return m_RulerVisible;
    }
    Math::Vector3 GetRulerOrigin() const
    {
        return m_RulerOrigin;
    }
    std::vector<physx::PxRigidDynamic*> GetProjectiles() const;
    float GetMaxHipsDisplacement() const
    {
        return m_MaxHipsDisplacement;
    }
    const std::string& GetStatusMessage() const
    {
        return m_Message;
    }

public:
    static constexpr float SphereRadius = .15f;

private:
    Math::Vector3 m_HipsReference{Math::kZero};
    Math::Vector3 m_RulerOrigin{Math::kZero};
    Math::Vector3 m_SpawnOrigin{3.5f, 0, -2.5f};
    PARS::World& m_World;
    PARS::ActorHandle m_Dummy;
    PARS::PhysicsAsset m_Asset;
    FootSoleDebug& m_FootSoles;
    std::vector<Shot> m_Shots;
    std::string m_Message;
    std::string m_ProjectileSettingsPath;
    std::string m_ProjectileSettingsMessage;
    std::string m_BodyImpactPath;
    std::string m_BodyImpactMessage;
    std::string m_DummySupportSettingsPath;
    std::string m_DummySupportSettingsMessage;
    float m_Mass = 2.0f;
    float m_Speed = 7.5f;
    float m_MaxHipsDisplacement = 0;
    bool m_AutomaticSupport = true;
    bool m_RulerVisible = true;
    bool m_BodyImpactCapsuleVisible = true;
};
