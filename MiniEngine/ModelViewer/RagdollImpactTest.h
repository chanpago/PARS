#pragma once

#include "FootSoleDebug.h"
#include "ImpactProjectile.h"
#include "PhysicsAsset.h"
#include "GameFramework/Actor.h"

class Model;
namespace PARS
{
    class AnimInstance;
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
        float radius;
        float length;
        ImpactProjectileType type;
    };

public:
    RagdollImpactTest(PARS::World& world, FootSoleDebug& footSoles)
        : m_World(world), m_FootSoles(footSoles)
    {
    }
    ~RagdollImpactTest();

public:
    bool Initialize(std::shared_ptr<const Model> idle, const PARS::PhysicsAsset& asset, const PARS::AnimInstance& motionSource);
    bool Reset();
    bool SetAutomaticSupport(bool enabled);
    bool FireFrom(const PARS::Character& player);
    bool FireAimed(const PARS::Character& player, Math::Vector3 cameraPosition, Math::Vector3 cameraDirection);
    void ClearProjectiles();
    void Update();
    void ResumeSimulation();
    void DrawUI(PARS::Character* player);
    void DrawContactOverlay(float topInset) const;
    bool LoadImpactSettings(PARS::Character& player, const std::string& path);
    bool SaveImpactSettings(const PARS::Character& player);
    void LoadBodyImpactSettings(PARS::Character& player, const std::string& path);
    bool LoadDummySupportSettings(const std::string& path);
    bool SaveDummySupportSettings();
    void BuildLines(std::vector<physx::PxDebugLine>& lines);

private:
    void ClearContactCheck();
    void DrawBodyImpactUI(PARS::Character* player);
    void DrawImpactSettingsUI(PARS::Character* player);
    void DrawDummySupportSettingsUI();
    void BuildBodyImpactLines(std::vector<physx::PxDebugLine>& lines);
    bool FireAt(const PARS::Character& player, Math::Vector3 target, bool aimed = false);

public:
    void SetContactCheckEnabled(bool enabled);
    bool IsContactCheckEnabled() const
    {
        return m_ContactCheckEnabled;
    }
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
    bool SetProjectileSettings(float mass, float speed, float bodyImpulse = 0, float torqueMultiplier = 0,
        float partImpulse = 0, float radius = DefaultProjectileRadius);
    bool SetProjectileSettings(ImpactProjectileType type, const ImpactProjectileSettings& settings);
    PARS::Character* GetDummy() const;
    std::vector<physx::PxRigidDynamic*> GetProjectiles() const;
    std::vector<ImpactProjectileVisual> GetProjectileVisuals() const;
    float GetMaxHipsDisplacement() const
    {
        return m_MaxHipsDisplacement;
    }
    const std::string& GetStatusMessage() const
    {
        return m_Message;
    }

private:
    const ImpactProjectileSettings& GetSelectedProjectileSettings() const;

public:
    static constexpr float DefaultProjectileRadius = ImpactProjectileSettings{}.radius;

private:
    Math::Vector3 m_HipsReference{Math::kZero};
    Math::Vector3 m_SpawnOrigin{3.5f, 0, -2.5f};
    Math::AffineTransform m_SpawnTransform{Math::kIdentity};
    PARS::World& m_World;
    PARS::ActorHandle m_Dummy;
    PARS::PhysicsAsset m_Asset;
    FootSoleDebug& m_FootSoles;
    std::vector<Shot> m_Shots;
    std::string m_Message;
    std::string m_ImpactSettingsPath;
    std::string m_ImpactSettingsMessage;
    std::string m_BodyImpactPath;
    std::string m_BodyImpactMessage;
    std::string m_DummySupportSettingsPath;
    std::string m_DummySupportSettingsMessage;
    std::string m_LegContactMessage;
    std::string m_PausedLegContactMessage;
    uint32_t m_PreviousLegContacts = 0;
    ImpactProjectileSettings m_SphereSettings;
    ImpactProjectileSettings m_ArrowSettings = ImpactProjectileSettings::ArrowDefaults();
    float m_MaxHipsDisplacement = 0;
    ImpactProjectileType m_ProjectileType = ImpactProjectileType::Sphere;
    bool m_EditingCapsuleImpact = false;
    bool m_AutomaticSupport = true;
    bool m_BodyImpactCapsuleVisible = true;
    bool m_ContactCheckEnabled = false;
};
