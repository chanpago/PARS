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
public:
    static constexpr float SphereRadius = .15f;
    RagdollImpactTest(PARS::World& world, FootSoleDebug& footSoles)
        : m_World(world), m_FootSoles(footSoles)
    {
    }
    ~RagdollImpactTest();
    bool Initialize(std::shared_ptr<const Model> idle, const PARS::PhysicsAsset& asset);
    bool Reset();
    void RememberPhysicsAsset(const PARS::PhysicsAsset& asset)
    {
        m_Asset = asset;
    }
    bool SetAutomaticSupport(bool enabled);
    bool UsesAutomaticSupport() const
    {
        return m_AutomaticSupport;
    }
    bool SetProjectileSettings(float mass, float speed);
    bool FireFrom(const PARS::Character& player);
    bool FireAimed(const PARS::Character& player, Math::Vector3 cameraPosition, Math::Vector3 cameraDirection);
    void ClearProjectiles();
    void Update();
    void DrawUI(PARS::Character* player);
    void BuildLines(std::vector<physx::PxDebugLine>& lines);
    PARS::Character* GetDummy() const;
    std::vector<physx::PxRigidDynamic*> GetProjectiles() const;
    float GetMaxHipsDisplacement() const
    {
        return m_MaxHipsDisplacement;
    }
    const std::string& GetStatusMessage() const
    {
        return m_Message;
    }

private:
    bool FireAt(const PARS::Character& player, Math::Vector3 target, bool checkMuzzle = false);
    struct Shot
    {
        physx::PxRigidDynamic* body;
        uint64_t expiresAtStep;
    };
    PARS::World& m_World;
    PARS::ActorHandle m_Dummy;
    PARS::PhysicsAsset m_Asset;
    FootSoleDebug& m_FootSoles;
    std::vector<Shot> m_Shots;
    Math::Vector3 m_HipsReference{Math::kZero};
    float m_Mass = .5f, m_Speed = 7.5f, m_MaxHipsDisplacement = 0;
    bool m_AutomaticSupport = true;
    std::string m_Message;
};
