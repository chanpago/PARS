#pragma once

#include "FootSoleDebug.h"
#include "GameFramework/Actor.h"

namespace PARS { class World; class Character; }

class RagdollImpactTest
{
public:
    static constexpr float SphereRadius = .3f;
    explicit RagdollImpactTest(PARS::World& world) : m_World(world) {}
    ~RagdollImpactTest();
    bool Initialize(std::shared_ptr<const Model> idle, const PARS::PhysicsAsset& asset);
    bool Reset();
    bool FireFrom(const PARS::Character& player);
    void ClearProjectiles();
    void Update();
    void DrawUI(PARS::Character* player);
    void BuildLines(std::vector<physx::PxDebugLine>& lines);
    PARS::Character* GetDummy() const;
    std::vector<physx::PxRigidDynamic*> GetProjectiles() const;
    float GetMaxHipsDisplacement() const { return m_MaxHipsDisplacement; }
    const std::string& GetStatusMessage() const { return m_Message; }

private:
    struct Shot { physx::PxRigidDynamic* body; uint64_t expiresAtStep; };
    PARS::World& m_World;
    PARS::ActorHandle m_Dummy;
    PARS::PhysicsAsset m_Asset;
    FootSoleDebug m_FootSoles;
    std::vector<Shot> m_Shots;
    Math::Vector3 m_HipsReference{Math::kZero};
    float m_Mass = 10, m_Speed = 15, m_MaxHipsDisplacement = 0;
    std::string m_Message;
};
