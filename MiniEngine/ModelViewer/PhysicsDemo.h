#pragma once

#include "PhysicsWorld.h"
#include "RootSignature.h"
#include "PipelineState.h"
#include <string>

namespace Math { class Camera; }
class GraphicsContext;

class PhysicsDemo
{
public:
    bool Startup();
    void Cleanup();
    void Update(float deltaTime);
    void Render(GraphicsContext& context, const Math::Camera& camera,
        const D3D12_VIEWPORT& viewport, const D3D12_RECT& scissor);
    void RenderUI();

private:
    void ResetBox();
    void StartDrop();
    void KickBox();
    void RecordHeight();
    void ConnectPvd();
    void UpdatePvdConnection();
    PhysicsWorld m_World;
    physx::PxRigidStatic* m_Floor = nullptr;
    physx::PxRigidDynamic* m_Box = nullptr;
    RootSignature m_RootSignature;
    GraphicsPSO m_PSO;
    bool m_Paused = true;
    bool m_SlowMotion = false;
    static constexpr int HistoryCapacity = 240;
    float m_HeightHistory[HistoryCapacity] = {};
    int m_HistoryCount = 0;
    int m_HistoryOffset = 0;
    bool m_PvdConnecting = false;
    uint64_t m_PvdConnectDeadline = 0;
    uint64_t m_PvdNextAttempt = 0;
    std::string m_PvdMessage;
};
