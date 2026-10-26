#pragma once

#include "PhysicsWorld.h"
#include "RootSignature.h"
#include "PipelineState.h"
#include <string>

namespace Math
{
    class Camera;
}
namespace PARS
{
    class World;
}
class GraphicsContext;

class PhysicsDemo
{
public:
    explicit PhysicsDemo(PARS::World& world);
    bool Startup();
    void Cleanup();
    void Update(float deltaTime);
    void Render(GraphicsContext& context, const Math::Camera& camera,
        const D3D12_VIEWPORT& viewport, const D3D12_RECT& scissor, bool comparisonScene = false);
    void RenderUI();
    void RenderCapsules(GraphicsContext& context, const Math::Camera& camera,
        const D3D12_VIEWPORT& viewport, const D3D12_RECT& scissor);
    void RenderDebugLines(GraphicsContext& context, const Math::Camera& camera,
        const D3D12_VIEWPORT& viewport, const D3D12_RECT& scissor,
        const std::vector<physx::PxDebugLine>& lines, bool throughGeometry = false);

private:
    void ResetBox();
    void StartDrop();
    void KickBox();
    void RecordHeight();
    void ConnectPvd();
    void RestartPvd();
    void UpdatePvdConnection();
    PARS::World& m_GameWorld;
    PhysicsWorld& m_World;
    physx::PxRigidStatic* m_Floor = nullptr;
    physx::PxRigidDynamic* m_Box = nullptr;
    RootSignature m_RootSignature;
    GraphicsPSO m_PSO;
    GraphicsPSO m_LinePSO;
    GraphicsPSO m_OverlayLinePSO;
    physx::PxRigidStatic* m_Walls[5] = {};
    bool m_DropReady = true;
    static constexpr int HistoryCapacity = 240;
    float m_HeightHistory[HistoryCapacity] = {};
    int m_HistoryCount = 0;
    int m_HistoryOffset = 0;
    bool m_PvdConnecting = false;
    uint32_t m_PvdProcessId = 0;
    bool m_PvdUnresponsive = false;
    uint64_t m_PvdConnectDeadline = 0;
    uint64_t m_PvdNextAttempt = 0;
    std::string m_PvdMessage;
    bool m_PvdSceneQueries = false;
};
