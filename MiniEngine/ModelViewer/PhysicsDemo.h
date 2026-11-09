#pragma once

#include "PhysicsWorld.h"
#include "RootSignature.h"
#include "PipelineState.h"
#include "GpuBuffer.h"
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
    enum class MapId : uint8_t
    {
        TestYard,
        Flat,
        Stairs,
        Cliff,
        Count
    };

private:
    struct MapBox
    {
    public:
        physx::PxRigidStatic* body;
        physx::PxVec3 halfExtents;
        physx::PxVec3 color;
    };

public:
    explicit PhysicsDemo(PARS::World& world);

public:
    bool Startup(MapId map = MapId::TestYard);
    bool LoadMap(MapId map);
    void Cleanup();
    void Update(float deltaTime);
    void Render(GraphicsContext& context, const Math::Camera& camera,
        const D3D12_VIEWPORT& viewport, const D3D12_RECT& scissor, bool comparisonScene = false);
    void RenderUI();
    void RenderSpheres(GraphicsContext& context, const Math::Camera& camera,
        const D3D12_VIEWPORT& viewport, const D3D12_RECT& scissor,
        const std::vector<physx::PxRigidDynamic*>& spheres, float radius);
    void RenderCapsules(GraphicsContext& context, const Math::Camera& camera,
        const D3D12_VIEWPORT& viewport, const D3D12_RECT& scissor);
    void RenderDebugLines(GraphicsContext& context, const Math::Camera& camera,
        const D3D12_VIEWPORT& viewport, const D3D12_RECT& scissor,
        const std::vector<physx::PxDebugLine>& lines, bool throughGeometry = false);
    void RenderRuler(GraphicsContext& context, const Math::Camera& camera,
        const D3D12_VIEWPORT& viewport, const D3D12_RECT& scissor, Math::Vector3 origin);

private:
    bool AddMapBox(std::vector<MapBox>& boxes, const physx::PxTransform& pose,
        const physx::PxVec3& halfExtents, const physx::PxVec3& color, const char* name,
        const PARS::PhysicsMaterialSettings& material = {});
    void ReleaseMap();
    void ResetBox();
    void StartDrop();
    void KickBox();
    void RecordHeight();
    void ConnectPvd();
    void RestartPvd();
    void UpdatePvdConnection();

public:
    static const char* GetMapName(MapId map);
    static const char* GetMapDescription(MapId map);
    MapId GetMap() const
    {
        return m_Map;
    }
    Math::Vector3 GetSpawnLocation(float halfHeight) const;

private:
    static constexpr int HistoryCapacity = 240;

private:
    PARS::World& m_GameWorld;
    PhysicsWorld& m_World;
    physx::PxRigidDynamic* m_Box = nullptr;
    RootSignature m_RootSignature;
    GraphicsPSO m_PSO;
    GraphicsPSO m_LinePSO;
    GraphicsPSO m_OverlayLinePSO;
    ByteAddressBuffer m_RulerMesh;
    std::vector<MapBox> m_MapBoxes;
    uint64_t m_PvdConnectDeadline = 0;
    uint64_t m_PvdNextAttempt = 0;
    std::string m_PvdMessage;
    float m_HeightHistory[HistoryCapacity] = {};
    int m_HistoryCount = 0;
    int m_HistoryOffset = 0;
    uint32_t m_PvdProcessId = 0;
    MapId m_Map = MapId::TestYard;
    bool m_DropReady = true;
    bool m_PvdConnecting = false;
    bool m_PvdUnresponsive = false;
    bool m_PvdSceneQueries = false;
};
