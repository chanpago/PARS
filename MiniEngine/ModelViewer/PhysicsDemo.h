#pragma once

#include "PhysicsWorld.h"
#include "RootSignature.h"
#include "PipelineState.h"
#include <memory>
#include <string>

namespace Math
{
    class AffineTransform;
    class Camera;
}
namespace PARS
{
    class World;
}
class GraphicsContext;
class ModelInstance;
namespace Renderer
{
    class MeshSorter;
}
struct ImpactProjectileVisual;

class PhysicsDemo
{
public:
    enum class MapId : uint8_t
    {
        TestYard,
        Flat,
        Stairs,
        Cliff,
        Gatehouse,
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
    ~PhysicsDemo();

public:
    bool Startup(MapId map = MapId::TestYard, const std::wstring& sceneryPath = {});
    bool LoadMap(MapId map, const std::wstring& sceneryPath = {});
    void Cleanup();
    void Update(float deltaTime);
    void Render(GraphicsContext& context, const Math::Camera& camera,
        const D3D12_VIEWPORT& viewport, const D3D12_RECT& scissor, bool comparisonScene = false);
    void RenderUI();
    void RenderScenery(Renderer::MeshSorter& sorter) const;
    void RenderProjectiles(GraphicsContext& context, const Math::Camera& camera,
        const D3D12_VIEWPORT& viewport, const D3D12_RECT& scissor,
        const std::vector<ImpactProjectileVisual>& projectiles);
    void RenderCapsules(GraphicsContext& context, const Math::Camera& camera,
        const D3D12_VIEWPORT& viewport, const D3D12_RECT& scissor);
    void RenderDebugLines(GraphicsContext& context, const Math::Camera& camera,
        const D3D12_VIEWPORT& viewport, const D3D12_RECT& scissor,
        const std::vector<physx::PxDebugLine>& lines, bool throughGeometry = false);
    void RenderDebugTriangles(GraphicsContext& context, const Math::Camera& camera,
        const D3D12_VIEWPORT& viewport, const D3D12_RECT& scissor,
        const std::vector<physx::PxDebugTriangle>& triangles);

private:
    bool PrepareGatehouse(const std::wstring& path);
    bool AddMapBox(std::vector<MapBox>& boxes, const physx::PxTransform& pose,
        const physx::PxVec3& halfExtents, const physx::PxVec3& color, const char* name,
        const PARS::PhysicsMaterialSettings& material = {});
    void ReleaseMap();
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
    Math::AffineTransform GetPlayerSpawn(const Math::AffineTransform& current, float halfHeight) const;
    Math::Vector3 GetDummySpawnOrigin() const;

private:
    PARS::World& m_GameWorld;
    PhysicsWorld& m_World;
    RootSignature m_RootSignature;
    GraphicsPSO m_PSO;
    GraphicsPSO m_LinePSO;
    GraphicsPSO m_OverlayLinePSO;
    GraphicsPSO m_RegionPSO;
    std::vector<MapBox> m_MapBoxes;
    std::unique_ptr<ModelInstance> m_Gatehouse;
    std::vector<physx::PxBounds3> m_GatehouseBounds;
    uint64_t m_PvdConnectDeadline = 0;
    uint64_t m_PvdNextAttempt = 0;
    std::string m_PvdMessage;
    uint32_t m_PvdProcessId = 0;
    MapId m_Map = MapId::TestYard;
    bool m_PvdConnecting = false;
    bool m_PvdUnresponsive = false;
    bool m_PvdSceneQueries = false;
};
