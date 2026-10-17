#pragma once

#include "GraphicsCore.h"
#include "Camera.h"
#include "ColorBuffer.h"
#include "ReadbackBuffer.h"
#include "PipelineState.h"
#include "RootSignature.h"
#include "DepthBuffer.h"
#include "PhysicsAsset.h"
#include <vector>

// Screen-sized bones/gizmos and translucent world-space collision surfaces.
// Visible color and exact 24-bit IDs are rasterized together; only visible
// color is blended, so selection never uses a CPU triangle/ray test.
class SkeletonOverlay
{
public:
    void Initialize(uint32_t resolution);
    void Cleanup(); // GPU must be idle.
    void Begin(const Math::Camera& camera, float width, float height);
    bool Project(Math::Vector3 position, Math::Vector3& pixel) const;
    void Line(Math::Vector3 start, Math::Vector3 end, Math::Vector3 color, uint32_t id, float width = 3);
    void Point(Math::Vector3 position, Math::Vector3 color, uint32_t id, float radius = 4);
    void FillTriangle(Math::Vector3 a, Math::Vector3 b, Math::Vector3 c, Math::Vector3 color, uint32_t id, float opacity);
    void Body(const PARS::PhysicsBody& body, const Math::Matrix4& world, Math::Vector3 color, uint32_t id, float opacity);
    void ConstraintLimits(const PARS::PhysicsConstraint& joint, const Math::Matrix4& parent,
        const Math::Matrix4& child, uint32_t id, bool selected);
    void Render(GraphicsContext& context, ColorBuffer& target);
    bool RequestPick(float x, float y);
    bool PollPick(uint32_t& id);
    void CancelPick() { m_AcceptPick = false; m_PickRequested = false; }

private:
    struct Vertex { float x, y, z, w; float r, g, b, a; uint32_t id; };
    struct SurfaceVertex { DirectX::XMFLOAT3 position, normal; DirectX::XMFLOAT4 color; uint32_t id; };
    void InitializeSurfaces();
    void Triangle(Math::Vector3 a, Math::Vector3 b, Math::Vector3 c, Math::Vector3 color, uint32_t id);
    Math::Matrix4 m_ViewProjection{Math::kIdentity};
    float m_Width = 1, m_Height = 1;
    uint32_t m_Resolution = 0, m_PickX = 0, m_PickY = 0;
    std::vector<Vertex> m_Vertices;
    std::vector<Vertex> m_FillVertices; // Translucent limits, drawn before all opaque lines/gizmos.
    std::vector<SurfaceVertex> m_SurfaceVertices;
    Math::Vector3 m_Eye{Math::kZero};
    DepthBuffer m_SurfaceDepth;
    RootSignature m_SurfaceRoot;
    GraphicsPSO m_SurfacePSO;
    ColorBuffer m_Ids{Color(0.0f, 0.0f, 0.0f, 0.0f)};
    ByteAddressBuffer m_PickResult;
    ReadbackBuffer m_Readback;
    RootSignature m_DrawRoot, m_PickRoot;
    GraphicsPSO m_DrawPSO;
    ComputePSO m_PickPSO;
    uint64_t m_PickFence = 0;
    bool m_PickRequested = false, m_AcceptPick = false;
};
