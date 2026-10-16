#include "SkeletonOverlay.h"
#include "GraphicsCommon.h"
#include "CompiledShaders/PhysicsAssetVS.h"
#include "CompiledShaders/PhysicsAssetPS.h"
#include <cmath>

using namespace Math;
using namespace Graphics;

void SkeletonOverlay::InitializeSurfaces()
{
    m_SurfaceDepth.Create(L"PhysicsAsset picking depth", m_Resolution, m_Resolution, DXGI_FORMAT_D32_FLOAT);
    m_SurfaceRoot.Reset(1);
    m_SurfaceRoot[0].InitAsConstantBuffer(0);
    m_SurfaceRoot.Finalize(L"PhysicsAsset surfaces", D3D12_ROOT_SIGNATURE_FLAG_ALLOW_INPUT_ASSEMBLER_INPUT_LAYOUT);
    const D3D12_INPUT_ELEMENT_DESC layout[] = {
        {"POSITION", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, 0, D3D12_INPUT_CLASSIFICATION_PER_VERTEX_DATA, 0},
        {"NORMAL", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, 12, D3D12_INPUT_CLASSIFICATION_PER_VERTEX_DATA, 0},
        {"COLOR", 0, DXGI_FORMAT_R32G32B32A32_FLOAT, 0, 24, D3D12_INPUT_CLASSIFICATION_PER_VERTEX_DATA, 0},
        {"TEXCOORD", 0, DXGI_FORMAT_R32_UINT, 0, 40, D3D12_INPUT_CLASSIFICATION_PER_VERTEX_DATA, 0}};
    auto blend = BlendTraditional;
    blend.IndependentBlendEnable = TRUE;
    blend.RenderTarget[1] = BlendDisable.RenderTarget[0]; // Exact IDs, regardless of opacity.
    m_SurfacePSO.SetRootSignature(m_SurfaceRoot);
    m_SurfacePSO.SetInputLayout(_countof(layout), layout);
    m_SurfacePSO.SetRasterizerState(RasterizerDefault);
    m_SurfacePSO.SetBlendState(blend);
    m_SurfacePSO.SetDepthStencilState(DepthStateReadWrite);
    m_SurfacePSO.SetPrimitiveTopologyType(D3D12_PRIMITIVE_TOPOLOGY_TYPE_TRIANGLE);
    const DXGI_FORMAT formats[] = {DXGI_FORMAT_R8G8B8A8_UNORM, DXGI_FORMAT_R8G8B8A8_UNORM};
    m_SurfacePSO.SetRenderTargetFormats(2, formats, DXGI_FORMAT_D32_FLOAT);
    m_SurfacePSO.SetVertexShader(g_pPhysicsAssetVS, sizeof(g_pPhysicsAssetVS));
    m_SurfacePSO.SetPixelShader(g_pPhysicsAssetPS, sizeof(g_pPhysicsAssetPS));
    m_SurfacePSO.Finalize();
}

void SkeletonOverlay::Body(const PARS::PhysicsBody& body, const Matrix4& world, Vector3 color, uint32_t id, float opacity)
{
    const auto vertex = [&](Vector3 p, Vector3 n) {
        SurfaceVertex v;
        DirectX::XMStoreFloat3(&v.position, world * p);
        DirectX::XMStoreFloat3(&v.normal, Normalize(Vector3(world * Vector4(n, 0))));
        v.color = DirectX::XMFLOAT4(color.GetX(), color.GetY(), color.GetZ(), opacity);
        v.id = id;
        m_SurfaceVertices.push_back(v);
    };
    if (body.shape == PARS::PhysicsShapeType::Box)
    {
        const Vector3 normals[] = {Vector3(1,0,0), Vector3(-1,0,0), Vector3(0,1,0), Vector3(0,-1,0), Vector3(0,0,1), Vector3(0,0,-1)};
        const Vector3 tangents[] = {Vector3(0,1,0), Vector3(0,1,0), Vector3(0,0,1), Vector3(0,0,1), Vector3(1,0,0), Vector3(1,0,0)};
        const float corners[6][2] = {{-1,-1},{1,-1},{1,1},{-1,-1},{1,1},{-1,1}};
        for (int face = 0; face < 6; ++face)
            for (const auto& corner : corners)
                vertex((normals[face] + tangents[face] * corner[0] + Cross(normals[face], tangents[face]) * corner[1]) * Vector3(body.halfExtents), normals[face]);
        return;
    }
    // Two hemispheres and an explicit cylinder band. Duplicated equators keep
    // the capsule's cylindrical section straight, including halfLength == 0.
    constexpr int sides = 32, hemisphere = 10, rings = hemisphere * 2 + 1;
    const float halfLength = body.shape == PARS::PhysicsShapeType::Capsule ? body.halfLength : 0;
    const auto sample = [&](int ring, int side, Vector3& p, Vector3& n) {
        const bool positive = ring > hemisphere;
        const int latitude = positive ? ring - 1 : ring;
        const float angle = -XM_PIDIV2 + latitude * XM_PI / (hemisphere * 2);
        const float longitude = side * XM_2PI / sides;
        n = Vector3(std::sin(angle), std::cos(angle) * std::cos(longitude), std::cos(angle) * std::sin(longitude));
        p = n * body.radius + Vector3(positive ? halfLength : -halfLength, 0, 0);
    };
    for (int ring = 0; ring < rings; ++ring)
        for (int side = 0; side < sides; ++side)
        {
            Vector3 p[4], n[4];
            sample(ring, side, p[0], n[0]); sample(ring, side + 1, p[1], n[1]);
            sample(ring + 1, side, p[2], n[2]); sample(ring + 1, side + 1, p[3], n[3]);
            for (int index : {0, 1, 2, 1, 3, 2}) vertex(p[index], n[index]);
        }
}
