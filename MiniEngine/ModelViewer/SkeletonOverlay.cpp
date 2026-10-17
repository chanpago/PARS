#include "SkeletonOverlay.h"
#include "CommandContext.h"
#include "CommandListManager.h"
#include "GraphicsCommon.h"
#include "CompiledShaders/SkeletonOverlayVS.h"
#include "CompiledShaders/SkeletonOverlayPS.h"
#include "CompiledShaders/SkeletonPickCS.h"
#include <cmath>

using namespace Math;
using namespace Graphics;

void SkeletonOverlay::Initialize(uint32_t resolution)
{
    m_Resolution = resolution;
    m_Ids.Create(L"Skeleton color IDs", resolution, resolution, 1, DXGI_FORMAT_R8G8B8A8_UNORM);
    m_PickResult.Create(L"Skeleton picked ID", 1, sizeof(uint32_t));
    m_Readback.Create(L"Skeleton ID readback", 1, sizeof(uint32_t));
    m_DrawRoot.Reset(0);
    m_DrawRoot.Finalize(L"Skeleton overlay", D3D12_ROOT_SIGNATURE_FLAG_ALLOW_INPUT_ASSEMBLER_INPUT_LAYOUT);
    const D3D12_INPUT_ELEMENT_DESC layout[] = {
        {"POSITION", 0, DXGI_FORMAT_R32G32B32A32_FLOAT, 0, 0, D3D12_INPUT_CLASSIFICATION_PER_VERTEX_DATA, 0},
        {"COLOR", 0, DXGI_FORMAT_R32G32B32A32_FLOAT, 0, 16, D3D12_INPUT_CLASSIFICATION_PER_VERTEX_DATA, 0},
        {"TEXCOORD", 0, DXGI_FORMAT_R32_UINT, 0, 32, D3D12_INPUT_CLASSIFICATION_PER_VERTEX_DATA, 0}};
    m_DrawPSO.SetRootSignature(m_DrawRoot);
    m_DrawPSO.SetInputLayout(_countof(layout), layout);
    m_DrawPSO.SetRasterizerState(RasterizerTwoSided);
    auto blend = BlendTraditional;
    blend.IndependentBlendEnable = TRUE;
    blend.RenderTarget[1] = BlendDisable.RenderTarget[0]; // IDs stay exact even on translucent limits.
    m_DrawPSO.SetBlendState(blend);
    m_DrawPSO.SetDepthStencilState(DepthStateDisabled); // X-ray bones, as in the reference viewer.
    m_DrawPSO.SetPrimitiveTopologyType(D3D12_PRIMITIVE_TOPOLOGY_TYPE_TRIANGLE);
    const DXGI_FORMAT formats[] = {DXGI_FORMAT_R8G8B8A8_UNORM, DXGI_FORMAT_R8G8B8A8_UNORM};
    m_DrawPSO.SetRenderTargetFormats(2, formats, DXGI_FORMAT_UNKNOWN);
    m_DrawPSO.SetVertexShader(g_pSkeletonOverlayVS, sizeof(g_pSkeletonOverlayVS));
    m_DrawPSO.SetPixelShader(g_pSkeletonOverlayPS, sizeof(g_pSkeletonOverlayPS));
    m_DrawPSO.Finalize();
    InitializeSurfaces();

    m_PickRoot.Reset(3);
    m_PickRoot[0].InitAsConstants(0, 2);
    m_PickRoot[1].InitAsDescriptorRange(D3D12_DESCRIPTOR_RANGE_TYPE_SRV, 0, 1);
    m_PickRoot[2].InitAsBufferUAV(0);
    m_PickRoot.Finalize(L"Read skeleton color ID");
    m_PickPSO.SetRootSignature(m_PickRoot);
    m_PickPSO.SetComputeShader(g_pSkeletonPickCS, sizeof(g_pSkeletonPickCS));
    m_PickPSO.Finalize();
}

void SkeletonOverlay::Cleanup()
{
    m_Ids.Destroy();
    m_SurfaceDepth.Destroy();
    m_SurfaceVertices.clear();
    m_PickResult.Destroy();
    m_Readback.Destroy();
    m_PickFence = 0;
    CancelPick();
    m_Vertices.clear();
    m_FillVertices.clear();
}

void SkeletonOverlay::Begin(const Camera& camera, float width, float height)
{
    m_ViewProjection = camera.GetViewProjMatrix();
    m_Eye = camera.GetPosition();
    m_SurfaceVertices.clear();
    m_Width = Max(width, 1.0f);
    m_Height = Max(height, 1.0f);
    m_Vertices.clear();
    m_FillVertices.clear();
}

bool SkeletonOverlay::Project(Vector3 position, Vector3& pixel) const
{
    const Vector4 clip = m_ViewProjection * position;
    const float w = clip.GetW(), z = clip.GetZ();
    if (w <= 1e-6f || z < 0 || z > w) return false;
    const float x = clip.GetX() / w, y = clip.GetY() / w;
    if (!std::isfinite(x) || !std::isfinite(y)) return false;
    pixel = Vector3((x + 1) * .5f * m_Width, (1 - y) * .5f * m_Height, z / w);
    return true;
}

void SkeletonOverlay::Triangle(Vector3 a, Vector3 b, Vector3 c, Vector3 color, uint32_t id)
{
    for (Vector3 p : {a, b, c})
        m_Vertices.push_back({float(p.GetX()) * 2 / m_Width - 1, 1 - float(p.GetY()) * 2 / m_Height, 0, 1,
            color.GetX(), color.GetY(), color.GetZ(), 1, id});
}

void SkeletonOverlay::FillTriangle(Vector3 a, Vector3 b, Vector3 c, Vector3 color, uint32_t id, float opacity)
{
    // Keep homogeneous coordinates: the GPU clips triangles crossing the near
    // plane instead of dropping the entire cone when its pivot leaves view.
    for (Vector3 p : {a, b, c})
    {
        const Vector4 clip = m_ViewProjection * p;
        m_FillVertices.push_back({clip.GetX(), clip.GetY(), clip.GetZ(), clip.GetW(),
            color.GetX(), color.GetY(), color.GetZ(), opacity, id});
    }
}

void SkeletonOverlay::Line(Vector3 start, Vector3 end, Vector3 color, uint32_t id, float width)
{
    // Clip in homogeneous space before dividing by W (also supports reversed Z).
    Vector4 a = m_ViewProjection * start, b = m_ViewProjection * end;
    const auto distance = [](Vector4 p, int plane) -> float {
        switch (plane) {
        case 0: return p.GetW() + p.GetX(); case 1: return p.GetW() - p.GetX();
        case 2: return p.GetW() + p.GetY(); case 3: return p.GetW() - p.GetY();
        case 4: return p.GetZ(); default: return p.GetW() - p.GetZ();
        }
    };
    for (int plane = 0; plane < 6; ++plane)
    {
        const float da = distance(a, plane), db = distance(b, plane);
        if (da < 0 && db < 0) return;
        if ((da < 0) != (db < 0))
        {
            const Vector4 intersection = a + (b - a) * (da / (da - db));
            if (da < 0) a = intersection; else b = intersection;
        }
    }
    if (float(a.GetW()) <= 1e-6f || float(b.GetW()) <= 1e-6f) return;
    Vector3 p((float(a.GetX() / a.GetW()) + 1) * .5f * m_Width,
        (1 - float(a.GetY() / a.GetW())) * .5f * m_Height, 0);
    Vector3 q((float(b.GetX() / b.GetW()) + 1) * .5f * m_Width,
        (1 - float(b.GetY() / b.GetW())) * .5f * m_Height, 0);
    const float dx = q.GetX() - p.GetX(), dy = q.GetY() - p.GetY();
    const float length = std::sqrt(dx * dx + dy * dy);
    if (!std::isfinite(length) || length < .01f) return;
    const Vector3 side(-dy * width / (2 * length), dx * width / (2 * length), 0);
    Triangle(p + side, p - side, q + side, color, id);
    Triangle(q + side, p - side, q - side, color, id);
}

void SkeletonOverlay::Point(Vector3 position, Vector3 color, uint32_t id, float radius)
{
    Vector3 center;
    if (!Project(position, center)) return;
    constexpr int segments = 12;
    for (int i = 0; i < segments; ++i)
    {
        const float a = i * XM_2PI / segments, b = (i + 1) * XM_2PI / segments;
        Triangle(center, center + Vector3(std::cos(a) * radius, std::sin(a) * radius, 0),
            center + Vector3(std::cos(b) * radius, std::sin(b) * radius, 0), color, id);
    }
}

bool SkeletonOverlay::RequestPick(float x, float y)
{
    if (m_PickFence || x < 0 || y < 0 || x >= m_Width || y >= m_Height) return false;
    m_PickX = static_cast<uint32_t>(x / m_Width * float(m_Resolution));
    m_PickY = static_cast<uint32_t>(y / m_Height * float(m_Resolution));
    if (m_PickX >= m_Resolution) m_PickX = m_Resolution - 1;
    if (m_PickY >= m_Resolution) m_PickY = m_Resolution - 1;
    m_PickRequested = m_AcceptPick = true;
    return true;
}

bool SkeletonOverlay::PollPick(uint32_t& id)
{
    if (!m_PickFence || !g_CommandManager.IsFenceComplete(m_PickFence)) return false;
    id = *static_cast<const uint32_t*>(m_Readback.Map());
    m_Readback.Unmap();
    m_PickFence = 0;
    const bool accepted = m_AcceptPick;
    m_AcceptPick = false;
    return accepted;
}

void SkeletonOverlay::Render(GraphicsContext& context, ColorBuffer& target)
{
    context.TransitionResource(target, D3D12_RESOURCE_STATE_RENDER_TARGET);
    context.TransitionResource(m_Ids, D3D12_RESOURCE_STATE_RENDER_TARGET, true);
    context.ClearColor(m_Ids);
    const D3D12_CPU_DESCRIPTOR_HANDLE rtvs[] = {target.GetRTV(), m_Ids.GetRTV()};
    context.SetViewportAndScissor(0, 0, m_Resolution, m_Resolution);
    if (!m_SurfaceVertices.empty())
    {
        // A separate depth buffer keeps bodies visible through the character,
        // while the nearest collider still wins the ID at each pixel.
        context.TransitionResource(m_SurfaceDepth, D3D12_RESOURCE_STATE_DEPTH_WRITE, true);
        context.ClearDepth(m_SurfaceDepth);
        context.SetRenderTargets(2, rtvs, m_SurfaceDepth.GetDSV());
        context.SetRootSignature(m_SurfaceRoot);
        context.SetPipelineState(m_SurfacePSO);
        struct Constants { Matrix4 viewProjection; Vector4 eye; } constants{m_ViewProjection, Vector4(m_Eye, 1)};
        context.SetDynamicConstantBufferView(0, sizeof(constants), &constants);
        context.SetPrimitiveTopology(D3D_PRIMITIVE_TOPOLOGY_TRIANGLELIST);
        context.SetDynamicVB(0, m_SurfaceVertices.size(), sizeof(SurfaceVertex), m_SurfaceVertices.data());
        context.Draw(static_cast<uint32_t>(m_SurfaceVertices.size()));
    }
    context.SetRenderTargets(2, rtvs);
    if (!m_FillVertices.empty() || !m_Vertices.empty())
    {
        context.SetRootSignature(m_DrawRoot);
        context.SetPipelineState(m_DrawPSO);
        context.SetPrimitiveTopology(D3D_PRIMITIVE_TOPOLOGY_TRIANGLELIST);
        for (const auto* vertices : {&m_FillVertices, &m_Vertices})
        {
            if (vertices->empty()) continue;
            context.SetDynamicVB(0, vertices->size(), sizeof(Vertex), vertices->data());
            context.Draw(static_cast<uint32_t>(vertices->size()));
        }
    }
    context.TransitionResource(target, D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE);
    if (m_PickRequested)
    {
        auto& compute = context.GetComputeContext();
        compute.TransitionResource(m_Ids, D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
        compute.TransitionResource(m_PickResult, D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
        compute.SetRootSignature(m_PickRoot);
        compute.SetPipelineState(m_PickPSO);
        compute.SetConstants(0, m_PickX, m_PickY);
        compute.SetDynamicDescriptor(1, 0, m_Ids.GetSRV());
        compute.SetBufferUAV(2, m_PickResult);
        compute.Dispatch(1, 1, 1);
        context.TransitionResource(m_PickResult, D3D12_RESOURCE_STATE_COPY_SOURCE);
        context.CopyBufferRegion(m_Readback, 0, m_PickResult, 0, sizeof(uint32_t));
        m_PickFence = context.Flush(false);
        m_PickRequested = false;
    }
}
