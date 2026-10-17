#include "SkeletonOverlay.h"
#include "GraphicsCommon.h"
#include "CompiledShaders/PhysicsAssetVS.h"
#include "CompiledShaders/PhysicsAssetPS.h"
#include <cmath>
#include <cstring>

using namespace Math;
using namespace Graphics;

namespace
{
    constexpr uint32_t BoxVertices = 36;
    constexpr int CapsuleSides = 32, CapsuleHemisphere = 10, CapsuleRings = CapsuleHemisphere * 2 + 1;
    constexpr uint32_t CapsuleVertices = CapsuleSides * CapsuleRings * 6;
}

void SkeletonOverlay::ConstraintLimits(const PARS::PhysicsConstraint& joint, const Matrix4& parent,
    const Matrix4& child, uint32_t id, bool selected)
{
    auto& geometry = m_ConstraintGeometry[id];
    const auto& previous = geometry.settings;
    if (!geometry.valid || geometry.selected != selected || previous.twist != joint.twist ||
        previous.swingY != joint.swingY || previous.swingZ != joint.swingZ ||
        previous.twistMin != joint.twistMin || previous.twistMax != joint.twistMax ||
        previous.swingYLimit != joint.swingYLimit || previous.swingZLimit != joint.swingZLimit)
        BuildConstraintGeometry(joint, selected, geometry);

    // Cache local geometry across animation/camera movement. When the projected
    // frame is unchanged too, reuse the clipped screen-width lines and fills.
    const Matrix4 clipTransform = m_ViewProjection * parent;
    if (!geometry.projected || geometry.width != m_Width || geometry.height != m_Height ||
        std::memcmp(&geometry.clipTransform, &clipTransform, sizeof(Matrix4)) != 0)
    {
        const size_t lineStart = m_Vertices.size(), fillStart = m_FillVertices.size();
        for (const auto& line : geometry.lines)
            ClipLine(clipTransform * line.a, clipTransform * line.b, line.color, id, line.width);
        for (const auto& triangle : geometry.triangles)
            for (Vector3 point : {triangle.a, triangle.b, triangle.c})
            {
                const Vector4 clip = clipTransform * point;
                m_FillVertices.push_back({clip.GetX(), clip.GetY(), clip.GetZ(), clip.GetW(),
                    triangle.color.GetX(), triangle.color.GetY(), triangle.color.GetZ(), triangle.opacity, id});
            }
        geometry.projectedLines.assign(m_Vertices.begin() + lineStart, m_Vertices.end());
        geometry.projectedFills.assign(m_FillVertices.begin() + fillStart, m_FillVertices.end());
        geometry.clipTransform = clipTransform;
        geometry.width = m_Width;
        geometry.height = m_Height;
        geometry.projected = true;
    }
    else
    {
        m_Vertices.insert(m_Vertices.end(), geometry.projectedLines.begin(), geometry.projectedLines.end());
        m_FillVertices.insert(m_FillVertices.end(), geometry.projectedFills.begin(), geometry.projectedFills.end());
    }

    // Current-angle pointers follow the child every frame, even when the parent
    // frame and its cached limit geometry stay still.
    const Vector3 origin(kZero), x(kXUnitVector), y(kYUnitVector), z(kZUnitVector), yellow(1, .85f, .05f);
    const float width = selected ? 2.5f : 1.25f;
    const auto pointer = [&](Vector3 direction, float length) {
        const Vector3 tip = direction * length;
        const Vector3 side = Normalize(Cross(direction, std::abs(float(direction.GetY())) < .9f ? y : z));
        const auto line = [&](Vector3 a, Vector3 b, float thickness) {
            ClipLine(clipTransform * a, clipTransform * b, yellow, id, thickness);
        };
        line(origin, tip, selected ? 3.5f : 2.5f);
        line(tip, tip - direction * (length * .18f) + side * (length * .08f), width);
        line(tip, tip - direction * (length * .18f) - side * (length * .08f), width);
    };
    const Quaternion relative = Normalize(Quaternion(Invert(parent) * child));
    pointer(relative * x, .1f);
    XMFLOAT4 q;
    XMStoreFloat4(&q, relative);
    const float twistNorm = q.x * q.x + q.w * q.w;
    // q = swing * twist. At a 180-degree swing the twist is undefined.
    if (twistNorm > 1e-10f)
    {
        const float invLength = 1 / std::sqrt(twistNorm);
        const Quaternion twist(XMVectorSet(q.x * invLength, 0, 0, q.w * invLength));
        pointer(twist * y, .062f);
    }
}

void SkeletonOverlay::BuildConstraintGeometry(const PARS::PhysicsConstraint& joint, bool selected, ConstraintGeometry& geometry)
{
    geometry.settings = joint;
    geometry.selected = selected;
    geometry.valid = true;
    geometry.projected = false;
    geometry.lines.clear();
    geometry.triangles.clear();
    using Motion = PARS::PhysicsJointMotion;
    constexpr float radius = .1f;
    constexpr int segments = 64;
    const Vector3 origin(kZero), x(kXUnitVector), y(kYUnitVector), z(kZUnitVector);
    const Vector3 red(1, .22f, .24f), green(.2f, .95f, .35f);
    const float width = selected ? 2.5f : 1.25f, opacity = selected ? .32f : .16f;
    const auto line = [&](Vector3 a, Vector3 b, Vector3 color, float thickness) {
        geometry.lines.push_back({a, b, color, thickness});
    };
    const auto fill = [&](Vector3 a, Vector3 b, Vector3 c, Vector3 color, float alpha) {
        geometry.triangles.push_back({a, b, c, color, alpha});
    };
    const auto sector = [&](Vector3 center, Vector3 axis, Vector3 start, float low, float high, float alpha, Vector3 color) {
        for (int step = 0; step < segments; ++step)
        {
            const Vector3 a = center + Quaternion(axis, low + (high - low) * step / segments) * start;
            const Vector3 b = center + Quaternion(axis, low + (high - low) * (step + 1) / segments) * start;
            line(a, b, color, width);
            if (alpha > 0) fill(center, a, b, color, alpha);
        }
        // A free axis is a full ring, with no misleading minimum/maximum spoke.
        if (high - low < XM_2PI - .001f)
        {
            line(center, center + Quaternion(axis, low) * start, color, width);
            line(center, center + Quaternion(axis, high) * start, color, width);
        }
    };

    if (joint.swingY == Motion::Limited && joint.swingZ == Motion::Limited)
    {
        // This SDK uses ConeLimitHelperTanLess: the ellipse is in swing-angle
        // coordinates, then converted to tan(angle/4), NOT the other way round.
        // The boundary is a pure swing quaternion applied to local X.
        const auto cone = [&](float angle) {
            const float sy = std::tan(joint.swingYLimit * std::cos(angle) * .25f);
            const float sz = std::tan(joint.swingZLimit * std::sin(angle) * .25f);
            const float norm = 1 + sy * sy + sz * sz;
            return Quaternion(XMVectorSet(0, 2 * sy / norm, 2 * sz / norm, (2 - norm) / norm)) * (x * radius);
        };
        for (int step = 0; step < segments; ++step)
        {
            const Vector3 a = cone(step * XM_2PI / segments), b = cone((step + 1) * XM_2PI / segments);
            fill(origin, a, b, red, opacity);
            line(a, b, red, width);
            if (step % (segments / 4) == 0) line(origin, a, red, width);
        }
    }
    else if (joint.swingY == Motion::Locked && joint.swingZ == Motion::Locked)
    {
        // Zero angular area: a cross marks the sole allowed direction.
        line(origin, x * radius, red, width);
        line(x * radius - y * .008f, x * radius + y * .008f, red, width);
        line(x * radius - z * .008f, x * radius + z * .008f, red, width);
    }
    else if (joint.swingY == Motion::Locked || joint.swingZ == Motion::Locked)
    {
        const bool aboutY = joint.swingZ == Motion::Locked;
        const Motion motion = aboutY ? joint.swingY : joint.swingZ;
        const float limit = motion == Motion::Free ? XM_PI : aboutY ? joint.swingYLimit : joint.swingZLimit;
        sector(origin, aboutY ? y : z, x * radius, -limit, limit, motion == Motion::Free ? opacity * .35f : opacity, red);
    }
    else if (joint.swingY == Motion::Free && joint.swingZ == Motion::Free)
    {
        // No swing restriction: an unfilled sphere, not a finite cone.
        sector(origin, x, y * radius, -XM_PI, XM_PI, 0, red);
        sector(origin, y, x * radius, -XM_PI, XM_PI, 0, red);
        sector(origin, z, x * radius, -XM_PI, XM_PI, 0, red);
    }
    else
    {
        // One swing free, the other limited: D6 constrains the elevation of
        // child X (asin), leaving azimuth free. Show the allowed spherical band.
        const bool limitedY = joint.swingY == Motion::Limited;
        const float limit = Min(limitedY ? joint.swingYLimit : joint.swingZLimit, XM_PIDIV2);
        const auto band = [&](float elevation, float azimuth) {
            const float a = radius * std::cos(elevation) * std::cos(azimuth);
            const float b = radius * std::cos(elevation) * std::sin(azimuth);
            const float c = radius * std::sin(elevation);
            return limitedY ? Vector3(a, b, c) : Vector3(a, c, b);
        };
        constexpr int rows = 12;
        for (int step = 0; step < segments; ++step)
        {
            const float a = step * XM_2PI / segments, b = (step + 1) * XM_2PI / segments;
            for (int row = 0; row < rows; ++row)
            {
                const float low = -limit + 2 * limit * row / rows, high = -limit + 2 * limit * (row + 1) / rows;
                const Vector3 p = band(low, a), q = band(low, b), r = band(high, a), s = band(high, b);
                fill(p, q, r, red, opacity * .5f);
                fill(q, s, r, red, opacity * .5f);
                if (step % (segments / 4) == 0) line(p, r, red, width);
            }
            line(band(-limit, a), band(-limit, b), red, width);
            line(band(limit, a), band(limit, b), red, width);
        }
    }

    // Swing and twist share the joint pivot, including their current-angle pointers.
    // Its zero is parent Y, positive angles rotate toward parent Z about parent X.
    const Vector3 twistCenter = origin;
    const float twistRadius = radius * .62f;
    const float low = joint.twist == Motion::Free ? -XM_PI : joint.twist == Motion::Locked ? 0 : joint.twistMin;
    const float high = joint.twist == Motion::Free ? XM_PI : joint.twist == Motion::Locked ? 0 : joint.twistMax;
    if (joint.twist == Motion::Locked)
    {
        const Vector3 tip = twistCenter + y * twistRadius;
        line(twistCenter, tip, green, width);
        line(tip - z * .008f, tip + z * .008f, green, width);
    }
    else sector(twistCenter, x, y * twistRadius, low, high, joint.twist == Motion::Free ? 0 : opacity, green);

}

void SkeletonOverlay::InitializeSurfaces()
{
    m_SurfaceDepth.Create(L"PhysicsAsset picking depth", m_Resolution, m_Resolution, DXGI_FORMAT_D32_FLOAT);
    const auto vertices = BuildSurfaceMesh();
    m_SurfaceMesh.Create(L"PhysicsAsset shared primitives", static_cast<uint32_t>(vertices.size()), sizeof(SurfaceVertex), vertices.data());
    m_SurfaceRoot.Reset(2);
    m_SurfaceRoot[0].InitAsConstantBuffer(0);
    m_SurfaceRoot[1].InitAsConstantBuffer(1, D3D12_SHADER_VISIBILITY_VERTEX);
    m_SurfaceRoot.Finalize(L"PhysicsAsset surfaces", D3D12_ROOT_SIGNATURE_FLAG_ALLOW_INPUT_ASSEMBLER_INPUT_LAYOUT);
    const D3D12_INPUT_ELEMENT_DESC layout[] = {
        {"POSITION", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, 0, D3D12_INPUT_CLASSIFICATION_PER_VERTEX_DATA, 0},
        {"NORMAL", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, 12, D3D12_INPUT_CLASSIFICATION_PER_VERTEX_DATA, 0},
        {"TEXCOORD", 0, DXGI_FORMAT_R32_FLOAT, 0, 24, D3D12_INPUT_CLASSIFICATION_PER_VERTEX_DATA, 0}};
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
    const bool box = body.shape == PARS::PhysicsShapeType::Box;
    const XMFLOAT4 dimensions = box ? XMFLOAT4(body.halfExtents.x, body.halfExtents.y, body.halfExtents.z, 0) :
        XMFLOAT4(body.radius, body.radius, body.radius, body.shape == PARS::PhysicsShapeType::Capsule ? body.halfLength : 0);
    m_BodyDraws.push_back({world, dimensions, XMFLOAT4(color.GetX(), color.GetY(), color.GetZ(), opacity),
        id, {}, box ? 0u : BoxVertices, box ? BoxVertices : CapsuleVertices});
}

std::vector<SkeletonOverlay::SurfaceVertex> SkeletonOverlay::BuildSurfaceMesh()
{
    // Build once, preserving the original tessellation and winding. Sizes and
    // rigid transforms are applied by the vertex shader, including live PA edits.
    std::vector<SurfaceVertex> vertices;
    vertices.reserve(BoxVertices + CapsuleVertices);
    const auto vertex = [&](Vector3 p, Vector3 n, float capOffset) {
        SurfaceVertex v;
        XMStoreFloat3(&v.position, p);
        XMStoreFloat3(&v.normal, n);
        v.capOffset = capOffset;
        vertices.push_back(v);
    };
    {
        const Vector3 normals[] = {Vector3(1,0,0), Vector3(-1,0,0), Vector3(0,1,0), Vector3(0,-1,0), Vector3(0,0,1), Vector3(0,0,-1)};
        const Vector3 tangents[] = {Vector3(0,1,0), Vector3(0,1,0), Vector3(0,0,1), Vector3(0,0,1), Vector3(1,0,0), Vector3(1,0,0)};
        const float corners[6][2] = {{-1,-1},{1,-1},{1,1},{-1,-1},{1,1},{-1,1}};
        for (int face = 0; face < 6; ++face)
            for (const auto& corner : corners)
                vertex(normals[face] + tangents[face] * corner[0] + Cross(normals[face], tangents[face]) * corner[1], normals[face], 0);
    }
    // Two hemispheres and an explicit cylinder band. Duplicated equators keep
    // the capsule's cylindrical section straight, including halfLength == 0.
    const auto sample = [&](int ring, int side, Vector3& n, float& capOffset) {
        const bool positive = ring > CapsuleHemisphere;
        const int latitude = positive ? ring - 1 : ring;
        const float angle = -XM_PIDIV2 + latitude * XM_PI / (CapsuleHemisphere * 2);
        const float longitude = side * XM_2PI / CapsuleSides;
        n = Vector3(std::sin(angle), std::cos(angle) * std::cos(longitude), std::cos(angle) * std::sin(longitude));
        capOffset = positive ? 1.0f : -1.0f;
    };
    for (int ring = 0; ring < CapsuleRings; ++ring)
        for (int side = 0; side < CapsuleSides; ++side)
        {
            Vector3 n[4];
            float offsets[4];
            sample(ring, side, n[0], offsets[0]); sample(ring, side + 1, n[1], offsets[1]);
            sample(ring + 1, side, n[2], offsets[2]); sample(ring + 1, side + 1, n[3], offsets[3]);
            for (int index : {0, 1, 2, 1, 3, 2}) vertex(n[index], n[index], offsets[index]);
        }
    return vertices;
}
