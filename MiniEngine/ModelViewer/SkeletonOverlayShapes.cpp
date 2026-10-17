#include "SkeletonOverlay.h"
#include "GraphicsCommon.h"
#include "CompiledShaders/PhysicsAssetVS.h"
#include "CompiledShaders/PhysicsAssetPS.h"
#include <cmath>

using namespace Math;
using namespace Graphics;

void SkeletonOverlay::ConstraintLimits(const PARS::PhysicsConstraint& joint, const Matrix4& parent,
    const Matrix4& child, uint32_t id, bool selected)
{
    using Motion = PARS::PhysicsJointMotion;
    constexpr float radius = .1f;
    constexpr int segments = 64;
    const Vector3 origin(kZero), x(kXUnitVector), y(kYUnitVector), z(kZUnitVector);
    const Vector3 red(1, .22f, .24f), green(.2f, .95f, .35f), yellow(1, .85f, .05f);
    const float width = selected ? 2.5f : 1.25f, opacity = selected ? .32f : .16f;
    const auto line = [&](Vector3 a, Vector3 b, Vector3 color, float thickness) {
        Line(Vector3(parent * a), Vector3(parent * b), color, id, thickness);
    };
    const auto fill = [&](Vector3 a, Vector3 b, Vector3 c, Vector3 color, float alpha) {
        FillTriangle(Vector3(parent * a), Vector3(parent * b), Vector3(parent * c), color, id, alpha);
    };
    const auto sector = [&](Vector3 center, Vector3 axis, Vector3 start, float low, float high, float alpha) {
        for (int step = 0; step < segments; ++step)
        {
            const Vector3 a = center + Quaternion(axis, low + (high - low) * step / segments) * start;
            const Vector3 b = center + Quaternion(axis, low + (high - low) * (step + 1) / segments) * start;
            line(a, b, green, width);
            if (alpha > 0) fill(center, a, b, green, alpha);
        }
        // A free axis is a full ring, with no misleading minimum/maximum spoke.
        if (high - low < XM_2PI - .001f)
        {
            line(center, center + Quaternion(axis, low) * start, green, width);
            line(center, center + Quaternion(axis, high) * start, green, width);
        }
    };
    const auto pointer = [&](Vector3 center, Vector3 direction, float length) {
        const Vector3 tip = center + direction * length;
        const Vector3 side = Normalize(Cross(direction, std::abs(float(direction.GetY())) < .9f ? y : z));
        line(center, tip, yellow, selected ? 3.5f : 2.5f);
        line(tip, tip - direction * (length * .18f) + side * (length * .08f), yellow, width);
        line(tip, tip - direction * (length * .18f) - side * (length * .08f), yellow, width);
    };

    if (joint.swingY == Motion::Limited && joint.swingZ == Motion::Limited)
    {
        // Match this project's PhysX D6 elliptical cone in tan(angle/4) space.
        // The boundary is a pure swing quaternion applied to the local X axis.
        const auto cone = [&](float angle) {
            const float sy = std::tan(joint.swingYLimit * .25f) * std::cos(angle);
            const float sz = std::tan(joint.swingZLimit * .25f) * std::sin(angle);
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
        sector(origin, aboutY ? y : z, x * radius, -limit, limit, motion == Motion::Free ? opacity * .35f : opacity);
    }
    else if (joint.swingY == Motion::Free && joint.swingZ == Motion::Free)
    {
        // No swing restriction: an unfilled sphere, not a finite cone.
        sector(origin, x, y * radius, -XM_PI, XM_PI, 0);
        sector(origin, y, x * radius, -XM_PI, XM_PI, 0);
        sector(origin, z, x * radius, -XM_PI, XM_PI, 0);
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

    // Offset the twist dial behind the pivot to separate it from the swing cone.
    // Its zero is parent Y, positive angles rotate toward parent Z about parent X.
    const Vector3 twistCenter = x * (-radius * .3f);
    const float twistRadius = radius * .62f;
    const float low = joint.twist == Motion::Free ? -XM_PI : joint.twist == Motion::Locked ? 0 : joint.twistMin;
    const float high = joint.twist == Motion::Free ? XM_PI : joint.twist == Motion::Locked ? 0 : joint.twistMax;
    if (joint.twist == Motion::Locked)
    {
        const Vector3 tip = twistCenter + y * twistRadius;
        line(twistCenter, tip, green, width);
        line(tip - z * .008f, tip + z * .008f, green, width);
    }
    else sector(twistCenter, x, y * twistRadius, low, high, joint.twist == Motion::Free ? 0 : opacity);

    // Compare orientations at the parent anchor even if the two anchors drift.
    // The orange anchor link is drawn separately by the viewer.
    const Quaternion relative = Normalize(Quaternion(Invert(parent) * child));
    pointer(origin, relative * x, radius);
    XMFLOAT4 q;
    XMStoreFloat4(&q, relative);
    const float twistNorm = q.x * q.x + q.w * q.w;
    // q = swing * twist. Projecting onto (x,w) removes swing from this dial.
    // At a 180-degree swing the decomposition is singular; omit that needle.
    if (twistNorm > 1e-10f)
    {
        const float invLength = 1 / std::sqrt(twistNorm);
        const Quaternion twist(XMVectorSet(q.x * invLength, 0, 0, q.w * invLength));
        pointer(twistCenter, twist * y, twistRadius);
    }
}

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
