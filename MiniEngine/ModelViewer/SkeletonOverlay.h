#pragma once

#include "GraphicsCore.h"
#include "Camera.h"
#include "ColorBuffer.h"
#include "ReadbackBuffer.h"
#include "PipelineState.h"
#include "RootSignature.h"
#include "DepthBuffer.h"
#include "PhysicsAsset.h"
#include <unordered_map>
#include <vector>

// 화면상 크기를 유지하는 본/기즈모와 반투명 월드 공간 충돌 표면이다.
// 표시 색상과 정확한 24비트 ID를 함께 래스터화한다. 색상에만 블렌딩을 적용하므로
// 선택 처리에는 CPU 삼각형/광선 검사를 사용하지 않는다.
class SkeletonOverlay
{
public:
    void Initialize(uint32_t resolution);
    void Cleanup(); // GPU가 유휴 상태여야 한다.
    void Begin(const Math::Camera& camera, float width, float height);
    bool Project(Math::Vector3 position, Math::Vector3& pixel) const;
    void Line(Math::Vector3 start, Math::Vector3 end, Math::Vector3 color, uint32_t id, float width = 3);
    void Point(Math::Vector3 position, Math::Vector3 color, uint32_t id, float radius = 4);
    void FillTriangle(Math::Vector3 a, Math::Vector3 b, Math::Vector3 c, Math::Vector3 color, uint32_t id, float opacity);
    void Body(const PARS::PhysicsBody& body, const Math::Matrix4& world, Math::Vector3 color, uint32_t id, float opacity);
    // 조인트의 YZ 평면에서 0도 기준이 되는 단위 벡터다. 각도 표시만 변경한다.
    void ConstraintLimits(const PARS::PhysicsConstraint& joint, const Math::Matrix4& parent,
        const Math::Matrix4& child, uint32_t id, bool selected,
        Math::Vector3 twistReference = Math::Vector3(Math::kYUnitVector), bool articulation = false);
    void Render(GraphicsContext& context, ColorBuffer& target);
    bool RequestPick(float x, float y);
    bool PollPick(uint32_t& id);
    void CancelPick()
    {
        m_AcceptPick = false;
        m_PickRequested = false;
    }

private:
    struct Vertex
    {
        float x, y, z, w;
        float r, g, b, a;
        uint32_t id;
    };
    // 공유하는 단위 기본 형상이다. 캡슐 양 끝의 반구는 반지름과 독립적으로 X축을 따라 이동한다.
    struct SurfaceVertex
    {
        DirectX::XMFLOAT3 position, normal;
        float capOffset;
    };
    struct BodyDraw
    {
        Math::Matrix4 world;
        DirectX::XMFLOAT4 dimensions, color;
        uint32_t id, padding[3]{}; // 버텍스 셰이더의 b1 레이아웃과 일치한다.
        uint32_t firstVertex, vertexCount;
    };
    struct LimitLine
    {
        Math::Vector3 a, b, color;
        float width;
    };
    struct LimitTriangle
    {
        Math::Vector3 a, b, c, color;
        float opacity;
    };
    struct ConstraintGeometry
    {
        PARS::PhysicsConstraint settings;
        Math::Vector3 twistReference{Math::kYUnitVector};
        bool selected = false, valid = false, projected = false;
        bool articulation = false;
        std::vector<LimitLine> lines;
        std::vector<LimitTriangle> triangles;
        Math::Matrix4 clipTransform{Math::kIdentity};
        float width = 0, height = 0;
        std::vector<Vertex> projectedLines, projectedFills;
    };
    void InitializeSurfaces();
    static std::vector<SurfaceVertex> BuildSurfaceMesh();
    static void BuildConstraintGeometry(const PARS::PhysicsConstraint& joint, bool selected,
        Math::Vector3 twistReference, ConstraintGeometry& geometry);
    void ClipLine(Math::Vector4 a, Math::Vector4 b, Math::Vector3 color, uint32_t id, float width);
    void Triangle(Math::Vector3 a, Math::Vector3 b, Math::Vector3 c, Math::Vector3 color, uint32_t id);
    Math::Matrix4 m_ViewProjection{Math::kIdentity};
    float m_Width = 1, m_Height = 1;
    uint32_t m_Resolution = 0, m_PickX = 0, m_PickY = 0;
    std::vector<Vertex> m_Vertices;
    std::vector<Vertex> m_FillVertices; // 반투명 제한 영역은 모든 불투명 선/기즈모보다 먼저 그린다.
    std::vector<BodyDraw> m_BodyDraws;
    ByteAddressBuffer m_SurfaceMesh;
    std::unordered_map<uint32_t, ConstraintGeometry> m_ConstraintGeometry;
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
