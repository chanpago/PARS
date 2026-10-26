#include "FootSoleDebug.h"
#include "Components/SkeletalMeshComponent.h"
#include "FootSupportHull.h"
#include "PhysicsPose.h"
#include "imgui.h"
#include <cmath>
#include <cstring>
#include <utility>

using namespace Math;
using namespace physx;

namespace
{
    const char* const FootNames[] = {"LeftFoot", "RightFoot"};
    const char* const ToeNames[] = {"LeftToeBase", "RightToeBase"};
    const char* const FootLabels[] = {u8"왼발", u8"오른발"};
    constexpr PxU32 FootColors[] = {0xff44ccff, 0xffffbb44};
    constexpr PxU32 HitColor = 0xff55ff77, MissColor = 0xffcc5566;
    constexpr PxU32 HullColor = 0xffee66ff;
    constexpr PxU32 ComColor = 0xffffdd44, OutsideColor = 0xffff0000, UnknownColor = 0xff000000;

    bool Matches(const std::string& name, const char* leaf)
    {
        const auto separator = name.find_last_of(':');
        return name.compare(separator == std::string::npos ? 0 : separator + 1,
            std::string::npos, leaf) == 0;
    }

    bool SameGeometry(const PARS::PhysicsBody& a, const PARS::PhysicsBody& b)
    {
        return a.bone == b.bone && a.shape == b.shape && a.radius == b.radius && a.halfLength == b.halfLength &&
            std::memcmp(&a.offset, &b.offset, sizeof(a.offset)) == 0 &&
            std::memcmp(&a.rotation, &b.rotation, sizeof(a.rotation)) == 0 &&
            std::memcmp(&a.halfExtents, &b.halfExtents, sizeof(a.halfExtents)) == 0;
    }

    float Extent(const PARS::PhysicsBody& body, const Matrix4& world, Vector3 direction)
    {
        const float x = std::abs(float(Dot(Vector3(world.GetX()), direction)));
        if (body.shape == PARS::PhysicsShapeType::Capsule)
            return body.radius + body.halfLength * x;
        if (body.shape == PARS::PhysicsShapeType::Sphere)
            return body.radius;
        return body.halfExtents.x * x +
            body.halfExtents.y * std::abs(float(Dot(Vector3(world.GetY()), direction))) +
            body.halfExtents.z * std::abs(float(Dot(Vector3(world.GetZ()), direction)));
    }

    PxVec3 ToPx(Vector3 point)
    {
        return PxVec3(float(point.GetX()), float(point.GetY()), float(point.GetZ()));
    }

    // 표시만 살짝 올려 바닥과 선의 깊이가 겹치는 것을 피한다.
    void Line(std::vector<PxDebugLine>& lines, Vector3 a, Vector3 b, PxU32 color)
    {
        const Vector3 offset(0, .002f, 0);
        lines.emplace_back(ToPx(a + offset), ToPx(b + offset), color);
    }

    void Point(std::vector<PxDebugLine>& lines, Vector3 point, PxU32 color, float radius = .008f)
    {
        Line(lines, point - Vector3(radius, 0, 0), point + Vector3(radius, 0, 0), color);
        Line(lines, point - Vector3(0, 0, radius), point + Vector3(0, 0, radius), color);
    }

    void ProjectionMarker(std::vector<PxDebugLine>& lines, Vector3 point, PxU32 color, bool bold)
    {
        constexpr float radius = .025f;
        // GPU 라인은 1픽셀이므로 촘촘히 겹쳐 원과 십자에 12mm 폭을 준다.
        const int strokes = bold ? 13 : 1;
        const int segments = bold ? 48 : 24;
        for (int stroke = 0; stroke < strokes; ++stroke)
        {
            const float offset = bold ? (stroke - 6) * .001f : 0;
            Line(lines, point + Vector3(-radius, 0, offset), point + Vector3(radius, 0, offset), color);
            Line(lines, point + Vector3(offset, 0, -radius), point + Vector3(offset, 0, radius), color);
            const float ringRadius = radius + offset;
            for (int segment = 0; segment < segments; ++segment)
            {
                const float a = XM_2PI * segment / segments, b = XM_2PI * (segment + 1) / segments;
                Line(lines, point + Vector3(std::cos(a) * ringRadius, 0, std::sin(a) * ringRadius),
                    point + Vector3(std::cos(b) * ringRadius, 0, std::sin(b) * ringRadius), color);
            }
        }
    }
}

bool FootSoleDebug::Prepare(PARS::SkeletalMeshComponent& mesh)
{
    const auto* asset = mesh.GetPhysicsAsset();
    const auto* model = mesh.GetSkeletalMesh().get();
    const auto& ragdoll = mesh.GetRagdoll();
    if (!asset || !model || !ragdoll.IsActive())
    {
        m_Message = u8"발바닥을 표시할 물리 바디가 없습니다.";
        return false;
    }

    std::array<int, 2> indices{-1, -1};
    for (size_t side = 0; side < m_Feet.size(); ++side)
    {
        for (size_t body = 0; body < asset->bodies.size(); ++body)
            if (Matches(asset->bodies[body].bone, FootNames[side]))
                indices[side] = static_cast<int>(body);
    }
    bool unchanged = m_Initialized && m_Model == model;
    for (size_t side = 0; side < m_Feet.size(); ++side)
    {
        const int index = indices[side];
        unchanged = unchanged && index == m_Feet[side].bodyIndex &&
            (index < 0 || SameGeometry(asset->bodies[index], m_Feet[side].source));
    }
    if (unchanged)
        return true;

    PARS::PhysicsSkeleton skeleton;
    if (!mesh.BuildPhysicsSkeleton(skeleton, m_Message))
        return false;
    const std::vector<GraphNode> reference(model->m_SceneGraph.get(), model->m_SceneGraph.get() + model->m_NumNodes);
    std::vector<Matrix4> referenceWorld;
    if (!PARS::EvaluatePhysicsPose(skeleton, reference, Matrix4(kIdentity), referenceWorld, m_Message))
        return false;

    m_Feet = {};
    const Vector3 up(kYUnitVector);
    bool found = false;
    for (size_t side = 0; side < m_Feet.size(); ++side)
    {
        const int index = indices[side];
        if (index < 0 || size_t(index) >= ragdoll.GetBodyCount() || size_t(index) >= ragdoll.GetBoneNodes().size())
            continue;
        const int node = ragdoll.GetBoneNodes()[index];
        if (node < 0 || size_t(node) >= referenceWorld.size())
            continue;

        auto& sole = m_Feet[side];
        sole.bodyIndex = index;
        sole.source = asset->bodies[index];
        const Matrix4 body = PARS::PhysicsBoneFrame(referenceWorld[node]) * sole.source.GetLocalTransform();
        Vector3 forward(body.GetX());
        for (size_t toe = 0; toe < skeleton.names.size(); ++toe)
            if (Matches(skeleton.names[toe], ToeNames[side]))
                forward = Vector3(referenceWorld[toe].GetW()) - Vector3(referenceWorld[node].GetW());
        forward.SetY(0);
        forward = float(LengthSquare(forward)) > 1e-8f ? Normalize(forward) : Vector3(kZUnitVector);
        const Vector3 right = Normalize(Cross(up, forward));

        // 기준 자세에서 발바닥 방향과 높이를 정한다. 이후에는 실제 발의 기울기까지 따라간다.
        const Vector3 center = Vector3(body.GetW()) - up * Extent(sole.source, body, up);
        sole.soleInBody = Invert(body) * Matrix4(right, up, forward, center);
        sole.halfWidth = Max(.005f, Extent(sole.source, body, right));
        sole.halfLength = Max(.005f, Extent(sole.source, body, forward));
        found = true;
    }
    m_Model = model;
    m_Initialized = found;
    m_Message = found ? "" : u8"LeftFoot / RightFoot 물리 바디를 찾을 수 없습니다.";
    return found;
}

void FootSoleDebug::BuildLines(PARS::SkeletalMeshComponent& mesh, const PhysicsWorld& physics,
    std::vector<PxDebugLine>& lines)
{
    m_SupportHull.clear();
    m_ProjectedPointCount = 0;
    m_HasCenterOfMass = m_HasComProjection = m_ComInsideHull = false;
    m_TotalMass = 0;
    if (!m_Visible || !Prepare(mesh))
        return;
    std::vector<PxVec3> supportPoints;
    supportPoints.reserve(8);
    const auto& ragdoll = mesh.GetRagdoll();
    double supportHeight = 0;
    const PxVec3 down(0, -RayLength, 0);
    for (size_t side = 0; side < m_Feet.size(); ++side)
    {
        auto& sole = m_Feet[side];
        if (sole.bodyIndex < 0)
            continue;
        const Matrix4 bodyWorld = ragdoll.GetBoneWorld(sole.bodyIndex) * sole.source.GetLocalTransform();
        const Quaternion rotation = Quaternion(Vector3(kZUnitVector), XMConvertToRadians(sole.rotation.z)) *
            Quaternion(Vector3(kYUnitVector), XMConvertToRadians(sole.rotation.y)) *
            Quaternion(Vector3(kXUnitVector), XMConvertToRadians(sole.rotation.x));
        const Matrix4 world = bodyWorld * sole.soleInBody * Matrix4(OrthogonalTransform(rotation, Vector3(sole.offset)));
        const float w = sole.halfWidth, l = sole.halfLength;
        const Vector3 local[] = {Vector3(-w, 0, -l), Vector3(w, 0, -l), Vector3(w, 0, l), Vector3(-w, 0, l)};
        std::array<Vector3, 4> corners, projected;
        for (size_t i = 0; i < corners.size(); ++i)
        {
            corners[i] = Vector3(world * local[i]);
            PxRaycastHit hit;
            sole.projected[i] = physics.Raycast(ToPx(corners[i]), down, hit, &mesh.GetOwner());
            sole.distances[i] = sole.projected[i] ? hit.distance : RayLength;
            projected[i] = sole.projected[i] ? Vector3(hit.position.x, hit.position.y, hit.position.z)
                                           : corners[i] + Vector3(0, -RayLength, 0);
            if (m_ShowRays)
                Line(lines, corners[i], projected[i], sole.projected[i] ? HitColor : MissColor);
            Point(lines, corners[i], FootColors[side]);
            if (sole.projected[i])
            {
                Point(lines, projected[i], HitColor);
                supportPoints.push_back(hit.position);
                supportHeight += hit.position.y;
            }
        }
        for (size_t i = 0; i < corners.size(); ++i)
        {
            const size_t next = (i + 1) % corners.size();
            Line(lines, corners[i], corners[next], FootColors[side]);
            if (sole.projected[i] && sole.projected[next])
                Line(lines, projected[i], projected[next], HitColor);
        }
    }
    // 양발의 유효한 지면 투영점을 함께 감싸는 2D 지지 영역이다.
    m_ProjectedPointCount = supportPoints.size();
    m_SupportHull = PARS::BuildFootSupportHull(std::move(supportPoints));
    Vector3 center(kZero);
    m_HasCenterOfMass = ragdoll.GetCenterOfMass(center, m_TotalMass);
    if (m_HasCenterOfMass)
    {
        m_CenterOfMass = ToPx(center);
        if (m_ProjectedPointCount > 0)
        {
            // 현재 수평 바닥 기준: 유효한 발 투영점의 평균 높이에 수직으로 내린다.
            m_ProjectedCenterOfMass = PxVec3(m_CenterOfMass.x,
                float(supportHeight / m_ProjectedPointCount), m_CenterOfMass.z);
            m_HasComProjection = true;
            m_ComInsideHull = PARS::IsInsideFootSupportHull(m_SupportHull, m_ProjectedCenterOfMass);
        }
    }
    if (m_ShowHull)
    {
        for (size_t i = 0; i < m_SupportHull.size(); ++i)
        {
            const auto& a = m_SupportHull[i];
            const auto& b = m_SupportHull[(i + 1) % m_SupportHull.size()];
            const Vector3 point(a.x, a.y, a.z);
            Line(lines, point, Vector3(b.x, b.y, b.z), HullColor);
            Point(lines, point, HullColor);
        }
    }
    if (m_ShowCenterOfMass && m_HasCenterOfMass)
    {
        Point(lines, center, ComColor, .025f);
        Line(lines, center - Vector3(0, .025f, 0), center + Vector3(0, .025f, 0), ComColor);
        if (m_HasComProjection)
        {
            const Vector3 projected(m_ProjectedCenterOfMass.x, m_ProjectedCenterOfMass.y, m_ProjectedCenterOfMass.z);
            const PxU32 color = m_SupportHull.empty() ? UnknownColor : m_ComInsideHull ? HitColor : OutsideColor;
            Line(lines, center, projected, ComColor);
            ProjectionMarker(lines, projected, color, m_SupportHull.empty() || !m_ComInsideHull);
        }
    }
}

void FootSoleDebug::DrawUI(PARS::SkeletalMeshComponent& mesh)
{
    if (!ImGui::CollapsingHeader(u8"발바닥 / 지면 투영", ImGuiTreeNodeFlags_DefaultOpen))
        return;
    ImGui::Checkbox(u8"발바닥 영역 표시", &m_Visible);
    ImGui::SameLine();
    ImGui::Checkbox(u8"레이 표시", &m_ShowRays);
    ImGui::Checkbox(u8"지지 영역 표시 (Convex Hull)", &m_ShowHull);
    ImGui::Checkbox(u8"질량중심 / 수직 투영 표시", &m_ShowCenterOfMass);
    ImGui::TextUnformatted(u8"아래 방향 20cm 이내 히트만 투영 · 초록: 성공 / 빨강: 범위 내 바닥 없음");
    ImGui::Text(u8"보라: 지지 영역 · 유효 투영점 %zu개 / 헐 꼭짓점 %zu개",
        m_ProjectedPointCount, m_SupportHull.size());
    if (m_Visible && m_SupportHull.empty())
        ImGui::TextUnformatted(u8"지지 영역 없음: 유효한 점이 부족하거나 일직선입니다.");
    if (m_HasCenterOfMass)
    {
        ImGui::Text(u8"전체 질량 %.2f kg / COM (%.3f, %.3f, %.3f) m", m_TotalMass,
            m_CenterOfMass.x, m_CenterOfMass.y, m_CenterOfMass.z);
        ImGui::TextUnformatted(u8"노랑: COM과 수직선 / 투영 원: 초록=영역 안, 굵은 빨강=영역 밖, 굵은 검정=판정 불가");
        ImGui::Text(u8"COM 판정: %s", !m_HasComProjection ? u8"지면 투영점 없음" :
            m_SupportHull.empty() ? u8"지지 영역 없음" : m_ComInsideHull ? u8"영역 안 (경계 포함)" : u8"영역 밖");
    }
    if (!Prepare(mesh))
    {
        ImGui::TextWrapped("%s", m_Message.c_str());
        return;
    }
    if (ImGui::Button(u8"발바닥 기본 영역으로 초기화"))
    {
        m_Initialized = false;
        if (!Prepare(mesh))
            return;
    }
    for (size_t side = 0; side < m_Feet.size(); ++side)
    {
        auto& sole = m_Feet[side];
        ImGui::PushID(static_cast<int>(side));
        if (sole.bodyIndex >= 0 && ImGui::TreeNodeEx(FootLabels[side], ImGuiTreeNodeFlags_DefaultOpen))
        {
            float dimensions[] = {sole.halfWidth * 200, sole.halfLength * 200};
            if (ImGui::DragFloat2(u8"폭 / 길이", dimensions, .1f, 1, 100, "%.1f cm"))
            {
                sole.halfWidth = dimensions[0] * .005f;
                sole.halfLength = dimensions[1] * .005f;
            }
            ImGui::DragFloat3(u8"로컬 위치", &sole.offset.x, .001f, -.5f, .5f, "%.3f m");
            ImGui::DragFloat3(u8"로컬 회전", &sole.rotation.x, .5f, -180, 180, "%.1f deg");
            unsigned count = 0;
            for (bool hit : sole.projected)
                count += hit ? 1 : 0;
            ImGui::Text(u8"투영점 %u / 4", count);
            for (int i = 0; i < 4; ++i)
            {
                if (sole.projected[i])
                    ImGui::Text(u8"점 %d: %.1f cm", i + 1, sole.distances[i] * 100);
                else
                    ImGui::Text(u8"점 %d: 투영 없음", i + 1);
            }
            ImGui::TreePop();
        }
        ImGui::PopID();
    }
}
