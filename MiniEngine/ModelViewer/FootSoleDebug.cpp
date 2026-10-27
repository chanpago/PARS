#include "FootSoleDebug.h"
#include "Components/SkeletalMeshComponent.h"
#include "imgui.h"
#include <cmath>

using namespace Math;
using namespace physx;

namespace
{
    constexpr PxU32 FootColors[] = {0xff44ccff, 0xffffbb44};
    constexpr PxU32 HitColor = 0xff55ff77, MissColor = 0xffcc5566;
    constexpr PxU32 HullColor = 0xffee66ff, BufferColor = 0xffffffff;
    constexpr PxU32 ComColor = 0xffffdd44, OutsideColor = 0xffff0000, UnknownColor = 0xff000000;

    Vector3 ToMath(const PxVec3& point) { return Vector3(point.x, point.y, point.z); }

    void DashedLine(std::vector<PxDebugLine>& lines, const PxVec3& a, const PxVec3& b)
    {
        const PxVec3 delta = b - a;
        const float length = delta.magnitude();
        if (length < 1e-6f) return;
        const PxVec3 direction = delta / length, offset(0, .004f, 0);
        for (float distance = 0; distance < length; distance += .06f)
            lines.emplace_back(a + offset + direction * distance,
                a + offset + direction * PxMin(distance + .035f, length), BufferColor);
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

void FootSoleDebug::BuildLines(const PARS::SkeletalMeshComponent& mesh, std::vector<PxDebugLine>& lines) const
{
    if (!m_Visible) return;
    const auto& sample = mesh.GetFootSupport().GetSnapshot();
    for (size_t side = 0; side < sample.feet.size(); ++side)
    {
        const auto& foot = sample.feet[side];
        if (!foot.available) continue;
        for (size_t i = 0; i < foot.corners.size(); ++i)
        {
            const size_t next = (i + 1) % foot.corners.size();
            const Vector3 corner = ToMath(foot.corners[i]), projected = ToMath(foot.projections[i]);
            if (m_ShowRays) Line(lines, corner, projected, foot.projected[i] ? HitColor : MissColor);
            Point(lines, corner, FootColors[side]);
            Line(lines, corner, ToMath(foot.corners[next]), FootColors[side]);
            if (foot.projected[i]) Point(lines, projected, HitColor);
            if (foot.projected[i] && foot.projected[next])
                Line(lines, projected, ToMath(foot.projections[next]), HitColor);
        }
    }
    if (m_ShowHull)
    {
        for (size_t i = 0; i < sample.hull.size(); ++i)
        {
            const Vector3 point = ToMath(sample.hull[i]);
            Line(lines, point, ToMath(sample.hull[(i + 1) % sample.hull.size()]), HullColor);
            Point(lines, point, HullColor);
        }
    }
    if (m_ShowBuffer)
    {
        const auto& buffer = sample.balance.bufferHull;
        for (size_t i = 0; i < buffer.size(); ++i)
            DashedLine(lines, buffer[i], buffer[(i + 1) % buffer.size()]);
        if (!buffer.empty()) Point(lines, ToMath(sample.balance.center), BufferColor);
    }
    if (m_ShowCenterOfMass && sample.hasCenterOfMass)
    {
        const Vector3 center = ToMath(sample.centerOfMass);
        Point(lines, center, ComColor, .025f);
        Line(lines, center - Vector3(0, .025f, 0), center + Vector3(0, .025f, 0), ComColor);
        if (sample.hasComProjection)
        {
            const Vector3 projected = ToMath(sample.projectedCenterOfMass);
            const PxU32 color = sample.hull.empty() ? UnknownColor : sample.comInsideHull ? HitColor : OutsideColor;
            Line(lines, center, projected, ComColor);
            ProjectionMarker(lines, projected, color, sample.hull.empty() || !sample.comInsideHull);
        }
    }
}

void FootSoleDebug::DrawUI(PARS::SkeletalMeshComponent& mesh)
{
    if (!ImGui::CollapsingHeader(u8"발바닥 / 지면 투영", ImGuiTreeNodeFlags_DefaultOpen)) return;
    auto& support = mesh.GetFootSupport();
    const auto& sample = support.GetSnapshot();
    const auto& balance = sample.balance;
    const char* zone = u8"판정 불가";
    switch (balance.zone)
    {
    case PARS::FootSupportZone::Inside: zone = u8"A 안 / 경계"; break;
    case PARS::FootSupportZone::Buffer: zone = u8"A와 B 사이"; break;
    case PARS::FootSupportZone::Outside: zone = u8"B 경계 / 밖"; break;
    default: break;
    }
    ImGui::Text(u8"COM 위치: %s / 계산 계수 w = %.3f", zone, balance.weight);
    ImGui::ProgressBar(balance.weight, ImVec2(-1, 0));
    ImGui::TextUnformatted(u8"관측 단계: 계산 계수는 골반 지지에 아직 적용하지 않습니다.");
    if (balance.hasDirection)
        ImGui::Text(u8"중심 거리: COM %.1f / A %.1f / B %.1f cm",
            balance.comDistance * 100, balance.innerDistance * 100, balance.outerDistance * 100);
    ImGui::Checkbox(u8"발바닥 영역 표시", &m_Visible);
    ImGui::SameLine();
    ImGui::Checkbox(u8"레이 표시", &m_ShowRays);
    ImGui::Checkbox(u8"지지 영역 A 표시 (Convex Hull)", &m_ShowHull);
    ImGui::Checkbox(u8"완충 영역 B 표시 (흰색 점선)", &m_ShowBuffer);
    ImGui::Checkbox(u8"질량중심 / 수직 투영 표시", &m_ShowCenterOfMass);
    ImGui::TextWrapped(u8"아래 방향 20cm 이내 히트만 투영 · 초록: 성공 / 빨강: 범위 내 바닥 없음");
    ImGui::Text(u8"보라: A · 유효 투영점 %zu개 / 헐 꼭짓점 %zu개", sample.projectedPointCount, sample.hull.size());
    float scale = support.GetBufferScale();
    if (ImGui::SliderFloat(u8"B 거리 배율", &scale, 1.1f, 4.0f, "%.2f")) support.SetBufferScale(scale);
    ImGui::TextWrapped(u8"B는 A의 면적 중심 기준 확장입니다. 거리 2배 = 면적 4배.");
    ImGui::Text(u8"물리 스냅샷: %llu / 완료 스텝: %llu",
        static_cast<unsigned long long>(sample.sampleNumber), static_cast<unsigned long long>(sample.completedPhysicsStep));
    ImGui::TextWrapped(u8"표시를 꺼도 고정 틱에서 계산합니다. 설정 변경은 다음 물리 스텝에 반영됩니다.");
    if (sample.hull.empty()) ImGui::TextUnformatted(u8"지지 영역 없음: 유효한 점이 부족하거나 일직선입니다.");
    if (sample.hasCenterOfMass)
    {
        ImGui::Text(u8"전체 질량 %.2f kg / COM (%.3f, %.3f, %.3f) m", sample.totalMass,
            sample.centerOfMass.x, sample.centerOfMass.y, sample.centerOfMass.z);
        ImGui::TextWrapped(u8"노랑: COM과 수직선 / 투영 원: 초록=A 안, 굵은 빨강=A 밖, 굵은 검정=판정 불가");
        if (!sample.hasComProjection) ImGui::TextUnformatted(u8"COM 지면 투영점 없음");
    }
    if (!sample.message.empty()) ImGui::TextWrapped("%s", sample.message.c_str());
    if (ImGui::Button(u8"발바닥 기본 영역으로 초기화")) support.ResetSoles();
    const char* const labels[] = {u8"왼발", u8"오른발"};
    for (size_t side = 0; side < sample.feet.size(); ++side)
    {
        auto& settings = support.GetSoleSettings(side);
        const auto& foot = sample.feet[side];
        ImGui::PushID(static_cast<int>(side));
        if (foot.available && ImGui::TreeNodeEx(labels[side], ImGuiTreeNodeFlags_DefaultOpen))
        {
            float dimensions[] = {settings.halfWidth * 200, settings.halfLength * 200};
            if (ImGui::DragFloat2(u8"폭 / 길이", dimensions, .1f, 1, 100, "%.1f cm", ImGuiSliderFlags_AlwaysClamp))
            {
                settings.halfWidth = dimensions[0] * .005f;
                settings.halfLength = dimensions[1] * .005f;
            }
            ImGui::DragFloat3(u8"로컬 위치", &settings.offset.x, .001f, -.5f, .5f, "%.3f m", ImGuiSliderFlags_AlwaysClamp);
            ImGui::DragFloat3(u8"로컬 회전", &settings.rotation.x, .5f, -180, 180, "%.1f deg", ImGuiSliderFlags_AlwaysClamp);
            unsigned count = 0;
            for (bool hit : foot.projected) count += hit ? 1 : 0;
            ImGui::Text(u8"투영점 %u / 4", count);
            for (int i = 0; i < 4; ++i)
            {
                if (foot.projected[i]) ImGui::Text(u8"점 %d: %.1f cm", i + 1, foot.distances[i] * 100);
                else ImGui::Text(u8"점 %d: 투영 없음", i + 1);
            }
            ImGui::TreePop();
        }
        ImGui::PopID();
    }
}
