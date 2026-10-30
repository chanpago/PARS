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
    constexpr PxU32 PredictionColor = 0xff00ffff, TargetColor = 0xffff8800, RawTargetColor = 0xffaaaaaa;
    constexpr PxU32 MotionTargetColor = 0xffff66cc;
    constexpr PxU32 IKStepColor = 0xffffff33, IKSupportColor = 0xff44dd88, KneeHintColor = 0xffcc88ff;

    const char* FootLabel(int foot)
    {
        return foot == 0 ? u8"왼발" : foot == 1 ? u8"오른발" : u8"없음";
    }

    Vector3 ToMath(const PxVec3& point)
    {
        return Vector3(point.x, point.y, point.z);
    }

    void DashedLine(std::vector<PxDebugLine>& lines, const PxVec3& a, const PxVec3& b)
    {
        const PxVec3 delta = b - a;
        const float length = delta.magnitude();
        if (length < 1e-6f)
        {
            return;
        }
        const PxVec3 direction = delta / length, offset(0, .004f, 0);
        for (float distance = 0; distance < length; distance += .06f)
        {
            lines.emplace_back(a + offset + direction * distance,
                a + offset + direction * PxMin(distance + .035f, length), BufferColor);
        }
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
    if (!m_Visible)
    {
        return;
    }
    const auto& sample = mesh.GetFootSupport().GetSnapshot();
    for (size_t side = 0; side < sample.feet.size(); ++side)
    {
        const auto& foot = sample.feet[side];
        if (!foot.available)
        {
            continue;
        }
        for (size_t i = 0; i < foot.corners.size(); ++i)
        {
            const size_t next = (i + 1) % foot.corners.size();
            const Vector3 corner = ToMath(foot.corners[i]), projected = ToMath(foot.projections[i]);
            if (m_ShowRays)
            {
                Line(lines, corner, projected, foot.projected[i] ? HitColor : MissColor);
            }
            Point(lines, corner, FootColors[side]);
            Line(lines, corner, ToMath(foot.corners[next]), FootColors[side]);
            if (foot.projected[i])
            {
                Point(lines, projected, HitColor);
            }
            if (foot.projected[i] && foot.projected[next])
            {
                Line(lines, projected, ToMath(foot.projections[next]), HitColor);
            }
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
        {
            DashedLine(lines, buffer[i], buffer[(i + 1) % buffer.size()]);
        }
        if (!buffer.empty())
        {
            Point(lines, ToMath(sample.balance.center), BufferColor);
        }
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
    if (m_ShowFootstep)
    {
        const auto& step = sample.footstep;
        if (step.hasPrediction)
        {
            Line(lines, ToMath(sample.projectedCenterOfMass), ToMath(step.predictedCom), PredictionColor);
            ProjectionMarker(lines, ToMath(step.predictedCom), PredictionColor, false);
        }
        for (size_t side = 0; side < sample.feet.size(); ++side)
        {
            const auto& contact = sample.feet[side].contact;
            if (contact.grounded)
            {
                Point(lines, ToMath(contact.position), HitColor, .015f);
                Line(lines, ToMath(contact.position), ToMath(contact.position + contact.normal * .12f), HitColor);
            }
            const auto& target = step.candidates[side];
            if (!target.valid)
            {
                continue;
            }
            Point(lines, ToMath(target.raw), RawTargetColor, .012f);
            Point(lines, ToMath(target.position), TargetColor, .025f);
            Line(lines, ToMath(target.position), ToMath(target.position + target.normal * .08f), TargetColor);
        }
        if (step.hasTarget && step.stepFoot >= 0 && step.stepFoot < 2)
        {
            const Vector3 foot(sample.feet[step.stepFoot].soleWorld.GetW());
            // 연결선은 목표의 대응 관계다. 실제 발 이동 궤적을 뜻하지 않는다.
            Line(lines, foot, ToMath(step.target), TargetColor);
            ProjectionMarker(lines, ToMath(step.target), TargetColor, true);
            Point(lines, foot, TargetColor, .03f);
        }
        if (step.motion.valid && step.stepFoot >= 0 && step.stepFoot < 2)
        {
            const Vector3 goal = ToMath(step.motion.position);
            const Vector3 foot(sample.feet[step.stepFoot].soleWorld.GetW());
            Point(lines, goal, MotionTargetColor, .035f);
            Line(lines, goal - Vector3(0, .025f, 0), goal + Vector3(0, .025f, 0), MotionTargetColor);
            // 지면 착지점과 높이를 더한 목표를 분리해 표시한다. 실제 발 궤적을 그리는 선은 아니다.
            Line(lines, ToMath(step.target), goal, MotionTargetColor);
            Line(lines, foot, goal, MotionTargetColor);
        }
        const auto& ik = mesh.GetFootstepIKSnapshot();
        if (ik.status == PARS::FootstepIKStatus::Applied)
        {
            for (size_t side = 0; side < ik.legs.size(); ++side)
            {
                const auto& leg = ik.legs[side];
                const PxU32 color = int(side) == ik.stepFoot ? IKStepColor : IKSupportColor;
                Line(lines, ToMath(leg.hip.p), ToMath(leg.knee.p), color);
                Line(lines, ToMath(leg.knee.p), ToMath(leg.ankle.p), color);
                Point(lines, ToMath(leg.knee.p), color, .025f);
                Point(lines, ToMath(leg.solvedSole.p), color, .02f);
                Line(lines, ToMath(leg.requestedSole.p), ToMath(leg.solvedSole.p), color);
                Line(lines, ToMath(leg.solvedSole.p), ToMath(leg.solvedSole.transform(PxVec3(0, 0, .12f))), color);
                Line(lines, ToMath(leg.hip.p), ToMath(leg.hip.p + leg.bendDirection * .18f), KneeHintColor);
            }
        }
    }
}

void FootSoleDebug::DrawUI(PARS::SkeletalMeshComponent& mesh)
{
    if (!ImGui::CollapsingHeader(u8"발바닥 / 지면 투영", ImGuiTreeNodeFlags_DefaultOpen))
    {
        return;
    }
    auto& support = mesh.GetFootSupport();
    const auto& sample = support.GetSnapshot();
    const auto& balance = sample.balance;
    const char* zone = u8"판정 불가";
    switch (balance.zone)
    {
    case PARS::FootSupportZone::Inside:
        zone = u8"A 안 / 경계";
        break;
    case PARS::FootSupportZone::Buffer:
        zone = u8"A와 B 사이";
        break;
    case PARS::FootSupportZone::Outside:
        zone = u8"B 경계 / 밖";
        break;
    default:
        break;
    }
    ImGui::Text(u8"COM 위치: %s / 계산 계수 w = %.3f", zone, balance.weight);
    ImGui::ProgressBar(balance.weight, ImVec2(-1, 0));
    if (mesh.UsesAutomaticRootSupport())
    {
        const auto released = mesh.GetRootSupportRelease();
        const char* state = released == PARS::RootSupportRelease::None ? u8"지지 중" :
            released == PARS::RootSupportRelease::OutsideBuffer ? u8"B 이탈로 해제" :
            released == PARS::RootSupportRelease::NoSupport ? u8"발 지지 없음으로 해제" : u8"지지 해제";
        ImGui::Text(u8"적용 w = %.3f / %s", mesh.GetRagdoll().GetRootSupportWeight(), state);
        if (released != PARS::RootSupportRelease::None)
        {
            ImGui::TextUnformatted(u8"더미 초기화 전까지 해제 상태를 유지합니다.");
        }
    }
    else
    {
        ImGui::TextWrapped(u8"수동 지지 모드: 계산 계수는 골반 지지에 적용하지 않습니다.");
    }
    if (balance.hasDirection)
    {
        ImGui::Text(u8"중심 거리: COM %.1f / A %.1f / B %.1f cm",
            balance.comDistance * 100, balance.innerDistance * 100, balance.outerDistance * 100);
    }
    ImGui::Checkbox(u8"레이 표시", &m_ShowRays);
    ImGui::Checkbox(u8"지지 영역 A 표시 (Convex Hull)", &m_ShowHull);
    ImGui::Checkbox(u8"완충 영역 B 표시 (흰색 점선)", &m_ShowBuffer);
    ImGui::Checkbox(u8"질량중심 / 수직 투영 표시", &m_ShowCenterOfMass);
    ImGui::TextWrapped(u8"아래 방향 20cm 이내 히트만 투영 · 초록: 성공 / 빨강: 범위 내 바닥 없음");
    ImGui::Text(u8"보라: A · 유효 투영점 %zu개 / 헐 꼭짓점 %zu개", sample.projectedPointCount, sample.hull.size());
    float scale = support.GetBufferScale();
    if (ImGui::SliderFloat(u8"B 거리 배율", &scale, 1.1f, 4.0f, "%.2f"))
    {
        support.SetBufferScale(scale);
    }
    ImGui::TextWrapped(u8"B는 A의 면적 중심 기준 확장입니다. 거리 2배 = 면적 4배.");
    ImGui::Text(u8"물리 스냅샷: %llu / 완료 스텝: %llu",
        static_cast<unsigned long long>(sample.sampleNumber), static_cast<unsigned long long>(sample.completedPhysicsStep));
    ImGui::TextWrapped(u8"표시를 꺼도 고정 틱에서 계산합니다. 설정 변경은 다음 물리 스텝에 반영됩니다.");
    if (sample.hull.empty())
    {
        ImGui::TextUnformatted(u8"지지 영역 없음: 유효한 점이 부족하거나 일직선입니다.");
    }
    if (sample.hasCenterOfMass)
    {
        ImGui::Text(u8"전체 질량 %.2f kg / COM (%.3f, %.3f, %.3f) m", sample.totalMass,
            sample.centerOfMass.x, sample.centerOfMass.y, sample.centerOfMass.z);
        ImGui::TextWrapped(u8"노랑: COM과 수직선 / 투영 원: 초록=A 안, 굵은 빨강=A 밖, 굵은 검정=판정 불가");
        if (!sample.hasComProjection)
        {
            ImGui::TextUnformatted(u8"COM 지면 투영점 없음");
        }
    }
    if (sample.hasCenterOfMassVelocity)
    {
        const auto& velocity = sample.centerOfMassVelocity;
        ImGui::Text(u8"실제 COM 속도 (%.3f, %.3f, %.3f) m/s", velocity.x, velocity.y, velocity.z);
    }
    if (ImGui::TreeNodeEx(u8"발딛기 판단 / 착지 목표", ImGuiTreeNodeFlags_DefaultOpen))
    {
        const auto& step = sample.footstep;
        auto& settings = support.GetFootstepSettings();
        ImGui::Checkbox(u8"발딛기 판단·목표 3D 표시", &m_ShowFootstep);
        ImGui::Text(u8"상태: %s", PARS::FootstepPhaseLabel(step.phase));
        ImGui::TextWrapped(u8"사유: %s", PARS::FootstepReasonLabel(step.reason));
        ImGui::Text(u8"원시 파란 후보: %s / 저장한 발: %s", FootLabel(step.rawBlueFoot), FootLabel(step.stepFoot));
        ImGui::Text(u8"시작 %llu / 착지 %llu / 중단 %llu",
            static_cast<unsigned long long>(step.startCount), static_cast<unsigned long long>(step.landingCount),
            static_cast<unsigned long long>(step.abortCount));
        ImGui::Text(u8"예측: 시작 %.2fs / 착지 %.2fs · 발 간격 %.2f · 최대 보폭 %.1fm",
            settings.startPredictionTime, settings.landingPredictionTime, settings.stanceScale, settings.maximumStepLength);
        ImGui::Checkbox(u8"발 교차 억제 (경계 0.1m)", &settings.preventCrossing);
        if (step.hasPrediction)
        {
            ImGui::Text(u8"예측 COM (%.3f, %.3f, %.3f) m / A %s", step.predictedCom.x,
                step.predictedCom.y, step.predictedCom.z, step.needsStep ? u8"밖" : u8"안");
        }
        for (size_t side = 0; side < step.candidates.size(); ++side)
        {
            const auto& target = step.candidates[side];
            if (target.valid)
            {
                ImGui::Text(u8"%s 목표 (%.3f, %.3f, %.3f) m", FootLabel(static_cast<int>(side)),
                    target.position.x, target.position.y, target.position.z);
                ImGui::Text(u8"  발 이동 %.2fm / 지지발 거리 %.2fm · 보폭 제한 %s / 교차 제한 %s",
                    target.travelDistance, target.stanceDistance, target.lengthLimited ? "ON" : "OFF",
                    target.crossingLimited ? "ON" : "OFF");
            }
            else
            {
                ImGui::Text(u8"%s 후보: %s", FootLabel(static_cast<int>(side)), PARS::FootstepReasonLabel(target.reason));
            }
        }
        if (step.IsInProgress())
        {
            ImGui::Text(u8"시작 시 이동 거리 %.2fm / 목표 출력 %s", step.initialDistance, step.hasTarget ? "ON" : "OFF");
        }
        if (step.IsInProgress() || step.phase == PARS::FootstepPhase::Landed)
        {
            const auto& motion = step.motion;
            ImGui::Text(u8"초기 들기 높이 %.1fcm / 시작 시 종아리 상한 %.1fcm",
                motion.initialHeight * 100, motion.heightLimit * 100);
            ImGui::Text(u8"이동 진행도 %.1f%% (현재 측정 %.1f%%) / 남은 수평 거리 %.1fcm",
                motion.progress * 100, motion.measuredProgress * 100, motion.remainingDistance * 100);
            ImGui::ProgressBar(motion.progress, ImVec2(-1, 0));
            if (motion.valid)
            {
                ImGui::Text(u8"추가 높이 %.1fcm / 발 이동 목표 (%.3f, %.3f, %.3f) m", motion.height * 100,
                    motion.position.x, motion.position.y, motion.position.z);
            }
            else
            {
                ImGui::TextUnformatted(u8"발 이동 목표 출력 OFF");
            }
            if (motion.hasLandingError)
            {
                ImGui::Text(u8"재접촉 시 수평 목표 오차 %.1fcm (목표 도달 여부와 접촉 종료는 별개)", motion.landingError * 100);
            }
        }
        ImGui::Text(u8"초기 높이 = min(시작 수평 거리 x %.2f, 기준 종아리 길이)", settings.liftHeightScale);
        ImGui::TextWrapped(u8"이륙 전에는 초기 높이를 유지합니다. 이륙 후 실제 수평 진행에 따라 낮추며, 100%가 되어도 실제 접촉 전에는 종료하지 않습니다.");
        ImGui::TextWrapped(u8"청록: 예측 COM · 회색: 원시 후보 · 주황 십자: 착지 후보 · 주황 원: 저장한 발의 목표 · 초록 선: 실제 접촉 법선");
        ImGui::TextWrapped(u8"분홍 십자: 높이를 더한 발 이동 목표. 노랑/연두: 이동/지지 다리의 제한 적용 전 IK 자세. 보라 선: 무릎 굽힘 방향. 연결선은 실제 이동 궤적이 아닙니다.");
        const auto& ik = mesh.GetFootstepIKSnapshot();
        ImGui::Text(u8"IK → 관절 PD: %s", PARS::FootstepIKStatusLabel(ik.status));
        if (ik.status == PARS::FootstepIKStatus::Applied)
        {
            for (size_t side = 0; side < ik.legs.size(); ++side)
            {
                const auto& leg = ik.legs[side];
                const float actualError = (ToPx(Vector3(sample.feet[side].soleWorld.GetW())) - leg.requestedSole.p).magnitude();
                ImGui::Text(u8"%s (%s): IK 도달 오차 %.2fcm / 실제 발 목표 오차 %.2fcm%s", FootLabel(int(side)),
                    int(side) == ik.stepFoot ? u8"이동" : u8"지지", leg.reachError * 100, actualError * 100,
                    leg.reachLimited ? u8" / 다리 길이 제한" : "");
                ImGui::Text(u8"  무릎 Hint: %s / 발목 목표 (%.3f, %.3f, %.3f) m",
                    PARS::KneeHintSourceLabel(leg.hintSource), leg.requestedAnkle.x, leg.requestedAnkle.y, leg.requestedAnkle.z);
            }
            ImGui::Text(u8"기존 관절 제한의 최대 목표 수정각 (전신): %.2f도", XMConvertToDegrees(mesh.GetRagdoll().GetMaxTargetClampAngle()));
            ImGui::TextWrapped(u8"IK 도달 오차는 다리 길이로 제한한 결과입니다. 이후 PA 관절 제한과 실제 물리 추종 오차는 별도이며, IK 표시만으로 착지나 회복 성공을 뜻하지 않습니다.");
        }
        ImGui::TreePop();
    }
    if (!sample.message.empty())
    {
        ImGui::TextWrapped("%s", sample.message.c_str());
    }
    if (ImGui::Button(u8"발바닥 기본 영역으로 초기화"))
    {
        support.ResetSoles();
    }
    const char* const labels[] = {u8"왼발", u8"오른발"};
    for (size_t side = 0; side < sample.feet.size(); ++side)
    {
        auto& settings = support.GetSoleSettings(side);
        const auto& foot = sample.feet[side];
        ImGui::PushID(static_cast<int>(side));
        if (foot.available && ImGui::TreeNodeEx(labels[side], ImGuiTreeNodeFlags_DefaultOpen))
        {
            const auto& contact = foot.contact;
            ImGui::Text(u8"실제 접촉: %s / 지면 접촉점 %u개", !contact.hasSample ? u8"물리 스텝 대기" :
                contact.grounded ? u8"지지 중" : u8"접촉 없음", contact.pointCount);
            ImGui::Text(u8"접촉 이력: 이륙 %llu회 / 착지 %llu회",
                static_cast<unsigned long long>(contact.takeoffCount), static_cast<unsigned long long>(contact.landingCount));
            if (contact.grounded)
            {
                ImGui::Text(u8"접촉점 (%.3f, %.3f, %.3f) m", contact.position.x, contact.position.y, contact.position.z);
                ImGui::Text(u8"지면 법선 (%.3f, %.3f, %.3f)", contact.normal.x, contact.normal.y, contact.normal.z);
            }
            if (foot.leg.available)
            {
                ImGui::Text(u8"다리 본: 고관절 %d / 무릎 %d / 발목 %d",
                    foot.leg.hipNode, foot.leg.kneeNode, foot.leg.ankleNode);
                ImGui::Text(u8"기준 길이: 허벅지 %.1f / 종아리 %.1f cm",
                    foot.leg.thighLength * 100, foot.leg.calfLength * 100);
            }
            else
            {
                ImGui::TextUnformatted(u8"다리 본 연결 또는 기준 길이를 찾을 수 없습니다.");
            }
            float dimensions[] = {settings.halfWidth * 200, settings.halfLength * 200};
            if (ImGui::DragFloat2(u8"폭 / 길이", dimensions, .1f, 1, 100, "%.1f cm", ImGuiSliderFlags_AlwaysClamp))
            {
                settings.halfWidth = dimensions[0] * .005f;
                settings.halfLength = dimensions[1] * .005f;
            }
            ImGui::DragFloat3(u8"로컬 위치", &settings.offset.x, .001f, -.5f, .5f, "%.3f m", ImGuiSliderFlags_AlwaysClamp);
            ImGui::DragFloat3(u8"로컬 회전", &settings.rotation.x, .5f, -180, 180, "%.1f deg", ImGuiSliderFlags_AlwaysClamp);
            unsigned count = 0;
            for (bool hit : foot.projected)
            {
                count += hit ? 1 : 0;
            }
            ImGui::Text(u8"투영점 %u / 4", count);
            for (int i = 0; i < 4; ++i)
            {
                if (foot.projected[i])
                {
                    ImGui::Text(u8"점 %d: %.1f cm", i + 1, foot.distances[i] * 100);
                }
                else
                {
                    ImGui::Text(u8"점 %d: 투영 없음", i + 1);
                }
            }
            ImGui::TreePop();
        }
        ImGui::PopID();
    }
}
