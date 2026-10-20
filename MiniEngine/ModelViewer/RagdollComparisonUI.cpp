#include "RagdollComparisonRunner.h"
#include "imgui.h"
#include "ViewerLabels.h"
#include <algorithm>
#include <numeric>
#include <cmath>
#include <cfloat>

using namespace PARS;

namespace
{
    constexpr float Degrees = 180.0f / DirectX::XM_PI;
    const char* const Axes[] = {u8"트위스트 X", u8"스윙 Y", u8"스윙 Z", u8"스윙 원뿔"};

    void Number(float value)
    {
        ImGui::TableNextColumn();
        ImGui::Text("%.3f", value);
    }

    const char* Motion(PhysicsJointMotion motion)
    {
        return motion == PhysicsJointMotion::Locked ? u8"잠금" : motion == PhysicsJointMotion::Limited ? u8"제한"
                                                                                                       : u8"자유";
    }

    struct PlotData
    {
        const std::vector<ComparisonFrame>& frames;
        int index, metric;
    };

    float PlotValue(void* data, int frame)
    {
        const auto& plot = *static_cast<PlotData*>(data);
        const auto& sample = plot.frames[frame];
        switch (plot.metric)
        {
        case 0:
            return sample.bodies[plot.index].positionError * 100;
        case 1:
            return sample.bodies[plot.index].movementError * 100;
        case 2:
            return sample.bodies[plot.index].rotationError * Degrees;
        case 3:
            return sample.joints[plot.index].clampError * Degrees;
        case 4:
            return sample.joints[plot.index].trackingError * Degrees;
        default:
            return sample.joints[plot.index].totalError * Degrees;
        }
    }

    void Plot(const char* label, const std::vector<ComparisonFrame>& frames, int index, int metric)
    {
        PlotData data{frames, index, metric};
        ImGui::PlotLines(label, PlotValue, &data, static_cast<int>(frames.size()), 0, nullptr, 0, FLT_MAX, ImVec2(0, 70));
    }
}

void RagdollComparisonRunner::DrawUI(const PhysicsAsset* editor, const PhysicsAsset* character)
{
    if (!m_Open)
    {
        return;
    }
    const auto display = ImGui::GetIO().DisplaySize;
    ImGui::SetNextWindowSize(ImVec2(std::min(1020.0f, display.x - 40), std::min(740.0f, display.y - 70)), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowPos(ImVec2(20, 40), ImGuiCond_FirstUseEver);
    if (m_WindowFocusRequested)
    {
        ImGui::SetNextWindowFocus();
        m_WindowFocusRequested = false;
    }
    if (!ImGui::Begin(u8"래그돌 비교", &m_Open))
    {
        ImGui::End();
        return;
    }
    ImGui::TextUnformatted(u8"키네마틱 기준 | 액티브 래그돌");
    ImGui::TextWrapped(u8"플레이어와 분리된 더미 장면입니다. 같은 애니메이션을 반복하며 지정된 +Z 경로로 이동합니다. "
                       u8"최초 배치 차이만 제거하며 원본 애니메이션을 절대 기준으로 사용합니다.");
    const bool busy = m_State == State::Running || m_State == State::Paused;
    ImGui::BeginDisabled(busy);
    ImGui::Checkbox(u8"에디터 PA 사용 (저장하지 않은 편집 포함)", &m_UseEditor);
    ImGui::SetNextItemWidth(160);
    if (ImGui::BeginCombo(u8"애니메이션", ViewerLabels::Animation(m_Names[m_RequestedSettings.clip])))
    {
        for (int i = 0; i < static_cast<int>(m_Names.size()); ++i)
        {
            if (ImGui::Selectable(ViewerLabels::Animation(m_Names[i]), m_RequestedSettings.clip == i))
            {
                m_RequestedSettings.clip = i;
            }
        }
        ImGui::EndCombo();
    }
    ImGui::SameLine();
    ImGui::SetNextItemWidth(100);
    ImGui::DragFloat(u8"측정 시간 (초)", &m_RequestedSettings.duration, .5f, 1, 60, "%.1f", ImGuiSliderFlags_AlwaysClamp);
    ImGui::SameLine();
    ImGui::SetNextItemWidth(100);
    ImGui::DragFloat(u8"이동 속도 (m/s)", &m_RequestedSettings.speed, .1f, 0, 3, "%.2f", ImGuiSliderFlags_AlwaysClamp);
    ImGui::Checkbox(u8"공중 지지 (+1 m): 바닥 접촉 없이 추종 성능 측정", &m_RequestedSettings.suspended);
    ImGui::EndDisabled();
    const auto* source = m_UseEditor ? editor : character;
    ImGui::BeginDisabled(!source || source->bodies.empty() || busy);
    if (ImGui::Button(u8"시작 / 다시 실행"))
    {
        m_RequestedAsset = *source;
        m_RequestedSource = m_UseEditor ? "Editor PA (snapshot at start)" : "Character PA (snapshot at start)";
        m_StartRequested = true;
    }
    ImGui::EndDisabled();
    ImGui::SameLine();
    ImGui::BeginDisabled(!busy);
    if (ImGui::Button(m_State == State::Paused ? u8"계속" : u8"일시정지"))
    {
        SetPaused(m_State != State::Paused);
    }
    ImGui::SameLine();
    if (ImGui::Button(u8"중단"))
    {
        Stop();
    }
    ImGui::EndDisabled();
    ImGui::SameLine();
    ImGui::BeginDisabled(m_State != State::Paused);
    if (ImGui::Button(u8"1/60초 진행"))
    {
        Step();
    }
    ImGui::EndDisabled();
    ImGui::SameLine();
    if (ImGui::Button(u8"초기화"))
    {
        m_ResetRequested = true;
    }
    ImGui::SameLine();
    ImGui::Checkbox(u8"더미 표시", &m_ShowScene);
    ImGui::SameLine();
    if (ImGui::Button(u8"더미에 화면 맞춤"))
    {
        m_ShowScene = true;
        m_FocusRequested = true;
    }
    ImGui::Checkbox(u8"카메라로 더미 따라가기", &m_FollowCamera);
    ImGui::SameLine();
    ImGui::TextDisabled(u8"키네마틱: -X 방향 | 액티브: +X 방향");
    ImGui::Text(u8"표본: %zu / %zu | 실험 시간: %.3f초 | 60 Hz | 시작 직후 포함",
        m_Steps, m_TargetSteps, m_Steps / 60.0);
    if (m_TargetSteps)
    {
        ImGui::ProgressBar(static_cast<float>(m_Steps) / m_TargetSteps, ImVec2(-1, 0));
    }
    if (!m_Message.empty())
    {
        ImGui::TextWrapped("%s", m_Message.c_str());
    }
    if (!m_Results.GetFrames().empty())
    {
        ImGui::TextWrapped("%s", GetDescription(true).c_str());
        ImGui::SetNextItemWidth(500);
        ImGui::InputText(u8"HTML 보고서", m_ReportPath, sizeof(m_ReportPath));
        ImGui::SameLine();
        if (ImGui::Button(u8"HTML 저장"))
        {
            Export(m_ReportPath);
        }
        ImGui::TextDisabled(u8"표, 시간별 측정값과 PA 설정을 한 파일에 저장합니다. 브라우저 인쇄 기능으로 PDF 저장이 가능합니다.");
        DrawSelectedSample();
        if (ImGui::BeginTabBar("Comparison results"))
        {
            if (ImGui::BeginTabItem(u8"바디 이동 비교"))
            {
                DrawBodyResults();
                ImGui::EndTabItem();
            }
            if (ImGui::BeginTabItem(u8"트위스트 / 스윙 진단"))
            {
                DrawJointResults();
                ImGui::EndTabItem();
            }
            ImGui::EndTabBar();
        }
    }
    ImGui::End();
}

void RagdollComparisonRunner::DrawSelectedSample()
{
    const auto& frames = m_Results.GetFrames();
    const int last = static_cast<int>(frames.size()) - 1;
    if (m_FollowLatest)
    {
        m_SelectedFrame = last;
    }
    m_SelectedFrame = std::max(0, std::min(last, m_SelectedFrame));
    ImGui::Checkbox(u8"최신 표본 따라가기", &m_FollowLatest);
    ImGui::SameLine();
    ImGui::SetNextItemWidth(360);
    if (ImGui::SliderInt(u8"기록된 표본", &m_SelectedFrame, 0, last))
    {
        m_FollowLatest = false;
    }
    const auto& frame = frames[m_SelectedFrame];
    ImGui::Text(u8"표본 %d | 실험 시간 %.4f초 | 애니메이션 시간 %.4f초 (반복)", m_SelectedFrame + 1, frame.time, frame.clipTime);
    ImGui::TextDisabled(u8"슬라이더로 기록된 수치를 확인합니다. 장면의 자세는 현재 시뮬레이션 상태를 유지합니다.");
}

void RagdollComparisonRunner::DrawBodyResults()
{
    const auto& results = m_Results.GetBodies();
    const auto& frames = m_Results.GetFrames();
    const auto& asset = m_Results.GetAsset();
    ImGui::TextWrapped(u8"위치 오차는 콜라이더 중심 간 거리입니다. 이동량 오차는 스텝별 변위 벡터의 차이이며 "
                       u8"이동 거리끼리의 차이가 아닙니다. 회전 오차는 월드 공간 바디 방향의 차이입니다.");
    if (ImGui::BeginTable("Body errors", 9, ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_ScrollY | ImGuiTableFlags_ScrollX | ImGuiTableFlags_Resizable, ImVec2(0, 240)))
    {
        const char* headers[] = {u8"바디 (큰 오차순)", u8"위치 오차 평균 (cm)", u8"위치 오차 RMS (cm)", u8"위치 오차 최대 (cm)", u8"최대 시각 (초)",
            u8"이동량 오차 평균 (cm/스텝)", u8"이동량 오차 최대 (cm/스텝)", u8"회전 오차 평균 (도)", u8"회전 오차 최대 (도)"};
        for (const auto* header : headers)
        {
            ImGui::TableSetupColumn(header);
        }
        ImGui::TableSetupScrollFreeze(1, 1);
        ImGui::TableHeadersRow();
        std::vector<int> order(results.size());
        std::iota(order.begin(), order.end(), 0);
        std::stable_sort(order.begin(), order.end(), [&](int a, int b) {
            return results[a].position.maximum > results[b].position.maximum;
        });
        for (int i : order)
        {
            const auto& body = results[i];
            ImGui::TableNextRow();
            ImGui::TableNextColumn();
            if (ImGui::Selectable(asset.bodies[i].bone.c_str(), m_SelectedBody == i, ImGuiSelectableFlags_SpanAllColumns))
            {
                m_SelectedBody = i;
            }
            Number(body.position.Mean() * 100);
            Number(body.position.Rms() * 100);
            Number(body.position.maximum * 100);
            Number(body.position.maximumTime);
            Number(body.movement.Mean() * 100);
            Number(body.movement.maximum * 100);
            Number(body.rotation.Mean() * Degrees);
            Number(body.rotation.maximum * Degrees);
        }
        ImGui::EndTable();
    }
    const auto& sample = frames[m_SelectedFrame].bodies[m_SelectedBody];
    ImGui::Text(u8"선택: %s", asset.bodies[m_SelectedBody].bone.c_str());
    if (ImGui::Button(u8"최대 위치 오차 보기"))
    {
        m_SelectedFrame = static_cast<int>(std::round(results[m_SelectedBody].position.maximumTime / m_Results.GetStep())) - 1;
        m_FollowLatest = false;
    }
    ImGui::Text(u8"기준 XYZ (m): %.4f  %.4f  %.4f | 실제: %.4f  %.4f  %.4f",
        sample.reference.x, sample.reference.y, sample.reference.z, sample.actual.x, sample.actual.y, sample.actual.z);
    ImGui::Text(u8"실제 - 기준 (cm): %.3f  %.3f  %.3f | 이동량 차이 (cm/스텝): %.3f  %.3f  %.3f",
        sample.difference.x * 100, sample.difference.y * 100, sample.difference.z * 100,
        sample.movementDifference.x * 100, sample.movementDifference.y * 100, sample.movementDifference.z * 100);
    Plot(u8"위치 오차 (cm)", frames, m_SelectedBody, 0);
    Plot(u8"이동량 오차 (cm/스텝)", frames, m_SelectedBody, 1);
    Plot(u8"회전 오차 (도)", frames, m_SelectedBody, 2);
}

void RagdollComparisonRunner::DrawJointResults()
{
    const auto& results = m_Results.GetJoints();
    const auto& frames = m_Results.GetFrames();
    const auto& asset = m_Results.GetAsset();
    if (results.empty())
    {
        ImGui::TextUnformatted(u8"이 PA에는 관절이 없습니다.");
        return;
    }
    ImGui::TextWrapped(u8"원본 → 보정 목표는 제한에 따른 목표 변경량이고, 보정 목표 → 실제는 추종 오차입니다. "
                       u8"제한 위반은 자세와 설정이 맞지 않는다는 뜻입니다. 제한값뿐 아니라 축, 기준 좌표계와 원본 애니메이션을 함께 확인하세요.");
    if (ImGui::BeginTable("Joint errors", 9, ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_ScrollY | ImGuiTableFlags_ScrollX | ImGuiTableFlags_Resizable, ImVec2(0, 240)))
    {
        const char* headers[] = {u8"관절 (큰 보정순)", u8"목표 보정 평균 (도)", u8"목표 보정 최대 (도)", u8"추종 오차 평균 (도)", u8"추종 오차 최대 (도)",
            u8"트위스트 위반 (%)", u8"스윙 Y 위반 (%)", u8"스윙 Z 위반 (%)", u8"원뿔 위반 (%)"};
        for (const auto* header : headers)
        {
            ImGui::TableSetupColumn(header);
        }
        ImGui::TableSetupScrollFreeze(1, 1);
        ImGui::TableHeadersRow();
        std::vector<int> order(results.size());
        std::iota(order.begin(), order.end(), 0);
        std::stable_sort(order.begin(), order.end(), [&](int a, int b) {
            return results[a].clamp.maximum > results[b].clamp.maximum;
        });
        for (int i : order)
        {
            const auto& joint = results[i];
            ImGui::TableNextRow();
            ImGui::TableNextColumn();
            if (joint.clamp.maximum > .01f)
            {
                ImGui::TableSetBgColor(ImGuiTableBgTarget_RowBg0, IM_COL32(115, 64, 15, 90));
            }
            if (ImGui::Selectable(asset.constraints[i].childBone.c_str(), m_SelectedJoint == i, ImGuiSelectableFlags_SpanAllColumns))
            {
                m_SelectedJoint = i;
            }
            Number(joint.clamp.Mean() * Degrees);
            Number(joint.clamp.maximum * Degrees);
            Number(joint.tracking.Mean() * Degrees);
            Number(joint.tracking.maximum * Degrees);
            for (const auto& violation : joint.violations)
            {
                Number(100.0f * violation.frames / frames.size());
            }
        }
        ImGui::EndTable();
    }
    const auto& config = asset.constraints[m_SelectedJoint];
    const auto& result = results[m_SelectedJoint];
    const auto& sample = frames[m_SelectedFrame].joints[m_SelectedJoint];
    ImGui::Text(u8"%s → %s | %s | 원본 → 실제: %.3f도",
        config.parentBone.c_str(), config.childBone.c_str(), config.useSlerpDrive ? "SLERP" : u8"스윙 / 트위스트", sample.totalError * Degrees);
    if (ImGui::Button(u8"최대 목표 보정 보기"))
    {
        m_SelectedFrame = static_cast<int>(std::round(result.clamp.maximumTime / m_Results.GetStep())) - 1;
        m_FollowLatest = false;
    }
    const PhysicsJointMotion motions[] = {config.twist, config.swingY, config.swingZ};
    const float minimum[] = {config.twistMin, -config.swingYLimit, -config.swingZLimit};
    const float maximum[] = {config.twistMax, config.swingYLimit, config.swingZLimit};
    if (ImGui::BeginTable("Axis values", 10, ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_ScrollX))
    {
        const char* headers[] = {u8"축", u8"허용 범위 (도)", u8"원본 (도)", u8"보정 목표 (도)", u8"실제 (도)",
            u8"원본 최대 초과 (도)", u8"위반 총시간 (초)", u8"최장 연속 (초)", u8"최초 / 마지막 (초)", u8"실제 최대 초과 (도)"};
        for (const auto* header : headers)
        {
            ImGui::TableSetupColumn(header);
        }
        ImGui::TableHeadersRow();
        for (int axis = 0; axis < 4; ++axis)
        {
            const auto& violation = result.violations[axis];
            ImGui::TableNextRow();
            ImGui::TableNextColumn();
            ImGui::TextUnformatted(Axes[axis]);
            ImGui::TableNextColumn();
            if (axis == 3)
            {
                ImGui::TextUnformatted(config.swingY == PhysicsJointMotion::Limited && config.swingZ == PhysicsJointMotion::Limited ? u8"두 축 타원 제한" : u8"적용 안 됨");
            }
            else if (motions[axis] == PhysicsJointMotion::Limited)
            {
                ImGui::Text("%.1f .. %.1f", minimum[axis] * Degrees, maximum[axis] * Degrees);
            }
            else
            {
                ImGui::TextUnformatted(Motion(motions[axis]));
            }
            if (axis < 3)
            {
                Number(sample.raw.angles[axis] * Degrees);
                Number(sample.bounded.angles[axis] * Degrees);
                Number(sample.actual.angles[axis] * Degrees);
            }
            else
            {
                for (int column = 0; column < 3; ++column)
                {
                    ImGui::TableNextColumn();
                    ImGui::TextUnformatted("-");
                }
            }
            Number(violation.excess.maximum * Degrees);
            Number(violation.frames * m_Results.GetStep());
            Number(violation.longestFrames * m_Results.GetStep());
            ImGui::TableNextColumn();
            if (violation.frames)
            {
                ImGui::Text("%.3f / %.3f", violation.firstTime, violation.lastTime);
            }
            else
            {
                ImGui::TextUnformatted("-");
            }
            Number(result.actualViolations[axis].excess.maximum * Degrees);
        }
        ImGui::EndTable();
    }
    ImGui::TextDisabled(u8"원뿔 초과량은 스윙 각도 좌표에서 경계까지의 방사 방향 거리입니다. 위반 판정 허용치: 0.0001 rad.");
    for (int axis = 0; axis < 3; ++axis)
    {
        const auto& drive = config.drives[config.useSlerpDrive ? 0 : axis];
        ImGui::Text(u8"%s: 원본 범위 [%.2f, %.2f]도 | 드라이브 %s | Kp %.1f Kd %.1f 최대 %.1f Nm",
            Axes[axis], result.minimumAngles[axis] * Degrees, result.maximumAngles[axis] * Degrees,
            drive.enabled ? u8"켜짐" : u8"꺼짐", drive.stiffness, drive.damping, drive.maxTorque);
    }
    if (result.singularFrames)
    {
        ImGui::TextWrapped(u8"주의: 원본 표본 %zu개에서 스윙이 180도에 가까워 트위스트 분해가 불명확합니다. "
                           u8"해당 트위스트 값은 모호하므로 관절 기준 좌표계를 확인하세요.",
            result.singularFrames);
    }
    Plot(u8"원본 → 보정 목표 (도)", frames, m_SelectedJoint, 3);
    Plot(u8"보정 목표 → 실제 (도)", frames, m_SelectedJoint, 4);
    Plot(u8"원본 → 실제 (도)", frames, m_SelectedJoint, 5);
}
