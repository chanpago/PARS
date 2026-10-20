#include "RagdollComparisonRunner.h"
#include "imgui.h"
#include <algorithm>
#include <numeric>
#include <cmath>
#include <cfloat>

using namespace PARS;

namespace
{
    constexpr float Degrees = 180.0f / DirectX::XM_PI;
    const char* const Axes[] = {"Twist X", "Swing Y", "Swing Z", "Swing cone"};

    void Number(float value)
    {
        ImGui::TableNextColumn();
        ImGui::Text("%.3f", value);
    }

    const char* Motion(PhysicsJointMotion motion)
    {
        return motion == PhysicsJointMotion::Locked ? "Locked" : motion == PhysicsJointMotion::Limited ? "Limited"
                                                                                                       : "Free";
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
    if (!ImGui::Begin("Ragdoll Comparison", &m_Open))
    {
        ImGui::End();
        return;
    }
    ImGui::TextUnformatted("Kinematic reference  |  Active ragdoll");
    ImGui::TextWrapped("Independent dummy scene. Both use the same looping clip and prescribed +Z path; no player input. "
                       "Only initial placement is removed. Raw animation remains the absolute reference.");
    const bool busy = m_State == State::Running || m_State == State::Paused;
    ImGui::BeginDisabled(busy);
    ImGui::Checkbox("Use editor PA (including unsaved edits)", &m_UseEditor);
    ImGui::SetNextItemWidth(160);
    if (ImGui::BeginCombo("Clip", m_Names[m_RequestedSettings.clip].c_str()))
    {
        for (int i = 0; i < static_cast<int>(m_Names.size()); ++i)
        {
            if (ImGui::Selectable(m_Names[i].c_str(), m_RequestedSettings.clip == i))
            {
                m_RequestedSettings.clip = i;
            }
        }
        ImGui::EndCombo();
    }
    ImGui::SameLine();
    ImGui::SetNextItemWidth(100);
    ImGui::DragFloat("Seconds", &m_RequestedSettings.duration, .5f, 1, 60, "%.1f", ImGuiSliderFlags_AlwaysClamp);
    ImGui::SameLine();
    ImGui::SetNextItemWidth(100);
    ImGui::DragFloat("Travel m/s", &m_RequestedSettings.speed, .1f, 0, 3, "%.2f", ImGuiSliderFlags_AlwaysClamp);
    ImGui::Checkbox("Suspended (+1 m): isolate tracking from the floor", &m_RequestedSettings.suspended);
    ImGui::EndDisabled();
    const auto* source = m_UseEditor ? editor : character;
    ImGui::BeginDisabled(!source || source->bodies.empty() || busy);
    if (ImGui::Button("Start / rerun"))
    {
        m_RequestedAsset = *source;
        m_RequestedSource = m_UseEditor ? "Editor PA (snapshot at start)" : "Character PA (snapshot at start)";
        m_StartRequested = true;
    }
    ImGui::EndDisabled();
    ImGui::SameLine();
    ImGui::BeginDisabled(!busy);
    if (ImGui::Button(m_State == State::Paused ? "Resume" : "Pause"))
    {
        SetPaused(m_State != State::Paused);
    }
    ImGui::SameLine();
    if (ImGui::Button("Stop"))
    {
        Stop();
    }
    ImGui::EndDisabled();
    ImGui::SameLine();
    ImGui::BeginDisabled(m_State != State::Paused);
    if (ImGui::Button("Step 1/60 s"))
    {
        Step();
    }
    ImGui::EndDisabled();
    ImGui::SameLine();
    if (ImGui::Button("Reset"))
    {
        m_ResetRequested = true;
    }
    ImGui::SameLine();
    ImGui::Checkbox("Show dummies in main view", &m_ShowScene);
    ImGui::SameLine();
    if (ImGui::Button("Focus dummies"))
    {
        m_ShowScene = true;
        m_FocusRequested = true;
    }
    ImGui::Checkbox("Follow dummies with camera", &m_FollowCamera);
    ImGui::SameLine();
    ImGui::TextDisabled("Kinematic: -X side | Active: +X side");
    ImGui::Text("Samples: %zu / %zu | simulation: %.3f s | 60 Hz | startup included",
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
        ImGui::TextWrapped("%s", GetDescription().c_str());
        ImGui::SetNextItemWidth(500);
        ImGui::InputText("HTML report", m_ReportPath, sizeof(m_ReportPath));
        ImGui::SameLine();
        if (ImGui::Button("Export HTML"))
        {
            Export(m_ReportPath);
        }
        ImGui::TextDisabled("Self-contained report: tables, timelines, per-step values and PA settings. Browser Print -> PDF.");
        DrawSelectedSample();
        if (ImGui::BeginTabBar("Comparison results"))
        {
            if (ImGui::BeginTabItem("Body trajectories"))
            {
                DrawBodyResults();
                ImGui::EndTabItem();
            }
            if (ImGui::BeginTabItem("Twist / swing diagnostics"))
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
    ImGui::Checkbox("Follow latest sample", &m_FollowLatest);
    ImGui::SameLine();
    ImGui::SetNextItemWidth(360);
    if (ImGui::SliderInt("Recorded sample", &m_SelectedFrame, 0, last))
    {
        m_FollowLatest = false;
    }
    const auto& frame = frames[m_SelectedFrame];
    ImGui::Text("Sample %d | t = %.4f s | clip t = %.4f s (looped)", m_SelectedFrame + 1, frame.time, frame.clipTime);
    ImGui::TextDisabled("The slider inspects recorded numbers; the scene stays at its current simulation pose.");
}

void RagdollComparisonRunner::DrawBodyResults()
{
    const auto& results = m_Results.GetBodies();
    const auto& frames = m_Results.GetFrames();
    const auto& asset = m_Results.GetAsset();
    ImGui::TextWrapped("Position: collider centre distance. Movement: difference between per-step displacement vectors "
                       "(not difference of distances). Rotation: world body orientation error.");
    if (ImGui::BeginTable("Body errors", 9, ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_ScrollY | ImGuiTableFlags_ScrollX | ImGuiTableFlags_Resizable, ImVec2(0, 240)))
    {
        const char* headers[] = {"Body (worst first)", "Mean cm", "RMS cm", "Peak cm", "Peak at s",
            "Move mean cm/step", "Move peak cm/step", "Rot mean deg", "Rot peak deg"};
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
    ImGui::Text("Selected: %s", asset.bodies[m_SelectedBody].bone.c_str());
    if (ImGui::Button("Inspect position peak"))
    {
        m_SelectedFrame = static_cast<int>(std::round(results[m_SelectedBody].position.maximumTime / m_Results.GetStep())) - 1;
        m_FollowLatest = false;
    }
    ImGui::Text("Reference xyz (m): %.4f  %.4f  %.4f | Actual: %.4f  %.4f  %.4f",
        sample.reference.x, sample.reference.y, sample.reference.z, sample.actual.x, sample.actual.y, sample.actual.z);
    ImGui::Text("Actual - reference (cm): %.3f  %.3f  %.3f | Movement diff (cm/step): %.3f  %.3f  %.3f",
        sample.difference.x * 100, sample.difference.y * 100, sample.difference.z * 100,
        sample.movementDifference.x * 100, sample.movementDifference.y * 100, sample.movementDifference.z * 100);
    Plot("Position cm", frames, m_SelectedBody, 0);
    Plot("Movement cm/step", frames, m_SelectedBody, 1);
    Plot("Rotation deg", frames, m_SelectedBody, 2);
}

void RagdollComparisonRunner::DrawJointResults()
{
    const auto& results = m_Results.GetJoints();
    const auto& frames = m_Results.GetFrames();
    const auto& asset = m_Results.GetAsset();
    if (results.empty())
    {
        ImGui::TextUnformatted("No joints in this PA.");
        return;
    }
    ImGui::TextWrapped("Raw -> bounded measures target clamping. Bounded -> actual measures remaining tracking error. "
                       "A violation identifies an incompatible pose, not automatically an incorrect limit. Check axes, rest frames and source animation.");
    if (ImGui::BeginTable("Joint errors", 9, ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_ScrollY | ImGuiTableFlags_ScrollX | ImGuiTableFlags_Resizable, ImVec2(0, 240)))
    {
        const char* headers[] = {"Joint (clamp first)", "Clamp mean deg", "Clamp peak deg", "Track mean deg", "Track peak deg",
            "Twist bad %", "Swing Y bad %", "Swing Z bad %", "Cone bad %"};
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
    ImGui::Text("%s -> %s | %s | raw->actual: %.3f deg",
        config.parentBone.c_str(), config.childBone.c_str(), config.useSlerpDrive ? "SLERP" : "Swing / twist", sample.totalError * Degrees);
    if (ImGui::Button("Inspect clamp peak"))
    {
        m_SelectedFrame = static_cast<int>(std::round(result.clamp.maximumTime / m_Results.GetStep())) - 1;
        m_FollowLatest = false;
    }
    const PhysicsJointMotion motions[] = {config.twist, config.swingY, config.swingZ};
    const float minimum[] = {config.twistMin, -config.swingYLimit, -config.swingZLimit};
    const float maximum[] = {config.twistMax, config.swingYLimit, config.swingZLimit};
    if (ImGui::BeginTable("Axis values", 10, ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_ScrollX))
    {
        const char* headers[] = {"Axis", "Allowed deg", "Raw deg", "Bounded deg", "Actual deg",
            "Raw excess peak deg", "Bad total s", "Longest s", "First / last s", "Actual excess peak deg"};
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
                ImGui::TextUnformatted(config.swingY == PhysicsJointMotion::Limited && config.swingZ == PhysicsJointMotion::Limited ? "coupled ellipse" : "not applied");
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
    ImGui::TextDisabled("Cone excess is radial distance in swing-angle coordinates. Violation tolerance: 0.0001 rad.");
    for (int axis = 0; axis < 3; ++axis)
    {
        const auto& drive = config.drives[config.useSlerpDrive ? 0 : axis];
        ImGui::Text("%s: source range [%.2f, %.2f] deg | drive %s | Kp %.1f Kd %.1f max %.1f Nm",
            Axes[axis], result.minimumAngles[axis] * Degrees, result.maximumAngles[axis] * Degrees,
            drive.enabled ? "on" : "off", drive.stiffness, drive.damping, drive.maxTorque);
    }
    if (result.singularFrames)
    {
        ImGui::TextWrapped("Warning: %zu raw samples have a singular twist decomposition near 180-degree swing. "
                           "Their individual twist values are ambiguous; inspect joint frames.",
            result.singularFrames);
    }
    Plot("Raw -> bounded deg", frames, m_SelectedJoint, 3);
    Plot("Bounded -> actual deg", frames, m_SelectedJoint, 4);
    Plot("Raw -> actual deg", frames, m_SelectedJoint, 5);
}
