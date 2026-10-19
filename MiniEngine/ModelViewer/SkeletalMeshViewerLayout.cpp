#include "SkeletalMeshViewer.h"
#include "Components/SkeletalMeshComponent.h"
#include "imgui.h"
#include <algorithm>
#include <cmath>

using namespace Math;

namespace
{
    std::string ShortBoneName(const std::string& name)
    {
        const auto separator = name.find_last_of(':');
        return name.substr(separator == std::string::npos ? 0 : separator + 1);
    }
}

void SkeletalMeshViewer::DrawOrientationAxes(float left, float top, float width, float height)
{
    if (width < 48 || height < 48)
    {
        return;
    }
    const float scale = Min(ImGui::GetFontSize() / 13.0f, Min(width, height) / 120.0f);
    const float length = 28 * scale, margin = 16 * scale, extent = 43 * scale;
    const ImVec2 center(left + margin + extent, top + height - margin - extent);
    struct Axis
    {
        Vector3 world;
        ImU32 color;
        const char* label;
        float depth;
    };
    Axis axes[] = {
        {Vector3(kXUnitVector), IM_COL32(235, 70, 70, 255), "X", 0},
        {Vector3(kYUnitVector), IM_COL32(85, 220, 90, 255), "Y", 0},
        {Vector3(kZUnitVector), IM_COL32(75, 145, 255, 255), "Z", 0}};
    // Project world directions onto the camera basis, using rotation only.
    // PARS is Y-up. Camera movement, FOV and zoom must not move/resize this widget.
    for (auto& axis : axes)
    {
        axis.depth = Dot(axis.world, m_Camera.GetForwardVec());
    }
    std::sort(std::begin(axes), std::end(axes), [](const Axis& a, const Axis& b) {
        return a.depth > b.depth;
    });
    auto* draw = ImGui::GetWindowDrawList();
    draw->PushClipRect(ImVec2(left, top), ImVec2(left + width, top + height), true);
    constexpr ImU32 shadow = IM_COL32(0, 0, 0, 210);
    draw->AddCircleFilled(center, 3 * scale, IM_COL32(220, 225, 230, 255));
    for (const auto& axis : axes)
    {
        const float dx = Dot(axis.world, m_Camera.GetRightVec());
        const float dy = -float(Dot(axis.world, m_Camera.GetUpVec())); // Screen Y points down.
        const float projectedLength = std::sqrt(dx * dx + dy * dy);
        const ImVec2 end(center.x + dx * length, center.y + dy * length);
        ImVec2 label;
        if (projectedLength > .15f)
        {
            const ImVec2 direction(dx / projectedLength, dy / projectedLength);
            draw->AddLine(center, end, shadow, 4 * scale);
            draw->AddLine(center, end, axis.color, 2 * scale);
            const float head = Min(5 * scale, projectedLength * length * .4f);
            draw->AddTriangleFilled(end,
                ImVec2(end.x - direction.x * head - direction.y * head * .5f, end.y - direction.y * head + direction.x * head * .5f),
                ImVec2(end.x - direction.x * head + direction.y * head * .5f, end.y - direction.y * head - direction.x * head * .5f), axis.color);
            label = ImVec2(end.x + direction.x * 9 * scale, end.y + direction.y * 9 * scale);
        }
        else
        {
            // Looking along an axis: keep its end and letter visible without
            // normalizing a near-zero screen direction into an unstable arrow.
            draw->AddCircleFilled(end, 4 * scale, shadow);
            draw->AddCircleFilled(end, 2.5f * scale, axis.color);
            label = ImVec2(end.x + 9 * scale, end.y + 9 * scale);
        }
        const float fontSize = 13 * scale;
        const ImVec2 textSize = ImGui::GetFont()->CalcTextSizeA(fontSize, 100, 0, axis.label);
        label.x -= textSize.x * .5f;
        label.y -= textSize.y * .5f;
        draw->AddText(ImGui::GetFont(), fontSize, ImVec2(label.x + scale, label.y + scale), shadow, axis.label);
        draw->AddText(ImGui::GetFont(), fontSize, label, axis.color, axis.label);
    }
    draw->PopClipRect();
}

void SkeletalMeshViewer::DrawNavigation()
{
    const float height = ImGui::GetContentRegionAvail().y;
    ImGui::BeginChild("SkeletonTree", ImVec2(0, Max(80.0f, height * m_HierarchyFraction)), true);
    ImGui::TextUnformatted("Skeleton Tree");
    ImGui::SameLine();
    if (ImGui::Checkbox("Bodies only", &m_PhysicsTreeOnly))
    {
        m_RevealSelection = true;
    }
    ImGui::Separator();
    ImGui::BeginChild("BoneTree", ImVec2(0, 0), false, ImGuiWindowFlags_HorizontalScrollbar);
    ImGui::PushStyleVar(ImGuiStyleVar_IndentSpacing, 13.0f);
    for (int32_t root : m_BoneRoots)
    {
        DrawBoneTree(root);
    }
    ImGui::PopStyleVar();
    m_RevealSelection = false;
    ImGui::EndChild();
    ImGui::EndChild();

    ImGui::InvisibleButton("##HierarchyGraphSplitter", ImVec2(-1, 5));
    if (ImGui::IsItemHovered() || ImGui::IsItemActive())
    {
        ImGui::SetMouseCursor(ImGuiMouseCursor_ResizeNS);
    }
    if (ImGui::IsItemActive())
    {
        m_HierarchyFraction = Clamp(m_HierarchyFraction + ImGui::GetIO().MouseDelta.y / Max(height, 1.0f), .25f, .75f);
    }
    ImGui::BeginChild("Relationships", ImVec2(0, 0), true, ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);
    DrawRelationshipGraph();
    ImGui::EndChild();
}

void SkeletalMeshViewer::DrawDetails()
{
    ImGui::TextUnformatted("Details");
    ImGui::Separator();
    int target = m_EditPhysics ? 1 : 0;
    ImGui::SetNextItemWidth(-1);
    if (ImGui::Combo("##EditTarget", &target, "Bone pose\0Physics asset\0"))
    {
        CancelInteraction();
        m_EditPhysics = target == 1;
        SelectBone(m_SelectedBone);
        if (m_EditPhysics)
        {
            if (m_EditConstraints && m_GizmoMode == 2)
            {
                m_GizmoMode = 0;
            }
            m_Mesh->GetAnimInstance().SetPlaying(false);
            const auto& model = *m_Mesh->GetSkeletalMesh();
            m_EditPose.assign(model.m_SceneGraph.get(), model.m_SceneGraph.get() + model.m_NumNodes);
        }
    }
    if (m_SelectedBone >= 0)
    {
        ImGui::TextWrapped("%s", m_Mesh->GetSkeletalMesh()->m_NodeNames[m_SelectedBone].c_str());
    }
    if (m_EditPhysics)
    {
        DrawPhysicsPanel();
    }
    else if (m_SelectedBone >= 0)
    {
        const auto& pose = m_EditPose.empty() ? m_Mesh->GetAnimInstance().GetPose() : m_EditPose;
        const auto& node = pose[m_SelectedBone];
        ImGui::TextUnformatted("Local position / scale");
        ImGui::Text("%.2f  %.2f  %.2f", float(node.xform.GetW().GetX()), float(node.xform.GetW().GetY()), float(node.xform.GetW().GetZ()));
        ImGui::Text("%.2f  %.2f  %.2f", node.scale.x, node.scale.y, node.scale.z);
        ImGui::TextWrapped("Use the viewport gizmo to edit the bone pose. Play restores animation.");
    }
    else
    {
        ImGui::TextWrapped("Select a bone, body or constraint in the tree, graph or viewport.");
    }
}

void SkeletalMeshViewer::DrawRelationshipGraph()
{
    // Rebuild from the PA each frame: create/delete/load cannot leave stale
    // node pointers or connections. Selection is shared with tree and viewport.
    struct Node
    {
        bool joint;
        int index;
        float x, y, width;
        const char* role;
    };
    struct Link
    {
        int a, b;
    };
    std::vector<Node> nodes;
    std::vector<Link> links;
    constexpr float bodyWidth = 104, jointWidth = 132, step = 78, nodeHeight = 62;
    constexpr float middle = bodyWidth + 22, right = middle + jointWidth + 22;
    int body = m_SelectedBody;
    if (body < 0 && m_SelectedBone >= 0)
    {
        body = m_PhysicsAsset.FindBody(m_Mesh->GetSkeletalMesh()->m_NodeNames[m_SelectedBone]);
    }
    const int selectedJoint = EditingConstraint() ? m_SelectedConstraint : -1;
    const int selection = selectedJoint >= 0 ? 0x600001 + selectedJoint : body >= 0 ? 0x400001 + body
                                                                                    : -1;
    if (selection != m_GraphSelection)
    {
        m_GraphSelection = selection;
        m_GraphZoom = 1;
        m_GraphPanX = m_GraphPanY = 0;
    }
    if (selectedJoint >= 0)
    {
        const auto& joint = m_PhysicsAsset.constraints[selectedJoint];
        const int parent = m_PhysicsAsset.FindBody(joint.parentBone), child = m_PhysicsAsset.FindBody(joint.childBone);
        if (parent >= 0 && child >= 0)
        {
            nodes = {{false, parent, 0, 0, bodyWidth, "Parent"},
                {true, selectedJoint, middle, 0, jointWidth, "Constraint"},
                {false, child, right, 0, bodyWidth, "Child"}};
            links = {{0, 1}, {1, 2}};
        }
    }
    else if (body >= 0 && body < static_cast<int>(m_PhysicsAsset.bodies.size()))
    {
        const auto& name = m_PhysicsAsset.bodies[body].bone;
        nodes.push_back({false, body, 0, 0, bodyWidth, "Body"});
        int row = 0;
        for (int i = 0; i < static_cast<int>(m_PhysicsAsset.constraints.size()); ++i)
        {
            const auto& joint = m_PhysicsAsset.constraints[i];
            if (joint.parentBone != name && joint.childBone != name)
            {
                continue;
            }
            const bool outgoing = joint.parentBone == name;
            const int other = m_PhysicsAsset.FindBody(outgoing ? joint.childBone : joint.parentBone);
            if (other < 0)
            {
                continue;
            }
            const int jointNode = static_cast<int>(nodes.size());
            nodes.push_back({true, i, middle, row * step, jointWidth, "Constraint"});
            nodes.push_back({false, other, right, row * step, bodyWidth, outgoing ? "Child" : "Parent"});
            links.push_back({0, jointNode});
            links.push_back({jointNode, jointNode + 1});
            ++row;
        }
        nodes[0].y = Max(0.0f, (row - 1) * step * .5f);
    }

    ImGui::TextUnformatted("Graph");
    ImGui::SameLine();
    if (ImGui::SmallButton("Fit"))
    {
        m_GraphZoom = 1;
        m_GraphPanX = m_GraphPanY = 0;
    }
    ImGui::SameLine();
    ImGui::TextDisabled("%zu constraints", links.size() / 2);
    if (ImGui::IsItemHovered())
    {
        ImGui::SetTooltip("Each connection is Body - Constraint - Body.\nWheel: zoom | MMB drag: pan | Click a node: select");
    }
    ImGui::BeginChild("GraphCanvas", ImVec2(0, 0), false, ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);
    const ImVec2 origin = ImGui::GetCursorScreenPos(), size = ImGui::GetContentRegionAvail();
    if (size.x < 1 || size.y < 1)
    {
        ImGui::EndChild();
        return;
    }
    auto* draw = ImGui::GetWindowDrawList();
    draw->PushClipRect(origin, ImVec2(origin.x + size.x, origin.y + size.y), true);
    draw->AddRectFilled(origin, ImVec2(origin.x + size.x, origin.y + size.y), IM_COL32(23, 26, 30, 255));
    for (float x = std::fmod(m_GraphPanX, 24.0f); x < size.x; x += 24)
    {
        draw->AddLine(ImVec2(origin.x + x, origin.y), ImVec2(origin.x + x, origin.y + size.y), IM_COL32(41, 45, 50, 255));
    }
    for (float y = std::fmod(m_GraphPanY, 24.0f); y < size.y; y += 24)
    {
        draw->AddLine(ImVec2(origin.x, origin.y + y), ImVec2(origin.x + size.x, origin.y + y), IM_COL32(41, 45, 50, 255));
    }
    if (nodes.empty())
    {
        ImGui::TextWrapped(body < 0 && m_SelectedBone >= 0 ? "This bone has no physics body." : "Select a body or constraint to see its connections.");
        draw->PopClipRect();
        ImGui::EndChild();
        return;
    }
    float contentWidth = 0, contentHeight = 0;
    for (const auto& node : nodes)
    {
        contentWidth = Max(contentWidth, node.x + node.width);
        contentHeight = Max(contentHeight, node.y + nodeHeight);
    }
    const float fit = Max(.05f, Min(1.0f, Min((size.x - 16) / contentWidth, (size.y - 16) / contentHeight)));
    auto& io = ImGui::GetIO();
    if (ImGui::IsWindowHovered())
    {
        if (ImGui::IsMouseDragging(ImGuiMouseButton_Middle))
        {
            m_GraphPanX += io.MouseDelta.x;
            m_GraphPanY += io.MouseDelta.y;
        }
        if (io.MouseWheel != 0)
        {
            const float zoom = Clamp(m_GraphZoom * std::pow(1.15f, io.MouseWheel), .35f, 4.0f), ratio = zoom / m_GraphZoom;
            const float x = io.MousePos.x - origin.x - size.x * .5f, y = io.MousePos.y - origin.y - size.y * .5f;
            m_GraphPanX = x - (x - m_GraphPanX) * ratio;
            m_GraphPanY = y - (y - m_GraphPanY) * ratio;
            m_GraphZoom = zoom;
        }
    }
    const float scale = fit * m_GraphZoom;
    const auto screen = [&](float x, float y) {
        return ImVec2(origin.x + (size.x - contentWidth * scale) * .5f + m_GraphPanX + x * scale,
            origin.y + (size.y - contentHeight * scale) * .5f + m_GraphPanY + y * scale);
    };
    for (const auto& link : links)
    {
        const auto& a = nodes[link.a];
        const auto& b = nodes[link.b];
        const ImVec2 start = screen(a.x + a.width, a.y + nodeHeight * .5f), end = screen(b.x, b.y + nodeHeight * .5f);
        const float bend = Max(12.0f, (end.x - start.x) * .5f);
        draw->AddBezierCubic(start, ImVec2(start.x + bend, start.y), ImVec2(end.x - bend, end.y), end,
            IM_COL32(208, 215, 204, 255), Max(1.0f, scale * 2));
    }
    int clickedBody = -1, clickedJoint = -1;
    for (int i = 0; i < static_cast<int>(nodes.size()); ++i)
    {
        const auto& node = nodes[i];
        const bool selected = node.joint ? node.index == selectedJoint : selectedJoint < 0 && node.index == body;
        const ImVec2 a = screen(node.x, node.y), b = screen(node.x + node.width, node.y + nodeHeight);
        ImGui::SetCursorScreenPos(a);
        ImGui::PushID(i);
        if (ImGui::InvisibleButton("Node", ImVec2(b.x - a.x, b.y - a.y)))
        {
            if (node.joint)
            {
                clickedJoint = node.index;
            }
            else
            {
                clickedBody = node.index;
            }
        }
        const bool hovered = ImGui::IsItemHovered();
        ImGui::PopID();
        draw->AddRectFilled(a, b, node.joint ? IM_COL32(99, 91, 49, 255) : IM_COL32(54, 87, 60, 255), 4);
        draw->AddRect(a, b, selected ? IM_COL32(255, 201, 66, 255) : hovered ? IM_COL32(235, 240, 230, 255)
                                                                             : IM_COL32(112, 128, 107, 255),
            4, 0, selected ? 2.5f : 1.0f);
        draw->AddCircleFilled(ImVec2(a.x, (a.y + b.y) * .5f), Max(2.0f, 4 * scale), IM_COL32(225, 232, 216, 255));
        draw->AddCircleFilled(ImVec2(b.x, (a.y + b.y) * .5f), Max(2.0f, 4 * scale), IM_COL32(225, 232, 216, 255));
        std::string name, detail, tooltip;
        if (node.joint)
        {
            const auto& joint = m_PhysicsAsset.constraints[node.index];
            name = ShortBoneName(joint.childBone);
            detail = "from " + ShortBoneName(joint.parentBone);
            tooltip = joint.parentBone + " -> " + joint.childBone;
        }
        else
        {
            const auto& entry = m_PhysicsAsset.bodies[node.index];
            name = ShortBoneName(entry.bone);
            detail = entry.shape == PARS::PhysicsShapeType::Capsule ? "Capsule" : entry.shape == PARS::PhysicsShapeType::Box ? "Box"
                                                                                                                             : "Sphere";
            tooltip = entry.bone;
        }
        const ImVec4 clip(a.x + 5 * scale, a.y, b.x - 5 * scale, b.y);
        const auto text = [&](float y, const char* label, ImU32 color) {
            const float fontSize = ImGui::GetFontSize() * scale * Min(.9f, (node.width - 14) / Max(1.0f, ImGui::CalcTextSize(label).x));
            draw->AddText(ImGui::GetFont(), fontSize, ImVec2(a.x + 7 * scale, a.y + y * scale), color, label, nullptr, 0, &clip);
        };
        text(5, node.role, IM_COL32(219, 225, 202, 255));
        text(23, name.c_str(), IM_COL32(255, 255, 245, 255));
        text(41, detail.c_str(), IM_COL32(205, 214, 194, 255));
        if (hovered)
        {
            ImGui::SetTooltip("%s\nClick to select", tooltip.c_str());
        }
    }
    draw->PopClipRect();
    ImGui::SetCursorScreenPos(origin);
    ImGui::Dummy(size);
    ImGui::EndChild();
    if (clickedJoint >= 0)
    {
        SelectConstraint(clickedJoint);
    }
    else if (clickedBody >= 0)
    {
        SelectBody(clickedBody);
    }
}
