#include "SkeletalMeshViewer.h"
#include "Components/SkeletalMeshComponent.h"
#include "imgui.h"
#include <cmath>

using namespace Math;
namespace { constexpr uint32_t GizmoId = 0x800000, BodyId = 0x400000, ConstraintId = 0x600000; }

void SkeletalMeshViewer::BuildSkeleton()
{
    const auto& model = *m_Mesh->GetSkeletalMesh();
    const uint32_t count = model.m_NumNodes;
    m_BoneChildren.assign(count, {});
    m_BoneRoots.clear();
    m_IsJoint.assign(count, false);
    m_InSkeleton.assign(count, false);
    m_BoneParents.assign(count, -1);
    m_NodeWorld.assign(count, Matrix4(kIdentity));
    m_NodeMeshWorld.assign(count, Matrix4(kIdentity));
    for (uint32_t i = 0; i < model.m_NumJoints; ++i)
    {
        const uint32_t node = model.m_JointIndices[i];
        if (node >= count) continue;
        m_IsJoint[node] = true;
        for (int32_t ancestor = static_cast<int32_t>(node); ancestor >= 0; ancestor = model.m_NodeParents[ancestor])
            m_InSkeleton[ancestor] = true;
    }
    for (uint32_t i = 0; i < count; ++i)
    {
        if (!m_InSkeleton[i]) continue;
        const int32_t parent = model.m_NodeParents[i];
        if (parent < 0) m_BoneRoots.push_back(static_cast<int32_t>(i));
        else m_BoneChildren[parent].push_back(static_cast<int32_t>(i));
        int32_t boneParent = parent;
        while (boneParent >= 0 && !m_IsJoint[boneParent]) boneParent = model.m_NodeParents[boneParent];
        m_BoneParents[i] = boneParent;
    }
}

void SkeletalMeshViewer::DrawBoneTree(int32_t node)
{
    const auto& model = *m_Mesh->GetSkeletalMesh();
    const auto& fullName = model.m_NodeNames[node];
    const int body = m_PhysicsAsset.FindBody(fullName);
    if (m_PhysicsTreeOnly && body < 0)
    {
        for (int32_t child : m_BoneChildren[node]) DrawBoneTree(child);
        return;
    }
    std::vector<int> constraints;
    for (int i = 0; i < static_cast<int>(m_PhysicsAsset.constraints.size()); ++i)
        if (m_PhysicsAsset.constraints[i].childBone == fullName) constraints.push_back(i);
    bool reveal = false;
    if (m_RevealSelection)
        for (int32_t ancestor = m_SelectedBone; ancestor >= 0; ancestor = model.m_NodeParents[ancestor])
            if (ancestor == node) { reveal = true; break; }
    if (reveal) ImGui::SetNextItemOpen(true);
    ImGuiTreeNodeFlags flags = ImGuiTreeNodeFlags_OpenOnArrow | ImGuiTreeNodeFlags_SpanAvailWidth;
    const bool hasChildren = !m_BoneChildren[node].empty() || !constraints.empty();
    if (!hasChildren) flags |= ImGuiTreeNodeFlags_Leaf | ImGuiTreeNodeFlags_NoTreePushOnOpen;
    if (node == m_SelectedBone && !EditingConstraint()) flags |= ImGuiTreeNodeFlags_Selected;
    if (m_PhysicsTreeOnly || model.m_NodeParents[node] < 0 || m_BoneParents[node] < 0) flags |= ImGuiTreeNodeFlags_DefaultOpen;
    if (!m_IsJoint[node]) ImGui::PushStyleColor(ImGuiCol_Text, ImGui::GetStyleColorVec4(ImGuiCol_TextDisabled));
    const auto separator = fullName.find_last_of(':');
    const char* label = fullName.c_str() + (separator == std::string::npos ? 0 : separator + 1);
    const bool open = ImGui::TreeNodeEx(reinterpret_cast<void*>(static_cast<intptr_t>(node + 1)), flags,
        "%s%s%s", label, m_IsJoint[node] ? "" : " (root)", body >= 0 ? " [Body]" : "");
    if (!m_IsJoint[node]) ImGui::PopStyleColor();
    if (ImGui::IsItemHovered()) ImGui::SetTooltip("%s\nNode %d", fullName.c_str(), node);
    if (m_IsJoint[node] && ImGui::IsItemClicked() && !ImGui::IsItemToggledOpen())
    {
        if (body >= 0 && m_EditPhysics) SelectBody(body); else SelectBone(node);
    }
    if (reveal && node == m_SelectedBone && !EditingConstraint()) ImGui::SetScrollHereY(.5f);
    if (open && hasChildren)
    {
        for (int index : constraints)
        {
            const auto& joint = m_PhysicsAsset.constraints[index];
            const auto colon = joint.parentBone.find_last_of(':');
            const std::string title = "[Joint] from " + joint.parentBone.substr(colon == std::string::npos ? 0 : colon + 1);
            ImGui::PushID(index + 0x600000);
            ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(.85f, .8f, .45f, 1));
            const bool selected = EditingConstraint() && m_SelectedConstraint == index;
            if (ImGui::Selectable(title.c_str(), selected)) SelectConstraint(index);
            if (ImGui::IsItemHovered()) ImGui::SetTooltip("%s -> %s", joint.parentBone.c_str(), joint.childBone.c_str());
            if (reveal && selected) ImGui::SetScrollHereY(.5f);
            ImGui::PopStyleColor();
            ImGui::PopID();
        }
        for (int32_t child : m_BoneChildren[node]) DrawBoneTree(child);
        ImGui::TreePop();
    }
}

void SkeletalMeshViewer::SelectBone(int32_t node)
{
    CancelInteraction();
    m_SelectedBone = node;
    m_SelectedBody = node >= 0 && m_EditPhysics ? m_PhysicsAsset.FindBody(m_Mesh->GetSkeletalMesh()->m_NodeNames[node]) : -1;
    m_SelectedConstraint = -1;
    m_ConstraintParent = FindConstraintParent();
    m_RevealSelection = true;
}

void SkeletalMeshViewer::CancelInteraction()
{
    if (m_DragAxis >= 0)
    {
        if (EditingConstraint()) { m_PhysicsAsset.constraints[m_SelectedConstraint] = m_DragConstraintOriginal; m_PhysicsDirty = m_DragWasDirty; }
        else if (EditingBody()) { m_PhysicsAsset.bodies[m_SelectedBody] = m_DragBodyOriginal; m_PhysicsDirty = m_DragWasDirty; }
        else if (m_SelectedBone >= 0 && !m_EditPose.empty()) m_EditPose[m_SelectedBone] = m_DragOriginal;
    }
    m_DragAxis = -1;
    m_Flying = false;
    m_HasPickRequest = false;
    m_PickPose.clear();
    m_Overlay.CancelPick();
}

void SkeletalMeshViewer::UpdateBoneWorld()
{
    const auto& model = *m_Mesh->GetSkeletalMesh();
    const auto& instance = m_Mesh->GetModelInstance();
    std::vector<bool> assigned(model.m_NumNodes, false);
    // Follow the SAME mesh * animated-joint space composition as DefaultVS.
    // In particular, the armature's .01 scale must not be applied twice.
    const uint8_t* meshData = model.m_MeshData.get();
    for (uint32_t i = 0; i < model.m_NumMeshes; ++i)
    {
        const Mesh& mesh = *reinterpret_cast<const Mesh*>(meshData);
        Matrix4 meshWorld(kIdentity);
        // matrixIdx and node index are aligned by the importer.
        if (mesh.numJoints && instance.GetNodeTransform(mesh.meshCBV, meshWorld))
            for (uint32_t j = mesh.startJoint; j < uint32_t(mesh.startJoint) + mesh.numJoints && j < model.m_NumJoints; ++j)
                for (int32_t node = model.m_JointIndices[j]; node >= 0 && !assigned[node]; node = model.m_NodeParents[node])
                {
                    Matrix4 joint;
                    if (!instance.GetNodeTransform(node, joint)) break;
                    m_NodeWorld[node] = meshWorld * joint;
                    m_NodeMeshWorld[node] = meshWorld;
                    assigned[node] = true;
                }
        meshData += sizeof(Mesh) + (mesh.numDraws - 1) * sizeof(Mesh::Draw);
    }
}

Vector3 SkeletalMeshViewer::GetBoneAxis(int axis) const
{
    const auto matrix = GetGizmoWorld();
    const Vector3 direction = axis == 0 ? Vector3(matrix.GetX()) : axis == 1 ? Vector3(matrix.GetY()) : Vector3(matrix.GetZ());
    const float length = Length(direction);
    return length > 1e-6f ? direction / length : Vector3(axis == 0 ? 1.0f : 0, axis == 1 ? 1.0f : 0, axis == 2 ? 1.0f : 0);
}

float SkeletalMeshViewer::GetGizmoLength() const
{
    const Vector3 center(GetGizmoWorld().GetW());
    const float depth = Dot(center - m_Camera.GetPosition(), m_Camera.GetForwardVec());
    return Max(depth, m_Radius * .01f) * (2.0f * std::tan(XM_PI / 8)) * 85.0f / Max(m_ViewportHeight, 1.0f);
}

void SkeletalMeshViewer::DrawSkeleton()
{
    const auto& model = *m_Mesh->GetSkeletalMesh();
    if (m_ShowBones)
    {
        for (uint32_t node = 0; node < model.m_NumNodes; ++node)
        {
            if (!m_IsJoint[node] || m_BoneParents[node] < 0) continue;
            const bool selected = static_cast<int32_t>(node) == m_SelectedBone;
            m_Overlay.Line(Vector3(m_NodeWorld[m_BoneParents[node]].GetW()), Vector3(m_NodeWorld[node].GetW()),
                selected ? Vector3(1, .72f, .12f) : Vector3(.2f, .8f, .65f), node + 1, selected ? 4.0f : 2.5f);
        }
        for (uint32_t node = 0; node < model.m_NumNodes; ++node)
            if (m_IsJoint[node])
                m_Overlay.Point(Vector3(m_NodeWorld[node].GetW()), Vector3(.8f, .95f, .95f), node + 1, 3.5f);
    }
    DrawConstraints();
    if (m_SelectedBone < 0) return;
    if (m_EditPhysics && !EditingBody() && !EditingConstraint()) return;
    if (EditingBody() && !m_ShowBodies) return;
    if (EditingConstraint() && !m_ShowConstraints && !m_ShowLimits) return;
    const Vector3 center(GetGizmoWorld().GetW());
    m_Overlay.Point(center, Vector3(1, .72f, .12f), EditingConstraint() ? ConstraintId + m_SelectedConstraint + 1 :
        EditingBody() ? BodyId + m_SelectedBody + 1 : m_SelectedBone + 1, 5);
    const float length = GetGizmoLength();
    for (int axis = 0; axis < 3; ++axis)
    {
        const uint32_t id = GizmoId + axis + 1;
        Vector3 color = axis == 0 ? Vector3(1, .25f, .25f) : axis == 1 ? Vector3(.35f, 1, .35f) : Vector3(.3f, .55f, 1);
        if (axis == m_DragAxis) color = Vector3(1, .9f, .15f);
        if (m_GizmoMode == 1)
        {
            const Vector3 u = GetBoneAxis((axis + 1) % 3) * length;
            const Vector3 v = GetBoneAxis((axis + 2) % 3) * length;
            constexpr int segments = 64;
            for (int i = 0; i < segments; ++i)
            {
                const float a = i * XM_2PI / segments, b = (i + 1) * XM_2PI / segments;
                m_Overlay.Line(center + u * std::cos(a) + v * std::sin(a),
                    center + u * std::cos(b) + v * std::sin(b), color, id, 4);
            }
        }
        else
        {
            const Vector3 direction = GetBoneAxis(axis), end = center + direction * length;
            m_Overlay.Line(center, end, color, id, 4);
            m_Overlay.Point(end, color, id, m_GizmoMode == 2 ? 7.0f : 5.0f);
            if (m_GizmoMode == 0)
            {
                const Vector3 side = GetBoneAxis((axis + 1) % 3) * length * .09f;
                m_Overlay.Line(end, end - direction * length * .2f + side, color, id, 3);
                m_Overlay.Line(end, end - direction * length * .2f - side, color, id, 3);
            }
        }
    }
}

void SkeletalMeshViewer::HandlePick(uint32_t id)
{
    if (!m_HasPickRequest) return;
    m_HasPickRequest = false;
    if (id > GizmoId && id <= GizmoId + 3)
    {
        if (m_SelectedBone >= 0 && ImGui::IsMouseDown(ImGuiMouseButton_Left)) BeginGizmoDrag(id - GizmoId - 1);
    }
    else if (id > ConstraintId && id <= ConstraintId + m_PhysicsAsset.constraints.size()) SelectConstraint(static_cast<int>(id - ConstraintId - 1));
    else if (id > BodyId && id <= BodyId + m_PhysicsAsset.bodies.size()) SelectBody(static_cast<int>(id - BodyId - 1));
    else if (id > 0 && id <= m_IsJoint.size() && m_IsJoint[id - 1]) SelectBone(static_cast<int32_t>(id - 1));
    else SelectBone(-1);
}

void SkeletalMeshViewer::BeginGizmoDrag(uint32_t axis)
{
    auto& animation = m_Mesh->GetAnimInstance();
    animation.SetPlaying(false);
    if (m_EditPose.empty()) m_EditPose = m_PickPose.empty() ? animation.GetPose() : m_PickPose;
    m_DragAxis = static_cast<int>(axis);
    m_DragOriginal = m_EditPose[m_SelectedBone];
    if (EditingBody()) { m_DragBodyOriginal = m_PhysicsAsset.bodies[m_SelectedBody]; m_DragWasDirty = m_PhysicsDirty; }
    if (EditingConstraint()) { m_DragConstraintOriginal = m_PhysicsAsset.constraints[m_SelectedConstraint]; m_DragWasDirty = m_PhysicsDirty; }
    m_DragLength = GetGizmoLength();
    const Vector3 center(GetGizmoWorld().GetW()), direction = GetBoneAxis(axis);
    Vector3 endpoint;
    if (!m_Overlay.Project(center, m_DragCenter) || !m_Overlay.Project(center + direction * m_DragLength, endpoint))
    { m_DragAxis = -1; return; }
    m_DragScreenAxis = endpoint - m_DragCenter;
    const auto& model = *m_Mesh->GetSkeletalMesh();
    const int32_t parent = model.m_NodeParents[m_SelectedBone];
    const Matrix4 parentWorld = EditingConstraint() ? GetConstraintBodyWorld(m_ConstraintFrame) :
        EditingBody() ? PARS::PhysicsBoneFrame(m_NodeWorld[m_SelectedBone]) :
        (parent < 0 || m_DragOriginal.skeletonRoot) ? m_NodeMeshWorld[m_SelectedBone] : m_NodeWorld[parent];
    m_DragLocalDirection = Vector3(Invert(parentWorld) * Vector4(direction, 0));
    Vector3 u, v;
    if (m_Overlay.Project(center + GetBoneAxis((axis + 1) % 3) * m_DragLength, u) &&
        m_Overlay.Project(center + GetBoneAxis((axis + 2) % 3) * m_DragLength, v))
    { m_DragU = u - m_DragCenter; m_DragV = v - m_DragCenter; }
    else { m_DragU = Vector3(kZero); m_DragV = Vector3(kZero); }
    m_DragAngle = 0;
    m_LastDragAngle = GetDragAngle(m_ClickX, m_ClickY);
}

float SkeletalMeshViewer::GetDragAngle(float x, float y) const
{
    const float ux = m_DragU.GetX(), uy = m_DragU.GetY(), vx = m_DragV.GetX(), vy = m_DragV.GetY();
    const float determinant = ux * vy - uy * vx;
    if (std::abs(determinant) < 8) return (x - m_ClickX - y + m_ClickY) * .01f;
    const float dx = x - m_DragCenter.GetX(), dy = y - m_DragCenter.GetY();
    return std::atan2((ux * dy - uy * dx) / determinant, (dx * vy - dy * vx) / determinant);
}

void SkeletalMeshViewer::UpdateGizmoDrag(float mouseX, float mouseY)
{
    if (m_DragAxis < 0) return;
    if (ImGui::IsKeyPressed(ImGuiKey_Escape)) { CancelInteraction(); return; }
    if (!ImGui::IsMouseDown(ImGuiMouseButton_Left)) { m_DragAxis = -1; return; }
    const float dx = mouseX - m_ClickX, dy = mouseY - m_ClickY;
    const float sx = m_DragScreenAxis.GetX(), sy = m_DragScreenAxis.GetY();
    const float denominator = sx * sx + sy * sy;
    const float amount = denominator > 36 ? (dx * sx + dy * sy) / denominator : -dy / 80;
    if (EditingConstraint())
    {
        auto& joint = m_PhysicsAsset.constraints[m_SelectedConstraint];
        joint = m_DragConstraintOriginal;
        auto& frame = m_ConstraintFrame == 0 ? joint.parentFrame : joint.childFrame;
        if (m_GizmoMode == 0)
            XMStoreFloat3(&frame.offset, Vector3(frame.offset) + m_DragLocalDirection * (amount * m_DragLength));
        else if (m_GizmoMode == 1)
        {
            const float angle = GetDragAngle(mouseX, mouseY);
            m_DragAngle += std::remainder(angle - m_LastDragAngle, XM_2PI);
            m_LastDragAngle = angle;
            const Vector3 axis(m_DragAxis == 0 ? 1.0f : 0, m_DragAxis == 1 ? 1.0f : 0, m_DragAxis == 2 ? 1.0f : 0);
            XMStoreFloat4(&frame.rotation, Normalize(Quaternion(XMLoadFloat4(&frame.rotation)) * Quaternion(axis, m_DragAngle)));
        }
        if (m_MoveBothFrames) MatchConstraintFrames();
        m_PhysicsDirty = true;
        return;
    }
    if (EditingBody())
    {
        auto& body = m_PhysicsAsset.bodies[m_SelectedBody];
        body = m_DragBodyOriginal;
        if (m_GizmoMode == 0)
            XMStoreFloat3(&body.offset, Vector3(body.offset) + m_DragLocalDirection * (amount * m_DragLength));
        else if (m_GizmoMode == 1)
        {
            const float angle = GetDragAngle(mouseX, mouseY);
            m_DragAngle += std::remainder(angle - m_LastDragAngle, XM_2PI);
            m_LastDragAngle = angle;
            const Vector3 axis(m_DragAxis == 0 ? 1.0f : 0, m_DragAxis == 1 ? 1.0f : 0, m_DragAxis == 2 ? 1.0f : 0);
            XMStoreFloat4(&body.rotation, Normalize(Quaternion(XMLoadFloat4(&body.rotation)) * Quaternion(axis, m_DragAngle)));
        }
        else
        {
            const float factor = std::exp(Clamp(amount, -5.0f, 5.0f));
            if (body.shape == PARS::PhysicsShapeType::Box) (&body.halfExtents.x)[m_DragAxis] = Clamp((&body.halfExtents.x)[m_DragAxis] * factor, .001f, 100.0f);
            else if (body.shape == PARS::PhysicsShapeType::Capsule && m_DragAxis == 0) body.halfLength = Clamp(Max(body.halfLength, .001f) * factor, 0.0f, 100.0f);
            else body.radius = Clamp(body.radius * factor, .001f, 100.0f);
        }
        m_PhysicsDirty = true;
        return;
    }
    auto& node = m_EditPose[m_SelectedBone];
    node = m_DragOriginal;
    if (m_GizmoMode == 0)
        node.xform.SetW(Vector4(Vector3(m_DragOriginal.xform.GetW()) + m_DragLocalDirection * (amount * m_DragLength), 1));
    else if (m_GizmoMode == 1)
    {
        const float angle = GetDragAngle(mouseX, mouseY);
        m_DragAngle += std::remainder(angle - m_LastDragAngle, XM_2PI);
        m_LastDragAngle = angle;
        const Vector3 axis(m_DragAxis == 0 ? 1.0f : 0, m_DragAxis == 1 ? 1.0f : 0, m_DragAxis == 2 ? 1.0f : 0);
        node.rotation = Normalize(m_DragOriginal.rotation * Quaternion(axis, m_DragAngle));
        node.staleMatrix = true;
    }
    else
    {
        float* scale = &node.scale.x;
        scale[m_DragAxis] = Clamp(scale[m_DragAxis] * std::exp(Clamp(amount, -5.0f, 5.0f)), .001f, 100.0f);
        node.staleMatrix = true;
    }
}
