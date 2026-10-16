#include "SkeletalMeshViewer.h"
#include "Components/SkeletalMeshComponent.h"
#include "imgui.h"
#include <algorithm>
#include <cmath>

using namespace Math;
using namespace PARS;

int SkeletalMeshViewer::FindBodyNode(int index) const
{
    const auto& names = m_Mesh->GetSkeletalMesh()->m_NodeNames;
    const auto& bone = m_PhysicsAsset.bodies[index].bone;
    const auto it = std::find(names.begin(), names.end(), bone);
    return it == names.end() ? -1 : static_cast<int>(it - names.begin());
}

Matrix4 SkeletalMeshViewer::GetBodyWorld(int index) const
{
    const int node = FindBodyNode(index);
    return node < 0 ? Matrix4(kIdentity) : PhysicsBoneFrame(m_NodeWorld[node]) * m_PhysicsAsset.bodies[index].GetLocalTransform();
}

Matrix4 SkeletalMeshViewer::GetGizmoWorld() const
{
    return EditingBody() ? GetBodyWorld(m_SelectedBody) : m_NodeWorld[m_SelectedBone];
}

void SkeletalMeshViewer::SelectBody(int index)
{
    CancelInteraction();
    m_EditPhysics = true;
    m_SelectedBody = index;
    m_SelectedBone = index < 0 ? -1 : FindBodyNode(index);
    m_RevealSelection = true;
}

void SkeletalMeshViewer::CreatePhysicsBody(PhysicsShapeType shape)
{
    if (m_SelectedBone < 0) return;
    CancelInteraction();
    const auto& names = m_Mesh->GetSkeletalMesh()->m_NodeNames;
    const int existing = m_PhysicsAsset.FindBody(names[m_SelectedBone]);
    if (existing >= 0) { SelectBody(existing); return; }
    auto& animation = m_Mesh->GetAnimInstance();
    animation.SetPlaying(false);
    if (m_EditPose.empty()) m_EditPose = animation.GetPose();
    PhysicsBody body;
    body.bone = names[m_SelectedBone];
    body.shape = shape;
    const Matrix4 frame = PhysicsBoneFrame(m_NodeWorld[m_SelectedBone]);
    Vector3 endpoint(.15f, 0, 0);
    // Fit to the nearest child joint; leaf joints get a small editable body.
    float shortest = 1e10f;
    for (uint32_t i = 0; i < m_BoneParents.size(); ++i)
        if (m_IsJoint[i] && m_BoneParents[i] == m_SelectedBone)
        {
            const Vector3 local(Invert(frame) * Vector3(m_NodeWorld[i].GetW()));
            const float length = Length(local);
            if (length > .005f && length < shortest) { shortest = length; endpoint = local; }
        }
    const float length = Length(endpoint);
    const Vector3 axis = endpoint / length;
    const float dot = Clamp(float(axis.GetX()), -1.0f, 1.0f);
    Quaternion rotation(kIdentity);
    if (dot < -.9999f) rotation = Quaternion(Vector3(kYUnitVector), XM_PI);
    else if (dot < .9999f) rotation = Quaternion(Normalize(Cross(Vector3(kXUnitVector), axis)), std::acos(dot));
    XMStoreFloat4(&body.rotation, rotation);
    XMStoreFloat3(&body.offset, endpoint * .5f);
    body.radius = Clamp(length * .22f, .015f, .18f);
    body.halfLength = Max(0.0f, length * .5f - body.radius);
    body.halfExtents = XMFLOAT3(length * .5f, body.radius, body.radius);
    m_PhysicsAsset.bodies.push_back(body);
    SelectBody(static_cast<int>(m_PhysicsAsset.bodies.size()) - 1);
    m_ShowBodies = true;
    m_PhysicsDirty = true;
    m_PhysicsMessage.clear();
}

void SkeletalMeshViewer::DrawPhysicsBodies()
{
    if (!m_ShowBodies) return;
    std::vector<int> sorted;
    for (size_t i = 0; i < m_PhysicsAsset.bodies.size(); ++i)
        if (FindBodyNode(static_cast<int>(i)) >= 0) sorted.push_back(static_cast<int>(i));
    // Convex surfaces are backface culled; sort bodies far-to-near for alpha.
    std::sort(sorted.begin(), sorted.end(), [&](int a, int b) {
        return float(Dot(Vector3(GetBodyWorld(a).GetW()) - m_Camera.GetPosition(), m_Camera.GetForwardVec())) >
               float(Dot(Vector3(GetBodyWorld(b).GetW()) - m_Camera.GetPosition(), m_Camera.GetForwardVec()));
    });
    for (int index : sorted)
    {
        const bool selected = EditingBody() && index == m_SelectedBody;
        m_Overlay.Body(m_PhysicsAsset.bodies[index], GetBodyWorld(index),
            selected ? Vector3(1, .68f, .12f) : Vector3(.2f, .7f, 1),
            0x400001 + index, m_BodyOpacity);
    }
}

void SkeletalMeshViewer::DrawPhysicsPanel()
{
    ImGui::Text("PhysicsAsset | %zu bodies%s", m_PhysicsAsset.bodies.size(), m_PhysicsDirty ? " *" : "");
    ImGui::SetNextItemWidth(-1);
    ImGui::InputText("##PhysicsAssetPath", m_PhysicsPath, sizeof(m_PhysicsPath));
    if (ImGui::IsItemHovered()) ImGui::SetTooltip("PhysicsAsset JSON path (relative to the working directory or absolute)");
    if (ImGui::Button("Save PA"))
    {
        CancelInteraction();
        if (m_PhysicsAsset.Save(m_PhysicsPath, m_PhysicsMessage)) { m_PhysicsDirty = false; m_PhysicsMessage = "Saved."; }
    }
    ImGui::SameLine();
    bool load = false;
    if (ImGui::Button("Load PA"))
    {
        if (m_PhysicsDirty) ImGui::OpenPopup("Load and replace edits?");
        else load = true;
    }
    if (ImGui::BeginPopupModal("Load and replace edits?", nullptr, ImGuiWindowFlags_AlwaysAutoResize))
    {
        ImGui::TextUnformatted("Replace the unsaved PA edits with this file?");
        if (ImGui::Button("Load")) { load = true; ImGui::CloseCurrentPopup(); }
        ImGui::SameLine();
        if (ImGui::Button("Cancel")) ImGui::CloseCurrentPopup();
        ImGui::EndPopup();
    }
    if (load)
    {
        CancelInteraction();
        PhysicsAsset candidate;
        if (candidate.Load(m_PhysicsPath, m_PhysicsMessage))
        {
            const auto& model = *m_Mesh->GetSkeletalMesh();
            bool compatible = true;
            for (const auto& body : candidate.bodies)
            {
                const auto it = std::find(model.m_NodeNames.begin(), model.m_NodeNames.end(), body.bone);
                if (it == model.m_NodeNames.end() || !m_IsJoint[it - model.m_NodeNames.begin()]) { compatible = false; break; }
            }
            if (compatible)
            {
                m_PhysicsAsset = std::move(candidate);
                m_PhysicsDirty = false;
                m_PhysicsMessage = "Loaded.";
                SelectBone(m_SelectedBone);
                m_ShowBodies = true;
            }
            else m_PhysicsMessage = "File contains bones outside this skeleton.";
        }
    }
    if (!m_PhysicsMessage.empty()) ImGui::TextWrapped("%s", m_PhysicsMessage.c_str());
    ImGui::Separator();
    if (m_SelectedBone < 0) ImGui::TextWrapped("Select a bone in the hierarchy to add a body.");
    ImGui::BeginDisabled(m_SelectedBone < 0 || m_SelectedBody >= 0);
    if (ImGui::Button("+ Capsule")) CreatePhysicsBody(PhysicsShapeType::Capsule);
    ImGui::SameLine();
    if (ImGui::Button("+ Box")) CreatePhysicsBody(PhysicsShapeType::Box);
    ImGui::SameLine();
    if (ImGui::Button("+ Sphere")) CreatePhysicsBody(PhysicsShapeType::Sphere);
    ImGui::EndDisabled();
    if (!EditingBody()) return;
    auto& body = m_PhysicsAsset.bodies[m_SelectedBody];
    bool changed = false;
    ImGui::SetNextItemWidth(100);
    int shape = static_cast<int>(body.shape);
    if (ImGui::Combo("Shape", &shape, "Capsule\0Box\0Sphere\0")) { body.shape = static_cast<PhysicsShapeType>(shape); changed = true; }
    ImGui::TextUnformatted("Bone offset (m)");
    ImGui::SetNextItemWidth(-1);
    changed |= ImGui::DragFloat3("##BodyOffset", &body.offset.x, .002f, -100, 100, "%.3f", ImGuiSliderFlags_AlwaysClamp);
    if (body.shape == PhysicsShapeType::Box)
    {
        ImGui::TextUnformatted("Half extents X / Y / Z (m)");
        ImGui::SetNextItemWidth(-1);
        changed |= ImGui::DragFloat3("##HalfExtents", &body.halfExtents.x, .001f, .001f, 100, "%.3f", ImGuiSliderFlags_AlwaysClamp);
    }
    else
    {
        ImGui::SetNextItemWidth(110);
        changed |= ImGui::DragFloat("Radius (m)", &body.radius, .001f, .001f, 100, "%.3f", ImGuiSliderFlags_AlwaysClamp);
        if (body.shape == PhysicsShapeType::Capsule)
        {
            ImGui::SetNextItemWidth(110);
            changed |= ImGui::DragFloat("Half cylinder (m)", &body.halfLength, .001f, 0, 100, "%.3f", ImGuiSliderFlags_AlwaysClamp);
            ImGui::TextDisabled("X axis | Full length: %.3f m", 2 * (body.halfLength + body.radius));
        }
    }
    ImGui::SetNextItemWidth(110);
    changed |= ImGui::DragFloat("Mass (kg)", &body.mass, .1f, .001f, 10000, "%.2f", ImGuiSliderFlags_AlwaysClamp);
    if (ImGui::Button("Reset rotation")) { body.rotation = XMFLOAT4(0, 0, 0, 1); changed = true; }
    ImGui::SameLine();
    if (ImGui::Button("Delete body"))
    {
        CancelInteraction();
        m_PhysicsAsset.bodies.erase(m_PhysicsAsset.bodies.begin() + m_SelectedBody);
        m_SelectedBody = -1;
        changed = true;
    }
    if (changed) { m_PhysicsDirty = true; m_PhysicsMessage.clear(); }
}
