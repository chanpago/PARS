#include "SkeletalMeshViewer.h"
#include "Components/SkeletalMeshComponent.h"
#include "imgui.h"
#include <cmath>
#include <cstring>

using namespace Math;
using namespace PARS;

namespace
{
    bool EditMotion(const char* label, PhysicsJointMotion& motion)
    {
        int value = static_cast<int>(motion);
        ImGui::SetNextItemWidth(105);
        if (!ImGui::Combo(label, &value, "Locked\0Limited\0Free\0")) return false;
        motion = static_cast<PhysicsJointMotion>(value);
        return true;
    }

    bool EditAngle(const char* label, float& radians, float low, float high)
    {
        float degrees = XMConvertToDegrees(radians);
        ImGui::SetNextItemWidth(105);
        if (!ImGui::DragFloat(label, &degrees, .5f, low, high, "%.1f deg", ImGuiSliderFlags_AlwaysClamp)) return false;
        radians = XMConvertToRadians(degrees);
        return true;
    }

    bool EditFrame(PhysicsJointFrame& frame)
    {
        ImGui::TextUnformatted("Body-local offset (m)");
        ImGui::SetNextItemWidth(-1);
        bool changed = ImGui::DragFloat3("##JointOffset", &frame.offset.x, .002f, -100, 100, "%.3f", ImGuiSliderFlags_AlwaysClamp);
        const auto& q = frame.rotation;
        float xyz[] = {
            XMConvertToDegrees(std::atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y))),
            XMConvertToDegrees(std::asin(Clamp(2 * (q.w * q.y - q.z * q.x), -1.0f, 1.0f))),
            XMConvertToDegrees(std::atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))};
        ImGui::TextUnformatted("Frame rotation X / Y / Z (deg)");
        ImGui::SetNextItemWidth(-1);
        if (ImGui::DragFloat3("##JointRotation", xyz, .5f, -180, 180, "%.1f", ImGuiSliderFlags_AlwaysClamp))
        {
            XMStoreFloat4(&frame.rotation, Normalize(Quaternion(Vector3(kZUnitVector), XMConvertToRadians(xyz[2])) *
                Quaternion(Vector3(kYUnitVector), XMConvertToRadians(xyz[1])) * Quaternion(Vector3(kXUnitVector), XMConvertToRadians(xyz[0]))));
            changed = true;
        }
        return changed;
    }
}

int SkeletalMeshViewer::FindConstraintParent() const
{
    if (m_SelectedBody < 0 || m_SelectedBone < 0) return -1;
    const auto& model = *m_Mesh->GetSkeletalMesh();
    // Skip helper nodes and bones without bodies, rather than requiring every bone.
    for (int node = model.m_NodeParents[m_SelectedBone]; node >= 0; node = model.m_NodeParents[node])
    {
        const int body = m_PhysicsAsset.FindBody(model.m_NodeNames[node]);
        if (body >= 0) return body;
    }
    return -1;
}

Matrix4 SkeletalMeshViewer::GetConstraintWorld(int index, int frame) const
{
    const auto& joint = m_PhysicsAsset.constraints[index];
    const int body = m_PhysicsAsset.FindBody(frame == 0 ? joint.parentBone : joint.childBone);
    return body < 0 ? Matrix4(kIdentity) : GetBodyWorld(body) * (frame == 0 ? joint.parentFrame : joint.childFrame).GetTransform();
}

Matrix4 SkeletalMeshViewer::GetConstraintBodyWorld(int frame) const
{
    const auto& joint = m_PhysicsAsset.constraints[m_SelectedConstraint];
    return GetBodyWorld(m_PhysicsAsset.FindBody(frame == 0 ? joint.parentBone : joint.childBone));
}

void SkeletalMeshViewer::SelectConstraint(int index)
{
    CancelInteraction();
    m_EditPhysics = true;
    m_EditConstraints = true;
    m_SelectedConstraint = index;
    if (m_GizmoMode == 2) m_GizmoMode = 0;
    if (index < 0) return;
    const auto& joint = m_PhysicsAsset.constraints[index];
    m_SelectedBody = m_PhysicsAsset.FindBody(joint.childBone);
    m_SelectedBone = FindBodyNode(m_SelectedBody);
    m_ConstraintParent = m_PhysicsAsset.FindBody(joint.parentBone);
    m_RevealSelection = true;
}

void SkeletalMeshViewer::ResetConstraintFrames(PhysicsConstraint& joint)
{
    const int parent = m_PhysicsAsset.FindBody(joint.parentBone), child = m_PhysicsAsset.FindBody(joint.childBone);
    // Capsule X already follows the limb in PARS. Use that orientation, with
    // the pivot at the child bone origin. Mundi's fixed -90 degree rotations
    // would be wrong here (Y-up, column-vector Parent * Local convention).
    Matrix4 world = GetBodyWorld(child);
    world.SetW(m_NodeWorld[FindBodyNode(child)].GetW());
    joint.SetWorldFrame(GetBodyWorld(parent), GetBodyWorld(child), world);
}

void SkeletalMeshViewer::MoveOtherConstraintFrame(const Matrix4& previousWorld)
{
    auto& joint = m_PhysicsAsset.constraints[m_SelectedConstraint];
    const Matrix4 delta = GetConstraintWorld(m_SelectedConstraint, m_ConstraintFrame) * Invert(previousWorld);
    auto& other = m_ConstraintFrame == 0 ? joint.childFrame : joint.parentFrame;
    const Matrix4 otherBody = GetConstraintBodyWorld(1 - m_ConstraintFrame);
    // Apply the same world-space edit, preserving the anatomical bias between
    // the frames. Snapping the other frame to this one would erase asymmetric ROM.
    other = PhysicsJointFrame::FromTransform(Invert(otherBody) * delta * otherBody * other.GetTransform());
}

void SkeletalMeshViewer::CreateConstraint()
{
    if (m_SelectedBody < 0 || m_ConstraintParent < 0 || m_SelectedBody == m_ConstraintParent) return;
    CancelInteraction();
    const auto& parent = m_PhysicsAsset.bodies[m_ConstraintParent].bone;
    const auto& child = m_PhysicsAsset.bodies[m_SelectedBody].bone;
    const int existing = m_PhysicsAsset.FindConstraint(parent, child);
    if (existing >= 0) { SelectConstraint(existing); return; }
    auto& animation = m_Mesh->GetAnimInstance();
    animation.SetPlaying(false);
    if (m_EditPose.empty()) m_EditPose = animation.GetPose();
    PhysicsConstraint joint;
    joint.parentBone = parent; joint.childBone = child;
    ResetConstraintFrames(joint);
    m_PhysicsAsset.constraints.push_back(joint);
    SelectConstraint(static_cast<int>(m_PhysicsAsset.constraints.size()) - 1);
    m_ShowConstraints = true;
    m_PhysicsDirty = true;
    m_PhysicsMessage.clear();
}

void SkeletalMeshViewer::DrawConstraintPanel()
{
    ImGui::TextWrapped("Select a child bone with a body, then choose the parent body.");
    ImGui::BeginDisabled(m_SelectedBody < 0);
    const char* parentName = m_ConstraintParent >= 0 ? m_PhysicsAsset.bodies[m_ConstraintParent].bone.c_str() : "Choose parent body";
    ImGui::SetNextItemWidth(-1);
    if (ImGui::BeginCombo("##ParentBody", parentName))
    {
        for (int i = 0; i < static_cast<int>(m_PhysicsAsset.bodies.size()); ++i)
            if (i != m_SelectedBody && ImGui::Selectable(m_PhysicsAsset.bodies[i].bone.c_str(), i == m_ConstraintParent))
            { CancelInteraction(); m_ConstraintParent = i; }
        ImGui::EndCombo();
    }
    ImGui::BeginDisabled(m_ConstraintParent < 0 || m_ConstraintParent == m_SelectedBody);
    if (ImGui::Button("+ Connect bodies")) CreateConstraint();
    ImGui::EndDisabled();
    ImGui::EndDisabled();
    if (ImGui::BeginListBox("##Joints", ImVec2(-1, ImGui::GetTextLineHeightWithSpacing() * 4)))
    {
        for (int i = 0; i < static_cast<int>(m_PhysicsAsset.constraints.size()); ++i)
        {
            const auto& joint = m_PhysicsAsset.constraints[i];
            ImGui::PushID(i);
            const auto label = joint.parentBone + " -> " + joint.childBone;
            if (ImGui::Selectable(label.c_str(), i == m_SelectedConstraint)) SelectConstraint(i);
            if (ImGui::IsItemHovered()) ImGui::SetTooltip("%s", label.c_str());
            ImGui::PopID();
        }
        ImGui::EndListBox();
    }
    if (!EditingConstraint()) return;
    auto& joint = m_PhysicsAsset.constraints[m_SelectedConstraint];
    ImGui::TextWrapped("%s -> %s", joint.parentBone.c_str(), joint.childBone.c_str());
    if (ImGui::Button("Delete joint"))
    {
        CancelInteraction();
        m_PhysicsAsset.constraints.erase(m_PhysicsAsset.constraints.begin() + m_SelectedConstraint);
        m_SelectedConstraint = -1;
        m_PhysicsDirty = true;
        m_PhysicsMessage.clear();
        return;
    }
    bool changed = false;
    if (ImGui::RadioButton("Parent frame", m_ConstraintFrame == 0)) { CancelInteraction(); m_ConstraintFrame = 0; }
    ImGui::SameLine();
    if (ImGui::RadioButton("Child frame", m_ConstraintFrame == 1)) { CancelInteraction(); m_ConstraintFrame = 1; }
    bool moveBoth = m_MoveBothFrames;
    if (ImGui::Checkbox("Move both frames", &moveBoth)) { CancelInteraction(); m_MoveBothFrames = moveBoth; }
    ImGui::BeginDisabled(m_DragAxis >= 0);
    const Matrix4 previousWorld = GetConstraintWorld(m_SelectedConstraint, m_ConstraintFrame);
    if (EditFrame(m_ConstraintFrame == 0 ? joint.parentFrame : joint.childFrame))
    {
        if (m_MoveBothFrames) MoveOtherConstraintFrame(previousWorld);
        changed = true;
    }
    if (ImGui::Button("Align both to child bone")) { ResetConstraintFrames(joint); changed = true; }
    if (ImGui::IsItemHovered()) ImGui::SetTooltip("Replaces both frames, including any anatomical axis calibration and asymmetric ROM bias.");
    ImGui::TextDisabled("Translation locked | X twist, Y/Z swing");
    ImGui::TextWrapped("Swing values are cone half-angles about the parent frame. Its rotation can bias the range; zero need not be the reference pose. Move both frames preserves this bias.");
    changed |= EditMotion("Twist X", joint.twist);
    ImGui::BeginDisabled(joint.twist != PhysicsJointMotion::Limited);
    changed |= EditAngle("Minimum", joint.twistMin, -179.9f, XMConvertToDegrees(joint.twistMax) - .1f);
    changed |= EditAngle("Maximum", joint.twistMax, XMConvertToDegrees(joint.twistMin) + .1f, 179.9f);
    ImGui::EndDisabled();
    changed |= EditMotion("Swing Y", joint.swingY);
    ImGui::BeginDisabled(joint.swingY != PhysicsJointMotion::Limited);
    changed |= EditAngle("Y limit", joint.swingYLimit, .1f, 179.9f);
    ImGui::EndDisabled();
    changed |= EditMotion("Swing Z", joint.swingZ);
    ImGui::BeginDisabled(joint.swingZ != PhysicsJointMotion::Limited);
    changed |= EditAngle("Z limit", joint.swingZLimit, .1f, 179.9f);
    ImGui::EndDisabled();
    changed |= ImGui::Checkbox("Connected bodies collide", &joint.enableCollision);
    ImGui::EndDisabled();
    ImGui::TextWrapped("Red cone/arc: allowed swing. Green sector: allowed X twist. Yellow: current child direction / twist (swing removed). Both limits share the joint pivot. Orange: separated anchors.");
    ImGui::TextWrapped("Twist zero follows the body's front, the hand's palm, or the forearm/shin direction for elbow/knee hinges. The sector and needle show X rotation with swing removed.");
    if (changed) { m_PhysicsDirty = true; m_PhysicsMessage.clear(); }
}

Vector3 SkeletalMeshViewer::GetTwistDisplayReference(const PhysicsConstraint& joint)
{
    const auto colon = joint.childBone.find_last_of(':');
    const char* bone = joint.childBone.c_str() + (colon == std::string::npos ? 0 : colon + 1);

    // Match BuildMixamoPhysicsAsset's anatomical CHILD frames, not world axes:
    // -Z is front for the trunk/neck/head, shoulder/upper arm, thigh and foot;
    // for the hand it is the palm normal (down in the imported T-pose).
    // The same reference rotates the parent dial and the child twist needle.
    // It stays attached to the joint during animation and frame editing.
    for (const char* name : {"Spine", "Spine1", "Spine2", "Neck", "Head",
        "LeftShoulder", "RightShoulder", "LeftArm", "RightArm",
        "LeftHand", "RightHand",
        "LeftUpLeg", "RightUpLeg", "LeftFoot", "RightFoot"})
        if (std::strcmp(bone, name) == 0) return Vector3(0, 0, -1);

    // Hinge X is the elbow/knee flexion axis and Y follows the forearm/shin.
    // The green twist sector therefore follows the actual bending motion.
    if (std::strcmp(bone, "LeftLeg") == 0 || std::strcmp(bone, "RightLeg") == 0 ||
        std::strcmp(bone, "LeftForeArm") == 0 || std::strcmp(bone, "RightForeArm") == 0)
        return Vector3(kYUnitVector);

    // Unrecognized rigs retain the joint-local convention; do not guess their anatomy.
    return Vector3(kYUnitVector);
}

void SkeletalMeshViewer::DrawConstraints()
{
    if (!m_ShowConstraints && !m_ShowLimits) return;
    for (int i = 0; i < static_cast<int>(m_PhysicsAsset.constraints.size()); ++i)
    {
        const auto& joint = m_PhysicsAsset.constraints[i];
        const Matrix4 parent = GetConstraintWorld(i, 0), child = GetConstraintWorld(i, 1);
        const Vector3 pivot(parent.GetW()), childPivot(child.GetW());
        const uint32_t id = 0x600001 + i;
        const bool selected = EditingConstraint() && m_SelectedConstraint == i;
        const Vector3 color = selected ? Vector3(1, .7f, .12f) : Vector3(.55f, .6f, .65f);
        if (m_ShowConstraints)
        {
            m_Overlay.Line(Vector3(GetBodyWorld(m_PhysicsAsset.FindBody(joint.parentBone)).GetW()), pivot, color, id, 1);
            m_Overlay.Line(pivot, childPivot, Vector3(1, .5f, .1f), id, 3);
            m_Overlay.Line(childPivot, Vector3(GetBodyWorld(m_PhysicsAsset.FindBody(joint.childBone)).GetW()), color, id, 1);
            m_Overlay.Point(pivot, color, id, selected ? 6.0f : 3.0f);
            m_Overlay.Point(childPivot, color, id, selected ? 5.0f : 3.0f);
        }
        if (m_ShowLimits)
            m_Overlay.ConstraintLimits(joint, parent, child, id, selected, GetTwistDisplayReference(joint));
    }
}
