#include "SkeletalMeshViewer.h"
#include "Components/SkeletalMeshComponent.h"
#include "ViewerLabels.h"
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
        if (!ImGui::Combo(label, &value, u8"잠금\0제한\0자유\0"))
        {
            return false;
        }
        motion = static_cast<PhysicsJointMotion>(value);
        return true;
    }

    bool EditAngle(const char* label, float& radians, float low, float high)
    {
        float degrees = XMConvertToDegrees(radians);
        ImGui::SetNextItemWidth(105);
        if (!ImGui::DragFloat(label, &degrees, .5f, low, high, u8"%.1f도", ImGuiSliderFlags_AlwaysClamp))
        {
            return false;
        }
        radians = XMConvertToRadians(degrees);
        return true;
    }

    bool EditFrame(PhysicsJointFrame& frame)
    {
        ImGui::TextUnformatted(u8"바디 기준 위치 (m)");
        ImGui::SetNextItemWidth(-1);
        bool changed = ImGui::DragFloat3("##JointOffset", &frame.offset.x, .002f, -100, 100, "%.3f", ImGuiSliderFlags_AlwaysClamp);
        const auto& q = frame.rotation;
        float xyz[] = {
            XMConvertToDegrees(std::atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y))),
            XMConvertToDegrees(std::asin(Clamp(2 * (q.w * q.y - q.z * q.x), -1.0f, 1.0f))),
            XMConvertToDegrees(std::atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))};
        ImGui::TextUnformatted(u8"기준 좌표계 회전 X / Y / Z (도)");
        ImGui::SetNextItemWidth(-1);
        if (ImGui::DragFloat3("##JointRotation", xyz, .5f, -180, 180, "%.1f", ImGuiSliderFlags_AlwaysClamp))
        {
            XMStoreFloat4(&frame.rotation, Normalize(Quaternion(Vector3(kZUnitVector), XMConvertToRadians(xyz[2])) * Quaternion(Vector3(kYUnitVector), XMConvertToRadians(xyz[1])) * Quaternion(Vector3(kXUnitVector), XMConvertToRadians(xyz[0]))));
            changed = true;
        }
        return changed;
    }
}

int SkeletalMeshViewer::FindConstraintParent() const
{
    if (m_SelectedBody < 0 || m_SelectedBone < 0)
    {
        return -1;
    }
    const auto& model = *m_Mesh->GetSkeletalMesh();
    // 모든 본을 요구하지 않고 보조 노드와 바디가 없는 본은 건너뛴다.
    for (int node = model.m_NodeParents[m_SelectedBone]; node >= 0; node = model.m_NodeParents[node])
    {
        const int body = m_PhysicsAsset.FindBody(model.m_NodeNames[node]);
        if (body >= 0)
        {
            return body;
        }
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
    if (m_GizmoMode == 2)
    {
        m_GizmoMode = 0;
    }
    if (index < 0)
    {
        return;
    }
    const auto& joint = m_PhysicsAsset.constraints[index];
    m_SelectedBody = m_PhysicsAsset.FindBody(joint.childBone);
    m_SelectedBone = FindBodyNode(m_SelectedBody);
    m_ConstraintParent = m_PhysicsAsset.FindBody(joint.parentBone);
    m_RevealSelection = true;
}

void SkeletalMeshViewer::ResetConstraintFrames(PhysicsConstraint& joint)
{
    const int parent = m_PhysicsAsset.FindBody(joint.parentBone), child = m_PhysicsAsset.FindBody(joint.childBone);
    // PARS의 캡슐 X축은 이미 팔다리를 따른다. 이 방향을 사용하고 피벗은 자식 본 원점에 둔다.
    // 여기에 Mundi의 고정 -90도 회전을 적용하면 맞지 않는다.
    // PARS는 Y가 위쪽이며, 열벡터에 Parent * Local 순서를 사용하는 규칙이기 때문이다.
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
    // 두 좌표계 사이의 해부학적 편향을 유지하면서 동일한 월드 공간 편집을 적용한다.
    // 반대쪽 좌표계를 이쪽에 그대로 맞추면 비대칭 가동 범위가 사라진다.
    other = PhysicsJointFrame::FromTransform(Invert(otherBody) * delta * otherBody * other.GetTransform());
}

void SkeletalMeshViewer::CreateConstraint()
{
    if (m_SelectedBody < 0 || m_ConstraintParent < 0 || m_SelectedBody == m_ConstraintParent)
    {
        return;
    }
    CancelInteraction();
    const auto& parent = m_PhysicsAsset.bodies[m_ConstraintParent].bone;
    const auto& child = m_PhysicsAsset.bodies[m_SelectedBody].bone;
    const int existing = m_PhysicsAsset.FindConstraint(parent, child);
    if (existing >= 0)
    {
        SelectConstraint(existing);
        return;
    }
    auto& animation = m_Mesh->GetAnimInstance();
    animation.SetPlaying(false);
    if (m_EditPose.empty())
    {
        m_EditPose = animation.GetPose();
    }
    PhysicsConstraint joint;
    joint.parentBone = parent;
    joint.childBone = child;
    ResetConstraintFrames(joint);
    m_PhysicsAsset.constraints.push_back(joint);
    SelectConstraint(static_cast<int>(m_PhysicsAsset.constraints.size()) - 1);
    m_ShowConstraints = true;
    m_PhysicsDirty = true;
    m_PhysicsMessage.clear();
}

void SkeletalMeshViewer::DrawConstraintPanel()
{
    ImGui::TextWrapped(u8"바디가 있는 자식 본을 선택한 뒤 부모 바디를 선택하세요.");
    ImGui::BeginDisabled(m_SelectedBody < 0);
    const char* parentName = m_ConstraintParent >= 0 ? m_PhysicsAsset.bodies[m_ConstraintParent].bone.c_str() : u8"부모 바디 선택";
    ImGui::SetNextItemWidth(-1);
    if (ImGui::BeginCombo("##ParentBody", parentName))
    {
        for (int i = 0; i < static_cast<int>(m_PhysicsAsset.bodies.size()); ++i)
        {
            if (i != m_SelectedBody && ImGui::Selectable(m_PhysicsAsset.bodies[i].bone.c_str(), i == m_ConstraintParent))
            {
                CancelInteraction();
                m_ConstraintParent = i;
            }
        }
        ImGui::EndCombo();
    }
    ImGui::BeginDisabled(m_ConstraintParent < 0 || m_ConstraintParent == m_SelectedBody);
    if (ImGui::Button(u8"+ 바디 연결"))
    {
        CreateConstraint();
    }
    ImGui::EndDisabled();
    ImGui::EndDisabled();
    if (ImGui::BeginListBox("##Joints", ImVec2(-1, ImGui::GetTextLineHeightWithSpacing() * 4)))
    {
        for (int i = 0; i < static_cast<int>(m_PhysicsAsset.constraints.size()); ++i)
        {
            const auto& joint = m_PhysicsAsset.constraints[i];
            ImGui::PushID(i);
            const auto label = joint.parentBone + " -> " + joint.childBone;
            if (ImGui::Selectable(label.c_str(), i == m_SelectedConstraint))
            {
                SelectConstraint(i);
            }
            if (ImGui::IsItemHovered())
            {
                ImGui::SetTooltip("%s", label.c_str());
            }
            ImGui::PopID();
        }
        ImGui::EndListBox();
    }
    if (!EditingConstraint())
    {
        return;
    }
    auto& joint = m_PhysicsAsset.constraints[m_SelectedConstraint];
    ImGui::TextWrapped("%s -> %s", joint.parentBone.c_str(), joint.childBone.c_str());
    if (ImGui::Button(u8"관절 삭제"))
    {
        CancelInteraction();
        m_PhysicsAsset.constraints.erase(m_PhysicsAsset.constraints.begin() + m_SelectedConstraint);
        m_SelectedConstraint = -1;
        m_PhysicsDirty = true;
        m_PhysicsMessage.clear();
        return;
    }
    bool changed = false;
    if (ImGui::RadioButton(u8"부모 좌표계", m_ConstraintFrame == 0))
    {
        CancelInteraction();
        m_ConstraintFrame = 0;
    }
    ImGui::SameLine();
    if (ImGui::RadioButton(u8"자식 좌표계", m_ConstraintFrame == 1))
    {
        CancelInteraction();
        m_ConstraintFrame = 1;
    }
    bool moveBoth = m_MoveBothFrames;
    if (ImGui::Checkbox(u8"두 좌표계 함께 이동", &moveBoth))
    {
        CancelInteraction();
        m_MoveBothFrames = moveBoth;
    }
    ImGui::BeginDisabled(m_DragAxis >= 0);
    const Matrix4 previousWorld = GetConstraintWorld(m_SelectedConstraint, m_ConstraintFrame);
    if (EditFrame(m_ConstraintFrame == 0 ? joint.parentFrame : joint.childFrame))
    {
        if (m_MoveBothFrames)
        {
            MoveOtherConstraintFrame(previousWorld);
        }
        changed = true;
    }
    if (ImGui::Button(u8"두 좌표계를 자식 본에 정렬"))
    {
        ResetConstraintFrames(joint);
        changed = true;
    }
    if (ImGui::IsItemHovered())
    {
        ImGui::SetTooltip(u8"자식 본 위치에서 자식 바디 방향으로 두 좌표계를 일치시킵니다. 기존 두 좌표계의 상대 회전 보정은 초기화됩니다.");
    }
    const bool articulation = m_PhysicsAsset.UsesArticulation();
    ImGui::TextDisabled(articulation ? u8"위치 잠금 | 회전 벡터 X/Y/Z" : u8"위치 잠금 | X 트위스트, Y/Z 스윙");
    ImGui::TextWrapped(articulation ?
        u8"Articulation은 부모 관절 좌표계에서 상대 회전을 회전 벡터 X/Y/Z로 나누어 제한합니다. 부채꼴은 각 축만 회전할 때의 범위이며, 세 축을 함께 회전할 때의 전체 허용 영역을 뜻하지 않습니다." :
        u8"스윙 값은 부모 좌표계 기준 원뿔의 반각입니다. 좌표계 회전에 따라 허용 범위가 치우칠 수 있어 0도가 기준 자세와 같지 않을 수 있습니다. 두 좌표계를 함께 이동하면 이 보정을 유지합니다.");
    changed |= EditMotion(articulation ? u8"회전 X" : u8"트위스트 X", joint.twist);
    ImGui::BeginDisabled(joint.twist != PhysicsJointMotion::Limited);
    changed |= EditAngle(u8"최소각", joint.twistMin, -179.9f, XMConvertToDegrees(joint.twistMax) - .1f);
    changed |= EditAngle(u8"최대각", joint.twistMax, XMConvertToDegrees(joint.twistMin) + .1f, 179.9f);
    ImGui::EndDisabled();
    changed |= EditMotion(articulation ? u8"회전 Y" : u8"스윙 Y", joint.swingY);
    ImGui::BeginDisabled(joint.swingY != PhysicsJointMotion::Limited);
    changed |= EditAngle(u8"Y 제한각", joint.swingYLimit, .1f, 179.9f);
    ImGui::EndDisabled();
    changed |= EditMotion(articulation ? u8"회전 Z" : u8"스윙 Z", joint.swingZ);
    ImGui::BeginDisabled(joint.swingZ != PhysicsJointMotion::Limited);
    changed |= EditAngle(u8"Z 제한각", joint.swingZLimit, .1f, 179.9f);
    ImGui::EndDisabled();
    ImGui::BeginDisabled(articulation);
    changed |= ImGui::Checkbox(u8"연결된 바디끼리 충돌", &joint.enableCollision);
    ImGui::EndDisabled();
    if (articulation)
    {
        ImGui::TextDisabled(u8"Articulation의 직접 연결된 부모/자식은 충돌하지 않습니다.");
    }
    const bool locked = joint.twist == PhysicsJointMotion::Locked || joint.swingY == PhysicsJointMotion::Locked || joint.swingZ == PhysicsJointMotion::Locked;
    // 드라이브 패널이 접혀 있어도 운동 설정 편집 후 PA는 유효해야 한다.
    if (locked && joint.useSlerpDrive)
    {
        joint.useSlerpDrive = false;
        changed = true;
    }
    if (ImGui::CollapsingHeader(u8"액티브 래그돌 드라이브", ImGuiTreeNodeFlags_DefaultOpen))
    {
        ImGui::TextWrapped(u8"관성 기반 드라이브는 연결된 몸의 질량 행렬에서 축별 관성을 구해 토크 게인을 계산합니다. 골반 지지 중에는 중력과 운동 하중을 보상하며, PD와 보상이 최대 토크를 함께 사용합니다. 실행 중인 캐릭터에는 '에디터 드라이브 적용'으로 반영하세요.");
        if (ImGui::Button(u8"전체 관절 주파수 PD 기본값"))
        {
            ConfigureDefaultDrives(m_PhysicsAsset);
            changed = true;
        }
        ImGui::BeginDisabled(locked);
        changed |= ImGui::Checkbox(articulation ? u8"세 축의 주파수·감쇠비·토크 설정 공유" : u8"SLERP (회전 게인 공유)", &joint.useSlerpDrive);
        ImGui::EndDisabled();
        const char* names[] = {
            articulation ? u8"X 회전 드라이브" : u8"트위스트 X 드라이브",
            articulation ? u8"Y 회전 드라이브" : u8"스윙 Y 드라이브",
            articulation ? u8"Z 회전 드라이브" : u8"스윙 Z 드라이브"};
        const PhysicsJointMotion motions[] = {joint.twist, joint.swingY, joint.swingZ};
        for (int axis = 0; axis < (joint.useSlerpDrive ? 1 : 3); ++axis)
        {
            ImGui::PushID(axis);
            auto& drive = joint.drives[axis];
            ImGui::BeginDisabled(motions[axis] == PhysicsJointMotion::Locked);
            changed |= ImGui::Checkbox(joint.useSlerpDrive ? u8"회전 드라이브" : names[axis], &drive.enabled);
            ImGui::BeginDisabled(!drive.enabled);
            if (!drive.useAcceleration)
            {
                ImGui::TextUnformatted(u8"기존 토크 기준 드라이브");
                if (ImGui::Button(u8"관성 기반 기본값으로 전환"))
                {
                    const PhysicsAngularDrive defaults;
                    drive.useAcceleration = true;
                    drive.naturalFrequency = defaults.naturalFrequency;
                    drive.dampingRatio = defaults.dampingRatio;
                    changed = true;
                }
            }
            if (drive.useAcceleration)
            {
                changed |= ImGui::DragFloat(u8"고유각진동수 (rad/s)", &drive.naturalFrequency, .1f, 0, 200, "%.2f", ImGuiSliderFlags_AlwaysClamp);
                changed |= ImGui::DragFloat(u8"감쇠비", &drive.dampingRatio, .01f, 0, 10, "%.2f", ImGuiSliderFlags_AlwaysClamp);
                ImGui::TextDisabled(u8"정규화 계수: Kp/I %.2f, Kd/I %.2f", drive.GetStiffness(), drive.GetDamping());
            }
            else
            {
                changed |= ImGui::DragFloat(u8"강성 Kp", &drive.stiffness, 1, 0, 100000, "%.2f", ImGuiSliderFlags_AlwaysClamp);
                changed |= ImGui::DragFloat(u8"감쇠 Kd", &drive.damping, .1f, 0, 10000, "%.2f", ImGuiSliderFlags_AlwaysClamp);
            }
            changed |= ImGui::DragFloat(u8"최대 토크 (Nm)", &drive.maxTorque, .5f, 0, 10000, "%.2f", ImGuiSliderFlags_AlwaysClamp);
            ImGui::EndDisabled();
            ImGui::EndDisabled();
            ImGui::PopID();
        }
    }
    ImGui::EndDisabled();
    ImGui::TextWrapped("%s", ViewerLabels::ConstraintLimits(articulation));
    ImGui::TextWrapped(articulation ?
        u8"부채꼴은 부모 관절 좌표계를 기준으로 표시합니다. 세 회전 성분이 모두 0이면 부모·자식 관절 좌표계가 정렬된 상태입니다. 기준 자세가 반드시 0도인 것은 아닙니다." :
        u8"트위스트 0도는 몸의 정면, 손바닥 또는 팔꿈치/무릎 힌지의 아래팔/정강이 방향을 기준으로 합니다. 부채꼴과 바늘은 스윙을 제거한 X축 회전을 표시합니다.");
    if (changed)
    {
        m_PhysicsDirty = true;
        m_PhysicsMessage.clear();
    }
}

Vector3 SkeletalMeshViewer::GetTwistDisplayReference(const PhysicsConstraint& joint)
{
    const auto colon = joint.childBone.find_last_of(':');
    const char* bone = joint.childBone.c_str() + (colon == std::string::npos ? 0 : colon + 1);

    // 월드 축 대신 BuildMixamoPhysicsAsset의 해부학적 자식 좌표계에 맞춘다.
    // 몸통/목/머리, 어깨/위팔, 허벅지, 발에서는 -Z가 앞쪽이다.
    // 손에서는 손바닥 법선이며, 가져온 T 포즈에서는 아래쪽이다.
    // 같은 기준으로 부모의 각도 표시와 자식의 트위스트 바늘을 회전시킨다.
    // 애니메이션 재생과 좌표계 편집 중에도 조인트에 붙어 있도록 한다.
    for (const char* name : {"Spine", "Spine1", "Spine2", "Neck", "Head",
             "LeftShoulder", "RightShoulder", "LeftArm", "RightArm",
             "LeftHand", "RightHand",
             "LeftUpLeg", "RightUpLeg", "LeftFoot", "RightFoot"})
    {
        if (std::strcmp(bone, name) == 0)
        {
            return Vector3(0, 0, -1);
        }
    }

    // 힌지 X축은 팔꿈치/무릎의 굽힘 축이고, Y축은 아래팔/정강이를 따른다.
    // 따라서 초록색 트위스트 부채꼴이 실제 굽힘 동작을 따라간다.
    if (std::strcmp(bone, "LeftLeg") == 0 || std::strcmp(bone, "RightLeg") == 0 ||
        std::strcmp(bone, "LeftForeArm") == 0 || std::strcmp(bone, "RightForeArm") == 0)
    {
        return Vector3(kYUnitVector);
    }

    // 알 수 없는 리그는 조인트 로컬 규칙을 유지하며, 해부학적 구조를 추측하지 않는다.
    return Vector3(kYUnitVector);
}

void SkeletalMeshViewer::DrawConstraints()
{
    if (!m_ShowConstraints && !m_ShowLimits)
    {
        return;
    }
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
        {
            m_Overlay.ConstraintLimits(joint, parent, child, id, selected, GetTwistDisplayReference(joint), m_PhysicsAsset.UsesArticulation());
        }
    }
}
