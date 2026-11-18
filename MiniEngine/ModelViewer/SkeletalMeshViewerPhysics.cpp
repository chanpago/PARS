#include "SkeletalMeshViewer.h"
#include "Components/SkeletalMeshComponent.h"
#include "imgui.h"
#include "imgui_stdlib.h"
#include <algorithm>
#include <cmath>

using namespace Math;
using namespace PARS;

void SkeletalMeshViewer::SetDefaultPhysicsAssetPath(const std::string& path)
{
    m_DefaultPhysicsPath = path;
    m_PhysicsPath = path;
}

bool SkeletalMeshViewer::ConsumePhysicsSaveRequest(std::string& path)
{
    const bool requested = m_PhysicsSaveRequested;
    path = m_RequestedPhysicsSavePath;
    m_PhysicsSaveRequested = false;
    return requested;
}

void SkeletalMeshViewer::CompletePhysicsSave(bool saved, const std::string& message)
{
    if (saved)
    {
        m_PhysicsDirty = false;
    }
    m_PhysicsMessage = message;
}

bool SkeletalMeshViewer::SetPhysicsAsset(const PhysicsAsset& asset, std::string& error)
{
    if (!m_Mesh)
    {
        error = u8"미리보기 메시를 사용할 수 없습니다.";
        return false;
    }
    std::vector<std::string> joints;
    const Model& model = *m_Mesh->GetSkeletalMesh();
    for (uint32_t i = 0; i < model.m_NumNodes; ++i)
    {
        if (m_IsJoint[i])
        {
            joints.push_back(model.m_NodeNames[i]);
        }
    }
    if (!asset.ValidateSkeleton(joints, error))
    {
        return false;
    }
    CancelInteraction();
    m_PhysicsAsset = asset;
    m_PhysicsDirty = true;
    SelectBone(m_SelectedBone);
    return true;
}

int SkeletalMeshViewer::FindBodyNode(int index) const
{
    const std::vector<std::string>& names = m_Mesh->GetSkeletalMesh()->m_NodeNames;
    const std::string& bone = m_PhysicsAsset.bodies[index].bone;
    const std::vector<std::string>::const_iterator it = std::find(names.begin(), names.end(), bone);
    return it == names.end() ? -1 : static_cast<int>(it - names.begin());
}

Matrix4 SkeletalMeshViewer::GetBodyWorld(int index) const
{
    const int node = FindBodyNode(index);
    return node < 0 ? Matrix4(kIdentity) : PhysicsBoneFrame(m_NodeWorld[node]) * m_PhysicsAsset.bodies[index].GetLocalTransform();
}

Matrix4 SkeletalMeshViewer::GetGizmoWorld() const
{
    if (EditingConstraint())
    {
        return GetConstraintWorld(m_SelectedConstraint, m_ConstraintFrame);
    }
    return EditingBody() ? GetBodyWorld(m_SelectedBody) : m_NodeWorld[m_SelectedBone];
}

void SkeletalMeshViewer::SelectBody(int index)
{
    CancelInteraction();
    m_EditPhysics = true;
    m_EditConstraints = false;
    m_SelectedConstraint = -1;
    m_SelectedBody = index;
    m_SelectedBone = index < 0 ? -1 : FindBodyNode(index);
    m_RevealSelection = true;
}

void SkeletalMeshViewer::CreatePhysicsBody(PhysicsShapeType shape)
{
    if (m_SelectedBone < 0)
    {
        return;
    }
    CancelInteraction();
    const std::vector<std::string>& names = m_Mesh->GetSkeletalMesh()->m_NodeNames;
    const int existing = m_PhysicsAsset.FindBody(names[m_SelectedBone]);
    if (existing >= 0)
    {
        SelectBody(existing);
        return;
    }
    PARS::AnimInstance& animation = m_Mesh->GetAnimInstance();
    animation.SetPlaying(false);
    if (m_EditPose.empty())
    {
        m_EditPose = animation.GetPose();
    }
    PhysicsBody body;
    body.bone = names[m_SelectedBone];
    body.shape = shape;
    const Matrix4 frame = PhysicsBoneFrame(m_NodeWorld[m_SelectedBone]);
    Vector3 endpoint(.15f, 0, 0);
    // 가장 가까운 자식 조인트에 맞춘다. 말단 조인트에는 편집 가능한 작은 바디를 만든다.
    float shortest = 1e10f;
    for (uint32_t i = 0; i < m_BoneParents.size(); ++i)
    {
        if (m_IsJoint[i] && m_BoneParents[i] == m_SelectedBone)
        {
            const Vector3 local(Invert(frame) * Vector3(m_NodeWorld[i].GetW()));
            const float length = Length(local);
            if (length > .005f && length < shortest)
            {
                shortest = length;
                endpoint = local;
            }
        }
    }
    const float length = Length(endpoint);
    const Vector3 axis = endpoint / length;
    const float dot = Clamp(float(axis.GetX()), -1.0f, 1.0f);
    Quaternion rotation(kIdentity);
    if (dot < -.9999f)
    {
        rotation = Quaternion(Vector3(kYUnitVector), XM_PI);
    }
    else if (dot < .9999f)
    {
        rotation = Quaternion(Normalize(Cross(Vector3(kXUnitVector), axis)), std::acos(dot));
    }
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
    if (!m_ShowBodies)
    {
        return;
    }
    struct VisibleBody
    {
    public:
        Matrix4 world;
        int index;
        float depth;
    };
    std::vector<VisibleBody> sorted;
    sorted.reserve(m_PhysicsAsset.bodies.size());
    for (size_t i = 0; i < m_PhysicsAsset.bodies.size(); ++i)
    {
        const int node = FindBodyNode(static_cast<int>(i));
        if (node < 0)
        {
            continue;
        }
        const Matrix4 world = PhysicsBoneFrame(m_NodeWorld[node]) * m_PhysicsAsset.bodies[i].GetLocalTransform();
        sorted.push_back({world, static_cast<int>(i),
            Dot(Vector3(world.GetW()) - m_Camera.GetPosition(), m_Camera.GetForwardVec())});
    }
    // 볼록 표면은 뒷면을 컬링한다. 알파 블렌딩을 위해 바디를 먼 순서부터 정렬한다.
    // 본 검색과 강체 좌표계 추출은 정렬 비교마다 수행하지 않고 바디마다 한 번만 수행한다.
    std::sort(sorted.begin(), sorted.end(), [](const VisibleBody& a, const VisibleBody& b) -> bool
        {
        return a.depth > b.depth;
    });
    for (const VisibleBody& visible : sorted)
    {
        const int index = visible.index;
        const bool selected = EditingBody() && index == m_SelectedBody;
        m_Overlay.Body(m_PhysicsAsset.bodies[index], visible.world,
            selected ? Vector3(1, .68f, .12f) : Vector3(.2f, .7f, 1),
            0x400001 + index, m_BodyOpacity);
    }
}

void SkeletalMeshViewer::DrawPhysicsPanel()
{
    ImGui::Text(u8"PA | 바디 %zu개 / 관절 %zu개%s", m_PhysicsAsset.bodies.size(), m_PhysicsAsset.constraints.size(), m_PhysicsDirty ? " *" : "");
    ImGui::BeginDisabled(!m_PhysicsSaveAvailable || m_DefaultPhysicsPath.empty());
    if (ImGui::Button(u8"저장 후 기본 PA로 적용"))
    {
        CancelInteraction();
        m_PhysicsSaveRequested = true;
        m_RequestedPhysicsSavePath = m_DefaultPhysicsPath;
    }
    ImGui::EndDisabled();
    if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled))
    {
        ImGui::SetTooltip(u8"저장한 PA를 플레이어와 더미에 적용합니다.\n현재 위치와 모드는 유지하고 물리 바디를 재생성합니다.\n저장 위치: %s",
            m_DefaultPhysicsPath.c_str());
    }
    ImGui::TextWrapped(u8"다시 실행해도 유지하려면 위 버튼으로 저장하세요.");
    ImGui::SetNextItemWidth(-1);
    ImGui::InputText("##PhysicsAssetPath", &m_PhysicsPath);
    if (ImGui::IsItemHovered())
    {
        ImGui::SetTooltip(u8"아래 파일 저장/불러오기에 사용할 JSON 경로입니다. 파일 저장 시 플레이어와 더미에도 적용합니다. 기본 PA 적용 버튼은 캐릭터 에셋 폴더에 저장합니다.");
    }
    if (ImGui::Button(u8"PA 파일 저장"))
    {
        CancelInteraction();
        if (m_PhysicsSaveAvailable)
        {
            m_PhysicsSaveRequested = true;
            m_RequestedPhysicsSavePath = m_PhysicsPath;
        }
        else if (m_PhysicsAsset.Save(m_PhysicsPath, m_PhysicsMessage))
        {
            m_PhysicsDirty = false;
            m_PhysicsMessage = u8"PA 파일을 저장했습니다.";
        }
    }
    ImGui::SameLine();
    bool load = false;
    if (ImGui::Button(u8"PA 불러오기"))
    {
        if (m_PhysicsDirty)
        {
            ImGui::OpenPopup(u8"편집 내용을 바꿀까요?");
        }
        else
        {
            load = true;
        }
    }
    if (ImGui::BeginPopupModal(u8"편집 내용을 바꿀까요?", nullptr, ImGuiWindowFlags_AlwaysAutoResize))
    {
        ImGui::TextUnformatted(u8"저장하지 않은 PA 편집 내용을 이 파일로 바꿀까요?");
        if (ImGui::Button(u8"불러오기"))
        {
            load = true;
            ImGui::CloseCurrentPopup();
        }
        ImGui::SameLine();
        if (ImGui::Button(u8"취소"))
        {
            ImGui::CloseCurrentPopup();
        }
        ImGui::EndPopup();
    }
    if (load)
    {
        CancelInteraction();
        PhysicsAsset candidate;
        if (candidate.Load(m_PhysicsPath, m_PhysicsMessage))
        {
            const Model& model = *m_Mesh->GetSkeletalMesh();
            std::vector<std::string> joints;
            for (uint32_t i = 0; i < model.m_NumNodes; ++i)
            {
                if (m_IsJoint[i])
                {
                    joints.push_back(model.m_NodeNames[i]);
                }
            }
            if (candidate.ValidateSkeleton(joints, m_PhysicsMessage))
            {
                m_PhysicsAsset = std::move(candidate);
                m_PhysicsDirty = false;
                m_PhysicsMessage = u8"불러왔습니다.";
                SelectBone(m_SelectedBone);
                m_ShowBodies = true;
                m_ShowConstraints = true;
            }
        }
    }
    if (ImGui::Button(u8"미리보기 메시에 적용"))
    {
        CancelInteraction();
        if (m_Mesh->SetPhysicsAsset(m_PhysicsAsset, m_PhysicsMessage))
        {
            m_PhysicsMessage = u8"현재 PA를 미리보기 메시에 적용했습니다. 물리 시뮬레이션은 시작하지 않습니다.";
        }
    }
    if (ImGui::IsItemHovered())
    {
        ImGui::SetTooltip(u8"현재 PA 설정을 미리보기 메시에 복사합니다. 이후 편집한 내용을 반영하려면 다시 적용하세요.");
    }
    if (!m_PhysicsMessage.empty())
    {
        ImGui::TextWrapped("%s", m_PhysicsMessage.c_str());
    }
    ImGui::Separator();
    if (ImGui::RadioButton(u8"바디##PA", !m_EditConstraints))
    {
        CancelInteraction();
        m_EditConstraints = false;
    }
    ImGui::SameLine();
    if (ImGui::RadioButton(u8"관절##PA", m_EditConstraints))
    {
        CancelInteraction();
        m_EditConstraints = true;
        if (m_GizmoMode == 2)
        {
            m_GizmoMode = 0;
        }
        m_ConstraintParent = FindConstraintParent();
    }
    if (m_EditConstraints)
    {
        DrawConstraintPanel();
        return;
    }
    if (m_SelectedBone < 0)
    {
        ImGui::TextWrapped(u8"바디를 추가하려면 트리에서 본을 선택하세요.");
    }
    ImGui::BeginDisabled(m_SelectedBone < 0 || m_SelectedBody >= 0);
    if (ImGui::Button(u8"+ 캡슐"))
    {
        CreatePhysicsBody(PhysicsShapeType::Capsule);
    }
    ImGui::SameLine();
    if (ImGui::Button(u8"+ 상자"))
    {
        CreatePhysicsBody(PhysicsShapeType::Box);
    }
    ImGui::SameLine();
    if (ImGui::Button(u8"+ 구"))
    {
        CreatePhysicsBody(PhysicsShapeType::Sphere);
    }
    ImGui::EndDisabled();
    if (!EditingBody())
    {
        return;
    }
    PARS::PhysicsBody body = m_PhysicsAsset.bodies[m_SelectedBody];
    bool changed = false;
    ImGui::SetNextItemWidth(100);
    int shape = static_cast<int>(body.shape);
    if (ImGui::Combo(u8"형상", &shape, u8"캡슐\0상자\0구\0"))
    {
        body.shape = static_cast<PhysicsShapeType>(shape);
        changed = true;
    }
    ImGui::TextUnformatted(u8"본 기준 위치 (m)");
    ImGui::SetNextItemWidth(-1);
    changed |= ImGui::DragFloat3("##BodyOffset", &body.offset.x, .002f, -100, 100, "%.3f", ImGuiSliderFlags_AlwaysClamp);
    XMFLOAT4 rotation;
    XMStoreFloat4(&rotation, Normalize(Quaternion(XMLoadFloat4(&body.rotation))));
    const float sineY = Clamp(2 * (rotation.w * rotation.y - rotation.z * rotation.x), -1.0f, 1.0f);
    const float sineXCosineY = 2 * (rotation.w * rotation.x + rotation.y * rotation.z);
    const float cosineXCosineY = 1 - 2 * (rotation.x * rotation.x + rotation.y * rotation.y);
    const float cosineY = std::hypot(sineXCosineY, cosineXCosineY);
    float rotationDegrees[3];
    rotationDegrees[1] = XMConvertToDegrees(std::atan2(sineY, cosineY));
    if (cosineY > 1e-5f)
    {
        rotationDegrees[0] = XMConvertToDegrees(std::atan2(sineXCosineY, cosineXCosineY));
        rotationDegrees[2] = XMConvertToDegrees(std::atan2(2 * (rotation.w * rotation.z + rotation.x * rotation.y),
            1 - 2 * (rotation.y * rotation.y + rotation.z * rotation.z)));
    }
    else
    {
        // Y축이 ±90도이면 분리할 수 없는 X/Z 회전을 X축에 합쳐 같은 방향을 표시한다.
        rotationDegrees[0] = XMConvertToDegrees(std::atan2(2 * (rotation.w * rotation.x - rotation.y * rotation.z),
            1 - 2 * (rotation.x * rotation.x + rotation.z * rotation.z)));
        rotationDegrees[2] = 0;
    }
    ImGui::TextUnformatted(u8"본 기준 회전 X / Y / Z (도)");
    ImGui::SetNextItemWidth(-1);
    if (ImGui::DragFloat3("##BodyRotation", rotationDegrees, .5f, -180, 180, "%.2f", ImGuiSliderFlags_AlwaysClamp))
    {
        XMStoreFloat4(&body.rotation, Normalize(
            Quaternion(Vector3(kZUnitVector), XMConvertToRadians(rotationDegrees[2])) *
            Quaternion(Vector3(kYUnitVector), XMConvertToRadians(rotationDegrees[1])) *
            Quaternion(Vector3(kXUnitVector), XMConvertToRadians(rotationDegrees[0]))));
        changed = true;
    }
    if (ImGui::IsItemHovered())
    {
        ImGui::SetTooltip(u8"본 좌표계 기준 X → Y → Z 순서의 회전입니다.\n숫자를 더블클릭하거나 Ctrl+클릭하면 직접 입력할 수 있습니다.");
    }
    if (body.shape == PhysicsShapeType::Box)
    {
        ImGui::TextUnformatted(u8"상자 반크기 X / Y / Z (m)");
        ImGui::SetNextItemWidth(-1);
        changed |= ImGui::DragFloat3("##HalfExtents", &body.halfExtents.x, .001f, .001f, 100, "%.3f", ImGuiSliderFlags_AlwaysClamp);
    }
    else
    {
        ImGui::SetNextItemWidth(110);
        changed |= ImGui::DragFloat(u8"반지름 (m)", &body.radius, .001f, .001f, 100, "%.3f", ImGuiSliderFlags_AlwaysClamp);
        if (body.shape == PhysicsShapeType::Capsule)
        {
            ImGui::SetNextItemWidth(110);
            changed |= ImGui::DragFloat(u8"원통 반길이 (m)", &body.halfLength, .001f, 0, 100, "%.3f", ImGuiSliderFlags_AlwaysClamp);
            ImGui::TextDisabled(u8"X축 | 전체 길이: %.3f m", 2 * (body.halfLength + body.radius));
        }
    }
    ImGui::SetNextItemWidth(110);
    changed |= ImGui::DragFloat(u8"질량 (kg)", &body.mass, .1f, .001f, 10000, "%.2f", ImGuiSliderFlags_AlwaysClamp);
    if (ImGui::CollapsingHeader(u8"물리 머터리얼"))
    {
        ImGui::SetNextItemWidth(110);
        changed |= ImGui::DragFloat(u8"정지 마찰", &body.material.staticFriction, .01f, 0, 10, "%.3f", ImGuiSliderFlags_AlwaysClamp);
        ImGui::SetNextItemWidth(110);
        changed |= ImGui::DragFloat(u8"운동 마찰", &body.material.dynamicFriction, .01f, 0, 10, "%.3f", ImGuiSliderFlags_AlwaysClamp);
        ImGui::SetNextItemWidth(110);
        changed |= ImGui::DragFloat(u8"반발 계수", &body.material.restitution, .01f, 0, 1, "%.3f", ImGuiSliderFlags_AlwaysClamp);
        int combine = static_cast<int>(body.material.frictionCombine);
        ImGui::SetNextItemWidth(110);
        if (ImGui::Combo(u8"마찰 결합", &combine, u8"평균\0작은 값\0곱\0큰 값\0"))
        {
            body.material.frictionCombine = static_cast<PARS::PhysicsFrictionCombine>(combine);
            changed = true;
        }
        ImGui::TextWrapped(u8"기본 바닥에서 이 바디만 미끄럽게 하려면 마찰을 낮추고 '작은 값'을 선택하세요.");
    }
    if (ImGui::Button(u8"회전 초기화"))
    {
        body.rotation = XMFLOAT4(0, 0, 0, 1);
        changed = true;
    }
    ImGui::SameLine();
    if (ImGui::Button(u8"바디 삭제"))
    {
        CancelInteraction();
        m_PhysicsAsset.RemoveBody(m_SelectedBody);
        m_SelectedConstraint = -1;
        m_ConstraintParent = -1;
        m_SelectedBody = -1;
        changed = true;
    }
    if (ImGui::IsItemHovered())
    {
        ImGui::SetTooltip(u8"이 바디에 연결된 관절도 함께 삭제합니다.");
    }
    if (changed)
    {
        if (m_SelectedBody >= 0)
        {
            m_PhysicsAsset.UpdateBody(m_SelectedBody, body);
        }
        m_PhysicsDirty = true;
        m_PhysicsMessage.clear();
    }
}
