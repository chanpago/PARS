#include "SkeletalMeshViewer.h"
#include "Components/SkeletalMeshComponent.h"
#include "ImGuiOverlay.h"
#include "ModelLoader.h"
#include "Renderer.h"
#include "imgui.h"
#include <cmath>
#include <exception>

using namespace Math;

namespace
{
    void FindCharacterMeshes(const std::wstring& root, const std::wstring& folder,
        std::map<std::wstring, std::wstring>& files)
    {
        WIN32_FIND_DATAW entry{};
        const HANDLE search = FindFirstFileW((root + folder + L"*").c_str(), &entry);
        if (search == INVALID_HANDLE_VALUE)
        {
            return;
        }
        do
        {
            const std::wstring name = entry.cFileName;
            if (name == L"." || name == L".." || (entry.dwFileAttributes & FILE_ATTRIBUTE_REPARSE_POINT))
            {
                continue;
            }
            const std::wstring relative = folder + name;
            if (entry.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY)
            {
                // Mixamo의 변환 폴더에는 같은 외형을 담은 동작별 파일이 있다. 기준 메시는 별도로 등록한다.
                if (Utility::ToLower(relative) != L"mixamo/converted")
                {
                    FindCharacterMeshes(root, relative + L"/", files);
                }
                continue;
            }
            const std::wstring extension = Utility::ToLower(Utility::GetFileExtension(name));
            const std::wstring key = Utility::ToLower(Utility::RemoveExtension(relative));
            if (extension == L"gltf")
            {
                // 같은 모델의 단일 파일 배포본도 있을 때는 엔진용 외부 텍스처 경로를 사용한다.
                files[key] = relative;
            }
            else if (extension == L"glb")
            {
                files.emplace(key, relative);
            }
        }
        while (FindNextFileW(search, &entry));
        FindClose(search);
    }
}

void SkeletalMeshViewer::InitializeMeshPreview()
{
    CharacterPreview& preview = m_CharacterPreview;
    // 두 창은 HDR과 깊이 버퍼를 순서대로 사용하고, ImGui가 읽는 최종 이미지만 따로 보관한다.
    preview.display.Create(L"Character mesh preview", m_Display.GetWidth(), m_Display.GetHeight(), 1, m_Display.GetFormat());
    preview.image = ImGuiOverlay::RegisterTexture(preview.display.GetSRV());
    std::map<std::wstring, std::wstring> files;
    FindCharacterMeshes(preview.directory, L"", files);
    preview.meshes.push_back({u8"Mixamo 기본 캐릭터", std::make_unique<ModelInstance>(m_Mesh->GetSkeletalMesh())});
    for (const std::map<std::wstring, std::wstring>::value_type& file : files)
    {
        std::shared_ptr<const Model> model;
        try
        {
            model = Renderer::LoadModel(preview.directory + file.second, preview.forceRebuild);
        }
        catch (const std::exception& error)
        {
            Utility::Printf("Failed to load character mesh %s: %s\n", Utility::WideStringToUTF8(file.second).c_str(), error.what());
        }
        if (!model)
        {
            preview.loadError += Utility::WideStringToUTF8(file.second) + "\n";
            continue;
        }
        if (model->m_NumMeshes == 0 || model->m_NumNodes == 0 || model->m_NumJoints == 0)
        {
            continue;
        }
        const bool isGuard = file.first == L"hernandguard/hernandguard";
        const std::string label = isGuard ? u8"에르난드 성문 경비병" : Utility::WideStringToUTF8(Utility::RemoveExtension(file.second));
        if (isGuard)
        {
            preview.selected = preview.meshes.size();
        }
        preview.meshes.push_back({label, std::make_unique<ModelInstance>(model)});
    }
    for (PreviewMesh& mesh : preview.meshes)
    {
        mesh.instance->PlayAnimation(0, true);
        // 창처럼 손에 고정된 부품도 본이 움직인 뒤에는 캐릭터 전체 경계로 가시성을 판단한다.
        mesh.instance->SetSkinnedWorldBounds(mesh.instance->GetBoundingSphere());
    }
    ResetMeshPreviewCamera();
}

void SkeletalMeshViewer::ResetMeshPreviewCamera()
{
    m_CharacterPreview.yaw = -0.12f;
    m_CharacterPreview.pitch = 0.08f;
    m_CharacterPreview.zoom = 1.0f;
}

void SkeletalMeshViewer::DrawMeshPreview(GraphicsContext& context)
{
    CharacterPreview& preview = m_CharacterPreview;
    preview.focused = false;
    if (!preview.open || !m_Mesh)
    {
        return;
    }
    const ImVec2 screen = ImGui::GetIO().DisplaySize;
    ImGui::SetNextWindowPos(ImVec2(screen.x * 0.5f, screen.y * 0.5f), ImGuiCond_FirstUseEver, ImVec2(0.5f, 0.5f));
    ImGui::SetNextWindowSize(ImVec2(Min(840.0f, screen.x * 0.82f), Min(780.0f, screen.y * 0.85f)), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowBgAlpha(1.0f);
    if (preview.focusRequested)
    {
        ImGui::SetNextWindowFocus();
        preview.focusRequested = false;
    }
    if (ImGui::Begin(u8"캐릭터 메시 미리보기", &preview.open, ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse))
    {
        if (!preview.image)
        {
            InitializeMeshPreview();
        }
        preview.focused = ImGui::IsWindowFocused(ImGuiFocusedFlags_RootAndChildWindows) && !ImGui::GetIO().AppFocusLost;
        ImGui::SetNextItemWidth(Min(250.0f, ImGui::GetContentRegionAvail().x * 0.55f));
        if (ImGui::BeginCombo(u8"캐릭터", preview.meshes[preview.selected].label.c_str()))
        {
            for (size_t index = 0; index < preview.meshes.size(); ++index)
            {
                const bool selected = index == preview.selected;
                if (ImGui::Selectable(preview.meshes[index].label.c_str(), selected))
                {
                    preview.selected = index;
                    ResetMeshPreviewCamera();
                }
                if (selected)
                {
                    ImGui::SetItemDefaultFocus();
                }
            }
            ImGui::EndCombo();
        }
        ImGui::SameLine();
        if (ImGui::Button(u8"화면 맞춤 (F)") || (preview.focused && !ImGui::GetIO().WantTextInput && ImGui::IsKeyPressed(ImGuiKey_F, false)))
        {
            ResetMeshPreviewCamera();
        }
        ImGui::TextDisabled(u8"드래그로 회전  |  마우스 휠로 확대·축소");
        if (!preview.loadError.empty())
        {
            ImGui::TextWrapped(u8"불러오지 못한 모델\n%s", preview.loadError.c_str());
        }
        DrawMeshPreviewViewport(context);
    }
    ImGui::End();
}

void SkeletalMeshViewer::DrawMeshPreviewViewport(GraphicsContext& context)
{
    const ImVec2 size = ImGui::GetContentRegionAvail();
    if (size.x < 2.0f || size.y < 2.0f)
    {
        return;
    }
    CharacterPreview& preview = m_CharacterPreview;
    ImGui::InvisibleButton("##CharacterMeshViewport", size);
    const ImGuiIO& io = ImGui::GetIO();
    if (ImGui::IsItemActive() && ImGui::IsMouseDragging(ImGuiMouseButton_Left))
    {
        preview.yaw = std::remainder(preview.yaw - io.MouseDelta.x * 0.009f, XM_2PI);
        preview.pitch = Clamp(preview.pitch + io.MouseDelta.y * 0.009f, -1.5f, 1.5f);
    }
    if (ImGui::IsItemHovered() && io.MouseWheel != 0.0f)
    {
        preview.zoom = Clamp(preview.zoom * std::exp(-0.16f * io.MouseWheel), 0.25f, 4.0f);
    }
    ModelInstance& instance = *preview.meshes[preview.selected].instance;
    const Vector3 target = instance.GetBoundingBox().GetCenter();
    const float radius = Max(0.1f, float(instance.GetRadius()));
    const float aspect = size.y / size.x;
    const float distance = radius * 2.6f * Max(1.0f, aspect) * preview.zoom;
    const float horizontal = std::cos(preview.pitch);
    const Vector3 direction(std::sin(preview.yaw) * horizontal, std::sin(preview.pitch), std::cos(preview.yaw) * horizontal);
    preview.camera.SetEyeAtUp(target + direction * distance, target, Vector3(kYUnitVector));
    preview.camera.SetPerspectiveMatrix(XM_PIDIV4, aspect, radius * 0.01f, distance + radius * 4.0f);
    preview.camera.Update();
    instance.Update(context, io.DeltaTime);
    Renderer::MeshSorter sorter(Renderer::MeshSorter::kDefault);
    sorter.SetCamera(preview.camera);
    instance.Render(sorter);
    RenderPreview(context, sorter, preview.display);
    ImGui::GetWindowDrawList()->AddImage(static_cast<ImTextureID>(preview.image), ImGui::GetItemRectMin(), ImGui::GetItemRectMax());
}
