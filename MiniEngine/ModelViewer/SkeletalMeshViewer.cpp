#include "SkeletalMeshViewer.h"
#include "Components/SkeletalMeshComponent.h"
#include "Renderer.h"
#include "ConstantBuffers.h"
#include "BufferManager.h"
#include "GraphicsCommon.h"
#include "ImGuiOverlay.h"
#include "ViewerLabels.h"
#include "imgui.h"
#include "CompiledShaders/MeshPreviewVS.h"
#include "CompiledShaders/MeshPreviewPS.h"
#include <cmath>
#include <cfloat>

using namespace Math;
using namespace Graphics;

namespace
{
    constexpr uint32_t PreviewResolution = 1024;
}

bool SkeletalMeshViewer::Initialize(const std::shared_ptr<const Model>* models,
    const char* const* names, uint32_t count)
{
    if (!count || !models[0])
    {
        return false;
    }
    auto* actor = m_World.SpawnActor<PARS::Actor>("MeshPreview");
    m_Mesh = &actor->CreateDefaultSubobject<PARS::SkeletalMeshComponent>("SkeletalMeshComponent");
    actor->SetRootComponent(*m_Mesh);
    if (!m_Mesh->SetSkeletalMesh(models[0]))
    {
        return false;
    }
    for (uint32_t i = 0; i < count; ++i)
    {
        if (!m_Mesh->GetAnimInstance().AddAnimation(names[i], models[i]))
        {
            return false;
        }
    }
    m_Mesh->GetAnimInstance().PlayAnimation(0);
    BuildSkeleton();
    m_World.BeginPlay();
    m_World.Tick(0);
    m_Radius = Max(0.1f, float(m_Mesh->GetLocalBounds().GetRadius()));
    ResetCamera();

    // 고정 크기 렌더 텍스처를 사용해 크기 변경/닫기/다시 열기 중 디스크립터와 리소스 수명
    // 문제를 방지한다. 카메라는 화면에 표시되는 이미지의 종횡비를 사용한다.
    m_Color.SetClearColor(Color(0.025f, 0.035f, 0.055f, 1));
    m_Color.Create(L"Mesh preview HDR", PreviewResolution, PreviewResolution, 1, g_SceneColorBuffer.GetFormat());
    m_Display.Create(L"Mesh preview image", PreviewResolution, PreviewResolution, 1, DXGI_FORMAT_R8G8B8A8_UNORM);
    m_Depth.Create(L"Mesh preview depth", PreviewResolution, PreviewResolution, g_SceneDepthBuffer.GetFormat());
    m_AmbientOcclusion.Create(L"Mesh preview AO", PreviewResolution, PreviewResolution, 1, DXGI_FORMAT_R8_UNORM);
    m_Unshadowed.Create(L"Mesh preview shadow", 1, 1, DXGI_FORMAT_D32_FLOAT);

    // IBL/머티리얼 조명은 유지하면서 AO/그림자 입력은 독립적으로 사용한다. 이 테이블은
    // 렌더러 힙을 공유하므로 머티리얼과 스키닝 디스크립터가 유효하게 유지된다.
    m_Textures = Renderer::CreateCommonTextureTable(m_AmbientOcclusion.GetSRV(), m_Unshadowed.GetDepthSRV());

    m_PresentRoot.Reset(1, 1);
    m_PresentRoot[0].InitAsDescriptorRange(D3D12_DESCRIPTOR_RANGE_TYPE_SRV, 0, 1, D3D12_SHADER_VISIBILITY_PIXEL);
    m_PresentRoot.InitStaticSampler(0, SamplerLinearClampDesc, D3D12_SHADER_VISIBILITY_PIXEL);
    m_PresentRoot.Finalize(L"Mesh preview display");
    m_PresentPSO.SetRootSignature(m_PresentRoot);
    m_PresentPSO.SetRasterizerState(RasterizerTwoSided);
    m_PresentPSO.SetBlendState(BlendDisable);
    m_PresentPSO.SetDepthStencilState(DepthStateDisabled);
    m_PresentPSO.SetPrimitiveTopologyType(D3D12_PRIMITIVE_TOPOLOGY_TYPE_TRIANGLE);
    m_PresentPSO.SetRenderTargetFormat(m_Display.GetFormat(), DXGI_FORMAT_UNKNOWN);
    m_PresentPSO.SetVertexShader(g_pMeshPreviewVS, sizeof(g_pMeshPreviewVS));
    m_PresentPSO.SetPixelShader(g_pMeshPreviewPS, sizeof(g_pMeshPreviewPS));
    m_PresentPSO.Finalize();
    m_Image = ImGuiOverlay::RegisterTexture(m_Display.GetSRV());
    m_Overlay.Initialize(PreviewResolution);

    auto& context = GraphicsContext::Begin(L"Initialize mesh viewer");
    context.TransitionResource(m_AmbientOcclusion, D3D12_RESOURCE_STATE_RENDER_TARGET);
    context.TransitionResource(m_Unshadowed, D3D12_RESOURCE_STATE_DEPTH_WRITE, true);
    context.ClearColor(m_AmbientOcclusion);
    context.ClearDepth(m_Unshadowed);
    context.TransitionResource(m_AmbientOcclusion, D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE);
    context.TransitionResource(m_Unshadowed, D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE);
    context.Finish(true);
    return true;
}

void SkeletalMeshViewer::Cleanup()
{
    CancelInteraction();
    ImGuiOverlay::UnregisterTexture(m_Image);
    m_Image = 0;
    m_Overlay.Cleanup();
    m_Mesh = nullptr;
    m_World.Clear();
    m_EditPose.clear();
    m_PickPose.clear();
    m_SelectedBone = -1;
    m_SelectedBody = -1;
    m_SelectedConstraint = -1;
    m_ConstraintParent = -1;
    m_PhysicsAsset = {};
    m_PhysicsDirty = false;
    m_Color.Destroy();
    m_Display.Destroy();
    m_Depth.Destroy();
    m_AmbientOcclusion.Destroy();
    m_Unshadowed.Destroy();
}

void SkeletalMeshViewer::Update(float deltaTime)
{
    if (m_Open && m_Mesh)
    {
        m_World.Tick(deltaTime);
    }
}

void SkeletalMeshViewer::ResetCamera()
{
    m_Yaw = 0;
    m_Pitch = 0.12f;
    const Vector3 target = m_Mesh->GetLocalBounds().GetCenter();
    m_Camera.SetEyeAtUp(target + Vector3(0, std::sin(m_Pitch), std::cos(m_Pitch)) * (m_Radius * 3.4f),
        target, Vector3(kYUnitVector));
}

void SkeletalMeshViewer::UpdateCamera(bool hovered)
{
    const auto& io = ImGui::GetIO();
    if (io.AppFocusLost || ImGui::IsKeyPressed(ImGuiKey_Escape, false))
    {
        m_Flying = false;
        return;
    }
    if (hovered && m_DragAxis < 0 && ImGui::IsMouseClicked(ImGuiMouseButton_Right))
    {
        CancelInteraction();
        // ImGui Image는 마우스 오른쪽 버튼으로 포커스를 얻지 않는다. 포커스 검사 전에
        // 뷰포트에 포커스를 줘서 다른 패널에서 돌아왔을 때 첫 클릭부터 동작하도록 한다.
        ImGui::SetWindowFocus();
        m_Focused = ImGui::IsWindowFocused(ImGuiFocusedFlags_RootAndChildWindows);
        m_Flying = true;
    }
    if (!m_Focused)
    {
        m_Flying = false;
        return;
    }
    if (!ImGui::IsMouseDown(ImGuiMouseButton_Right))
    {
        m_Flying = false;
    }
    if (m_Flying)
    {
        // 카메라 자신의 위치를 중심으로 회전한다. 메시는 화면 구도를 잡는 대상으로만 사용한다.
        m_Yaw = std::remainder(m_Yaw - io.MouseDelta.x * 0.008f, XM_2PI);
        m_Pitch = Clamp(m_Pitch + io.MouseDelta.y * 0.008f, -1.55f, 1.55f);
        const Vector3 forward(-std::sin(m_Yaw) * std::cos(m_Pitch), -std::sin(m_Pitch),
            -std::cos(m_Yaw) * std::cos(m_Pitch));
        m_Camera.SetLookDirection(forward, Vector3(kYUnitVector));
        if (!io.WantTextInput)
        {
            const float forwardInput = float(ImGui::IsKeyDown(ImGuiKey_W)) - float(ImGui::IsKeyDown(ImGuiKey_S));
            const float rightInput = float(ImGui::IsKeyDown(ImGuiKey_D)) - float(ImGui::IsKeyDown(ImGuiKey_A));
            const float upInput = float(ImGui::IsKeyDown(ImGuiKey_E)) - float(ImGui::IsKeyDown(ImGuiKey_Q));
            const Vector3 movement = forward * forwardInput + m_Camera.GetRightVec() * rightInput + Vector3(kYUnitVector) * upInput;
            if (float(LengthSquare(movement)) > 0.000001f)
            {
                // ImGui는 미리보기 애니메이션 재생과 별개로 실제 경과 시간을 사용한다.
                const float speed = m_Radius * 1.5f * (io.KeyShift ? 3.0f : 1.0f);
                m_Camera.SetPosition(m_Camera.GetPosition() + Normalize(movement) * (speed * io.DeltaTime));
            }
        }
    }
    if (hovered && m_DragAxis < 0 && io.MouseWheel != 0)
    {
        m_Overlay.CancelPick();
        m_HasPickRequest = false;
        m_Camera.SetPosition(m_Camera.GetPosition() + m_Camera.GetForwardVec() * (io.MouseWheel * m_Radius * 0.25f));
    }
}

void SkeletalMeshViewer::RenderUI(GraphicsContext& context, bool comparisonAvailable)
{
    m_Focused = false;
    if (!m_Open || !m_Mesh)
    {
        CancelInteraction();
        return;
    }
    const auto display = ImGui::GetIO().DisplaySize;
    ImGui::SetNextWindowPos(ImVec2(display.x * 0.5f, display.y * 0.5f), ImGuiCond_FirstUseEver, ImVec2(0.5f, 0.5f));
    ImGui::SetNextWindowSize(ImVec2(Min(1700.0f, display.x * .94f), display.y * .9f), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSizeConstraints(ImVec2(Min(1180.0f, display.x * .96f), Min(620.0f, display.y * .9f)), ImVec2(FLT_MAX, FLT_MAX));
    ImGui::SetNextWindowBgAlpha(1.0f);
    if (m_FocusRequested)
    {
        ImGui::SetNextWindowFocus();
        m_FocusRequested = false;
    }
    if (ImGui::Begin(u8"스켈레탈 메시 뷰어", &m_Open, ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse))
    {
        m_Focused = ImGui::IsWindowFocused(ImGuiFocusedFlags_RootAndChildWindows) && !ImGui::GetIO().AppFocusLost;
        if (!m_Focused)
        {
            CancelInteraction();
        }
        uint32_t pickedId;
        if (m_Overlay.PollPick(pickedId) && m_Focused)
        {
            HandlePick(pickedId);
        }
        auto& animation = m_Mesh->GetAnimInstance();
        ImGui::TextUnformatted(u8"Mixamo 캐릭터");
        ImGui::SameLine();
        ImGui::TextDisabled(u8"| 본 %u개 | 바디 %zu개 | 관절 %zu개", m_Mesh->GetSkeletalMesh()->m_NumJoints,
            m_PhysicsAsset.bodies.size(), m_PhysicsAsset.constraints.size());
        ImGui::BeginDisabled(!comparisonAvailable || m_PhysicsAsset.bodies.empty());
        if (ImGui::Button(u8"걷기 비교 시작"))
        {
            m_ComparisonRequested = true;
        }
        ImGui::EndDisabled();
        ImGui::SameLine();
        ImGui::TextDisabled(u8"키네마틱 / 액티브 비교 | 6초 | 현재 에디터 PA | 한글 보고서");
        ImGui::SetNextItemWidth(130);
        if (ImGui::BeginCombo("##Animation", ViewerLabels::Animation(animation.GetAnimationName(animation.GetCurrentAnimationIndex()))))
        {
            for (uint32_t i = 0; i < animation.GetAnimationCount(); ++i)
            {
                if (ImGui::Selectable(ViewerLabels::Animation(animation.GetAnimationName(i)), i == animation.GetCurrentAnimationIndex()))
                {
                    CancelInteraction();
                    m_EditPose.clear();
                    animation.PlayAnimation(i);
                }
            }
            ImGui::EndCombo();
        }
        ImGui::SameLine();
        bool playing = animation.IsPlaying();
        if (ImGui::Checkbox(u8"재생", &playing))
        {
            CancelInteraction();
            if (playing)
            {
                m_EditPose.clear();
            }
            animation.SetPlaying(playing);
        }
        ImGui::SameLine();
        if (ImGui::Button(u8"기준 자세"))
        {
            CancelInteraction();
            animation.SetPlaying(false);
            const auto& model = *m_Mesh->GetSkeletalMesh();
            m_EditPose.assign(model.m_SceneGraph.get(), model.m_SceneGraph.get() + model.m_NumNodes);
        }
        ImGui::SameLine();
        if (ImGui::Button(u8"애니메이션 자세"))
        {
            CancelInteraction();
            m_EditPose.clear();
        }
        ImGui::SameLine();
        if (ImGui::Button(u8"메시에 화면 맞춤"))
        {
            CancelInteraction();
            ResetCamera();
        }
        if (ImGui::Checkbox(u8"본", &m_ShowBones))
        {
            CancelInteraction();
        }
        ImGui::SameLine();
        if (ImGui::Checkbox(u8"바디", &m_ShowBodies))
        {
            CancelInteraction();
        }
        ImGui::SameLine();
        ImGui::SetNextItemWidth(110);
        ImGui::SliderFloat(u8"불투명도", &m_BodyOpacity, .05f, .75f, "%.2f");
        ImGui::SameLine();
        if (ImGui::Checkbox(u8"관절", &m_ShowConstraints))
        {
            CancelInteraction();
        }
        ImGui::SameLine();
        if (ImGui::Checkbox(u8"제한 범위", &m_ShowLimits))
        {
            CancelInteraction();
        }
        if (ImGui::IsItemHovered())
        {
            ImGui::SetTooltip(u8"빨간 원뿔: 스윙 허용 범위. 초록 부채꼴: 회전 허용 범위. 노랑: 현재 방향 / 트위스트. 제한 표시를 클릭하면 해당 관절을 선택합니다.");
        }
        ImGui::SameLine();
        ImGui::SetNextItemWidth(120);
        int mode = m_GizmoMode;
        if (ImGui::Combo(u8"로컬 기즈모", &mode, m_EditPhysics && m_EditConstraints ? u8"이동 (W)\0회전 (E)\0" : u8"이동 (W)\0회전 (E)\0크기 (R)\0"))
        {
            CancelInteraction();
            m_GizmoMode = mode;
        }
        ImGui::SameLine();
        ImGui::TextDisabled("(?)");
        if (ImGui::IsItemHovered())
        {
            ImGui::SetTooltip(u8"뷰포트: 우클릭 + WASD 이동, Q/E 아래/위, Shift 빠르게\n휠: 앞뒤 이동 | F: 메시 맞춤 | Esc: 취소\n그래프: 노드 클릭 선택, 휠 확대/축소, 가운데 버튼 드래그 이동");
        }
        // 같은 프레임에 마우스 오른쪽 버튼과 W/E가 들어올 수 있으므로 모드와 버튼을 함께 검사한다.
        if (m_Focused && !ImGui::GetIO().WantTextInput && m_DragAxis < 0 && !m_Flying &&
            !ImGui::IsMouseDown(ImGuiMouseButton_Right))
        {
            if (ImGui::IsKeyPressed(ImGuiKey_F, false))
            {
                CancelInteraction();
                ResetCamera();
            }
            int keyMode = m_GizmoMode;
            if (ImGui::IsKeyPressed(ImGuiKey_W, false))
            {
                keyMode = 0;
            }
            if (ImGui::IsKeyPressed(ImGuiKey_E, false))
            {
                keyMode = 1;
            }
            if (ImGui::IsKeyPressed(ImGuiKey_R, false) && !(m_EditPhysics && m_EditConstraints))
            {
                keyMode = 2;
            }
            if (keyMode != m_GizmoMode)
            {
                CancelInteraction();
                m_GizmoMode = keyMode;
            }
        }
        const ImVec2 available = ImGui::GetContentRegionAvail();
        if (ImGui::BeginTable("EditorLayout", 3, ImGuiTableFlags_Resizable | ImGuiTableFlags_BordersInnerV,
                ImVec2(0, available.y)))
        {
            ImGui::TableSetupColumn(u8"탐색", ImGuiTableColumnFlags_WidthFixed, 360);
            ImGui::TableSetupColumn(u8"뷰포트", ImGuiTableColumnFlags_WidthStretch);
            ImGui::TableSetupColumn(u8"세부 설정", ImGuiTableColumnFlags_WidthFixed, 320);
            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0);
            ImGui::BeginChild(u8"탐색", ImVec2(0, available.y - 4), false, ImGuiWindowFlags_NoScrollbar);
            DrawNavigation();
            ImGui::EndChild();
            // 현재 프레임에 편집 결과가 표시되도록 세부 설정 패널을 뷰포트보다 먼저 그린다.
            ImGui::TableSetColumnIndex(2);
            ImGui::BeginChild(u8"세부 설정", ImVec2(0, available.y - 4), true);
            DrawDetails();
            ImGui::EndChild();
            ImGui::TableSetColumnIndex(1);
            ImGui::BeginChild("PreviewViewport", ImVec2(0, available.y - 4), false,
                ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);
            const ImVec2 size = ImGui::GetContentRegionAvail();
            if (size.x > 1 && size.y > 1)
            {
                ImGui::Image(static_cast<ImTextureID>(m_Image), size);
                const ImVec2 origin = ImGui::GetItemRectMin();
                const ImVec2 mouse = ImGui::GetIO().MousePos;
                const bool hovered = ImGui::IsItemHovered();
                UpdateGizmoDrag(mouse.x - origin.x, mouse.y - origin.y);
                UpdateCamera(hovered);
                RenderPreview(context, size.y / size.x);
                UpdateBoneWorld();
                m_ViewportHeight = size.y;
                m_Overlay.Begin(m_Camera, size.x, size.y);
                DrawPhysicsBodies();
                DrawSkeleton();
                if (hovered && !m_Flying && m_DragAxis < 0 && ImGui::IsMouseClicked(ImGuiMouseButton_Left))
                {
                    m_ClickX = mouse.x - origin.x;
                    m_ClickY = mouse.y - origin.y;
                    m_HasPickRequest = m_Overlay.RequestPick(m_ClickX, m_ClickY);
                    if (m_HasPickRequest)
                    {
                        m_PickPose = m_EditPose.empty() ? animation.GetPose() : m_EditPose;
                    }
                }
                m_Overlay.Render(context, m_Display);
                DrawOrientationAxes(origin.x, origin.y, size.x, size.y);
            }
            else
            {
                CancelInteraction();
            }
            ImGui::EndChild();
            ImGui::EndTable();
        }
    }
    else
    {
        CancelInteraction();
    }
    ImGui::End();
    if (!m_Open)
    {
        CancelInteraction();
    }
}

void SkeletalMeshViewer::RenderPreview(GraphicsContext& context, float aspect)
{
    m_Camera.SetPerspectiveMatrix(XM_PIDIV4, aspect, m_Radius * 0.01f, m_Radius * 100);
    m_Camera.Update();
    m_Mesh->UpdateRenderData(context, m_EditPose.empty() ? nullptr : &m_EditPose);
    context.TransitionResource(m_Color, D3D12_RESOURCE_STATE_RENDER_TARGET);
    context.TransitionResource(m_Depth, D3D12_RESOURCE_STATE_DEPTH_WRITE, true);
    context.ClearColor(m_Color);
    context.ClearDepth(m_Depth);
    Renderer::MeshSorter sorter(Renderer::MeshSorter::kDefault);
    sorter.SetCamera(m_Camera);
    sorter.SetDepthStencilTarget(m_Depth);
    sorter.AddRenderTarget(m_Color);
    sorter.SetCommonTextures(m_Textures);
    m_Mesh->Render(sorter);
    sorter.Sort();
    GlobalConstants globals{};
    globals.SunShadowMatrix = Matrix4(Vector4(kZero), Vector4(kZero), Vector4(kZero), Vector4(0.5f, 0.5f, 1, 1));
    globals.SunDirection = Normalize(Vector3(0.5f, 1, 0.5f));
    globals.SunIntensity = Vector3(3, 3, 3);
    sorter.RenderMeshes(Renderer::MeshSorter::kTransparent, context, globals);

    context.TransitionResource(m_Color, D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE);
    context.TransitionResource(m_Display, D3D12_RESOURCE_STATE_RENDER_TARGET, true);
    context.SetRootSignature(m_PresentRoot);
    context.SetPipelineState(m_PresentPSO);
    context.SetPrimitiveTopology(D3D_PRIMITIVE_TOPOLOGY_TRIANGLELIST);
    context.SetRenderTarget(m_Display.GetRTV());
    context.SetViewportAndScissor(0, 0, PreviewResolution, PreviewResolution);
    context.SetDynamicDescriptor(0, 0, m_Color.GetSRV());
    context.Draw(3);
    context.TransitionResource(m_Display, D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE);
}
