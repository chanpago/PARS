#include "SkeletalMeshViewer.h"
#include "Components/SkeletalMeshComponent.h"
#include "Renderer.h"
#include "ConstantBuffers.h"
#include "BufferManager.h"
#include "GraphicsCommon.h"
#include "ImGuiOverlay.h"
#include "imgui.h"
#include "CompiledShaders/MeshPreviewVS.h"
#include "CompiledShaders/MeshPreviewPS.h"
#include <cmath>
#include <cfloat>

using namespace Math;
using namespace Graphics;

namespace { constexpr uint32_t PreviewResolution = 1024; }

bool SkeletalMeshViewer::Initialize(const std::shared_ptr<const Model>* models,
    const char* const* names, uint32_t count)
{
    if (!count || !models[0]) return false;
    auto* actor = m_World.SpawnActor<PARS::Actor>("MeshPreview");
    m_Mesh = &actor->CreateDefaultSubobject<PARS::SkeletalMeshComponent>("SkeletalMeshComponent");
    actor->SetRootComponent(*m_Mesh);
    if (!m_Mesh->SetSkeletalMesh(models[0])) return false;
    for (uint32_t i = 0; i < count; ++i)
        if (!m_Mesh->GetAnimInstance().AddAnimation(names[i], models[i])) return false;
    m_Mesh->GetAnimInstance().PlayAnimation(0);
    BuildSkeleton();
    m_World.BeginPlay();
    m_World.Tick(0);
    m_Radius = Max(0.1f, float(m_Mesh->GetLocalBounds().GetRadius()));
    ResetCamera();

    // Fixed backing textures keep resize/close/reopen free of descriptor and
    // resource lifetime hazards. The camera uses the displayed image's aspect.
    m_Color.SetClearColor(Color(0.025f, 0.035f, 0.055f, 1));
    m_Color.Create(L"Mesh preview HDR", PreviewResolution, PreviewResolution, 1, g_SceneColorBuffer.GetFormat());
    m_Display.Create(L"Mesh preview image", PreviewResolution, PreviewResolution, 1, DXGI_FORMAT_R8G8B8A8_UNORM);
    m_Depth.Create(L"Mesh preview depth", PreviewResolution, PreviewResolution, g_SceneDepthBuffer.GetFormat());
    m_AmbientOcclusion.Create(L"Mesh preview AO", PreviewResolution, PreviewResolution, 1, DXGI_FORMAT_R8_UNORM);
    m_Unshadowed.Create(L"Mesh preview shadow", 1, 1, DXGI_FORMAT_D32_FLOAT);

    // Keep IBL/material lighting, with independent AO/shadow inputs. This table
    // shares the renderer heap so material and skinning descriptors stay valid.
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
    ImGuiOverlay::UnregisterTexture(m_Image);
    m_Image = 0;
    m_Overlay.Cleanup();
    m_Mesh = nullptr;
    m_World.Clear();
    m_EditPose.clear();
    m_PickPose.clear();
    m_SelectedBone = -1;
    m_Color.Destroy();
    m_Display.Destroy();
    m_Depth.Destroy();
    m_AmbientOcclusion.Destroy();
    m_Unshadowed.Destroy();
}

void SkeletalMeshViewer::Update(float deltaTime)
{
    if (m_Open && m_Mesh) m_World.Tick(deltaTime);
}

void SkeletalMeshViewer::ResetCamera()
{
    m_Target = m_Mesh->GetLocalBounds().GetCenter();
    m_Distance = m_Radius * 3.4f;
    m_Yaw = 0;
    m_Pitch = 0.12f;
}

void SkeletalMeshViewer::RenderUI(GraphicsContext& context)
{
    m_Focused = false;
    if (!m_Open || !m_Mesh) { CancelInteraction(); return; }
    const auto display = ImGui::GetIO().DisplaySize;
    ImGui::SetNextWindowPos(ImVec2(display.x * 0.5f, display.y * 0.5f), ImGuiCond_FirstUseEver, ImVec2(0.5f, 0.5f));
    ImGui::SetNextWindowSize(ImVec2(Min(1050.0f, display.x * 0.88f), Min(740.0f, display.y * 0.9f)), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSizeConstraints(ImVec2(640, 420), ImVec2(FLT_MAX, FLT_MAX));
    if (m_FocusRequested) { ImGui::SetNextWindowFocus(); m_FocusRequested = false; }
    if (ImGui::Begin("Skeletal Mesh Viewer", &m_Open, ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse))
    {
        m_Focused = ImGui::IsWindowFocused(ImGuiFocusedFlags_RootAndChildWindows);
        if (!m_Focused) CancelInteraction();
        uint32_t pickedId;
        if (m_Overlay.PollPick(pickedId) && m_Focused) HandlePick(pickedId);
        auto& animation = m_Mesh->GetAnimInstance();
        ImGui::TextUnformatted("Mixamo character");
        ImGui::SameLine();
        ImGui::TextDisabled("| %u joints", m_Mesh->GetSkeletalMesh()->m_NumJoints);
        ImGui::SetNextItemWidth(130);
        if (ImGui::BeginCombo("##Animation", animation.GetAnimationName(animation.GetCurrentAnimationIndex()).c_str()))
        {
            for (uint32_t i = 0; i < animation.GetAnimationCount(); ++i)
                if (ImGui::Selectable(animation.GetAnimationName(i).c_str(), i == animation.GetCurrentAnimationIndex()))
                {
                    CancelInteraction();
                    m_EditPose.clear();
                    animation.PlayAnimation(i);
                }
            ImGui::EndCombo();
        }
        ImGui::SameLine();
        bool playing = animation.IsPlaying();
        if (ImGui::Checkbox("Play", &playing))
        {
            CancelInteraction();
            if (playing) m_EditPose.clear();
            animation.SetPlaying(playing);
        }
        ImGui::SameLine();
        if (ImGui::Button("Reference pose"))
        {
            CancelInteraction();
            animation.SetPlaying(false);
            const auto& model = *m_Mesh->GetSkeletalMesh();
            m_EditPose.assign(model.m_SceneGraph.get(), model.m_SceneGraph.get() + model.m_NumNodes);
        }
        ImGui::SameLine();
        if (ImGui::Button("Animation pose")) { CancelInteraction(); m_EditPose.clear(); }
        ImGui::SameLine();
        if (ImGui::Button("Frame mesh")) { CancelInteraction(); ResetCamera(); }
        if (ImGui::Checkbox("Bones", &m_ShowBones)) CancelInteraction();
        ImGui::SameLine();
        ImGui::SetNextItemWidth(120);
        int mode = m_GizmoMode;
        if (ImGui::Combo("Local gizmo", &mode, "Move (W)\0Rotate (E)\0Scale (R)\0"))
        { CancelInteraction(); m_GizmoMode = mode; }
        ImGui::SameLine();
        ImGui::TextDisabled("%s", m_EditPose.empty() ? "Animation pose" : "Preview pose (Play to restore animation)");
        ImGui::TextDisabled("Click: select | RMB: orbit | Wheel: zoom | F: frame | Esc: cancel drag");
        if (m_Focused && !ImGui::GetIO().WantTextInput && m_DragAxis < 0 && !m_Orbiting)
        {
            if (ImGui::IsKeyPressed(ImGuiKey_F, false)) { CancelInteraction(); ResetCamera(); }
            int keyMode = m_GizmoMode;
            if (ImGui::IsKeyPressed(ImGuiKey_W, false)) keyMode = 0;
            if (ImGui::IsKeyPressed(ImGuiKey_E, false)) keyMode = 1;
            if (ImGui::IsKeyPressed(ImGuiKey_R, false)) keyMode = 2;
            if (keyMode != m_GizmoMode) { CancelInteraction(); m_GizmoMode = keyMode; }
        }
        const ImVec2 available = ImGui::GetContentRegionAvail();
        ImGui::BeginChild("Skeleton", ImVec2(Min(280.0f, available.x * .36f), 0), true, ImGuiWindowFlags_HorizontalScrollbar);
        ImGui::TextUnformatted("Bone hierarchy");
        if (m_SelectedBone >= 0)
        {
            const auto& pose = m_EditPose.empty() ? animation.GetPose() : m_EditPose;
            const auto& node = pose[m_SelectedBone];
            ImGui::TextWrapped("%s", m_Mesh->GetSkeletalMesh()->m_NodeNames[m_SelectedBone].c_str());
            ImGui::TextUnformatted("Local position / scale");
            ImGui::Text("%.2f  %.2f  %.2f", float(node.xform.GetW().GetX()), float(node.xform.GetW().GetY()), float(node.xform.GetW().GetZ()));
            ImGui::Text("%.2f  %.2f  %.2f", node.scale.x, node.scale.y, node.scale.z);
        }
        ImGui::Separator();
        ImGui::PushStyleVar(ImGuiStyleVar_IndentSpacing, 14.0f);
        for (int32_t root : m_BoneRoots) DrawBoneTree(root);
        ImGui::PopStyleVar();
        m_RevealSelection = false;
        ImGui::EndChild();
        ImGui::SameLine();
        ImGui::BeginChild("PreviewViewport", ImVec2(0, 0), false,
            ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);
        const ImVec2 size = ImGui::GetContentRegionAvail();
        if (size.x > 1 && size.y > 1)
        {
            ImGui::Image(static_cast<ImTextureID>(m_Image), size);
            const ImVec2 origin = ImGui::GetItemRectMin();
            const ImVec2 mouse = ImGui::GetIO().MousePos;
            const bool hovered = ImGui::IsItemHovered();
            UpdateGizmoDrag(mouse.x - origin.x, mouse.y - origin.y);
            if (hovered && m_DragAxis < 0 && ImGui::IsMouseClicked(ImGuiMouseButton_Right))
            { CancelInteraction(); m_Orbiting = true; }
            if (!ImGui::IsMouseDown(ImGuiMouseButton_Right)) m_Orbiting = false;
            if (m_Orbiting)
            {
                const auto delta = ImGui::GetIO().MouseDelta;
                m_Yaw = std::remainder(m_Yaw - delta.x * 0.008f, XM_2PI);
                m_Pitch = Clamp(m_Pitch + delta.y * 0.008f, -1.4f, 1.4f);
            }
            if (hovered && m_DragAxis < 0 && ImGui::GetIO().MouseWheel != 0)
            {
                m_Overlay.CancelPick(); m_HasPickRequest = false;
                m_Distance = Clamp(m_Distance * std::exp(-ImGui::GetIO().MouseWheel * 0.15f), m_Radius * 0.4f, m_Radius * 15);
            }
            RenderPreview(context, size.y / size.x);
            UpdateBoneWorld();
            m_ViewportHeight = size.y;
            m_Overlay.Begin(m_Camera, size.x, size.y);
            DrawSkeleton();
            if (hovered && !m_Orbiting && m_DragAxis < 0 && ImGui::IsMouseClicked(ImGuiMouseButton_Left))
            {
                m_ClickX = mouse.x - origin.x; m_ClickY = mouse.y - origin.y;
                m_HasPickRequest = m_Overlay.RequestPick(m_ClickX, m_ClickY);
                if (m_HasPickRequest) m_PickPose = m_EditPose.empty() ? animation.GetPose() : m_EditPose;
            }
            m_Overlay.Render(context, m_Display);
        }
        ImGui::EndChild();
    }
    else CancelInteraction();
    ImGui::End();
    if (!m_Open) CancelInteraction();
}

void SkeletalMeshViewer::RenderPreview(GraphicsContext& context, float aspect)
{
    m_Camera.SetPerspectiveMatrix(XM_PIDIV4, aspect, m_Radius * 0.01f, m_Radius * 100);
    const Vector3 direction(std::sin(m_Yaw) * std::cos(m_Pitch), std::sin(m_Pitch), std::cos(m_Yaw) * std::cos(m_Pitch));
    m_Camera.SetEyeAtUp(m_Target + direction * m_Distance, m_Target, Vector3(kYUnitVector));
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
