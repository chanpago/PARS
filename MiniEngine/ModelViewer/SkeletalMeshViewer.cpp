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
        // An ImGui Image does not take focus on RMB. Acquire the viewport before
        // testing focus so returning from another panel works on the first press.
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
        // Rotate about the camera's own position; the mesh is only a framing target.
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
                // ImGui uses elapsed wall time, independently of preview animation playback.
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

void SkeletalMeshViewer::RenderUI(GraphicsContext& context)
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
    if (ImGui::Begin("Skeletal Mesh Viewer", &m_Open, ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse))
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
        ImGui::TextUnformatted("Mixamo character");
        ImGui::SameLine();
        ImGui::TextDisabled("| %u bones | %zu bodies | %zu constraints", m_Mesh->GetSkeletalMesh()->m_NumJoints,
            m_PhysicsAsset.bodies.size(), m_PhysicsAsset.constraints.size());
        ImGui::SetNextItemWidth(130);
        if (ImGui::BeginCombo("##Animation", animation.GetAnimationName(animation.GetCurrentAnimationIndex()).c_str()))
        {
            for (uint32_t i = 0; i < animation.GetAnimationCount(); ++i)
            {
                if (ImGui::Selectable(animation.GetAnimationName(i).c_str(), i == animation.GetCurrentAnimationIndex()))
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
        if (ImGui::Checkbox("Play", &playing))
        {
            CancelInteraction();
            if (playing)
            {
                m_EditPose.clear();
            }
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
        if (ImGui::Button("Animation pose"))
        {
            CancelInteraction();
            m_EditPose.clear();
        }
        ImGui::SameLine();
        if (ImGui::Button("Frame mesh"))
        {
            CancelInteraction();
            ResetCamera();
        }
        if (ImGui::Checkbox("Bones", &m_ShowBones))
        {
            CancelInteraction();
        }
        ImGui::SameLine();
        if (ImGui::Checkbox("Bodies", &m_ShowBodies))
        {
            CancelInteraction();
        }
        ImGui::SameLine();
        ImGui::SetNextItemWidth(110);
        ImGui::SliderFloat("Opacity", &m_BodyOpacity, .05f, .75f, "%.2f");
        ImGui::SameLine();
        if (ImGui::Checkbox("Constraints", &m_ShowConstraints))
        {
            CancelInteraction();
        }
        ImGui::SameLine();
        if (ImGui::Checkbox("Limits", &m_ShowLimits))
        {
            CancelInteraction();
        }
        if (ImGui::IsItemHovered())
        {
            ImGui::SetTooltip("Red cone: allowed swing. Green sector: allowed rotation angle. Yellow: current direction / twist. Click a limit to select its constraint.");
        }
        ImGui::SameLine();
        ImGui::SetNextItemWidth(120);
        int mode = m_GizmoMode;
        if (ImGui::Combo("Local gizmo", &mode, m_EditPhysics && m_EditConstraints ? "Move (W)\0Rotate (E)\0" : "Move (W)\0Rotate (E)\0Scale (R)\0"))
        {
            CancelInteraction();
            m_GizmoMode = mode;
        }
        ImGui::SameLine();
        ImGui::TextDisabled("(?)");
        if (ImGui::IsItemHovered())
        {
            ImGui::SetTooltip("Viewport: RMB + WASD move, Q/E down/up, Shift fast\nWheel: dolly | F: frame mesh | Esc: cancel\nGraph: click a node to select, wheel to zoom, MMB drag to pan");
        }
        // Check the button as well as the mode: RMB and W/E may arrive in the same frame.
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
            ImGui::TableSetupColumn("Navigation", ImGuiTableColumnFlags_WidthFixed, 360);
            ImGui::TableSetupColumn("Viewport", ImGuiTableColumnFlags_WidthStretch);
            ImGui::TableSetupColumn("Details", ImGuiTableColumnFlags_WidthFixed, 320);
            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0);
            ImGui::BeginChild("Navigation", ImVec2(0, available.y - 4), false, ImGuiWindowFlags_NoScrollbar);
            DrawNavigation();
            ImGui::EndChild();
            // Draw details before the viewport so edits appear in this frame.
            ImGui::TableSetColumnIndex(2);
            ImGui::BeginChild("Details", ImVec2(0, available.y - 4), true);
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
