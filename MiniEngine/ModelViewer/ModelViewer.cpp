//
// Copyright (c) Microsoft. All rights reserved.
// This code is licensed under the MIT License (MIT).
// THIS CODE IS PROVIDED *AS IS* WITHOUT WARRANTY OF
// ANY KIND, EITHER EXPRESS OR IMPLIED, INCLUDING ANY
// IMPLIED WARRANTIES OF FITNESS FOR A PARTICULAR
// PURPOSE, MERCHANTABILITY, OR NON-INFRINGEMENT.
//
// Developed by Minigraph
//
// Author:  James Stanard
//

#include "GameCore.h"
#include "CameraController.h"
#include "BufferManager.h"
#include "Camera.h"
#include "CommandContext.h"
#include "TemporalEffects.h"
#include "MotionBlur.h"
#include "DepthOfField.h"
#include "PostEffects.h"
#include "SSAO.h"
#include "FXAA.h"
#include "SystemTime.h"
#include "TextRenderer.h"
#include "ParticleEffectManager.h"
#include "GameInput.h"
#include "SponzaRenderer.h"
#include "glTF.h"
#include "Renderer.h"
#include "Model.h"
#include "ModelLoader.h"
#include "ShadowCamera.h"
#include "Display.h"
#include "imgui.h"
#if defined(_M_X64)
#include "PhysicsDemo.h"
#endif

#define LEGACY_RENDERER

using namespace GameCore;
using namespace Math;
using namespace Graphics;
using namespace std;

using Renderer::MeshSorter;

namespace
{
    const wchar_t* const kPreviewFiles[] = {
        L"Idle/Idle.gltf", L"Walking/Walking.gltf", L"SlowRun/SlowRun.gltf"
    };
    const char* const kPreviewNames[] = { "Idle", "Walking", "Slow Run" };
    constexpr uint32_t kPreviewCount = 3;

    // Find the project assets from either the executable or the working directory.
    // This also works when launching directly from the build output folder.
    std::wstring FindPreviewAssetRoot()
    {
        wchar_t path[32768];
        for (int location = 0; location < 2; ++location)
        {
            DWORD length = location == 0 ? GetModuleFileNameW(nullptr, path, _countof(path))
                                        : GetCurrentDirectoryW(_countof(path), path);
            if (length == 0 || length >= _countof(path))
                continue;
            std::wstring directory(path, length);
            if (location == 0)
                directory = directory.substr(0, directory.find_last_of(L"/\\"));
            while (!directory.empty())
            {
                const std::wstring root = directory + L"/Assets/Characters/Mixamo/Converted/";
                bool found = true;
                for (const wchar_t* file : kPreviewFiles)
                {
                    const DWORD attributes = GetFileAttributesW((root + file).c_str());
                    found &= attributes != INVALID_FILE_ATTRIBUTES && !(attributes & FILE_ATTRIBUTE_DIRECTORY);
                }
                if (found)
                    return root;
                const size_t separator = directory.find_last_of(L"/\\");
                if (separator == std::wstring::npos)
                    break;
                directory.resize(separator);
            }
        }
        return L"";
    }
}

class ModelViewer : public GameCore::IGameApp
{
public:

    ModelViewer( void ) {}

    virtual void Startup( void ) override;
    virtual void Cleanup( void ) override;

    virtual void Update( float deltaT ) override;
    virtual void RenderScene( void ) override;
    virtual void RenderUI(GraphicsContext&) override;
    virtual bool UsesImGui() const override { return true; }

private:

    void LoadAnimationPreview(bool forceRebuild);
    void SelectPreview(uint32_t index);
    void ResetPreviewCamera();
    ModelInstance& ActiveModel() { return m_PreviewMode ? m_PreviewModels[m_PreviewIndex] : m_ModelInst; }

    Camera m_Camera;
    unique_ptr<CameraController> m_CameraController;

    D3D12_VIEWPORT m_MainViewport;
    D3D12_RECT m_MainScissor;

    ModelInstance m_ModelInst;
    ModelInstance m_PreviewModels[kPreviewCount];
    float m_PreviewDurations[kPreviewCount] = {};
    uint32_t m_PreviewIndex = 0;
    float m_PreviewElapsed = 0.0f;
    bool m_PreviewMode = false;
    bool m_PreviewReady = false;
    bool m_PreviewPlaying = true;
    bool m_PreviewAutoCycle = true;
    std::string m_PreviewError;
    ShadowCamera m_SunShadowCamera;
    bool m_SponzaStarted = false;
#if defined(_M_X64)
    unique_ptr<PhysicsDemo> m_PhysicsDemo;
#endif
};

CREATE_APPLICATION( ModelViewer )

ExpVar g_SunLightIntensity("Viewer/Lighting/Sun Light Intensity", 4.0f, 0.0f, 16.0f, 0.1f);
NumVar g_SunOrientation("Viewer/Lighting/Sun Orientation", -0.5f, -100.0f, 100.0f, 0.1f );
NumVar g_SunInclination("Viewer/Lighting/Sun Inclination", 0.75f, 0.0f, 1.0f, 0.01f );

void ChangeIBLSet(EngineVar::ActionType);
void ChangeIBLBias(EngineVar::ActionType);

DynamicEnumVar g_IBLSet("Viewer/Lighting/Environment", ChangeIBLSet);
std::vector<std::pair<TextureRef, TextureRef>> g_IBLTextures;
NumVar g_IBLBias("Viewer/Lighting/Gloss Reduction", 2.0f, 0.0f, 10.0f, 1.0f, ChangeIBLBias);

void ChangeIBLSet(EngineVar::ActionType)
{
    int setIdx = g_IBLSet - 1;
    if (setIdx < 0)
    {
        Renderer::SetIBLTextures(nullptr, nullptr);
    }
    else
    {
        auto texturePair = g_IBLTextures[setIdx];
        Renderer::SetIBLTextures(texturePair.first, texturePair.second);
    }
}

void ChangeIBLBias(EngineVar::ActionType)
{
    Renderer::SetIBLBias(g_IBLBias);
}

#include <direct.h> // for _getcwd() to check data root path

void LoadIBLTextures()
{
    char CWD[256];
    _getcwd(CWD, 256);

    Utility::Printf("Loading IBL environment maps\n");

    WIN32_FIND_DATA ffd;
    HANDLE hFind = FindFirstFile(L"Textures/*_diffuseIBL.dds", &ffd);

    g_IBLSet.AddEnum(L"None");

    if (hFind != INVALID_HANDLE_VALUE) do
    {
        if (ffd.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY)
            continue;

       std::wstring diffuseFile = ffd.cFileName;
       std::wstring baseFile = diffuseFile; 
       baseFile.resize(baseFile.rfind(L"_diffuseIBL.dds"));
       std::wstring specularFile = baseFile + L"_specularIBL.dds";

       TextureRef diffuseTex = TextureManager::LoadDDSFromFile(L"Textures/" + diffuseFile);
       if (diffuseTex.IsValid())
       {
           TextureRef specularTex = TextureManager::LoadDDSFromFile(L"Textures/" + specularFile);
           if (specularTex.IsValid())
           {
               g_IBLSet.AddEnum(baseFile);
               g_IBLTextures.push_back(std::make_pair(diffuseTex, specularTex));
           }
       }
    }
    while (FindNextFile(hFind, &ffd) != 0);

    FindClose(hFind);

    Utility::Printf("Found %u IBL environment map sets\n", g_IBLTextures.size());

    if (g_IBLTextures.size() > 0)
        g_IBLSet.Increment();
}

void ModelViewer::Startup( void )
{
    MotionBlur::Enable = true;
    TemporalEffects::EnableTAA = true;
    FXAA::Enable = false;
    PostEffects::EnableHDR = true;
    PostEffects::EnableAdaptation = true;
    SSAO::Enable = true;

    Renderer::Initialize();

    // The default scene combines the physics map and the animated character.
    // -physics 0 selects Sponza; -model still selects a standalone model.
    uint32_t physicsMode = 1;
    const bool hasPhysicsOption = CommandLineArgs::GetInteger(L"physics", physicsMode);
    std::wstring gltfFileName;
    const bool hasModel = CommandLineArgs::GetString(L"model", gltfFileName);
    bool forceRebuild = false;
    uint32_t rebuildValue;
    if (CommandLineArgs::GetInteger(L"rebuild", rebuildValue))
        forceRebuild = rebuildValue != 0;
#if defined(_M_X64)
    if (physicsMode && !hasModel)
    {
        m_PhysicsDemo.reset(new PhysicsDemo());
        if (m_PhysicsDemo->Startup())
        {
            MotionBlur::Enable = false;
            TemporalEffects::EnableTAA = false; // This simple pass has no object velocity buffer.
            PostEffects::EnableAdaptation = false;
            SSAO::Enable = false;
            LoadIBLTextures();
            LoadAnimationPreview(forceRebuild);
            return;
        }
        Utility::Printf("Physics demo initialization failed; loading the model viewer.\n");
        m_PhysicsDemo.reset();
    }
#endif

    LoadIBLTextures();

    if (!hasModel && !hasPhysicsOption)
    {
        LoadAnimationPreview(forceRebuild);
        return;
    }

    if (!hasModel)
    {
#ifdef LEGACY_RENDERER
        Sponza::Startup(m_Camera);
        m_SponzaStarted = true;
#else
        m_ModelInst = Renderer::LoadModel(L"Sponza/PBR/sponza2.gltf", forceRebuild);
        m_ModelInst.Resize(100.0f * m_ModelInst.GetRadius());
        OrientedBox obb = m_ModelInst.GetBoundingBox();
        float modelRadius = Length(obb.GetDimensions()) * 0.5f;
        const Vector3 eye = obb.GetCenter() + Vector3(modelRadius * 0.5f, 0.0f, 0.0f);
        m_Camera.SetEyeAtUp( eye, Vector3(kZero), Vector3(kYUnitVector) );
#endif
    }
    else
    {
        m_ModelInst = Renderer::LoadModel(gltfFileName, forceRebuild);
        m_ModelInst.LoopAllAnimations();
        m_ModelInst.Resize(10.0f);

        MotionBlur::Enable = false;
        // Skinned meshes do not yet write object velocities for temporal reprojection.
        if (m_ModelInst.GetNumAnimations() > 0)
        {
            TemporalEffects::EnableTAA = false;
            FXAA::Enable = true;
        }
    }

    m_Camera.SetZRange(1.0f, 10000.0f);
    if (gltfFileName.size() == 0)
        m_CameraController.reset(new FlyingFPSCamera(m_Camera, Vector3(kYUnitVector)));
    else
        m_CameraController.reset(new OrbitCamera(m_Camera, m_ModelInst.GetBoundingSphere(), Vector3(kYUnitVector)));
}

void ModelViewer::LoadAnimationPreview(bool forceRebuild)
{
    m_PreviewMode = true;
    MotionBlur::Enable = false;
    TemporalEffects::EnableTAA = false;
    FXAA::Enable = true;
    ResetPreviewCamera();

    const std::wstring root = FindPreviewAssetRoot();
    if (root.empty())
    {
        m_PreviewError = "Cannot find Idle, Walking and SlowRun in Assets/Characters/Mixamo/Converted.";
        return;
    }

    // Keep each instance alive so changing clips never reallocates GPU buffers.
    for (uint32_t i = 0; i < kPreviewCount; ++i)
    {
        auto model = Renderer::LoadModel(root + kPreviewFiles[i], forceRebuild);
        if (!model || model->m_NumAnimations == 0 || model->m_Animations[0].duration <= 0.0f)
        {
            m_PreviewError = std::string("Could not load animation: ") + kPreviewNames[i];
            return;
        }
        m_PreviewModels[i] = model;
#if defined(_M_X64)
        if (m_PhysicsDemo)
            // The converted glTF is already in metres. Keep the same fixed
            // placement for every clip, beside the falling box at the origin.
            m_PreviewModels[i].SetPosition(Vector3(2.0f, 0.0f, 0.0f));
        else
#endif
            m_PreviewModels[i].Resize(10.0f);
        const float duration = model->m_Animations[0].duration;
        // Show complete loops for at least four seconds before changing clips.
        m_PreviewDurations[i] = duration * ceilf(4.0f / duration);
    }
    m_PreviewReady = true;
    SelectPreview(0);
    ResetPreviewCamera();
}

void ModelViewer::SelectPreview(uint32_t index)
{
    m_PreviewIndex = index % kPreviewCount;
    m_PreviewElapsed = 0.0f;
    m_PreviewModels[m_PreviewIndex].LoopAllAnimations();
}

void ModelViewer::ResetPreviewCamera()
{
    m_Camera.SetZRange(0.1f, 1000.0f);
#if defined(_M_X64)
    if (m_PhysicsDemo)
    {
        m_Camera.SetZRange(0.1f, 100.0f);
        m_Camera.SetEyeAtUp(Vector3(5, 3.2f, 7), Vector3(1, 1.8f, 0), Vector3(kYUnitVector));
        auto controller = new FlyingFPSCamera(m_Camera, Vector3(kYUnitVector));
        controller->SetMoveSpeed(5.0f);
        m_CameraController.reset(controller);
        return;
    }
#endif
    if (!m_PreviewReady)
    {
        m_CameraController.reset(new OrbitCamera(m_Camera, BoundingSphere(Vector3(0, 8, 0), 14.0f)));
        return;
    }
    // Leave room for the hands and feet throughout all three animations.
    BoundingSphere bounds = m_PreviewModels[0].GetBoundingSphere();
    for (uint32_t i = 1; i < kPreviewCount; ++i)
        bounds = bounds.Union(m_PreviewModels[i].GetBoundingSphere());
    m_CameraController.reset(new OrbitCamera(m_Camera,
        BoundingSphere(bounds.GetCenter(), bounds.GetRadius() * 1.4f), Vector3(kYUnitVector)));
}

void ModelViewer::Cleanup( void )
{
#if defined(_M_X64)
    if (m_PhysicsDemo)
    {
        m_PhysicsDemo->Cleanup();
        m_PhysicsDemo.reset();
    }
#endif
    m_ModelInst = nullptr;
    for (auto& model : m_PreviewModels)
        model = nullptr;

    g_IBLTextures.clear();

#ifdef LEGACY_RENDERER
    if (m_SponzaStarted)
        Sponza::Cleanup();
#endif

    Renderer::Shutdown();
}

namespace Graphics
{
    extern EnumVar DebugZoom;
}

void ModelViewer::Update( float deltaT )
{
    ScopedTimer _prof(L"Update State");

    if (GameInput::IsFirstPressed(GameInput::kLShoulder))
        DebugZoom.Decrement();
    else if (GameInput::IsFirstPressed(GameInput::kRShoulder))
        DebugZoom.Increment();

#if defined(_M_X64)
    if (m_PhysicsDemo)
        m_PhysicsDemo->Update(deltaT);
#endif
    m_Camera.SetAspectRatio(float(g_SceneColorBuffer.GetHeight()) / float(g_SceneColorBuffer.GetWidth()));

    // Only mouse look requires a right-button drag. Keyboard/gamepad input is
    // updated every frame, after GameInput has masked input captured by the UI.
    m_CameraController->EnableMouseInput(GameInput::IsPressed(GameInput::kMouse1));
    m_CameraController->Update(deltaT);

    GraphicsContext& gfxContext = GraphicsContext::Begin(L"Scene Update");

    float animationDelta = deltaT;
    if (m_PreviewMode)
    {
        // Pausing also freezes the automatic switch timer; camera input still works.
        animationDelta = m_PreviewPlaying ? Clamp(deltaT, 0.0f, 0.1f) : 0.0f;
        if (m_PreviewReady && m_PreviewAutoCycle)
        {
            m_PreviewElapsed += animationDelta;
            if (m_PreviewElapsed >= m_PreviewDurations[m_PreviewIndex])
                SelectPreview(m_PreviewIndex + 1);
        }
    }
    ActiveModel().Update(gfxContext, animationDelta);

    gfxContext.Finish();

    // We use viewport offsets to jitter sample positions from frame to frame (for TAA.)
    // D3D has a design quirk with fractional offsets such that the implicit scissor
    // region of a viewport is floor(TopLeftXY) and floor(TopLeftXY + WidthHeight), so
    // having a negative fractional top left, e.g. (-0.25, -0.25) would also shift the
    // BottomRight corner up by a whole integer.  One solution is to pad your viewport
    // dimensions with an extra pixel.  My solution is to only use positive fractional offsets,
    // but that means that the average sample position is +0.5, which I use when I disable
    // temporal AA.
    TemporalEffects::GetJitterOffset(m_MainViewport.TopLeftX, m_MainViewport.TopLeftY);

    m_MainViewport.Width = (float)g_SceneColorBuffer.GetWidth();
    m_MainViewport.Height = (float)g_SceneColorBuffer.GetHeight();
    m_MainViewport.MinDepth = 0.0f;
    m_MainViewport.MaxDepth = 1.0f;

    m_MainScissor.left = 0;
    m_MainScissor.top = 0;
    m_MainScissor.right = (LONG)g_SceneColorBuffer.GetWidth();
    m_MainScissor.bottom = (LONG)g_SceneColorBuffer.GetHeight();
}

void ModelViewer::RenderUI(GraphicsContext&)
{
#if defined(_M_X64)
    if (m_PhysicsDemo)
    {
        m_PhysicsDemo->RenderUI();
        if (!m_PreviewMode)
            return;
    }
#endif
    if (m_PreviewMode)
    {
        ImGui::SetNextWindowPos(ImVec2(ImGui::GetIO().DisplaySize.x - 24.0f, 70.0f),
            ImGuiCond_FirstUseEver, ImVec2(1.0f, 0.0f));
        ImGui::SetNextWindowSize(ImVec2(330.0f, 0.0f), ImGuiCond_FirstUseEver);
        if (ImGui::Begin("Character Animation", nullptr, ImGuiWindowFlags_AlwaysAutoResize))
        {
            if (!m_PreviewReady)
                ImGui::TextWrapped("%s", m_PreviewError.c_str());
            else
            {
                ImGui::Text("Now playing: %s", kPreviewNames[m_PreviewIndex]);
                ImGui::TextUnformatted("Idle -> Walking -> Slow Run -> repeat");
                ImGui::Separator();
                ImGui::Checkbox("Play", &m_PreviewPlaying);
                ImGui::SameLine();
                if (ImGui::Checkbox("Auto cycle", &m_PreviewAutoCycle))
                    m_PreviewElapsed = 0.0f;
                if (m_PreviewAutoCycle)
                    ImGui::Text("Next animation in %.1f s", m_PreviewDurations[m_PreviewIndex] - m_PreviewElapsed);
                for (uint32_t i = 0; i < kPreviewCount; ++i)
                {
                    if (i > 0)
                        ImGui::SameLine();
                    if (ImGui::Button(kPreviewNames[i]))
                        SelectPreview(i);
                }
                if (ImGui::Button("Reset camera"))
                    ResetPreviewCamera();
#if defined(_M_X64)
                if (m_PhysicsDemo)
                    ImGui::TextWrapped("WASD: move camera. Q/E: down/up. Hold right mouse to look around.");
                else
#endif
                    ImGui::TextWrapped("Hold right mouse and drag to orbit. Hold right mouse and scroll to zoom.");
            }
        }
        ImGui::End();
        return;
    }
    static bool showWindow = true;
    static bool showDemo = false;
    static float testValue = 0.5f;
    static int clicks = 0;
    static char text[64] = "Hello PARS!";

    if (ImGui::IsKeyPressed(ImGuiKey_F1, false))
        showWindow = !showWindow;

    if (showWindow)
    {
        ImGui::SetNextWindowPos(ImVec2(30.0f, 80.0f), ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowSize(ImVec2(420.0f, 330.0f), ImGuiCond_FirstUseEver);
        if (ImGui::Begin("PARS - ImGui Test", &showWindow))
        {
            ImGui::TextColored(ImVec4(0.4f, 0.95f, 0.65f, 1.0f), "Win32 + DirectX 12 connected");
            ImGui::Text("%.1f FPS", ImGui::GetIO().Framerate);
            ImGui::Separator();
            ImGui::SliderFloat("Test value", &testValue, 0.0f, 1.0f);
            if (ImGui::Button("Click me"))
                ++clicks;
            ImGui::SameLine();
            ImGui::Text("Clicked %d times", clicks);
            ImGui::InputText("Text", text, IM_ARRAYSIZE(text));
            ImGui::Checkbox("Show ImGui demo", &showDemo);
            ImGui::Separator();
            ImGui::TextWrapped("Drag the title bar to move this window. F1 toggles this panel.");
            ImGui::TextWrapped("Use WASD to move and Q/E to move down/up. Hold right mouse outside the UI to look around.");
        }
        ImGui::End();
    }
    if (showDemo)
        ImGui::ShowDemoWindow(&showDemo);
}

void ModelViewer::RenderScene( void )
{
    GraphicsContext& gfxContext = GraphicsContext::Begin(L"Scene Render");

    uint32_t FrameIndex = TemporalEffects::GetFrameIndexMod2();
    const D3D12_VIEWPORT& viewport = m_MainViewport;
    const D3D12_RECT& scissor = m_MainScissor;

    bool physicsScene = false;
#if defined(_M_X64)
    if (m_PhysicsDemo)
    {
        physicsScene = true;
        m_PhysicsDemo->Render(gfxContext, m_Camera, viewport, scissor);
        if (ActiveModel().IsNull())
        {
            gfxContext.Finish();
            return;
        }
    }
#endif

    ParticleEffectManager::Update(gfxContext.GetComputeContext(), Graphics::GetFrameTime());

    ModelInstance& model = ActiveModel();
    if (model.IsNull())
    {
#ifdef LEGACY_RENDERER
        if (m_SponzaStarted)
            Sponza::RenderScene(gfxContext, m_Camera, viewport, scissor);
        else
#endif
        {
            gfxContext.TransitionResource(g_SceneColorBuffer, D3D12_RESOURCE_STATE_RENDER_TARGET, true);
            gfxContext.ClearColor(g_SceneColorBuffer);
            gfxContext.Finish();
            return;
        }
    }
    else
    {
        // Update global constants
        float costheta = cosf(g_SunOrientation);
        float sintheta = sinf(g_SunOrientation);
        float cosphi = cosf(g_SunInclination * 3.14159f * 0.5f);
        float sinphi = sinf(g_SunInclination * 3.14159f * 0.5f);

        Vector3 SunDirection = Normalize(Vector3( costheta * cosphi, sinphi, sintheta * cosphi ));
        m_SunShadowCamera.UpdateMatrix(-SunDirection,
            physicsScene ? Vector3(0, 2, 0) : Vector3(0, -500.0f, 0),
            physicsScene ? Vector3(16, 10, 16) : Vector3(5000, 3000, 3000),
            (uint32_t)g_ShadowBuffer.GetWidth(), (uint32_t)g_ShadowBuffer.GetHeight(), 16);

        GlobalConstants globals;
        globals.ViewProjMatrix = m_Camera.GetViewProjMatrix();
        globals.SunShadowMatrix = m_SunShadowCamera.GetShadowMatrix();
        globals.CameraPos = m_Camera.GetPosition();
        globals.SunDirection = SunDirection;
        globals.SunIntensity = Vector3(Scalar(g_SunLightIntensity));

        // Begin rendering depth
        gfxContext.TransitionResource(g_SceneDepthBuffer, D3D12_RESOURCE_STATE_DEPTH_WRITE, true);
        // Preserve the floor and box depth so they correctly occlude the character.
        if (!physicsScene)
            gfxContext.ClearDepth(g_SceneDepthBuffer);

        MeshSorter sorter(MeshSorter::kDefault);
		sorter.SetCamera(m_Camera);
		sorter.SetViewport(viewport);
		sorter.SetScissor(scissor);
		sorter.SetDepthStencilTarget(g_SceneDepthBuffer);
		sorter.AddRenderTarget(g_SceneColorBuffer);

        model.Render(sorter);

        sorter.Sort();

        {
            ScopedTimer _prof(L"Depth Pre-Pass", gfxContext);
            sorter.RenderMeshes(MeshSorter::kZPass, gfxContext, globals);
        }

        SSAO::Render(gfxContext, m_Camera);

        if (!SSAO::DebugDraw)
        {
            ScopedTimer _outerprof(L"Main Render", gfxContext);

            {
                ScopedTimer _prof(L"Sun Shadow Map", gfxContext);

                MeshSorter shadowSorter(MeshSorter::kShadows);
				shadowSorter.SetCamera(m_SunShadowCamera);
				shadowSorter.SetDepthStencilTarget(g_ShadowBuffer);

                model.Render(shadowSorter);

                shadowSorter.Sort();
                shadowSorter.RenderMeshes(MeshSorter::kZPass, gfxContext, globals);
            }

            gfxContext.TransitionResource(g_SceneColorBuffer, D3D12_RESOURCE_STATE_RENDER_TARGET, true);
            if (!physicsScene)
                gfxContext.ClearColor(g_SceneColorBuffer);

            {
                ScopedTimer _prof(L"Render Color", gfxContext);

                gfxContext.TransitionResource(g_SSAOFullScreen, D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE);
                gfxContext.TransitionResource(g_SceneDepthBuffer, D3D12_RESOURCE_STATE_DEPTH_READ);
                gfxContext.SetRenderTarget(g_SceneColorBuffer.GetRTV(), g_SceneDepthBuffer.GetDSV_DepthReadOnly());
                gfxContext.SetViewportAndScissor(viewport, scissor);

                sorter.RenderMeshes(MeshSorter::kOpaque, gfxContext, globals);
            }

            if (!physicsScene)
                Renderer::DrawSkybox(gfxContext, m_Camera, viewport, scissor);

            sorter.RenderMeshes(MeshSorter::kTransparent, gfxContext, globals);
        }
    }

    // Some systems generate a per-pixel velocity buffer to better track dynamic and skinned meshes.  Everything
    // is static in our scene, so we generate velocity from camera motion and the depth buffer.  A velocity buffer
    // is necessary for all temporal effects (and motion blur).
    MotionBlur::GenerateCameraVelocityBuffer(gfxContext, m_Camera, true);

    if (TemporalEffects::EnableTAA)
        TemporalEffects::ResolveImage(gfxContext);

    ParticleEffectManager::Render(gfxContext, m_Camera, g_SceneColorBuffer, g_SceneDepthBuffer,  g_LinearDepth[FrameIndex]);

    // Until I work out how to couple these two, it's "either-or".
    if (DepthOfField::Enable)
        DepthOfField::Render(gfxContext, m_Camera.GetNearClip(), m_Camera.GetFarClip());
    else
        MotionBlur::RenderObjectBlur(gfxContext, g_VelocityBuffer);

    gfxContext.Finish();
}
