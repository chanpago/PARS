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
#include "CommandListManager.h"
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
#include "GameFramework/Character.h"
#include "GameFramework/World.h"
#include "WorldRenderer.h"
#include "PreviewAnimInstance.h"
#include "SkeletalMeshViewer.h"
#include "GameFramework/PlayerController.h"
#include "ShadowCamera.h"
#include "Display.h"
#include "imgui.h"
#if defined(_M_X64)
#include "PhysicsDemo.h"
bool RunRagdollValidation(PARS::World& world, PARS::Character& character, const std::wstring& reportPath);
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
    virtual bool IsDone() override
    {
        if (m_RagdollTestDone) return true;
        // Escape belongs to the preview while it is focused (cancel a gizmo drag).
        return !(m_MeshViewer && m_MeshViewer->WantsInput()) && GameCore::IGameApp::IsDone();
    }

private:

    void LoadAnimationPreview(bool forceRebuild);
    void SelectPreview(uint32_t index);
    void ResetPreviewCamera();
    void UpdateCharacterInput(float deltaTime);
    void UpdateFollowCamera();
    void SetFreeCamera(bool freeCamera);
    PARS::PlayerController* GetPlayerController() const { return m_World.FindActor<PARS::PlayerController>(m_PlayerController); }
    PARS::Character* GetPreviewCharacter() const { return m_World.FindActor<PARS::Character>(m_PreviewCharacter); }
    PreviewAnimInstance* GetPreviewAnimation() const;
    void RenderModels(Renderer::MeshSorter& sorter);
    void DrawRagdollPanel(PARS::Character& character);

    Camera m_Camera;
    unique_ptr<CameraController> m_CameraController;

    D3D12_VIEWPORT m_MainViewport;
    D3D12_RECT m_MainScissor;

    ModelInstance m_ModelInst;
    PARS::World m_World;
    PARS::ActorHandle m_PreviewCharacter; // Observation only; World owns the Actor.
    PARS::ActorHandle m_PlayerController;
    bool m_FreeCamera = false;
    float m_ControlYaw = XM_PI;
    float m_ControlPitch = 0.28f;
    bool m_PreviewMode = false;
    bool m_PreviewReady = false;
    std::string m_PreviewError;
    std::string m_RagdollMessage;
    bool m_ActiveSupport = true;
    char m_CharacterPAPath[260] = "Mixamo.physics.json";
    bool m_RagdollTestDone = false;
    unique_ptr<SkeletalMeshViewer> m_MeshViewer;
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
    m_World.BeginPlay();

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
        if (m_World.InitializePhysics())
            m_PhysicsDemo.reset(new PhysicsDemo(m_World));
        if (m_PhysicsDemo && m_PhysicsDemo->Startup())
        {
            MotionBlur::Enable = false;
            TemporalEffects::EnableTAA = false; // This simple pass has no object velocity buffer.
            PostEffects::EnableAdaptation = false;
            SSAO::Enable = false;
            LoadIBLTextures();
            LoadAnimationPreview(forceRebuild);
            std::wstring ragdollReport;
            if (CommandLineArgs::GetString(L"ragdolltest", ragdollReport))
            {
                if (auto* character = GetPreviewCharacter()) RunRagdollValidation(m_World, *character, ragdollReport);
                m_RagdollTestDone = true;
            }
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

    std::shared_ptr<const Model> models[kPreviewCount];
    for (uint32_t i = 0; i < kPreviewCount; ++i)
    {
        auto& model = models[i];
        model = Renderer::LoadModel(root + kPreviewFiles[i], forceRebuild);
        if (!model || model->m_NumAnimations == 0 || model->m_Animations[0].duration <= 0.0f)
        {
            m_PreviewError = std::string("Could not load animation: ") + kPreviewNames[i];
            return;
        }
    }

    // Configure the world-owned Character before its first BeginPlay/Tick.
    auto* character = m_World.SpawnActor<PARS::Character>("MixamoCharacter");
    if (!character)
        return;
    auto& mesh = character->GetMesh();
    for (uint32_t i = 0; i < kPreviewCount; ++i)
    {
        const auto& model = models[i];
        if (i == 0)
        {
            if (!mesh.SetSkeletalMesh(model))
            {
                m_PreviewError = "The character model has no usable skeleton.";
                character->Destroy();
                return;
            }
            mesh.SetAnimInstance(std::make_unique<PreviewAnimInstance>(mesh));
        }
        if (!mesh.GetAnimInstance().AddAnimation(kPreviewNames[i], model))
        {
            m_PreviewError = std::string("Animation skeleton does not match the character: ") + kPreviewNames[i];
            character->Destroy();
            return;
        }
    }
    const float halfHeight = character->GetCapsuleComponent().GetHalfHeight();
    character->SetActorLocation(Vector3(2.0f, halfHeight, 0.0f));
#if defined(_M_X64)
    if (!m_PhysicsDemo)
#endif
    {
        const float scale = 10.0f / float(mesh.GetLocalBounds().GetRadius());
        character->SetActorTransform(AffineTransform(Matrix3::MakeScale(scale), Vector3(0, halfHeight * scale, 0)));
    }
    m_PreviewCharacter = character->GetHandle();
    PARS::PhysicsSkeleton physicsSkeleton;
    PARS::PhysicsAsset defaultPA;
    const std::vector<GraphNode> reference(models[0]->m_SceneGraph.get(), models[0]->m_SceneGraph.get() + models[0]->m_NumNodes);

    if (mesh.BuildPhysicsSkeleton(physicsSkeleton, m_RagdollMessage) &&
        PARS::BuildMixamoPhysicsAsset(physicsSkeleton, reference, defaultPA, m_RagdollMessage) &&
        mesh.SetPhysicsAsset(defaultPA, m_RagdollMessage)) m_RagdollMessage = "Mixamo default PA assigned.";
    mesh.GetAnimInstance().PlayAnimation(0);
    auto animation = GetPreviewAnimation();
    bool characterControl = false;
#if defined(_M_X64)
    characterControl = m_PhysicsDemo != nullptr;
#endif
    animation->SetLocomotionEnabled(characterControl);
    if (characterControl)
    {
        auto player = m_World.SpawnActor<PARS::PlayerController>("PlayerController");
        if (player && player->Possess(*character))
        {
            m_PlayerController = player->GetHandle();
        }      
    }

    m_World.Tick(0.0f);
    auto& context = GraphicsContext::Begin(L"Initialize Character Pose");
    Renderer::UpdateWorldRenderData(m_World, context);
    context.Finish(true);
    m_PreviewReady = true;
    ResetPreviewCamera();
    m_MeshViewer = std::make_unique<SkeletalMeshViewer>();

    if (!m_MeshViewer->Initialize(models, kPreviewNames, kPreviewCount))
    {
        m_MeshViewer.reset();
    }        
    else if (mesh.GetPhysicsAsset())
    {
		m_MeshViewer->SetPhysicsAsset(*mesh.GetPhysicsAsset(), m_PreviewError);
    }
}

void ModelViewer::SelectPreview(uint32_t index)
{
    if (auto animation = GetPreviewAnimation())
        animation->SelectAnimation(index);
}

PreviewAnimInstance* ModelViewer::GetPreviewAnimation() const
{
    auto character = GetPreviewCharacter();
    return character ? dynamic_cast<PreviewAnimInstance*>(&character->GetMesh().GetAnimInstance()) : nullptr;
}

void ModelViewer::RenderModels(Renderer::MeshSorter& sorter)
{
    Renderer::AddWorldMeshes(m_World, sorter);
    m_ModelInst.Render(sorter);
}

void ModelViewer::ResetPreviewCamera()
{
    m_Camera.SetZRange(0.1f, 1000.0f);
#if defined(_M_X64)
    if (m_PhysicsDemo)
    {
        m_Camera.SetZRange(0.1f, 100.0f);
        m_ControlYaw = XM_PI;
        m_ControlPitch = 0.28f;
        if (GetPlayerController() && !m_FreeCamera)
        {
            UpdateFollowCamera();
            return;
        }
        m_Camera.SetEyeAtUp(Vector3(5, 3.2f, 7), Vector3(1, 1.8f, 0), Vector3(kYUnitVector));
        auto controller = new FlyingFPSCamera(m_Camera, Vector3(kYUnitVector));
        controller->SetMoveSpeed(5.0f);
        m_CameraController.reset(controller);
        return;
    }
#endif
    auto character = GetPreviewCharacter();
    if (!m_PreviewReady || !character)
    {
        m_CameraController.reset(new OrbitCamera(m_Camera, BoundingSphere(Vector3(0, 8, 0), 14.0f)));
        return;
    }
    // Leave room for the hands and feet throughout all three animations.
    const BoundingSphere bounds = character->GetMesh().GetModelInstance().GetBoundingSphere();
    m_CameraController.reset(new OrbitCamera(m_Camera,
        BoundingSphere(bounds.GetCenter(), bounds.GetRadius() * 1.4f), Vector3(kYUnitVector)));
}

void ModelViewer::SetFreeCamera(bool freeCamera)
{
    m_FreeCamera = freeCamera;
    if (auto player = GetPlayerController()) player->SetInputEnabled(!freeCamera);
    if (freeCamera)
    {
        auto camera = new FlyingFPSCamera(m_Camera, Vector3(kYUnitVector));
        camera->SetMoveSpeed(5.0f);
        m_CameraController.reset(camera);
    }
    else
    {
        const auto forward = m_Camera.GetForwardVec();
        m_ControlYaw = std::atan2(float(forward.GetX()), float(forward.GetZ()));
        m_ControlPitch = Clamp(std::asin(Clamp(-float(forward.GetY()), -1.0f, 1.0f)), -0.1f, 1.1f);
    }
}

void ModelViewer::UpdateCharacterInput(float deltaTime)
{
    using namespace GameInput;
    auto player = GetPlayerController();
    if (!player) return;
    if (m_MeshViewer && m_MeshViewer->WantsInput())
    {
        player->SetInputEnabled(false);
        return;
    }
    if (IsFirstPressed(kKey_tab)) SetFreeCamera(!m_FreeCamera);
    auto* character = GetPreviewCharacter();
    player->SetInputEnabled(!m_FreeCamera && character && character->CanControlMovement());
    if (m_FreeCamera) return;
    m_ControlYaw -= GetAnalogInput(kAnalogRightStickX) * deltaTime * 2.0f;
    m_ControlPitch -= GetAnalogInput(kAnalogRightStickY) * deltaTime * 2.0f;
    if (IsPressed(kMouse1))
    {
        m_ControlYaw -= GetAnalogInput(kAnalogMouseX);
        m_ControlPitch -= GetAnalogInput(kAnalogMouseY);
    }
    m_ControlYaw = std::remainder(m_ControlYaw, XM_2PI);
    m_ControlPitch = Clamp(m_ControlPitch, -0.1f, 1.1f);
    player->SetControlYaw(m_ControlYaw);
    player->SetMoveInput(float(IsPressed(kKey_w)) - float(IsPressed(kKey_s)) + GetAnalogInput(kAnalogLeftStickY),
        float(IsPressed(kKey_d)) - float(IsPressed(kKey_a)) + GetAnalogInput(kAnalogLeftStickX),
        IsPressed(kKey_lshift) || IsPressed(kKey_rshift) || IsPressed(kLThumbClick));
}

void ModelViewer::UpdateFollowCamera()
{
    auto character = GetPreviewCharacter();
    if (!character) return;
    const Vector3 target = character->IsRagdoll() ? character->GetMesh().GetRagdoll().GetPosition() + Vector3(0, .25f, 0) :
        character->GetActorLocation() + Vector3(0, 0.55f, 0);
    const Vector3 forward(std::sin(m_ControlYaw) * std::cos(m_ControlPitch), -std::sin(m_ControlPitch),
        std::cos(m_ControlYaw) * std::cos(m_ControlPitch));
    m_Camera.SetEyeAtUp(target - forward * 4.5f, target, Vector3(kYUnitVector));
    m_Camera.Update();
}

void ModelViewer::Cleanup( void )
{
    // GameCore waits for the GPU before Cleanup. End actor lifetimes while the
    // renderer and physics services are still available to their components.
    m_MeshViewer.reset();
    m_PreviewCharacter = {};
    m_PlayerController = {};
    m_World.Clear();
#if defined(_M_X64)
    if (m_PhysicsDemo)
    {
        m_PhysicsDemo->Cleanup();
        m_PhysicsDemo.reset();
    }
#endif
    m_ModelInst = nullptr;

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

    if (m_World.HasPendingDestroy())
    {
        // Actor meshes may still be referenced by the previous GPU frame. Only
        // deletion waits; ordinary world updates do not force a GPU idle.
        g_CommandManager.IdleGPU();
        m_World.CollectGarbage();
    }

    if (GameInput::IsFirstPressed(GameInput::kLShoulder))
    {
        DebugZoom.Decrement();
    }
    else if (GameInput::IsFirstPressed(GameInput::kRShoulder))
    {
        DebugZoom.Increment();
    }
       
    m_Camera.SetAspectRatio(float(g_SceneColorBuffer.GetHeight()) / float(g_SceneColorBuffer.GetWidth()));
    UpdateCharacterInput(deltaT);

    if ((!GetPlayerController() || m_FreeCamera) && !(m_MeshViewer && m_MeshViewer->WantsInput()))
    {
        m_CameraController->EnableMouseInput(GameInput::IsPressed(GameInput::kMouse1));
        m_CameraController->Update(deltaT);
    }

    GraphicsContext& gfxContext = GraphicsContext::Begin(L"Scene Update");
    m_World.Tick(deltaT);


#if defined(_M_X64)
    if (m_PhysicsDemo)
    {
        m_PhysicsDemo->Update(deltaT);
    }       
#endif
    if (GetPlayerController() && !m_FreeCamera)
    {
		UpdateFollowCamera();
    }

    Renderer::UpdateWorldRenderData(m_World, gfxContext);
    m_ModelInst.Update(gfxContext, deltaT);

    if (m_MeshViewer)
    {
        m_MeshViewer->Update(deltaT);
    }

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

void ModelViewer::DrawRagdollPanel(PARS::Character& character)
{
    if (!ImGui::CollapsingHeader("Ragdoll / PhysicsAsset", ImGuiTreeNodeFlags_DefaultOpen)) return;
    auto& mesh = character.GetMesh();
    const auto* asset = mesh.GetPhysicsAsset();
    if (asset) ImGui::Text("Assigned PA: %zu bodies / %zu joints", asset->bodies.size(), asset->constraints.size());
    else ImGui::TextUnformatted("No PhysicsAsset assigned");
    const auto& runtime = mesh.GetRagdoll();
    ImGui::Text("PhysX: %zu bodies / %zu joints", runtime.GetBodyCount(), runtime.GetJointCount());
    if (runtime.IsActive())
    {
        ImGui::Text("Mode: %s", mesh.IsActiveRagdoll() ? "Active Ragdoll" : character.IsRagdoll() ? "Passive Ragdoll" : "Animation");
        ImGui::Text("Kinematic bodies: %zu", runtime.GetKinematicBodyCount());
    }
    if (!mesh.GetPhysicsError().empty()) ImGui::TextWrapped("%s", mesh.GetPhysicsError().c_str());
    ImGui::BeginDisabled(character.IsRagdoll());
    ImGui::SetNextItemWidth(290);
    ImGui::InputText("##CharacterPAPath", m_CharacterPAPath, sizeof(m_CharacterPAPath));
    if (ImGui::Button("Load character PA"))
    {
        PARS::PhysicsAsset candidate;
        if (candidate.Load(m_CharacterPAPath, m_RagdollMessage) && mesh.SetPhysicsAsset(candidate, m_RagdollMessage))
            m_RagdollMessage = "PA loaded and assigned to the play character.";
    }
    ImGui::SameLine();
    ImGui::BeginDisabled(!m_MeshViewer || m_MeshViewer->GetPhysicsAsset().bodies.empty());
    if (ImGui::Button("Use editor PA") && mesh.SetPhysicsAsset(m_MeshViewer->GetPhysicsAsset(), m_RagdollMessage))
        m_RagdollMessage = "Current editor PA assigned to the play character.";
    ImGui::EndDisabled();
    ImGui::EndDisabled();
    ImGui::BeginDisabled(!asset || mesh.HasPendingDriveSettings());
    if (ImGui::Button("Save character PA"))
    {
        if (mesh.GetPhysicsAsset()->Save(m_CharacterPAPath, m_RagdollMessage)) m_RagdollMessage = "Character PA saved.";
    }
    ImGui::EndDisabled();
    asset = mesh.GetPhysicsAsset();
    ImGui::BeginDisabled(!asset || asset->bodies.empty() || !m_World.GetPhysicsWorld());
    ImGui::BeginDisabled(mesh.GetPhysicsMode() == PARS::PhysicsAnimationMode::PassiveRagdoll);
    if (ImGui::Button("Passive Ragdoll") && character.StartRagdoll(m_RagdollMessage))
        m_RagdollMessage = "Passive ragdoll: angular drives off, pelvis released.";
    ImGui::EndDisabled();
    ImGui::SameLine();
    ImGui::BeginDisabled(mesh.IsActiveRagdoll() || mesh.HasPendingDriveSettings());
    if (ImGui::Button("Active Ragdoll"))
    {
        const bool entering = !character.IsRagdoll();
        if (character.SetPhysicsMode(PARS::PhysicsAnimationMode::ActiveRagdoll, m_RagdollMessage))
        {
            if (auto* animation = GetPreviewAnimation()) animation->SetAutoCycleEnabled(false);
            if (!entering || mesh.GetRagdoll().SetRootSupported(m_ActiveSupport, m_RagdollMessage))
                m_RagdollMessage = "Active ragdoll: animation targets drive the physical joints.";
        }
    }
    ImGui::EndDisabled();
    ImGui::EndDisabled();
    bool supported = character.IsRagdoll() ? runtime.IsRootSupported() : m_ActiveSupport;
    ImGui::BeginDisabled(mesh.GetPhysicsMode() == PARS::PhysicsAnimationMode::PassiveRagdoll);
    if (ImGui::Checkbox("Pelvis force / torque support", &supported))
    {
        if (!character.IsRagdoll() || mesh.GetRagdoll().SetRootSupported(supported, m_RagdollMessage)) m_ActiveSupport = supported;
    }
    ImGui::EndDisabled();
    if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled)) ImGui::SetTooltip("Dynamic pelvis: limited forces follow animation position, limited torques follow rotation. Contacts and impacts can displace it. Passive disables this support.");
    if (mesh.IsActiveRagdoll())
    {
        bool drives = runtime.AreDrivesEnabled();
        if (ImGui::Checkbox("Angular drives", &drives)) mesh.GetRagdoll().SetDrivesEnabled(drives);
        ImGui::Text("Driven joints: %zu", runtime.GetDrivenJointCount());
        ImGui::Text("Max target error: %.1f deg", XMConvertToDegrees(runtime.GetMaxDriveError()));
        ImGui::Text("Target limit correction: %.1f deg", XMConvertToDegrees(runtime.GetMaxTargetClampAngle()));
        if (runtime.IsRootSupported())
            ImGui::Text("Pelvis target error: %.1f cm / %.1f deg", runtime.GetRootPositionError() * 100.0f,
                XMConvertToDegrees(runtime.GetRootRotationError()));
        ImGui::TextUnformatted(character.CanControlMovement() ? "WASD: move | Shift: run" : "Movement requires pelvis support.");
    }
    ImGui::BeginDisabled(!asset);
    if (ImGui::Button("Use default PD gains"))
    {
        auto candidate = *asset; PARS::ConfigureDefaultDrives(candidate);
        if (mesh.QueueDriveSettings(candidate, m_RagdollMessage)) m_RagdollMessage = "Default gains queued for the next physics step.";
    }
    ImGui::BeginDisabled(!m_MeshViewer || m_MeshViewer->GetPhysicsAsset().bodies.empty());
    if (ImGui::Button("Apply editor drive settings") && mesh.QueueDriveSettings(m_MeshViewer->GetPhysicsAsset(), m_RagdollMessage))
        m_RagdollMessage = "Drive settings queued. Frames, shapes and limits use the assigned character PA.";
    ImGui::EndDisabled();
    ImGui::EndDisabled();
    if (mesh.HasPendingDriveSettings()) ImGui::TextUnformatted("Drive settings pending next physics step.");
    if (character.IsRagdoll())
    {
        if (ImGui::Button("Stop / return to animation") && character.StopRagdoll(m_RagdollMessage))
            m_RagdollMessage = "Returned to animation and character control at the current movement target.";
        if (ImGui::Button("Restart ragdoll"))
        {
            const auto mode = mesh.GetPhysicsMode(); const bool support = runtime.IsRootSupported();
            if (character.StopRagdoll(m_RagdollMessage) && character.SetPhysicsMode(mode, m_RagdollMessage) &&
                mode == PARS::PhysicsAnimationMode::ActiveRagdoll)
                mesh.GetRagdoll().SetRootSupported(support, m_RagdollMessage);
        }
        if (character.IsRagdoll())
        {
            if (ImGui::Button("Push ragdoll")) mesh.GetRagdoll().AddVelocity(Vector3(1.5f, 2, 0));
            ImGui::Text("Joint separation: %.4f m", mesh.GetRagdoll().GetMaxAnchorError());
        }
    }
    bool paused = m_World.IsPhysicsPaused();
    if (ImGui::Checkbox("Pause world##Ragdoll", &paused)) m_World.SetPhysicsPaused(paused);
    ImGui::SameLine();
    ImGui::BeginDisabled(!paused);
    if (ImGui::Button("Step 1/60##Ragdoll")) m_World.RequestPhysicsStep();
    ImGui::EndDisabled();
    if (!m_RagdollMessage.empty()) ImGui::TextWrapped("%s", m_RagdollMessage.c_str());
}

void ModelViewer::RenderUI(GraphicsContext& context)
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
        if (ImGui::BeginMainMenuBar())
        {
            if (ImGui::BeginMenu("Window"))
            {
                if (ImGui::MenuItem("Skeletal Mesh Viewer", nullptr, false, m_MeshViewer != nullptr))
                    m_MeshViewer->Open();
                ImGui::EndMenu();
            }
            ImGui::EndMainMenuBar();
        }
        ImGui::SetNextWindowPos(ImVec2(ImGui::GetIO().DisplaySize.x - 24.0f, 70.0f),
            ImGuiCond_FirstUseEver, ImVec2(1.0f, 0.0f));
        ImGui::SetNextWindowSize(ImVec2(330.0f, 0.0f), ImGuiCond_FirstUseEver);
        if (ImGui::Begin("Character Animation", nullptr, ImGuiWindowFlags_AlwaysAutoResize))
        {
            auto animation = GetPreviewAnimation();
            if (!m_PreviewReady || !animation)
                ImGui::TextWrapped("%s", m_PreviewError.empty() ? "Preview character is no longer in the world." : m_PreviewError.c_str());
            else
            {
                ImGui::Text("Now playing: %s", animation->GetAnimationName(animation->GetCurrentAnimationIndex()).c_str());
                if (auto character = GetPreviewCharacter())
                {
                    const auto& movement = character->GetCharacterMovement();
                    ImGui::Text("Speed: %.2f m/s | %s", movement.GetGroundSpeed(),
                        movement.IsMovingOnGround() ? "Grounded" : "Airborne");
                    ImGui::Text("Blend: %.0f%%", animation->GetBlendAlpha() * 100.0f);
                }
                if (auto character = GetPreviewCharacter()) DrawRagdollPanel(*character);
                ImGui::Separator();
                ImGui::BeginDisabled(GetPreviewCharacter() && GetPreviewCharacter()->GetMesh().GetPhysicsMode() == PARS::PhysicsAnimationMode::PassiveRagdoll);
                if (GetPlayerController())
                {
                    bool automatic = animation->IsLocomotionEnabled();
                    if (ImGui::Checkbox("Animation follows movement", &automatic))
                        animation->SetLocomotionEnabled(automatic);
                    bool freeCamera = m_FreeCamera;
                    if (ImGui::Checkbox("Free camera (Tab)", &freeCamera))
                        SetFreeCamera(freeCamera);
                }
                if (!animation->IsLocomotionEnabled())
                {
                    bool playing = animation->IsPlaying();
                    if (ImGui::Checkbox("Play", &playing)) animation->SetPlaying(playing);
                    ImGui::SameLine();
                    bool autoCycle = animation->IsAutoCycleEnabled();
                    if (ImGui::Checkbox("Auto cycle", &autoCycle)) animation->SetAutoCycleEnabled(autoCycle);
                    if (autoCycle) ImGui::Text("Next animation in %.1f s", animation->GetTimeUntilNextAnimation());
                }
                for (uint32_t i = 0; i < kPreviewCount; ++i)
                {
                    if (i > 0)
                        ImGui::SameLine();
                    if (ImGui::Button(kPreviewNames[i]))
                        SelectPreview(i);
                }
                if (ImGui::Button("Reset camera"))
                    ResetPreviewCamera();
                ImGui::EndDisabled();
#if defined(_M_X64)
                if (m_PhysicsDemo)
                {
                    if (ImGui::Button("Reset character"))
                    {
                        if (auto character = GetPreviewCharacter())
                        {
                            if (character->StopRagdoll(m_RagdollMessage))
                            {
                                character->SetActorLocation(Vector3(2, 0.95f, 0));
                                character->GetCharacterMovement().StopMovementImmediately();
                            }
                        }
                    }
                    ImGui::TextWrapped(m_FreeCamera ? "Free camera: WASD + Q/E. Tab: control character." :
                        "Character: WASD move, Shift run. Tab: free camera.");
                    ImGui::TextWrapped("Hold right mouse to look. Animation buttons enable manual preview.");
                }
                else
#endif
                    ImGui::TextWrapped("Hold right mouse and drag to orbit. Hold right mouse and scroll to zoom.");
            }
        }
        ImGui::End();
        if (m_MeshViewer) m_MeshViewer->RenderUI(context);
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
    const bool hasModels = Renderer::HasWorldMeshes(m_World) || !m_ModelInst.IsNull();
#if defined(_M_X64)
    if (m_PhysicsDemo)
    {
        physicsScene = true;
        m_PhysicsDemo->Render(gfxContext, m_Camera, viewport, scissor);
        if (!hasModels)
        {
            m_PhysicsDemo->RenderCapsules(gfxContext, m_Camera, viewport, scissor);
            gfxContext.Finish();
            return;
        }
    }
#endif

    ParticleEffectManager::Update(gfxContext.GetComputeContext(), Graphics::GetFrameTime());

    if (!hasModels)
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

        RenderModels(sorter);

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

                RenderModels(shadowSorter);

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

#if defined(_M_X64)
    if (m_PhysicsDemo) m_PhysicsDemo->RenderCapsules(gfxContext, m_Camera, viewport, scissor);
#endif
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
