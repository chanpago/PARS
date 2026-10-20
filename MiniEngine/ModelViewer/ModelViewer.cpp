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
#include "RagdollComparisonRunner.h"
bool RunRagdollValidation(PARS::World& world, PARS::Character& character, const std::wstring& reportPath);
bool RunRagdollComparisonValidation(RagdollComparisonRunner& runner, const PARS::PhysicsAsset& asset,
    PhysicsWorld& owner, const std::wstring& reportPath);
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
        L"Idle/Idle.gltf", L"Walking/Walking.gltf", L"SlowRun/SlowRun.gltf"};
    const char* const kPreviewNames[] = {"Idle", "Walking", "Slow Run"};
    constexpr uint32_t kPreviewCount = 3;

    // 실행 파일 위치와 작업 디렉터리 양쪽을 기준으로 프로젝트 에셋을 찾는다.
    // 빌드 출력 폴더에서 직접 실행한 경우에도 동작한다.
    std::wstring FindPreviewAssetRoot()
    {
        wchar_t path[32768];
        for (int location = 0; location < 2; ++location)
        {
            DWORD length = location == 0 ? GetModuleFileNameW(nullptr, path, _countof(path))
                                         : GetCurrentDirectoryW(_countof(path), path);
            if (length == 0 || length >= _countof(path))
            {
                continue;
            }
            std::wstring directory(path, length);
            if (location == 0)
            {
                directory = directory.substr(0, directory.find_last_of(L"/\\"));
            }
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
                {
                    return root;
                }
                const size_t separator = directory.find_last_of(L"/\\");
                if (separator == std::wstring::npos)
                {
                    break;
                }
                directory.resize(separator);
            }
        }
        return L"";
    }
}

class ModelViewer : public GameCore::IGameApp
{
public:
    ModelViewer(void)
    {
    }

    virtual void Startup(void) override;
    virtual void Cleanup(void) override;

    virtual void Update(float deltaT) override;
    virtual void RenderScene(void) override;
    virtual void RenderUI(GraphicsContext&) override;
    virtual bool UsesImGui() const override
    {
        return true;
    }
    virtual bool IsDone() override
    {
        if (m_RagdollTestDone)
        {
            return true;
        }
#if defined(_M_X64)
        std::wstring comparisonCapture;
        if (m_Comparison && CommandLineArgs::GetString(L"comparisoncapture", comparisonCapture))
        {
            // Read back the rendered buffers; PrintWindow cannot capture a DX12 flip-model window.
            if (++m_ComparisonCaptureFrames == 10)
            {
                g_OverlayBuffer.ExportToFile(comparisonCapture + L".launch.ui");
            }
            if (m_Comparison->GetState() == RagdollComparisonRunner::State::Complete)
            {
                g_OverlayBuffer.ExportToFile(comparisonCapture + L".ui");
                g_SceneColorBuffer.ExportToFile(comparisonCapture + L".scene");
                return true;
            }
        }
#endif
        // 미리보기에 포커스가 있으면 Escape를 미리보기에서 처리한다(기즈모 드래그 취소).
        return !(m_MeshViewer && m_MeshViewer->WantsInput()) && GameCore::IGameApp::IsDone();
    }

private:
    void LoadAnimationPreview(bool forceRebuild);
    void SelectPreview(uint32_t index);
    void ResetPreviewCamera();
    void UpdateCharacterInput(float deltaTime);
    void UpdateFollowCamera();
    void SetFreeCamera(bool freeCamera);
    PARS::PlayerController* GetPlayerController() const
    {
        return m_World.FindActor<PARS::PlayerController>(m_PlayerController);
    }
    PARS::Character* GetPreviewCharacter() const
    {
        return m_World.FindActor<PARS::Character>(m_PreviewCharacter);
    }
    PreviewAnimInstance* GetPreviewAnimation() const;
    void RenderModels(Renderer::MeshSorter& sorter);
    void DrawRagdollPanel(PARS::Character& character);

    Camera m_Camera;
    unique_ptr<CameraController> m_CameraController;

    D3D12_VIEWPORT m_MainViewport;
    D3D12_RECT m_MainScissor;

    ModelInstance m_ModelInst;
    PARS::World m_World;
    PARS::ActorHandle m_PreviewCharacter; // 참조만 보관하며, Actor의 소유자는 World다.
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
    unique_ptr<RagdollComparisonRunner> m_Comparison;
    bool m_WasComparing = false;
    bool m_PreComparisonFreeCamera = false;
    uint32_t m_ComparisonCaptureFrames = 0;
#endif
};

CREATE_APPLICATION(ModelViewer)

ExpVar g_SunLightIntensity("Viewer/Lighting/Sun Light Intensity", 4.0f, 0.0f, 16.0f, 0.1f);
NumVar g_SunOrientation("Viewer/Lighting/Sun Orientation", -0.5f, -100.0f, 100.0f, 0.1f);
NumVar g_SunInclination("Viewer/Lighting/Sun Inclination", 0.75f, 0.0f, 1.0f, 0.01f);

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

#include <direct.h> // _getcwd()로 데이터 루트 경로를 확인하기 위해 사용한다.

void LoadIBLTextures()
{
    char CWD[256];
    _getcwd(CWD, 256);

    Utility::Printf("Loading IBL environment maps\n");

    WIN32_FIND_DATA ffd;
    HANDLE hFind = FindFirstFile(L"Textures/*_diffuseIBL.dds", &ffd);

    g_IBLSet.AddEnum(L"None");

    if (hFind != INVALID_HANDLE_VALUE)
    {
        do
        {
            if (ffd.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY)
            {
                continue;
            }

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
        } while (FindNextFile(hFind, &ffd) != 0);
    }

    FindClose(hFind);

    Utility::Printf("Found %u IBL environment map sets\n", g_IBLTextures.size());

    if (g_IBLTextures.size() > 0)
    {
        g_IBLSet.Increment();
    }
}

void ModelViewer::Startup(void)
{
    MotionBlur::Enable = true;
    TemporalEffects::EnableTAA = true;
    FXAA::Enable = false;
    PostEffects::EnableHDR = true;
    PostEffects::EnableAdaptation = true;
    SSAO::Enable = true;

    Renderer::Initialize();
    m_World.BeginPlay();

    // 기본 씬은 물리 맵과 애니메이션 캐릭터를 함께 사용한다.
    // -physics 0은 Sponza를 선택하며, -model은 기존처럼 단일 모델을 선택한다.
    uint32_t physicsMode = 1;
    const bool hasPhysicsOption = CommandLineArgs::GetInteger(L"physics", physicsMode);
    std::wstring gltfFileName;
    const bool hasModel = CommandLineArgs::GetString(L"model", gltfFileName);
    bool forceRebuild = false;
    uint32_t rebuildValue;
    if (CommandLineArgs::GetInteger(L"rebuild", rebuildValue))
    {
        forceRebuild = rebuildValue != 0;
    }
#if defined(_M_X64)
    if (physicsMode && !hasModel)
    {
        if (m_World.InitializePhysics())
        {
            m_PhysicsDemo.reset(new PhysicsDemo(m_World));
        }
        if (m_PhysicsDemo && m_PhysicsDemo->Startup())
        {
            MotionBlur::Enable = false;
            TemporalEffects::EnableTAA = false; // 이 단순 렌더링 패스에는 객체 속도 버퍼가 없다.
            PostEffects::EnableAdaptation = false;
            SSAO::Enable = false;
            LoadIBLTextures();
            LoadAnimationPreview(forceRebuild);
            std::wstring ragdollReport;
            if (CommandLineArgs::GetString(L"ragdolltest", ragdollReport))
            {
                if (auto* character = GetPreviewCharacter())
                {
                    RunRagdollValidation(m_World, *character, ragdollReport);
                }
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
        m_Camera.SetEyeAtUp(eye, Vector3(kZero), Vector3(kYUnitVector));
#endif
    }
    else
    {
        m_ModelInst = Renderer::LoadModel(gltfFileName, forceRebuild);
        m_ModelInst.LoopAllAnimations();
        m_ModelInst.Resize(10.0f);

        MotionBlur::Enable = false;
        // 스키닝 메시는 아직 시간적 재투영에 사용할 객체 속도를 기록하지 않는다.
        if (m_ModelInst.GetNumAnimations() > 0)
        {
            TemporalEffects::EnableTAA = false;
            FXAA::Enable = true;
        }
    }

    m_Camera.SetZRange(1.0f, 10000.0f);
    if (gltfFileName.size() == 0)
    {
        m_CameraController.reset(new FlyingFPSCamera(m_Camera, Vector3(kYUnitVector)));
    }
    else
    {
        m_CameraController.reset(new OrbitCamera(m_Camera, m_ModelInst.GetBoundingSphere(), Vector3(kYUnitVector)));
    }
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

    // World가 소유하는 Character를 첫 BeginPlay/Tick 전에 설정한다.
    auto* character = m_World.SpawnActor<PARS::Character>("MixamoCharacter");
    if (!character)
    {
        return;
    }
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
        mesh.SetPhysicsAsset(defaultPA, m_RagdollMessage))
    {
        m_RagdollMessage = "Mixamo default PA assigned.";
    }
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
#if defined(_M_X64)
    if (m_World.GetPhysicsWorld())
    {
        m_Comparison = std::make_unique<RagdollComparisonRunner>(*m_World.GetPhysicsWorld(), models, kPreviewNames, kPreviewCount);
        std::wstring report;
        if (CommandLineArgs::GetString(L"comparisontest", report) && mesh.GetPhysicsAsset())
        {
            RunRagdollComparisonValidation(*m_Comparison, *mesh.GetPhysicsAsset(), *m_World.GetPhysicsWorld(), report);
            m_RagdollTestDone = true;
        }
        uint32_t comparisonPreview = 0;
        if (!m_RagdollTestDone && CommandLineArgs::GetInteger(L"comparisonpreview", comparisonPreview) &&
            comparisonPreview && mesh.GetPhysicsAsset())
        {
            m_Comparison->RequestWalkingComparison(m_MeshViewer ? &m_MeshViewer->GetPhysicsAsset() : nullptr,
                mesh.GetPhysicsAsset());
        }
    }
#endif
}

void ModelViewer::SelectPreview(uint32_t index)
{
    if (auto animation = GetPreviewAnimation())
    {
        animation->SelectAnimation(index);
    }
}

PreviewAnimInstance* ModelViewer::GetPreviewAnimation() const
{
    auto character = GetPreviewCharacter();
    return character ? dynamic_cast<PreviewAnimInstance*>(&character->GetMesh().GetAnimInstance()) : nullptr;
}

void ModelViewer::RenderModels(Renderer::MeshSorter& sorter)
{
#if defined(_M_X64)
    if (m_Comparison && m_Comparison->IsShowingScene())
    {
        Renderer::AddWorldMeshes(m_Comparison->GetWorld(), sorter);
        return;
    }
#endif
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
    // 세 애니메이션 모두에서 손과 발이 화면에 들어오도록 여유 공간을 둔다.
    const BoundingSphere bounds = character->GetMesh().GetModelInstance().GetBoundingSphere();
    m_CameraController.reset(new OrbitCamera(m_Camera,
        BoundingSphere(bounds.GetCenter(), bounds.GetRadius() * 1.4f), Vector3(kYUnitVector)));
}

void ModelViewer::SetFreeCamera(bool freeCamera)
{
    m_FreeCamera = freeCamera;
    if (auto player = GetPlayerController())
    {
        player->SetInputEnabled(!freeCamera);
    }
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
    if (!player)
    {
        return;
    }
#if defined(_M_X64)
    if (m_Comparison && m_Comparison->IsShowingScene())
    {
        player->SetInputEnabled(false);
        return;
    }
#endif
    if (m_MeshViewer && m_MeshViewer->WantsInput())
    {
        player->SetInputEnabled(false);
        return;
    }
    if (IsFirstPressed(kKey_tab))
    {
        SetFreeCamera(!m_FreeCamera);
    }
    auto* character = GetPreviewCharacter();
    player->SetInputEnabled(!m_FreeCamera && character && character->CanControlMovement());
    if (m_FreeCamera)
    {
        return;
    }
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
    if (!character)
    {
        return;
    }
    const Vector3 target = character->IsRagdoll() ? character->GetMesh().GetRagdoll().GetPosition() + Vector3(0, .25f, 0) : character->GetActorLocation() + Vector3(0, 0.55f, 0);
    const Vector3 forward(std::sin(m_ControlYaw) * std::cos(m_ControlPitch), -std::sin(m_ControlPitch),
        std::cos(m_ControlYaw) * std::cos(m_ControlPitch));
    m_Camera.SetEyeAtUp(target - forward * 4.5f, target, Vector3(kYUnitVector));
    m_Camera.Update();
}

void ModelViewer::Cleanup(void)
{
    // GameCore는 Cleanup 전에 GPU 작업 완료를 기다린다. 컴포넌트가 렌더러와 물리 서비스를
    // 아직 사용할 수 있는 동안 Actor의 수명을 종료한다.
    m_MeshViewer.reset();
#if defined(_M_X64)
    m_Comparison.reset();
#endif
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
    {
        Sponza::Cleanup();
    }
#endif

    Renderer::Shutdown();
}

namespace Graphics
{
    extern EnumVar DebugZoom;
}

void ModelViewer::Update(float deltaT)
{
    ScopedTimer _prof(L"Update State");
#if defined(_M_X64)
    if (m_Comparison)
    {
        m_Comparison->ProcessRequests();
        m_Comparison->Advance(deltaT);
        if (!m_WasComparing && m_Comparison->IsShowingScene())
        {
            m_PreComparisonFreeCamera = m_FreeCamera;
        }
        if (m_Comparison->ConsumeFocusRequest() && m_Comparison->IsShowingScene())
        {
            const auto target = m_Comparison->GetViewCenter();
            m_Camera.SetEyeAtUp(target + Vector3(5, 3, 7), target, Vector3(kYUnitVector));
            SetFreeCamera(true);
            m_Camera.Update();
            if (m_MeshViewer)
            {
                m_MeshViewer->Close();
            }
        }
        if (m_WasComparing && !m_Comparison->IsShowingScene())
        {
            SetFreeCamera(m_PreComparisonFreeCamera);
            ResetPreviewCamera();
        }
        m_WasComparing = m_Comparison->IsShowingScene();
    }
#endif

    if (m_World.HasPendingDestroy())
    {
        // 이전 GPU 프레임이 Actor 메시를 아직 참조할 수 있다. 삭제할 때만 완료를 기다리며,
        // 일반적인 월드 갱신에서는 GPU가 유휴 상태가 되도록 강제하지 않는다.
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
#if defined(_M_X64)
    if (m_Comparison)
    {
        if (m_Comparison->ShouldFollowCamera())
        {
            const auto target = m_Comparison->GetViewCenter();
            m_Camera.SetEyeAtUp(target + Vector3(5, 3, 7), target, Vector3(kYUnitVector));
            m_Camera.Update();
        }
        Renderer::UpdateWorldRenderData(m_Comparison->GetWorld(), gfxContext);
    }
#endif

    Renderer::UpdateWorldRenderData(m_World, gfxContext);
    m_ModelInst.Update(gfxContext, deltaT);

    if (m_MeshViewer)
    {
        m_MeshViewer->Update(deltaT);
    }

    gfxContext.Finish();

    // TAA를 위해 뷰포트 오프셋으로 프레임마다 샘플 위치를 조금씩 이동시킨다.
    // D3D에서는 소수 오프셋을 사용할 때 뷰포트의 암묵적인 시저 영역이
    // floor(TopLeftXY)와 floor(TopLeftXY + WidthHeight)로 정해진다.
    // 따라서 왼쪽 위 좌표가 (-0.25, -0.25)처럼 음의 소수이면
    // 오른쪽 아래 좌표도 정수 한 칸만큼 위로 이동한다. 뷰포트 크기에
    // 한 픽셀을 더하는 방법도 있지만, 여기서는 양의 소수 오프셋만 사용한다.
    // 이때 평균 샘플 위치는 +0.5가 되므로, 시간적 안티앨리어싱을 끈 경우에도
    // 이 값을 사용한다.
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
    if (!ImGui::CollapsingHeader("Ragdoll / PhysicsAsset", ImGuiTreeNodeFlags_DefaultOpen))
    {
        return;
    }
    auto& mesh = character.GetMesh();
    const auto* asset = mesh.GetPhysicsAsset();
    if (asset)
    {
        ImGui::Text("Assigned PA: %zu bodies / %zu joints", asset->bodies.size(), asset->constraints.size());
    }
    else
    {
        ImGui::TextUnformatted("No PhysicsAsset assigned");
    }
    const auto& runtime = mesh.GetRagdoll();
    ImGui::Text("PhysX: %zu bodies / %zu joints", runtime.GetBodyCount(), runtime.GetJointCount());
    if (runtime.IsActive())
    {
        ImGui::Text("Mode: %s", mesh.IsActiveRagdoll() ? "Active Ragdoll" : character.IsRagdoll() ? "Passive Ragdoll"
                                                                                                  : "Animation");
        ImGui::Text("Kinematic bodies: %zu", runtime.GetKinematicBodyCount());
    }
    if (!mesh.GetPhysicsError().empty())
    {
        ImGui::TextWrapped("%s", mesh.GetPhysicsError().c_str());
    }
    ImGui::BeginDisabled(character.IsRagdoll());
    ImGui::SetNextItemWidth(290);
    ImGui::InputText("##CharacterPAPath", m_CharacterPAPath, sizeof(m_CharacterPAPath));
    if (ImGui::Button("Load character PA"))
    {
        PARS::PhysicsAsset candidate;
        if (candidate.Load(m_CharacterPAPath, m_RagdollMessage) && mesh.SetPhysicsAsset(candidate, m_RagdollMessage))
        {
            m_RagdollMessage = "PA loaded and assigned to the play character.";
        }
    }
    ImGui::SameLine();
    ImGui::BeginDisabled(!m_MeshViewer || m_MeshViewer->GetPhysicsAsset().bodies.empty());
    if (ImGui::Button("Use editor PA") && mesh.SetPhysicsAsset(m_MeshViewer->GetPhysicsAsset(), m_RagdollMessage))
    {
        m_RagdollMessage = "Current editor PA assigned to the play character.";
    }
    ImGui::EndDisabled();
    ImGui::EndDisabled();
    ImGui::BeginDisabled(!asset || mesh.HasPendingDriveSettings());
    if (ImGui::Button("Save character PA"))
    {
        if (mesh.GetPhysicsAsset()->Save(m_CharacterPAPath, m_RagdollMessage))
        {
            m_RagdollMessage = "Character PA saved.";
        }
    }
    ImGui::EndDisabled();
    asset = mesh.GetPhysicsAsset();
    ImGui::BeginDisabled(!asset || asset->bodies.empty() || !m_World.GetPhysicsWorld());
    ImGui::BeginDisabled(mesh.GetPhysicsMode() == PARS::PhysicsAnimationMode::PassiveRagdoll);
    if (ImGui::Button("Passive Ragdoll") && character.StartRagdoll(m_RagdollMessage))
    {
        m_RagdollMessage = "Passive ragdoll: angular drives off, pelvis released.";
    }
    ImGui::EndDisabled();
    ImGui::SameLine();
    ImGui::BeginDisabled(mesh.IsActiveRagdoll() || mesh.HasPendingDriveSettings());
    if (ImGui::Button("Active Ragdoll"))
    {
        const bool entering = !character.IsRagdoll();
        if (character.SetPhysicsMode(PARS::PhysicsAnimationMode::ActiveRagdoll, m_RagdollMessage))
        {
            if (auto* animation = GetPreviewAnimation())
            {
                animation->SetAutoCycleEnabled(false);
            }
            if (!entering || mesh.GetRagdoll().SetRootSupported(m_ActiveSupport, m_RagdollMessage))
            {
                m_RagdollMessage = "Active ragdoll: animation targets drive the physical joints.";
            }
        }
    }
    ImGui::EndDisabled();
    ImGui::EndDisabled();
    bool supported = character.IsRagdoll() ? runtime.IsRootSupported() : m_ActiveSupport;
    ImGui::BeginDisabled(mesh.GetPhysicsMode() == PARS::PhysicsAnimationMode::PassiveRagdoll);
    if (ImGui::Checkbox("Pelvis force / torque support", &supported))
    {
        if (!character.IsRagdoll() || mesh.GetRagdoll().SetRootSupported(supported, m_RagdollMessage))
        {
            m_ActiveSupport = supported;
        }
    }
    ImGui::EndDisabled();
    if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled))
    {
        ImGui::SetTooltip("Dynamic pelvis: limited forces follow animation position, limited torques follow rotation. Contacts and impacts can displace it. Passive disables this support.");
    }
    if (mesh.IsActiveRagdoll())
    {
        bool drives = runtime.AreDrivesEnabled();
        if (ImGui::Checkbox("Angular drives", &drives))
        {
            mesh.GetRagdoll().SetDrivesEnabled(drives);
        }
        ImGui::Text("Driven joints: %zu", runtime.GetDrivenJointCount());
        ImGui::Text("Max target error: %.1f deg", XMConvertToDegrees(runtime.GetMaxDriveError()));
        ImGui::Text("Target limit correction: %.1f deg", XMConvertToDegrees(runtime.GetMaxTargetClampAngle()));
        if (runtime.IsRootSupported())
        {
            ImGui::Text("Pelvis target error: %.1f cm / %.1f deg", runtime.GetRootPositionError() * 100.0f,
                XMConvertToDegrees(runtime.GetRootRotationError()));
        }
        ImGui::TextUnformatted(character.CanControlMovement() ? "WASD: move | Shift: run" : "Movement requires pelvis support.");
    }
    ImGui::BeginDisabled(!asset);
    if (ImGui::Button("Use default PD gains"))
    {
        auto candidate = *asset;
        PARS::ConfigureDefaultDrives(candidate);
        if (mesh.QueueDriveSettings(candidate, m_RagdollMessage))
        {
            m_RagdollMessage = "Default gains queued for the next physics step.";
        }
    }
    ImGui::BeginDisabled(!m_MeshViewer || m_MeshViewer->GetPhysicsAsset().bodies.empty());
    if (ImGui::Button("Apply editor drive settings") && mesh.QueueDriveSettings(m_MeshViewer->GetPhysicsAsset(), m_RagdollMessage))
    {
        m_RagdollMessage = "Drive settings queued. Frames, shapes and limits use the assigned character PA.";
    }
    ImGui::EndDisabled();
    ImGui::EndDisabled();
    if (mesh.HasPendingDriveSettings())
    {
        ImGui::TextUnformatted("Drive settings pending next physics step.");
    }
    if (character.IsRagdoll())
    {
        if (ImGui::Button("Stop / return to animation") && character.StopRagdoll(m_RagdollMessage))
        {
            m_RagdollMessage = "Returned to animation and character control at the current movement target.";
        }
        if (ImGui::Button("Restart ragdoll"))
        {
            const auto mode = mesh.GetPhysicsMode();
            const bool support = runtime.IsRootSupported();
            if (character.StopRagdoll(m_RagdollMessage) && character.SetPhysicsMode(mode, m_RagdollMessage) &&
                mode == PARS::PhysicsAnimationMode::ActiveRagdoll)
            {
                mesh.GetRagdoll().SetRootSupported(support, m_RagdollMessage);
            }
        }
        if (character.IsRagdoll())
        {
            if (ImGui::Button("Push ragdoll"))
            {
                mesh.GetRagdoll().AddVelocity(Vector3(1.5f, 2, 0));
            }
            ImGui::Text("Joint separation: %.4f m", mesh.GetRagdoll().GetMaxAnchorError());
        }
    }
    bool paused = m_World.IsPhysicsPaused();
    if (ImGui::Checkbox("Pause world##Ragdoll", &paused))
    {
        m_World.SetPhysicsPaused(paused);
    }
    ImGui::SameLine();
    ImGui::BeginDisabled(!paused);
    if (ImGui::Button("Step 1/60##Ragdoll"))
    {
        m_World.RequestPhysicsStep();
    }
    ImGui::EndDisabled();
    if (!m_RagdollMessage.empty())
    {
        ImGui::TextWrapped("%s", m_RagdollMessage.c_str());
    }
}

void ModelViewer::RenderUI(GraphicsContext& context)
{
#if defined(_M_X64)
    if (m_PhysicsDemo)
    {
        m_PhysicsDemo->RenderUI();
        if (!m_PreviewMode)
        {
            return;
        }
    }
#endif
    if (m_PreviewMode)
    {
        if (ImGui::BeginMainMenuBar())
        {
            if (ImGui::BeginMenu("Window"))
            {
                if (ImGui::MenuItem("Skeletal Mesh Viewer", nullptr, false, m_MeshViewer != nullptr))
                {
                    m_MeshViewer->Open();
                }
#if defined(_M_X64)
                if (ImGui::MenuItem("Ragdoll Comparison", nullptr, false, m_Comparison != nullptr))
                {
                    m_Comparison->Open();
                }
#endif
                ImGui::EndMenu();
            }
            ImGui::EndMainMenuBar();
        }
        ImGui::SetNextWindowPos(ImVec2(ImGui::GetIO().DisplaySize.x - 24.0f, 70.0f),
            ImGuiCond_FirstUseEver, ImVec2(1.0f, 0.0f));
        ImGui::SetNextWindowSize(ImVec2(330.0f, 0.0f), ImGuiCond_FirstUseEver);
        if (ImGui::Begin("Character Animation", nullptr, ImGuiWindowFlags_AlwaysAutoResize))
        {
#if defined(_M_X64)
            if (m_Comparison && ImGui::Button("Start Walking comparison"))
            {
                auto character = GetPreviewCharacter();
                m_Comparison->RequestWalkingComparison(m_MeshViewer ? &m_MeshViewer->GetPhysicsAsset() : nullptr,
                    character ? character->GetMesh().GetPhysicsAsset() : nullptr);
            }
#endif
            auto animation = GetPreviewAnimation();
            if (!m_PreviewReady || !animation)
            {
                ImGui::TextWrapped("%s", m_PreviewError.empty() ? "Preview character is no longer in the world." : m_PreviewError.c_str());
            }
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
                if (auto character = GetPreviewCharacter())
                {
                    DrawRagdollPanel(*character);
                }
                ImGui::Separator();
                ImGui::BeginDisabled(GetPreviewCharacter() && GetPreviewCharacter()->GetMesh().GetPhysicsMode() == PARS::PhysicsAnimationMode::PassiveRagdoll);
                if (GetPlayerController())
                {
                    bool automatic = animation->IsLocomotionEnabled();
                    if (ImGui::Checkbox("Animation follows movement", &automatic))
                    {
                        animation->SetLocomotionEnabled(automatic);
                    }
                    bool freeCamera = m_FreeCamera;
                    if (ImGui::Checkbox("Free camera (Tab)", &freeCamera))
                    {
                        SetFreeCamera(freeCamera);
                    }
                }
                if (!animation->IsLocomotionEnabled())
                {
                    bool playing = animation->IsPlaying();
                    if (ImGui::Checkbox("Play", &playing))
                    {
                        animation->SetPlaying(playing);
                    }
                    ImGui::SameLine();
                    bool autoCycle = animation->IsAutoCycleEnabled();
                    if (ImGui::Checkbox("Auto cycle", &autoCycle))
                    {
                        animation->SetAutoCycleEnabled(autoCycle);
                    }
                    if (autoCycle)
                    {
                        ImGui::Text("Next animation in %.1f s", animation->GetTimeUntilNextAnimation());
                    }
                }
                for (uint32_t i = 0; i < kPreviewCount; ++i)
                {
                    if (i > 0)
                    {
                        ImGui::SameLine();
                    }
                    if (ImGui::Button(kPreviewNames[i]))
                    {
                        SelectPreview(i);
                    }
                }
                if (ImGui::Button("Reset camera"))
                {
                    ResetPreviewCamera();
                }
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
                    ImGui::TextWrapped(m_FreeCamera ? "Free camera: WASD + Q/E. Tab: control character." : "Character: WASD move, Shift run. Tab: free camera.");
                    ImGui::TextWrapped("Hold right mouse to look. Animation buttons enable manual preview.");
                }
                else
#endif
                    ImGui::TextWrapped("Hold right mouse and drag to orbit. Hold right mouse and scroll to zoom.");
            }
        }
        ImGui::End();
        if (m_MeshViewer)
        {
#if defined(_M_X64)
            m_MeshViewer->RenderUI(context, m_Comparison != nullptr);
            if (m_MeshViewer->ConsumeComparisonRequest() && m_Comparison)
            {
                auto character = GetPreviewCharacter();
                m_Comparison->RequestWalkingComparison(&m_MeshViewer->GetPhysicsAsset(),
                    character ? character->GetMesh().GetPhysicsAsset() : nullptr);
            }
#else
            m_MeshViewer->RenderUI(context);
#endif
        }
#if defined(_M_X64)
        if (m_Comparison)
        {
            auto character = GetPreviewCharacter();
            m_Comparison->DrawUI(m_MeshViewer ? &m_MeshViewer->GetPhysicsAsset() : nullptr,
                character ? character->GetMesh().GetPhysicsAsset() : nullptr);
        }
#endif
        return;
    }
    static bool showWindow = true;
    static bool showDemo = false;
    static float testValue = 0.5f;
    static int clicks = 0;
    static char text[64] = "Hello PARS!";

    if (ImGui::IsKeyPressed(ImGuiKey_F1, false))
    {
        showWindow = !showWindow;
    }

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
            {
                ++clicks;
            }
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
    {
        ImGui::ShowDemoWindow(&showDemo);
    }
}

void ModelViewer::RenderScene(void)
{
    GraphicsContext& gfxContext = GraphicsContext::Begin(L"Scene Render");

    uint32_t FrameIndex = TemporalEffects::GetFrameIndexMod2();
    const D3D12_VIEWPORT& viewport = m_MainViewport;
    const D3D12_RECT& scissor = m_MainScissor;

    bool physicsScene = false;
    bool hasModels = Renderer::HasWorldMeshes(m_World) || !m_ModelInst.IsNull();
#if defined(_M_X64)
    if (m_Comparison && m_Comparison->IsShowingScene())
    {
        hasModels = Renderer::HasWorldMeshes(m_Comparison->GetWorld());
    }
    if (m_PhysicsDemo)
    {
        physicsScene = true;
        m_PhysicsDemo->Render(gfxContext, m_Camera, viewport, scissor, m_Comparison && m_Comparison->IsShowingScene());
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
        {
            Sponza::RenderScene(gfxContext, m_Camera, viewport, scissor);
        }
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
        // 전역 상수를 갱신한다.
        float costheta = cosf(g_SunOrientation);
        float sintheta = sinf(g_SunOrientation);
        float cosphi = cosf(g_SunInclination * 3.14159f * 0.5f);
        float sinphi = sinf(g_SunInclination * 3.14159f * 0.5f);

        Vector3 SunDirection = Normalize(Vector3(costheta * cosphi, sinphi, sintheta * cosphi));
        Vector3 shadowCenter = physicsScene ? Vector3(0, 2, 0) : Vector3(0, -500.0f, 0);
#if defined(_M_X64)
        if (m_Comparison && m_Comparison->IsShowingScene())
        {
            shadowCenter = m_Comparison->GetViewCenter();
        }
#endif
        m_SunShadowCamera.UpdateMatrix(-SunDirection, shadowCenter,
            physicsScene ? Vector3(16, 10, 16) : Vector3(5000, 3000, 3000),
            (uint32_t)g_ShadowBuffer.GetWidth(), (uint32_t)g_ShadowBuffer.GetHeight(), 16);

        GlobalConstants globals;
        globals.ViewProjMatrix = m_Camera.GetViewProjMatrix();
        globals.SunShadowMatrix = m_SunShadowCamera.GetShadowMatrix();
        globals.CameraPos = m_Camera.GetPosition();
        globals.SunDirection = SunDirection;
        globals.SunIntensity = Vector3(Scalar(g_SunLightIntensity));

        // 깊이 렌더링을 시작한다.
        gfxContext.TransitionResource(g_SceneDepthBuffer, D3D12_RESOURCE_STATE_DEPTH_WRITE, true);
        // 바닥과 상자의 깊이를 유지해 캐릭터를 올바르게 가리도록 한다.
        if (!physicsScene)
        {
            gfxContext.ClearDepth(g_SceneDepthBuffer);
        }

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
            {
                gfxContext.ClearColor(g_SceneColorBuffer);
            }

            {
                ScopedTimer _prof(L"Render Color", gfxContext);

                gfxContext.TransitionResource(g_SSAOFullScreen, D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE);
                gfxContext.TransitionResource(g_SceneDepthBuffer, D3D12_RESOURCE_STATE_DEPTH_READ);
                gfxContext.SetRenderTarget(g_SceneColorBuffer.GetRTV(), g_SceneDepthBuffer.GetDSV_DepthReadOnly());
                gfxContext.SetViewportAndScissor(viewport, scissor);

                sorter.RenderMeshes(MeshSorter::kOpaque, gfxContext, globals);
            }

            if (!physicsScene)
            {
                Renderer::DrawSkybox(gfxContext, m_Camera, viewport, scissor);
            }

            sorter.RenderMeshes(MeshSorter::kTransparent, gfxContext, globals);
        }
    }

#if defined(_M_X64)
    if (m_PhysicsDemo && !(m_Comparison && m_Comparison->IsShowingScene()))
    {
        m_PhysicsDemo->RenderCapsules(gfxContext, m_Camera, viewport, scissor);
    }
#endif
    // 일부 시스템은 동적/스키닝 메시를 더 잘 추적하려고 픽셀별 속도 버퍼를 생성한다.
    // 이 경로는 물체를 정적으로 가정하고 카메라 움직임과 깊이 버퍼로 속도를 생성한다.
    // 속도 버퍼는 모든 시간적 효과와 모션 블러에 필요하다.
    MotionBlur::GenerateCameraVelocityBuffer(gfxContext, m_Camera, true);

    if (TemporalEffects::EnableTAA)
    {
        TemporalEffects::ResolveImage(gfxContext);
    }

    ParticleEffectManager::Render(gfxContext, m_Camera, g_SceneColorBuffer, g_SceneDepthBuffer, g_LinearDepth[FrameIndex]);

    // 두 효과를 함께 사용하는 방법을 마련하기 전까지는 둘 중 하나만 사용한다.
    if (DepthOfField::Enable)
    {
        DepthOfField::Render(gfxContext, m_Camera.GetNearClip(), m_Camera.GetFarClip());
    }
    else
    {
        MotionBlur::RenderObjectBlur(gfxContext, g_VelocityBuffer);
    }

    gfxContext.Finish();
}
