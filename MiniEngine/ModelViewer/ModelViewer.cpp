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
#include "ViewerLabels.h"
#include "SkeletalMeshViewer.h"
#include "GameFramework/PlayerController.h"
#include "GameFramework/SpringArmComponent.h"
#include "ShadowCamera.h"
#include "Display.h"
#include "imgui.h"
#include "imgui_stdlib.h"
#include <fstream>
#if defined(_M_X64)
#include "PhysicsDemo.h"
#include "RagdollComparisonRunner.h"
bool RunRagdollValidation(PARS::World& world, PARS::Character& character, const std::wstring& reportPath);
bool RunPelvisDriveValidation(PARS::World& world, PARS::Character& character, const std::wstring& reportPath,
    uint32_t profile, uint32_t clip);
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
                const std::wstring root = directory + L"/Assets/Characters/Mixamo/";
                bool found = true;
                for (const wchar_t* file : kPreviewFiles)
                {
                    const DWORD attributes = GetFileAttributesW((root + L"Converted/" + file).c_str());
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
        const bool captureReturn = CommandLineArgs::GetString(L"comparisonreturncapture", comparisonCapture);
        if (m_Comparison && (captureReturn || CommandLineArgs::GetString(L"comparisoncapture", comparisonCapture)))
        {
            // Read back the rendered buffers; PrintWindow cannot capture a DX12 flip-model window.
            if (++m_ComparisonCaptureFrames == 10)
            {
                g_OverlayBuffer.ExportToFile(comparisonCapture + L".launch.ui");
            }
            const bool returned = captureReturn && !m_Comparison->IsShowingScene() && !m_WasComparing &&
                !m_Comparison->GetResults().GetFrames().empty();
            if (returned || (!captureReturn && m_Comparison->GetState() == RagdollComparisonRunner::State::Complete))
            {
                g_OverlayBuffer.ExportToFile(comparisonCapture + L".ui");
                g_SceneColorBuffer.ExportToFile(comparisonCapture + L".scene");
                if (captureReturn)
                {
                    const auto* player = GetPlayerController();
                    const bool controlsRestored = player && player->IsInputEnabled() && !m_FreeCamera &&
                        !(m_MeshViewer && m_MeshViewer->WantsInput());
                    const bool springArmApplied = m_SpringArm && &m_SpringArm->GetOwner() == GetPreviewCharacter() &&
                        float(Length(m_Camera.GetPosition() - m_SpringArm->GetCameraLocation())) < 1e-4f;
                    std::ofstream report(comparisonCapture + L".txt");
                    report << (controlsRestored && springArmApplied ? "PASS" : "FAIL") << ": returned to the player scene with movement input and spring arm camera\n"
                           << "samples=" << m_Comparison->GetResults().GetFrames().size() << '\n'
                           << "spring_arm_m=" << (m_SpringArm ? m_SpringArm->GetCurrentArmLength() : -1) << '\n';
                }
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
    void UpdateFollowCamera(float deltaTime = 0);
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
    void SaveDefaultPhysicsAsset();

    Camera m_Camera;
    unique_ptr<CameraController> m_CameraController;

    D3D12_VIEWPORT m_MainViewport;
    D3D12_RECT m_MainScissor;

    ModelInstance m_ModelInst;
    PARS::World m_World;
    PARS::ActorHandle m_PreviewCharacter; // 참조만 보관하며, Actor의 소유자는 World다.
    PARS::ActorHandle m_PlayerController;
    PARS::SpringArmComponent* m_SpringArm = nullptr; // 캐릭터 Actor가 소유한다.
    bool m_FreeCamera = false;
    float m_ControlYaw = XM_PI;
    float m_ControlPitch = 0.28f;
    bool m_PreviewMode = false;
    bool m_PreviewReady = false;
    std::string m_PreviewError;
    std::string m_RagdollMessage;
    bool m_ActiveSupport = true;
    std::string m_CharacterPAPath = "Mixamo.physics.json";
    std::string m_DefaultPAPath;
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
            if (CommandLineArgs::GetString(L"pelvistest", ragdollReport))
            {
                uint32_t profile = 2;
                uint32_t clip = 0;
                CommandLineArgs::GetInteger(L"turnprofile", profile);
                CommandLineArgs::GetInteger(L"testclip", clip);
                if (auto* character = GetPreviewCharacter())
                {
                    RunPelvisDriveValidation(m_World, *character, ragdollReport, profile, clip);
                }
                m_RagdollTestDone = true;
            }
            else if (CommandLineArgs::GetString(L"ragdolltest", ragdollReport))
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
        m_PreviewError = u8"Assets/Characters/Mixamo/Converted에서 대기, 걷기, 달리기 애니메이션을 찾을 수 없습니다.";
        return;
    }

    std::shared_ptr<const Model> models[kPreviewCount];
    for (uint32_t i = 0; i < kPreviewCount; ++i)
    {
        auto& model = models[i];
        model = Renderer::LoadModel(root + L"Converted/" + kPreviewFiles[i], forceRebuild);
        if (!model || model->m_NumAnimations == 0 || model->m_Animations[0].duration <= 0.0f)
        {
            m_PreviewError = std::string(u8"애니메이션을 불러오지 못했습니다: ") + kPreviewNames[i];
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
                m_PreviewError = u8"캐릭터 모델에 사용할 수 있는 스켈레톤이 없습니다.";
                character->Destroy();
                return;
            }
            mesh.SetAnimInstance(std::make_unique<PreviewAnimInstance>(mesh));
        }
        if (!mesh.GetAnimInstance().AddAnimation(kPreviewNames[i], model))
        {
            m_PreviewError = std::string(u8"애니메이션과 캐릭터의 스켈레톤이 다릅니다: ") + kPreviewNames[i];
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
    // 실행 작업 폴더와 관계없이 캐릭터 에셋 옆의 PA를 저장/복원한다.
    const std::wstring defaultPAPath = root + L"Mixamo.physics.json";
    const int pathLength = static_cast<int>(defaultPAPath.size());
    const int utf8Length = WideCharToMultiByte(CP_UTF8, 0, defaultPAPath.data(), pathLength, nullptr, 0, nullptr, nullptr);
    m_DefaultPAPath.resize(utf8Length);
    if (utf8Length > 0)
    {
        WideCharToMultiByte(CP_UTF8, 0, defaultPAPath.data(), pathLength, &m_DefaultPAPath[0], utf8Length, nullptr, nullptr);
    }
    m_CharacterPAPath = m_DefaultPAPath;
    bool loadedPA = false;
    std::string loadError;
    if (GetFileAttributesW(defaultPAPath.c_str()) != INVALID_FILE_ATTRIBUTES)
    {
        PARS::PhysicsAsset savedPA;
        loadedPA = savedPA.Load(m_DefaultPAPath, loadError) && mesh.SetPhysicsAsset(savedPA, loadError);
    }
    else if (GetLastError() != ERROR_FILE_NOT_FOUND && GetLastError() != ERROR_PATH_NOT_FOUND)
    {
        loadError = u8"기본 PA 파일에 접근할 수 없습니다.";
    }
    if (loadedPA)
    {
        m_RagdollMessage = u8"저장된 기본 PA를 자동 적용했습니다: " + m_DefaultPAPath;
    }
    else
    {
        PARS::PhysicsSkeleton physicsSkeleton;
        PARS::PhysicsAsset defaultPA;
        const std::vector<GraphNode> reference(models[0]->m_SceneGraph.get(), models[0]->m_SceneGraph.get() + models[0]->m_NumNodes);
        if (mesh.BuildPhysicsSkeleton(physicsSkeleton, m_RagdollMessage) &&
            PARS::BuildMixamoPhysicsAsset(physicsSkeleton, reference, defaultPA, m_RagdollMessage) &&
            mesh.SetPhysicsAsset(defaultPA, m_RagdollMessage))
        {
            m_RagdollMessage = u8"코드에서 생성한 Mixamo 기본 PA를 적용했습니다.";
        }
        if (!loadError.empty())
        {
            m_RagdollMessage = u8"저장된 PA를 불러오지 못했습니다: " + loadError + "\n" + m_RagdollMessage;
        }
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
        m_SpringArm = &character->CreateDefaultSubobject<PARS::SpringArmComponent>("SpringArmComponent");
        m_SpringArm->AttachToComponent(character->GetCapsuleComponent());
        m_SpringArm->SetRelativeLocation(Vector3(0, .55f, 0));
        // 래그돌의 최종 위치와 렌더 프레임 시간을 사용하도록 World::Tick 뒤에 갱신한다.
        m_SpringArm->SetTickEnabled(false);
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
        m_MeshViewer->SetDefaultPhysicsAssetPath(m_DefaultPAPath);
        m_MeshViewer->CompletePhysicsSave(loadedPA, m_RagdollMessage);
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
            m_Comparison->RequestLocomotionComparison(m_MeshViewer ? &m_MeshViewer->GetPhysicsAsset() : nullptr,
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
    if (m_SpringArm)
    {
        m_SpringArm->ResetLagState();
    }
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
        if (m_SpringArm)
        {
            m_SpringArm->ResetLagState();
        }
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

void ModelViewer::UpdateFollowCamera(float deltaTime)
{
    auto character = GetPreviewCharacter();
    if (!character || !m_SpringArm)
    {
        return;
    }
    const Vector3 target = character->IsRagdoll() ? character->GetMesh().GetRagdoll().GetPosition() + Vector3(0, .25f, 0) : character->GetActorLocation() + Vector3(0, 0.55f, 0);
    m_SpringArm->SetWorldTransform(AffineTransform(Matrix3(kIdentity), target));
    m_SpringArm->SetControlRotation(m_ControlYaw, m_ControlPitch);
    m_SpringArm->UpdateArm(deltaTime);
    m_Camera.SetPosition(m_SpringArm->GetCameraLocation());
    m_Camera.SetLookDirection(m_SpringArm->GetViewDirection(), Vector3(kYUnitVector));
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
    m_SpringArm = nullptr;
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
        const bool returnToPlayer = m_Comparison->ConsumePlayerControlRequest();
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
        if (returnToPlayer || (m_WasComparing && !m_Comparison->IsShowingScene()))
        {
            if (returnToPlayer && m_MeshViewer)
            {
                m_MeshViewer->Close();
            }
            SetFreeCamera(returnToPlayer ? false : m_PreComparisonFreeCamera);
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
        UpdateFollowCamera(deltaT);
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

void ModelViewer::SaveDefaultPhysicsAsset()
{
    auto* character = GetPreviewCharacter();
    if (!character || !m_MeshViewer || m_DefaultPAPath.empty())
    {
        return;
    }
    const auto& asset = m_MeshViewer->GetPhysicsAsset();
    bool saved = false;
    std::string error;
    // 기존 캐릭터에서 실제 물리 상태를 생성할 수 있는지 확인한 뒤 파일을 교체한다.
    if (!character->GetMesh().SetPhysicsAsset(asset, error))
    {
        m_RagdollMessage = u8"PA 적용에 실패했습니다. 기본 파일은 유지됩니다: " + error;
    }
    else if (!asset.Save(m_DefaultPAPath, error))
    {
        m_RagdollMessage = u8"현재 캐릭터에는 적용했지만 저장하지 못했습니다. 다시 저장하세요: " + error;
    }
    else
    {
        saved = true;
        m_RagdollMessage = u8"현재 캐릭터에 적용하고 기본 PA로 저장했습니다. 다음 실행 때 자동 적용됩니다.";
    }
    m_MeshViewer->CompletePhysicsSave(saved, m_RagdollMessage);
}

void ModelViewer::DrawRagdollPanel(PARS::Character& character)
{
    if (!ImGui::CollapsingHeader(u8"래그돌 / 물리 에셋", ImGuiTreeNodeFlags_DefaultOpen))
    {
        return;
    }
    auto& mesh = character.GetMesh();
    const auto* asset = mesh.GetPhysicsAsset();
    if (asset)
    {
        ImGui::Text(u8"적용된 PA: 바디 %zu개 / 관절 %zu개", asset->bodies.size(), asset->constraints.size());
    }
    else
    {
        ImGui::TextUnformatted(u8"물리 에셋이 적용되지 않았습니다.");
    }
    const auto& runtime = mesh.GetRagdoll();
    ImGui::Text(u8"PhysX: 바디 %zu개 / 관절 %zu개", runtime.GetBodyCount(), runtime.GetJointCount());
    if (runtime.IsActive())
    {
        ImGui::Text(u8"모드: %s", mesh.IsActiveRagdoll() ? u8"액티브 래그돌" : character.IsRagdoll() ? u8"패시브 래그돌"
                                                                                                     : u8"애니메이션");
        ImGui::Text(u8"키네마틱 바디: %zu개", runtime.GetKinematicBodyCount());
    }
    if (!mesh.GetPhysicsError().empty())
    {
        ImGui::TextWrapped("%s", mesh.GetPhysicsError().c_str());
    }
    ImGui::BeginDisabled(character.IsRagdoll());
    ImGui::SetNextItemWidth(290);
    ImGui::InputText("##CharacterPAPath", &m_CharacterPAPath);
    if (ImGui::Button(u8"캐릭터 PA 불러오기"))
    {
        PARS::PhysicsAsset candidate;
        if (candidate.Load(m_CharacterPAPath, m_RagdollMessage) && mesh.SetPhysicsAsset(candidate, m_RagdollMessage))
        {
            m_RagdollMessage = u8"PA를 불러와 플레이 캐릭터에 적용했습니다.";
        }
    }
    ImGui::SameLine();
    ImGui::BeginDisabled(!m_MeshViewer || m_MeshViewer->GetPhysicsAsset().bodies.empty());
    if (ImGui::Button(u8"에디터 PA 적용") && mesh.SetPhysicsAsset(m_MeshViewer->GetPhysicsAsset(), m_RagdollMessage))
    {
        m_RagdollMessage = u8"현재 에디터 PA를 플레이 캐릭터에 적용했습니다.";
    }
    ImGui::EndDisabled();
    ImGui::EndDisabled();
    ImGui::BeginDisabled(!asset || mesh.HasPendingDriveSettings());
    if (ImGui::Button(u8"캐릭터 PA 저장"))
    {
        if (mesh.GetPhysicsAsset()->Save(m_CharacterPAPath, m_RagdollMessage))
        {
            m_RagdollMessage = u8"캐릭터 PA를 저장했습니다.";
        }
    }
    ImGui::EndDisabled();
    asset = mesh.GetPhysicsAsset();
    ImGui::BeginDisabled(!asset || asset->bodies.empty() || !m_World.GetPhysicsWorld());
    ImGui::BeginDisabled(mesh.GetPhysicsMode() == PARS::PhysicsAnimationMode::PassiveRagdoll);
    if (ImGui::Button(u8"패시브 래그돌") && character.StartRagdoll(m_RagdollMessage))
    {
        m_RagdollMessage = u8"패시브 래그돌: 회전 드라이브를 끄고 골반 지지를 해제했습니다.";
    }
    ImGui::EndDisabled();
    ImGui::SameLine();
    ImGui::BeginDisabled(mesh.IsActiveRagdoll() || mesh.HasPendingDriveSettings());
    if (ImGui::Button(u8"액티브 래그돌"))
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
                m_RagdollMessage = u8"액티브 래그돌: 애니메이션 목표에 맞춰 물리 관절을 구동합니다.";
            }
        }
    }
    ImGui::EndDisabled();
    ImGui::EndDisabled();
    bool supported = character.IsRagdoll() ? runtime.IsRootSupported() : m_ActiveSupport;
    ImGui::BeginDisabled(mesh.GetPhysicsMode() == PARS::PhysicsAnimationMode::PassiveRagdoll);
    if (ImGui::Checkbox(u8"골반 힘 / 토크 지지", &supported))
    {
        if (!character.IsRagdoll() || mesh.GetRagdoll().SetRootSupported(supported, m_RagdollMessage))
        {
            m_ActiveSupport = supported;
        }
    }
    ImGui::EndDisabled();
    if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled))
    {
        ImGui::SetTooltip(u8"동적 골반은 제한된 힘과 토크로 애니메이션의 위치와 회전을 따라갑니다. 접촉과 충격으로 목표에서 벗어날 수 있습니다. 패시브 모드에서는 지지를 해제합니다.");
    }
    if (mesh.IsActiveRagdoll())
    {
        bool drives = runtime.AreDrivesEnabled();
        if (ImGui::Checkbox(u8"회전 드라이브", &drives))
        {
            mesh.GetRagdoll().SetDrivesEnabled(drives);
        }
        ImGui::Text(u8"구동 관절: %zu개", runtime.GetDrivenJointCount());
        ImGui::Text(u8"최대 목표 오차: %.1f도", XMConvertToDegrees(runtime.GetMaxDriveError()));
        ImGui::Text(u8"제한에 따른 목표 보정: %.1f도", XMConvertToDegrees(runtime.GetMaxTargetClampAngle()));
        if (runtime.IsRootSupported())
        {
            ImGui::Text(u8"골반 목표 오차: %.1f cm / %.1f도", runtime.GetRootPositionError() * 100.0f,
                XMConvertToDegrees(runtime.GetRootRotationError()));
        }
        ImGui::TextUnformatted(character.CanControlMovement() ? u8"WASD: 이동 | Shift: 달리기" : u8"이동하려면 골반 지지가 필요합니다.");
    }
    ImGui::BeginDisabled(!asset);
    if (ImGui::Button(u8"가속도 PD 기본값 적용"))
    {
        auto candidate = *asset;
        PARS::ConfigureDefaultDrives(candidate);
        if (mesh.QueueDriveSettings(candidate, m_RagdollMessage))
        {
            m_RagdollMessage = u8"다음 물리 스텝에 가속도 PD 기본값을 적용합니다.";
        }
    }
    ImGui::BeginDisabled(!m_MeshViewer || m_MeshViewer->GetPhysicsAsset().bodies.empty());
    if (ImGui::Button(u8"에디터 드라이브 적용") && mesh.QueueDriveSettings(m_MeshViewer->GetPhysicsAsset(), m_RagdollMessage))
    {
        m_RagdollMessage = u8"다음 물리 스텝에 드라이브 설정을 적용합니다. 좌표계, 형상, 제한값은 캐릭터에 적용된 PA를 사용합니다.";
    }
    ImGui::EndDisabled();
    ImGui::EndDisabled();
    if (mesh.HasPendingDriveSettings())
    {
        ImGui::TextUnformatted(u8"다음 물리 스텝에 드라이브 설정을 적용합니다.");
    }
    if (character.IsRagdoll())
    {
        if (ImGui::Button(u8"종료 / 애니메이션 복귀") && character.StopRagdoll(m_RagdollMessage))
        {
            m_RagdollMessage = u8"현재 이동 목표 위치에서 애니메이션과 캐릭터 조작으로 돌아왔습니다.";
        }
        if (ImGui::Button(u8"래그돌 다시 시작"))
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
            if (ImGui::Button(u8"래그돌 밀기"))
            {
                mesh.GetRagdoll().AddVelocity(Vector3(1.5f, 2, 0));
            }
            ImGui::Text(u8"관절 연결점 간격: %.4f m", mesh.GetRagdoll().GetMaxAnchorError());
        }
    }
    bool paused = m_World.IsPhysicsPaused();
    if (ImGui::Checkbox(u8"월드 일시정지##Ragdoll", &paused))
    {
        m_World.SetPhysicsPaused(paused);
    }
    ImGui::SameLine();
    ImGui::BeginDisabled(!paused);
    if (ImGui::Button(u8"1/60초 진행##Ragdoll"))
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
            if (ImGui::BeginMenu(u8"창"))
            {
                if (ImGui::MenuItem(u8"스켈레탈 메시 뷰어", nullptr, false, m_MeshViewer != nullptr))
                {
                    m_MeshViewer->Open();
                }
#if defined(_M_X64)
                if (ImGui::MenuItem(u8"래그돌 비교", nullptr, false, m_Comparison != nullptr))
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
        if (ImGui::Begin(u8"캐릭터 애니메이션", nullptr, ImGuiWindowFlags_AlwaysAutoResize))
        {
#if defined(_M_X64)
            if (m_Comparison && ImGui::Button(u8"시뮬레이션 A 시작"))
            {
                auto character = GetPreviewCharacter();
                m_Comparison->RequestLocomotionComparison(m_MeshViewer ? &m_MeshViewer->GetPhysicsAsset() : nullptr,
                    character ? character->GetMesh().GetPhysicsAsset() : nullptr);
            }
            if (m_Comparison)
            {
                ImGui::SameLine();
                if (ImGui::Button(u8"시뮬레이션 B 시작"))
                {
                    auto character = GetPreviewCharacter();
                    m_Comparison->RequestLocomotionComparison(m_MeshViewer ? &m_MeshViewer->GetPhysicsAsset() : nullptr,
                        character ? character->GetMesh().GetPhysicsAsset() : nullptr, RagdollComparisonRunner::Simulation::B);
                }
                if (m_Comparison->IsShowingScene() && ImGui::Button(u8"캐릭터 조작으로 돌아가기 (WASD)"))
                {
                    m_Comparison->ReturnToPlayer();
                }
            }
#endif
            auto animation = GetPreviewAnimation();
            if (!m_PreviewReady || !animation)
            {
                ImGui::TextWrapped("%s", m_PreviewError.empty() ? u8"월드에 미리보기 캐릭터가 없습니다." : m_PreviewError.c_str());
            }
            else
            {
                ImGui::Text(u8"재생 중: %s", ViewerLabels::Animation(animation->GetAnimationName(animation->GetCurrentAnimationIndex())));
                if (auto character = GetPreviewCharacter())
                {
                    const auto& movement = character->GetCharacterMovement();
                    ImGui::Text(u8"속도: %.2f m/s | %s", movement.GetGroundSpeed(),
                        movement.IsMovingOnGround() ? u8"지면 접촉" : u8"공중");
                    ImGui::Text(u8"블렌딩: %.0f%%", animation->GetBlendAlpha() * 100.0f);
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
                    if (ImGui::Checkbox(u8"이동에 맞춰 애니메이션 재생", &automatic))
                    {
                        animation->SetLocomotionEnabled(automatic);
                    }
                    bool freeCamera = m_FreeCamera;
                    if (ImGui::Checkbox(u8"자유 카메라 (Tab)", &freeCamera))
                    {
                        SetFreeCamera(freeCamera);
                    }
                }
                if (!animation->IsLocomotionEnabled())
                {
                    bool playing = animation->IsPlaying();
                    if (ImGui::Checkbox(u8"재생", &playing))
                    {
                        animation->SetPlaying(playing);
                    }
                    ImGui::SameLine();
                    bool autoCycle = animation->IsAutoCycleEnabled();
                    if (ImGui::Checkbox(u8"자동 전환", &autoCycle))
                    {
                        animation->SetAutoCycleEnabled(autoCycle);
                    }
                    if (autoCycle)
                    {
                        ImGui::Text(u8"%.1f초 후 다음 애니메이션", animation->GetTimeUntilNextAnimation());
                    }
                }
                for (uint32_t i = 0; i < kPreviewCount; ++i)
                {
                    if (i > 0)
                    {
                        ImGui::SameLine();
                    }
                    if (ImGui::Button(ViewerLabels::Animation(kPreviewNames[i])))
                    {
                        SelectPreview(i);
                    }
                }
                if (ImGui::Button(u8"카메라 초기화"))
                {
                    ResetPreviewCamera();
                }
                ImGui::EndDisabled();
                if (m_SpringArm && ImGui::CollapsingHeader(u8"스프링암 카메라"))
                {
                    auto settings = m_SpringArm->GetSettings();
                    ImGui::SetNextItemWidth(110);
                    bool changed = ImGui::DragFloat(u8"기본 거리 (m)", &settings.targetArmLength, .1f, .5f, 8, "%.1f", ImGuiSliderFlags_AlwaysClamp);
                    ImGui::SetNextItemWidth(110);
                    changed |= ImGui::DragFloat(u8"따라오기 속도", &settings.cameraLagSpeed, .5f, 0, 30, "%.1f", ImGuiSliderFlags_AlwaysClamp);
                    ImGui::SetNextItemWidth(110);
                    changed |= ImGui::DragFloat(u8"거리 복원 속도", &settings.armReturnSpeed, .5f, 0, 30, "%.1f", ImGuiSliderFlags_AlwaysClamp);
                    if (changed)
                    {
                        m_SpringArm->SetSettings(settings);
                    }
                    ImGui::TextWrapped(u8"속도 0: 즉시 반영. 충돌 시에는 즉시 거리를 줄입니다.");
                    ImGui::Text(u8"현재 거리: %.2f m | 장애물: %s", m_SpringArm->GetCurrentArmLength(),
                        m_SpringArm->IsCollisionFixApplied() ? u8"감지" : u8"없음");
                }
#if defined(_M_X64)
                if (m_PhysicsDemo)
                {
                    if (ImGui::Button(u8"캐릭터 초기화"))
                    {
                        if (auto character = GetPreviewCharacter())
                        {
                            if (character->StopRagdoll(m_RagdollMessage))
                            {
                                character->SetActorLocation(Vector3(2, 0.95f, 0));
                                character->GetCharacterMovement().StopMovementImmediately();
                                if (m_SpringArm)
                                {
                                    m_SpringArm->ResetLagState();
                                }
                            }
                        }
                    }
                    ImGui::TextWrapped(m_FreeCamera ? u8"자유 카메라: WASD + Q/E 이동. Tab: 캐릭터 조작." : u8"캐릭터: WASD 이동, Shift 달리기. Tab: 자유 카메라.");
                    ImGui::TextWrapped(u8"마우스 오른쪽 버튼을 누르면 시점을 움직입니다. 애니메이션 버튼으로 수동 미리보기를 시작합니다.");
                }
                else
#endif
                    ImGui::TextWrapped(u8"우클릭 드래그로 시점을 회전하고 우클릭 상태에서 휠로 확대/축소합니다.");
            }
        }
        ImGui::End();
        if (m_MeshViewer)
        {
            const auto* character = GetPreviewCharacter();
            const bool physicsSaveAvailable = character && !character->IsRagdoll();
#if defined(_M_X64)
            m_MeshViewer->RenderUI(context, m_Comparison != nullptr, physicsSaveAvailable);
            const auto comparison = m_MeshViewer->ConsumeComparisonRequest();
            if (comparison != SkeletalMeshViewer::ComparisonRequest::None && m_Comparison)
            {
                m_Comparison->RequestLocomotionComparison(&m_MeshViewer->GetPhysicsAsset(),
                    character ? character->GetMesh().GetPhysicsAsset() : nullptr,
                    comparison == SkeletalMeshViewer::ComparisonRequest::B ? RagdollComparisonRunner::Simulation::B : RagdollComparisonRunner::Simulation::A);
            }
#else
            m_MeshViewer->RenderUI(context, false, physicsSaveAvailable);
#endif
            if (m_MeshViewer->ConsumePhysicsSaveRequest())
            {
                SaveDefaultPhysicsAsset();
            }
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
    static char text[64] = u8"PARS에 오신 것을 환영합니다!";

    if (ImGui::IsKeyPressed(ImGuiKey_F1, false))
    {
        showWindow = !showWindow;
    }

    if (showWindow)
    {
        ImGui::SetNextWindowPos(ImVec2(30.0f, 80.0f), ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowSize(ImVec2(420.0f, 330.0f), ImGuiCond_FirstUseEver);
        if (ImGui::Begin(u8"PARS - ImGui 테스트", &showWindow))
        {
            ImGui::TextColored(ImVec4(0.4f, 0.95f, 0.65f, 1.0f), u8"Win32 + DirectX 12 연결됨");
            ImGui::Text("%.1f FPS", ImGui::GetIO().Framerate);
            ImGui::Separator();
            ImGui::SliderFloat(u8"테스트 값", &testValue, 0.0f, 1.0f);
            if (ImGui::Button(u8"클릭"))
            {
                ++clicks;
            }
            ImGui::SameLine();
            ImGui::Text(u8"%d번 클릭했습니다.", clicks);
            ImGui::InputText(u8"텍스트", text, IM_ARRAYSIZE(text));
            ImGui::Checkbox(u8"ImGui 예제 창 보기", &showDemo);
            ImGui::Separator();
            ImGui::TextWrapped(u8"제목 표시줄을 드래그하면 창이 이동합니다. F1로 패널을 숨기거나 표시합니다.");
            ImGui::TextWrapped(u8"WASD로 이동하고 Q/E로 아래/위로 움직입니다. UI 바깥에서 우클릭한 채로 시점을 움직입니다.");
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
