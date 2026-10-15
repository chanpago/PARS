#pragma once

#include "GraphicsCore.h"
#include "GameFramework/World.h"
#include "Camera.h"
#include "ColorBuffer.h"
#include "DepthBuffer.h"
#include "DescriptorHeap.h"
#include "PipelineState.h"
#include "RootSignature.h"
#include <memory>

class Model;
namespace PARS { class SkeletalMeshComponent; }

// An editor preview scene. It shares immutable assets with the game, but owns
// its pose, camera and render targets and has no physics/controller attached.
class SkeletalMeshViewer
{
public:
    ~SkeletalMeshViewer() { Cleanup(); }
    bool Initialize(const std::shared_ptr<const Model>* models, const char* const* names, uint32_t count);
    void Cleanup(); // Call after the GPU is idle, before renderer/ImGui shutdown.
    void Update(float deltaTime);
    void RenderUI(GraphicsContext& context);
    void Open() { m_Open = true; m_FocusRequested = true; }
    bool WantsInput() const { return m_Open && m_Focused; }

private:
    void ResetCamera();
    void RenderPreview(GraphicsContext& context, float aspect);
    PARS::World m_World;
    PARS::SkeletalMeshComponent* m_Mesh = nullptr; // Owned by the preview World.
    Math::Camera m_Camera;
    Math::Vector3 m_Target{Math::kZero};
    float m_Radius = 1;
    float m_Yaw = 0;
    float m_Pitch = 0.12f;
    float m_Distance = 3;
    bool m_Open = true;
    bool m_Focused = false;
    bool m_FocusRequested = true;
    bool m_Orbiting = false;
    ColorBuffer m_Color;
    ColorBuffer m_Display;
    ColorBuffer m_AmbientOcclusion{Color(1.0f, 1.0f, 1.0f, 1.0f)};
    DepthBuffer m_Depth;
    DepthBuffer m_Unshadowed;
    DescriptorHandle m_Textures;
    uint64_t m_Image = 0;
    RootSignature m_PresentRoot;
    GraphicsPSO m_PresentPSO;
};
