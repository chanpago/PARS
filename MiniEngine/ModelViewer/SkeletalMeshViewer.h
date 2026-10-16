#pragma once

#include "GraphicsCore.h"
#include "GameFramework/World.h"
#include "Camera.h"
#include "ColorBuffer.h"
#include "DepthBuffer.h"
#include "DescriptorHeap.h"
#include "PipelineState.h"
#include "RootSignature.h"
#include "Animation.h"
#include "SkeletonOverlay.h"
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
    void BuildSkeleton();
    void DrawBoneTree(int32_t node);
    void UpdateBoneWorld();
    void DrawSkeleton();
    void SelectBone(int32_t node);
    void HandlePick(uint32_t id);
    void BeginGizmoDrag(uint32_t axis);
    void UpdateGizmoDrag(float mouseX, float mouseY);
    void CancelInteraction();
    float GetGizmoLength() const;
    Math::Vector3 GetBoneAxis(int axis) const;
    float GetDragAngle(float x, float y) const;
    void DrawPhysicsPanel();
    void CreatePhysicsBody(PARS::PhysicsShapeType shape);
    void DrawPhysicsBodies();
    void SelectBody(int index);
    bool EditingBody() const { return m_EditPhysics && m_SelectedBody >= 0; }
    Math::Matrix4 GetGizmoWorld() const;
    Math::Matrix4 GetBodyWorld(int index) const;
    int FindBodyNode(int index) const;
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
    SkeletonOverlay m_Overlay;
    std::vector<std::vector<int32_t>> m_BoneChildren;
    std::vector<int32_t> m_BoneRoots, m_BoneParents;
    std::vector<bool> m_IsJoint, m_InSkeleton;
    std::vector<Math::Matrix4> m_NodeWorld, m_NodeMeshWorld;
    std::vector<GraphNode> m_EditPose;
    std::vector<GraphNode> m_PickPose; // Pose rendered when the GPU pick was requested.
    bool m_ShowBones = true, m_RevealSelection = false;
    int32_t m_SelectedBone = -1;
    int m_GizmoMode = 0; // 0 = move, 1 = rotate, 2 = scale; local axes.
    int m_DragAxis = -1;
    GraphNode m_DragOriginal{};
    Math::Vector3 m_DragLocalDirection{Math::kZero};
    Math::Vector3 m_DragScreenAxis{Math::kZero};
    Math::Vector3 m_DragCenter{Math::kZero}, m_DragU{Math::kZero}, m_DragV{Math::kZero};
    float m_DragLength = 1, m_DragAngle = 0, m_LastDragAngle = 0;
    float m_ClickX = 0, m_ClickY = 0;
    bool m_HasPickRequest = false;
    float m_ViewportHeight = 1;
    PARS::PhysicsAsset m_PhysicsAsset;
    PARS::PhysicsBody m_DragBodyOriginal;
    bool m_EditPhysics = false, m_ShowBodies = true, m_PhysicsDirty = false, m_DragWasDirty = false;
    int m_SelectedBody = -1;
    float m_BodyOpacity = .28f;
    char m_PhysicsPath[260] = "Mixamo.physics.json";
    std::string m_PhysicsMessage;
};
