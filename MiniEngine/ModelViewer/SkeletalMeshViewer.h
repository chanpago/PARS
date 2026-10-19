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
namespace PARS
{
    class SkeletalMeshComponent;
}

// 에디터 미리보기 씬이다. 게임과 불변 에셋을 공유하지만 포즈, 카메라, 렌더 타깃은
// 직접 소유하며, 물리나 컨트롤러는 연결하지 않는다.
class SkeletalMeshViewer
{
public:
    ~SkeletalMeshViewer()
    {
        Cleanup();
    }
    bool Initialize(const std::shared_ptr<const Model>* models, const char* const* names, uint32_t count);
    void Cleanup(); // GPU가 유휴 상태가 된 후, 렌더러/ImGui를 종료하기 전에 호출한다.
    void Update(float deltaTime);
    void RenderUI(GraphicsContext& context);
    void Open()
    {
        m_Open = true;
        m_FocusRequested = true;
    }
    bool WantsInput() const
    {
        return m_Open && m_Focused;
    }
    const PARS::PhysicsAsset& GetPhysicsAsset() const
    {
        return m_PhysicsAsset;
    }
    bool SetPhysicsAsset(const PARS::PhysicsAsset& asset, std::string& error);

private:
    void ResetCamera();
    void UpdateCamera(bool hovered);
    void RenderPreview(GraphicsContext& context, float aspect);
    void BuildSkeleton();
    void DrawBoneTree(int32_t node);
    void DrawNavigation();
    void DrawRelationshipGraph();
    void DrawDetails();
    void DrawOrientationAxes(float left, float top, float width, float height);
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
    bool EditingBody() const
    {
        return m_EditPhysics && !m_EditConstraints && m_SelectedBody >= 0;
    }
    bool EditingConstraint() const
    {
        return m_EditPhysics && m_EditConstraints && m_SelectedConstraint >= 0;
    }
    void DrawConstraintPanel();
    void DrawConstraints();
    static Math::Vector3 GetTwistDisplayReference(const PARS::PhysicsConstraint& joint);
    void SelectConstraint(int index);
    void CreateConstraint();
    void ResetConstraintFrames(PARS::PhysicsConstraint& joint);
    void MoveOtherConstraintFrame(const Math::Matrix4& previousWorld);
    int FindConstraintParent() const;
    Math::Matrix4 GetConstraintBodyWorld(int frame) const;
    Math::Matrix4 GetConstraintWorld(int index, int frame) const;
    Math::Matrix4 GetGizmoWorld() const;
    Math::Matrix4 GetBodyWorld(int index) const;
    int FindBodyNode(int index) const;
    PARS::World m_World;
    PARS::SkeletalMeshComponent* m_Mesh = nullptr; // 미리보기 World가 소유한다.
    Math::Camera m_Camera;
    float m_Radius = 1;
    float m_Yaw = 0;
    float m_Pitch = 0.12f;
    bool m_Open = true;
    bool m_Focused = false;
    bool m_FocusRequested = true;
    bool m_Flying = false;
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
    std::vector<GraphNode> m_PickPose; // GPU 피킹을 요청했을 때 렌더링한 포즈다.
    bool m_ShowBones = true, m_RevealSelection = false;
    bool m_PhysicsTreeOnly = true;
    float m_HierarchyFraction = .57f;
    float m_GraphZoom = 1, m_GraphPanX = 0, m_GraphPanY = 0;
    int m_GraphSelection = -1;
    int32_t m_SelectedBone = -1;
    int m_GizmoMode = 0; // 0 = 이동, 1 = 회전, 2 = 스케일이며, 로컬 축을 사용한다.
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
    PARS::PhysicsConstraint m_DragConstraintOriginal;
    bool m_EditConstraints = false, m_ShowConstraints = true, m_ShowLimits = true, m_MoveBothFrames = true;
    int m_SelectedConstraint = -1, m_ConstraintParent = -1, m_ConstraintFrame = 0;
    bool m_EditPhysics = true, m_ShowBodies = true, m_PhysicsDirty = false, m_DragWasDirty = false;
    int m_SelectedBody = -1;
    float m_BodyOpacity = .28f;
    char m_PhysicsPath[260] = "Mixamo.physics.json";
    std::string m_PhysicsMessage;
};
