#pragma once

#include "GameFramework/World.h"
#include "RagdollComparison.h"
#include <memory>

class Model;
class PhysicsWorld;
namespace PARS
{
    class SkeletalMeshComponent;
}

// 플레이어가 없는 별도 물리 씬에서 두 더미를 같은 고정 스텝으로 진행한다.
class RagdollComparisonRunner
{
public:
    enum class State
    {
        Empty,
        Running,
        Paused,
        Complete,
        Stopped,
        Failed
    };
    enum class Simulation
    {
        A,
        B,
        SingleClip
    };
    struct Settings
    {
        explicit Settings(Simulation selected = Simulation::A)
            : simulation(selected), duration(selected == Simulation::B ? 20.0f : 6.0f)
        {
        }
        Simulation simulation;
        // 단일 클립 모드에서 클립/속도를 사용한다. B와 단일 클립의 측정 시간은 조절할 수 있다.
        int clip = 1;
        float duration, speed = 1.6f, uTurnRadius = 1;
        bool suspended = true;
    };

    RagdollComparisonRunner(PhysicsWorld& owner, const std::shared_ptr<const Model>* models,
        const char* const* names, uint32_t count);
    // 렌더러가 더미를 참조하지 않는 GPU 유휴 경계에서만 Start/Reset을 호출한다.
    bool Start(const PARS::PhysicsAsset& asset, const Settings& settings);
    void Reset();
    void Advance(float deltaTime);
    bool Step();
    void SetPaused(bool paused);
    void Stop();
    void DrawUI(const PARS::PhysicsAsset* editor, const PARS::PhysicsAsset* character);
    void RequestLocomotionComparison(const PARS::PhysicsAsset* editor, const PARS::PhysicsAsset* character,
        Simulation simulation = Simulation::A);
    // UI 변경은 다음 Update의 렌더 명령 기록 전에 처리한다.
    void ProcessRequests();
    bool ConsumeFocusRequest();
    void ReturnToPlayer();
    bool ConsumePlayerControlRequest();
    void Open()
    {
        m_Open = true;
        m_WindowFocusRequested = true;
    }
    bool IsShowingScene() const
    {
        return m_ShowScene && m_Meshes[0] && m_Meshes[1];
    }
    bool ShouldFollowCamera() const
    {
        return IsShowingScene() && m_FollowCamera;
    }
    State GetState() const
    {
        return m_State;
    }
    PARS::World& GetWorld()
    {
        return m_World;
    }
    const PARS::RagdollComparison& GetResults() const
    {
        return m_Results;
    }
    const std::string& GetStatusMessage() const
    {
        return m_Message;
    }
    Math::Vector3 GetViewCenter() const;
    std::string GetDescription(bool korean = false) const;
    bool Export(const std::string& path);

private:
    bool Capture(bool baseline);
    int GetTargetClip() const;
    float GetTravelDistance(double time) const;
    Math::AffineTransform GetPathTransform(double time) const;
    void Fail(const std::string& message);
    void DrawBodyResults();
    void DrawJointResults();
    void DrawSelectedSample();
    PhysicsWorld& m_Owner;
    PARS::World m_World;
    std::vector<std::shared_ptr<const Model>> m_Models;
    std::vector<std::string> m_Names;
    PARS::SkeletalMeshComponent* m_Meshes[2]{};
    Math::Vector3 m_Placements[2]{Math::kZero, Math::kZero};
    PARS::RagdollComparison m_Results;
    Settings m_Settings, m_RequestedSettings;
    PARS::PhysicsAsset m_RequestedAsset;
    State m_State = State::Empty;
    double m_Accumulator = 0, m_DroppedTime = 0;
    size_t m_Steps = 0, m_TargetSteps = 0;
    std::string m_Message, m_SourceLabel, m_RequestedSource;
    bool m_Open = false, m_ShowScene = false, m_FocusRequested = false;
    bool m_WindowFocusRequested = false;
    bool m_PlayerControlRequested = false;
    bool m_StartRequested = false, m_ResetRequested = false, m_UseEditor = true;
    bool m_FollowLatest = true;
    bool m_FollowCamera = true;
    int m_SelectedBody = 0, m_SelectedJoint = 0, m_SelectedFrame = 0;
    char m_ReportPath[512] = "ragdoll-comparison-a.html";
};
