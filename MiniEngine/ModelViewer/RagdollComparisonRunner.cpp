#include "RagdollComparisonRunner.h"
#include "Components/SkeletalMeshComponent.h"
#include "PhysicsWorld.h"
#include "GraphicsCore.h"
#include "CommandListManager.h"
#include <algorithm>
#include <cmath>
#include <sstream>

using namespace Math;
using namespace PARS;

RagdollComparisonRunner::RagdollComparisonRunner(PhysicsWorld& owner,
    const std::shared_ptr<const Model>* models, const char* const* names, uint32_t count)
    : m_Owner(owner), m_Models(models, models + count)
{
    for (uint32_t i = 0; i < count; ++i)
    {
        m_Names.push_back(names[i]);
    }
}

void RagdollComparisonRunner::Reset()
{
    m_Meshes[0] = m_Meshes[1] = nullptr;
    m_World.Clear();
    if (auto* physics = m_World.GetPhysicsWorld())
    {
        physics->Shutdown();
    }
    m_Results = {};
    m_State = State::Empty;
    m_Steps = m_TargetSteps = 0;
    m_Accumulator = m_DroppedTime = 0;
    m_ShowScene = false;
    m_Message.clear();
}

void RagdollComparisonRunner::Fail(const std::string& message)
{
    m_State = State::Failed;
    m_Message = message;
}

bool RagdollComparisonRunner::Start(const PhysicsAsset& asset, const Settings& settings)
{
    std::string error;
    if (!asset.Validate(error) || asset.bodies.empty() || settings.clip < 0 ||
        size_t(settings.clip) >= m_Models.size() || !std::isfinite(settings.duration) ||
        settings.duration < 1 || settings.duration > 60 || !std::isfinite(settings.speed) ||
        settings.speed < 0 || settings.speed > 3)
    {
        m_Message = error.empty() ? u8"비교 설정이 올바르지 않습니다 (시간 1~60초, 속도 0~3 m/s)." : error;
        return false;
    }
    Reset();
    m_Settings = settings;
    m_SourceLabel = m_RequestedSource.empty() ? "Supplied PA snapshot" : m_RequestedSource;
    // Recreate the scene as well as its actors so reruns start with identical solver state.
    if (!m_World.InitializePhysics(false, &m_Owner))
    {
        Fail(u8"독립된 비교용 물리 장면을 생성하지 못했습니다.");
        return false;
    }
    if (!m_World.GetPhysicsWorld()->CreateStaticBox(physx::PxTransform(physx::PxVec3(0, -.5f, 0)),
            physx::PxVec3(250, .5f, 250), "Comparison floor"))
    {
        Fail(u8"비교용 바닥을 생성하지 못했습니다.");
        return false;
    }
    auto& physics = *m_World.GetPhysicsWorld();
    physics.ResetClock();
    m_World.SetPhysicsPaused(true);
    // 서로의 배치만 제거한다. 진행 경로와 골반의 추종 오차는 기록에 남긴다.
    const float height = settings.suspended ? 1.0f : 0.0f;
    m_Placements[0] = Vector3(-1.5f, height, 0);
    m_Placements[1] = Vector3(1.5f, height, 0);
    for (int side = 0; side < 2; ++side)
    {
        auto* actor = m_World.SpawnActor<Actor>(side == 0 ? "Kinematic reference" : "Active ragdoll");
        auto& mesh = actor->CreateDefaultSubobject<SkeletalMeshComponent>("Comparison mesh");
        actor->SetRootComponent(mesh);
        actor->SetActorLocation(m_Placements[side]);
        m_Meshes[side] = &mesh;
        if (!mesh.SetSkeletalMesh(m_Models[0]))
        {
            Fail(u8"비교할 스켈레톤을 사용할 수 없습니다.");
            return false;
        }
        for (size_t clip = 0; clip < m_Models.size(); ++clip)
        {
            if (!mesh.GetAnimInstance().AddAnimation(m_Names[clip], m_Models[clip]))
            {
                Fail(u8"비교할 애니메이션과 스켈레톤이 다릅니다.");
                return false;
            }
        }
        mesh.GetAnimInstance().PlayAnimation(settings.clip);
        if (!mesh.SetPhysicsAsset(asset, error))
        {
            Fail(error);
            return false;
        }
    }
    m_World.BeginPlay();
    m_World.Tick(0);
    for (auto* mesh : m_Meshes)
    {
        if (!mesh->GetRagdoll().IsActive() || !mesh->GetPhysicsError().empty())
        {
            Fail(u8"더미 바디를 초기화하지 못했습니다: " + mesh->GetPhysicsError());
            return false;
        }
    }
    // 기준 더미는 물리 접촉으로 실험 대상에 영향을 주지 않는다.
    m_Meshes[0]->GetRagdoll().SetCollisionEnabled(false);
    if (!m_Meshes[1]->SetPhysicsMode(PhysicsAnimationMode::ActiveRagdoll, Vector3(kZero), error) ||
        !m_Meshes[1]->GetRagdoll().SetRootSupported(true, error))
    {
        Fail(error);
        return false;
    }
    if (!m_Results.Initialize(asset, Matrix4(Matrix3(kIdentity), m_Placements[0]),
            Matrix4(Matrix3(kIdentity), m_Placements[1]), static_cast<float>(PhysicsWorld::FixedDeltaTime), error))
    {
        Fail(error);
        return false;
    }
    m_TargetSteps = static_cast<size_t>(std::round(settings.duration / PhysicsWorld::FixedDeltaTime));
    m_State = State::Running;
    if (!Capture(true))
    {
        return false;
    }
    m_ShowScene = true;
    m_FocusRequested = true;
    m_FollowLatest = true;
    m_SelectedFrame = m_SelectedBody = m_SelectedJoint = 0;
    m_Message = u8"1/60초마다 기록 중입니다. 시작 직후의 과도응답도 포함합니다.";
    return true;
}

bool RagdollComparisonRunner::Capture(bool baseline)
{
    std::vector<Matrix4> poses[2];
    const auto& asset = m_Results.GetAsset();
    for (int side = 0; side < 2; ++side)
    {
        auto& ragdoll = m_Meshes[side]->GetRagdoll();
        if (!ragdoll.IsFinite() || !m_Meshes[side]->GetPhysicsError().empty())
        {
            Fail(u8"물리 상태가 올바르지 않거나 유효하지 않은 수치가 있습니다: " + m_Meshes[side]->GetPhysicsError());
            return false;
        }
        for (size_t i = 0; i < asset.bodies.size(); ++i)
        {
            poses[side].push_back(ragdoll.GetBoneWorld(i) * asset.bodies[i].GetLocalTransform());
        }
    }
    const float time = m_Meshes[0]->GetAnimInstance().GetAnimationTime();
    if (std::abs(time - m_Meshes[1]->GetAnimInstance().GetAnimationTime()) > 1e-6f ||
        !m_Meshes[1]->IsActiveRagdoll())
    {
        Fail(u8"두 애니메이션의 시간이 달라졌거나 액티브 제어가 중단되었습니다.");
        return false;
    }
    std::string error;
    if (!m_Results.Sample(poses[0], poses[1], time, error, baseline))
    {
        Fail(error);
        return false;
    }
    return true;
}

bool RagdollComparisonRunner::Step()
{
    if (m_State != State::Running && m_State != State::Paused)
    {
        return false;
    }
    const float time = static_cast<float>((m_Steps + 1) * PhysicsWorld::FixedDeltaTime);
    for (int side = 0; side < 2; ++side)
    {
        m_Meshes[side]->GetOwner().SetActorLocation(m_Placements[side] + Vector3(0, 0, m_Settings.speed * time));
    }
    m_World.RequestPhysicsStep();
    m_World.Tick(0);
    if (!Capture(false))
    {
        return false;
    }
    ++m_Steps;
    if (m_Steps >= m_TargetSteps)
    {
        m_State = State::Complete;
        m_Message = u8"완료했습니다. 바디/관절을 선택해 시간별 측정값을 확인하거나 HTML로 저장하세요.";
    }
    return true;
}

void RagdollComparisonRunner::Advance(float deltaTime)
{
    if (m_State != State::Running || !std::isfinite(deltaTime) || deltaTime < 0)
    {
        return;
    }
    const double budget = PhysicsWorld::FixedDeltaTime * PhysicsWorld::MaxSubsteps;
    m_DroppedTime += std::max(0.0, m_Accumulator + deltaTime - budget);
    m_Accumulator = std::min(budget, m_Accumulator + deltaTime);
    while (m_Accumulator + 1e-9 >= PhysicsWorld::FixedDeltaTime && m_State == State::Running)
    {
        m_Accumulator -= PhysicsWorld::FixedDeltaTime;
        Step();
    }
}

void RagdollComparisonRunner::SetPaused(bool paused)
{
    if (m_State == State::Running || m_State == State::Paused)
    {
        m_State = paused ? State::Paused : State::Running;
        m_Accumulator = 0;
    }
}

void RagdollComparisonRunner::Stop()
{
    if (m_State == State::Running || m_State == State::Paused)
    {
        m_State = State::Stopped;
        m_Message = u8"중단했습니다. 지금까지의 기록은 유지됩니다.";
        m_Accumulator = 0;
    }
}

void RagdollComparisonRunner::RequestWalkingComparison(const PhysicsAsset* editor, const PhysicsAsset* character)
{
    Open();
    if (m_State == State::Running || m_State == State::Paused || m_StartRequested)
    {
        return;
    }
    const bool useEditor = editor && !editor->bodies.empty();
    const auto* source = useEditor ? editor : character;
    if (!source || source->bodies.empty())
    {
        m_Message = u8"비교를 시작하기 전에 물리 에셋을 적용하세요.";
        return;
    }
    m_UseEditor = useEditor;
    m_RequestedAsset = *source;
    m_RequestedSettings = Settings{};
    m_RequestedSource = useEditor ? "Editor PA (snapshot at start)" : "Character PA (snapshot at start)";
    m_ResetRequested = false;
    m_StartRequested = true;
}

void RagdollComparisonRunner::ProcessRequests()
{
    if (m_StartRequested || m_ResetRequested)
    {
        Graphics::g_CommandManager.IdleGPU();
        if (m_ResetRequested)
        {
            Reset();
        }
        else
        {
            Start(m_RequestedAsset, m_RequestedSettings);
        }
        m_StartRequested = m_ResetRequested = false;
    }
}

bool RagdollComparisonRunner::ConsumeFocusRequest()
{
    const bool requested = m_FocusRequested;
    m_FocusRequested = false;
    return requested;
}

Vector3 RagdollComparisonRunner::GetViewCenter() const
{
    return Vector3(0, (m_Settings.suspended ? 1.0f : 0.0f) + 1.0f,
        m_Settings.speed * static_cast<float>(m_Steps * PhysicsWorld::FixedDeltaTime));
}

std::string RagdollComparisonRunner::GetDescription(bool korean) const
{
    std::ostringstream text;
    if (korean)
    {
        const auto& name = m_Names[m_Settings.clip];
        const auto clip = name == "Walking" ? u8"걷기 (Walking)" : name == "Idle" ? u8"대기 (Idle)"
            : name == "Slow Run"                                                  ? u8"달리기 (Slow Run)"
                                                                                  : name.c_str();
        const auto source = m_SourceLabel == "Editor PA (snapshot at start)" ? u8"에디터 PA (시작 시 복사)"
            : m_SourceLabel == "Character PA (snapshot at start)"            ? u8"캐릭터 PA (시작 시 복사)"
                                                                             : u8"입력된 PA (시작 시 복사)";
        text << u8"클립: " << clip << u8" | 물리 에셋: " << source
             << " | " << (m_Settings.suspended ? u8"공중 지지 (+1 m)" : u8"바닥 접촉")
             << u8" | 지정 경로 +Z 이동: " << m_Settings.speed << " m/s"
             << u8" | 표본: " << m_Steps << "/" << m_TargetSteps
             << u8" | 클립 길이: " << (m_Meshes[0] ? m_Meshes[0]->GetAnimInstance().GetAnimationDuration(m_Settings.clip) : 0)
             << u8"초 | 지연으로 제외된 실시간: " << m_DroppedTime << u8"초"
             << u8" | 상태: " << (m_State == State::Complete ? u8"완료" : m_State == State::Failed ? u8"실패 / 부분 결과"
                                                                                                   : u8"부분 결과");
        return text.str();
    }
    text << "Clip: " << m_Names[m_Settings.clip] << " | PA: " << m_SourceLabel
         << " | " << (m_Settings.suspended ? "suspended (+1 m)" : "floor contact")
         << " | prescribed +Z travel: " << m_Settings.speed << " m/s"
         << " | samples: " << m_Steps << "/" << m_TargetSteps
         << " | clip duration: " << (m_Meshes[0] ? m_Meshes[0]->GetAnimInstance().GetAnimationDuration(m_Settings.clip) : 0)
         << " s | wall time discarded during stalls: " << m_DroppedTime << " s"
         << " | state: " << (m_State == State::Complete ? "complete" : m_State == State::Failed ? "failed / partial"
                                                                                                : "partial");
    return text.str();
}

bool RagdollComparisonRunner::Export(const std::string& path)
{
    if (m_Results.ExportHtml(path, GetDescription(true), m_Message))
    {
        m_Message = u8"저장 완료: " + path;
        return true;
    }
    return false;
}
