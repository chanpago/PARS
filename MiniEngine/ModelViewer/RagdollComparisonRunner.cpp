#include "RagdollComparisonRunner.h"
#include "Components/SkeletalMeshComponent.h"
#include "PhysicsWorld.h"
#include "GraphicsCore.h"
#include "CommandListManager.h"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <sstream>

using namespace Math;
using namespace PARS;

namespace
{
    struct SequenceStage
    {
        double endTime, speed;
    };
    constexpr SequenceStage Sequence[] = {{2, 0}, {6, 1.6}, {10, 3}};
    constexpr size_t SequenceCount = sizeof(Sequence) / sizeof(Sequence[0]);
    constexpr float BlendDuration = .2f;
}

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
    auto playback = settings;
    if (playback.simulation == Simulation::A || playback.simulation == Simulation::B)
    {
        if (m_Models.size() < (playback.simulation == Simulation::A ? SequenceCount : 2))
        {
            m_Message = playback.simulation == Simulation::B ? u8"시뮬레이션 B에는 Idle, Walking 클립이 필요합니다."
                                                             : u8"시뮬레이션 A에는 Idle, Walking, Slow Run 클립이 필요합니다.";
            return false;
        }
        playback.clip = 0;
        if (playback.simulation == Simulation::A)
        {
            playback.duration = static_cast<float>(Sequence[SequenceCount - 1].endTime);
        }
        playback.speed = 0;
    }
    if (playback.simulation == Simulation::B && (!std::isfinite(playback.uTurnRadius) || playback.uTurnRadius < .5f || playback.uTurnRadius > 3 || playback.duration < 3))
    {
        m_Message = u8"시뮬레이션 B는 측정 시간 3~60초, 유턴 반경 0.5~3m를 사용합니다.";
        return false;
    }
    if (!asset.Validate(error) || asset.bodies.empty() || playback.clip < 0 ||
        size_t(playback.clip) >= m_Models.size() || !std::isfinite(playback.duration) ||
        playback.duration < 1 || playback.duration > 60 || !std::isfinite(playback.speed) ||
        playback.speed < 0 || playback.speed > 3)
    {
        m_Message = error.empty() ? u8"비교 설정이 올바르지 않습니다 (시간 1~60초, 속도 0~3 m/s)." : error;
        return false;
    }
    Reset();
    if (m_Settings.simulation != playback.simulation)
    {
        strcpy_s(m_ReportPath, playback.simulation == Simulation::B ? "ragdoll-comparison-b.html" : playback.simulation == Simulation::A ? "ragdoll-comparison-a.html"
                                                                                                                                         : "ragdoll-comparison.html");
    }
    m_Settings = playback;
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
    const float spacing = playback.simulation == Simulation::B ? playback.uTurnRadius + 1.5f : 1.5f;
    m_Placements[0] = Vector3(-spacing, height, 0);
    m_Placements[1] = Vector3(spacing, height, 0);
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
        mesh.GetAnimInstance().PlayAnimation(playback.clip);
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
    m_TargetSteps = static_cast<size_t>(std::round(playback.duration / PhysicsWorld::FixedDeltaTime));
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
    const auto& reference = m_Meshes[0]->GetAnimInstance();
    const auto& actual = m_Meshes[1]->GetAnimInstance();
    const float time = reference.GetAnimationTime();
    if (std::abs(time - actual.GetAnimationTime()) > 1e-6f ||
        reference.GetCurrentAnimationIndex() != actual.GetCurrentAnimationIndex() ||
        std::abs(reference.GetBlendAlpha() - actual.GetBlendAlpha()) > 1e-6f ||
        !m_Meshes[1]->IsActiveRagdoll())
    {
        Fail(u8"두 애니메이션의 클립·시간·블렌딩이 달라졌거나 액티브 제어가 중단되었습니다.");
        return false;
    }
    std::string error;
    if (!m_Results.Sample(poses[0], poses[1], time, error, baseline,
            reference.GetAnimationName(reference.GetCurrentAnimationIndex()), reference.GetBlendAlpha()))
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
    const int clip = GetTargetClip();
    const auto path = GetPathTransform((m_Steps + 1) * PhysicsWorld::FixedDeltaTime);
    for (int side = 0; side < 2; ++side)
    {
        auto& animation = m_Meshes[side]->GetAnimInstance();
        if (animation.GetCurrentAnimationIndex() != uint32_t(clip) &&
            !animation.TransitionToAnimation(clip, BlendDuration))
        {
            Fail(u8"다음 동작으로 블렌딩하지 못했습니다.");
            return false;
        }
        auto placement = path;
        placement.SetTranslation(m_Placements[side] + path.GetTranslation());
        m_Meshes[side]->GetOwner().SetActorTransform(placement);
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

int RagdollComparisonRunner::GetTargetClip() const
{
    if (m_Settings.simulation == Simulation::SingleClip)
    {
        return m_Settings.clip;
    }
    if (m_Settings.simulation == Simulation::B)
    {
        return m_Steps >= static_cast<size_t>(std::round(Sequence[0].endTime / PhysicsWorld::FixedDeltaTime)) ? 1 : 0;
    }
    size_t clip = 0;
    while (clip + 1 < SequenceCount &&
        m_Steps >= static_cast<size_t>(std::round(Sequence[clip].endTime / PhysicsWorld::FixedDeltaTime)))
    {
        ++clip;
    }
    return static_cast<int>(clip);
}

float RagdollComparisonRunner::GetTravelDistance(double time) const
{
    if (m_Settings.simulation == Simulation::SingleClip)
    {
        return m_Settings.speed * static_cast<float>(time);
    }
    if (m_Settings.simulation == Simulation::B)
    {
        const double elapsed = std::max(0.0, time - Sequence[0].endTime);
        const double blend = std::min(elapsed, double(BlendDuration));
        return static_cast<float>(Sequence[1].speed * (blend * blend / (2 * BlendDuration) + elapsed - blend));
    }
    // 자세 블렌딩과 같은 시간 동안 경로 속도도 바꾼다. 구간별 속도를 적분해
    // 전환 시 위치가 뛰지 않으며, 일시정지나 렌더 FPS가 경로에 영향을 주지 않는다.
    double distance = 0, start = 0, previousSpeed = 0;
    for (const auto& stage : Sequence)
    {
        const double elapsed = std::max(0.0, std::min(time - start, stage.endTime - start));
        const double blend = std::min(elapsed, double(BlendDuration));
        distance += previousSpeed * blend + (stage.speed - previousSpeed) * blend * blend / (2 * BlendDuration) +
            stage.speed * (elapsed - blend);
        start = stage.endTime;
        previousSpeed = stage.speed;
    }
    return static_cast<float>(distance);
}

AffineTransform RagdollComparisonRunner::GetPathTransform(double time) const
{
    float distance = GetTravelDistance(time);
    if (m_Settings.simulation != Simulation::B)
    {
        return AffineTransform(Matrix3(kIdentity), Vector3(0, 0, distance));
    }
    // 3m 직선 두 개를 반원 두 개로 연결한다. 위치와 접선 방향이 경계에서 연속이며,
    // 바디를 순간이동시키지 않고 애니메이션 목표의 월드 변환만 갱신한다.
    constexpr float straight = 3;
    const float radius = m_Settings.uTurnRadius;
    const float arc = DirectX::XM_PI * radius;
    distance = std::fmod(distance, 2 * (straight + arc));
    float x = 0, z = 0, yaw = 0;
    if (distance < straight)
    {
        z = distance;
    }
    else if (distance < straight + arc)
    {
        yaw = (distance - straight) / radius;
        x = radius * (1 - std::cos(yaw));
        z = straight + radius * std::sin(yaw);
    }
    else if (distance < 2 * straight + arc)
    {
        yaw = DirectX::XM_PI;
        x = 2 * radius;
        z = straight - (distance - straight - arc);
    }
    else
    {
        const float angle = (distance - 2 * straight - arc) / radius;
        yaw = DirectX::XM_PI + angle;
        x = radius * (1 + std::cos(angle));
        z = -radius * std::sin(angle);
    }
    return AffineTransform(Matrix3::MakeYRotation(yaw), Vector3(x, 0, z));
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

void RagdollComparisonRunner::RequestLocomotionComparison(const PhysicsAsset* editor, const PhysicsAsset* character,
    Simulation simulation)
{
    Open();
    if (m_StartRequested || ((m_State == State::Running || m_State == State::Paused) && m_Settings.simulation == simulation))
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
    m_RequestedSettings = Settings(simulation);
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
    return GetPathTransform(m_Steps * PhysicsWorld::FixedDeltaTime).GetTranslation() +
        Vector3(0, (m_Settings.suspended ? 1.0f : 0.0f) + 1.0f, 0);
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
        if (m_Settings.simulation == Simulation::A)
        {
            text << u8"시뮬레이션 A | 연속 재생: 대기 0–2초 → 걷기 2–6초 → 달리기 (Slow Run) 6–10초"
                 << u8" | 전환 블렌딩: " << BlendDuration << u8"초 (각 구간 시간에 포함)"
                 << u8" | +Z 이동 속도: 0 → 1.6 → 3 m/s (블렌딩 중 선형 변경)";
        }
        else if (m_Settings.simulation == Simulation::B)
        {
            text << u8"시뮬레이션 B | 대기 2초 → 걷기 (Walking) | 직선 3m → 반원 유턴 반복"
                 << u8" | 유턴 반경: " << m_Settings.uTurnRadius << u8"m | 걷기 속도: 1.6 m/s"
                 << u8" | 전환 블렌딩 / 출발 가속: " << BlendDuration << u8"초 | 측정 시간: " << m_Settings.duration << u8"초";
        }
        else
        {
            text << u8"클립: " << clip << u8" | 지정 경로 +Z 이동: " << m_Settings.speed << " m/s"
                 << u8" | 클립 길이: " << (m_Meshes[0] ? m_Meshes[0]->GetAnimInstance().GetAnimationDuration(m_Settings.clip) : 0) << u8"초";
        }
        text << u8" | 물리 에셋: " << source
             << " | " << (m_Settings.suspended ? u8"공중 지지 (+1 m)" : u8"바닥 접촉")
             << u8" | 표본: " << m_Steps << "/" << m_TargetSteps
             << u8" | 지연으로 제외된 실시간: " << m_DroppedTime << u8"초"
             << u8" | 상태: " << (m_State == State::Complete ? u8"완료" : m_State == State::Failed ? u8"실패 / 부분 결과"
                                                                                                   : u8"부분 결과");
        return text.str();
    }
    if (m_Settings.simulation == Simulation::A)
    {
        text << "Simulation A | Sequence: Idle 0-2 s -> Walking 2-6 s -> Slow Run 6-10 s"
             << " | blend: " << BlendDuration << " s (included in each stage)"
             << " | +Z speed: 0 -> 1.6 -> 3 m/s (linear during blends)";
    }
    else if (m_Settings.simulation == Simulation::B)
    {
        text << "Simulation B | Idle 2 s -> Walking | repeat 3 m straight / semicircular U-turn"
             << " | turn radius: " << m_Settings.uTurnRadius << " m | walking speed: 1.6 m/s"
             << " | blend / acceleration: " << BlendDuration << " s | duration: " << m_Settings.duration << " s";
    }
    else
    {
        text << "Clip: " << m_Names[m_Settings.clip] << " | prescribed +Z travel: " << m_Settings.speed << " m/s"
             << " | clip duration: " << (m_Meshes[0] ? m_Meshes[0]->GetAnimInstance().GetAnimationDuration(m_Settings.clip) : 0) << " s";
    }
    text << " | PA: " << m_SourceLabel
         << " | " << (m_Settings.suspended ? "suspended (+1 m)" : "floor contact")
         << " | samples: " << m_Steps << "/" << m_TargetSteps
         << " | wall time discarded during stalls: " << m_DroppedTime << " s"
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
