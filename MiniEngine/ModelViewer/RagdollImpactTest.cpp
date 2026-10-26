#include "RagdollImpactTest.h"
#include "GameFramework/Character.h"
#include "GameFramework/World.h"
#include "imgui.h"
#include <algorithm>
#include <cmath>

using namespace Math;
using namespace PARS;
using namespace physx;

RagdollImpactTest::~RagdollImpactTest()
{
    ClearProjectiles();
    if (auto* dummy = GetDummy()) dummy->Destroy();
}

Character* RagdollImpactTest::GetDummy() const
{
    return m_World.FindActor<Character>(m_Dummy);
}

bool RagdollImpactTest::Initialize(std::shared_ptr<const Model> idle, const PhysicsAsset& asset)
{
    m_Asset = asset;
    if (std::none_of(m_Asset.constraints.begin(), m_Asset.constraints.end(), HasEnabledAngularDrive))
        ConfigureDefaultDrives(m_Asset);
    auto* dummy = m_World.SpawnActor<Character>("IdleImpactDummy");
    if (!dummy) return false;
    m_Dummy = dummy->GetHandle();
    dummy->GetCharacterMovement().SetTickEnabled(false);
    dummy->GetCapsuleComponent().SetPhysicsEnabled(false);
    auto& mesh = dummy->GetMesh();
    if (!mesh.SetSkeletalMesh(idle))
    {
        m_Message = u8"더미 메시를 초기화하지 못했습니다.";
        return false;
    }
    // 이동 상태나 자동 클립 전환 없이 Idle 하나만 반복한다.
    mesh.SetAnimInstance(std::make_unique<AnimInstance>(mesh));
    if (!mesh.GetAnimInstance().AddAnimation("Idle", idle) || !mesh.GetAnimInstance().PlayAnimation(0) ||
        !mesh.SetPhysicsAsset(m_Asset, m_Message))
        return false;
    m_World.InitializePendingActors();
    return Reset();
}

bool RagdollImpactTest::Reset()
{
    auto* dummy = GetDummy();
    if (!dummy) return false;
    ClearProjectiles();
    if (!dummy->StopRagdoll(m_Message)) return false;
    dummy->SetActorLocation(Vector3(3.5f, dummy->GetCapsuleComponent().GetHalfHeight(), -2.5f));
    auto& mesh = dummy->GetMesh();
    mesh.GetAnimInstance().PlayAnimation(0);
    if (!mesh.SetPhysicsAsset(m_Asset, m_Message)) return false;
    // 캡슐 중심 대신 PA의 실제 최하단을 바닥에 맞춘다.
    const float lowest = mesh.GetRagdoll().GetLowestPoint();
    dummy->SetActorLocation(dummy->GetActorLocation() - Vector3(0, lowest, 0));
    if (!dummy->SetPhysicsMode(PhysicsAnimationMode::ActiveRagdoll, m_Message) ||
        !mesh.GetRagdoll().SetRootSupported(true, m_Message))
        return false;
    m_HipsReference = mesh.GetRagdoll().GetPosition();
    m_MaxHipsDisplacement = 0;
    m_Message.clear();
    return true;
}

bool RagdollImpactTest::FireFrom(const Character& player)
{
    auto* dummy = GetDummy();
    auto* physics = m_World.GetPhysicsWorld();
    if (!dummy || !physics || !dummy->GetMesh().GetRagdoll().IsActive()) return false;
    const Vector3 target = dummy->GetMesh().GetRagdoll().GetPosition();
    const Vector3 origin = player.GetActorLocation() + Vector3(0, .15f, 0);
    const Vector3 delta = target - origin;
    const float clearance = player.GetCapsuleComponent().GetRadius() + SphereRadius + .15f;
    if (float(Length(delta)) < clearance + SphereRadius)
    {
        m_Message = u8"더미에서 조금 더 떨어져 발사하세요.";
        return false;
    }
    const Vector3 direction = Normalize(delta);
    const Vector3 muzzle = origin + direction * clearance;
    const auto toPx = [](Vector3 v) { return PxVec3(float(v.GetX()), float(v.GetY()), float(v.GetZ())); };
    auto* sphere = physics->CreateDynamicBody(PxTransform(toPx(muzzle)), PxSphereGeometry(SphereRadius),
        m_Mass, "Impact sphere");
    if (!sphere)
    {
        m_Message = u8"발사 구를 생성하지 못했습니다.";
        return false;
    }
    // 생성할 때만 속도를 지정한다. 이후 궤적과 충격 전달은 PhysX 충돌로 처리한다.
    sphere->setRigidBodyFlag(PxRigidBodyFlag::eENABLE_SPECULATIVE_CCD, true);
    sphere->setLinearVelocity(toPx(direction * m_Speed));
    if (m_Shots.size() >= 12)
    {
        m_Shots.front().body->release();
        m_Shots.erase(m_Shots.begin());
    }
    m_Shots.push_back({sphere, physics->GetStepCount() + 600});
    m_HipsReference = target;
    m_MaxHipsDisplacement = 0;
    m_Message.clear();
    return true;
}

void RagdollImpactTest::ClearProjectiles()
{
    for (const auto& shot : m_Shots) shot.body->release();
    m_Shots.clear();
}

std::vector<PxRigidDynamic*> RagdollImpactTest::GetProjectiles() const
{
    std::vector<PxRigidDynamic*> bodies;
    for (const auto& shot : m_Shots) bodies.push_back(shot.body);
    return bodies;
}

void RagdollImpactTest::Update()
{
    const auto step = m_World.GetPhysicsWorld()->GetStepCount();
    m_Shots.erase(std::remove_if(m_Shots.begin(), m_Shots.end(), [&](const Shot& shot)
    {
        if (step < shot.expiresAtStep) return false;
        shot.body->release();
        return true;
    }), m_Shots.end());
    if (auto* dummy = GetDummy())
        m_MaxHipsDisplacement = Max(m_MaxHipsDisplacement,
            float(Length(dummy->GetMesh().GetRagdoll().GetPosition() - m_HipsReference)));
}

void RagdollImpactTest::BuildLines(std::vector<PxDebugLine>& lines)
{
    if (auto* dummy = GetDummy())
        m_FootSoles.BuildLines(dummy->GetMesh(), *m_World.GetPhysicsWorld(), lines);
}

void RagdollImpactTest::DrawUI(Character* player)
{
    ImGui::SetNextWindowPos(ImVec2(450, 70), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(420, 570), ImGuiCond_FirstUseEver);
    if (ImGui::Begin(u8"PARS - 더미 충격 테스트"))
    {
        ImGui::TextUnformatted(u8"R: 플레이어 앞에서 더미 골반 방향으로 구 발사");
        ImGui::TextUnformatted(u8"반지름 30cm / 중력과 실제 충돌 적용");
        ImGui::DragFloat(u8"구 질량", &m_Mass, .5f, .1f, 100, "%.1f kg", ImGuiSliderFlags_AlwaysClamp);
        ImGui::DragFloat(u8"발사 속도", &m_Speed, .5f, 1, 50, "%.1f m/s", ImGuiSliderFlags_AlwaysClamp);
        ImGui::BeginDisabled(!player);
        if (ImGui::Button(u8"구 발사 (R)") && player) FireFrom(*player);
        ImGui::EndDisabled();
        ImGui::SameLine();
        if (ImGui::Button(u8"더미 초기화")) Reset();
        if (ImGui::Button(u8"발사한 구 제거")) ClearProjectiles();
        ImGui::SameLine();
        ImGui::Text(u8"남은 구: %zu개", m_Shots.size());
        if (auto* dummy = GetDummy())
        {
            auto& ragdoll = dummy->GetMesh().GetRagdoll();
            bool supported = ragdoll.IsRootSupported();
            if (ImGui::Checkbox(u8"더미 골반 지지", &supported)) ragdoll.SetRootSupported(supported, m_Message);
            const auto hips = ragdoll.GetPosition();
            ImGui::Text(u8"실제 Hips: (%.3f, %.3f, %.3f) m", float(hips.GetX()), float(hips.GetY()), float(hips.GetZ()));
            ImGui::Text(u8"기준 대비 골반 이동: %.2f cm / 최대 %.2f cm",
                float(Length(hips - m_HipsReference)) * 100, m_MaxHipsDisplacement * 100);
            if (supported) ImGui::Text(u8"골반 목표 위치 오차: %.2f cm", ragdoll.GetRootPositionError() * 100);
            ImGui::TextUnformatted(u8"기준은 초기화 또는 마지막 발사 시점입니다.");
            ImGui::PushID("ImpactDummy");
            m_FootSoles.DrawUI(dummy->GetMesh());
            ImGui::PopID();
        }
        if (!m_Message.empty()) ImGui::TextWrapped("%s", m_Message.c_str());
    }
    ImGui::End();
}
