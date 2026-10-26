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
    return dummy && FireAt(player, dummy->GetMesh().GetRagdoll().GetPosition());
}

bool RagdollImpactTest::FireAimed(const Character& player, Vector3 cameraPosition, Vector3 cameraDirection)
{
    auto* physics = m_World.GetPhysicsWorld();
    if (!physics) return false;
    const auto toPx = [](Vector3 v) { return PxVec3(float(v.GetX()), float(v.GetY()), float(v.GetZ())); };
    PxVec3 ray = toPx(cameraDirection);
    if (!toPx(cameraPosition).isFinite() || !ray.isFinite() || ray.magnitudeSquared() < 1e-8f)
    {
        m_Message = u8"조준 방향을 확인하세요.";
        return false;
    }
    ray.normalize();
    // 화면 중앙의 가장 가까운 표면을 조준한다. 자기 몸은 제외하고, 빈 공간은 1km 앞을 사용한다.
    PxRaycastHit hit;
    const PxVec3 target = physics->Raycast(toPx(cameraPosition), ray * 1000.0f, hit, &player)
        ? hit.position : toPx(cameraPosition) + ray * 1000.0f;
    const PxVec3 origin = toPx(player.GetActorLocation()) + PxVec3(0, .15f, 0);
    if ((target - origin).dot(ray) <= 0)
    {
        m_Message = u8"플레이어 앞쪽의 지점을 조준하세요.";
        return false;
    }
    return FireAt(player, Vector3(target.x, target.y, target.z), true);
}

bool RagdollImpactTest::FireAt(const Character& player, Vector3 target, bool checkMuzzle)
{
    auto* dummy = GetDummy();
    auto* physics = m_World.GetPhysicsWorld();
    if (!dummy || !physics || !dummy->GetMesh().GetRagdoll().IsActive()) return false;
    const Vector3 origin = player.GetActorLocation() + Vector3(0, .15f, 0);
    const Vector3 delta = target - origin;
    const float clearance = player.GetCapsuleComponent().GetRadius() + SphereRadius + .15f;
    if (float(Length(delta)) < clearance + SphereRadius)
    {
        m_Message = u8"목표에서 조금 더 떨어져 발사하세요.";
        return false;
    }
    const Vector3 direction = Normalize(delta);
    const Vector3 muzzle = origin + direction * clearance;
    const auto toPx = [](Vector3 v) { return PxVec3(float(v.GetX()), float(v.GetY()), float(v.GetZ())); };
    // 카메라에서 보이는 표적이라도 발사 위치 앞이 막혀 있으면 장애물 너머에 구를 생성하지 않는다.
    if (checkMuzzle && physics->SweepSphereDistance(toPx(origin), toPx(muzzle - origin), SphereRadius, &player) < clearance - .001f)
    {
        m_Message = u8"발사 위치가 막혀 있습니다. 장애물에서 떨어져 발사하세요.";
        return false;
    }
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
    m_HipsReference = dummy->GetMesh().GetRagdoll().GetPosition();
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
        ImGui::TextWrapped("%s", u8"우클릭 유지: 조준 / 조준 중 좌클릭 또는 R: 발사");
        ImGui::TextWrapped("%s", u8"조준하지 않을 때 R: 더미 골반으로 발사");
        ImGui::TextUnformatted(u8"반지름 15cm / 중력과 실제 충돌 적용");
        ImGui::DragFloat(u8"구 질량", &m_Mass, .5f, .1f, 100, "%.1f kg", ImGuiSliderFlags_AlwaysClamp);
        ImGui::DragFloat(u8"발사 속도", &m_Speed, .5f, 1, 100, "%.1f m/s", ImGuiSliderFlags_AlwaysClamp);
        ImGui::BeginDisabled(!player);
        if (ImGui::Button(u8"더미로 구 발사") && player) FireFrom(*player);
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
