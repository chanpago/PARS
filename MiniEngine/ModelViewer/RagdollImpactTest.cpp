#include "RagdollImpactTest.h"
#include "GameFramework/Character.h"
#include "GameFramework/World.h"
#include "imgui.h"
#pragma warning(push)
#pragma warning(disable : 4100)
#include "json.hpp"
#pragma warning(pop)
#include <algorithm>
#include <cmath>
#include <fstream>
#include <stdexcept>
#include <Windows.h>

using namespace Math;
using namespace PARS;
using namespace physx;

namespace
{
    bool ValidProjectileSettings(float mass, float speed)
    {
        return std::isfinite(mass) && std::isfinite(speed) && mass >= .1f && mass <= 100 && speed >= 1 && speed <= 100;
    }

    std::wstring ProjectileSettingsPath(const std::string& path)
    {
        const int count = MultiByteToWideChar(CP_UTF8, MB_ERR_INVALID_CHARS, path.c_str(), -1, nullptr, 0);
        if (count <= 1)
        {
            throw std::runtime_error(u8"구 설정 파일 경로가 올바르지 않습니다.");
        }
        std::wstring result(count, L'\0');
        MultiByteToWideChar(CP_UTF8, MB_ERR_INVALID_CHARS, path.c_str(), -1, &result[0], count);
        result.resize(count - 1);
        return result;
    }
}

RagdollImpactTest::~RagdollImpactTest()
{
    ClearProjectiles();
    if (PARS::Character* dummy = GetDummy())
    {
        dummy->Destroy();
    }
}

Character* RagdollImpactTest::GetDummy() const
{
    return m_World.FindActor<Character>(m_Dummy);
}

bool RagdollImpactTest::Initialize(std::shared_ptr<const Model> idle, const PhysicsAsset& asset)
{
    m_Asset = asset;
    if (std::none_of(m_Asset.constraints.begin(), m_Asset.constraints.end(), HasEnabledAngularDrive))
    {
        ConfigureDefaultDrives(m_Asset);
    }
    PARS::Character* dummy = m_World.SpawnActor<Character>("IdleImpactDummy");
    if (!dummy)
    {
        return false;
    }
    m_Dummy = dummy->GetHandle();
    dummy->GetCharacterMovement().SetTickEnabled(false);
    dummy->GetCapsuleComponent().SetPhysicsEnabled(false);
    PARS::SkeletalMeshComponent& mesh = dummy->GetMesh();
    if (!mesh.SetSkeletalMesh(idle))
    {
        m_Message = u8"더미 메시를 초기화하지 못했습니다.";
        return false;
    }
    // 이동 상태나 자동 클립 전환 없이 Idle 하나만 반복한다.
    mesh.SetAnimInstance(std::make_unique<AnimInstance>(mesh));
    if (!mesh.GetAnimInstance().AddAnimation("Idle", idle) || !mesh.GetAnimInstance().PlayAnimation(0) ||
        !mesh.SetPhysicsAsset(m_Asset, m_Message))
    {
        return false;
    }
    m_World.InitializePendingActors();
    return Reset();
}

bool RagdollImpactTest::Reset()
{
    PARS::Character* dummy = GetDummy();
    if (!dummy)
    {
        return false;
    }
    ClearProjectiles();
    if (!dummy->GetMesh().SetAutomaticRootSupport(false, m_Message))
    {
        return false;
    }
    dummy->GetMesh().SetAutomaticRagdoll(false);
    if (!dummy->StopRagdoll(m_Message))
    {
        return false;
    }
    dummy->SetActorLocation(Vector3(3.5f, dummy->GetCapsuleComponent().GetHalfHeight(), -2.5f));
    PARS::SkeletalMeshComponent& mesh = dummy->GetMesh();
    mesh.GetAnimInstance().PlayAnimation(0);
    if (!mesh.SetPhysicsAsset(m_Asset, m_Message))
    {
        return false;
    }
    // 캡슐 중심 대신 PA의 실제 최하단을 바닥에 맞춘다.
    const float lowest = mesh.GetRagdoll().GetLowestPoint();
    dummy->SetActorLocation(dummy->GetActorLocation() - Vector3(0, lowest, 0));
    if (!dummy->SetPhysicsMode(PhysicsAnimationMode::ActiveRagdoll, m_Message) ||
        !mesh.GetRagdoll().SetRootSupported(true, m_Message))
    {
        return false;
    }
    if (!mesh.SetAutomaticRootSupport(m_AutomaticSupport, m_Message))
    {
        return false;
    }
    mesh.SetAutomaticRagdoll(m_AutomaticSupport);
    if (m_AutomaticSupport && !dummy->StopRagdoll(m_Message))
    {
        return false;
    }
    m_HipsReference = mesh.GetRagdoll().GetPosition();
    m_MaxHipsDisplacement = 0;
    // 눈금 0은 바닥 Y=0이다. 더미가 밀려도 눈금자는 초기 위치 옆에 고정한다.
    const Vector3 dummyPosition = dummy->GetActorLocation();
    m_RulerOrigin = Vector3(float(dummyPosition.GetX()) - .75f, 0, float(dummyPosition.GetZ()));
    m_Message.clear();
    return true;
}

bool RagdollImpactTest::SetAutomaticSupport(bool enabled)
{
    m_AutomaticSupport = enabled;
    return Reset();
}

bool RagdollImpactTest::SetProjectileSettings(float mass, float speed)
{
    if (!ValidProjectileSettings(mass, speed))
    {
        return false;
    }
    m_Mass = mass;
    m_Speed = speed;
    return true;
}

bool RagdollImpactTest::LoadProjectileSettings(const std::string& path)
{
    m_ProjectileSettingsPath = path;
    try
    {
        std::ifstream file(ProjectileSettingsPath(path));
        if (!file)
        {
            throw std::runtime_error(u8"구 설정 파일을 열 수 없습니다. 현재 설정을 유지합니다.");
        }
        nlohmann::json data;
        file >> data;
        if (data.at("version") != 1)
        {
            throw std::runtime_error(u8"지원하지 않는 구 설정 파일 버전입니다.");
        }
        const float mass = data.at("massKg").get<float>();
        const float speed = data.at("speedMetersPerSecond").get<float>();
        // 두 값을 모두 검증한 뒤 적용해 불러오기 실패 시 현재 설정을 유지한다.
        if (!SetProjectileSettings(mass, speed))
        {
            throw std::runtime_error(u8"구 질량은 0.1~100kg, 발사 속도는 1~100m/s여야 합니다.");
        }
        m_ProjectileSettingsMessage = u8"저장된 구 설정을 불러왔습니다.";
        return true;
    }
    catch (const std::exception& exception)
    {
        m_ProjectileSettingsMessage = u8"구 설정 불러오기 실패: " + std::string(exception.what());
        return false;
    }
}

bool RagdollImpactTest::SaveProjectileSettings()
{
    try
    {
        if (!ValidProjectileSettings(m_Mass, m_Speed))
        {
            throw std::runtime_error(u8"구 질량은 0.1~100kg, 발사 속도는 1~100m/s여야 합니다.");
        }
        const nlohmann::json data = {{"version", 1}, {"massKg", m_Mass}, {"speedMetersPerSecond", m_Speed}};
        std::ofstream file(ProjectileSettingsPath(m_ProjectileSettingsPath), std::ios::trunc);
        file << data.dump(4) << '\n';
        file.flush();
        if (!file)
        {
            throw std::runtime_error(u8"구 설정 파일에 저장할 수 없습니다.");
        }
        m_ProjectileSettingsMessage = u8"구 설정을 저장했습니다. 다음 실행부터 자동으로 불러옵니다.";
        return true;
    }
    catch (const std::exception& exception)
    {
        m_ProjectileSettingsMessage = u8"구 설정 저장 실패: " + std::string(exception.what());
        return false;
    }
}

bool RagdollImpactTest::FireFrom(const Character& player)
{
    PARS::Character* dummy = GetDummy();
    return dummy && FireAt(player, dummy->GetMesh().GetRagdoll().GetPosition());
}

bool RagdollImpactTest::FireAimed(const Character& player, Vector3 cameraPosition, Vector3 cameraDirection)
{
    PhysicsWorld* physics = m_World.GetPhysicsWorld();
    if (!physics)
    {
        return false;
    }
    physx::PxVec3 (*const toPx)(Math::Vector3) = [](Vector3 v) -> physx::PxVec3
    {
        return PxVec3(float(v.GetX()), float(v.GetY()), float(v.GetZ()));
    };
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
    PARS::Character* dummy = GetDummy();
    PhysicsWorld* physics = m_World.GetPhysicsWorld();
    if (!dummy || !physics || !dummy->GetMesh().GetRagdoll().IsActive())
    {
        return false;
    }
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
    physx::PxVec3 (*const toPx)(Math::Vector3) = [](Vector3 v) -> physx::PxVec3
    {
        return PxVec3(float(v.GetX()), float(v.GetY()), float(v.GetZ()));
    };
    // 카메라에서 보이는 표적이라도 발사 위치 앞이 막혀 있으면 장애물 너머에 구를 생성하지 않는다.
    if (checkMuzzle && physics->SweepSphereDistance(toPx(origin), toPx(muzzle - origin), SphereRadius, &player) < clearance - .001f)
    {
        m_Message = u8"발사 위치가 막혀 있습니다. 장애물에서 떨어져 발사하세요.";
        return false;
    }
    physx::PxRigidDynamic* sphere = physics->CreateDynamicBody(PxTransform(toPx(muzzle)), PxSphereGeometry(SphereRadius),
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
    for (const RagdollImpactTest::Shot& shot : m_Shots)
    {
        shot.body->release();
    }
    m_Shots.clear();
}

std::vector<PxRigidDynamic*> RagdollImpactTest::GetProjectiles() const
{
    std::vector<PxRigidDynamic*> bodies;
    for (const RagdollImpactTest::Shot& shot : m_Shots)
    {
        bodies.push_back(shot.body);
    }
    return bodies;
}

void RagdollImpactTest::Update()
{
    const uint64_t step = m_World.GetPhysicsWorld()->GetStepCount();
    m_Shots.erase(std::remove_if(m_Shots.begin(), m_Shots.end(), [&](const Shot& shot) -> bool
                      {
        if (step < shot.expiresAtStep)
        {
            return false;
        }
        shot.body->release();
        return true;
    }),
        m_Shots.end());
    if (PARS::Character* dummy = GetDummy())
    {
        m_MaxHipsDisplacement = Max(m_MaxHipsDisplacement,
            float(Length(dummy->GetMesh().GetRagdoll().GetPosition() - m_HipsReference)));
    }
}

void RagdollImpactTest::BuildLines(std::vector<PxDebugLine>& lines)
{
    BuildBodyImpactLines(lines);
    if (PARS::Character* dummy = GetDummy())
    {
        m_FootSoles.BuildLines(dummy->GetMesh(), lines);
    }
}

void RagdollImpactTest::DrawUI(Character* player)
{
    ImGui::SetNextWindowPos(ImVec2(450, 70), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(440, 760), ImGuiCond_FirstUseEver);
    if (ImGui::Begin(u8"PARS - 더미 충격 테스트"))
    {
        DrawBodyImpactUI(player);
        ImGui::TextWrapped("%s", u8"우클릭 유지: 조준 / 조준 중 좌클릭 또는 R: 발사");
        ImGui::TextWrapped("%s", u8"조준하지 않을 때 R: 더미 골반으로 발사");
        ImGui::TextUnformatted(u8"반지름 15cm / 중력과 실제 충돌 적용");
        ImGui::Checkbox(u8"2m 눈금자 표시", &m_RulerVisible);
        ImGui::TextUnformatted(u8"눈금자: 1cm 눈금 / 10cm 숫자 / 주황색 8cm 표시");
        ImGui::DragFloat(u8"구 질량", &m_Mass, .5f, .1f, 100, "%.1f kg", ImGuiSliderFlags_AlwaysClamp);
        ImGui::DragFloat(u8"발사 속도", &m_Speed, .5f, 1, 100, "%.1f m/s", ImGuiSliderFlags_AlwaysClamp);
        ImGui::BeginDisabled(m_ProjectileSettingsPath.empty());
        if (ImGui::Button(u8"구 설정 저장"))
        {
            SaveProjectileSettings();
        }
        ImGui::SameLine();
        if (ImGui::Button(u8"구 설정 불러오기"))
        {
            LoadProjectileSettings(m_ProjectileSettingsPath);
        }
        ImGui::EndDisabled();
        ImGui::TextWrapped("%s", u8"구 질량과 발사 속도를 저장하면 다음 실행 시 자동으로 적용합니다.");
        if (!m_ProjectileSettingsMessage.empty())
        {
            ImGui::TextWrapped("%s", m_ProjectileSettingsMessage.c_str());
        }
        ImGui::BeginDisabled(!player);
        if (ImGui::Button(u8"더미로 구 발사") && player)
        {
            FireFrom(*player);
        }
        ImGui::EndDisabled();
        ImGui::SameLine();
        if (ImGui::Button(u8"더미 초기화"))
        {
            Reset();
        }
        if (ImGui::Button(u8"발사한 구 제거"))
        {
            ClearProjectiles();
        }
        ImGui::SameLine();
        ImGui::Text(u8"남은 구: %zu개", m_Shots.size());
        if (PARS::Character* dummy = GetDummy())
        {
            PARS::Ragdoll& ragdoll = dummy->GetMesh().GetRagdoll();
            bool automatic = m_AutomaticSupport;
            if (ImGui::Checkbox(u8"COM 자동 골반 지지", &automatic))
            {
                SetAutomaticSupport(automatic);
            }
            ImGui::TextUnformatted(u8"모드 변경 시 더미를 초기화합니다.");
            SkeletalMeshComponent& mesh = dummy->GetMesh();
            ImGui::Text(u8"물리 모드: %s", mesh.IsActiveRagdoll() ? u8"액티브 래그돌" :
                mesh.IsRagdoll() ? u8"패시브 래그돌" : mesh.IsBlendingToKinematic() ? u8"키네마틱 복귀 보간" : u8"키네마틱");
            float stableDuration = mesh.GetKinematicStableDuration();
            if (ImGui::DragFloat(u8"A영역 복귀 대기 시간", &stableDuration, .1f, .1f, 10.0f, "%.1f s", ImGuiSliderFlags_AlwaysClamp))
            {
                mesh.SetKinematicStableDuration(stableDuration);
            }
            ImGui::Text(u8"A영역 연속 유지: %.2f / %.2f s", mesh.GetKinematicStableTime(), stableDuration);
            bool supported = ragdoll.IsRootSupported();
            ImGui::BeginDisabled(m_AutomaticSupport);
            if (ImGui::Checkbox(u8"더미 골반 지지", &supported))
            {
                ragdoll.SetRootSupported(supported, m_Message);
            }
            ImGui::EndDisabled();
            ImGui::Text(u8"지지 축: X %s / Y %s / Z %s / 회전 %s",
                ragdoll.IsRootAxisSupported(0) ? "ON" : "OFF", ragdoll.IsRootAxisSupported(1) ? "ON" : "OFF",
                ragdoll.IsRootAxisSupported(2) ? "ON" : "OFF", ragdoll.IsRootAxisSupported(3) ? "ON" : "OFF");
            const Math::Vector3 hips = ragdoll.GetPosition();
            ImGui::Text(u8"실제 Hips: (%.3f, %.3f, %.3f) m", float(hips.GetX()), float(hips.GetY()), float(hips.GetZ()));
            ImGui::Text(u8"기준 대비 골반 이동: %.2f cm / 최대 %.2f cm",
                float(Length(hips - m_HipsReference)) * 100, m_MaxHipsDisplacement * 100);
            if (supported)
            {
                ImGui::Text(u8"목표 Y %.3f m / 지지 위치 오차 %.2f cm",
                    ragdoll.GetRootTargetHeight(), ragdoll.GetRootPositionError() * 100);
            }
            ImGui::TextUnformatted(u8"기준은 초기화 또는 마지막 발사 시점입니다.");
            ImGui::PushID("ImpactDummy");
            m_FootSoles.DrawUI(dummy->GetMesh());
            ImGui::PopID();
        }
        if (!m_Message.empty())
        {
            ImGui::TextWrapped("%s", m_Message.c_str());
        }
    }
    ImGui::End();
}
