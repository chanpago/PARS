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
    bool ValidProjectileSettings(float mass, float speed, float bodyImpulse, float torqueMultiplier)
    {
        return std::isfinite(mass) && std::isfinite(speed) && mass >= .1f && mass <= 100 && speed >= 1 && speed <= 100 &&
            std::isfinite(bodyImpulse) && bodyImpulse >= 0 && std::isfinite(torqueMultiplier) && torqueMultiplier >= 0;
    }

    std::wstring ImpactSettingsPath(const std::string& path)
    {
        const int count = MultiByteToWideChar(CP_UTF8, MB_ERR_INVALID_CHARS, path.c_str(), -1, nullptr, 0);
        if (count <= 1)
        {
            throw std::runtime_error(u8"설정 파일 경로가 올바르지 않습니다.");
        }
        std::wstring result(count, L'\0');
        MultiByteToWideChar(CP_UTF8, MB_ERR_INVALID_CHARS, path.c_str(), -1, &result[0], count);
        result.resize(count - 1);
        return result;
    }
}

RagdollImpactTest::~RagdollImpactTest()
{
    ClearContactCheck();
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

bool RagdollImpactTest::Initialize(std::shared_ptr<const Model> idle, const PhysicsAsset& asset, const AnimInstance& motionSource)
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
    mesh.SetFootCandidateDebugVisible(true);
    mesh.SetArmDriveStrength(1.0f);
    if (!mesh.SetSkeletalMesh(idle))
    {
        m_Message = u8"더미 메시를 초기화하지 못했습니다.";
        return false;
    }
    if (!mesh.SetAnimInstance(std::make_unique<AnimInstance>(mesh)) ||
        !mesh.GetAnimInstance().AddAnimation("Idle", idle) || !mesh.GetAnimInstance().PlayAnimation(0))
    {
        m_Message = u8"더미의 Idle 애니메이션을 등록하지 못했습니다. 파일과 본 구조를 확인하세요.";
        return false;
    }
    if (!mesh.GetAnimInstance().ShareMotionFramesFrom(motionSource, m_Message) || !mesh.SetPhysicsAsset(m_Asset, m_Message))
    {
        return false;
    }
    m_World.InitializePendingActors();
    m_SpawnTransform = dummy->GetActorTransform();
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
    // 회복 중 달라진 회전을 남기지 않고 최초 생성 당시 방향과 크기를 함께 복원한다.
    AffineTransform spawnTransform = m_SpawnTransform;
    spawnTransform.SetTranslation(m_SpawnOrigin + Vector3(0, dummy->GetCapsuleComponent().GetHalfHeight(), 0));
    if (!dummy->SetActorTransform(spawnTransform))
    {
        m_Message = u8"더미의 초기 위치와 방향을 복원하지 못했습니다.";
        return false;
    }
    PARS::SkeletalMeshComponent& mesh = dummy->GetMesh();
    mesh.GetAnimInstance().PlayAnimation(0);
    if (!mesh.SetPhysicsAsset(m_Asset, m_Message))
    {
        return false;
    }
    // 캡슐 중심 대신 PA의 실제 최하단을 시작 지점의 바닥 높이에 맞춘다.
    const float lowest = mesh.GetRagdoll().GetLowestPoint();
    dummy->SetActorLocation(dummy->GetActorLocation() - Vector3(0, lowest - float(m_SpawnOrigin.GetY()), 0));
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
    m_Message.clear();
    ClearContactCheck();
    return true;
}

void RagdollImpactTest::SetContactCheckEnabled(bool enabled)
{
    if (m_ContactCheckEnabled != enabled)
    {
        m_ContactCheckEnabled = enabled;
        m_PreviousLegContacts = 0;
    }
}

void RagdollImpactTest::ResumeSimulation()
{
    // 직전 접촉 쌍은 유지해 P 직후 같은 접촉으로 다시 멈추지 않게 한다.
    m_PausedLegContactMessage.clear();
    m_World.SetPhysicsPaused(false);
}

void RagdollImpactTest::ClearContactCheck()
{
    if (!m_PausedLegContactMessage.empty())
    {
        ResumeSimulation();
    }
    m_LegContactMessage.clear();
    m_PreviousLegContacts = 0;
}

bool RagdollImpactTest::SetAutomaticSupport(bool enabled)
{
    m_AutomaticSupport = enabled;
    return Reset();
}

bool RagdollImpactTest::SetProjectileSettings(float mass, float speed, float bodyImpulse, float torqueMultiplier)
{
    if (!ValidProjectileSettings(mass, speed, bodyImpulse, torqueMultiplier))
    {
        return false;
    }
    m_Mass = mass;
    m_Speed = speed;
    m_BodyImpulseMagnitude = bodyImpulse;
    m_BodyTorqueMultiplier = torqueMultiplier;
    return true;
}

bool RagdollImpactTest::LoadProjectileSettings(const std::string& path)
{
    m_ProjectileSettingsPath = path;
    try
    {
        std::ifstream file(ImpactSettingsPath(path));
        if (!file)
        {
            throw std::runtime_error(u8"구 설정 파일을 열 수 없습니다. 현재 설정을 유지합니다.");
        }
        nlohmann::json data;
        file >> data;
        const int version = data.at("version").get<int>();
        if (version != 1 && version != 2)
        {
            throw std::runtime_error(u8"지원하지 않는 구 설정 파일 버전입니다.");
        }
        const float mass = data.at("massKg").get<float>();
        const float speed = data.at("speedMetersPerSecond").get<float>();
        const float bodyImpulse = version == 1 ? 0.0f : data.at("bodyImpulseNs").get<float>();
        const float torqueMultiplier = version == 1 ? 0.0f : data.at("bodyTorqueMultiplier").get<float>();
        if (!SetProjectileSettings(mass, speed, bodyImpulse, torqueMultiplier))
        {
            throw std::runtime_error(u8"구 질량은 0.1~100kg, 속도는 1~100m/s, 전신 충격량과 회전 배율은 0 이상이어야 합니다.");
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
        if (!ValidProjectileSettings(m_Mass, m_Speed, m_BodyImpulseMagnitude, m_BodyTorqueMultiplier))
        {
            throw std::runtime_error(u8"구 질량은 0.1~100kg, 발사 속도는 1~100m/s여야 합니다.");
        }
        const nlohmann::json data = {{"version", 2}, {"massKg", m_Mass}, {"speedMetersPerSecond", m_Speed},
            {"bodyImpulseNs", m_BodyImpulseMagnitude}, {"bodyTorqueMultiplier", m_BodyTorqueMultiplier}};
        std::ofstream file(ImpactSettingsPath(m_ProjectileSettingsPath), std::ios::trunc);
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

bool RagdollImpactTest::LoadDummySupportSettings(const std::string& path)
{
    m_DummySupportSettingsPath = path;
    try
    {
        PARS::Character* dummy = GetDummy();
        if (!dummy)
        {
            throw std::runtime_error(u8"설정을 적용할 더미가 없습니다.");
        }
        std::ifstream file(ImpactSettingsPath(path));
        if (!file)
        {
            throw std::runtime_error(u8"더미 지지 설정 파일을 열 수 없습니다. 현재 배율을 유지합니다.");
        }
        nlohmann::json data;
        file >> data;
        if (data.at("version") != 1)
        {
            throw std::runtime_error(u8"지원하지 않는 더미 지지 설정 파일 버전입니다.");
        }
        const float scale = data.at("bufferDistanceScale").get<float>();
        // 기존 범위 검증을 통과한 값만 적용해 불러오기 실패 시 현재 배율을 유지한다.
        if (!dummy->GetMesh().GetFootSupport().SetBufferScale(scale))
        {
            throw std::runtime_error(u8"B 거리 배율은 1.01~10이어야 합니다.");
        }
        m_DummySupportSettingsMessage = u8"저장된 더미 B 배율을 불러왔습니다.";
        return true;
    }
    catch (const std::exception& exception)
    {
        m_DummySupportSettingsMessage = u8"B 배율 불러오기 실패: " + std::string(exception.what());
        return false;
    }
}

bool RagdollImpactTest::SaveDummySupportSettings()
{
    try
    {
        const PARS::Character* dummy = GetDummy();
        if (!dummy)
        {
            throw std::runtime_error(u8"설정을 저장할 더미가 없습니다.");
        }
        const float scale = dummy->GetMesh().GetFootSupport().GetBufferScale();
        const nlohmann::json data = {{"version", 1}, {"bufferDistanceScale", scale}};
        std::ofstream file(ImpactSettingsPath(m_DummySupportSettingsPath), std::ios::trunc);
        file << data.dump(4) << '\n';
        file.close();
        if (!file)
        {
            throw std::runtime_error(u8"더미 지지 설정 파일에 저장할 수 없습니다.");
        }
        m_DummySupportSettingsMessage = u8"더미 B 배율을 저장했습니다. 다음 실행부터 자동으로 불러옵니다.";
        return true;
    }
    catch (const std::exception& exception)
    {
        m_DummySupportSettingsMessage = u8"B 배율 저장 실패: " + std::string(exception.what());
        return false;
    }
}

void RagdollImpactTest::DrawDummySupportSettingsUI()
{
    ImGui::BeginDisabled(m_DummySupportSettingsPath.empty());
    if (ImGui::Button(u8"B 배율 저장"))
    {
        SaveDummySupportSettings();
    }
    ImGui::SameLine();
    if (ImGui::Button(u8"B 배율 불러오기"))
    {
        LoadDummySupportSettings(m_DummySupportSettingsPath);
    }
    ImGui::EndDisabled();
    ImGui::TextWrapped("%s", u8"저장한 B 거리 배율은 다음 실행 시 더미에 자동 적용합니다.");
    if (!m_DummySupportSettingsMessage.empty())
    {
        ImGui::TextWrapped("%s", m_DummySupportSettingsMessage.c_str());
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
    const PxVec3 eye = toPx(cameraPosition);
    PxVec3 ray = toPx(cameraDirection);
    if (!eye.isFinite() || !ray.isFinite() || ray.magnitudeSquared() < 1e-8f)
    {
        m_Message = u8"조준 방향을 확인하세요.";
        return false;
    }
    ray.normalize();
    PxGeomRaycastHit closest;
    closest.distance = 1000.0f;
    closest.position = eye + ray * closest.distance;
    closest.normal = PxVec3(0);
    PxRaycastHit sceneHit;
    if (physics->Raycast(eye, ray * closest.distance, sceneHit, &player))
    {
        closest = sceneHit;
    }
    // 서 있는 더미의 본은 이동 쿼리에서 빠지므로 조준할 때만 실제 충돌 형상을 검사한다.
    const Character* dummy = GetDummy();
    if (dummy && !dummy->GetMesh().GetRagdoll().IsSimulating())
    {
        const Ragdoll& ragdoll = dummy->GetMesh().GetRagdoll();
        for (size_t index = 0; index < ragdoll.GetBodyCount(); ++index)
        {
            PxGeometryHolder geometry;
            PxTransform inBone;
            PxTransform pose;
            PxGeomRaycastHit bodyHit;
            if (ragdoll.GetBodyCollisionShape(index, geometry, inBone, pose) &&
                PxGeometryQuery::raycast(eye, ray, geometry.any(), pose, closest.distance, PxHitFlag::eDEFAULT, 1, &bodyHit) &&
                bodyHit.distance < closest.distance)
            {
                closest = bodyHit;
            }
        }
    }
    // 공 표면이 조준점에 닿는 순간의 중심은 피격 표면에서 반지름만큼 떨어져 있다.
    const PxVec3 target = closest.position + closest.normal * SphereRadius;
    const PxVec3 origin = toPx(player.GetActorLocation()) + PxVec3(0, .15f, 0);
    if ((target - origin).dot(ray) <= 0)
    {
        m_Message = u8"플레이어 앞쪽의 지점을 조준하세요.";
        return false;
    }
    return FireAt(player, Vector3(target.x, target.y, target.z), true);
}

bool RagdollImpactTest::FireAt(const Character& player, Vector3 target, bool aimed)
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
    if (aimed && physics->SweepSphereDistance(toPx(origin), toPx(muzzle - origin), SphereRadius, &player) < clearance - .001f)
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
    // 바닥 하중이나 캐릭터의 지속 밀기와 구별해 실제 발사 구의 접촉만 피격 반응에 사용한다.
    physx::PxShape* shape = nullptr;
    if (sphere->getShapes(&shape, 1) != 1 || !physics->SetProjectileBodyImpulse(*sphere, m_BodyImpulseMagnitude, m_BodyTorqueMultiplier))
    {
        sphere->release();
        return false;
    }
    PxFilterData filter = shape->getSimulationFilterData();
    filter.word3 |= PhysicsWorld::ProjectileFilterFlag;
    if (aimed)
    {
        filter.word3 |= PhysicsWorld::AimedProjectileFilterFlag;
        sphere->setActorFlag(PxActorFlag::eDISABLE_GRAVITY, true);
        sphere->setMinCCDAdvanceCoefficient(0.0f);
    }
    shape->setSimulationFilterData(filter);
    // 조준 공은 이번 스텝에 실제로 지나가는 구간에서 최초 충돌 시점을 구한다.
    sphere->setRigidBodyFlag(aimed ? PxRigidBodyFlag::eENABLE_CCD : PxRigidBodyFlag::eENABLE_SPECULATIVE_CCD, true);
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
        const uint32_t contacts = m_FootSoles.GetLegContacts(&dummy->GetMesh(), m_LegContactMessage);
        if (!m_World.IsPhysicsPaused())
        {
            m_PausedLegContactMessage.clear();
            if (m_ContactCheckEnabled && (contacts & ~m_PreviousLegContacts) != 0)
            {
                m_PausedLegContactMessage = m_LegContactMessage;
                m_World.SetPhysicsPaused(true);
            }
            m_PreviousLegContacts = contacts;
        }
    }
    else
    {
        ClearContactCheck();
    }
}

void RagdollImpactTest::DrawContactOverlay(float topInset) const
{
    const PARS::Character* dummy = GetDummy();
    if (!dummy)
    {
        return;
    }
    const bool contactPaused = m_World.IsPhysicsPaused() && !m_PausedLegContactMessage.empty();
    std::string text = contactPaused ? m_PausedLegContactMessage : m_LegContactMessage;
    const bool touching = !text.empty();
    if (!touching)
    {
        text = u8"[더미] 이동 다리의 실제 자기 접촉: 없음";
    }
    text += '\n';
    const PARS::SkeletalMeshComponent& mesh = dummy->GetMesh();
    text += m_FootSoles.GetLegPredictionText(mesh);
    text += m_ContactCheckEnabled ? u8"\n접촉 확인 모드: 켜짐 (T: 끄기)" : u8"\n접촉 확인 모드: 꺼짐 (T: 켜기)";
    if (contactPaused)
    {
        text += u8"\n다리 접촉으로 시뮬레이션 일시정지 · P: 재개";
    }
    else if (m_World.IsPhysicsPaused())
    {
        text += u8"\n시뮬레이션 일시정지 · P: 재개";
    }
    else
    {
        text += u8"\n시뮬레이션 중 · P: 일시정지";
    }
    const bool predicted = mesh.GetFootSupport().GetFootstepSettings().pathAvoidanceEnabled &&
        mesh.GetFootstepCollisionSnapshot().overlapPairs != 0;
    m_FootSoles.DrawContactOverlay(text, touching || predicted, topInset);
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
        ImGui::DragFloat(u8"구 질량", &m_Mass, .5f, .1f, 100, "%.1f kg", ImGuiSliderFlags_AlwaysClamp);
        ImGui::DragFloat(u8"발사 속도", &m_Speed, .5f, 1, 100, "%.1f m/s", ImGuiSliderFlags_AlwaysClamp);
        ImGui::DragFloat(u8"전신 충격량", &m_BodyImpulseMagnitude, 1.0f, 0.0f, PX_MAX_F32, u8"%.1f N·s", ImGuiSliderFlags_AlwaysClamp);
        ImGui::DragFloat(u8"전신 회전 배율", &m_BodyTorqueMultiplier, .01f, 0.0f, PX_MAX_F32, "%.2f", ImGuiSliderFlags_AlwaysClamp);
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
        ImGui::TextWrapped("%s", u8"구 설정을 저장하면 다음 실행 시 자동으로 적용합니다.");
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
            ImGui::Text(u8"추종 애니메이션  %s", FootSoleDebug::GetFollowedAnimationText(mesh).c_str());
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
            if (m_FootSoles.IsVisible())
            {
                m_FootSoles.DrawUI(dummy->GetMesh(), [this]() -> void
                {
                    DrawDummySupportSettingsUI();
                });
            }
            ImGui::PopID();
        }
        if (!m_Message.empty())
        {
            ImGui::TextWrapped("%s", m_Message.c_str());
        }
    }
    ImGui::End();
}
