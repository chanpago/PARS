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
    constexpr float MinimumProjectileRadius = .01f;
    constexpr float MaximumProjectileRadius = 1.0f;
    constexpr float MinimumArrowRadius = .002f;
    constexpr float MaximumArrowRadius = .05f;
    constexpr float MinimumArrowLength = .15f;
    constexpr float MaximumArrowLength = 1.5f;

    bool ValidProjectileSettings(ImpactProjectileType type, const ImpactProjectileSettings& settings)
    {
        const bool arrow = type == ImpactProjectileType::Arrow;
        const float minRadius = arrow ? MinimumArrowRadius : MinimumProjectileRadius;
        const float maxRadius = arrow ? MaximumArrowRadius : MaximumProjectileRadius;
        return (arrow || type == ImpactProjectileType::Sphere) && std::isfinite(settings.mass) && settings.mass >= .1f && settings.mass <= 100 &&
            std::isfinite(settings.speed) && settings.speed >= 1 && settings.speed <= 100 &&
            std::isfinite(settings.radius) && settings.radius >= minRadius && settings.radius <= maxRadius &&
            (!arrow || (std::isfinite(settings.length) && settings.length >= MinimumArrowLength && settings.length <= MaximumArrowLength)) &&
            settings.IsValid();
    }

    ImpactImpulseSettings ReadImpulseSettings(const nlohmann::json& data)
    {
        ImpactImpulseSettings settings;
        settings.partImpulse = data.at("partImpulseNs").get<float>();
        settings.bodyImpulse = data.at("bodyImpulseNs").get<float>();
        settings.torqueMultiplier = data.at("bodyTorqueMultiplier").get<float>();
        if (!settings.IsValid())
        {
            throw std::runtime_error(u8"충격량과 회전 배율은 유한한 0 이상의 값이어야 합니다.");
        }
        return settings;
    }

    nlohmann::json WriteImpulseSettings(const ImpactImpulseSettings& settings)
    {
        return {{"partImpulseNs", settings.partImpulse}, {"bodyImpulseNs", settings.bodyImpulse},
            {"bodyTorqueMultiplier", settings.torqueMultiplier}};
    }

    ImpactProjectileSettings ReadProjectileSettings(const nlohmann::json& data, int version, ImpactProjectileType type)
    {
        ImpactProjectileSettings settings;
        settings.mass = data.at("massKg").get<float>();
        settings.speed = data.at("speedMetersPerSecond").get<float>();
        if (version >= 3)
        {
            static_cast<ImpactImpulseSettings&>(settings) = ReadImpulseSettings(data);
        }
        else if (version == 2)
        {
            settings.bodyImpulse = data.at("bodyImpulseNs").get<float>();
            settings.torqueMultiplier = data.at("bodyTorqueMultiplier").get<float>();
        }
        if (version >= 4)
        {
            settings.radius = data.at("radiusMeters").get<float>();
        }
        if (type == ImpactProjectileType::Arrow)
        {
            settings.length = data.at("lengthMeters").get<float>();
        }
        if (!ValidProjectileSettings(type, settings))
        {
            throw std::runtime_error(u8"발사체 설정값이 허용 범위를 벗어났습니다.");
        }
        return settings;
    }

    nlohmann::json WriteProjectileSettings(ImpactProjectileType type, const ImpactProjectileSettings& settings)
    {
        if (!ValidProjectileSettings(type, settings))
        {
            throw std::runtime_error(u8"발사체 설정값이 허용 범위를 벗어났습니다.");
        }
        nlohmann::json data = WriteImpulseSettings(settings);
        data["massKg"] = settings.mass;
        data["speedMetersPerSecond"] = settings.speed;
        data["radiusMeters"] = settings.radius;
        if (type == ImpactProjectileType::Arrow)
        {
            data["lengthMeters"] = settings.length;
        }
        return data;
    }

    bool GetAttachmentPose(const PhysicsProjectileAttachment& hit, const Character* dummy, const PxScene* scene, PxTransform& pose)
    {
        PxTransform targetPose(PxIdentity);
        if (dummy && hit.targetOwner == dummy)
        {
            // 키네마틱 전환으로 물리 바디가 교체되어도 같은 본의 현재 자세를 찾는다.
            if (!dummy->GetMesh().GetRagdoll().GetBodyPose(hit.targetName, targetPose))
            {
                return false;
            }
        }
        else if (hit.target && hit.target->getScene() == scene)
        {
            targetPose = hit.target->getGlobalPose();
        }
        else
        {
            return false;
        }
        pose = targetPose * hit.localPose;
        return true;
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

bool RagdollImpactTest::Initialize(std::shared_ptr<const Model> dummyMesh, const PhysicsAsset& asset, const AnimInstance& motionSource)
{
    if (!dummyMesh)
    {
        m_Message = u8"피격 더미 메시 Erika Archer를 불러오지 못했습니다.";
        return false;
    }
    m_Asset = asset;
    if (std::none_of(m_Asset.constraints.begin(), m_Asset.constraints.end(), HasEnabledAngularDrive))
    {
        ConfigureDefaultDrives(m_Asset);
    }
    PARS::Character* dummy = m_World.SpawnActor<Character>("Erika Archer");
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
    if (!mesh.SetSkeletalMesh(dummyMesh))
    {
        m_Message = u8"더미 메시를 초기화하지 못했습니다.";
        return false;
    }
    AnimInstance& animation = mesh.GetAnimInstance();
    if (!animation.ShareMotionFramesFrom(motionSource, m_Message))
    {
        return false;
    }
    if (!animation.PlayAnimation(0))
    {
        m_Message = u8"더미의 Idle 애니메이션을 등록하지 못했습니다. 파일과 본 구조를 확인하세요.";
        return false;
    }
    if (!mesh.SetPhysicsAsset(m_Asset, m_Message))
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

bool RagdollImpactTest::SetProjectileSettings(float mass, float speed, float bodyImpulse, float torqueMultiplier,
    float partImpulse, float radius)
{
    ImpactProjectileSettings settings;
    settings.mass = mass;
    settings.speed = speed;
    settings.radius = radius;
    settings.partImpulse = partImpulse;
    settings.bodyImpulse = bodyImpulse;
    settings.torqueMultiplier = torqueMultiplier;
    return SetProjectileSettings(ImpactProjectileType::Sphere, settings);
}

const ImpactProjectileSettings& RagdollImpactTest::GetSelectedProjectileSettings() const
{
    return m_ProjectileType == ImpactProjectileType::Arrow ? m_ArrowSettings : m_SphereSettings;
}

bool RagdollImpactTest::SetProjectileSettings(ImpactProjectileType type, const ImpactProjectileSettings& settings)
{
    if (!ValidProjectileSettings(type, settings))
    {
        return false;
    }
    ImpactProjectileSettings& selected = type == ImpactProjectileType::Arrow ? m_ArrowSettings : m_SphereSettings;
    selected = settings;
    m_ProjectileType = type;
    return true;
}

bool RagdollImpactTest::LoadImpactSettings(Character& player, const std::string& path)
{
    m_ImpactSettingsPath = path;
    try
    {
        std::ifstream file(ImpactSettingsPath(path));
        if (!file)
        {
            throw std::runtime_error(u8"충격 설정 파일을 열 수 없습니다.");
        }
        nlohmann::json data;
        file >> data;
        const int version = data.at("version").get<int>();
        if (version < 1 || version > 6)
        {
            throw std::runtime_error(u8"지원하지 않는 충격 설정 파일 버전입니다.");
        }
        const ImpactProjectileSettings sphere = ReadProjectileSettings(version >= 5 ? data.at("sphere") : data, version, ImpactProjectileType::Sphere);
        const ImpactProjectileSettings arrow = version >= 5 ? ReadProjectileSettings(data.at("arrow"), version, ImpactProjectileType::Arrow) :
            ImpactProjectileSettings::ArrowDefaults();
        const std::string selected = version >= 5 ? data.at("selected").get<std::string>() : "sphere";
        const bool editCapsule = version >= 6 && data.value("editingCapsule", false);
        const ImpactImpulseSettings capsule = version >= 6 ? ReadImpulseSettings(data.at("playerCapsule")) : ImpactImpulseSettings{};
        if (selected != "sphere" && selected != "arrow")
        {
            throw std::runtime_error(u8"알 수 없는 발사체 종류입니다.");
        }
        m_SphereSettings = sphere;
        m_ArrowSettings = arrow;
        m_ProjectileType = selected == "arrow" ? ImpactProjectileType::Arrow : ImpactProjectileType::Sphere;
        m_EditingCapsuleImpact = editCapsule;
        player.GetCharacterImpact().SetImpactImpulseSettings(capsule);
        m_ImpactSettingsMessage = u8"충격 설정을 불러왔습니다.";
        return true;
    }
    catch (const std::exception& exception)
    {
        m_ImpactSettingsMessage = u8"충격 설정을 불러오지 못했습니다. " + std::string(exception.what());
        return false;
    }
}

bool RagdollImpactTest::SaveImpactSettings(const Character& player)
{
    std::wstring temporary;
    try
    {
        const nlohmann::json data = {{"version", 6}, {"selected", m_ProjectileType == ImpactProjectileType::Arrow ? "arrow" : "sphere"},
            {"editingCapsule", m_EditingCapsuleImpact}, {"playerCapsule", WriteImpulseSettings(player.GetCharacterImpact().GetImpactImpulseSettings())},
            {"sphere", WriteProjectileSettings(ImpactProjectileType::Sphere, m_SphereSettings)},
            {"arrow", WriteProjectileSettings(ImpactProjectileType::Arrow, m_ArrowSettings)}};
        const std::wstring destination = ImpactSettingsPath(m_ImpactSettingsPath);
        temporary = destination + L".tmp." + std::to_wstring(GetCurrentProcessId()) + L"." + std::to_wstring(GetCurrentThreadId());
        std::ofstream file(temporary, std::ios::trunc);
        file << data.dump(4) << '\n';
        file.close();
        if (!file || !MoveFileExW(temporary.c_str(), destination.c_str(), MOVEFILE_REPLACE_EXISTING | MOVEFILE_WRITE_THROUGH))
        {
            throw std::runtime_error(u8"충격 설정 파일에 저장할 수 없습니다.");
        }
        m_ImpactSettingsMessage = u8"충격 설정을 저장했습니다. 다음 실행부터 자동으로 불러옵니다.";
        return true;
    }
    catch (const std::exception& exception)
    {
        if (!temporary.empty())
        {
            DeleteFileW(temporary.c_str());
        }
        m_ImpactSettingsMessage = u8"충격 설정을 저장하지 못했습니다. " + std::string(exception.what());
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
    // 구의 표면 또는 화살 끝의 충돌 구가 조준점에 닿도록 중심 위치를 보정한다.
    const PxVec3 target = closest.position + closest.normal * GetSelectedProjectileSettings().radius;
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
    const ImpactProjectileSettings& settings = GetSelectedProjectileSettings();
    if (!ValidProjectileSettings(m_ProjectileType, settings))
    {
        m_Message = u8"발사체 설정값을 확인하세요.";
        return false;
    }
    const bool arrow = m_ProjectileType == ImpactProjectileType::Arrow;
    const float clearance = player.GetCapsuleComponent().GetRadius() + .15f + (arrow ? settings.length : settings.radius);
    if (float(Length(delta)) < clearance + settings.radius)
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
    if ((aimed || arrow) && physics->SweepSphereDistance(toPx(origin), toPx(muzzle - origin), settings.radius, &player) < clearance - .001f)
    {
        m_Message = u8"발사 위치가 막혀 있습니다. 장애물에서 떨어져 발사하세요.";
        return false;
    }
    const PxVec3 heading = toPx(direction);
    const PxTransform start(toPx(muzzle), arrow ? PxShortestRotation(PxVec3(1, 0, 0), heading) : PxQuat(PxIdentity));
    PxRigidDynamic* projectile = physics->CreateDynamicBody(start, PxSphereGeometry(settings.radius),
        settings.mass, arrow ? "Impact arrow tip" : "Impact sphere");
    if (!projectile)
    {
        m_Message = u8"발사체를 생성하지 못했습니다.";
        return false;
    }
    // 바닥 하중이나 캐릭터의 지속 밀기와 구별해 실제 발사 구의 접촉만 피격 반응에 사용한다.
    physx::PxShape* shape = nullptr;
    if (projectile->getShapes(&shape, 1) != 1 ||
        !physics->SetImpactImpulseSettings(*projectile, settings))
    {
        projectile->release();
        return false;
    }
    PxFilterData filter = shape->getSimulationFilterData();
    filter.word3 |= PhysicsWorld::ProjectileFilterFlag;
    if (arrow)
    {
        filter.word3 |= PhysicsWorld::AttachOnContactFilterFlag;
    }
    if (aimed)
    {
        filter.word3 |= PhysicsWorld::AimedProjectileFilterFlag;
        projectile->setActorFlag(PxActorFlag::eDISABLE_GRAVITY, true);
    }
    shape->setSimulationFilterData(filter);
    if (aimed || arrow)
    {
        projectile->setMinCCDAdvanceCoefficient(0.0f);
        projectile->setRigidBodyFlag(PxRigidBodyFlag::eENABLE_CCD, true);
    }
    else
    {
        projectile->setRigidBodyFlag(PxRigidBodyFlag::eENABLE_SPECULATIVE_CCD, true);
    }
    projectile->setLinearVelocity(heading * settings.speed);
    if (m_Shots.size() >= 12)
    {
        m_Shots.front().body->release();
        m_Shots.erase(m_Shots.begin());
    }
    m_Shots.push_back({projectile, physics->GetStepCount() + 600, settings.radius, arrow ? settings.length : 0.0f, m_ProjectileType});
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

std::vector<ImpactProjectileVisual> RagdollImpactTest::GetProjectileVisuals() const
{
    std::vector<ImpactProjectileVisual> visuals;
    visuals.reserve(m_Shots.size());
    const PhysicsWorld* physics = m_World.GetPhysicsWorld();
    const Character* dummy = GetDummy();
    for (const Shot& shot : m_Shots)
    {
        ImpactProjectileVisual visual;
        visual.type = shot.type;
        visual.radius = shot.radius;
        visual.length = shot.length;
        visual.pose = shot.body->getGlobalPose();
        if (shot.type == ImpactProjectileType::Arrow)
        {
            const PhysicsProjectileAttachment* hit = physics->GetProjectileAttachment(*shot.body);
            float tipOffset = shot.radius;
            if (hit)
            {
                if (!GetAttachmentPose(*hit, dummy, shot.body->getScene(), visual.pose))
                {
                    continue;
                }
                tipOffset = shot.length * .1f;
            }
            else
            {
                PxVec3 direction = shot.body->getLinearVelocity();
                if (direction.normalize() > 1e-5f)
                {
                    visual.pose.q = PxShortestRotation(PxVec3(1, 0, 0), direction);
                }
            }
            visual.pose.p += visual.pose.q.getBasisVector0() * (tipOffset - shot.length * .5f);
        }
        visuals.push_back(visual);
    }
    return visuals;
}

void RagdollImpactTest::Update()
{
    const PhysicsWorld* physics = m_World.GetPhysicsWorld();
    const uint64_t step = physics->GetStepCount();
    PARS::Character* dummy = GetDummy();
    m_Shots.erase(std::remove_if(m_Shots.begin(), m_Shots.end(), [physics, step, dummy](const Shot& shot) -> bool
    {
        const PhysicsProjectileAttachment* hit = shot.type == ImpactProjectileType::Arrow ? physics->GetProjectileAttachment(*shot.body) : nullptr;
        PxTransform pose(PxIdentity);
        if (hit ? GetAttachmentPose(*hit, dummy, shot.body->getScene(), pose) : step < shot.expiresAtStep)
        {
            return false;
        }
        shot.body->release();
        return true;
    }),
        m_Shots.end());
    if (dummy)
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

void RagdollImpactTest::DrawContactOverlay(const ImVec2& sceneOrigin, const ImVec2& sceneSize) const
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
    m_FootSoles.DrawContactOverlay(text, touching || predicted, sceneOrigin, sceneSize);
}

void RagdollImpactTest::BuildLines(std::vector<PxDebugLine>& lines)
{
    BuildBodyImpactLines(lines);
    if (PARS::Character* dummy = GetDummy())
    {
        m_FootSoles.BuildLines(dummy->GetMesh(), lines);
    }
}

namespace
{
    void DrawProjectileLaunchSettings(bool arrow, ImpactProjectileSettings& settings)
    {
        float radiusCm = settings.radius * 100.0f;
        const float minRadius = arrow ? MinimumArrowRadius : MinimumProjectileRadius;
        const float maxRadius = arrow ? MaximumArrowRadius : MaximumProjectileRadius;
        if (ImGui::DragFloat(arrow ? u8"화살대 반지름" : u8"공 반지름", &radiusCm, arrow ? .1f : .5f,
            minRadius * 100.0f, maxRadius * 100.0f, "%.1f cm", ImGuiSliderFlags_AlwaysClamp) && std::isfinite(radiusCm))
        {
            settings.radius = radiusCm / 100.0f;
        }
        if (arrow)
        {
            float lengthCm = settings.length * 100.0f;
            if (ImGui::DragFloat(u8"화살 길이", &lengthCm, .5f, MinimumArrowLength * 100.0f, MaximumArrowLength * 100.0f,
                "%.1f cm", ImGuiSliderFlags_AlwaysClamp) && std::isfinite(lengthCm))
            {
                settings.length = lengthCm / 100.0f;
            }
        }
        ImGui::DragFloat(arrow ? u8"화살 질량" : u8"구 질량", &settings.mass, arrow ? .1f : .5f, .1f, 100, "%.1f kg", ImGuiSliderFlags_AlwaysClamp);
        ImGui::DragFloat(u8"발사 속도", &settings.speed, .5f, 1, 100, "%.1f m/s", ImGuiSliderFlags_AlwaysClamp);
    }

    bool DrawImpulseSettings(ImpactImpulseSettings& settings)
    {
        bool changed = ImGui::DragFloat(u8"피격 부위 추가 충격량", &settings.partImpulse, 1.0f, 0.0f, PX_MAX_F32, u8"%.1f N·s", ImGuiSliderFlags_AlwaysClamp);
        changed |= ImGui::DragFloat(u8"전신 충격량", &settings.bodyImpulse, 1.0f, 0.0f, PX_MAX_F32, u8"%.1f N·s", ImGuiSliderFlags_AlwaysClamp);
        changed |= ImGui::DragFloat(u8"전신 회전 배율", &settings.torqueMultiplier, .01f, 0.0f, PX_MAX_F32, "%.2f", ImGuiSliderFlags_AlwaysClamp);
        return changed;
    }
}

void RagdollImpactTest::DrawImpactSettingsUI(Character* player)
{
    if (!ImGui::CollapsingHeader(u8"충격 설정", ImGuiTreeNodeFlags_DefaultOpen))
    {
        return;
    }
    if (ImGui::RadioButton(u8"구", !m_EditingCapsuleImpact && m_ProjectileType == ImpactProjectileType::Sphere))
    {
        m_ProjectileType = ImpactProjectileType::Sphere;
        m_EditingCapsuleImpact = false;
    }
    ImGui::SameLine();
    if (ImGui::RadioButton(u8"화살", !m_EditingCapsuleImpact && m_ProjectileType == ImpactProjectileType::Arrow))
    {
        m_ProjectileType = ImpactProjectileType::Arrow;
        m_EditingCapsuleImpact = false;
    }
    ImGui::SameLine();
    if (ImGui::RadioButton(u8"플레이어 캡슐", m_EditingCapsuleImpact))
    {
        m_EditingCapsuleImpact = true;
    }
    const bool arrow = m_ProjectileType == ImpactProjectileType::Arrow;
    ImGui::PushID(m_EditingCapsuleImpact ? "playerCapsule" : (arrow ? "arrow" : "sphere"));
    if (m_EditingCapsuleImpact)
    {
        if (player)
        {
            CharacterImpactComponent& component = player->GetCharacterImpact();
            ImpactImpulseSettings settings = component.GetImpactImpulseSettings();
            if (DrawImpulseSettings(settings))
            {
                component.SetImpactImpulseSettings(settings);
            }
        }
        ImGui::TextWrapped("%s", u8"플레이어 몸통이 더미에 처음 부딪힐 때 적용합니다. 떨어졌다 다시 부딪히면 새 충격을 줍니다.");
    }
    else
    {
        ImpactProjectileSettings& settings = arrow ? m_ArrowSettings : m_SphereSettings;
        DrawProjectileLaunchSettings(arrow, settings);
        DrawImpulseSettings(settings);
    }
    ImGui::PopID();
    ImGui::BeginDisabled(m_ImpactSettingsPath.empty() || !player);
    if (ImGui::Button(u8"충격 설정 저장") && player)
    {
        SaveImpactSettings(*player);
    }
    ImGui::SameLine();
    if (ImGui::Button(u8"충격 설정 불러오기") && player)
    {
        LoadImpactSettings(*player, m_ImpactSettingsPath);
    }
    ImGui::EndDisabled();
    if (!m_ImpactSettingsMessage.empty())
    {
        ImGui::TextWrapped("%s", m_ImpactSettingsMessage.c_str());
    }
    if (m_EditingCapsuleImpact)
    {
        return;
    }
    ImGui::TextWrapped("%s", u8"우클릭으로 조준하고 좌클릭 또는 R로 발사합니다. 조준하지 않고 R을 누르면 더미 골반을 향해 발사합니다.");
    ImGui::BeginDisabled(!player);
    if (ImGui::Button(arrow ? u8"더미로 화살 발사" : u8"더미로 구 발사") && player)
    {
        FireFrom(*player);
    }
    ImGui::EndDisabled();
    ImGui::SameLine();
    if (ImGui::Button(u8"발사체 제거"))
    {
        ClearProjectiles();
    }
    ImGui::SameLine();
    ImGui::Text(u8"남은 발사체 %zu개", m_Shots.size());
}

void RagdollImpactTest::DrawUI(Character* player)
{
    if (ImGui::BeginChild("DummyImpactSettings"))
    {
        DrawBodyImpactUI(player);
        DrawImpactSettingsUI(player);
        PARS::Character* dummy = GetDummy();
        if (ImGui::CollapsingHeader(u8"더미 상태 및 초기화"))
        {
            if (ImGui::Button(u8"더미 초기화"))
            {
                Reset();
            }
            if (dummy)
            {
                const SkeletalMeshComponent& mesh = dummy->GetMesh();
                ImGui::Text(u8"물리 모드: %s", mesh.IsActiveRagdoll() ? u8"액티브 래그돌" :
                    mesh.IsRagdoll() ? u8"패시브 래그돌" : mesh.IsBlendingToKinematic() ? u8"키네마틱 복귀 보간" : u8"키네마틱");
                ImGui::Text(u8"추종 애니메이션  %s", FootSoleDebug::GetFollowedAnimationText(mesh).c_str());
            }
        }
        if (dummy)
        {
            SkeletalMeshComponent& mesh = dummy->GetMesh();
            PARS::Ragdoll& ragdoll = mesh.GetRagdoll();
            if (ImGui::CollapsingHeader(u8"골반 지지 및 복귀"))
            {
                bool automatic = m_AutomaticSupport;
                if (ImGui::Checkbox(u8"COM 자동 골반 지지", &automatic))
                {
                    SetAutomaticSupport(automatic);
                }
                ImGui::TextUnformatted(u8"모드 변경 시 더미를 초기화합니다.");
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
                float stableDuration = mesh.GetKinematicStableDuration();
                if (ImGui::DragFloat(u8"A영역 복귀 대기 시간", &stableDuration, .1f, .1f, 10.0f, "%.1f s", ImGuiSliderFlags_AlwaysClamp))
                {
                    mesh.SetKinematicStableDuration(stableDuration);
                }
                ImGui::Text(u8"A영역 연속 유지: %.2f / %.2f s", mesh.GetKinematicStableTime(), stableDuration);
            }
            if (ImGui::CollapsingHeader(u8"골반 이동 측정"))
            {
                const Math::Vector3 hips = ragdoll.GetPosition();
                ImGui::Text(u8"실제 Hips: (%.3f, %.3f, %.3f) m", float(hips.GetX()), float(hips.GetY()), float(hips.GetZ()));
                ImGui::Text(u8"기준 대비 골반 이동: %.2f cm / 최대 %.2f cm",
                    float(Length(hips - m_HipsReference)) * 100, m_MaxHipsDisplacement * 100);
                if (ragdoll.IsRootSupported())
                {
                    ImGui::Text(u8"목표 Y %.3f m / 지지 위치 오차 %.2f cm",
                        ragdoll.GetRootTargetHeight(), ragdoll.GetRootPositionError() * 100);
                }
                ImGui::TextUnformatted(u8"기준은 초기화 또는 마지막 발사 시점입니다.");
            }
        }
        if (!m_Message.empty() && ImGui::CollapsingHeader(u8"테스트 메시지", ImGuiTreeNodeFlags_DefaultOpen))
        {
            ImGui::TextWrapped("%s", m_Message.c_str());
        }
        if (dummy && m_FootSoles.IsVisible())
        {
            ImGui::PushID("ImpactDummy");
            m_FootSoles.DrawUI(dummy->GetMesh(), [this]() -> void
            {
                DrawDummySupportSettingsUI();
            });
            ImGui::PopID();
        }
    }
    ImGui::EndChild();
}
