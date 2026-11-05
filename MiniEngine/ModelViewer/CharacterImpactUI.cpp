#include "RagdollImpactTest.h"
#include "GameFramework/Character.h"
#include "GameFramework/World.h"
#include "imgui.h"
#include <cmath>

namespace
{
    void AppendBodyImpactCapsule(std::vector<physx::PxDebugLine>& lines, const physx::PxTransform& pose,
        float radius, float halfLength)
    {
        constexpr physx::PxU32 color = 0xff00ffff;
        // PhysX 캡슐의 로컬 X축을 기준으로 윤곽을 만든 뒤 판정에 쓰는 자세로 변환한다.
        for (int end : {-1, 1})
        {
            const float x = end * halfLength;
            for (int segment = 0; segment < 24; ++segment)
            {
                const float a = segment * physx::PxTwoPi / 24;
                const float b = (segment + 1) * physx::PxTwoPi / 24;
                const physx::PxVec3 p(x, radius * std::cos(a), radius * std::sin(a));
                const physx::PxVec3 q(x, radius * std::cos(b), radius * std::sin(b));
                lines.emplace_back(pose.transform(p), pose.transform(q), color);
            }
            for (int plane = 0; plane < 2; ++plane)
            {
                for (int segment = 0; segment < 12; ++segment)
                {
                    const float a = segment * physx::PxPi / 12;
                    const float b = (segment + 1) * physx::PxPi / 12;
                    const physx::PxVec3 p(x + end * radius * std::sin(a), radius * std::cos(a), 0);
                    const physx::PxVec3 q(x + end * radius * std::sin(b), radius * std::cos(b), 0);
                    const physx::PxVec3 from = plane ? physx::PxVec3(p.x, 0, p.y) : p;
                    const physx::PxVec3 to = plane ? physx::PxVec3(q.x, 0, q.y) : q;
                    lines.emplace_back(pose.transform(from), pose.transform(to), color);
                }
            }
        }
        for (int side = 0; side < 4; ++side)
        {
            const float angle = side * physx::PxHalfPi;
            const float y = radius * std::cos(angle);
            const float z = radius * std::sin(angle);
            lines.emplace_back(pose.transform(physx::PxVec3(-halfLength, y, z)),
                pose.transform(physx::PxVec3(halfLength, y, z)), color);
        }
    }
}

void RagdollImpactTest::BuildBodyImpactLines(std::vector<physx::PxDebugLine>& lines)
{
    if (!m_BodyImpactCapsuleVisible)
    {
        return;
    }
    m_World.ForEachActor([&](PARS::Actor& actor) -> void
    {
        const PARS::Character* character = dynamic_cast<const PARS::Character*>(&actor);
        if (!character)
        {
            return;
        }
        const PARS::Ragdoll& ragdoll = character->GetMesh().GetRagdoll();
        const PARS::CharacterImpactSettings& settings = character->GetCharacterImpact().GetSettings();
        if (!settings.enabled || !ragdoll.HasCharacterCapsule())
        {
            return;
        }
        physx::PxTransform pose(physx::PxIdentity);
        physx::PxVec3 targetOffset(0);
        float radius = 0;
        float halfLength = 0;
        if (!ragdoll.GetCharacterImpactCapsule(settings.capsuleRadiusScale, pose, radius, halfLength, targetOffset))
        {
            return;
        }
        // 실제 시뮬레이션 형상의 자세와 크기를 표시한다.
        AppendBodyImpactCapsule(lines, pose, radius, halfLength);
    });
}

void RagdollImpactTest::LoadBodyImpactSettings(PARS::Character& player, const std::string& path)
{
    m_BodyImpactPath = path;
    PARS::CharacterImpactSettings settings;
    if (settings.Load(path, m_BodyImpactMessage))
    {
        if (player.GetCharacterImpact().SetSettings(settings, m_BodyImpactMessage))
        {
            m_BodyImpactMessage = u8"저장된 캡슐 설정을 불러왔습니다.";
        }
    }
}

void RagdollImpactTest::DrawBodyImpactUI(PARS::Character* player)
{
    if (!player || !ImGui::CollapsingHeader(u8"캐릭터 몸통 충돌", ImGuiTreeNodeFlags_DefaultOpen))
    {
        return;
    }
    PARS::CharacterImpactComponent& component = player->GetCharacterImpact();
    PARS::CharacterImpactSettings settings = component.GetSettings();
    ImGui::Checkbox(u8"몸통 충돌 캡슐 표시", &m_BodyImpactCapsuleVisible);
    ImGui::TextWrapped("%s", u8"청록색 윤곽: 기존 동적 골반 바디에 붙은 실제 몸통 충돌체.");
    bool changed = ImGui::Checkbox(u8"몸통 캡슐 충돌 사용", &settings.enabled);
    changed |= ImGui::InputFloat(u8"캡슐 바디 질량 (kg)", &settings.bodyMass, 1, 10, "%.3f");
    ImGui::Separator();
    ImGui::TextUnformatted(u8"캡슐 크기");
    changed |= ImGui::SliderFloat(u8"반경 배율", &settings.capsuleRadiusScale, .1f, 3.0f, u8"%.2f 배", ImGuiSliderFlags_AlwaysClamp);
    changed |= ImGui::SliderFloat(u8"높이 배율", &settings.capsuleHeightScale, .1f, 3.0f, u8"%.2f 배", ImGuiSliderFlags_AlwaysClamp);
    if (ImGui::Button(u8"캡슐 크기 초기화"))
    {
        settings.capsuleRadiusScale = .95f;
        settings.capsuleHeightScale = 1.0f;
        changed = true;
    }
    ImGui::TextWrapped("%s", u8"상체 범위에 대한 배율입니다. 전체 높이는 지름보다 작아질 수 없습니다.");
    if (ImGui::TreeNode(u8"접촉 설정"))
    {
        changed |= ImGui::InputFloat(u8"접촉 여유 (m)", &settings.contactSkin, .005f, .01f, "%.3f");
        ImGui::TreePop();
    }
    if (changed)
    {
        component.SetSettings(settings, m_BodyImpactMessage);
    }
    physx::PxTransform capsulePose(physx::PxIdentity);
    physx::PxVec3 targetOffset(0);
    float capsuleRadius = 0;
    float capsuleHalfLength = 0;
    const PARS::CharacterImpactSettings& applied = component.GetSettings();
    if (player->GetMesh().GetRagdoll().GetCharacterImpactCapsule(applied.capsuleRadiusScale, capsulePose,
        capsuleRadius, capsuleHalfLength, targetOffset, applied.capsuleHeightScale))
    {
        ImGui::Text(u8"크기: 반경 %.1f cm / 전체 높이 %.1f cm", capsuleRadius * 100, (capsuleHalfLength + capsuleRadius) * 200);
    }
    ImGui::Separator();
    if (ImGui::Button(u8"캡슐 설정 저장"))
    {
        if (component.GetSettings().Save(m_BodyImpactPath, m_BodyImpactMessage))
        {
            m_BodyImpactMessage = u8"캡슐 설정을 저장했습니다. 다음 실행부터 자동으로 불러옵니다.";
        }
    }
    ImGui::SameLine();
    if (ImGui::Button(u8"캡슐 설정 불러오기"))
    {
        LoadBodyImpactSettings(*player, m_BodyImpactPath);
    }
    ImGui::TextWrapped("%s", u8"크기·질량·충돌 사용 여부를 함께 저장하며, 다음 실행 시 자동으로 적용합니다.");
    ImGui::TextWrapped("%s", u8"루트 포함 상체와의 충격·마찰·관절 반응을 PhysX가 계산합니다. 끄면 캡슐을 제거하고 원래 질량과 PA 충돌로 복원합니다.");
    const PARS::Ragdoll& ragdoll = player->GetMesh().GetRagdoll();
    Math::Vector3 center(Math::kZero);
    float totalMass = 0;
    ragdoll.GetCenterOfMass(center, totalMass);
    ImGui::Text(u8"현재 캡슐 바디 %.3f kg / 캐릭터 전체 %.3f kg", ragdoll.GetCharacterCapsuleMass(), totalMass);
    ImGui::TextWrapped("%s", u8"설정 질량은 골반 바디의 질량을 교체합니다. 별도의 캡슐 무게를 추가하지 않습니다.");
    const PARS::CharacterImpactSnapshot& current = component.GetSnapshot();
    const PARS::CharacterImpactSnapshot& last = component.GetLastImpact();
    const char* reasons[] = {u8"접촉 없음", u8"실제 접촉", u8"비활성"};
    ImGui::Text(u8"현재: %s", reasons[static_cast<int>(current.reason)]);
    ImGui::Text(u8"접촉 기록 %llu / 최근 법선 충격량 %.3f N s",
        static_cast<unsigned long long>(last.eventCount), last.normalImpulse);
    ImGui::Text(u8"접촉 바디: %s", last.hitBone.c_str());
    ImGui::Text(u8"방향: (%.2f, %.2f, %.2f)", last.direction.x, last.direction.y, last.direction.z);
    if (ImGui::TreeNode(u8"최근 바디별 접촉 측정"))
    {
        for (const PARS::BodyImpactShare& share : last.shares)
        {
            ImGui::Text("%s: %.2f N s", share.bone.c_str(), share.impulse.magnitude());
        }
        ImGui::TextWrapped("%s", u8"표시량은 완료된 물리 스텝에서 PhysX 솔버가 계산한 접촉 충격량입니다.");
        ImGui::TreePop();
    }
    if (!m_BodyImpactMessage.empty())
    {
        ImGui::TextWrapped("%s", m_BodyImpactMessage.c_str());
    }
    if (!component.GetPhysicsError().empty())
    {
        ImGui::TextWrapped("%s", component.GetPhysicsError().c_str());
    }
    ImGui::Separator();
}
