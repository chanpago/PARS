#include "RagdollImpactTest.h"
#include "GameFramework/Character.h"
#include "imgui.h"

void RagdollImpactTest::LoadBodyImpactSettings(PARS::Character& player, const std::string& path)
{
    m_BodyImpactPath = path;
    PARS::CharacterImpactSettings settings;
    if (settings.Load(path, m_BodyImpactMessage))
    {
        player.GetCharacterImpact().SetSettings(settings, m_BodyImpactMessage);
    }
}

void RagdollImpactTest::DrawBodyImpactUI(PARS::Character* player)
{
    if (!player || !ImGui::CollapsingHeader(u8"캐릭터 몸통 밀침", ImGuiTreeNodeFlags_DefaultOpen))
    {
        return;
    }
    PARS::CharacterImpactComponent& component = player->GetCharacterImpact();
    PARS::CharacterImpactSettings settings = component.GetSettings();
    bool changed = ImGui::Checkbox(u8"캡슐 밀침 사용", &settings.enabled);
    changed |= ImGui::InputFloat(u8"기준 충격량 (N s)", &settings.referenceImpulse, 10, 100, "%.1f");
    changed |= ImGui::InputFloat(u8"기준 속도 (m/s)", &settings.referenceSpeed, .1f, 1, "%.2f");
    changed |= ImGui::InputFloat(u8"충격 배율", &settings.strengthMultiplier, .1f, 1, "%.2f");
    changed |= ImGui::InputFloat(u8"충격량 상한 (N s)", &settings.maximumImpulse, 50, 200, "%.1f");
    changed |= ImGui::InputFloat(u8"최소 접근 속도 (m/s)", &settings.minimumClosingSpeed, .05f, .1f, "%.2f");
    changed |= ImGui::SliderFloat(u8"골반 분배 비율", &settings.pelvisShare, 0, 1, "%.2f");
    if (ImGui::TreeNode(u8"접촉·재발동 설정"))
    {
        changed |= ImGui::InputFloat(u8"분리 유지 시간 (s)", &settings.separationTime, .05f, .1f, "%.2f");
        changed |= ImGui::InputFloat(u8"재발동 분리 거리 (m)", &settings.separationDistance, .01f, .05f, "%.2f");
        changed |= ImGui::InputFloat(u8"재발동 대기 (s)", &settings.cooldown, .05f, .1f, "%.2f");
        changed |= ImGui::InputFloat(u8"몸통 캡슐 반경 배율", &settings.capsuleRadiusScale, .05f, .1f, "%.2f");
        changed |= ImGui::InputFloat(u8"접촉 여유 (m)", &settings.contactSkin, .005f, .01f, "%.3f");
        ImGui::TreePop();
    }
    if (changed)
    {
        component.SetSettings(settings, m_BodyImpactMessage);
    }
    if (ImGui::Button(u8"밀침 설정 저장"))
    {
        if (component.GetSettings().Save(m_BodyImpactPath, m_BodyImpactMessage))
        {
            m_BodyImpactMessage = u8"밀침 설정을 저장했습니다.";
        }
    }
    ImGui::SameLine();
    if (ImGui::Button(u8"밀침 설정 불러오기"))
    {
        LoadBodyImpactSettings(*player, m_BodyImpactPath);
    }
    ImGui::TextWrapped("%s", u8"변경값은 다음 충돌부터 적용됩니다. 골반 외 나머지 충격량은 척추 바디에 나눕니다.");
    const PARS::CharacterImpactSnapshot& current = component.GetSnapshot();
    const PARS::CharacterImpactSnapshot& last = component.GetLastImpact();
    const char* reasons[] = {u8"접촉 없음", u8"최소 속도 미달", u8"접촉 유지 중", u8"재발동 대기",
        u8"적용 예약", u8"적용", u8"대상 상태 변경으로 취소", u8"비활성"};
    ImGui::Text(u8"현재: %s / 접근 %.2f m/s", reasons[static_cast<int>(current.reason)], current.closingSpeed);
    ImGui::Text(u8"발동 %llu회 / 최근 충격: 요구 %.1f → 적용 입력 %.1f N s%s",
        static_cast<unsigned long long>(last.eventCount), last.requestedImpulse, last.commandedImpulse,
        last.capped ? u8" (상한 도달)" : "");
    ImGui::Text(u8"접촉 바디: %s", last.hitBone.c_str());
    ImGui::Text(u8"방향: (%.2f, %.2f, %.2f)", last.direction.x, last.direction.y, last.direction.z);
    if (ImGui::TreeNode(u8"최근 바디별 입력"))
    {
        for (const PARS::BodyImpactShare& share : last.shares)
        {
            ImGui::Text("%s: %.2f N s", share.bone.c_str(), share.impulse.magnitude());
        }
        ImGui::TextWrapped("%s", u8"표시량은 솔버에 전달한 외부 입력입니다. 실제 운동량 변화는 접촉과 관절 제어에도 영향을 받습니다.");
        ImGui::TreePop();
    }
    if (!m_BodyImpactMessage.empty())
    {
        ImGui::TextWrapped("%s", m_BodyImpactMessage.c_str());
    }
    ImGui::Separator();
}
