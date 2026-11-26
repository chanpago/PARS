#pragma once

#include "PhysicsWorld.h"
#include <cstdint>
#include <functional>
#include <string>

struct ImVec2;

namespace PARS
{
    class SkeletalMeshComponent;
}

// 실행 중인 물리 정보를 표시하며 표시 여부는 물리 관측에 영향을 주지 않는다.
class FootSoleDebug
{
public:
    void DrawUI(PARS::SkeletalMeshComponent& mesh, const std::function<void()>& drawBufferSettings = {});
    void DrawContactOverlay(const std::string& text, bool touching, const ImVec2& sceneOrigin, const ImVec2& sceneSize) const;
    void BuildLines(const PARS::SkeletalMeshComponent& mesh, std::vector<physx::PxDebugLine>& lines) const;
    static void BuildLandingCandidateLines(const PARS::SkeletalMeshComponent& mesh, std::vector<physx::PxDebugLine>& lines);
    void BuildRegionTriangles(const PARS::SkeletalMeshComponent& mesh,
        std::vector<physx::PxDebugTriangle>& triangles) const;

public:
    uint32_t GetLegContacts(PARS::SkeletalMeshComponent* dummy, std::string& text) const;
    static std::string GetFollowedAnimationText(const PARS::SkeletalMeshComponent& mesh);
    std::string GetLegPredictionText(const PARS::SkeletalMeshComponent& mesh) const;
    void SetVisible(bool visible)
    {
        m_Visible = visible;
    }
    bool IsVisible() const
    {
        return m_Visible;
    }

private:
    bool m_Visible = true;
    bool m_ShowRays = true;
    bool m_ShowHull = true;
    bool m_ShowCenterOfMass = true;
    bool m_ShowBuffer = true;
    bool m_ShowFootstep = true;
    bool m_ShowLandingComClamp = true;
    bool m_ShowLandingFoot = true;
    bool m_ShowRecoveryRegions = true;
};
