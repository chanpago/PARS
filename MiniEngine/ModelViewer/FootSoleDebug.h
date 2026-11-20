#pragma once

#include "PhysicsWorld.h"
#include <cstdint>
#include <functional>
#include <string>

namespace PARS
{
    class SkeletalMeshComponent;
}

// 실행 중인 물리 정보를 표시하며 표시 여부는 물리 관측에 영향을 주지 않는다.
class FootSoleDebug
{
public:
    void DrawUI(PARS::SkeletalMeshComponent& mesh, const std::function<void()>& drawBufferSettings = {});
    void DrawContactOverlay(const std::string& text, bool touching, float topInset) const;
    void BuildLines(const PARS::SkeletalMeshComponent& mesh, std::vector<physx::PxDebugLine>& lines) const;
    void BuildRegionTriangles(const PARS::SkeletalMeshComponent& mesh,
        std::vector<physx::PxDebugTriangle>& triangles) const;
    void BuildCrossRegionGeometry(const PARS::SkeletalMeshComponent& mesh, const physx::PxBounds3& ground,
        std::vector<physx::PxDebugLine>& lines, std::vector<physx::PxDebugTriangle>& triangles) const;

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
    bool m_ShowCrossRegion = true;
    bool m_ShowLandingFoot = true;
    bool m_ShowRecoveryRegions = true;
};
