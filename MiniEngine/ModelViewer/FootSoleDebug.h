#pragma once

#include "PhysicsWorld.h"
#include <functional>

namespace PARS
{
    class SkeletalMeshComponent;
}

// 실행 중인 물리 정보를 표시하며 표시 여부는 물리 관측에 영향을 주지 않는다.
class FootSoleDebug
{
public:
    void DrawUI(PARS::SkeletalMeshComponent& mesh, const std::function<void()>& drawBufferSettings = {});
    void DrawContactOverlay(PARS::SkeletalMeshComponent* player,
        PARS::SkeletalMeshComponent* dummy, float topInset) const;
    void BuildLines(const PARS::SkeletalMeshComponent& mesh, std::vector<physx::PxDebugLine>& lines) const;

public:
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
    bool m_ShowLandingFoot = true;
};
