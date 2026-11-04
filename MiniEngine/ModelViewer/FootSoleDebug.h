#pragma once

#include "PhysicsWorld.h"

namespace PARS
{
    class SkeletalMeshComponent;
}

// Draws the runtime snapshot. Visibility never controls physics sampling.
class FootSoleDebug
{
public:
    void DrawUI(PARS::SkeletalMeshComponent& mesh);
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
