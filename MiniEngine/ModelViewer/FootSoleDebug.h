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
    void SetVisible(bool visible)
    {
        m_Visible = visible;
    }

private:
    bool m_Visible = true, m_ShowRays = true, m_ShowHull = true;
    bool m_ShowCenterOfMass = true, m_ShowBuffer = true;
};
