#pragma once

#include "FootstepCollision.h"

namespace PARS
{
    enum class FootstepAvoidanceStatus
    {
        Inactive,
        Direct,
        Detouring,
        Failed
    };

    struct FootstepAvoidanceSnapshot
    {
    public:
        uint64_t revision = 0;
        std::array<uint64_t, 2> startedSteps{};
        FootstepCollisionSnapshot collision;
        std::array<FootstepPath, 2> paths;
        std::array<int, 2> directions{};
        FootstepAvoidanceStatus status = FootstepAvoidanceStatus::Inactive;
        uint32_t candidatesChecked = 0;
        uint32_t totalIntervals = 0;
        uint32_t totalQueries = 0;
        bool searchBudgetExhausted = false;
    };

    class FootstepAvoidance
    {
    public:
        void Reset();
        void Update(FootstepCollisionInput input, const physx::PxVec3& characterForward);

    public:
        const FootstepAvoidanceSnapshot& GetSnapshot() const
        {
            return m_Snapshot;
        }

    private:
        FootstepAvoidanceSnapshot m_Snapshot;
    };
}
