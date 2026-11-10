#pragma once

#include "FootstepPlanner.h"

namespace PARS
{
    // 고정 높이 역진자와 한 번의 후속 스텝으로 후보를 비교한다. 전신의 회복 보장은 아니다.
    FootstepTarget PlanFootstepRecovery(const FootstepInput& input, const FootstepSettings& settings, size_t side);
    float FootstepCaptureError(const FootstepInput& input);
}
