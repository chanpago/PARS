#include "PlayerController.h"
#include "Pawn.h"
#include <cmath>

namespace PARS
{
    void PlayerController::SetMoveInput(float forward, float right, bool run)
    {
        m_Forward = std::isfinite(forward) ? Math::Clamp(forward, -1.0f, 1.0f) : 0;
        m_Right = std::isfinite(right) ? Math::Clamp(right, -1.0f, 1.0f) : 0;
        m_Run = run;
    }
    void PlayerController::SetControlYaw(float radians)
    { if (std::isfinite(radians)) m_Yaw = radians; }

    void PlayerController::SetInputEnabled(bool enabled)
    {
        m_InputEnabled = enabled;
        if (!enabled)
        {
            SetMoveInput(0, 0, false);
            if (auto pawn = GetPawn()) { pawn->ConsumeMovementInput(); pawn->SetRunRequested(false); }
        }
    }
    void PlayerController::Tick(float deltaTime)
    {
        Actor::Tick(deltaTime);
        if (!HasBegunPlay() || IsPendingDestroy())
        {
            return;
        }

        if (auto pawn = GetPawn())
        {
            pawn->SetRunRequested(m_InputEnabled && m_Run);
            if (m_InputEnabled)
            {
                const Math::Vector3 forward(std::sin(m_Yaw), 0, std::cos(m_Yaw));
                const Math::Vector3 right(-std::cos(m_Yaw), 0, std::sin(m_Yaw));
                pawn->AddMovementInput(forward, m_Forward);
                pawn->AddMovementInput(right, m_Right);
            }
        }
    }
}
