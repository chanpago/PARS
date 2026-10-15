#pragma once
#include "Controller.h"

namespace PARS
{
    // Platform input is supplied by the application. AI can use another Controller.
    class PlayerController : public Controller
    {
    public:
        using Controller::Controller;
        void SetMoveInput(float forward, float right, bool run);
        void SetControlYaw(float radians);
        void SetInputEnabled(bool enabled);
        bool IsInputEnabled() const { return m_InputEnabled; }
        void Tick(float deltaTime) override;
    private:
        float m_Forward = 0, m_Right = 0, m_Yaw = 0;
        bool m_Run = false, m_InputEnabled = true;
    };
}
