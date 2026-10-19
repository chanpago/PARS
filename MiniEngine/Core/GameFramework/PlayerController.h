#pragma once
#include "Controller.h"

namespace PARS
{
    // 플랫폼 입력은 애플리케이션에서 전달한다. AI는 별도의 Controller를 사용할 수 있다.
    class PlayerController : public Controller
    {
    public:
        using Controller::Controller;
        void SetMoveInput(float forward, float right, bool run);
        void SetControlYaw(float radians);
        void SetInputEnabled(bool enabled);
        bool IsInputEnabled() const
        {
            return m_InputEnabled;
        }
        void Tick(float deltaTime) override;

    private:
        float m_Forward = 0, m_Right = 0, m_Yaw = 0;
        bool m_Run = false, m_InputEnabled = true;
    };
}
