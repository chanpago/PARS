#pragma once

#include "Actor.h"

namespace PARS
{
    class Controller;
    class Pawn : public Actor
    {
    public:
        using Actor::Actor;
        Controller* GetController() const;
        void EndPlay() override;
        void SetRunRequested(bool running)
        {
            m_RunRequested = running;
        }
        bool IsRunRequested() const
        {
            return m_RunRequested;
        }
        void AddMovementInput(Math::Vector3 worldDirection, float scale = 1.0f)
        {
            m_PendingMovementInput += worldDirection * scale;
        }
        Math::Vector3 ConsumeMovementInput()
        {
            const auto input = m_PendingMovementInput;
            m_PendingMovementInput = Math::Vector3(Math::kZero);
            return input;
        }

    private:
        friend class Controller;
        ActorHandle m_Controller;
        bool m_RunRequested = false;
        Math::Vector3 m_PendingMovementInput{Math::kZero};
    };
}
