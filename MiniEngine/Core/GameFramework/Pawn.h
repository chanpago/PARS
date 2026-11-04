#pragma once

#include "Actor.h"

namespace PARS
{
    class Controller;
    class Pawn : public Actor
    {
    private:
        friend class Controller;

    public:
        using Actor::Actor;

    public:
        void EndPlay() override;
        void AddMovementInput(Math::Vector3 worldDirection, float scale = 1.0f)
        {
            m_PendingMovementInput += worldDirection * scale;
        }
        Math::Vector3 ConsumeMovementInput()
        {
            const Math::Vector3 input = m_PendingMovementInput;
            m_PendingMovementInput = Math::Vector3(Math::kZero);
            return input;
        }

    public:
        Controller* GetController() const;
        void SetRunRequested(bool running)
        {
            m_RunRequested = running;
        }
        bool IsRunRequested() const
        {
            return m_RunRequested;
        }

    private:
        Math::Vector3 m_PendingMovementInput{Math::kZero};
        ActorHandle m_Controller;
        bool m_RunRequested = false;
    };
}
