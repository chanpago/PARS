#pragma once

#include "Actor.h"

namespace PARS
{
    class Pawn : public Actor
    {
    public:
        using Actor::Actor;
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
        Math::Vector3 m_PendingMovementInput{ Math::kZero };
    };
}
