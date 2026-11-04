#pragma once
#include "MovementComponent.h"
#include "Pawn.h"

namespace PARS
{
    class PawnMovementComponent : public MovementComponent
    {
    public:
        using MovementComponent::MovementComponent;

    public:
        Pawn* GetPawnOwner() const
        {
            return dynamic_cast<Pawn*>(&GetOwner());
        }
    };
}
