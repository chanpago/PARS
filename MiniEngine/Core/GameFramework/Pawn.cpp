#include "Pawn.h"
#include "Controller.h"
#include "World.h"

namespace PARS
{
    Controller* Pawn::GetController() const
    {
        return GetWorld() ? GetWorld()->FindActor<Controller>(m_Controller) : nullptr;
    }
    void Pawn::EndPlay()
    {
        if (auto controller = GetController())
            if (controller->m_Pawn == GetHandle()) controller->UnPossess();
        m_Controller = {};
        ConsumeMovementInput();
        m_RunRequested = false;
        Actor::EndPlay();
    }
}
