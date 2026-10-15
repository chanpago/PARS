#include "Controller.h"
#include "Pawn.h"
#include "World.h"

namespace PARS
{
    Pawn* Controller::GetPawn() const
    {
        return GetWorld() ? GetWorld()->FindActor<Pawn>(m_Pawn) : nullptr;
    }

    bool Controller::Possess(Pawn& pawn)
    {
        if (!GetWorld() || pawn.GetWorld() != GetWorld() || pawn.IsPendingDestroy() || IsPendingDestroy())
            return false;
        if (GetPawn() == &pawn) return true;
        UnPossess();
        if (auto old = pawn.GetController()) old->UnPossess();
        m_Pawn = pawn.GetHandle();
        pawn.m_Controller = GetHandle();
        pawn.ConsumeMovementInput();
        pawn.SetRunRequested(false);
        return true;
    }

    void Controller::UnPossess()
    {
        if (auto pawn = GetPawn())
        {
            pawn->m_Controller = {};
            pawn->ConsumeMovementInput();
            pawn->SetRunRequested(false);
        }
        m_Pawn = {};
    }

    void Controller::EndPlay() { UnPossess(); Actor::EndPlay(); }
}
