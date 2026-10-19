#pragma once
#include "Actor.h"

namespace PARS
{
    class Pawn;
    class Controller : public Actor
    {
    public:
        explicit Controller(const std::string& name = "Controller")
            : Actor(name)
        {
            SetTickGroup(TickGroup::PrePhysics);
        }
        bool Possess(Pawn& pawn);
        void UnPossess();
        Pawn* GetPawn() const;
        void EndPlay() override;

    private:
        friend class Pawn;
        ActorHandle m_Pawn;
    };
}
