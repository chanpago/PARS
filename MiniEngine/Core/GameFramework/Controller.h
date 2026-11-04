#pragma once
#include "Actor.h"

namespace PARS
{
    class Pawn;
    class Controller : public Actor
    {
    private:
        friend class Pawn;

    public:
        explicit Controller(const std::string& name = "Controller")
            : Actor(name)
        {
            SetTickGroup(TickGroup::PrePhysics);
        }

    public:
        bool Possess(Pawn& pawn);
        void UnPossess();
        void EndPlay() override;

    public:
        Pawn* GetPawn() const;

    private:
        ActorHandle m_Pawn;
    };
}
