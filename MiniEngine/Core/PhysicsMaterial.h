#pragma once

#include <cmath>

namespace PARS
{
    enum class PhysicsFrictionCombine
    {
        Average,
        Minimum,
        Multiply,
        Maximum
    };

    struct PhysicsMaterialSettings
    {
        float staticFriction = .6f;
        float dynamicFriction = .5f;
        float restitution = .1f;
        PhysicsFrictionCombine frictionCombine = PhysicsFrictionCombine::Average;

        bool IsValid() const
        {
            return std::isfinite(staticFriction) && staticFriction >= 0 &&
                std::isfinite(dynamicFriction) && dynamicFriction >= 0 &&
                std::isfinite(restitution) && restitution >= 0 && restitution <= 1 &&
                frictionCombine >= PhysicsFrictionCombine::Average && frictionCombine <= PhysicsFrictionCombine::Maximum;
        }
    };
}
