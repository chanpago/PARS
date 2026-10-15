#pragma once

#include "SceneComponent.h"
#include "../Math/BoundingSphere.h"

namespace PARS
{
    // Spatial component with geometry. Physics registration is a separate concern.
    class PrimitiveComponent : public SceneComponent
    {
    public:
        using SceneComponent::SceneComponent;
        void SetVisibility(bool visible) { m_Visible = visible; }
        bool IsVisible() const { return m_Visible; }
        virtual Math::BoundingSphere GetLocalBounds() const = 0;

    private:
        bool m_Visible = true;
    };
}
