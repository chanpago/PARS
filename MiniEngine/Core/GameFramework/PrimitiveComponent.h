#pragma once

#include "SceneComponent.h"
#include "../Math/BoundingSphere.h"

namespace PARS
{
    // 형상을 가진 공간 컴포넌트다. 물리 등록은 별도로 처리한다.
    class PrimitiveComponent : public SceneComponent
    {
    public:
        using SceneComponent::SceneComponent;
        void SetVisibility(bool visible)
        {
            m_Visible = visible;
        }
        bool IsVisible() const
        {
            return m_Visible;
        }
        virtual Math::BoundingSphere GetLocalBounds() const = 0;

    private:
        bool m_Visible = true;
    };
}
