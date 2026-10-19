#pragma once

#include "ActorComponent.h"
#include "../VectorMath.h"
#include <vector>

namespace PARS
{
    enum class AttachmentRule
    {
        KeepRelative,
        KeepWorld
    };

    class SceneComponent : public ActorComponent
    {
    public:
        using ActorComponent::ActorComponent;
        ~SceneComponent() override;

        // 현재는 같은 Actor 안에서만 부착할 수 있다. 순환 부착과 루트의 부모 부착은 거부한다.
        bool AttachToComponent(SceneComponent& parent, AttachmentRule rule = AttachmentRule::KeepRelative);
        void DetachFromParent(AttachmentRule rule = AttachmentRule::KeepWorld);
        SceneComponent* GetAttachParent() const
        {
            return m_Parent;
        }
        const std::vector<SceneComponent*>& GetAttachChildren() const
        {
            return m_Children;
        }

        const Math::AffineTransform& GetRelativeTransform() const
        {
            return m_RelativeTransform;
        }
        void SetRelativeTransform(const Math::AffineTransform& transform)
        {
            m_RelativeTransform = transform;
        }
        void SetRelativeLocation(Math::Vector3 location)
        {
            m_RelativeTransform.SetTranslation(location);
        }
        Math::AffineTransform GetComponentTransform() const;
        bool SetWorldTransform(const Math::AffineTransform& transform);

    private:
        SceneComponent* m_Parent = nullptr;
        std::vector<SceneComponent*> m_Children;
        Math::AffineTransform m_RelativeTransform{Math::kIdentity};
    };
}
