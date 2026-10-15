#include "SceneComponent.h"
#include "Actor.h"
#include <algorithm>
#include <cmath>

using namespace Math;

namespace
{
    bool MakeRelative(const AffineTransform& world, const AffineTransform& parent, AffineTransform& relative)
    {
        const float determinant = float(Dot(parent.GetX(), Cross(parent.GetY(), parent.GetZ())));
        if (!std::isfinite(determinant) || std::abs(determinant) < 1e-8f)
            return false;
        relative = AffineTransform(static_cast<DirectX::XMMATRIX>(Invert(Matrix4(parent)) * Matrix4(world)));
        return true;
    }
}

namespace PARS
{
    SceneComponent::~SceneComponent()
    {
        while (!m_Children.empty())
            m_Children.back()->DetachFromParent();
        DetachFromParent();
    }

    AffineTransform SceneComponent::GetComponentTransform() const
    {
        return m_Parent ? m_Parent->GetComponentTransform() * m_RelativeTransform : m_RelativeTransform;
    }

    bool SceneComponent::SetWorldTransform(const AffineTransform& transform)
    {
        if (!m_Parent)
        {
            m_RelativeTransform = transform;
            return true;
        }
        return MakeRelative(transform, m_Parent->GetComponentTransform(), m_RelativeTransform);
    }

    bool SceneComponent::AttachToComponent(SceneComponent& parent, AttachmentRule rule)
    {
        if (&parent.GetOwner() != &GetOwner() || !GetOwner().OwnsComponent(*this) ||
            !GetOwner().OwnsComponent(parent) || GetOwner().GetRootComponent() == this)
            return false;
        for (SceneComponent* ancestor = &parent; ancestor; ancestor = ancestor->m_Parent)
            if (ancestor == this)
                return false;
        if (m_Parent == &parent)
            return true;

        AffineTransform relative = m_RelativeTransform;
        if (rule == AttachmentRule::KeepWorld &&
            !MakeRelative(GetComponentTransform(), parent.GetComponentTransform(), relative))
            return false;
        DetachFromParent(AttachmentRule::KeepRelative);
        m_Parent = &parent;
        m_Parent->m_Children.push_back(this);
        m_RelativeTransform = relative;
        return true;
    }

    void SceneComponent::DetachFromParent(AttachmentRule rule)
    {
        if (!m_Parent)
            return;
        const AffineTransform world = GetComponentTransform();
        auto& siblings = m_Parent->m_Children;
        siblings.erase(std::remove(siblings.begin(), siblings.end(), this), siblings.end());
        m_Parent = nullptr;
        if (rule == AttachmentRule::KeepWorld)
            m_RelativeTransform = world;
    }
}
