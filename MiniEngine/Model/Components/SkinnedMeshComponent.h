#pragma once

#include "MeshComponent.h"
#include "../Model.h"

namespace PARS
{
    class SkinnedMeshComponent : public MeshComponent
    {
    public:
        using MeshComponent::MeshComponent;
        const std::shared_ptr<const Model>& GetSkeletalMesh() const
        {
            return m_SkeletalMesh;
        }
        virtual bool SetSkeletalMesh(std::shared_ptr<const Model> mesh) = 0;
        Math::BoundingSphere GetLocalBounds() const override
        {
            return m_SkeletalMesh ? m_SkeletalMesh->m_BoundingSphere : Math::BoundingSphere(Math::kZero);
        }

    protected:
        std::shared_ptr<const Model> m_SkeletalMesh;
    };
}
