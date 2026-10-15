#pragma once

#include "SkinnedMeshComponent.h"
#include "../AnimInstance.h"

namespace PARS
{
    class SkeletalMeshComponent : public SkinnedMeshComponent
    {
    public:
        SkeletalMeshComponent(Actor& owner, const std::string& name);
        // Assign assets before rendering starts (or while the GPU is idle).
        bool SetSkeletalMesh(std::shared_ptr<const Model> mesh) override;
        bool SetAnimInstance(std::unique_ptr<AnimInstance> instance);
        AnimInstance& GetAnimInstance() const { return *m_AnimInstance; }

        bool UpdateRenderData(GraphicsContext& context);
        void Render(Renderer::MeshSorter& sorter) const;
        const ModelInstance& GetModelInstance() const { return m_ModelInstance; }

    protected:
        void TickComponent(float deltaTime) override;

    private:
        ModelInstance m_ModelInstance;
        std::unique_ptr<AnimInstance> m_AnimInstance;
    };
}
