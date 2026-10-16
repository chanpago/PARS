#include "SkeletalMeshComponent.h"

namespace PARS
{
    SkeletalMeshComponent::SkeletalMeshComponent(Actor& owner, const std::string& name)
        : SkinnedMeshComponent(owner, name), m_AnimInstance(std::make_unique<AnimInstance>(*this))
    {
    }

    bool SkeletalMeshComponent::SetSkeletalMesh(std::shared_ptr<const Model> mesh)
    {
        if (mesh && (mesh->m_NumNodes == 0 || mesh->m_NumJoints == 0))
            return false;
        m_SkeletalMesh = std::move(mesh);
        m_ModelInstance = m_SkeletalMesh;
        m_AnimInstance->Initialize(m_SkeletalMesh);
        return true;
    }

    bool SkeletalMeshComponent::SetAnimInstance(std::unique_ptr<AnimInstance> instance)
    {
        if (!instance || &instance->GetOwningComponent() != this)
            return false;
        instance->Initialize(m_SkeletalMesh);
        m_AnimInstance = std::move(instance);
        return true;
    }

    void SkeletalMeshComponent::TickComponent(float deltaTime)
    {
        m_AnimInstance->UpdateAnimation(deltaTime);
    }

    bool SkeletalMeshComponent::UpdateRenderData(GraphicsContext& context, const std::vector<GraphNode>* poseOverride)
    {
        if (!m_SkeletalMesh)
            return false;
        m_ModelInstance.SetTransform(GetComponentTransform());
        return m_ModelInstance.UpdatePose(context, poseOverride ? *poseOverride : m_AnimInstance->GetPose());
    }

    void SkeletalMeshComponent::Render(Renderer::MeshSorter& sorter) const
    {
        if (IsVisible())
            m_ModelInstance.Render(sorter);
    }
}
