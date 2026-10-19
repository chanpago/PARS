#include "WorldRenderer.h"
#include "../Core/GameFramework/World.h"
#include "Components/SkeletalMeshComponent.h"

namespace
{
    template<class Visitor>
    void ForEachMesh(PARS::World& world, Visitor&& visitor)
    {
        world.ForEachActor([&](PARS::Actor& actor) {
            if (!actor.HasBegunPlay())
            {
                return;
            }
            for (const auto& component : actor.GetComponents())
            {
                if (auto mesh = dynamic_cast<PARS::SkeletalMeshComponent*>(component.get()))
                {
                    if (mesh->GetSkeletalMesh())
                    {
                        visitor(*mesh);
                    }
                }
            }
        });
    }
}

namespace Renderer
{
    bool HasWorldMeshes(PARS::World& world)
    {
        bool found = false;
        ForEachMesh(world, [&](PARS::SkeletalMeshComponent& mesh) {
            found |= mesh.IsVisible();
        });
        return found;
    }

    void UpdateWorldRenderData(PARS::World& world, GraphicsContext& context)
    {
        ForEachMesh(world, [&](PARS::SkeletalMeshComponent& mesh) {
            mesh.UpdateRenderData(context);
        });
    }

    void AddWorldMeshes(PARS::World& world, MeshSorter& sorter)
    {
        ForEachMesh(world, [&](PARS::SkeletalMeshComponent& mesh) {
            mesh.Render(sorter);
        });
    }
}
