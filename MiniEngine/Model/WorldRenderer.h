#pragma once

namespace PARS { class World; }
class GraphicsContext;

namespace Renderer
{
    class MeshSorter;
    bool HasWorldMeshes(PARS::World& world);
    void UpdateWorldRenderData(PARS::World& world, GraphicsContext& context);
    void AddWorldMeshes(PARS::World& world, MeshSorter& sorter);
}
