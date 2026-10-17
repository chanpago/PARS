#pragma once
#include "Animation.h"
#include "PhysicsAsset.h"

namespace PARS
{
    // Mesh node per joint, matching DefaultVS: meshWorld * skinJoint * IBM.
    // -1 marks a node outside the skin. Parents are in depth-first order.
    struct PhysicsSkeleton
    {
        std::vector<std::string> names;
        std::vector<int32_t> parents, meshNodes;
    };
    bool EvaluatePhysicsPose(const PhysicsSkeleton& skeleton, const std::vector<GraphNode>& pose,
        const Math::Matrix4& component, std::vector<Math::Matrix4>& world, std::string& error);
    // Physical targets include the original bone stretch. Unsimulated bones
    // keep their animation-local transforms and follow their simulated parents.
    bool OverridePhysicsPose(const PhysicsSkeleton& skeleton, const std::vector<GraphNode>& source,
        const Math::Matrix4& component, const std::vector<int>& nodes,
        const std::vector<Math::Matrix4>& targets, std::vector<GraphNode>& result);
    // Full major-bone preset (20 bodies / 19 constraints): fingers, toes and
    // terminal end markers are excluded. Angular limits are editable defaults.
    bool BuildMixamoPhysicsAsset(const PhysicsSkeleton& skeleton, const std::vector<GraphNode>& reference,
        PhysicsAsset& result, std::string& error);
}
