#include "PhysicsPose.h"
#include <algorithm>
#include <cmath>

using namespace Math;
namespace PARS
{
    namespace
    {
        Matrix4 Local(const GraphNode& node)
        {
            Matrix4 result = node.xform;
            if (node.staleMatrix) result.Set3x3(Matrix3(node.rotation) * Matrix3::MakeScale(node.scale));
            return result;
        }
        bool SceneTransforms(const PhysicsSkeleton& skeleton, const std::vector<GraphNode>& pose,
            const Matrix4& component, std::vector<Matrix4>& scene)
        {
            const size_t count = pose.size();
            if (!count || skeleton.parents.size() != count || skeleton.names.size() != count || skeleton.meshNodes.size() != count) return false;
            scene.resize(count);
            for (size_t i = 0; i < count; ++i)
            {
                const int parent = skeleton.parents[i], mesh = skeleton.meshNodes[i];
                if (parent < -1 || parent >= static_cast<int>(i) || mesh < -1 || mesh >= static_cast<int>(count)) return false;
                scene[i] = pose[i].skeletonRoot ? Local(pose[i]) : (parent < 0 ? component : scene[parent]) * Local(pose[i]);
            }
            return true;
        }
    }

    bool EvaluatePhysicsPose(const PhysicsSkeleton& skeleton, const std::vector<GraphNode>& pose,
        const Matrix4& component, std::vector<Matrix4>& world, std::string& error)
    {
        std::vector<Matrix4> scene;
        if (!SceneTransforms(skeleton, pose, component, scene)) { error = "Invalid physics skeleton hierarchy."; return false; }
        world = scene;
        for (size_t i = 0; i < scene.size(); ++i)
            if (skeleton.meshNodes[i] >= 0) world[i] = scene[skeleton.meshNodes[i]] * scene[i];
        error.clear();
        return true;
    }

    bool OverridePhysicsPose(const PhysicsSkeleton& skeleton, const std::vector<GraphNode>& source,
        const Matrix4& component, const std::vector<int>& nodes, const std::vector<Matrix4>& targets,
        std::vector<GraphNode>& result)
    {
        std::vector<Matrix4> scene;
        if (nodes.size() != targets.size() || !SceneTransforms(skeleton, source, component, scene)) return false;
        const auto sourceScene = scene;
        std::vector<int> bodyAtNode(source.size(), -1);
        for (size_t i = 0; i < nodes.size(); ++i)
        {
            if (nodes[i] < 0 || nodes[i] >= static_cast<int>(source.size()) || skeleton.meshNodes[nodes[i]] < 0) return false;
            bodyAtNode[nodes[i]] = static_cast<int>(i);
        }
        result = source;
        for (size_t i = 0; i < result.size(); ++i)
        {
            const int parent = skeleton.parents[i], mesh = skeleton.meshNodes[i];
            const Matrix4 parentWorld = parent < 0 ? component : scene[parent];
            if (bodyAtNode[i] >= 0)
            {
                // Skeleton roots deliberately skip their scene parent in Model.cpp.
                const Matrix4 desired = Invert(sourceScene[mesh]) * targets[bodyAtNode[i]];
                result[i].xform = result[i].skeletonRoot ? desired : Invert(parentWorld) * desired;
                result[i].staleMatrix = false; // Preserve exact scale/shear, including centimetre imports.
            }
            scene[i] = result[i].skeletonRoot ? Local(result[i]) : parentWorld * Local(result[i]);
        }
        return true;
    }

    bool BuildMixamoPhysicsAsset(const PhysicsSkeleton& skeleton, const std::vector<GraphNode>& reference,
        PhysicsAsset& result, std::string& error)
    {
        std::vector<Matrix4> world;
        if (!EvaluatePhysicsPose(skeleton, reference, Matrix4(kIdentity), world, error)) return false;
        const auto find = [&](const std::string& suffix) {
            for (size_t i = 0; i < skeleton.names.size(); ++i)
            {
                const auto& name = skeleton.names[i]; const auto colon = name.find_last_of(':');
                if (name.substr(colon == std::string::npos ? 0 : colon + 1) == suffix && skeleton.meshNodes[i] >= 0) return static_cast<int>(i);
            }
            return -1;
        };
        struct Part { const char* bone; const char* end; float radius, mass, swing; PhysicsShapeType shape; };
        // All articulated Mixamo bones except fingers, toes and terminal end
        // markers. Keep spine/neck/clavicle bodies so joints follow the actual
        // hierarchy instead of attaching the head and upper arms to Spine2.
        // Split the former 16 kg torso across the spine, neck and shoulders.
        const Part parts[] = {
            {"Hips", "Spine", .13f, 12, 30, PhysicsShapeType::Box},
            {"Spine", "Spine1", .115f, 4, 30, PhysicsShapeType::Box},
            {"Spine1", "Spine2", .125f, 4, 30, PhysicsShapeType::Box},
            {"Spine2", "Neck", .13f, 5, 30, PhysicsShapeType::Box},
            {"Neck", "Head", .04f, 1, 30, PhysicsShapeType::Capsule},
            {"Head", "HeadTop_End", .09f, 5, 45, PhysicsShapeType::Capsule},
            {"LeftShoulder", "LeftArm", .035f, 1, 45, PhysicsShapeType::Capsule},
            {"LeftArm", "LeftForeArm", .05f, 2.5f, 110, PhysicsShapeType::Capsule},
            {"LeftForeArm", "LeftHand", .04f, 1.5f, 100, PhysicsShapeType::Capsule},
            {"LeftHand", "LeftHandMiddle1", .035f, .5f, 40, PhysicsShapeType::Capsule},
            {"RightShoulder", "RightArm", .035f, 1, 45, PhysicsShapeType::Capsule},
            {"RightArm", "RightForeArm", .05f, 2.5f, 110, PhysicsShapeType::Capsule},
            {"RightForeArm", "RightHand", .04f, 1.5f, 100, PhysicsShapeType::Capsule},
            {"RightHand", "RightHandMiddle1", .035f, .5f, 40, PhysicsShapeType::Capsule},
            {"LeftUpLeg", "LeftLeg", .075f, 6, 65, PhysicsShapeType::Capsule},
            {"LeftLeg", "LeftFoot", .055f, 3, 100, PhysicsShapeType::Capsule},
            {"LeftFoot", "LeftToeBase", .045f, 1, 35, PhysicsShapeType::Capsule},
            {"RightUpLeg", "RightLeg", .075f, 6, 65, PhysicsShapeType::Capsule},
            {"RightLeg", "RightFoot", .055f, 3, 100, PhysicsShapeType::Capsule},
            {"RightFoot", "RightToeBase", .045f, 1, 35, PhysicsShapeType::Capsule}
        };
        PhysicsAsset candidate;
        std::vector<int> nodes;
        for (const auto& part : parts)
        {
            const int node = find(part.bone), end = find(part.end);
            if (node < 0 || end < 0) { error = "Mixamo PA preset cannot find bone: " + std::string(node < 0 ? part.bone : part.end); return false; }
            const Vector3 endpoint(Invert(PhysicsBoneFrame(world[node])) * Vector3(world[end].GetW()));
            const float length = Length(endpoint);
            if (!std::isfinite(length) || length < .005f) { error = "Invalid Mixamo bone length."; return false; }
            const Vector3 axis = endpoint / length;
            const float dot = Clamp(float(axis.GetX()), -1.0f, 1.0f);
            Quaternion rotation(kIdentity);
            if (dot < -.9999f) rotation = Quaternion(Vector3(kYUnitVector), XM_PI);
            else if (dot < .9999f) rotation = Quaternion(Normalize(Cross(Vector3(kXUnitVector), axis)), std::acos(dot));
            PhysicsBody body;
            body.bone = skeleton.names[node]; body.shape = part.shape; body.mass = part.mass;
            body.radius = part.radius; body.halfLength = Max(.0f, length * .5f - body.radius);
            // Torso boxes cover their own bone segment. A fixed minimum of
            // 7 cm per half-length would overlap the added short spine bodies.
            body.halfExtents = XMFLOAT3(length * .45f, part.radius, part.radius * .7f);
            XMStoreFloat3(&body.offset, endpoint * .5f); XMStoreFloat4(&body.rotation, rotation);
            candidate.bodies.push_back(body); nodes.push_back(node);
        }
        for (size_t i = 1; i < nodes.size(); ++i)
        {
            int parent = skeleton.parents[nodes[i]], parentBody = -1;
            while (parent >= 0)
            {
                parentBody = candidate.FindBody(skeleton.names[parent]);
                if (parentBody >= 0) break;
                parent = skeleton.parents[parent];
            }
            if (parentBody < 0) { error = "Mixamo PA preset has disconnected limbs."; return false; }
            PhysicsConstraint joint;
            joint.parentBone = candidate.bodies[parentBody].bone; joint.childBone = candidate.bodies[i].bone;
            joint.swingYLimit = joint.swingZLimit = XMConvertToRadians(parts[i].swing);
            const Matrix4 parentWorld = PhysicsBoneFrame(world[parent]) * candidate.bodies[parentBody].GetLocalTransform();
            const Matrix4 childWorld = PhysicsBoneFrame(world[nodes[i]]) * candidate.bodies[i].GetLocalTransform();
            Matrix4 pivot = childWorld; pivot.SetW(world[nodes[i]].GetW());
            joint.SetWorldFrame(parentWorld, childWorld, pivot);
            candidate.constraints.push_back(joint);
        }
        if (!candidate.Validate(error)) return false;
        result = std::move(candidate);
        return true;
    }
}
