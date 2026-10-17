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

        // X is the segment's longitudinal/twist axis; Y is its flexion axis.
        // Build a right-handed frame from landmarks, not the importer's bone axes.
        Matrix3 AnatomicalAxes(Vector3 x, Vector3 y)
        {
            x = Normalize(x);
            y = Normalize(y - x * Dot(x, y));
            return Matrix3(x, y, Cross(x, y));
        }

        struct AnatomicalCone
        {
            // Centre is a pure swing rotation vector (degrees), NOT Euler angles.
            float centerY, centerZ, halfY, halfZ;
        };

        void SetAnatomicalFrames(PhysicsConstraint& joint, const Matrix4& parentBody,
            const Matrix4& childBody, Vector3 pivot, const Matrix3& parentNeutral,
            const Matrix3& childNeutral, AnatomicalCone cone, float twistMin, float twistMax)
        {
            const Vector3 bias(0, XMConvertToRadians(cone.centerY), XMConvertToRadians(cone.centerZ));
            const float angle = Length(bias);
            const Quaternion rotation = angle > 1e-6f ? Quaternion(bias / angle, angle) : Quaternion(kIdentity);
            // A symmetric PhysX cone can have asymmetric anatomical endpoints:
            // rotate ONLY its parent frame toward the middle of the allowed ROM.
            // Do not align the child frame with that bias, which would cancel it.
            joint.parentFrame = PhysicsJointFrame::FromTransform(
                Invert(parentBody) * Matrix4(parentNeutral * Matrix3(rotation), pivot));
            joint.childFrame = PhysicsJointFrame::FromTransform(Invert(childBody) * Matrix4(childNeutral, pivot));
            joint.swingYLimit = XMConvertToRadians(cone.halfY);
            joint.swingZLimit = XMConvertToRadians(cone.halfZ);
            joint.twistMin = XMConvertToRadians(twistMin);
            joint.twistMax = XMConvertToRadians(twistMax);
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
        // de Leva (1996), Table 4, male sample: 73 kg; each limb fraction is
        // for ONE side. DOI: https://doi.org/10.1016/0021-9290(95)00178-6
        // Head/neck and trunk/shoulder grouping follows Karatsidis et al. (2017),
        // Table 1: https://doi.org/10.3390/s17010075
        // This is a young-adult male reference, not a Korean/world population mean.
        constexpr float totalMassKg = 73.0f;
        // Roos et al. (2020), Table 3: male model head 4.20 kg, neck 1.66 kg.
        // https://doi.org/10.1371/journal.pone.0219954
        // Borrow only their within-group ratio, retaining de Leva's 6.94% total.
        // This cross-study mapping is an approximation; a small collision capsule
        // is not a reliable estimate of the whole neck's tissue volume.
        constexpr float neckShare = 1.66f / (4.20f + 1.66f);
        enum MassGroup
        {
            Pelvis, MiddleTrunk, UpperTrunk, Neck, Head,
            LeftUpperArm, LeftForearm, LeftHand, RightUpperArm, RightForearm, RightHand,
            LeftThigh, LeftShank, LeftFoot, RightThigh, RightShank, RightFoot, MassGroupCount
        };
        constexpr float massFractions[MassGroupCount] = {
            .1117f, .1633f, .1596f, .0694f * neckShare, .0694f * (1.0f - neckShare),
            .0271f, .0162f, .0061f, .0271f, .0162f, .0061f,
            .1416f, .0433f, .0137f, .1416f, .0433f, .0137f
        };
        struct Part { const char* bone; const char* end; float radius; MassGroup massGroup; PhysicsShapeType shape; };
        // All articulated Mixamo bones except fingers, toes and terminal end
        // markers. Keep spine/neck/clavicle bodies so joints follow the actual
        // hierarchy instead of attaching the head and upper arms to Spine2.
        // Fingers/toes have no separate bodies; their mass stays in hands/feet.
        const Part parts[] = {
            {"Hips", "Spine", .13f, Pelvis, PhysicsShapeType::Box},
            {"Spine", "Spine1", .115f, MiddleTrunk, PhysicsShapeType::Box},
            {"Spine1", "Spine2", .125f, MiddleTrunk, PhysicsShapeType::Box},
            {"Spine2", "Neck", .13f, UpperTrunk, PhysicsShapeType::Box},
            {"Neck", "Head", .04f, Neck, PhysicsShapeType::Capsule},
            {"Head", "HeadTop_End", .09f, Head, PhysicsShapeType::Capsule},
            {"LeftShoulder", "LeftArm", .035f, UpperTrunk, PhysicsShapeType::Capsule},
            {"LeftArm", "LeftForeArm", .05f, LeftUpperArm, PhysicsShapeType::Capsule},
            {"LeftForeArm", "LeftHand", .04f, LeftForearm, PhysicsShapeType::Capsule},
            {"LeftHand", "LeftHandMiddle1", .035f, LeftHand, PhysicsShapeType::Capsule},
            {"RightShoulder", "RightArm", .035f, UpperTrunk, PhysicsShapeType::Capsule},
            {"RightArm", "RightForeArm", .05f, RightUpperArm, PhysicsShapeType::Capsule},
            {"RightForeArm", "RightHand", .04f, RightForearm, PhysicsShapeType::Capsule},
            {"RightHand", "RightHandMiddle1", .035f, RightHand, PhysicsShapeType::Capsule},
            {"LeftUpLeg", "LeftLeg", .075f, LeftThigh, PhysicsShapeType::Capsule},
            {"LeftLeg", "LeftFoot", .055f, LeftShank, PhysicsShapeType::Capsule},
            {"LeftFoot", "LeftToeBase", .045f, LeftFoot, PhysicsShapeType::Capsule},
            {"RightUpLeg", "RightLeg", .075f, RightThigh, PhysicsShapeType::Capsule},
            {"RightLeg", "RightFoot", .055f, RightShank, PhysicsShapeType::Capsule},
            {"RightFoot", "RightToeBase", .045f, RightFoot, PhysicsShapeType::Capsule}
        };
        PhysicsAsset candidate;
        std::vector<int> nodes;
        std::vector<double> bodyVolumes;
        double groupVolumes[MassGroupCount]{};
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
            body.bone = skeleton.names[node]; body.shape = part.shape;
            body.radius = part.radius; body.halfLength = Max(.0f, length * .5f - body.radius);
            // Torso boxes cover their own bone segment. A fixed minimum of
            // 7 cm per half-length would overlap the added short spine bodies.
            body.halfExtents = XMFLOAT3(length * .45f, part.radius, part.radius * .7f);
            XMStoreFloat3(&body.offset, endpoint * .5f); XMStoreFloat4(&body.rotation, rotation);
            const double radius = body.radius;
            const double volume = body.shape == PhysicsShapeType::Box
                ? 8.0 * body.halfExtents.x * body.halfExtents.y * body.halfExtents.z
                : XM_PI * radius * radius * (2.0 * body.halfLength + 4.0 * radius / 3.0);
            bodyVolumes.push_back(volume);
            groupVolumes[part.massGroup] += volume;
            candidate.bodies.push_back(body); nodes.push_back(node);
        }
        // de Leva has 16 segments; this rig has 20 bodies. Head/neck are split
        // above. Preserve each trunk group mass and split only within that group
        // by collider volume (uniform density). This extra trunk split and the
        // Mixamo anatomical boundaries are approximations, not measured bone ratios.
        // Shoulders belong to the upper trunk, so they add no extra body mass.
        // PhysX still derives COM/inertia from primitives, not the study's tensors.
        for (size_t i = 0; i < candidate.bodies.size(); ++i)
        {
            const MassGroup group = parts[i].massGroup;
            candidate.bodies[i].mass = static_cast<float>(
                totalMassKg * massFractions[group] * bodyVolumes[i] / groupVolumes[group]);
        }

        const auto position = [&](int node) { return Vector3(world[node].GetW()); };
        const Vector3 left = Normalize(position(find("LeftUpLeg")) - position(find("RightUpLeg")));
        const Vector3 upHint = position(find("Neck")) - position(find("Hips"));
        if (float(LengthSquare(Cross(left, upHint))) < 1e-8f)
        { error = "Mixamo PA preset needs an upright reference skeleton."; return false; }
        const Vector3 up = Normalize(upHint - left * Dot(left, upHint));
        const Vector3 forward = Normalize(Cross(left, up));
        std::vector<Vector3> longitudinal;
        for (size_t i = 0; i < nodes.size(); ++i)
            longitudinal.push_back(Normalize(position(find(parts[i].end)) - position(nodes[i])));
        Vector3 radial[2];
        for (int side = 0; side < 2; ++side)
        {
            const std::string prefix = side == 0 ? "Left" : "Right";
            const int index = find(prefix + "HandIndex1"), pinky = find(prefix + "HandPinky1");
            if (index < 0 || pinky < 0 || float(LengthSquare(position(index) - position(pinky))) < 1e-8f)
            { error = "Mixamo PA preset needs hand landmarks to calibrate forearm rotation."; return false; }
            radial[side] = Normalize(position(index) - position(pinky));
            // The preset is calibrated from the imported T-pose, never a sampled
            // animation frame. Reject degenerate flexion axes instead of making NaNs.
            for (const char* suffix : {"Arm", "ForeArm", "Hand"})
            {
                const int body = candidate.FindBody(skeleton.names[find(prefix + suffix)]);
                if (std::abs(float(Dot(longitudinal[body], forward))) > .95f ||
                    std::abs(float(Dot(longitudinal[body], radial[side]))) > .95f)
                { error = "Mixamo PA preset needs an arms-out reference pose."; return false; }
            }
        }

        // ROM sources (degrees):
        // CDC/Soucie 2011, passive means, males 20-44: elbow -0.8..144.6,
        // pronation 76.9 / supination 85, knee -1..137.7, hip -17.4..130.4,
        // ankle plantarflexion 54.6 / dorsiflexion 12.7.
        // https://archive.cdc.gov/www_cdc_gov/ncbddd/jointrom/index.html
        // Moromizato 2016, Table 1, passive male means (42 men aged 20-29):
        // wrist extension/flexion 79/87; trunk extension/flexion 28.2/37.3,
        // lateral bend 24.8, rotation 52.2; hip ab/adduction 32.1/13.7,
        // external/internal rotation 46.7/37.1; shoulder horizontal extension/
        // flexion 54.3/130.7, abduction 179.8, external/internal rotation 91.5/58.4.
        // https://doi.org/10.1186/s40101-016-0112-8
        // Reynolds 2009, Table 5, mixed-sex ACTIVE whole-neck totals:
        // flexion+extension 125, lateral bending 79.8, rotation 131.
        // https://doi.org/10.1007/s00586-009-0894-z
        // These are editable reference envelopes, not universal hard stops.
        // Fixed cones cannot reproduce posture-dependent coupled human limits.
        //
        // Two-axis cone coefficients below fit the four pure-plane endpoints in
        // this SDK's ConeLimitHelperTanLess angle space:
        // (swingY/halfY)^2 + (swingZ/halfZ)^2 = 1, with swingY/Z =
        // 4*atan2(q.y/z, 1+q.w) after removing twist. The biased parent frame is a pure swing so
        // neutral has zero twist. Simply using half(range) on both axes would
        // incorrectly shrink lateral ROM when flexion has a large bias.
        // Mirroring centerZ and twist signs preserves left/right anatomy.
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
            const Matrix4 parentWorld = PhysicsBoneFrame(world[parent]) * candidate.bodies[parentBody].GetLocalTransform();
            const Matrix4 childWorld = PhysicsBoneFrame(world[nodes[i]]) * candidate.bodies[i].GetLocalTransform();
            const std::string name(parts[i].bone);
            const bool isLeft = name.find("Left") == 0;
            const float side = isLeft ? 1.0f : -1.0f;
            const Vector3 x = longitudinal[i], parentX = longitudinal[parentBody];
            Matrix3 parentAxes = AnatomicalAxes(up, left), childAxes = parentAxes;
            AnatomicalCone cone{};
            float twistMin = -1, twistMax = 1; // Ignored on locked axes.
            if (name.find("Spine") == 0)
            {
                // Three rig joints share the WHOLE trunk budget equally. The
                // allocation is a rig approximation, not per-vertebra ROM data.
                cone = {1.516667f, 0, 10.916667f, 8.347644f};
                twistMax = 52.2f / 3; twistMin = -twistMax;
            }
            else if (name == "Neck" || name == "Head")
            {
                // Conservative active envelope: choose 55 flexion / 70 extension
                // within the measured 125 total; split 60:40 across the rig's
                // Neck/Head joints. Directional split and rig allocation are
                // modelling choices; these are NOT two anatomical vertebrae.
                const bool neck = name == "Neck";
                cone = neck ? AnatomicalCone{-4.5f, 0, 37.5f, 24.121613f}
                            : AnatomicalCone{-3, 0, 25, 16.078565f};
                twistMax = 65.5f * (neck ? .6f : .4f); twistMin = -twistMax;
            }
            else if (name.find("Shoulder") != std::string::npos)
            {
                // Shoulder bone proxies the girdle (no scapula body in this rig).
                // Allocate horizontal -10..20 and elevation -10..30 to it;
                // subtract those budgets from the humerus below. Approximation.
                parentAxes = childAxes = AnatomicalAxes(left * side, up * -side);
                cone = {4.973932f, -9.956774f * side, 17.052221f, 20.855535f};
                // Small allowance for swing/twist coupling in the biased frame.
                twistMin = -3; twistMax = 3;
            }
            else if (name == "LeftArm" || name == "RightArm")
            {
                parentAxes = AnatomicalAxes(left * side, up * -side);
                childAxes = AnatomicalAxes(x, Cross(x, forward));
                // Humerus horizontal -44.3..110.7; lower 90 / raise 59.8
                // from T-pose. Together with the girdle this approximates the
                // measured total shoulder envelope, without double-counting it.
                cone = {32.595316f, 13.336539f * side, 77.849753f, 77.409744f};
                twistMin = isLeft ? -91.5f : -58.4f;
                twistMax = isLeft ? 58.4f : 91.5f;
            }
            else if (name.find("ForeArm") != std::string::npos)
            {
                parentAxes = AnatomicalAxes(parentX, Cross(parentX, forward));
                // Thumb-side hand landmarks define neutral pronation/supination.
                // The Mixamo palm-down T-pose is already pronated; never declare
                // that bind rotation to be zero and add another 85 degrees to it.
                childAxes = AnatomicalAxes(x, radial[isLeft ? 0 : 1] * -side);
                cone = {71.9f, 0, 72.7f, 1}; // -0.8 extension .. 144.6 flexion.
                joint.swingZ = PhysicsJointMotion::Locked; // No elbow side-bending.
                twistMin = isLeft ? -85.0f : -76.9f;
                twistMax = isLeft ? 76.9f : 85.0f;
            }
            else if (name.find("Hand") != std::string::npos)
            {
                const Vector3 hinge = radial[isLeft ? 0 : 1] * -side;
                parentAxes = AnatomicalAxes(parentX, hinge);
                childAxes = AnatomicalAxes(x, hinge);
                // -79 extension .. 87 flexion. Radial/ulnar 20/30 are conservative
                // rig caps, not values measured by the above wrist study.
                cone = {3.959488f, 5.002095f * side, 83.083931f, 25.045937f};
                // Forearm carries pronation/supination. This small allowance is
                // for the biased cone's coordinate coupling, not another +/-85.
                twistMin = -5; twistMax = 5;
            }
            else if (name.find("UpLeg") != std::string::npos)
            {
                parentAxes = AnatomicalAxes(-up, -left);
                childAxes = AnatomicalAxes(x, -left);
                cone = {55.314192f, -9.524431f * side, 74.981983f, 37.496596f};
                twistMin = isLeft ? -46.7f : -37.1f;
                twistMax = isLeft ? 37.1f : 46.7f;
            }
            else if (name.find("Leg") != std::string::npos)
            {
                parentAxes = AnatomicalAxes(parentX, left);
                childAxes = AnatomicalAxes(x, left);
                cone = {68.35f, 0, 69.35f, 1}; // -1 hyperextension .. 137.7 flexion.
                // Hinge approximation. Knee laxity/axial rotation depends strongly
                // on flexion; do not give a fully extended knee a free twist cone.
                joint.twist = joint.swingZ = PhysicsJointMotion::Locked;
            }
            else // Foot: use a virtual downward axis, not ankle-to-toe capsule X.
            {
                parentAxes = AnatomicalAxes(parentX, -left);
                childAxes = AnatomicalAxes(-up, -left);
                // Sagittal -54.6 plantar .. 12.7 dorsi. Inversion/eversion 30/15
                // are rig caps for the combined ankle/subtalar proxy, not CDC data.
                cone = {-20.754667f, 7.333241f * side, 34.654347f, 27.971366f};
                twistMin = -6; twistMax = 6; // Biased-frame coupling, not free foot yaw.
            }
            SetAnatomicalFrames(joint, parentWorld, childWorld, position(nodes[i]),
                parentAxes, childAxes, cone, twistMin, twistMax);
            candidate.constraints.push_back(joint);
        }
        if (!candidate.Validate(error)) return false;
        result = std::move(candidate);
        return true;
    }
}
