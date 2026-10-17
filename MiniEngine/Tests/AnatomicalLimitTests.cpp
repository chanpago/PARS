#include "../Model/PhysicsPose.h"
#include "../Model/Ragdoll.h"
#include "../Core/PhysicsWorld.h"
#include <cmath>
#include <cstdio>
#include <cstdlib>

using namespace Math;
using namespace PARS;
namespace
{
    void Check(bool value, const char* message)
    { if (!value) { std::fprintf(stderr, "FAIL: %s\n", message); std::exit(1); } }

    struct Fixture
    {
        PhysicsSkeleton skeleton;
        std::vector<GraphNode> pose;
        std::vector<Vector3> positions;
        int Add(const std::string& name, int parent, Vector3 position)
        {
            const int index = static_cast<int>(pose.size());
            skeleton.names.push_back(name); skeleton.parents.push_back(parent);
            skeleton.meshNodes.push_back(index == 0 ? -1 : 0);
            GraphNode node{};
            node.xform = Matrix4(Matrix3(kIdentity), position - (parent < 0 ? Vector3(kZero) : positions[parent]));
            node.rotation = Quaternion(kIdentity); node.scale = XMFLOAT3(1, 1, 1);
            pose.push_back(node); positions.push_back(position);
            return index;
        }
        Fixture()
        {
            Add("mesh", -1, Vector3(kZero));
            const int hips = Add("Hips", -1, Vector3(0, 1, 0));
            const int spine = Add("Spine", hips, Vector3(0, 1.10f, 0));
            const int spine1 = Add("Spine1", spine, Vector3(0, 1.23f, 0));
            const int spine2 = Add("Spine2", spine1, Vector3(0, 1.36f, 0));
            const int neck = Add("Neck", spine2, Vector3(0, 1.50f, 0));
            const int head = Add("Head", neck, Vector3(0, 1.57f, 0));
            Add("HeadTop_End", head, Vector3(0, 1.78f, 0));
            for (float side : {1.0f, -1.0f})
            {
                const std::string p = side > 0 ? "Left" : "Right";
                const int shoulder = Add(p + "Shoulder", spine2, Vector3(side * .06f, 1.46f, 0));
                const int arm = Add(p + "Arm", shoulder, Vector3(side * .19f, 1.44f, 0));
                const int forearm = Add(p + "ForeArm", arm, Vector3(side * .44f, 1.44f, 0));
                const int hand = Add(p + "Hand", forearm, Vector3(side * .68f, 1.44f, 0));
                Add(p + "HandMiddle1", hand, Vector3(side * .80f, 1.44f, 0));
                Add(p + "HandIndex1", hand, Vector3(side * .79f, 1.44f, .025f));
                Add(p + "HandPinky1", hand, Vector3(side * .79f, 1.44f, -.025f));
                const int thigh = Add(p + "UpLeg", hips, Vector3(side * .08f, .95f, 0));
                const int shin = Add(p + "Leg", thigh, Vector3(side * .08f, .52f, 0));
                const int foot = Add(p + "Foot", shin, Vector3(side * .08f, .1f, 0));
                Add(p + "ToeBase", foot, Vector3(side * .08f, .01f, .13f));
            }
        }
        int Find(const std::string& name) const
        {
            for (int i = 0; i < static_cast<int>(skeleton.names.size()); ++i)
                if (skeleton.names[i] == name) return i;
            return -1;
        }
    };
    bool Near(const Matrix4& a, const Matrix4& b)
    {
        return float(Length(Vector3(a.GetW()) - Vector3(b.GetW()))) < 1e-4f &&
            float(Length(Vector3(a.GetX()) - Vector3(b.GetX()))) < 1e-4f &&
            float(Length(Vector3(a.GetY()) - Vector3(b.GetY()))) < 1e-4f;
    }
}

void RunAnatomicalLimitTests()
{
    Fixture rig;
    PhysicsAsset asset;
    std::string error;
    Check(BuildMixamoPhysicsAsset(rig.skeleton, rig.pose, asset, error), "Generate anatomical T-pose preset");
    std::vector<Matrix4> world;
    Check(EvaluatePhysicsPose(rig.skeleton, rig.pose, Matrix4(kIdentity), world, error), "Evaluate ROM fixture");

    // Anatomical directions must not depend on world +X/+Y or importer axes.
    const Matrix4 moved(OrthogonalTransform(Quaternion(Normalize(Vector3(.3f, .7f, .2f)), 1.2f), Vector3(3, 4, -2)));
    rig.pose[0].xform = moved;
    PhysicsAsset rotated;
    Check(BuildMixamoPhysicsAsset(rig.skeleton, rig.pose, rotated, error), "Generate rotated/translated anatomical preset");
    for (size_t i = 0; i < asset.bodies.size(); ++i)
        Check(Near(asset.bodies[i].GetLocalTransform(), rotated.bodies[i].GetLocalTransform()), "Body calibration is invariant under rigid import transform");
    for (size_t i = 0; i < asset.constraints.size(); ++i)
        Check(Near(asset.constraints[i].parentFrame.GetTransform(), rotated.constraints[i].parentFrame.GetTransform()) &&
            Near(asset.constraints[i].childFrame.GetTransform(), rotated.constraints[i].childFrame.GetTransform()),
            "Both anatomical frames are invariant under rigid import transform");

    // A slightly splayed bind pose still needs each hinge's green twist dial
    // to follow its distal segment, with the hinge perpendicular to that segment.
    {
        Fixture splayed;
        for (float side : {1.0f, -1.0f})
        {
            const std::string prefix = side > 0 ? "Left" : "Right";
            for (const char* suffix : {"Leg", "Foot"})
            {
                auto& transform = splayed.pose[splayed.Find(prefix + suffix)].xform;
                transform.SetW(Vector4(Vector3(transform.GetW()) + Vector3(side * .025f, 0, 0), 1));
            }
            for (const char* suffix : {"ForeArm", "Hand"})
            {
                auto& transform = splayed.pose[splayed.Find(prefix + suffix)].xform;
                transform.SetW(Vector4(Vector3(transform.GetW()) + Vector3(0, -.02f, .015f), 1));
            }
        }
        PhysicsAsset splayedAsset;
        std::vector<Matrix4> splayedWorld;
        Check(BuildMixamoPhysicsAsset(splayed.skeleton, splayed.pose, splayedAsset, error) &&
            EvaluatePhysicsPose(splayed.skeleton, splayed.pose, Matrix4(kIdentity), splayedWorld, error),
            "Generate a splayed reference pose");
        for (const auto& joint : splayedAsset.constraints)
            if (joint.childBone == "LeftLeg" || joint.childBone == "RightLeg" ||
                joint.childBone == "LeftForeArm" || joint.childBone == "RightForeArm")
            {
                const auto& body = splayedAsset.bodies[splayedAsset.FindBody(joint.childBone)];
                const bool elbow = joint.childBone.find("ForeArm") != std::string::npos;
                const std::string prefix = joint.childBone.find("Left") == 0 ? "Left" : "Right";
                const int node = splayed.Find(joint.childBone);
                const int end = splayed.Find(prefix + (elbow ? "Hand" : "Foot"));
                const Vector3 direction = Normalize(Vector3(splayedWorld[end].GetW()) - Vector3(splayedWorld[node].GetW()));
                const Matrix4 frame = PhysicsBoneFrame(splayedWorld[node]) * body.GetLocalTransform() * joint.childFrame.GetTransform();
                Check(float(Length(Vector3(frame.GetY()) - direction)) < 1e-4f &&
                    std::abs(float(Dot(Vector3(frame.GetX()), direction))) < 1e-4f,
                    "Splayed elbow/knee twist references follow their distal segments with perpendicular hinge axes");
            }
    }

    PhysicsWorld physics;
    Check(physics.Initialize(false), "Initialize anatomical limit tests");
    const Vector3 left(kXUnitVector), up(kYUnitVector), forward(kZUnitVector);
    for (float side : {1.0f, -1.0f})
    {
        const std::string prefix = side > 0 ? "Left" : "Right";
        for (const char* suffix : {"ForeArm", "Leg", "UpLeg", "Hand", "Foot"})
        {
            const std::string name = prefix + suffix;
            const PhysicsConstraint* joint = nullptr;
            for (const auto& entry : asset.constraints) if (entry.childBone == name) joint = &entry;
            Check(joint != nullptr, "Find anatomical test joint");
            const auto& parent = asset.bodies[asset.FindBody(joint->parentBone)];
            const auto& child = asset.bodies[asset.FindBody(joint->childBone)];
            PhysicsAsset pair; pair.bodies = {parent, child}; pair.constraints = {*joint};
            const int parentNode = rig.Find(parent.bone), childNode = rig.Find(child.bone);
            std::vector<Matrix4> bones{world[parentNode], world[childNode]};
            const std::vector<std::string> names{parent.bone, child.bone};
            Ragdoll runtime;
            Check(runtime.Create(physics, pair, names, bones, Vector3(kZero), error, false), "Create real D6 pair for ROM boundary checks");
            Check(runtime.GetMaxAnchorError() < 1e-4f, "Anatomical bias preserves coincident anchors");
            Vector3 x = -up, y = -left;
            float low = 0, high = 0;
            if (std::string(suffix) == "ForeArm")
            {
                x = left * side; y = up * -side; low = -.8f; high = 144.6f;
            }
            else if (std::string(suffix) == "Leg")
            {
                y = left; low = -1; high = 137.7f;
            }
            else if (std::string(suffix) == "UpLeg") { low = -17.4f; high = 130.4f; }
            else if (std::string(suffix) == "Hand")
            { x = left * side; y = forward * -side; low = -79; high = 87; }
            else { low = -54.6f; high = 12.7f; }
            const bool hinge = std::string(suffix) == "Leg" || std::string(suffix) == "ForeArm";
            if (hinge)
                Check(joint->twist == PhysicsJointMotion::Limited &&
                    joint->swingY == PhysicsJointMotion::Locked && joint->swingZ == PhysicsJointMotion::Locked,
                    "Elbows and knees use single-twist hinges with both swing axes locked");
            const Matrix3 neutral(x, y, Cross(x, y));
            const auto sample = [&](float bend, float lateral, float twist) {
                const Matrix3 rotation(Quaternion(Vector3(kYUnitVector), XMConvertToRadians(bend)) *
                    Quaternion(Vector3(kZUnitVector), XMConvertToRadians(lateral)) *
                    Quaternion(Vector3(kXUnitVector), XMConvertToRadians(twist)));
                if (hinge)
                {
                    // Move the actual limb about the anatomical hinge pivot,
                    // independently of the generated constraint frames. This
                    // catches swapping limits without rotating the hinge axis.
                    const Matrix4 anatomicalFrame(neutral, Vector3(world[childNode].GetW()));
                    bones[1] = anatomicalFrame * Matrix4(rotation) * Invert(anatomicalFrame) * world[childNode];
                }
                else
                {
                    const Matrix4 desiredFrame(neutral * rotation, Vector3(world[childNode].GetW()));
                    bones[1] = desiredFrame * Invert(joint->childFrame.GetTransform()) * Invert(child.GetLocalTransform());
                }
                Check(runtime.UpdateKinematicPose(bones, true, error), "Set anatomical test orientation");
                return runtime.GetMaxAngularError();
            };
            for (float angle : {low, 0.0f, high})
                Check(sample(angle, 0, 0) < .001f, "Neutral and published sagittal endpoints lie inside the real D6 limit");
            Check(sample(low - 3, 0, 0) > .01f && sample(high + 3, 0, 0) > .01f,
                "D6 rejects both hyperextension and excessive flexion beyond anatomical endpoints");
            if (hinge)
            {
                for (float bend : {0.0f, 45.0f, 90.0f, 130.0f})
                {
                    Check(sample(bend, 0, 0) < .001f, "Twist hinges allow anatomical flexion throughout their range");
                    Check(sample(bend, 10, 0) > .1f && sample(bend, 0, 15) > .15f,
                        "Pure elbow/knee hinges reject side-bending and longitudinal rotation throughout flexion");
                }
            }
            if (std::string(suffix) == "UpLeg")
            {
                const float abduction = -32.1f * side, adduction = 13.7f * side;
                Check(sample(0, abduction, 0) < .001f && sample(0, adduction, 0) < .001f &&
                    sample(0, abduction - 3 * side, 0) > .01f && sample(0, adduction + 3 * side, 0) > .01f,
                    "Mirrored hips preserve distinct abduction/adduction despite the flexion-biased cone");
            }
        }
    }
    // Regression: checking Y and Z independently misses a diagonal cone escape.
    {
        PhysicsAsset diagonal;
        PhysicsBody a, b; a.bone = "a"; b.bone = "b";
        diagonal.bodies = {a, b};
        PhysicsConstraint joint; joint.parentBone = "a"; joint.childBone = "b";
        diagonal.constraints.push_back(joint); // 30-degree circular swing cone.
        const Quaternion swing(Normalize(Vector3(0, 1, 1)), XMConvertToRadians(40));
        const std::vector<Matrix4> bones{Matrix4(kIdentity), Matrix4(Matrix3(swing), Vector3(kZero))};
        Ragdoll runtime;
        Check(runtime.Create(physics, diagonal, {"a", "b"}, bones, Vector3(kZero), error, false) &&
            runtime.GetMaxAngularError() > XMConvertToRadians(9) && runtime.GetMaxAngularError() < XMConvertToRadians(11),
            "Diagonal swing outside the cone is detected even when both component angles are below their limits");
    }
    Check(physics.GetErrorCount() == 0, "No PhysX errors in anatomical boundary checks");
    std::puts("PASS: anatomical frame calibration, transformed imports, asymmetric ROM boundaries and mirrored hinge/twist limits");
}
