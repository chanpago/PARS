#include "../Model/Ragdoll.h"
#include "../Model/PhysicsPose.h"
#include "../Core/PhysicsWorld.h"
#include <cstdio>
#include <cstdlib>
#include <cmath>

using namespace Math;
using namespace PARS;
namespace
{
    void Check(bool value, const char* message)
    { if (!value) { std::fprintf(stderr, "FAIL: %s\n", message); std::exit(1); } }
    bool Near(const Matrix4& a, const Matrix4& b)
    {
        return float(Length(Vector3(a.GetW()) - Vector3(b.GetW()))) < .0001f &&
            float(Length(Vector3(a.GetX()) - Vector3(b.GetX()))) < .0001f &&
            float(Length(Vector3(a.GetY()) - Vector3(b.GetY()))) < .0001f;
    }
    GraphNode Node(Vector3 position, float scale = 1)
    {
        GraphNode result{};
        result.xform = Matrix4(Matrix3::MakeScale(scale), position);
        result.rotation = Quaternion(kIdentity); result.scale = XMFLOAT3(scale, scale, scale);
        return result;
    }
}

void RunRagdollTests()
{
    // A skin root ignores the .01 armature above it. The mesh supplies that
    // scale exactly once, as in the imported Mixamo skeleton and DefaultVS.
    PhysicsSkeleton skeleton{{"armature", "hip", "helper", "leg", "toe", "mesh"},
        {-1, 0, 1, 2, 3, 0}, {-1, 5, -1, 5, 5, -1}};
    std::vector<GraphNode> pose{Node(Vector3(kZero), .01f), Node(Vector3(0, 200, 0)),
        Node(Vector3(0, -30, 0)), Node(Vector3(0, -30, 0)), Node(Vector3(0, -20, 0)), Node(Vector3(kZero))};
    pose[1].skeletonRoot = true;
    const Matrix4 component(OrthogonalTransform(Quaternion(Vector3(kYUnitVector), .7f), Vector3(3, 1, -2)));
    std::string error;
    std::vector<Matrix4> initial;
    Check(EvaluatePhysicsPose(skeleton, pose, component, initial, error), "Evaluate skin-space world pose");
    Check(float(Length(Vector3(initial[1].GetW()) - Vector3(component * Vector3(0, 2, 0)))) < 1e-5f,
        "Centimetres convert once and component location/rotation apply once");
    std::vector<int> nodes{1, 3};
    const Matrix4 motion(OrthogonalTransform(Quaternion(Vector3(kZUnitVector), .6f), Vector3(-2, 4, 1)));
    std::vector<Matrix4> targets{motion * initial[1], motion * initial[3]};
    std::vector<GraphNode> finalPose;
    Check(OverridePhysicsPose(skeleton, pose, component, nodes, targets, finalPose), "Convert physical world bones back to skin-local pose");
    std::vector<Matrix4> reconstructed;
    Check(EvaluatePhysicsPose(skeleton, finalPose, component, reconstructed, error) && Near(reconstructed[1], targets[0]) &&
        Near(reconstructed[3], targets[1]) && Near(reconstructed[4], motion * initial[4]),
        "Physics-to-render round trip preserves scale and helper/unsimulated child attachment");
    PhysicsAsset asset;
    PhysicsBody upper; upper.bone = "hip"; upper.radius = .12f; upper.halfLength = .18f;
    XMStoreFloat4(&upper.rotation, Quaternion(Vector3(kZUnitVector), XM_PIDIV2));
    PhysicsBody lower = upper; lower.bone = "leg"; lower.shape = PhysicsShapeType::Box; lower.halfExtents = XMFLOAT3(.3f, .1f, .1f);
    PhysicsBody toe; toe.bone = "toe"; toe.shape = PhysicsShapeType::Sphere; toe.radius = .08f;
    asset.bodies = {upper, lower, toe};
    for (int child = 1; child < 3; ++child)
    {
        PhysicsConstraint joint; joint.parentBone = asset.bodies[child - 1].bone; joint.childBone = asset.bodies[child].bone;
        const int a = child == 1 ? 1 : 3, b = child == 1 ? 3 : 4;
        Matrix4 pivot = PhysicsBoneFrame(initial[b]);
        joint.SetWorldFrame(PhysicsBoneFrame(initial[a]) * asset.bodies[child - 1].GetLocalTransform(),
            PhysicsBoneFrame(initial[b]) * asset.bodies[child].GetLocalTransform(), pivot);
        asset.constraints.push_back(joint);
    }
    asset.constraints[0].twist = PhysicsJointMotion::Locked;
    asset.constraints[0].swingY = PhysicsJointMotion::Free;
    PhysicsWorld physics;
    Check(physics.Initialize(false), "Initialize ragdoll world");
    physics.CreateStaticBox(physx::PxTransform(physx::PxVec3(0, -.25f, 0)), physx::PxVec3(100, .25f, 100), "RagdollFloor");
    Ragdoll ragdoll;
    Check(ragdoll.Create(physics, asset, skeleton.names, initial, Vector3(kZero), error) &&
        ragdoll.GetBodyCount() == 3 && ragdoll.GetJointCount() == 2, "Create all three PA shapes and D6 joints");
    Check(Near(ragdoll.GetBoneWorld(0), PhysicsBoneFrame(initial[1])) && ragdoll.GetMaxAnchorError() < .0001f,
        "Runtime bodies preserve animated bone offsets and joint anchors");
    Check(!ragdoll.Create(physics, asset, skeleton.names, initial, Vector3(kZero), error) && ragdoll.GetBodyCount() == 3,
        "Reject recreation without leaking or destroying live bodies");
    for (int i = 0; i < 360; ++i)
    {
        physics.Step();
        Check(ragdoll.IsFinite() && ragdoll.GetMaxAnchorError() < .08f, "Ragdoll remains finite and joints stay connected during fall");
    }
    Check(float(ragdoll.GetPosition().GetY()) < 1 && float(ragdoll.GetPosition().GetY()) > -.1f, "Ragdoll falls and rests on floor");
    Check(ragdoll.SetSimulating(false, Vector3(kZero), error) && !ragdoll.IsSimulating() &&
        ragdoll.GetKinematicBodyCount() == 3 && physics.GetConstraintCount() == 2,
        "Return the same bodies and joints to kinematic animation");
    Check(ragdoll.UpdateKinematicPose(initial, true, error), "Restore animation pose immediately");
    for (int i = 0; i < 120; ++i) physics.Step();
    Check(Near(ragdoll.GetBoneWorld(0), PhysicsBoneFrame(initial[1])), "Kinematic bones resist gravity");
    auto moved = initial;
    for (auto& bone : moved) bone = motion * bone;
    Check(ragdoll.UpdateKinematicPose(moved, false, error), "Set animation targets in rotated/transformed world space");
    physics.Step();
    Check(Near(ragdoll.GetBoneWorld(0), PhysicsBoneFrame(moved[1])) &&
        Near(ragdoll.GetBoneWorld(1), PhysicsBoneFrame(moved[3])) && Near(ragdoll.GetBoneWorld(2), PhysicsBoneFrame(moved[4])),
        "Fixed step moves every shape to its animated target, preserving PA offsets");
    Check(ragdoll.SetSimulating(true, Vector3(1, 0, 0), error) && ragdoll.IsSimulating() &&
        ragdoll.GetKinematicBodyCount() == 0 && physics.GetDynamicBodyCount() == 3 && physics.GetConstraintCount() == 2,
        "Switch existing kinematic objects to dynamic simulation without duplicates");
    const auto beforeFall = ragdoll.GetPosition();
    for (int i = 0; i < 30; ++i) physics.Step();
    Check(float(ragdoll.GetPosition().GetY()) < float(beforeFall.GetY()) - .2f, "Gravity resumes after kinematic-to-dynamic transition");
    ragdoll.Destroy();
    Check(!ragdoll.IsActive() && ragdoll.GetJointCount() == 0 && physics.GetDynamicBodyCount() == 0 &&
        physics.GetConstraintCount() == 0, "Release joints before bodies without leaving objects in the scene");
    for (int cycle = 0; cycle < 5; ++cycle)
    {
        Check(ragdoll.Create(physics, asset, skeleton.names, initial, Vector3(kZero), error, false) &&
            ragdoll.GetKinematicBodyCount() == 3, "Register PA directly in kinematic mode");
        Check(ragdoll.SetSimulating(true, Vector3(1, 0, 0), error), "Restart ragdoll");
        physics.Step(); ragdoll.Destroy();
    }
    Check(physics.GetErrorCount() == 0, "No PhysX errors during create, simulate, destroy and restart");
    std::puts("PASS: ragdoll skin/world conversion, imported scale, all shapes, D6 fall, floor contact and repeated lifecycle");
}
