#include "../Model/Ragdoll.h"
#include "../Model/PhysicsPose.h"
#include "../Core/PhysicsWorld.h"
#include <cmath>
#include <cstdio>
#include <cstdlib>

using namespace PARS;
using namespace Math;
using namespace physx;

namespace
{
    void CheckHorizontalForce(bool valid, const char* message)
    {
        if (!valid)
        {
            std::fprintf(stderr, "FAIL: %s\n", message);
            std::exit(1);
        }
    }
}

void RunHorizontalBalanceRagdollTests()
{
    PhysicsWorld world;
    CheckHorizontalForce(world.Initialize(false), "Initialize horizontal force fixture");
    PxRigidStatic* marker = world.CreateStaticBox(PxTransform(PxVec3(0, -10, 0)), PxVec3(1), "marker");
    CheckHorizontalForce(marker != nullptr, "Create scene marker");
    marker->getScene()->setGravity(PxVec3(0));
    PhysicsBody root;
    root.bone = "root";
    root.shape = PhysicsShapeType::Box;
    root.halfExtents = DirectX::XMFLOAT3(.05f, .05f, .05f);
    root.mass = 4;
    PhysicsBody child = root;
    child.bone = "child";
    PhysicsAsset asset;
    asset.bodies = {root, child};
    const std::vector<std::string> names{root.bone, child.bone};
    const std::vector<Matrix4> pose{
        Matrix4(OrthogonalTransform(Quaternion(kIdentity), Vector3(0, 3, 0))),
        Matrix4(OrthogonalTransform(Quaternion(kIdentity), Vector3(0, 3.5f, 0)))};
    PhysicsConstraint joint;
    joint.parentBone = root.bone;
    joint.childBone = child.bone;
    joint.twist = PhysicsJointMotion::Locked;
    joint.swingY = PhysicsJointMotion::Locked;
    joint.swingZ = PhysicsJointMotion::Locked;
    joint.SetWorldFrame(pose[0], pose[1], pose[1]);
    asset.constraints.push_back(joint);
    ConfigureDefaultDrives(asset);
    Ragdoll ragdoll;
    std::string error;
    CheckHorizontalForce(ragdoll.Create(world, asset, names, pose, Vector3(kZero), error) && ragdoll.UsesArticulation(),
        "Create a floating articulated root");
    ragdoll.SetDrivesEnabled(true);
    ragdoll.SetFootDrivenSupport(true);
    CheckHorizontalForce(ragdoll.SetRootSupport(1, false, error), "Keep root translation free and rotation supported");
    PxArticulationReducedCoordinate* articulation = nullptr;
    CheckHorizontalForce(marker->getScene()->getArticulations(&articulation, 1) == 1, "Find the force test articulation");
    PxArticulationLink* links[2]{};
    articulation->getLinks(links, 2);
    for (PxArticulationLink* link : links)
    {
        // 외력의 효과만 비교하도록 이 시험에서는 기본 바디 감쇠를 없앤다.
        link->setLinearDamping(0);
        link->setAngularDamping(0);
    }
    ragdoll.SetHorizontalSupportForce(PxVec3(80, 500, 0));
    CheckHorizontalForce(ragdoll.GetHorizontalSupportForce().y == 0 && ragdoll.UpdateDriveTargets(pose, 0, error),
        "Horizontal support rejects Y and zero-time updates do not accumulate force");
    world.Update(1.0 / 60.0);
    Vector3 velocity;
    CheckHorizontalForce(ragdoll.GetCenterOfMassVelocity(velocity) && float(Length(velocity)) < 1e-5f,
        "A settings-only update has no physical acceleration");
    const float dt = 1.0f / 60.0f;
    CheckHorizontalForce(ragdoll.UpdateDriveTargets(pose, dt, error) && ragdoll.UpdateDriveTargets(pose, dt, error),
        "Repeated positive-time queries share one pending physics step");
    world.Update(1.0 / 60.0);
    const float expectedSpeed = 80.0f / (root.mass + child.mass) / 60.0f;
    ragdoll.GetCenterOfMassVelocity(velocity);
    std::printf("horizontal force actual COM velocity %.8f %.8f %.8f expected %.8f, step %llu\n",
        float(velocity.GetX()), float(velocity.GetY()), float(velocity.GetZ()), expectedSpeed,
        static_cast<unsigned long long>(world.GetStepCount()));
    CheckHorizontalForce(ragdoll.GetCenterOfMassVelocity(velocity) &&
        std::abs(float(velocity.GetX()) - expectedSpeed) < 1e-4f &&
        std::abs(float(velocity.GetY())) < 1e-5f && std::abs(float(velocity.GetZ())) < 1e-5f,
        "The actual COM receives F divided by total mass exactly once");
    ragdoll.SetHorizontalSupportForce(PxVec3(-80, 0, 0));
    CheckHorizontalForce(ragdoll.UpdateDriveTargets(pose, dt, error), "Apply the opposing braking force");
    world.Update(1.0 / 60.0);
    CheckHorizontalForce(ragdoll.GetCenterOfMassVelocity(velocity) && float(Length(velocity)) < 1e-4f,
        "Direct support brakes COM without the inverse dynamics cancelling the force");
    ragdoll.SetHorizontalSupportForce(PxVec3(80, 0, 0));
    ragdoll.SetFootDrivenSupport(false);
    CheckHorizontalForce(ragdoll.GetHorizontalSupportForce().isZero(), "Leaving leg support clears assistance");
    ragdoll.SetHorizontalSupportForce(PxVec3(80, 0, 0));
    ragdoll.SetCollisionEnabled(false);
    CheckHorizontalForce(ragdoll.GetHorizontalSupportForce().isZero(), "Contact invalidation clears pending support force");
    CheckHorizontalForce(world.GetErrorCount() == 0, "Horizontal force control has no PhysX errors");
    std::puts("PASS: actual horizontal COM force, mass scaling, once-per-step, zero-time and invalidation");
}
