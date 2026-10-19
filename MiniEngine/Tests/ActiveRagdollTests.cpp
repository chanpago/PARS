#include "../Model/Ragdoll.h"
#include "../Model/PhysicsPose.h"
#include "../Core/PhysicsWorld.h"
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <cstring>

using namespace Math;
using namespace PARS;
using namespace physx;
namespace
{
    void Check(bool value, const char* message)
    { if (!value) { std::fprintf(stderr, "FAIL: %s\n", message); std::exit(1); } }
    float Difference(Quaternion a, Quaternion b)
    { return 2 * std::acos((std::min)(1.0f, std::abs(XMVectorGetX(XMVector4Dot(a, b))))); }
    Matrix4 Pose(Vector3 position, Quaternion rotation = Quaternion(kIdentity)) { return Matrix4(OrthogonalTransform(rotation, position)); }

    void TestDynamicPelvis()
    {
        PhysicsWorld world;
        Check(world.Initialize(false), "Initialize dynamic pelvis world");
        auto* floor = world.CreateStaticBox(PxTransform(PxVec3(0, -1, 0)), PxVec3(20, .1f, 20), "floor");
        PhysicsAsset asset;
        PhysicsBody pelvis; pelvis.bone = "pelvis"; pelvis.mass = 6;
        pelvis.shape = PhysicsShapeType::Box; pelvis.halfExtents = XMFLOAT3(.14f, .1f, .12f);
        pelvis.offset = XMFLOAT3(.03f, -.08f, .02f);
        XMStoreFloat4(&pelvis.rotation, Quaternion(Vector3(kZUnitVector), .4f));
        asset.bodies.push_back(pelvis);
        const std::vector<std::string> names{"pelvis"};
        const auto initial = Pose(Vector3(-2, 3, 1), Quaternion(Vector3(kYUnitVector), .7f));
        Ragdoll ragdoll; std::string error;
        Check(ragdoll.Create(world, asset, names, {initial}, Vector3(kZero), error) &&
            ragdoll.SetRootSupported(true, error) && ragdoll.GetKinematicBodyCount() == 0 && world.GetConstraintCount() == 1,
            "Pelvis support adds one force/torque constraint and no kinematic bodies");
        auto* scene = floor->getScene();
        PxActor* actor = nullptr; scene->getActors(PxActorTypeFlag::eRIGID_DYNAMIC, &actor, 1);
        auto* body = actor->is<PxRigidDynamic>();
        const auto step = [&](const Matrix4& target) {
            Check(ragdoll.UpdateDriveTargets({target}, 1.0f / 60, error), "Update pelvis target");
            world.Step();
            Check(ragdoll.IsFinite() && ragdoll.GetKinematicBodyCount() == 0, "Pelvis remains finite and dynamic");
        };
        // A distant target must produce bounded acceleration, never a teleport.
        const auto before = body->getGlobalPose();
        const auto distant = Pose(Vector3(10, 3, 1), Quaternion(Vector3(kYUnitVector), .7f));
        Check(ragdoll.UpdateDriveTargets({distant}, 0, error) &&
            (before.p - body->getGlobalPose().p).magnitude() < 1e-7f,
            "Setting a pelvis target does not overwrite the body pose");
        world.Step();
        Check(body->getLinearVelocity().x > .1f && body->getLinearVelocity().x <= 60.0f / 60 * 1.02f,
            "Pelvis linear acceleration obeys the finite force limit");
        for (int i = 0; i < 240; ++i) step(initial);
        Check(ragdoll.GetRootPositionError() < .025f && ragdoll.GetRootRotationError() < .02f,
            "Force/torque support holds a static world target against gravity");
        float positionError = 0, rotationError = 0;
        Matrix4 target = initial;
        for (int i = 0; i < 360; ++i)
        {
            const float t = i / 60.0f;
            target = Pose(Vector3(-2 + .25f * std::sin(t), 3 + .12f * std::sin(t * 1.3f), 1 + .15f * std::sin(t * .8f)),
                Quaternion(Vector3(kYUnitVector), .7f + .4f * std::sin(t)) * Quaternion(Vector3(kXUnitVector), .2f * std::sin(t * 1.2f)));
            step(target);
            if (i > 60)
            {
                const auto actual = ragdoll.GetBoneWorld(0);
                positionError = (std::max)(positionError, float(Length(Vector3(actual.GetW()) - Vector3(target.GetW()))));
                rotationError = (std::max)(rotationError, Difference(Quaternion(actual), Quaternion(target)));
            }
        }
        std::printf("dynamic pelvis moving error=%.6f m / %.6f rad\n", positionError, rotationError);
        Check(positionError < .04f && rotationError < .04f, "Pelvis tracks world translation and rotation including PA offset/rotation");
        for (int i = 0; i < 120; ++i) step(target);
        body->addForce(PxVec3(2, 1, 0), PxForceMode::eVELOCITY_CHANGE);
        body->addTorque(PxVec3(0, 3, 1), PxForceMode::eVELOCITY_CHANGE);
        float pushedPosition = 0, pushedRotation = 0;
        for (int i = 0; i < 180; ++i)
        {
            step(target);
            pushedPosition = (std::max)(pushedPosition, ragdoll.GetRootPositionError());
            pushedRotation = (std::max)(pushedRotation, ragdoll.GetRootRotationError());
        }
        Check(pushedPosition > .02f && pushedRotation > .005f &&
            ragdoll.GetRootPositionError() < .025f && ragdoll.GetRootRotationError() < .02f,
            "Impulses displace and rotate the supported pelvis, then the drives recover");
        // Block the target with a wall: finite motors must yield to contact.
        auto* wall = world.CreateStaticBox(PxTransform(PxVec3(-1, 3, 1)), PxVec3(.1f, 2, 2), "pelvis wall");
        const auto blocked = Pose(Vector3(0, 3, 1));
        for (int i = 0; i < 240; ++i) step(blocked);
        Check(body->getGlobalPose().p.x < -1.15f && ragdoll.GetRootPositionError() > 1,
            "Supported dynamic pelvis is blocked by a static wall instead of following through it");
        const auto releasePose = body->getGlobalPose();
        const auto linear = body->getLinearVelocity(), angular = body->getAngularVelocity();
        Check(ragdoll.SetRootSupported(false, error) && world.GetConstraintCount() == 0 &&
            (releasePose.p - body->getGlobalPose().p).magnitude() < 1e-7f &&
            (linear - body->getLinearVelocity()).magnitude() < 1e-7f &&
            (angular - body->getAngularVelocity()).magnitude() < 1e-7f,
            "Removing pelvis support preserves state and releases its world constraint");
        for (int i = 0; i < 30; ++i) world.Step();
        Check(body->getGlobalPose().p.y < releasePose.p.y - .2f, "Unsupported pelvis falls under gravity");
        Check(ragdoll.SetRootSupported(true, error) && ragdoll.SetRootSupported(true, error) && world.GetConstraintCount() == 1,
            "Repeated enable does not duplicate pelvis support");
        Ragdoll swapped; ragdoll.Swap(swapped);
        Check(!ragdoll.IsActive() && swapped.IsRootSupported(), "Swap transfers support ownership");
        swapped.Destroy();
        Check(world.GetConstraintCount() == 0 && world.GetDynamicBodyCount() == 0 && world.GetErrorCount() == 0,
            "Destroying supported ragdoll releases all PhysX objects without errors");
        wall->release();
        std::puts("PASS: dynamic pelvis force limits, animated targets, impacts, blocked contact and support lifecycle");
    }
}

void RunActiveRagdollTests()
{
    PhysicsAsset asset;
    PhysicsBody root; root.bone = "root"; root.shape = PhysicsShapeType::Sphere; root.radius = .08f;
    PhysicsBody child = root; child.bone = "child"; child.mass = 1.5f;
    child.offset = XMFLOAT3(.02f, -.1f, .03f);
    XMStoreFloat4(&child.rotation, Quaternion(Vector3(kZUnitVector), .3f));
    asset.bodies = {root, child};
    const std::vector<std::string> names{"root", "child"};
    const std::vector<Matrix4> initial{Pose(Vector3(0, 3, 0)), Pose(Vector3(0, 2.5f, 0))};
    PhysicsConstraint hinge; hinge.parentBone = "root"; hinge.childBone = "child";
    hinge.SetWorldFrame(initial[0], initial[1] * child.GetLocalTransform(), initial[0]);
    hinge.swingY = hinge.swingZ = PhysicsJointMotion::Locked;
    hinge.twistMin = -.9f; hinge.twistMax = .9f;
    hinge.drives[0] = {true, 720, 18, 25};
    asset.constraints.push_back(hinge);
    const auto target = [&](float angle) {
        auto result = initial;
        result[1] = initial[0] * Pose(Vector3(kZero), Quaternion(Vector3(kXUnitVector), angle)) * Invert(initial[0]) * initial[1];
        return result;
    };
    const auto desired = target(.65f);
    const auto goal = EvaluateJointDriveTarget(hinge, desired[0], desired[1] * child.GetLocalTransform());
    Check(Difference(goal, Quaternion(Vector3(kXUnitVector), .65f)) < .001f, "Drive conversion includes body offsets and both joint frames");
    const Matrix4 global = Pose(Vector3(-2, 1, 4), Quaternion(Vector3(kYUnitVector), .9f));
    Check(Difference(goal, EvaluateJointDriveTarget(hinge, global * desired[0], global * desired[1] * child.GetLocalTransform())) < .001f,
        "Joint drive target is invariant under actor world translation and rotation");
    auto excessive = target(1.5f);
    Check(Difference(EvaluateJointDriveTarget(hinge, excessive[0], excessive[1] * child.GetLocalTransform()), Quaternion(Vector3(kXUnitVector), .9f)) < .001f,
        "Hinge target respects asymmetric twist endpoints");
    PhysicsConstraint cone;
    cone.twist = PhysicsJointMotion::Locked; cone.swingYLimit = .45f; cone.swingZLimit = .7f;
    const auto diagonal = EvaluateJointDriveTarget(cone, Matrix4(kIdentity), Pose(Vector3(kZero), Quaternion(Normalize(Vector3(0, 1, 1)), 1.5f)));
    XMFLOAT4 q; XMStoreFloat4(&q, diagonal);
    const float y = 4 * std::atan2(q.y, 1 + q.w), z = 4 * std::atan2(q.z, 1 + q.w);
    Check(y * y / (.45f * .45f) + z * z / (.7f * .7f) <= 1.0001f && std::abs(q.x) < .0001f,
        "Diagonal target is projected into actual swing ellipse, with locked twist removed");

    PhysicsWorld world;
    Check(world.Initialize(false), "Initialize active ragdoll test world");
    auto* floor = world.CreateStaticBox(PxTransform(PxVec3(0, -1, 0)), PxVec3(10, .1f, 10), "floor");
    Ragdoll ragdoll; std::string error;
    Check(ragdoll.Create(world, asset, names, initial, Vector3(kZero), error), "Create dynamic twist hinge");
    Check(ragdoll.SetRootSupported(true, error) && ragdoll.GetKinematicBodyCount() == 0, "Support dynamic parent independently of drive mode");
    Check(ragdoll.UpdateDriveTargets(desired, 0, error), "Prepare stationary drive target");
    ragdoll.SetDrivesEnabled(true);
    Check(ragdoll.GetDrivenJointCount() == 1, "Locked swings allow twist-only D6 drive");
    auto incompatible = asset;
    incompatible.constraints[0].swingY = incompatible.constraints[0].swingZ = PhysicsJointMotion::Limited;
    incompatible.constraints[0].useSlerpDrive = true;
    Check(!ragdoll.ApplyDriveSettings(incompatible, error) && ragdoll.GetDrivenJointCount() == 1,
        "Runtime drive edits reject SLERP on live locked axes without changing the hinge");
    for (int i = 0; i < 180; ++i) { ragdoll.UpdateDriveTargets(desired, 1.0f / 60, error); world.Step(); }
    const float heldError = ragdoll.GetMaxDriveError();
    std::printf("active hinge settled error=%.6f rad gap=%.6f limit=%.6f\n", heldError, ragdoll.GetMaxAnchorError(), ragdoll.GetMaxAngularError());
    Check(heldError < .08f && ragdoll.GetMaxAnchorError() < .005f, "PD hinge reaches static target against gravity");
    ragdoll.AddVelocity(Vector3(0, 0, 2));
    float peakError = 0;
    for (int i = 0; i < 180; ++i)
    {
        ragdoll.UpdateDriveTargets(desired, 1.0f / 60, error); world.Step();
        peakError = (std::max)(peakError, ragdoll.GetMaxDriveError());
    }
    Check(peakError > heldError + .02f && ragdoll.GetMaxDriveError() < .08f, "Physical impulse disturbs hinge and PD restores target");
    PxActor* actors[2]; floor->getScene()->getActors(PxActorTypeFlag::eRIGID_DYNAMIC, actors, 2);
    auto* moving = actors[0]->is<PxRigidDynamic>();
    if (std::strcmp(moving->getName(), "child") != 0) moving = actors[1]->is<PxRigidDynamic>();
    const auto pose = moving->getGlobalPose(); const auto velocity = moving->getLinearVelocity(); const auto angular = moving->getAngularVelocity();
    ragdoll.SetDrivesEnabled(false); ragdoll.SetDrivesEnabled(true);
    Check((pose.p - moving->getGlobalPose().p).magnitude() < 1e-7f &&
        (velocity - moving->getLinearVelocity()).magnitude() < 1e-7f && (angular - moving->getAngularVelocity()).magnitude() < 1e-7f,
        "Drive mode switches preserve physical pose and both velocities");
    auto weak = asset; weak.constraints[0].drives[0].maxTorque = .01f;
    Check(ragdoll.ApplyDriveSettings(weak, error), "Apply lower torque limit without recreating objects");
    float weakPeak = 0;
    for (int i = 0; i < 180; ++i) { ragdoll.UpdateDriveTargets(desired, 1.0f / 60, error); world.Step(); weakPeak = (std::max)(weakPeak, ragdoll.GetMaxDriveError()); }
    Check(weakPeak > .25f, "Low torque cannot hold limb against gravity throughout the interval");
    Check(ragdoll.ApplyDriveSettings(asset, error), "Restore torque limit");
    for (int i = 0; i < 180; ++i) { ragdoll.UpdateDriveTargets(desired, 1.0f / 60, error); world.Step(); }
    Check(ragdoll.GetMaxDriveError() < .08f, "Restored torque wakes and recovers the limb");
    float movingError = 0;
    for (int i = 0; i < 240; ++i)
    {
        const auto movingTarget = target(.4f * std::sin(i / 60.0f));
        ragdoll.UpdateDriveTargets(movingTarget, 1.0f / 60, error); world.Step();
        if (i > 60) movingError = (std::max)(movingError, ragdoll.GetMaxDriveError());
    }
    std::printf("moving hinge peak error=%f\n", movingError);
    Check(movingError < .12f && ragdoll.GetMaxAngularError() < .02f, "Moving target tracks without unlocking hinge swings");
    auto traveling = target(.2f);
    const auto rootMotion = Pose(Vector3(.02f, .01f, -.01f), Quaternion(Vector3(kYUnitVector), .015f));
    for (auto& bone : traveling) bone = rootMotion * bone;
    Check(ragdoll.UpdateDriveTargets(traveling, 1.0f / 60, error), "Move supported root before release");
    world.Step();
    auto* supported = actors[0]->is<PxRigidDynamic>();
    if (std::strcmp(supported->getName(), "root") != 0) supported = actors[1]->is<PxRigidDynamic>();
    const auto rootPose = supported->getGlobalPose();
    const auto rootLinear = supported->getLinearVelocity(), rootAngular = supported->getAngularVelocity();
    Check(rootLinear.magnitude() > .01f && rootAngular.magnitude() > .01f, "Supported dynamic root responds to moving position and rotation targets");
    ragdoll.SetDrivesEnabled(false);
    Check(ragdoll.GetDrivenJointCount() == 0 && ragdoll.IsRootSupported(), "Disabling angular drives alone preserves independent support");
    Check(ragdoll.SetRootSupported(false, error) && ragdoll.GetKinematicBodyCount() == 0, "Release support into full dynamics");
    Check((rootPose.p - supported->getGlobalPose().p).magnitude() < 1e-7f &&
        std::abs(rootPose.q.dot(supported->getGlobalPose().q)) > .99999f &&
        (rootLinear - supported->getLinearVelocity()).magnitude() < 1e-7f &&
        (rootAngular - supported->getAngularVelocity()).magnitude() < 1e-7f,
        "Releasing support preserves the pelvis pose and both animated velocities");
    world.Step(); ragdoll.Destroy();
    // Exercise the separate SLERP solver path with simultaneous XYZ rotation.
    auto ball = asset;
    auto& ballJoint = ball.constraints[0];
    ballJoint.useSlerpDrive = true;
    ballJoint.swingY = ballJoint.swingZ = PhysicsJointMotion::Limited;
    ballJoint.swingYLimit = ballJoint.swingZLimit = 1.2f;
    ballJoint.drives[0] = {true, 720, 18, 25};
    Check(ragdoll.Create(world, ball, names, initial, Vector3(kZero), error) && ragdoll.SetRootSupported(true, error),
        "Create supported three-axis SLERP joint");
    ragdoll.SetDrivesEnabled(true);
    float ballError = 0;
    // The cone and its enclosing solver pyramid must not produce a second,
    // metre-sized overlay that ignores the requested joint visualization scale.
    auto* scene = floor->getScene();
    scene->setVisualizationParameter(PxVisualizationParameter::eSCALE, 1.0f);
    scene->setVisualizationParameter(PxVisualizationParameter::eJOINT_LIMITS, .04f);
    world.Step();
    const auto& visualization = scene->getRenderBuffer();
    Check(visualization.getNbLines() > 0, "D6 angular limit visualization is present");
    for (PxU32 i = 0; i < visualization.getNbLines(); ++i)
    {
        const auto& line = visualization.getLines()[i];
        Check((line.pos0 - PxVec3(0, 3, 0)).magnitude() < .06f &&
            (line.pos1 - PxVec3(0, 3, 0)).magnitude() < .06f,
            "Cone plus enclosing pyramid respects joint visualization scale");
    }
    scene->setVisualizationParameter(PxVisualizationParameter::eSCALE, 0.0f);
    for (int i = 0; i < 300; ++i)
    {
        const float t = i / 60.0f;
        const auto rotation = Quaternion(Vector3(kZUnitVector), .35f * std::sin(t * 1.3f)) *
            Quaternion(Vector3(kYUnitVector), .3f * std::cos(t)) * Quaternion(Vector3(kXUnitVector), .4f * std::sin(t));
        auto sample = initial;
        sample[1] = initial[0] * Pose(Vector3(kZero), rotation) * Invert(initial[0]) * initial[1];
        Check(ragdoll.UpdateDriveTargets(sample, 1.0f / 60, error), "Update three-axis moving target");
        world.Step();
        if (i > 60) ballError = (std::max)(ballError, ragdoll.GetMaxDriveError());
    }
    std::printf("moving SLERP peak error=%f\n", ballError);
    Check(ballError < .12f && ragdoll.GetMaxAnchorError() < .005f, "SLERP follows combined XYZ rotation with correct target velocity");
    ragdoll.Destroy();
    Check(world.GetConstraintCount() == 0 && world.GetDynamicBodyCount() == 0 && world.GetErrorCount() == 0, "No errors or leaks in active lifecycle");
    std::puts("PASS: active target frames, limit projection, PD hinge tracking, torque limits, impulse recovery and mode preservation");
    world.Shutdown();
    TestDynamicPelvis();
}
