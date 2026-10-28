#include "../Model/Ragdoll.h"
#include "../Core/PhysicsWorld.h"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>

using namespace Math;
using namespace PARS;
using namespace physx;

namespace
{
    void Require(bool value, const char* message, const std::string& error = {})
    {
        if (!value)
        {
            std::fprintf(stderr, "FAIL: %s: %s\n", message, error.c_str());
            std::exit(1);
        }
    }

    void PrintTiming(const char* name, std::vector<double> values)
    {
        std::sort(values.begin(), values.end());
        std::printf("%s median_ms=%.6f p95_ms=%.6f max_ms=%.6f samples=%zu\n", name,
            values[values.size() / 2], values[values.size() * 95 / 100], values.back(), values.size());
    }
}

void RunRagdollReuseTests(bool verifyReuse)
{
    PhysicsWorld world;
    Require(world.Initialize(false), "Initialize reusable ragdoll world");
    auto* floor = world.CreateStaticBox(PxTransform(PxVec3(0, -10, 0)), PxVec3(20, 1, 20), "floor");
    Require(floor != nullptr, "Create scene reference");
    auto& scene = *floor->getScene();
    auto& physics = scene.getPhysics();
    const auto materialsBefore = physics.getNbMaterials();
    const auto shapesBefore = physics.getNbShapes();
    const auto articulationsBefore = physics.getNbArticulations();
    PhysicsAsset asset;
    std::vector<std::string> names;
    std::vector<Matrix4> pose;
    for (int i = 0; i < 20; ++i)
    {
        PhysicsBody body;
        body.bone = "reuse_" + std::to_string(i);
        body.shape = PhysicsShapeType::Sphere;
        body.radius = .04f;
        body.mass = 1.0f;
        names.push_back(body.bone);
        asset.bodies.push_back(body);
        pose.emplace_back(OrthogonalTransform(Quaternion(kIdentity), Vector3(0, 3 + .2f * i, 0)));
        if (i > 0)
        {
            PhysicsConstraint joint;
            joint.parentBone = names[i - 1];
            joint.childBone = names[i];
            joint.parentFrame.offset.y = .2f;
            asset.constraints.push_back(joint);
        }
    }
    ConfigureDefaultDrives(asset);
    Ragdoll ragdoll;
    std::string error;
    using Clock = std::chrono::steady_clock;
    auto begin = Clock::now();
    Require(ragdoll.Create(world, asset, names, pose, Vector3(kZero), error, false), "Prepare initial bodies", error);
    const double creationMs = std::chrono::duration<double, std::milli>(Clock::now() - begin).count();
    std::printf("reuse_initialization_ms=%.6f\n", creationMs);
    if (verifyReuse)
    {
        Require(physics.getNbArticulations() == articulationsBefore + 1 && scene.getNbArticulations() == 0,
            "Articulation is allocated before activation and absent from the scene");
        Require(physics.getNbShapes() == shapesBefore + 40, "Both complete shape sets are prepared");
    }
    const auto preparedMaterials = physics.getNbMaterials();
    PxActor* kinematic[20];
    Require(scene.getActors(PxActorTypeFlag::eRIGID_DYNAMIC, kinematic, 20) == 20,
        "Only the kinematic set is registered initially");
    PxArticulationReducedCoordinate* originalArticulation = nullptr;
    std::vector<double> activateMs, deactivateMs;
    int owner = 0;
    ragdoll.SetQueryOwner(&owner);
    ragdoll.SetCollisionEnabled(false);
    for (int cycle = 0; cycle < 100; ++cycle)
    {
        pose[0] = Matrix4(OrthogonalTransform(Quaternion(Vector3(kYUnitVector), .01f * cycle), Vector3(.03f * cycle, 3, 0)));
        for (size_t i = 1; i < pose.size(); ++i)
        {
            pose[i] = pose[i - 1] * Matrix4(OrthogonalTransform(
                Quaternion(Vector3(kZUnitVector), .04f * std::sin(.2f * cycle + float(i))), Vector3(0, .2f, 0)));
        }
        Require(ragdoll.UpdateKinematicPose(pose, true, error), "Update animated pose while articulation is parked", error);
        begin = Clock::now();
        Require(ragdoll.SetSimulating(true, Vector3(2, 0, .5f), error), "Activate prepared articulation", error);
        activateMs.push_back(std::chrono::duration<double, std::milli>(Clock::now() - begin).count());
        PxArticulationReducedCoordinate* articulation = nullptr;
        Require(scene.getArticulations(&articulation, 1) == 1 && world.GetDynamicBodyCount() == 20,
            "Exactly one complete body set participates in simulation");
        if (originalArticulation && verifyReuse)
        {
            Require(articulation == originalArticulation && physics.getNbMaterials() == preparedMaterials,
                "Repeated activation retains articulation identity and material allocations");
        }
        originalArticulation = articulation;
        for (size_t i = 0; i < pose.size(); ++i)
        {
            const auto actual = ragdoll.GetBoneWorld(i);
            Require(float(Length(Vector3(actual.GetW()) - Vector3(pose[i].GetW()))) < .0001f &&
                    float(Length(Vector3(actual.GetX()) - Vector3(pose[i].GetX()))) < .0001f,
                "Activation inherits current root and joint pose, including bent limbs");
        }
        Require((articulation->getRootLinearVelocity() - PxVec3(2, 0, .5f)).magnitude() < .0001f,
            "Activation resets the root velocity to the requested value");
        PxArticulationLink* links[20];
        Require(articulation->getLinks(links, 20) == 20, "Read active links");
        for (auto* link : links)
        {
            PxShape* shape = nullptr;
            link->getShapes(&shape, 1);
            Require(link->userData == &owner && shape && !(shape->getFlags() & PxShapeFlag::eSIMULATION_SHAPE),
                "Owner and disabled contact flags survive activation");
        }
        // 대기 전에 남긴 외력이 다음 활성화에 재적용되지 않는지 실제 물리 스텝으로 확인한다.
        world.Step();
        float velocityX = 0;
        for (auto* link : links)
        {
            velocityX += link->getLinearVelocity().x / 20.0f;
        }
        const float expectedVelocityX = 2 * (1 - .05f * float(PhysicsWorld::FixedDeltaTime));
        Require(std::abs(velocityX - expectedVelocityX) < .0001f, "No residual impulse survives parking");
        ragdoll.SetDrivesEnabled(true);
        Require(ragdoll.UpdateDriveTargets(pose, 0, error), "Initialize current drive targets", error);
        if (cycle % 2 == 0)
        {
            Require(ragdoll.SetRootSupport(.4f, false, error), "Use partial pelvis support before parking", error);
        }
        links[0]->addForce(PxVec3(10000, 0, 0));
        begin = Clock::now();
        Require(ragdoll.SetSimulating(false, Vector3(kZero), error), "Return to prepared kinematic bodies", error);
        deactivateMs.push_back(std::chrono::duration<double, std::milli>(Clock::now() - begin).count());
        Require(!ragdoll.IsRootSupported() && !ragdoll.AreDrivesEnabled() && scene.getNbArticulations() == 0 &&
                world.GetDynamicBodyCount() == 20 && ragdoll.GetKinematicBodyCount() == 20,
            "Parked articulation has no active support or duplicate collision bodies");
        if (verifyReuse)
        {
            PxActor* current[20];
            Require(scene.getActors(PxActorTypeFlag::eRIGID_DYNAMIC, current, 20) == 20, "Read reused kinematic bodies");
            for (auto* body : kinematic)
            {
                Require(std::find(current, current + 20, body) != current + 20, "Kinematic body identities are retained");
            }
        }
    }
    PrintTiming("reuse_activate", activateMs);
    PrintTiming("reuse_deactivate", deactivateMs);
    if (verifyReuse)
    {
        const float invalid = std::numeric_limits<float>::quiet_NaN();
        Require(!ragdoll.SetSimulating(true, Vector3(invalid, 0, 0), error) &&
                !ragdoll.IsSimulating() && scene.getNbArticulations() == 0 && world.GetDynamicBodyCount() == 20,
            "Invalid activation leaves the current configuration intact");
        auto edited = asset;
        edited.constraints[0].drives[0].enabled = false;
        Require(ragdoll.ApplyDriveSettings(edited, error), "Edit drives while articulation is parked", error);
        Require(ragdoll.SetSimulating(true, Vector3(kZero), error), "Activate after drive edits", error);
        ragdoll.SetDrivesEnabled(true);
        PxArticulationReducedCoordinate* articulation = nullptr;
        Require(scene.getArticulations(&articulation, 1) == 1 && articulation == originalArticulation,
            "Drive edits retain the prepared articulation");
        PxArticulationLink* links[20];
        articulation->getLinks(links, 20);
        bool checked = false;
        for (auto* link : links)
        {
            const auto* joint = link->getInboundJoint();
            if (joint && std::string(joint->getName()) == names[1])
            {
                const auto drive = joint->getDriveParams(PxArticulationAxis::eTWIST);
                Require(drive.stiffness == 0 && drive.damping == 0 && drive.maxForce == 0,
                    "The reactivated joint receives the edited drive settings");
                checked = true;
            }
        }
        Require(checked, "Inspect edited joint");
        for (auto& joint : edited.constraints)
        {
            for (auto& drive : joint.drives)
            {
                drive.useAcceleration = false;
            }
        }
        Require(ragdoll.ApplyDriveSettings(edited, error) && !ragdoll.UsesArticulation() &&
                physics.getNbArticulations() == articulationsBefore && world.GetDynamicBodyCount() == 20,
            "Changing to D6 releases the obsolete prepared articulation", error);
        Require(ragdoll.SetSimulating(false, Vector3(kZero), error), "Park D6 bodies", error);
        Require(ragdoll.ApplyDriveSettings(asset, error) && !ragdoll.IsSimulating() &&
                scene.getNbArticulations() == 0 && physics.getNbArticulations() == articulationsBefore + 1,
            "Selecting articulation drives in kinematic mode prepares the new backend immediately", error);
        Require(ragdoll.SetSimulating(true, Vector3(kZero), error) && ragdoll.UsesArticulation(),
            "Activate the newly prepared backend", error);
        Require(ragdoll.ApplyDriveSettings(edited, error) && !ragdoll.UsesArticulation(),
            "Switch a running articulation to D6", error);
        Require(ragdoll.ApplyDriveSettings(asset, error) && ragdoll.UsesArticulation(),
            "Switch running D6 bodies to an articulation and prepare the kinematic set", error);
        Require(ragdoll.SetSimulating(false, Vector3(kZero), error) &&
                physics.getNbShapes() == shapesBefore + 40 && scene.getNbArticulations() == 0,
            "The replacement backend also retains exactly two complete configurations", error);
    }
    ragdoll.Destroy();
    Require(physics.getNbArticulations() == articulationsBefore && physics.getNbMaterials() == materialsBefore &&
            physics.getNbShapes() == shapesBefore && world.GetDynamicBodyCount() == 0,
        "Destroy releases both registered and parked configurations");
    Require(world.GetErrorCount() == 0, "No PhysX errors across one hundred mode switches");
    std::puts("PASS: prepared articulation lifecycle, pose/velocity transfer, contact isolation and cleanup");
}
