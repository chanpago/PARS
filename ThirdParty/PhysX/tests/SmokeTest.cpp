#include <PxPhysicsAPI.h>
#include "../physx/source/pvd/include/PxPvdUserRenderer.h"

#include <atomic>
#include <cmath>
#include <cstdio>
#include <stdexcept>
#include <vector>

using namespace physx;

namespace
{
    void Require(bool condition, const char* message)
    {
        if (!condition)
            throw std::runtime_error(message);
    }

#if defined(_DEBUG)
    void TestJointVisualization(PxPhysics& physics, PxScene& scene, PxMaterial& material)
    {
        const PxTransform anchor(PxVec3(5, 5, 5), PxQuat(.7f, PxVec3(0, 0, 1)));
        auto* parent = PxCreateDynamic(physics, anchor, PxSphereGeometry(.02f), material, 100.0f);
        auto* child = PxCreateDynamic(physics, anchor, PxSphereGeometry(.02f), material, 100.0f);
        Require(parent && child, "Visualization test bodies");
        parent->setRigidBodyFlag(PxRigidBodyFlag::eKINEMATIC, true);
        child->setRigidBodyFlag(PxRigidBodyFlag::eKINEMATIC, true);
        scene.addActor(*parent); scene.addActor(*child);
        scene.setVisualizationParameter(PxVisualizationParameter::eSCALE, 1.0f);
        scene.setVisualizationParameter(PxVisualizationParameter::eJOINT_LIMITS, .04f);
        scene.setVisualizationParameter(PxVisualizationParameter::eJOINT_LOCAL_FRAMES, 0.0f);
        PxD6Joint* joint = nullptr;
        const auto resetJoint = [&]() {
            if (joint) joint->release();
            joint = PxD6JointCreate(physics, parent, PxTransform(PxIdentity), child, PxTransform(PxIdentity));
            Require(joint != nullptr, "Visualization test joint");
            joint->setConstraintFlag(PxConstraintFlag::eVISUALIZATION, true);
            joint->setMotion(PxD6Axis::eTWIST, PxD6Motion::eFREE);
            joint->setMotion(PxD6Axis::eSWING1, PxD6Motion::eFREE);
            joint->setMotion(PxD6Axis::eSWING2, PxD6Motion::eFREE);
        };
        const auto sample = [&](PxQuat relative, bool violated, const char* label, float radius = .04f) {
            const PxTransform pose(anchor.p, anchor.q * relative);
            parent->setKinematicTarget(anchor); child->setKinematicTarget(pose);
            scene.simulate(1.0f / 60.0f); Require(scene.fetchResults(true), "Visualization simulation");
            // The immediate render buffer samples poses at visualizeStartStep,
            // before kinematic targets are applied (PVD samples after fetch).
            // Hold the same target for the next step to compare the same pose.
            parent->setKinematicTarget(anchor); child->setKinematicTarget(pose);
            scene.simulate(1.0f / 60.0f); Require(scene.fetchResults(true), "Visualization sample step");
            Require((child->getGlobalPose().p - pose.p).magnitude() < 1e-5f &&
                PxAbs(child->getGlobalPose().q.dot(pose.q)) > .99999f, "Kinematic pose must not be clamped by visualization");
            const auto& buffer = scene.getRenderBuffer();
            Require(buffer.getNbLines() > 0, "Kinematic limits must be visualized");
            const PxU32 color = violated ? PxDebugColor::eARGB_RED : PxDebugColor::eARGB_GREEN;
            for (PxU32 i = 0; i < buffer.getNbLines(); ++i)
            {
                const auto& line = buffer.getLines()[i];
                if (line.color0 != color || line.color1 != color)
                    std::fprintf(stderr, "Visualization case: %s, color=%08x expected=%08x\n", label, line.color0, color);
                Require(line.color0 == color && line.color1 == color, "Actual kinematic limit state controls color");
                Require((line.pos0 - anchor.p).magnitude() < radius * 1.5f + 1e-5f &&
                    (line.pos1 - anchor.p).magnitude() < radius * 1.5f + 1e-5f,
                    "Angular visualization obeys scale without metre-sized duplicate pyramid");
            }
        };
        resetJoint();
        joint->setMotion(PxD6Axis::eTWIST, PxD6Motion::eLIMITED);
        joint->setTwistLimit(PxJointAngularLimitPair(-.2f, .4f));
        sample(PxQuat(PxIdentity), false, "twist inside");
        sample(PxQuat(.3f, PxVec3(1, 0, 0)), false, "asymmetric twist inside");
        sample(PxQuat(.5f, PxVec3(1, 0, 0)), true, "upper twist violation");
        sample(PxQuat(-.3f, PxVec3(1, 0, 0)), true, "lower twist violation");
        sample(-PxQuat(.3f, PxVec3(1, 0, 0)), false, "equivalent negative quaternion inside");
        sample(-PxQuat(.5f, PxVec3(1, 0, 0)), true, "equivalent negative quaternion outside");
        sample(PxQuat(PxIdentity), false, "twist recovers to green");

        resetJoint();
        joint->setMotion(PxD6Axis::eSWING1, PxD6Motion::eLIMITED);
        joint->setMotion(PxD6Axis::eSWING2, PxD6Motion::eLIMITED);
        joint->setSwingLimit(PxJointLimitCone(.4f, .7f));
        joint->setPyramidSwingLimit(PxJointLimitPyramid(-.4f, .4f, -.7f, .7f));
        sample(PxQuat(.3f, PxVec3(0, 1, 0)), false, "swing cone inside");
        sample(PxQuat(.5f, PxVec3(0, 1, 0)), true, "swing Y violation");
        sample(PxQuat(-.8f, PxVec3(0, 0, 1)), true, "swing Z violation");
        const float y = PxTan(.32f / 4), z = PxTan(.56f / 4), squared = y*y + z*z;
        const PxQuat diagonal(0, 2*y/(1+squared), 2*z/(1+squared), (1-squared)/(1+squared));
        sample(diagonal, true, "diagonal ellipse violation with individual angles inside");
        sample(-PxQuat(.3f, PxVec3(0, 1, 0)), false, "equivalent negative swing quaternion");
        scene.setVisualizationParameter(PxVisualizationParameter::eJOINT_LIMITS, .01f);
        sample(PxQuat(PxIdentity), false, "smaller cone scale", .01f);
        scene.setVisualizationParameter(PxVisualizationParameter::eJOINT_LIMITS, .04f);

        for (int axis = 0; axis < 2; ++axis)
        {
            resetJoint();
            const auto limited = axis ? PxD6Axis::eSWING2 : PxD6Axis::eSWING1;
            const auto other = axis ? PxD6Axis::eSWING1 : PxD6Axis::eSWING2;
            const PxVec3 direction = axis ? PxVec3(0, 0, 1) : PxVec3(0, 1, 0);
            joint->setMotion(limited, PxD6Motion::eLIMITED);
            joint->setMotion(other, PxD6Motion::eLOCKED);
            joint->setSwingLimit(PxJointLimitCone(.4f, .4f));
            sample(PxQuat(.2f, direction), false, "single-axis swing fan inside");
            sample(PxQuat(-.5f, direction), true, "single-axis swing fan outside");
            joint->setMotion(other, PxD6Motion::eFREE);
            sample(PxQuat(.2f, direction), false, "double cone inside");
            sample(PxQuat(-.5f, direction), true, "double cone outside");
        }
        joint->release(); child->release(); parent->release();
        scene.setVisualizationParameter(PxVisualizationParameter::eSCALE, 0.0f);

        // The legacy PVD wire format already contains an active byte. Verify
        // that both values reach it, rather than the old constant true value.
        struct Capture : pvdsdk::RendererEventClient
        {
            std::vector<uint8_t> bytes;
            void handleBufferFlush(const uint8_t* data, uint32_t size) override
            { bytes.assign(data, data + size); }
        } capture;
        auto* renderer = pvdsdk::PvdUserRenderer::create();
        renderer->setClient(&capture);
        for (int kind = 0; kind < 3; ++kind)
        {
            std::vector<uint8_t> inactive;
            for (bool active : {false, true})
            {
                if (kind == 0) renderer->visualizeAngularLimit(anchor, -.2f, .4f, active);
                if (kind == 1) renderer->visualizeLimitCone(anchor, .1f, .2f, active);
                if (kind == 2) renderer->visualizeDoubleCone(anchor, .4f, active);
                renderer->flushRenderEvents();
                Require(!capture.bytes.empty() && capture.bytes.back() == uint8_t(active), "PVD serializes actual limit state");
                if (!active) inactive = capture.bytes;
                else
                {
                    auto normalized = capture.bytes; normalized.back() = 0;
                    Require(normalized == inactive, "PVD geometry is unchanged when only violation state changes");
                }
            }
        }
        renderer->release();
        std::puts("PASS: kinematic D6 twist/cone/fan/double-cone colors, recovery, quaternion signs, display scale and PVD state bytes");
    }
#endif

    class ErrorCallback final : public PxErrorCallback
    {
    public:
        std::atomic<unsigned> count{0};

        void reportError(PxErrorCode::Enum code, const char* message,
            const char* file, int line) override
        {
            ++count;
            std::fprintf(stderr, "PhysX %d: %s (%s:%d)\n", int(code), message, file, line);
        }
    };

    struct TestWorld
    {
        PxDefaultAllocator allocator;
        ErrorCallback errors;
        PxFoundation* foundation = nullptr;
        PxPvd* pvd = nullptr;
        PxPhysics* physics = nullptr;
        bool extensions = false;
        PxDefaultCpuDispatcher* dispatcher = nullptr;
        PxScene* scene = nullptr;
        PxMaterial* material = nullptr;
        PxControllerManager* controllers = nullptr;

        ~TestWorld()
        {
            if (controllers) controllers->release();
            if (scene) scene->release();
            if (dispatcher) dispatcher->release();
            if (material) material->release();
            if (extensions) PxCloseExtensions();
            if (physics) physics->release();
            if (pvd) pvd->release();
            if (foundation) foundation->release();
        }
    };
}

int main()
{
    try
    {
        TestWorld world;
        world.foundation = PxCreateFoundation(PX_PHYSICS_VERSION, world.allocator, world.errors);
        Require(world.foundation != nullptr, "Foundation initialization failed");
        world.pvd = PxCreatePvd(*world.foundation);
        Require(world.pvd != nullptr, "PVD initialization failed");
        world.physics = PxCreatePhysics(PX_PHYSICS_VERSION, *world.foundation,
            PxTolerancesScale(), false, world.pvd);
        Require(world.physics != nullptr, "Physics initialization failed");
        world.extensions = PxInitExtensions(*world.physics, world.pvd);
        Require(world.extensions, "Extensions initialization failed");
        world.dispatcher = PxDefaultCpuDispatcherCreate(1);
        Require(world.dispatcher != nullptr, "CPU dispatcher creation failed");

        PxSceneDesc sceneDesc(world.physics->getTolerancesScale());
        sceneDesc.gravity = PxVec3(0.0f, -9.81f, 0.0f);
        sceneDesc.cpuDispatcher = world.dispatcher;
        sceneDesc.filterShader = PxDefaultSimulationFilterShader;
        world.scene = world.physics->createScene(sceneDesc);
        Require(world.scene != nullptr, "Scene creation failed");
        world.material = world.physics->createMaterial(0.5f, 0.5f, 0.0f);
        Require(world.material != nullptr, "Material creation failed");

        PxRigidStatic* floor = PxCreatePlane(*world.physics, PxPlane(0, 1, 0, 0), *world.material);
        Require(floor != nullptr, "Floor creation failed");
        world.scene->addActor(*floor);
        PxRigidDynamic* box = PxCreateDynamic(*world.physics, PxTransform(PxVec3(0, 3, 0)),
            PxBoxGeometry(0.5f, 0.5f, 0.5f), *world.material, 1.0f);
        Require(box != nullptr, "Rigid body creation failed");
        world.scene->addActor(*box);

        constexpr float step = 1.0f / 60.0f;
        for (unsigned i = 0; i < 180; ++i)
        {
            world.scene->simulate(step);
            Require(world.scene->fetchResults(true), "Simulation failed");
        }
        const float boxHeight = box->getGlobalPose().p.y;
        Require(std::isfinite(boxHeight) && std::abs(boxHeight - 0.5f) < 0.05f,
            "Box did not settle on the floor");
        Require(box->getLinearVelocity().magnitude() < 0.1f, "Box did not come to rest");

        // Exercise Cooking, used later to turn level geometry into collision meshes.
        const PxVec3 vertices[] = {PxVec3(-1, 0, -1), PxVec3(0, 0, 1), PxVec3(1, 0, -1)};
        const PxU32 indices[] = {0, 1, 2};
        PxTriangleMeshDesc meshDesc;
        meshDesc.points.count = 3;
        meshDesc.points.stride = sizeof(PxVec3);
        meshDesc.points.data = vertices;
        meshDesc.triangles.count = 1;
        meshDesc.triangles.stride = 3 * sizeof(PxU32);
        meshDesc.triangles.data = indices;
        PxTriangleMesh* mesh = PxCreateTriangleMesh(PxCookingParams(world.physics->getTolerancesScale()),
            meshDesc, world.physics->getPhysicsInsertionCallback());
        Require(mesh != nullptr, "Triangle mesh cooking failed");
        mesh->release();

        world.controllers = PxCreateControllerManager(*world.scene);
        Require(world.controllers != nullptr, "Character controller manager creation failed");
        PxCapsuleControllerDesc capsuleDesc;
        capsuleDesc.position = PxExtendedVec3(2, 2, 0);
        capsuleDesc.radius = 0.35f;
        capsuleDesc.height = 1.0f;
        capsuleDesc.upDirection = PxVec3(0, 1, 0);
        capsuleDesc.material = world.material;
        PxController* capsule = world.controllers->createController(capsuleDesc);
        Require(capsule != nullptr, "Capsule controller creation failed");
        bool grounded = false;
        for (unsigned i = 0; i < 120; ++i)
        {
            const PxControllerCollisionFlags flags = capsule->move(PxVec3(0.01f, -0.05f, 0),
                0.001f, step, PxControllerFilters());
            grounded = flags.isSet(PxControllerCollisionFlag::eCOLLISION_DOWN);
            world.scene->simulate(step);
            Require(world.scene->fetchResults(true), "Controller simulation failed");
        }
        const PxExtendedVec3 position = capsule->getPosition();
        Require(grounded && position.y > 0.8 && position.y < 1.1 && position.x > 3.0,
            "Capsule did not move along the floor");
#if defined(_DEBUG)
        TestJointVisualization(*world.physics, *world.scene, *world.material);
#endif
        Require(world.errors.count.load() == 0, "PhysX reported errors or warnings");
        std::printf("PhysX %d.%d.%d PASS: box y=%.3f, mesh cooked, capsule=(%.3f, %.3f, %.3f) grounded\n",
            PX_PHYSICS_VERSION_MAJOR, PX_PHYSICS_VERSION_MINOR, PX_PHYSICS_VERSION_BUGFIX,
            boxHeight, position.x, position.y, position.z);
        return 0;
    }
    catch (const std::exception& error)
    {
        std::fprintf(stderr, "PhysX smoke test FAILED: %s\n", error.what());
        return 1;
    }
}
