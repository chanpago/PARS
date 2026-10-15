#include <PxPhysicsAPI.h>

#include <atomic>
#include <cmath>
#include <cstdio>
#include <stdexcept>

using namespace physx;

namespace
{
    void Require(bool condition, const char* message)
    {
        if (!condition)
            throw std::runtime_error(message);
    }

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
