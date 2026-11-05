#include "../Model/CharacterImpact.h"
#include "../Model/Ragdoll.h"
#include "../Core/PhysicsWorld.h"
#include "../Core/GameFramework/World.h"
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <chrono>

using namespace PARS;
using namespace Math;
using namespace physx;

namespace
{
    void CheckImpact(bool condition, const char* message)
    {
        if (!condition)
        {
            std::fprintf(stderr, "FAIL: %s\n", message);
            std::exit(1);
        }
    }

    class ImpactPhaseActor : public Actor
    {
    public:
        ImpactPhaseActor(std::vector<int>& order, int value) : m_Order(order), m_Value(value)
        {
        }

    public:
        void FixedTick(float) override
        {
            m_Order.push_back(m_Value);
        }
        void ApplyPhysicsInputs(float) override
        {
            m_Order.push_back(m_Value + 10);
        }

    private:
        std::vector<int>& m_Order;
        int m_Value;
    };
}

void RunCharacterImpactTests()
{
    CharacterImpactSettings settings;
    std::string error;
    float requested = 0;
    CheckImpact(settings.ComputeImpulse(3.5f, requested) == 180 && requested == 180, "Reference impulse and speed");
    settings.strengthMultiplier = 10;
    CheckImpact(settings.ComputeImpulse(3.5f, requested) == 500 && requested == 1800, "Command clamp preserves demand");
    CheckImpact(settings.ComputeImpulse(-1, requested) == 0 && requested == 0, "Separating motion has no impulse");
    CheckImpact(settings.ComputeImpulse(.1f, requested) == 0, "Subthreshold approach has no impulse");
    const std::string profilePath = "CharacterImpactSettings." +
        std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()) + ".test.json";
    CheckImpact(settings.Save(profilePath, error), "Save independent impact profile");
    CharacterImpactSettings loaded;
    CheckImpact(loaded.Load(profilePath, error) && loaded.strengthMultiplier == 10,
        "Reload impact profile");
    std::remove(profilePath.c_str());
    settings.referenceSpeed = 0;
    CheckImpact(!settings.Validate(error), "Reject zero reference speed");
    settings.referenceSpeed = std::numeric_limits<float>::quiet_NaN();
    CheckImpact(!settings.Validate(error), "Reject nonfinite profile");

    World game;
    CheckImpact(game.InitializePhysics(false), "Initialize phase test world");
    std::vector<int> order;
    game.SpawnActor<ImpactPhaseActor>(order, 1);
    game.SpawnActor<ImpactPhaseActor>(order, 2);
    game.BeginPlay();
    game.SetPhysicsPaused(true);
    game.Tick(1.0f);
    CheckImpact(order.empty(), "Paused frame does not apply external input");
    game.RequestPhysicsStep();
    game.Tick(0);
    CheckImpact(order == std::vector<int>({1, 2, 11, 12}), "All fixed ticks precede external input phase");
    game.Clear();
    game.GetPhysicsWorld()->Shutdown();

    PhysicsWorld world;
    CheckImpact(world.Initialize(false), "Initialize query scene");
    int owner = 1;
    for (int i = 0; i < 70; ++i)
    {
        PxRigidDynamic* body = world.CreateDynamicBody(PxTransform(PxVec3(.001f * i, 3, 0)), PxSphereGeometry(.1f), 1, "Query body");
        body->userData = &owner;
        PxShape* shape = nullptr;
        body->getShapes(&shape, 1);
        PxFilterData data;
        data.word3 = PhysicsWorld::CharacterImpactFilterFlag;
        data.word2 = static_cast<uint32_t>(i + 1);
        shape->setSimulationFilterData(data);
    }
    std::vector<PhysicsCapsuleHit> hits;
    world.SweepCharacterCapsule(PxTransform(PxVec3(0, 3, 0)), .2f, .2f, PxVec3(0), nullptr, hits);
    CheckImpact(hits.size() == 70, "Streaming capsule query retains all initial overlaps");
    world.SweepCharacterCapsule(PxTransform(PxVec3(0, 3, 0)), .2f, .2f, PxVec3(0), &owner, hits);
    CheckImpact(hits.empty(), "Capsule query excludes own PA");
    world.Shutdown();
    CheckImpact(world.Initialize(false), "Initialize impulse scene");
    PxRigidDynamic* first = world.CreateDynamicBody(PxTransform(PxVec3(0, 3, 0)), PxSphereGeometry(.1f), 1, "First character");
    PxRigidDynamic* second = world.CreateDynamicBody(PxTransform(PxVec3(.15f, 3, 0)), PxSphereGeometry(.1f), 1, "Second character");
    uint32_t group = 1;
    for (PxRigidDynamic* body : {first, second})
    {
        PxShape* shape = nullptr;
        body->getShapes(&shape, 1);
        PxFilterData data;
        data.word2 = group++;
        data.word3 = PhysicsWorld::CharacterImpactFilterFlag;
        shape->setSimulationFilterData(data);
    }
    world.Step();
    CheckImpact(std::abs(first->getLinearVelocity().x) < 1e-6f && std::abs(second->getLinearVelocity().x) < 1e-6f,
        "Character pair does not receive duplicate solver response");
    // 초기 관통의 위치 보정과 구분해, 실제 접근 속도로 운동량 전달을 검사한다.
    PxRigidDynamic* projectile = world.CreateDynamicBody(PxTransform(PxVec3(-.3f, 3, 0)), PxSphereGeometry(.1f), 1, "Projectile");
    projectile->setLinearVelocity(PxVec3(3, 0, 0));
    for (int step = 0; step < 5; ++step)
    {
        world.Step();
    }
    CheckImpact(projectile->getLinearVelocity().x < 2.9f && first->getLinearVelocity().x > .1f,
        "Unmarked projectile still collides with marked character");
    projectile->release();
    first->release();
    second->release();
    PhysicsAsset asset;
    PhysicsBody hip;
    hip.bone = "Hips";
    hip.shape = PhysicsShapeType::Sphere;
    hip.radius = .1f;
    hip.mass = 3;
    PhysicsBody spine = hip;
    spine.bone = "Spine";
    spine.mass = 1;
    asset.bodies = {hip, spine};
    const std::vector<std::string> names{"Hips", "Spine"};
    const std::vector<Matrix4> poses{Matrix4(OrthogonalTransform(Quaternion(kIdentity), Vector3(0, 3, 0))),
        Matrix4(OrthogonalTransform(Quaternion(kIdentity), Vector3(0, 3.3f, 0)))};
    PhysicsConstraint joint;
    joint.parentBone = "Hips";
    joint.childBone = "Spine";
    joint.SetWorldFrame(poses[0], poses[1], poses[1]);
    asset.constraints = {joint};
    ConfigureDefaultDrives(asset);
    Ragdoll ragdoll;
    CheckImpact(ragdoll.Create(world, asset, names, poses, Vector3(kZero), error), "Create articulated receiver");
    ragdoll.SetDrivesEnabled(false);
    std::vector<BodyImpactShare> shares;
    world.Step([&](float dt) -> void
    {
        CheckImpact(ragdoll.ApplyBodyImpact(PxVec3(4, 0, 0), PxVec3(0, 3.2f, 0), .6f, dt, shares), "Apply distributed external force");
    });
    PxVec3 sum(0);
    for (const BodyImpactShare& share : shares)
    {
        sum += share.impulse;
    }
    CheckImpact(shares.size() == 2 && (sum - PxVec3(4, 0, 0)).magnitude() < 1e-5f, "Body shares preserve total impulse");
    Vector3 velocity(kZero);
    CheckImpact(ragdoll.GetCenterOfMassVelocity(velocity) && std::abs(float(velocity.GetX()) - 1) < .01f,
        "Articulation COM receives J divided by total mass");
    CheckImpact(std::abs(ragdoll.GetGameplayImpactImpulse() - 4) < 1e-5f, "Completed step exposes gameplay input to recovery");
    world.Step();
    CheckImpact(ragdoll.GetGameplayImpactImpulse() == 0, "Gameplay input expires after one completed step");
    CheckImpact(!ragdoll.ApplyBodyImpact(PxVec3(1, 0, 0), PxVec3(0), .6f, 0, shares), "Reject zero time step");
    CheckImpact(world.GetErrorCount() == 0 && ragdoll.IsFinite(), "No PhysX validation errors or nonfinite state");
    std::puts("PASS: character impact settings, phase ordering, pause, streaming query, impulse distribution and recovery lifetime");
}
