#include "../Model/CharacterImpact.h"
#include "../Model/Ragdoll.h"
#include "../Core/PhysicsWorld.h"
#include "../Core/GameFramework/World.h"
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <chrono>
#include <fstream>

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

    float SolveCapsuleContact(PhysicsWorld& service, float sourceMass, bool upperBody, bool enabled = true)
    {
        PhysicsWorld world;
        CheckImpact(world.InitializeScene(service), "Initialize the physical contact scene");
        PhysicsAsset asset;
        PhysicsBody root;
        root.bone = "Hips";
        root.shape = PhysicsShapeType::Sphere;
        root.radius = .1f;
        root.mass = 2;
        root.material.restitution = 0;
        root.material.staticFriction = 0;
        root.material.dynamicFriction = 0;
        asset.bodies.push_back(root);
        const std::vector<std::string> names{"Hips"};
        const std::vector<Matrix4> poses{Matrix4(OrthogonalTransform(Quaternion(kIdentity), Vector3(0, 3, 0)))};
        Ragdoll source;
        std::string error;
        int sourceOwner = 1;
        int targetOwner = 2;
        CheckImpact(source.Create(world, asset, names, poses, Vector3(kZero), error), "Create a finite-mass source body");
        source.SetQueryOwner(&sourceOwner);
        source.SetCharacterImpactEnabled(true, 101);
        CheckImpact(source.ConfigureCharacterCapsule(enabled, sourceMass, 1, .01f, error), "Configure the actual simulation capsule");
        const float activeMass = source.GetCharacterCapsuleMass();
        PxRigidDynamic* target = world.CreateDynamicBody(PxTransform(PxVec3(.25f, 3, 0)), PxSphereGeometry(.1f),
            1, "Contact receiver", root.material);
        target->userData = &targetOwner;
        PxShape* targetShape = nullptr;
        target->getShapes(&targetShape, 1);
        PxFilterData filter;
        filter.word2 = 102;
        filter.word3 = PhysicsWorld::CharacterImpactFilterFlag |
            (upperBody ? PhysicsWorld::CharacterImpactTargetFilterFlag : 0);
        targetShape->setSimulationFilterData(filter);
        source.AddVelocity(Vector3(2, 0, 0));
        float measuredImpulse = 0;
        for (int step = 0; step < 12; ++step)
        {
            world.Step();
            std::vector<PhysicsCapsuleContact> contacts;
            source.GetCharacterCapsuleContacts(contacts);
            for (const PhysicsCapsuleContact& contact : contacts)
            {
                CheckImpact(contact.actor == target, "Physical capsule contacts address the actual receiver");
                measuredImpulse += contact.normalImpulse;
            }
        }
        Vector3 sourceVelocity(kZero);
        CheckImpact(source.GetCenterOfMassVelocity(sourceVelocity), "Measure source recoil");
        const float targetVelocity = target->getLinearVelocity().x;
        if (upperBody)
        {
            // 중심을 지나는 비탄성 충돌의 해와 비교한다. 기존 선형 감쇠의 작은 영향만 허용한다.
            const float expectedVelocity = 2 * activeMass / (activeMass + 1);
            CheckImpact(std::abs(targetVelocity - expectedVelocity) < .08f &&
                float(sourceVelocity.GetX()) < 1.99f && (enabled ? measuredImpulse > .1f : measuredImpulse == 0),
                "PhysX produces the finite-mass collision solution and source recoil without manual forces");
            CheckImpact(std::abs(activeMass * float(sourceVelocity.GetX()) + targetVelocity - 2 * activeMass) < .12f,
                "Both dynamic bodies exchange momentum within the existing damping loss");
        }
        else
        {
            CheckImpact(std::abs(targetVelocity) < 1e-5f && measuredImpulse == 0,
                "Lower-body filter prevents simulation contact and momentum transfer");
        }
        CheckImpact(world.GetErrorCount() == 0 && source.IsFinite(), "Finite collision state with no PhysX errors");
        std::printf("capsule mass=%.3f enabled=%d upper=%d receiver_vx=%.6f source_vx=%.6f measured_J=%.6f\n",
            activeMass, enabled ? 1 : 0, upperBody ? 1 : 0, targetVelocity, float(sourceVelocity.GetX()), measuredImpulse);
        target->release();
        return targetVelocity;
    }
}

void RunCharacterImpactTests()
{
    CharacterImpactSettings settings;
    std::string error;
    settings.bodyMass = 20;
    settings.enabled = false;
    settings.capsuleHeightScale = 1.25f;
    const std::string profilePath = "CharacterImpactSettings." +
        std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()) + ".test.json";
    CheckImpact(settings.Save(profilePath, error), "Save independent impact profile");
    CharacterImpactSettings loaded;
    CheckImpact(loaded.Load(profilePath, error) && loaded.bodyMass == 20 && !loaded.enabled && loaded.capsuleHeightScale == 1.25f,
        "Reload impact profile");
    {
        std::ofstream legacy(profilePath);
        legacy << R"({"version":1,"enabled":true,"referenceImpulseNs":180,"referenceSpeedMps":3.5,
            "strengthMultiplier":1,"maximumImpulseNs":500,"minimumClosingSpeedMps":0.35,"pelvisShare":0.1,
            "separationTimeSeconds":0.1,"separationDistanceMeters":0.08,"cooldownSeconds":0.35,
            "capsuleRadiusScale":0.95,"contactSkinMeters":0.02})";
    }
    CheckImpact(loaded.Load(profilePath, error) && loaded.bodyMass == CharacterImpactSettings().bodyMass,
        "Legacy profile discards manual impulses and uses the new dynamic-body mass default");
    {
        std::ofstream legacy(profilePath);
        legacy << R"({"version":3,"enabled":false,"bodyMassKg":20,"capsuleRadiusScale":0.8,"contactSkinMeters":0.02})";
    }
    CheckImpact(loaded.Load(profilePath, error) && loaded.bodyMass == 20 && !loaded.enabled &&
        loaded.capsuleHeightScale == 1.0f && loaded.capsuleRadiusScale == .8f,
        "Version-three profiles retain their previous capsule dimensions with height scale one");
    std::remove(profilePath.c_str());
    settings.capsuleHeightScale = 0;
    CheckImpact(!settings.Validate(error), "Reject zero capsule height scale");
    settings.capsuleHeightScale = 1;
    settings.bodyMass = 0;
    CheckImpact(!settings.Validate(error), "Reject zero body mass");
    settings.bodyMass = std::numeric_limits<float>::quiet_NaN();
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
        data.word3 = PhysicsWorld::CharacterImpactFilterFlag | PhysicsWorld::CharacterImpactTargetFilterFlag;
        data.word2 = static_cast<uint32_t>(i + 1);
        shape->setSimulationFilterData(data);
    }
    std::vector<PhysicsCapsuleHit> hits;
    PxRigidDynamic* excluded = world.CreateDynamicBody(PxTransform(PxVec3(0, 3, 0)), PxSphereGeometry(.1f), 1, "Excluded lower body");
    excluded->userData = &owner;
    PxShape* excludedShape = nullptr;
    excluded->getShapes(&excludedShape, 1);
    PxFilterData excludedData;
    excludedData.word3 = PhysicsWorld::CharacterImpactFilterFlag;
    excludedShape->setSimulationFilterData(excludedData);
    world.SweepCharacterCapsule(PxTransform(PxVec3(0, 3, 0)), .2f, .2f, PxVec3(0), nullptr, hits);
    CheckImpact(hits.size() == 70, "Streaming query retains all upper overlaps and excludes non-target lower bodies");
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
        data.word3 = PhysicsWorld::CharacterImpactFilterFlag | PhysicsWorld::CharacterCapsuleOwnerFilterFlag;
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
    const std::vector<std::string> names{"Hips", "Spine", "RightArm", "RightHand", "LeftUpLeg", "LeftFoot", "Helper", "Head"};
    const std::vector<Vector3> positions{Vector3(0, 3, 0), Vector3(0, 3.3f, 0), Vector3(.3f, 3.3f, 0),
        Vector3(.6f, 3.3f, 0), Vector3(0, 2.7f, 0), Vector3(0, 2.4f, 0), Vector3(0, 2.1f, 0), Vector3(0, 3.6f, 0)};
    const int parents[] = {-1, 0, 1, 2, 0, 4, 5, 1};
    std::vector<Matrix4> poses;
    for (size_t i = 0; i < names.size(); ++i)
    {
        PhysicsBody body = i == 0 ? hip : spine;
        body.bone = names[i];
        asset.bodies.push_back(body);
        poses.push_back(Matrix4(OrthogonalTransform(Quaternion(kIdentity), positions[i])));
        if (parents[i] >= 0)
        {
            PhysicsConstraint joint;
            joint.parentBone = names[parents[i]];
            joint.childBone = names[i];
            joint.SetWorldFrame(poses[parents[i]], poses[i], poses[i]);
            asset.constraints.push_back(joint);
        }
    }
    ConfigureDefaultDrives(asset);
    Ragdoll ragdoll;
    CheckImpact(ragdoll.Create(world, asset, names, poses, Vector3(kZero), error), "Create articulated receiver");
    ragdoll.SetDrivesEnabled(false);
    ragdoll.SetQueryOwner(&owner);
    ragdoll.SetCharacterImpactEnabled(true, 100);
    for (size_t i = 0; i < names.size(); ++i)
    {
        CheckImpact(ragdoll.IsCharacterImpactBody(i) == (i < 4 || i == 7),
            "Root, torso, arms, hands and head are eligible; lower-body descendants are excluded");
    }
    world.SweepCharacterCapsule(PxTransform(PxVec3(0, 3, 0)), 2, .1f, PxVec3(0), nullptr, hits);
    CheckImpact(hits.size() == 5, "Ragdoll query flags expose exactly the five eligible upper bodies");
    const PxRigidBody* rootBody = nullptr;
    for (const PhysicsCapsuleHit& hit : hits)
    {
        if (ragdoll.FindPhysicsBody(hit.actor) == 0)
        {
            rootBody = hit.actor->is<PxRigidBody>();
        }
    }
    CheckImpact(rootBody != nullptr, "Locate the root mass and inertia for restoration checks");
    const PxVec3 originalInertia = rootBody->getMassSpaceInertiaTensor();
    const uint32_t bodyCount = world.GetDynamicBodyCount();
    CheckImpact(ragdoll.ConfigureCharacterCapsule(true, 6, .95f, .02f, error), "Attach a simulation capsule to the dynamic root");
    Vector3 center(kZero);
    float totalMass = 0;
    CheckImpact(ragdoll.HasCharacterCapsule() && ragdoll.GetCharacterCapsuleMass() == 6 &&
        ragdoll.GetCenterOfMass(center, totalMass) && std::abs(totalMass - 13) < 1e-4f,
        "Capsule mass replaces the three-kg root mass instead of adding another rigid body");
    CheckImpact(world.GetDynamicBodyCount() == bodyCount, "Capsule adds no duplicate rigid-body mass");
    CheckImpact((rootBody->getMassSpaceInertiaTensor() - originalInertia * 2).magnitude() < 1e-6f,
        "Changing root mass scales its existing inertia distribution consistently");
    PxTransform capsulePose(PxIdentity);
    PxVec3 targetOffset(0);
    float initialRadius = 0;
    float initialHalfLength = 0;
    CheckImpact(ragdoll.GetCharacterImpactCapsule(.95f, capsulePose, initialRadius, initialHalfLength, targetOffset),
        "Read the physical capsule size used by the debug outline");
    CheckImpact(ragdoll.ConfigureCharacterCapsule(true, 6, .95f, .02f, error, 1.5f), "Resize capsule height at runtime");
    float changedRadius = 0;
    float changedHalfLength = 0;
    CheckImpact(ragdoll.GetCharacterImpactCapsule(.95f, capsulePose, changedRadius, changedHalfLength, targetOffset) &&
        std::abs(changedRadius - initialRadius) < 1e-6f &&
        std::abs((changedRadius + changedHalfLength) - (initialRadius + initialHalfLength) * 1.5f) < 1e-5f &&
        rootBody->getNbShapes() == 2 && ragdoll.GetCharacterCapsuleMass() == 6,
        "Height control changes the simulation shape while preserving radius, shape count and mass");
    CheckImpact(ragdoll.ConfigureCharacterCapsule(true, 6, 3, .02f, error, .1f), "Allow a short, wide capsule");
    CheckImpact(ragdoll.GetCharacterImpactCapsule(3, capsulePose, changedRadius, changedHalfLength, targetOffset) &&
        changedHalfLength == 0 && changedRadius > initialRadius,
        "A requested height smaller than the diameter becomes a valid spherical capsule");
    CheckImpact(!ragdoll.ConfigureCharacterCapsule(true, 6, .95f, .02f, error, 0) && rootBody->getNbShapes() == 2,
        "Invalid resize leaves the current simulation shape intact");
    CheckImpact(ragdoll.ConfigureCharacterCapsule(true, 6, .95f, .02f, error), "Restore default capsule size");
    for (int cycle = 0; cycle < 20; ++cycle)
    {
        CheckImpact(ragdoll.ConfigureCharacterCapsule(false, 6, .95f, .02f, error) &&
            !ragdoll.HasCharacterCapsule() && rootBody->getNbShapes() == 1 && ragdoll.GetCharacterCapsuleMass() == 3 &&
            (rootBody->getMassSpaceInertiaTensor() - originalInertia).magnitude() < 1e-6f,
            "Turning the capsule off restores original root mass");
        CheckImpact(ragdoll.ConfigureCharacterCapsule(true, 6, .95f, .02f, error), "Re-enable without recreating the articulation");
        CheckImpact(rootBody->getNbShapes() == 2, "Repeated toggles retain exactly one additional capsule shape");
    }
    CheckImpact(!ragdoll.ConfigureCharacterCapsule(true, 0, .95f, .02f, error) && ragdoll.HasCharacterCapsule(),
        "Invalid runtime mass leaves the existing capsule intact");
    CheckImpact(ragdoll.SetSimulating(false, Vector3(kZero), error) && !ragdoll.HasCharacterCapsule(),
        "Leaving simulation removes the capsule before storing a standby articulation");
    CheckImpact(ragdoll.SetSimulating(true, Vector3(kZero), error) && ragdoll.GetCharacterCapsuleMass() == 3,
        "Reactivating the articulation restores its original mass without a stale capsule");
    ragdoll.SetCharacterImpactEnabled(true, 100);
    CheckImpact(ragdoll.ConfigureCharacterCapsule(true, 6, .95f, .02f, error),
        "A capsule can be enabled again after switching physics configurations");
    ragdoll.SetCharacterImpactEnabled(false, 100);
    world.SweepCharacterCapsule(PxTransform(PxVec3(0, 3, 0)), 2, .1f, PxVec3(0), nullptr, hits);
    CheckImpact(hits.empty(), "Disabling body impact clears upper-body query flags");
    CheckImpact(!ragdoll.HasCharacterCapsule() && ragdoll.GetCharacterCapsuleMass() == 3,
        "Disabling collision policy removes the physical capsule and restores root mass");
    CheckImpact(world.GetErrorCount() == 0 && ragdoll.IsFinite(), "No PhysX validation errors or nonfinite state");
    const float lightResponse = SolveCapsuleContact(world, .5f, true);
    const float heavyResponse = SolveCapsuleContact(world, 5, true);
    CheckImpact(heavyResponse > lightResponse + .5f, "A heavier capsule transfers more impulse at the same approach speed");
    SolveCapsuleContact(world, 5, false);
    CheckImpact(SolveCapsuleContact(world, 5, true, false) > .5f,
        "Characters without an active capsule retain their ordinary PA-to-PA collision response");
    std::puts("PASS: physical capsule settings, phase ordering, upper-body queries, mass replacement and repeated on/off restoration");
}
