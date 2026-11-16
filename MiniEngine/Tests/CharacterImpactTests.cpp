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
#include <iterator>

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

    float SolveCapsuleContact(PhysicsWorld& service, float collisionMass, bool upperBody, bool enabled = true)
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
        CheckImpact(source.Create(world, asset, names, poses, Vector3(2, 0, 0), error), "Create a finite-mass source body");
        source.SetQueryOwner(&sourceOwner);
        source.SetCharacterImpactEnabled(true, 101);
        CheckImpact(source.ConfigureCharacterCapsule(enabled, collisionMass, 1, .01f, error), "Configure an independent dynamic capsule");
        const float activeMass = enabled ? source.GetCharacterCapsuleMass() : source.GetImpactRootMass();
        Vector3 center(kZero);
        float totalMass = 0;
        CheckImpact(source.GetImpactRootMass() == 2 && source.GetCenterOfMass(center, totalMass) && totalMass == 2,
            "Collision mass does not change the source PA mass or total weight");
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
        // 한 번만 동기화한 뒤 두 동적 바디의 유한 질량 충돌을 검사한다.
        source.SyncCharacterCapsule();
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
        CheckImpact(source.GetCenterOfMassVelocity(sourceVelocity), "Measure the source PA velocity separately from its capsule");
        const float targetVelocity = target->getLinearVelocity().x;
        if (upperBody)
        {
            // 중심을 지나는 비탄성 충돌의 해와 비교한다. 기존 선형 감쇠의 작은 영향만 허용한다.
            const float expectedVelocity = 2 * activeMass / (activeMass + 1);
            CheckImpact(std::abs(targetVelocity - expectedVelocity) < .08f && (enabled ? measuredImpulse > .1f : measuredImpulse == 0),
                "PhysX uses the configured finite collision mass without manual forces");
            if (enabled)
            {
                CheckImpact(float(sourceVelocity.GetX()) > 1.97f && source.GetExternalImpactImpulse() == 0,
                    "Capsule recoil does not feed back into the source PA velocity or recovery input");
            }
            else
            {
                CheckImpact(std::abs(activeMass * float(sourceVelocity.GetX()) + targetVelocity - 2 * activeMass) < .12f,
                    "Ordinary PA collision still exchanges momentum when the capsule is disabled");
            }
        }
        else
        {
            CheckImpact(std::abs(targetVelocity) < 1e-5f && measuredImpulse == 0,
                "Lower-body filter prevents simulation contact and momentum transfer");
        }
        CheckImpact(world.GetErrorCount() == 0 && source.IsFinite(), "Finite collision state with no PhysX errors");
        source.SyncCharacterCapsule();
        CheckImpact(source.GetCenterOfMass(center, totalMass) && totalMass == 2 && source.GetImpactRootMass() == 2,
            "Following the source again after collision preserves PA mass");
        if (enabled)
        {
            PxTransform capsulePose(PxIdentity);
            PxVec3 targetOffset(0);
            float radius = 0;
            float halfLength = 0;
            CheckImpact(source.GetCharacterImpactCapsule(1, capsulePose, radius, halfLength, targetOffset) &&
                float(Length(Vector3(capsulePose.p.x, capsulePose.p.y, capsulePose.p.z) - source.GetPosition())) < 1e-5f,
                "The independent capsule follows the root again after receiving solver recoil");
        }
        std::printf("collision mass=%.3f enabled=%d upper=%d receiver_vx=%.6f source_PA_vx=%.6f measured_J=%.6f\n",
            activeMass, enabled ? 1 : 0, upperBody ? 1 : 0, targetVelocity, float(sourceVelocity.GetX()), measuredImpulse);
        target->release();
        return targetVelocity;
    }

    void TestKinematicCapsule(PhysicsWorld& service)
    {
        PhysicsWorld world;
        CheckImpact(world.InitializeScene(service), "Initialize kinematic capsule contact scene");
        PhysicsAsset asset;
        PhysicsBody root;
        root.bone = "Hips";
        root.shape = PhysicsShapeType::Box;
        root.halfExtents = DirectX::XMFLOAT3(.1f, .3f, .1f);
        root.mass = 2;
        root.material.restitution = 0;
        root.material.staticFriction = 0;
        root.material.dynamicFriction = 0;
        asset.bodies.push_back(root);
        const std::vector<std::string> names{"Hips"};
        std::vector<Matrix4> poses{Matrix4(OrthogonalTransform(Quaternion(kIdentity), Vector3(0, 3, 0)))};
        Ragdoll source;
        std::string error;
        int sourceOwner = 1;
        int targetOwner = 2;
        CheckImpact(source.Create(world, asset, names, poses, Vector3(kZero), error, false), "Create kinematic PA source");
        source.SetQueryOwner(&sourceOwner);
        source.SetCharacterImpactEnabled(true, 101);
        CheckImpact(source.ConfigureCharacterCapsule(true, 30, 1.15f, .02f, error, 1.14f),
            "Kinematic player accepts the same collision mass and size settings as ragdoll");
        std::printf("kinematic setup bodies=%zu capsule_mass=%.9f PA_mass=%.9f\n",
            source.GetKinematicBodyCount(), source.GetCharacterCapsuleMass(), source.GetImpactRootMass());
        CheckImpact(source.GetKinematicBodyCount() == 1 && std::abs(source.GetCharacterCapsuleMass() - 30) < .0001f &&
            std::abs(source.GetImpactRootMass() - 2) < .00001f,
            "Thirty-kg capsule preserves the kinematic PA and its original mass");
        PxTransform capsulePose(PxIdentity);
        PxVec3 targetOffset(0);
        float radius = 0;
        float halfLength = 0;
        CheckImpact(source.GetCharacterImpactCapsule(1.15f, capsulePose, radius, halfLength, targetOffset, 1.14f) &&
            std::abs(radius - .115f) < 1e-5f && std::abs(radius + halfLength - .342f) < 1e-5f,
            "Kinematic debug geometry matches the configured simulation dimensions");
        PxRigidDynamic* target = world.CreateDynamicBody(PxTransform(PxVec3(.28f, 3, 0)), PxSphereGeometry(.1f),
            5, "Kinematic capsule receiver", root.material);
        CheckImpact(target != nullptr, "Create dynamic capsule receiver");
        target->userData = &targetOwner;
        target->setActorFlag(PxActorFlag::eDISABLE_GRAVITY, true);
        PxShape* targetShape = nullptr;
        target->getShapes(&targetShape, 1);
        PxFilterData filter;
        filter.word2 = 102;
        filter.word3 = PhysicsWorld::CharacterImpactFilterFlag | PhysicsWorld::CharacterImpactTargetFilterFlag;
        targetShape->setSimulationFilterData(filter);
        float measuredImpulse = 0;
        for (int step = 0; step < 12; ++step)
        {
            const Vector3 destination((step + 1) * .04f, 3, 0);
            poses[0] = Matrix4(OrthogonalTransform(Quaternion(Vector3(kYUnitVector), (step + 1) * .02f), destination));
            CheckImpact(source.UpdateKinematicPose(poses, false, error), "Update animation position and rotation targets");
            source.SyncCharacterCapsule();
            world.Step();
            if (step == 0)
            {
                const PxQuat expectedRotation = PxQuat(.02f, PxVec3(0, 1, 0)) * PxQuat(PxHalfPi, PxVec3(0, 0, 1));
                CheckImpact(source.GetCharacterImpactCapsule(1.15f, capsulePose, radius, halfLength, targetOffset, 1.14f) &&
                    (capsulePose.p - PxVec3(.04f, 3, 0)).magnitude() < .0001f &&
                    PxAbs(capsulePose.q.dot(expectedRotation)) > .999999f,
                    "Capsule translation and rotation use the current target without a one-step delay");
            }
            CheckImpact(float(Length(source.GetPosition() - destination)) < 1e-5f && source.GetKinematicBodyCount() == 1,
                "Real solver contact never displaces the kinematic player from its animation target");
            std::vector<PhysicsCapsuleContact> contacts;
            source.GetCharacterCapsuleContacts(contacts);
            for (const PhysicsCapsuleContact& contact : contacts)
            {
                measuredImpulse += contact.normalImpulse;
            }
        }
        CheckImpact(measuredImpulse > 1 && target->getLinearVelocity().x > .5f,
            "Current-step animation motion drives actual finite-mass capsule contacts");
        std::printf("kinematic capsule mass=30 receiver_vx=%.6f measured_J=%.6f player_target_error=%.9f\n",
            target->getLinearVelocity().x, measuredImpulse, float(Length(source.GetPosition() - Vector3(.48f, 3, 0))));
        const Vector3 teleport(5, 4, 2);
        poses[0] = Matrix4(OrthogonalTransform(Quaternion(kIdentity), teleport));
        CheckImpact(source.UpdateKinematicPose(poses, true, error) &&
            source.GetCharacterImpactCapsule(1.15f, capsulePose, radius, halfLength, targetOffset, 1.14f) &&
            (capsulePose.p - PxVec3(5, 4, 2)).magnitude() < 1e-5f,
            "Paused teleport immediately moves the physical capsule and debug outline");
        source.SyncCharacterCapsule();
        world.Step();
        CheckImpact(source.GetCharacterImpactCapsule(1.15f, capsulePose, radius, halfLength, targetOffset, 1.14f) &&
            (capsulePose.p - PxVec3(5, 4, 2)).magnitude() < 1e-5f,
            "Teleport does not leave stale capsule velocity");
        CheckImpact(source.ConfigureCharacterCapsule(true, 45, 1.4f, .02f, error, 1.6f) &&
            source.GetCharacterImpactCapsule(1.4f, capsulePose, radius, halfLength, targetOffset, 1.6f) &&
            source.GetCharacterCapsuleMass() == 45 && std::abs(radius - .14f) < 1e-5f &&
            std::abs(radius + halfLength - .48f) < 1e-5f,
            "Kinematic mass and size edits update the existing shared settings path");
        CheckImpact(source.ConfigureCharacterCapsule(false, 45, 1.4f, .02f, error, 1.6f) && !source.HasCharacterCapsule(),
            "Kinematic capsule can be removed without enabling simulation");
        CheckImpact(world.GetErrorCount() == 0 && source.IsFinite(), "Kinematic capsule has finite state and no PhysX errors");
        target->release();
    }

    void CheckCapsuleMassProperties(const Ragdoll& ragdoll, const PxRigidBody& rootBody,
        const PxVec3& inertia, const PxTransform& localCenter, Vector3 center, float mass)
    {
        Vector3 currentCenter(kZero);
        float currentMass = 0;
        const PxTransform currentLocalCenter = rootBody.getCMassLocalPose();
        CheckImpact(ragdoll.GetImpactRootMass() == 3 && ragdoll.GetCenterOfMass(currentCenter, currentMass) &&
            std::abs(currentMass - mass) < 1e-6f && float(Length(currentCenter - center)) < 1e-6f &&
            (rootBody.getMassSpaceInertiaTensor() - inertia).magnitude() < 1e-6f &&
            (currentLocalCenter.p - localCenter.p).magnitude() < 1e-6f &&
            std::abs(currentLocalCenter.q.dot(localCenter.q)) > .999999f,
            "Independent capsule creation, mass changes, resize and removal preserve PA mass, inertia and center of mass");
    }
}

void RunCharacterImpactTests()
{
    CharacterImpactSettings settings;
    std::string error;
    settings.collisionMass = 20;
    settings.enabled = false;
    settings.capsuleHeightScale = 1.25f;
    const std::string profilePath = "CharacterImpactSettings." +
        std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()) + ".test.json";
    CheckImpact(settings.Save(profilePath, error), "Save independent impact profile");
    CharacterImpactSettings loaded;
    CheckImpact(loaded.Load(profilePath, error) && loaded.collisionMass == 20 && !loaded.enabled && loaded.capsuleHeightScale == 1.25f,
        "Reload impact profile");
    {
        std::ifstream saved(profilePath);
        const std::string content((std::istreambuf_iterator<char>(saved)), std::istreambuf_iterator<char>());
        CheckImpact(content.find("bodyMassKg") == std::string::npos && content.find("collisionMassKg") != std::string::npos,
            "Profiles explicitly store collision mass instead of a character mass override");
    }
    {
        std::ofstream legacy(profilePath);
        legacy << R"({"version":1,"enabled":true,"referenceImpulseNs":180,"referenceSpeedMps":3.5,
            "strengthMultiplier":1,"maximumImpulseNs":500,"minimumClosingSpeedMps":0.35,"pelvisShare":0.1,
            "separationTimeSeconds":0.1,"separationDistanceMeters":0.08,"cooldownSeconds":0.35,
            "capsuleRadiusScale":0.95,"contactSkinMeters":0.02})";
    }
    CheckImpact(loaded.Load(profilePath, error) && loaded.enabled && loaded.capsuleHeightScale == 1.0f &&
        loaded.collisionMass == CharacterImpactSettings().collisionMass,
        "Legacy profile discards manual impulses and retains collision settings");
    {
        std::ofstream legacy(profilePath);
        legacy << R"({"version":3,"enabled":false,"bodyMassKg":20,"capsuleRadiusScale":0.8,"contactSkinMeters":0.02})";
    }
    CheckImpact(loaded.Load(profilePath, error) && loaded.collisionMass == 20 && !loaded.enabled &&
        loaded.capsuleHeightScale == 1.0f && loaded.capsuleRadiusScale == .8f,
        "Version-three profiles retain their previous capsule dimensions with height scale one");
    {
        std::ofstream legacy(profilePath);
        legacy << R"({"version":4,"enabled":true,"bodyMassKg":9999,"capsuleRadiusScale":0.7,
            "capsuleHeightScale":1.5,"contactSkinMeters":0.02})";
    }
    CheckImpact(loaded.Load(profilePath, error) && loaded.collisionMass == 9999 && loaded.enabled && loaded.capsuleRadiusScale == .7f &&
        loaded.capsuleHeightScale == 1.5f && loaded.Save(profilePath, error),
        "Version-four profiles migrate their configured mass to the independent collision body");
    {
        std::ifstream saved(profilePath);
        const std::string content((std::istreambuf_iterator<char>(saved)), std::istreambuf_iterator<char>());
        CheckImpact(content.find("bodyMassKg") == std::string::npos && content.find("collisionMassKg") != std::string::npos,
            "Migrated profiles name the independent collision mass explicitly");
    }
    std::remove(profilePath.c_str());
    settings.capsuleHeightScale = 0;
    CheckImpact(!settings.Validate(error), "Reject zero capsule height scale");
    settings.capsuleHeightScale = 1;
    settings.capsuleRadiusScale = 0;
    CheckImpact(!settings.Validate(error), "Reject zero capsule radius scale");
    settings.capsuleRadiusScale = std::numeric_limits<float>::quiet_NaN();
    CheckImpact(!settings.Validate(error), "Reject nonfinite profile");
    settings.capsuleRadiusScale = .95f;
    settings.collisionMass = 0;
    CheckImpact(!settings.Validate(error), "Reject zero collision mass for a finite dynamic body");
    settings.collisionMass = std::numeric_limits<float>::quiet_NaN();
    CheckImpact(!settings.Validate(error), "Reject nonfinite collision mass");

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
    CheckImpact(rootBody != nullptr, "Locate the root mass and inertia for preservation checks");
    const PxVec3 originalInertia = rootBody->getMassSpaceInertiaTensor();
    const PxTransform originalLocalCenter = rootBody->getCMassLocalPose();
    Vector3 originalCenter(kZero);
    float originalMass = 0;
    CheckImpact(ragdoll.GetCenterOfMass(originalCenter, originalMass) && std::abs(originalMass - 10) < 1e-4f,
        "Record the ten-kg PA mass and center of mass before attaching the capsule");
    const uint32_t bodyCount = world.GetDynamicBodyCount();
    CheckImpact(ragdoll.ConfigureCharacterCapsule(true, 6, .95f, .02f, error), "Create an independent six-kg dynamic capsule");
    CheckImpact(ragdoll.HasCharacterCapsule() && ragdoll.GetCharacterCapsuleMass() == 6,
        "Independent capsule has the requested collision mass");
    CheckCapsuleMassProperties(ragdoll, *rootBody, originalInertia, originalLocalCenter, originalCenter, originalMass);
    CheckImpact(world.GetDynamicBodyCount() == bodyCount + 1 && rootBody->getNbShapes() == 1 && ragdoll.GetBodyCount() == names.size(),
        "Capsule uses a separate dynamic actor and never attaches a shape to the PA root");
    CheckImpact(ragdoll.ConfigureCharacterCapsule(true, 40, .95f, .02f, error) && ragdoll.GetCharacterCapsuleMass() == 40,
        "Change collision mass without rebuilding the character");
    CheckCapsuleMassProperties(ragdoll, *rootBody, originalInertia, originalLocalCenter, originalCenter, originalMass);
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
        rootBody->getNbShapes() == 1 && ragdoll.GetImpactRootMass() == 3 && ragdoll.GetCharacterCapsuleMass() == 6,
        "Height control changes the simulation shape while preserving radius, shape count and mass");
    CheckCapsuleMassProperties(ragdoll, *rootBody, originalInertia, originalLocalCenter, originalCenter, originalMass);
    CheckImpact(ragdoll.ConfigureCharacterCapsule(true, 6, 3, .02f, error, .1f), "Allow a short, wide capsule");
    CheckImpact(ragdoll.GetCharacterImpactCapsule(3, capsulePose, changedRadius, changedHalfLength, targetOffset) &&
        changedHalfLength == 0 && changedRadius > initialRadius,
        "A requested height smaller than the diameter becomes a valid spherical capsule");
    CheckCapsuleMassProperties(ragdoll, *rootBody, originalInertia, originalLocalCenter, originalCenter, originalMass);
    CheckImpact(!ragdoll.ConfigureCharacterCapsule(true, 6, .95f, .02f, error, 0) && rootBody->getNbShapes() == 1 &&
        world.GetDynamicBodyCount() == bodyCount + 1,
        "Invalid resize leaves the current simulation shape intact");
    CheckImpact(ragdoll.ConfigureCharacterCapsule(loaded.enabled, loaded.collisionMass, loaded.capsuleRadiusScale, loaded.contactSkin,
        error, loaded.capsuleHeightScale) && ragdoll.GetCharacterCapsuleMass() == 9999,
        "Apply migrated 9999-kg setting only to the independent collision body");
    CheckCapsuleMassProperties(ragdoll, *rootBody, originalInertia, originalLocalCenter, originalCenter, originalMass);
    CheckImpact(ragdoll.ConfigureCharacterCapsule(true, 6, .95f, .02f, error), "Restore default capsule size");
    for (int cycle = 0; cycle < 20; ++cycle)
    {
        CheckImpact(ragdoll.ConfigureCharacterCapsule(false, 6, .95f, .02f, error) &&
            !ragdoll.HasCharacterCapsule() && rootBody->getNbShapes() == 1 && world.GetDynamicBodyCount() == bodyCount,
            "Turning the capsule off removes only the independent actor");
        CheckCapsuleMassProperties(ragdoll, *rootBody, originalInertia, originalLocalCenter, originalCenter, originalMass);
        CheckImpact(ragdoll.ConfigureCharacterCapsule(true, 6, .95f, .02f, error), "Re-enable without recreating the articulation");
        CheckImpact(rootBody->getNbShapes() == 1 && world.GetDynamicBodyCount() == bodyCount + 1,
            "Repeated toggles retain exactly one independent capsule actor");
        CheckCapsuleMassProperties(ragdoll, *rootBody, originalInertia, originalLocalCenter, originalCenter, originalMass);
    }
    CheckImpact(!ragdoll.ConfigureCharacterCapsule(true, 6, 0, .02f, error) && ragdoll.HasCharacterCapsule(),
        "Invalid runtime geometry leaves the existing capsule intact");
    CheckImpact(!ragdoll.ConfigureCharacterCapsule(true, 0, .95f, .02f, error) && ragdoll.GetCharacterCapsuleMass() == 6,
        "Invalid runtime collision mass preserves the existing actor and mass");
    CheckImpact(ragdoll.SetSimulating(false, Vector3(kZero), error) && !ragdoll.HasCharacterCapsule(),
        "Leaving simulation removes the capsule before storing a standby articulation");
    CheckImpact(ragdoll.SetSimulating(true, Vector3(kZero), error) && ragdoll.GetImpactRootMass() == 3,
        "Reactivating the articulation preserves its PA mass without a stale capsule");
    ragdoll.SetCharacterImpactEnabled(true, 100);
    CheckImpact(ragdoll.ConfigureCharacterCapsule(true, 6, .95f, .02f, error),
        "A capsule can be enabled again after switching physics configurations");
    ragdoll.SetCharacterImpactEnabled(false, 100);
    world.SweepCharacterCapsule(PxTransform(PxVec3(0, 3, 0)), 2, .1f, PxVec3(0), nullptr, hits);
    CheckImpact(hits.empty(), "Disabling body impact clears upper-body query flags");
    CheckImpact(!ragdoll.HasCharacterCapsule() && ragdoll.GetImpactRootMass() == 3,
        "Disabling collision policy removes the physical capsule and preserves PA mass");
    CheckImpact(world.GetErrorCount() == 0 && ragdoll.IsFinite(), "No PhysX validation errors or nonfinite state");
    const float lightResponse = SolveCapsuleContact(world, .5f, true);
    const float heavyResponse = SolveCapsuleContact(world, 5, true);
    CheckImpact(heavyResponse > lightResponse + .5f, "A heavier independent capsule transfers more impulse with the same PA mass");
    SolveCapsuleContact(world, 5, false);
    CheckImpact(SolveCapsuleContact(world, 2, true, false) > .5f,
        "Characters without an active capsule retain their ordinary PA-to-PA collision response");
    TestKinematicCapsule(world);
    std::puts("PASS: independent capsule mass, actual solver contacts, isolated recoil, PA mass/COM/inertia and repeated on/off");
}
