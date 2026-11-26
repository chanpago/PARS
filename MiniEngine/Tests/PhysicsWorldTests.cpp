#include "PhysicsWorld.h"
#include "../Model/FallArmBrace.h"
#include "../Model/FootSupport.h"
#include "../Model/FootSupportHull.h"

#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <functional>

using namespace physx;

void TestFootstepPlanner();
void TestProjectileStagger();
void TestFootstepFalls();
void TestImpactWarmupPlanning();
void TestFootstepIK();
void TestFootstepCollision();
void TestStandingSupport();
void TestHorizontalBalance();

namespace
{
    void Check(bool condition, const char* message)
    {
        if (!condition)
        {
            std::fprintf(stderr, "FAIL: %s\n", message);
            std::exit(1);
        }
    }

    void TestFootProjectionRaycast()
    {
        PhysicsWorld world;
        Check(world.Initialize(false), "Initialize foot projection scene");
        physx::PxRigidStatic* floor = world.CreateStaticBox(PxTransform(PxVec3(0, -.25f, 0)),
            PxVec3(2, .25f, 2), "Projection floor");
        Check(floor != nullptr, "Create projection floor");
        PxRaycastHit hit;
        const PxVec3 down(0, -.2f, 0);
        Check(world.Raycast(PxVec3(1, .1f, 0), down, hit) && hit.actor == floor &&
                std::abs(hit.distance - .1f) < 1e-5f && std::abs(hit.position.y) < 1e-5f,
            "Foot point within 20 cm projects to the real floor");
        Check(!world.Raycast(PxVec3(1, .201f, 0), down, hit),
            "Foot point above 20 cm has no projection");
        Check(!world.Raycast(PxVec3(3, .1f, 0), down, hit),
            "No imaginary floor beyond a platform edge");
        int owner = 0;
        physx::PxRigidDynamic* ownBody = world.CreateDynamicBox(PxTransform(PxVec3(0, .1f, 0)),
            PxVec3(.03f), 1, "Own foot");
        Check(ownBody != nullptr, "Create own foot query obstacle");
        ownBody->userData = &owner;
        Check(world.Raycast(PxVec3(0, .19f, 0), down, hit) && hit.actor == ownBody,
            "Raycast normally sees the nearest rigid body");
        Check(world.Raycast(PxVec3(0, .19f, 0), down, hit, &owner) && hit.actor == floor,
            "Foot projection ignores its own character and finds the ground");
        Check(world.GetErrorCount() == 0, "Foot projection query has no PhysX errors");
        std::puts("PASS: foot projection raycast range, platform edge and owner filtering");
    }

    void TestApproachingImpact()
    {
        PhysicsWorld world;
        Check(world.Initialize(false), "Initialize collision activation scene");
        PxRigidDynamic* target = world.CreateDynamicBody(PxTransform(PxVec3(0, 3, 0)), PxSphereGeometry(.1f), 73, "Kinematic target");
        PxRigidDynamic* source = world.CreateDynamicBody(PxTransform(PxVec3(-.3f, 3, 0)), PxSphereGeometry(.1f), 2, "Incoming source");
        Check(target && source, "Create collision activation bodies");
        target->setRigidBodyFlag(PxRigidBodyFlag::eKINEMATIC, true);
        source->setLinearVelocity(PxVec3(7.5f, 0, 0));
        Check(world.HasApproachingImpact(*target, 73, .05f, nullptr), "An approaching impact activates before the solver treats the receiver as infinite mass");
        Check(!world.HasApproachingImpact(*target, 73, 10.0f, nullptr), "Subthreshold incoming momentum does not activate");
        source->setLinearVelocity(PxVec3(-7.5f, 0, 0));
        Check(!world.HasApproachingImpact(*target, 73, .05f, nullptr), "A departing projectile does not activate");
        source->setLinearVelocity(PxVec3(7.5f, 0, 0));
        source->setGlobalPose(PxTransform(PxVec3(-.3f, 4, 0)));
        Check(!world.HasApproachingImpact(*target, 73, .05f, nullptr), "An incoming path that misses the body does not activate");
        source->setGlobalPose(PxTransform(PxVec3(-.3f, 3, 0)));
        int owner = 0;
        source->userData = &owner;
        Check(!world.HasApproachingImpact(*target, 73, .05f, &owner), "Self-owned bodies cannot trigger recovery");
        source->userData = nullptr;
        PxShape* shape = nullptr;
        source->getShapes(&shape, 1);
        shape->setFlag(PxShapeFlag::eSIMULATION_SHAPE, false);
        Check(!world.HasApproachingImpact(*target, 73, .05f, nullptr), "Non-colliding shapes cannot trigger recovery");
        Check(world.GetStepCount() == 0 && world.GetErrorCount() == 0, "Prediction does not simulate or introduce PhysX errors");
        shape->setFlag(PxShapeFlag::eSIMULATION_SHAPE, true);
        PxFilterData filter = shape->getSimulationFilterData();
        filter.word3 |= PhysicsWorld::ProjectileFilterFlag;
        shape->setSimulationFilterData(filter);
        Check(world.SetProjectileBodyImpulse(*source, 25.0f, .5f, 10.0f), "Set independent body and part impulses on the projectile");
        Check(!world.SetProjectileBodyImpulse(*source, 0, 0, -1), "Reject a negative part impulse without replacing the settings");
        std::vector<PhysicsContactImpulse> contacts;
        float strength = 0;
        for (int step = 0; step < 6 && strength <= 1e-4f; ++step)
        {
            world.Step();
            strength = world.GetExternalImpactImpulse(*target, nullptr, &contacts);
        }
        Check(!contacts.empty(), "Fallback captures a real kinematic contact");
        const uint64_t firstHit = contacts.front().impactContactId;
        PhysicsImpactResponse response;
        Check(firstHit != 0 && world.GetImpactResponse(firstHit, response) && world.ConsumeImpactContact(firstHit) &&
            !world.ConsumeImpactContact(firstHit),
            "The first projectile contact can be consumed once per receiver");
        Check(std::abs(response.impulse.magnitude() - 25.0f) < .001f && response.impulse.x > 24.9f && response.torqueMultiplier == .5f,
            "The whole-body impulse uses the shot direction and configured magnitude");
        Check((response.partImpulse - response.impulse * (10.0f / 25.0f)).magnitude() < .001f,
            "The part impulse uses the incoming direction and its own magnitude");
        PxVec3 impulse(0);
        for (const PhysicsContactImpulse& contact : contacts)
        {
            impulse += contact.impulse;
            Check(contact.newProjectileHit && contact.outwardNormal.x < -.9f,
                "First ball impulse supplies the outward normal on the struck surface");
        }
        Check(impulse.x > 0 && impulse.isFinite(), "Captured fallback impulse points into the receiver");
        source->setActorFlag(PxActorFlag::eDISABLE_GRAVITY, true);
        bool sustained = false;
        for (int step = 0; step < 4; ++step)
        {
            const PxVec3 towardTarget = (target->getGlobalPose().p - source->getGlobalPose().p).getNormalized();
            source->setLinearVelocity(towardTarget * 2.0f);
            world.Step();
            sustained |= world.GetExternalImpactImpulse(*target, nullptr, &contacts) > 1e-4f;
            for (const PhysicsContactImpulse& contact : contacts)
            {
                Check(!contact.newProjectileHit, "A ball held against the body does not restart the gesture");
                Check(contact.impactContactId == firstHit && !world.ConsumeImpactContact(contact.impactContactId),
                    "A sustained projectile contact cannot repeat the whole-body impulse");
            }
        }
        Check(sustained, "The pressed ball continues transferring impulse");
        source->setGlobalPose(PxTransform(PxVec3(-.3f, 3, 0)));
        source->setLinearVelocity(PxVec3(0));
        world.Step();
        source->setLinearVelocity(PxVec3(7.5f, 0, 0));
        strength = 0;
        for (int step = 0; step < 6 && strength <= 1e-4f; ++step)
        {
            world.Step();
            strength = world.GetExternalImpactImpulse(*target, nullptr, &contacts);
        }
        Check(strength > 0 && !contacts.empty() && contacts.front().impactContactId != firstHit &&
            world.GetImpactResponse(contacts.front().impactContactId, response), "A separated ball can hit the same receiver again");
        std::puts("PASS: approaching collision, threshold, departing/missing source, ownership and collision policy");
    }

    void TestSharedImpactContact(bool capsule)
    {
        PhysicsWorld world;
        Check(world.Initialize(false), "Initialize shared projectile contact test");
        int owner = 0;
        std::array<PxRigidDynamic*, 2> targets{};
        for (size_t side = 0; side < targets.size(); ++side)
        {
            const float x = side == 0 ? -.21f : .21f;
            targets[side] = world.CreateDynamicBody(PxTransform(PxVec3(x, 3, 0)), PxBoxGeometry(.20f, .2f, .1f), 2, "Receiver part");
            Check(targets[side] != nullptr, "Create receiver part");
            targets[side]->setRigidBodyFlag(PxRigidBodyFlag::eKINEMATIC, true);
            targets[side]->userData = &owner;
            PxShape* targetShape = nullptr;
            targets[side]->getShapes(&targetShape, 1);
            targetShape->setSimulationFilterData(PxFilterData(0, 0, 2, PhysicsWorld::CharacterImpactTargetFilterFlag));
        }
        const PxSphereGeometry sphere(.12f);
        const PxCapsuleGeometry torso(.12f, .08f);
        const PxGeometry& geometry = capsule ? static_cast<const PxGeometry&>(torso) : static_cast<const PxGeometry&>(sphere);
        PxRigidDynamic* ball = world.CreateDynamicBody(PxTransform(PxVec3(0, 3, -.4f)), geometry, 1, "Shared hit");
        Check(ball && world.SetImpactImpulseSettings(*ball, {8, 0, 0}), "Configure one local impulse against two parts");
        ball->setActorFlag(PxActorFlag::eDISABLE_GRAVITY, true);
        PxShape* shape = nullptr;
        Check(ball->getShapes(&shape, 1) == 1, "Find shared projectile shape");
        PxFilterData filter = shape->getSimulationFilterData();
        filter.word2 = 1;
        filter.word3 |= capsule ? PhysicsWorld::CharacterCapsuleFilterFlag : PhysicsWorld::ProjectileFilterFlag;
        shape->setSimulationFilterData(filter);
        ball->setLinearVelocity(PxVec3(0, 0, 4));
        std::array<std::vector<PhysicsContactImpulse>, 2> contacts;
        for (int step = 0; step < 10; ++step)
        {
            world.Step();
            const float firstImpulse = world.GetExternalImpactImpulse(*targets[0], &owner, &contacts[0]);
            const float secondImpulse = world.GetExternalImpactImpulse(*targets[1], &owner, &contacts[1]);
            if (firstImpulse + secondImpulse > 1e-4f && !contacts[0].empty() && !contacts[1].empty())
            {
                break;
            }
        }
        Check(!contacts[0].empty() && !contacts[1].empty(), "The projectile touches both parts");
        const uint64_t first = contacts[0].front().impactContactId;
        const uint64_t second = contacts[1].front().impactContactId;
        PhysicsImpactResponse response;
        Check(first != 0 && first == second && world.GetImpactResponse(first, response) && world.ConsumeImpactContact(first) &&
            !world.ConsumeImpactContact(second) && response.impulse.isZero() && std::abs(response.partImpulse.z - 8) < .001f,
            "Deliver one local impulse across multiple receiver shapes");
        if (capsule)
        {
            Check(contacts[0].front().fromCharacterCapsule && !contacts[0].front().newProjectileHit &&
                contacts[0].front().newCapsuleHit && contacts[1].front().newCapsuleHit,
                "A capsule impact keeps its source type");
            targets[0]->setGlobalPose(PxTransform(PxVec3(-2, 3, 0)));
            Check(world.SetImpactImpulseSettings(*ball, {3, 12, .5f}), "Edit the capsule preset during a push");
            for (int step = 0; step < 4; ++step)
            {
                // 실제 몸통 동기화처럼 매 스텝 위치와 속도를 갱신하고 다른 본만 계속 누른다.
                world.InvalidateContacts(*ball);
                ball->setGlobalPose(PxTransform(PxVec3(.21f, 3, -.215f)));
                ball->setLinearVelocity(PxVec3(0, 0, 2));
                world.Step();
                Check(world.GetExternalImpactImpulse(*targets[1], &owner, &contacts[1]) > 0 && !contacts[1].empty() &&
                    contacts[1].front().impactContactId == first && !contacts[1].front().newCapsuleHit && !world.GetImpactResponse(first, response),
                    "Continuing the push on one spine cannot restart the configured impulse");
            }
            // 한 스텝에 이전 본의 접촉이 끝나고 다른 본에서 새 형상 쌍이 생긴다.
            targets[1]->setGlobalPose(PxTransform(PxVec3(2, 3, 0)));
            targets[0]->setGlobalPose(PxTransform(PxVec3(.21f, 3, 0)));
            world.InvalidateContacts(*ball);
            ball->setGlobalPose(PxTransform(PxVec3(.21f, 3, -.215f)));
            ball->setLinearVelocity(PxVec3(0, 0, 2));
            world.Step();
            Check(world.GetExternalImpactImpulse(*targets[0], &owner, &contacts[0]) > 0 && !contacts[0].empty() &&
                contacts[0].front().impactContactId == first && !contacts[0].front().newCapsuleHit && !world.GetImpactResponse(first, response),
                "Changing the only touching spine in one step keeps the same encounter");
            ball->setGlobalPose(PxTransform(PxVec3(.21f, 3, -2)));
            ball->setLinearVelocity(PxVec3(0));
            world.Step();
            ball->setGlobalPose(PxTransform(PxVec3(.21f, 3, -.215f)));
            ball->setLinearVelocity(PxVec3(0, 0, 2));
            world.Step();
            Check(world.GetExternalImpactImpulse(*targets[0], &owner, &contacts[0]) > 0 && !contacts[0].empty() &&
                contacts[0].front().impactContactId != first && contacts[0].front().newCapsuleHit &&
                world.GetImpactResponse(contacts[0].front().impactContactId, response) &&
                std::abs(response.partImpulse.z - 3) < .01f && std::abs(response.impulse.z - 12) < .01f && response.torqueMultiplier == .5f,
                "A new contact after separation uses the edited capsule settings");
            Check(world.SetImpactImpulseSettings(*ball, {}), "Disable additional capsule impulses");
            ball->setGlobalPose(PxTransform(PxVec3(.21f, 3, -2)));
            ball->setLinearVelocity(PxVec3(0));
            world.Step();
            ball->setGlobalPose(PxTransform(PxVec3(.21f, 3, -.215f)));
            ball->setLinearVelocity(PxVec3(0, 0, 2));
            world.Step();
            Check(world.GetExternalImpactImpulse(*targets[0], &owner, &contacts[0]) > 0 && !contacts[0].empty() &&
                contacts[0].front().newCapsuleHit && !world.GetImpactResponse(contacts[0].front().impactContactId, response),
                "A capsule's first solver impulse signals a hit even with both extra impulses disabled");
        }
        std::puts(capsule ? "PASS: capsule impact, continuous push and recontact" : "PASS: shared projectile impact");
    }

    void TestProjectileAttachment()
    {
        PhysicsWorld world;
        Check(world.Initialize(false), "Initialize attachment test");
        int owner = 0;
        PxRigidDynamic* target = world.CreateDynamicBody(PxTransform(PxVec3(0, 3, 0)), PxBoxGeometry(.2f, .2f, .1f), 2, "Head");
        PxRigidDynamic* arrow = world.CreateDynamicBody(PxTransform(PxVec3(.06f, 3.04f, -.65f)), PxSphereGeometry(.01f), .5f, "Tip");
        Check(target && arrow, "Create arrow and receiver");
        target->userData = &owner;
        target->setRigidBodyFlag(PxRigidBodyFlag::eKINEMATIC, true);
        arrow->setActorFlag(PxActorFlag::eDISABLE_GRAVITY, true);
        arrow->setRigidBodyFlag(PxRigidBodyFlag::eENABLE_CCD, true);
        arrow->setMinCCDAdvanceCoefficient(0);
        PxShape* shape = nullptr;
        Check(arrow->getShapes(&shape, 1) == 1, "Find arrow tip");
        PxFilterData filter = shape->getSimulationFilterData();
        filter.word3 |= PhysicsWorld::ProjectileFilterFlag | PhysicsWorld::AttachOnContactFilterFlag;
        shape->setSimulationFilterData(filter);
        Check(world.SetProjectileBodyImpulse(*arrow, 0, 0, 4), "Set arrow hit response");
        arrow->setLinearVelocity(PxVec3(0, 0, 100));
        for (int step = 0; step < 4 && !world.GetProjectileAttachment(*arrow); ++step)
        {
            world.Step();
        }
        const PhysicsProjectileAttachment* hit = world.GetProjectileAttachment(*arrow);
        Check(hit && hit->target == target && hit->targetOwner == &owner && hit->targetName == "Head", "Capture the first struck body");
        Check((hit->localPose.p - PxVec3(.06f, .04f, -.1f)).magnitude() < .025f && hit->localPose.q.getBasisVector0().z > .99f,
            "Fast arrow keeps the surface point and incoming direction in body space");
        world.Step([&world, target, &owner](float) -> void
        {
            std::vector<PhysicsContactImpulse> contacts;
            PhysicsImpactResponse response;
            Check(world.GetExternalImpactImpulse(*target, &owner, &contacts) > 0 && !contacts.empty() && contacts.front().newProjectileHit &&
                world.GetImpactResponse(contacts.front().impactContactId, response) && std::abs(response.partImpulse.z - 4) < .01f,
                "The receiver processes the original hit before the arrow stops simulating");
        });
        Check(arrow->getActorFlags().isSet(PxActorFlag::eDISABLE_SIMULATION) && !shape->getFlags().isSet(PxShapeFlag::eSCENE_QUERY_SHAPE),
            "Attached arrow no longer collides or blocks the next shot");
        target->release();
        hit = world.GetProjectileAttachment(*arrow);
        Check(hit && hit->target == nullptr && hit->targetName == "Head", "Keep the attachment identity across body replacement");
        arrow->release();
        Check(world.GetErrorCount() == 0, "Release arrow and target without PhysX errors");
        std::puts("PASS: fast arrow attachment, impact delivery and body replacement");
    }

    void TestClavicleReach()
    {
        const std::vector<int32_t> parents{-1, 0, 1, 2, 3, 0, 5, 6, 7};
        std::vector<PxTransform> rest(9, PxTransform(PxIdentity));
        std::array<PARS::FallArmSample, 2> arms;
        for (int side = 0; side < 2; ++side)
        {
            const float sign = side == 0 ? 1.0f : -1.0f;
            PARS::FallArmSetup& setup = arms[side].setup;
            setup.available = true;
            setup.chestNode = 0;
            setup.clavicleJoint.index = side;
            setup.clavicleNode = 1 + 4 * side;
            setup.anchorNode = setup.clavicleNode;
            setup.shoulderNode = setup.clavicleNode + 1;
            setup.elbowNode = setup.clavicleNode + 2;
            setup.handNode = setup.clavicleNode + 3;
            setup.rightInAnchor = PxVec3(-1, 0, 0);
            setup.palmInHand = PxTransform(PxVec3(0, .025f, 0));
            rest[setup.clavicleNode].p = PxVec3(.04f * sign, 0, 0);
            setup.clavicleJoint.inParentBone = rest[setup.clavicleNode];
            rest[setup.shoulderNode].p = PxVec3(.18f * sign, 0, 0);
            rest[setup.elbowNode].p = PxVec3(.22f * sign, -.28f, .02f);
            rest[setup.handNode].p = PxVec3(.2f * sign, -.55f, .07f);
            setup.upperLength = (rest[setup.elbowNode].p - rest[setup.shoulderNode].p).magnitude();
            setup.lowerLength = (rest[setup.handNode].p - rest[setup.elbowNode].p).magnitude();
        }
        const PxTransform tilted(PxVec3(1, -2, 3), PxQuat(.9f, PxVec3(1, 2, 3).getNormalized()));
        for (int side = 0; side < 2; ++side)
        {
            const PARS::FallArmSetup& setup = arms[side].setup;
            const float sign = side == 0 ? 1.0f : -1.0f;
            for (const PxVec3& hit : {PxVec3(-sign * .11f, -.12f, .14f), PxVec3(-sign * .18f, -.18f, .01f),
                PxVec3(sign * .13f, -.12f, -.14f), PxVec3(0, -.9f, -.12f)})
            {
                std::vector<PxTransform> uprightPose;
                for (const PxTransform& body : {PxTransform(PxIdentity), tilted})
                {
                    for (PARS::FallArmSample& arm : arms)
                    {
                        arm.actualChest = body;
                        arm.actualAnchor = body * rest[arm.setup.anchorNode];
                        arm.actualHand = body * rest[arm.setup.handNode];
                    }
                    PARS::ImpactPalmGoal goal;
                    goal.weight = 1;
                    goal.approach = 1;
                    goal.hand = side;
                    goal.hitNode = 0;
                    goal.point = body.transform(hit);
                    goal.normal = body.q.rotate(PxVec3(hit.x, 0, hit.z).getNormalized());
                    goal.limitJointRotation = [](int, const PxQuat& requested) -> PxQuat
                    {
                        PxVec3 axis;
                        float angle = 0;
                        requested.toRadiansAndUnitAxis(angle, axis);
                        return PxQuat(PxMin(angle, .075f), axis);
                    };
                    std::vector<PxTransform> pose = rest;
                    PARS::FallArmSnapshot result;
                    Check(PARS::ApplyImpactPalmTarget(goal, arms, parents, pose, result), "Solve a clavicle and arm together");
                    const float angle = pose[setup.clavicleNode].q.getAngle();
                    const PxVec3 wrist = body.transformInv((result.arms[side].palm * setup.palmInHand.getInverse()).p);
                    Check(angle > .001f && angle <= .0751f && pose[setup.shoulderNode].p.z * wrist.z > 0,
                        "The limited clavicle carries the shoulder toward the front or back");
                    const int opposite = arms[1 - side].setup.clavicleNode;
                    Check(pose[opposite] == rest[opposite] && !result.arms[1 - side].applied,
                        "Only the reaching side moves");
                    const PxVec3 palm = (body * pose[setup.handNode] * setup.palmInHand).p;
                    Check((palm - result.arms[side].solvedPalm).magnitude() < 1e-5f, "Report the palm in the chest frame");
                    const bool reachable = hit.y > -.8f;
                    Check(result.arms[side].reachLimited != reachable &&
                        (!reachable || (palm - goal.point - goal.normal * .006f).magnitude() < 1e-4f) &&
                        PxAbs((pose[setup.elbowNode].p - pose[setup.shoulderNode].p).magnitude() - setup.upperLength) < 1e-5f &&
                        PxAbs((pose[setup.handNode].p - pose[setup.elbowNode].p).magnitude() - setup.lowerLength) < 1e-5f,
                        "Reach the moving wound or stop at full arm length");
                    if (uprightPose.empty())
                    {
                        uprightPose = pose;
                    }
                    else
                    {
                        for (size_t node = 0; node < pose.size(); ++node)
                        {
                            Check((pose[node].p - uprightPose[node].p).magnitude() < 1e-4f &&
                                PxAbs(pose[node].q.dot(uprightPose[node].q)) > .99999f, "Follow the tilted chest");
                        }
                    }
                    goal.weight = .5f;
                    pose = rest;
                    Check(PARS::ApplyImpactPalmTarget(goal, arms, parents, pose, result) &&
                        PxAbs(pose[setup.clavicleNode].q.getAngle() - angle * .5f) < 1e-4f,
                        "Blend the clavicle once during approach and release");
                    const float fraction = goal.clavicleReachFraction;
                    goal.clavicleReachFraction = 0;
                    pose = rest;
                    Check(PARS::ApplyImpactPalmTarget(goal, arms, parents, pose, result), "Solve with zero clavicle assistance");
                    goal.clavicleReachFraction = fraction;
                    std::vector<PxTransform> fallback = rest;
                    arms[side].setup.clavicleJoint.index = -1;
                    Check(PARS::ApplyImpactPalmTarget(goal, arms, parents, fallback, result) && pose == fallback,
                        "A rig without an independent clavicle joint uses the existing anchor");
                    arms[side].setup.clavicleJoint.index = side;
                    goal.limitJointRotation = {};
                    fallback = rest;
                    Check(PARS::ApplyImpactPalmTarget(goal, arms, parents, fallback, result) && pose == fallback &&
                        pose[setup.clavicleNode] == rest[setup.clavicleNode], "Match the original arm solve for a short gesture");
                }
            }
            PARS::ImpactPalmGoal sweep;
            sweep.hand = side;
            sweep.hitNode = 0;
            sweep.approach = 1;
            sweep.weight = 1;
            sweep.limitJointRotation = [](int, const PxQuat& requested) -> PxQuat
            {
                return requested;
            };
            PxQuat previousUpper(PxIdentity);
            PxVec3 previousElbow(0);
            for (int frame = 0; frame <= 60; ++frame)
            {
                const float angle = PxPi * static_cast<float>(frame) / 60.0f;
                const PxVec3 normal(-sign * std::sin(angle), 0, std::cos(angle));
                sweep.point = tilted.transform(PxVec3(normal.x * .16f, -.14f, normal.z * .12f));
                sweep.normal = tilted.q.rotate(normal);
                std::vector<PxTransform> pose = rest;
                PARS::FallArmSnapshot result;
                Check(PARS::ApplyImpactPalmTarget(sweep, arms, parents, pose, result), "Follow a point moving around the torso");
                const PxQuat upper = pose[setup.shoulderNode].q;
                const PxVec3 elbow = pose[setup.elbowNode].p;
                if (frame > 0)
                {
                    Check(PxAbs(upper.dot(previousUpper)) > .99f && (elbow - previousElbow).magnitude() < .04f,
                        "Keep the upper arm and elbow continuous across the side of the torso");
                }
                previousUpper = upper;
                previousElbow = elbow;
            }
        }
        std::puts("PASS: mirrored clavicle reach, joint limit, chest rotation, blending and fallback");
    }

    void TestWoundHandSelection(const std::vector<int32_t>& parents, const std::vector<PxTransform>& rest,
        std::array<PARS::FallArmSample, 2> arms)
    {
        struct Surface
        {
        public:
            PxVec3 point;
            PxVec3 normal;
            int handForLeftHit;
        };

        // 이 리그에서 오른쪽은 +X, 정면은 -Z다. 평평한 가슴과 등은 좌우의 법선이 같다.
        const Surface surfaces[] = {
            {PxVec3(-.10f, -.12f, -.08f), PxVec3(0, 0, -1), 1},
            {PxVec3(-.10f, -.45f, -.07f), PxVec3(0, 0, -1), 1},
            {PxVec3(-.16f, -.18f, .02f), PxVec3(-1, 0, 0), 1},
            {PxVec3(-.13f, -.12f, .07f), PxVec3(-1, 0, 1).getNormalized(), 1},
            {PxVec3(-.13f, -.12f, .07f), PxVec3(-.7f, 0, 1).getNormalized(), 0},
            {PxVec3(-.08f, -.12f, .10f), PxVec3(0, 0, 1), 0},
            {PxVec3(-.02f, .13f, .09f), PxVec3(0, 1, 0), 0},
            {PxVec3(-.001f, -.14f, -.09f), PxVec3(0, 0, -1), 1},
            {PxVec3(-.30f, -1.2f, -.10f), PxVec3(0, 0, -1), 1}
        };
        const PxTransform tilted(PxVec3(2, -1, 3), PxQuat(2.2f, PxVec3(1, 3, 2).getNormalized()));
        for (const PxTransform& body : {PxTransform(PxIdentity), tilted})
        {
            const PxVec3 left = body.transform(rest[arms[0].setup.shoulderNode].p);
            const PxVec3 right = body.transform(rest[arms[1].setup.shoulderNode].p);
            const PxVec3 pelvis = body.transform(PxVec3(0, -.5f, 0));
            const PxVec3 center = body.transform(PxVec3(0, -.15f, -.10f));
            Check(PARS::SelectWoundHand(center, body.q.rotate(PxVec3(0, 0, -1)), left, right, pelvis).hand == -1,
                "An exact midline hit has no anatomical side");
            for (const Surface& surface : surfaces)
            {
                for (int mirror = 0; mirror < 2; ++mirror)
                {
                    const float sign = mirror == 0 ? 1.0f : -1.0f;
                    const int expected = mirror == 0 ? surface.handForLeftHit : 1 - surface.handForLeftHit;
                    PARS::ImpactPalmGoal goal;
                    goal.hitNode = 0;
                    goal.weight = 1;
                    goal.approach = 1;
                    goal.point = body.transform(PxVec3(sign * surface.point.x, surface.point.y, surface.point.z));
                    goal.normal = body.q.rotate(PxVec3(sign * surface.normal.x, surface.normal.y, surface.normal.z));
                    goal.selection = PARS::SelectWoundHand(goal.point, goal.normal, left, right, pelvis);
                    Check(goal.selection.hand == expected && goal.selection.onBack == (surface.handForLeftHit == 0),
                        "Mirrored wounds use the torso's front, side and back");
                    for (PARS::FallArmSample& arm : arms)
                    {
                        arm.actualAnchor = body;
                        arm.actualHand = body * rest[arm.setup.handNode];
                    }
                    // 반대 후보의 손바닥을 상처에 바로 놓아 거리로는 항상 그 손이 유리하게 한다.
                    arms[1 - expected].actualHand = PxTransform(goal.point, body.q) * arms[1 - expected].setup.palmInHand.getInverse();
                    std::vector<PxTransform> pose = rest;
                    PARS::FallArmSnapshot result;
                    Check(PARS::ApplyImpactPalmTarget(goal, arms, parents, pose, result) && goal.hand == expected &&
                        result.arms[expected].applied && !result.arms[1 - expected].applied,
                        "The anatomical hand wins even when the other palm already touches the wound");
                    if (surface.point.y < -1.0f)
                    {
                        Check(result.arms[expected].reachLimited, "An out-of-reach wound keeps the selected side");
                    }
                    goal.approach = .5f;
                    const PxVec3 contact = result.arms[expected].palm.p;
                    pose = rest;
                    Check(PARS::ApplyImpactPalmTarget(goal, arms, parents, pose, result) && goal.hand == expected,
                        "Keep the chosen hand during approach");
                    const PxVec3 detour = result.arms[expected].palm.p - contact - goal.normal * .05f;
                    const PxVec3 outward = body.q.rotate(PxVec3(expected == 0 ? -1.0f : 1.0f, 0, 0));
                    const float sideDistance = goal.selection.onBack ? arms[expected].setup.upperLength * .75f : 0;
                    Check((detour - outward * sideDistance).magnitude() < 1e-4f,
                        "A rear reach passes around the flank before closing onto the wound");
                }
            }
        }
        std::puts("PASS: wound hand selection on mirrored chest, abdomen, flank and back in a rotated torso");
    }

    void TestWoundContactLimits(const std::vector<int32_t>& parents, const std::vector<PxTransform>& rest,
        std::array<PARS::FallArmSample, 2> arms)
    {
        PARS::FallArmSetup& setup = arms[0].setup;
        for (size_t side = 0; side < arms.size(); ++side)
        {
            PARS::FallArmSetup& arm = arms[side].setup;
            const std::array<int, 4> chain{arm.anchorNode, arm.shoulderNode, arm.elbowNode, arm.handNode};
            for (size_t link = 0; link < arm.armJoints.size(); ++link)
            {
                PARS::FallArmJoint& joint = arm.armJoints[link];
                joint.index = static_cast<int>(side * arm.armJoints.size() + link);
                joint.inChildBone = PxTransform(PxQuat(.3f, PxVec3(1, 0, 0)));
                joint.inParentBone = rest[chain[link]].transformInv(rest[chain[link + 1]]) * joint.inChildBone;
            }
        }
        const PxTransform restingPalm = rest[setup.handNode] * setup.palmInHand;
        PARS::ImpactPalmGoal goal;
        PARS::WoundArmHistory history;
        goal.history = &history;
        goal.hitNode = 0;
        goal.hand = 0;
        goal.weight = 1;
        goal.approach = 1;
        goal.point = PxVec3(-.05f, -.22f, .15f);
        goal.normal = PxVec3(0, 0, 1);
        goal.limitJointRotation = [](int, const PxQuat& rotation) -> PxQuat
        {
            return rotation;
        };
        std::vector<PxTransform> pose = rest;
        PARS::FallArmSnapshot result;
        Check(PARS::ApplyImpactPalmTarget(goal, arms, parents, pose, result) && result.arms[0].canDriveWoundGrip && history.hand == 0,
            "A reachable contact survives projection through non-identity joint frames");
        Check(!result.arms[1].applied, "An ordinary torso wound uses a single hand");
        const PxTransform leadingPose = pose[setup.handNode];
        arms[0].actualHand = leadingPose;
        setup.handShape.radius = .025f;
        PARS::WoundArmHistory supportingHistory;
        PARS::ImpactPalmGoal critical = goal;
        critical.supportApproach = 1;
        critical.supportHistory = &supportingHistory;
        pose = rest;
        const PxTransform firstPalm = leadingPose * setup.palmInHand;
        const PxVec3 back = firstPalm.p + firstPalm.q.getBasisVector1() * .031f;
        Check(PARS::ApplyImpactPalmTarget(critical, arms, parents, pose, result) && result.arms[1].applied &&
            supportingHistory.hand == 1 && (result.arms[1].palm.p - back).magnitude() < 1e-4f &&
            (pose[setup.handNode].p - leadingPose.p).magnitude() < 1e-4f,
            "The second palm targets the collider back without displacing the first hand");
        critical.hitNode = arms[1].setup.handNode;
        pose = rest;
        Check(PARS::ApplyImpactPalmTarget(critical, arms, parents, pose, result) && !result.arms[1].applied,
            "A critically struck hand cannot support its own wound");
        critical.hitNode = goal.hitNode;
        critical.limitJointRotation = [](int joint, const PxQuat& rotation) -> PxQuat
        {
            return joint >= 3 ? PxQuat(PxIdentity) : rotation;
        };
        pose = rest;
        Check(PARS::ApplyImpactPalmTarget(critical, arms, parents, pose, result) && !result.arms[1].applied &&
            pose[arms[1].setup.handNode] == rest[arms[1].setup.handNode],
            "An unreachable supporting hand leaves the primary contact and idle arm intact");
        setup.handShape.radius = 0;
        goal.limitJointRotation = [](int, const PxQuat&) -> PxQuat
        {
            return PxQuat(PxIdentity);
        };
        pose = rest;
        Check(PARS::ApplyImpactPalmTarget(goal, arms, parents, pose, result) && !result.arms[0].canDriveWoundGrip &&
            (result.arms[0].solvedPalm - restingPalm.p).magnitude() < 1e-5f,
            "A cached arm pose respects the current joint limits when those joints lock");
        goal.point = restingPalm.p - goal.normal * .006f;
        pose = rest;
        Check(PARS::ApplyImpactPalmTarget(goal, arms, parents, pose, result) && !result.arms[0].canDriveWoundGrip &&
            result.arms[0].reachError < 1e-5f, "Reaching the point edge-on cannot engage the grip");
        goal.normal = restingPalm.q.getBasisVector1();
        goal.point = restingPalm.p - goal.normal * .006f;
        pose = rest;
        Check(PARS::ApplyImpactPalmTarget(goal, arms, parents, pose, result) && result.arms[0].canDriveWoundGrip,
            "A limited wrist can cover the wound with a different finger heading");
        goal.point -= goal.normal * .05f;
        goal.approach = .5f;
        goal.weight = .5f;
        pose = rest;
        Check(PARS::ApplyImpactPalmTarget(goal, arms, parents, pose, result) && result.arms[0].reachError < 1e-5f &&
            !result.arms[0].canDriveWoundGrip, "A reachable approach waypoint cannot pull toward an unreachable final contact");

        setup.palmInHand = PxTransform(PxIdentity);
        setup.handShape.inBone = PxTransform(PxVec3(0, .018f, 0));
        setup.handShape.halfExtents = PxVec3(.04f, .018f, .09f);
        goal.approach = 1;
        goal.weight = 1;
        goal.point = PxVec3(-.05f, -.22f, .15f);
        goal.normal = PxVec3(0, 0, 1);
        goal.limitJointRotation = [](int joint, const PxQuat& rotation) -> PxQuat
        {
            return joint == 2 ? PxQuat(PxIdentity) : rotation;
        };
        pose = rest;
        Check(PARS::ApplyImpactPalmTarget(goal, arms, parents, pose, result) && !result.arms[0].canDriveWoundGrip,
            "An angled hand can back away while the wrist stays locked");
        const PxTransform handShape = pose[setup.handNode] * setup.handShape.inBone;
        const float clearance = (handShape.p - goal.point).dot(goal.normal) -
            setup.handShape.GetHalfExtent(handShape.q.rotateInv(goal.normal));
        Check(clearance > .001f && PxAbs(pose[setup.handNode].q.dot(pose[setup.elbowNode].q)) > .99999f,
            "The hand collider clears the surface instead of pressing its edge into the wound");
    }

    void TestWoundLegPosture()
    {
        const std::vector<int32_t> parents{-1, 0, 1, 2, 3, 0, 5, 6, 0, 8, 9};
        const std::vector<PxTransform> rest{PxTransform(PxVec3(0, 1, 0)), PxTransform(PxVec3(0, 1.35f, 0)),
            PxTransform(PxVec3(-.17f, 1.45f, 0)), PxTransform(PxVec3(-.21f, 1.16f, .02f)), PxTransform(PxVec3(-.2f, .9f, .04f)),
            PxTransform(PxVec3(-.12f, 1, 0)), PxTransform(PxVec3(-.12f, .55f, .02f)), PxTransform(PxVec3(-.12f, .1f, 0)),
            PxTransform(PxVec3(.12f, 1, 0)), PxTransform(PxVec3(.12f, .55f, .02f)), PxTransform(PxVec3(.12f, .1f, 0))};
        std::array<PARS::FallArmSample, 2> arms;
        PARS::FallArmSetup& arm = arms[0].setup;
        arm.available = true;
        arm.chestNode = 1;
        arm.anchorNode = 1;
        arm.shoulderNode = 2;
        arm.elbowNode = 3;
        arm.handNode = 4;
        arm.upperLength = (rest[3].p - rest[2].p).magnitude();
        arm.lowerLength = (rest[4].p - rest[3].p).magnitude();
        arms[0].actualAnchor = rest[1];
        arms[0].actualHand = rest[4];
        PARS::ImpactPalmGoal goal;
        goal.hand = 0;
        goal.approach = 1;
        goal.weight = 1;
        goal.hitNode = 6;
        goal.pelvisNode = 0;
        std::array<PARS::FootLegSetup, 2> legs;
        for (int side = 0; side < 2; ++side)
        {
            PARS::FootLegSetup& leg = legs[side];
            leg.hipNode = 5 + 3 * side;
            leg.kneeNode = leg.hipNode + 1;
            leg.ankleNode = leg.kneeNode + 1;
            leg.thighLength = (rest[leg.kneeNode].p - rest[leg.hipNode].p).magnitude();
            leg.calfLength = (rest[leg.ankleNode].p - rest[leg.kneeNode].p).magnitude();
            leg.available = true;
        }
        goal.injuredLeg = &legs[0];
        goal.oppositeLeg = &legs[1];
        goal.point = rest[6].p;
        goal.normal = PxVec3(0, 0, 1);
        goal.limitJointRotation = [](int, const PxQuat& raw) -> PxQuat
        {
            return raw;
        };
        PARS::FallArmSnapshot result;
        std::vector<PxTransform> planted = rest;
        Check(PARS::ApplyImpactPalmTarget(goal, arms, parents, planted, result) &&
            planted[1].p.z > planted[0].p.z + .15f && planted[0].p.y < rest[0].p.y - .01f &&
            (planted[7].p - rest[7].p).magnitude() < 1e-5f && (planted[10].p - rest[10].p).magnitude() < 1e-5f &&
            PxAbs(planted[7].q.dot(rest[7].q)) > .99999f && PxAbs(planted[10].q.dot(rest[10].q)) > .99999f,
            "A knee hit folds the trunk and preserves the world targets of both planted feet");
        goal.freeLegWeight = 1;
        std::vector<PxTransform> freeLeg = rest;
        Check(PARS::ApplyImpactPalmTarget(goal, arms, parents, freeLeg, result) &&
            freeLeg[6].p.y > rest[6].p.y + .1f && freeLeg[1].p.z - freeLeg[0].p.z < planted[1].p.z - planted[0].p.z &&
            (freeLeg[10].p - rest[10].p).magnitude() < 1e-5f,
            "An unloaded injured knee can rise while the supporting foot keeps its target");
        std::puts("PASS: wound posture with planted feet and with an unloaded knee");
        goal.freeLegWeight = 0;
        goal.legBendWeight = 0;
        goal.restingLegWeight = 1;
        std::vector<PxTransform> resting = rest;
        Check(PARS::ApplyImpactPalmTarget(goal, arms, parents, resting, result) && resting[0] == rest[0] &&
            resting[6].p.y > rest[6].p.y + .2f && resting[10] == rest[10],
            "Once the torso rests on the ground only the injured leg folds toward the hand");
    }

    void TestImpactPalm()
    {
        const std::vector<int32_t> parents{-1, 0, 1, 2, 0, 4, 5};
        const std::vector<PxTransform> rest{PxTransform(PxIdentity),
            PxTransform(PxVec3(-.18f, 0, 0)), PxTransform(PxVec3(-.18f, -.30f, 0)), PxTransform(PxVec3(-.18f, -.58f, 0)),
            PxTransform(PxVec3(.18f, 0, 0)), PxTransform(PxVec3(.18f, -.30f, 0)), PxTransform(PxVec3(.18f, -.58f, 0))};
        std::array<PARS::FallArmSample, 2> arms;
        for (int side = 0; side < 2; ++side)
        {
            PARS::FallArmSetup& setup = arms[side].setup;
            setup.available = true;
            setup.anchorNode = 0;
            setup.shoulderNode = 1 + 3 * side;
            setup.elbowNode = setup.shoulderNode + 1;
            setup.handNode = setup.elbowNode + 1;
            setup.upperLength = .30f;
            setup.lowerLength = .28f;
            setup.palmInHand = PxTransform(PxVec3(0, 0, .02f));
        }
        PARS::ImpactPalmGoal goal;
        Check(goal.SampleTiming(.05f, .05f, true) && goal.weight == 0, "Hand waits for the initial recoil");
        Check(goal.SampleTiming(.22f, .22f, true) && std::abs(goal.weight - .5f) < 1e-5f, "Hand approaches as the arm weakens");
        for (unsigned step = 21; step <= 320; ++step)
        {
            const float age = static_cast<float>(step * PhysicsWorld::FixedDeltaTime);
            Check(goal.SampleTiming(age, age, true) && goal.approach == 1.0f && goal.weight == 1.0f,
                "Hand holds the moving arm for five full seconds");
        }
        Check(goal.SampleTiming(0, 2, true) && goal.weight == 1, "Renew a wound without retracting a hand already there");
        Check(goal.SampleTiming(5.465f, 5.465f, true) && std::abs(goal.weight - .5f) < 1e-5f &&
            !goal.SampleTiming(5.6f, 5.6f, true) && goal.weight == 0, "Release the arm over a quarter second");
        Check(goal.SampleTiming(6.20f, 6.20f, true, 1.20f) && goal.weight == 1.0f &&
            goal.SampleTiming(6.325f, 6.325f, true, 1.20f) && std::abs(goal.weight - .5f) < 1e-5f,
            "A palm that arrives late holds for five seconds from contact");
        Check(!goal.SampleTiming(1.15f, 1.15f, false), "Lower-body hits keep their short gesture");
        goal.weight = 1;
        goal.approach = 1;
        TestWoundHandSelection(parents, rest, arms);
        TestWoundContactLimits(parents, rest, arms);
        for (int side = 0; side < 2; ++side)
        {
            goal.hand = -1;
            goal.selection = {side, true};
            goal.hitNode = arms[side].setup.shoulderNode;
            const PxVec3 hit(rest[goal.hitNode].p.x, -.13f, .07f);
            for (const PxTransform& body : {PxTransform(PxIdentity), PxTransform(PxVec3(-2, 3, 1), PxQuat(.9f, PxVec3(0, 1, 0)))})
            {
                for (PARS::FallArmSample& arm : arms)
                {
                    arm.actualAnchor = body;
                    arm.actualHand = body * rest[arm.setup.handNode];
                }
                goal.point = body.transform(hit);
                goal.normal = body.q.rotate(PxVec3(0, 0, 1));
                std::vector<PxTransform> pose = rest;
                PARS::FallArmSnapshot result;
                Check(PARS::ApplyImpactPalmTarget(goal, arms, parents, pose, result) && goal.hand == 1 - side,
                    "The opposite hand reaches a struck arm");
                const PxTransform palm = body * pose[arms[goal.hand].setup.handNode] * arms[goal.hand].setup.palmInHand;
                Check((palm.p - (goal.point + goal.normal * .006f)).magnitude() < 1e-4f &&
                    palm.q.rotate(PxVec3(0, 1, 0)).dot(goal.normal) > .999f && !result.arms[side].applied,
                    "Only the reaching arm turns its palm to the same surface after the body moves");
            }
        }
        std::vector<PxTransform> pose = rest;
        goal.hand = -1;
        goal.selection = {};
        goal.hitNode = 0;
        goal.point = arms[1].actualHand.transform(PxVec3(0, -.8f, .05f));
        PARS::FallArmSnapshot result;
        Check(PARS::ApplyImpactPalmTarget(goal, arms, parents, pose, result) && goal.hand == 1 &&
            result.arms[goal.hand].reachLimited &&
            std::abs((pose[6].p - pose[5].p).magnitude() - arms[1].setup.lowerLength) < 1e-4f,
            "An unreachable body hit selects the closer hand and preserves arm length");
        std::puts("PASS: struck arm, moving surface, palm orientation, hand choice and reach limit");
        TestClavicleReach();
        TestWoundLegPosture();
    }

    void TestBodyPairContacts()
    {
        PhysicsWorld world;
        Check(world.Initialize(false), "Initialize body contact scene");
        PxRigidDynamic* first = world.CreateDynamicBody(PxTransform(PxVec3(0, 3, 0)),
            PxCapsuleGeometry(.2f, .2f), 2, "Moving leg");
        PxRigidDynamic* second = world.CreateDynamicBody(PxTransform(PxVec3(0, 3, .3f)),
            PxCapsuleGeometry(.2f, .2f), 2, "Other leg");
        PxRigidDynamic* distant = world.CreateDynamicBody(PxTransform(PxVec3(5, 3, 0)),
            PxSphereGeometry(.2f), 2, "Distant body");
        PxRigidDynamic* nearby = world.CreateDynamicBody(PxTransform(PxVec3(5.43f, 3, 0)),
            PxSphereGeometry(.2f), 2, "Contact offset only");
        Check(first && second && distant && nearby, "Create contact query bodies");
        const std::array<PxRigidDynamic*, 4> bodies{first, second, distant, nearby};
        int owner = 0;
        for (PxRigidDynamic* body : bodies)
        {
            body->setActorFlag(PxActorFlag::eDISABLE_GRAVITY, true);
            body->userData = &owner;
        }
        Check(!world.HasContact(*first, *second), "Geometric overlap is not reported before a real physics contact");
        world.Step();
        Check(world.HasContact(*first, *second) && world.HasContact(*second, *first),
            "Actual same-owner contact is available in either body order");
        Check(!world.HasContact(*first, *first) && !world.HasContact(*first, *distant),
            "Identical and unrelated bodies are excluded");
        Check(!world.HasContact(*distant, *nearby), "A contact offset without touching or impulse is not a collision");

        world.InvalidateContacts(*second);
        Check(!world.HasContact(*first, *second), "Invalidated contacts do not remain visible");

        // 접촉 해제를 솔버가 처리한 다음 다시 닿게 해 새 접촉 보고를 검증한다.
        second->setGlobalPose(PxTransform(PxVec3(0, 3, 5)));
        world.Step();
        first->setGlobalPose(PxTransform(PxVec3(0, 3, 0)));
        second->setGlobalPose(PxTransform(PxVec3(0, 3, .3f)));
        first->setLinearVelocity(PxVec3(0));
        second->setLinearVelocity(PxVec3(0));
        world.Step();
        Check(world.HasContact(*first, *second), "A new solver contact replaces invalidated data");
        world.ResetClock();
        Check(!world.HasContact(*first, *second), "Clock reset invalidates previous contact observations");
        second->setGlobalPose(PxTransform(PxVec3(0, 3, 5)));
        world.Step();
        Check(!world.HasContact(*first, *second), "Separated bodies do not retain a contact message");

        first->setGlobalPose(PxTransform(PxVec3(0, 3, 0)));
        second->setGlobalPose(PxTransform(PxVec3(0, 3, .3f)));
        first->setLinearVelocity(PxVec3(0));
        second->setLinearVelocity(PxVec3(0));
        world.Step();
        Check(world.HasContact(*first, *second), "Contacts resume after clock reset");
        PxShape* shape = nullptr;
        second->getShapes(&shape, 1);
        shape->setFlag(PxShapeFlag::eSIMULATION_SHAPE, false);
        Check(!world.HasContact(*first, *second), "Disabling collisions hides cached contact immediately");
        Check(world.GetErrorCount() == 0, "Contact queries introduce no PhysX errors");
        std::puts("PASS: same-owner body contacts, proximity, collision disabling, invalidation and reset");
    }

    void TestFootSupportHull()
    {
        const std::vector<PxVec3> left = {
            {-.15f, .4f, -.1f}, {-.05f, .4f, -.1f},
            {-.05f, .4f, .1f}, {-.15f, .4f, .1f}};
        std::vector<physx::PxVec3> points = left;
        points.insert(points.end(), {{.05f, .4f, 0}, {.15f, .4f, 0}, {.15f, .4f, .2f}, {.05f, .4f, .2f}});
        points.push_back(left[0]);
        points.emplace_back(0.0f, .4f, .05f); // Interior point must not affect the boundary.
        const std::vector<physx::PxVec3> hull = PARS::BuildFootSupportHull(points);
        Check(hull.size() == 6, "Offset feet form a six-vertex support hull");
        double area = 0;
        for (size_t i = 0; i < hull.size(); ++i)
        {
            const physx::PxVec3& a = hull[i];
            const physx::PxVec3& b = hull[(i + 1) % hull.size()];
            Check(a.y == .4f, "Hull preserves the real hit height");
            area += double(a.x) * b.z - double(a.z) * b.x;
            for (const physx::PxVec3& point : points)
            {
                Check((double(b.x) - a.x) * (double(point.z) - a.z) -
                            (double(b.z) - a.z) * (double(point.x) - a.x) >=
                        -1e-8,
                    "Every projected point lies inside the counterclockwise hull");
            }
        }
        Check(std::abs(area * .5 - .07) < 1e-7, "Support hull spans the space between feet");
        Check(PARS::BuildFootSupportHull(left).size() == 4, "One supported foot has its own hull");
        Check(PARS::BuildFootSupportHull({}).empty(), "No projected points have no support polygon");
        Check(PARS::BuildFootSupportHull({left[0]}).empty(), "One point has no support polygon");
        Check(PARS::BuildFootSupportHull({left[0], left[1]}).empty(), "Two points have no support polygon");
        Check(PARS::BuildFootSupportHull({{0, 0, 0}, {1, 0, 0}, {2, 0, 0}, {1, 1, 0}}).empty(),
            "Collinear and duplicate XZ points have no support polygon");
        const std::vector<physx::PxVec3> oneFoot = PARS::BuildFootSupportHull(left);
        Check(PARS::IsInsideFootSupportHull(oneFoot, PxVec3(-.1f, 100, 0)),
            "COM inside a foot is supported regardless of its height");
        Check(PARS::IsInsideFootSupportHull(hull, PxVec3(0, 2, .05f)),
            "COM between supported feet lies in the combined hull");
        Check(PARS::IsInsideFootSupportHull(oneFoot, PxVec3(-.15f, 0, 0)) &&
                PARS::IsInsideFootSupportHull(oneFoot, left[0]),
            "Hull edges and vertices count as inside");
        Check(!PARS::IsInsideFootSupportHull(oneFoot, PxVec3(-.1501f, 0, 0)) &&
                !PARS::IsInsideFootSupportHull(oneFoot, PxVec3(0, 0, 0)),
            "COM beyond an edge or the remaining foot is outside");
        Check(!PARS::IsInsideFootSupportHull({}, PxVec3(0)) &&
                !PARS::IsInsideFootSupportHull({left[0], left[1]}, left[0]),
            "Missing or degenerate support polygons cannot contain COM");
        std::puts("PASS: foot support hull, both feet, one foot and degenerate projections");
        std::puts("PASS: COM containment, boundary, height independence and missing support");
    }

    void TestFootSupportBalance()
    {
        using namespace PARS;
        const std::vector<physx::PxVec3> hull = BuildFootSupportHull({{-1, .4f, -2}, {1, .4f, -2}, {1, .4f, 2}, {-1, .4f, 2}});
        // 이 큰 사각형에서는 상한에 닿기 전의 배율과 보간을 확인한다.
        constexpr float radiusLimit = 8.0f;
        const PARS::FootSupportBalance center = EvaluateFootSupportBalance(hull, PxVec3(0), 2.0f, radiusLimit);
        Check(center.zone == FootSupportZone::Inside && center.weight == 1 && !center.hasDirection,
            "COM at the area centroid has full weight without normalizing a zero vector");
        Check(center.bufferHull.size() == 4 && center.bufferHull[0] == PxVec3(-2, .4f, -4),
            "B doubles XZ distances while preserving surface height");
        for (const PxVec3 direction : {PxVec3(1, 0, 0), PxVec3(0, 0, 2), PxVec3(1, 0, 2)})
        {
            const PARS::FootSupportBalance edgeA = EvaluateFootSupportBalance(hull, direction, 2.0f, radiusLimit);
            const PARS::FootSupportBalance middle = EvaluateFootSupportBalance(hull, direction * 1.5f, 2.0f, radiusLimit);
            const PARS::FootSupportBalance edgeB = EvaluateFootSupportBalance(hull, direction * 2, 2.0f, radiusLimit);
            const PARS::FootSupportBalance outside = EvaluateFootSupportBalance(hull, direction * 3, 2.0f, radiusLimit);
            Check(edgeA.weight == 1 && edgeA.zone == FootSupportZone::Inside,
                "A edges and corners have full weight");
            Check(std::abs(middle.weight - .5f) < 1e-6f && middle.zone == FootSupportZone::Buffer,
                "Halfway between A and B has half weight along narrow, long and corner directions");
            Check(edgeB.weight == 0 && edgeB.zone == FootSupportZone::Outside && outside.weight == 0,
                "B boundary and exterior have zero weight");
            Check(std::abs(middle.outerDistance - 2 * middle.innerDistance) < 1e-6f,
                "Radial B distance uses the same direction as A");
        }
        float previous = 1;
        for (int i = 0; i <= 100; ++i)
        {
            const float weight = EvaluateFootSupportBalance(hull, PxVec3(1 + i * .01f, 50, 0), 2.0f, radiusLimit).weight;
            Check(weight <= previous && weight >= 0 && weight <= 1, "Weight decreases monotonically across the band");
            previous = weight;
        }
        // This trapezoid's area centroid differs from the average of its vertices.
        const std::vector<physx::PxVec3> trapezoid = BuildFootSupportHull({{0, 0, 0}, {4, 0, 0}, {2, 0, 2}, {0, 0, 2}});
        const physx::PxVec3 centroid = EvaluateFootSupportBalance(trapezoid, PxVec3(0)).center;
        Check(std::abs(centroid.x - 14.0f / 9) < 1e-6f && std::abs(centroid.z - 8.0f / 9) < 1e-6f,
            "Expansion uses the polygon area centroid, not the vertex average");
        std::vector<physx::PxVec3> moved = hull;
        physx::PxVec3 (*const transform)(physx::PxVec3) = [](PxVec3 v) -> physx::PxVec3
        {
            return PxVec3(100 - v.z, v.y + 7, 200 + v.x);
        };
        for (physx::PxVec3& vertex : moved)
        {
            vertex = transform(vertex);
        }
        const PARS::FootSupportBalance movedBalance = EvaluateFootSupportBalance(moved, transform(PxVec3(1.5f, 0, 0)), 2.0f, radiusLimit);
        Check(std::abs(movedBalance.weight - .5f) < 1e-6f &&
                (movedBalance.center - transform(center.center)).magnitude() < 1e-5f,
            "Translation and rotation move A, B and centroid without changing weight");
        Check(EvaluateFootSupportBalance(hull, PxVec3(2, 0, 0), 3, radiusLimit).weight == .5f,
            "Changing B scale changes the fade interval");
        const std::vector<PxVec3> wide = BuildFootSupportHull({{-.6f, 0, 0}, {0, 0, -.15f}, {.6f, 0, 0}, {0, 0, .15f}});
        const FootSupportBalance capped = EvaluateFootSupportBalance(wide, PxVec3(.75f, 0, 0), 2.86f, .9f);
        Check(capped.zone == FootSupportZone::Buffer && std::abs(capped.weight - .5f) < 1e-6f &&
                std::abs(capped.outerDistance - .9f) < 1e-6f && IsInsideFootSupportHull(capped.bufferHull, PxVec3(.9f, 0, 0)) &&
                !IsInsideFootSupportHull(capped.bufferHull, PxVec3(.91f, 0, 0)),
            "The radius cap bounds both the support fade and the polygon used for the fall timer");
        const FootSupportBalance atFoot = EvaluateFootSupportBalance(wide, PxVec3(.6f, 0, 0), 2.86f, .5f);
        const FootSupportBalance pastFoot = EvaluateFootSupportBalance(wide, PxVec3(.61f, 0, 0), 2.86f, .5f);
        Check(atFoot.bufferHull == wide && atFoot.zone == FootSupportZone::Inside && atFoot.weight == 1 &&
                pastFoot.zone == FootSupportZone::Outside && pastFoot.weight == 0,
            "A stance exceeding the cap keeps its actual support and gains no extra buffer");
        const float nan = std::numeric_limits<float>::quiet_NaN();
        for (const std::vector<physx::PxVec3>& invalid : {std::vector<PxVec3>{}, std::vector<PxVec3>{{0, 0, 0}, {1, 0, 0}},
                 std::vector<PxVec3>{{0, 0, 0}, {1, 0, 0}, {2, 0, 0}},
                 std::vector<PxVec3>{{nan, 0, 0}, {1, 0, 0}, {0, 0, 1}}})
        {
            const PARS::FootSupportBalance result = EvaluateFootSupportBalance(invalid, PxVec3(0));
            Check(result.weight == 0 && result.zone == FootSupportZone::Invalid && result.bufferHull.empty(),
                "Missing, collinear and nonfinite support cannot produce a buffer or positive weight");
        }
        Check(EvaluateFootSupportBalance(hull, PxVec3(nan)).zone == FootSupportZone::Invalid &&
                EvaluateFootSupportBalance(hull, PxVec3(0), 1).weight == 0 &&
                EvaluateFootSupportBalance(hull, PxVec3(0), nan).weight == 0,
            "Invalid COM and expansion scales fail safely");
        std::puts("PASS: A/B centroid, expansion, radial weight, boundaries, transforms and invalid input");
    }

    float SimulateAtRate(int fps, double seconds)
    {
        PhysicsWorld world;
        Check(world.Initialize(false), "Initialize");
        Check(world.CreateStaticBox(PxTransform(PxVec3(0, -0.25f, 0)), PxVec3(6, 0.25f, 6), "Test ground"), "Ground");
        physx::PxRigidDynamic* box = world.CreateDynamicBox(PxTransform(PxVec3(0, 4, 0)), PxVec3(0.5f), 10, "Test box");
        Check(box != nullptr, "Box");
        for (int frame = 0; frame < static_cast<int>(fps * seconds); ++frame)
            world.Update(1.0 / fps);
        Check(world.GetStepCount() == static_cast<uint64_t>(seconds * 60), "Frame-rate-independent step count");
        const float y = box->getGlobalPose().p.y;
        if (seconds >= 3.0)
        {
            Check(std::abs(y - 0.5f) < 0.01f, "Box settles on floor at y=0.5");
            Check(std::abs(box->getLinearVelocity().y) < 0.01f, "Resting velocity");
            Check(box->isSleeping(), "Box sleeps at rest");
        }
        else
            Check(y > 0.5f && y < 4.0f, "Gravity moves box before contact");
        world.Shutdown();
        world.Shutdown();
        Check(world.GetErrorCount() == 0, "No PhysX errors during simulation/shutdown");
        std::printf("PASS: %d FPS, %.1f s, y=%.6f\n", fps, seconds, y);
        return y;
    }
}

void TestFramePerformance();

int main(int argc, char** argv)
{
    if (argc == 2 && std::strcmp(argv[1], "--projectile-stagger-only") == 0)
    {
        TestProjectileStagger();
        return 0;
    }
    if (argc == 2 && std::strcmp(argv[1], "--impact-hand-only") == 0)
    {
        TestApproachingImpact();
        TestSharedImpactContact(false);
        TestSharedImpactContact(true);
        TestProjectileAttachment();
        TestImpactPalm();
        return 0;
    }
    if (argc == 2 && std::strcmp(argv[1], "--horizontal-balance-only") == 0)
    {
        TestHorizontalBalance();
        return 0;
    }
    if (argc == 2 && std::strcmp(argv[1], "--standing-support-only") == 0)
    {
        TestStandingSupport();
        return 0;
    }
    if (argc == 2 && std::strcmp(argv[1], "--impact-warmup-only") == 0)
    {
        TestImpactWarmupPlanning();
        return 0;
    }
    if (argc == 2 && std::strcmp(argv[1], "--footstep-only") == 0)
    {
        TestFootstepPlanner();
        TestFootstepIK();
        TestFootstepCollision();
        TestStandingSupport();
        return 0;
    }
    if (argc == 2 && std::strcmp(argv[1], "--footstep-fall-only") == 0)
    {
        TestFootstepFalls();
        return 0;
    }
    if (argc == 2 && std::strcmp(argv[1], "--foot-command-only") == 0)
    {
        TestFootstepPlanner();
        return 0;
    }
    TestFramePerformance();
    if (argc == 2 && std::strcmp(argv[1], "--frame-performance-only") == 0)
    {
        return 0;
    }
    TestFootProjectionRaycast();
    TestApproachingImpact();
    TestSharedImpactContact(false);
    TestSharedImpactContact(true);
    TestProjectileAttachment();
    TestImpactPalm();
    TestBodyPairContacts();
    TestFootSupportHull();
    TestFootSupportBalance();
    TestHorizontalBalance();
    TestFootstepPlanner();
    TestFootstepIK();
    TestFootstepCollision();
    TestStandingSupport();
    const float reference = SimulateAtRate(60, 0.5);
    Check(std::abs(SimulateAtRate(30, 0.5) - reference) < 1e-5f, "30/60 FPS agree in flight");
    Check(std::abs(SimulateAtRate(144, 0.5) - reference) < 1e-5f, "144/60 FPS agree in flight");
    SimulateAtRate(30, 3.0);
    SimulateAtRate(144, 3.0);

    PhysicsWorld world;
    Check(world.Update(1.0) == 0, "Safe before initialization");
    Check(world.Initialize(false), "Initialize for timing tests");
    Check(world.Initialize(false), "Repeated Initialize is harmless");
    Check(world.Update(-1) == 0 && world.Update(0) == 0, "Ignore nonpositive deltas");
    Check(world.Update(std::numeric_limits<double>::infinity()) == 0, "Ignore infinity");
    Check(world.Update(std::numeric_limits<double>::quiet_NaN()) == 0, "Ignore NaN");
    Check(world.Update(PhysicsWorld::FixedDeltaTime / 2) == 0, "Keep fractional step");
    Check(world.Update(PhysicsWorld::FixedDeltaTime / 2) == 1, "Accumulate fractional step");
    world.ResetClock();
    Check(world.Update(10) == PhysicsWorld::MaxSubsteps, "Bound debugger-stall catch-up");
    Check(world.GetDroppedTime() > 9.8, "Record discarded stall time");
    Check(world.Update(PhysicsWorld::FixedDeltaTime) == 1, "No stall backlog");
    world.ResetClock();
    world.Step();
    Check(world.GetStepCount() == 1, "Single step");

    // Exercise actor replacement exactly as the UI's Reset drop command does.
    for (int i = 0; i < 10; ++i)
    {
        physx::PxRigidDynamic* box = world.CreateDynamicBox(PxTransform(PxVec3(0, 4, 0)), PxVec3(0.5f), 10, "Reset box");
        Check(box != nullptr, "Recreate body");
        world.Step();
        box->release();
    }
    world.Shutdown();
    Check(world.GetErrorCount() == 0, "No lifecycle errors");
    Check(world.Initialize(false), "Reinitialize after Shutdown");
    if (argc > 1 && std::strcmp(argv[1], "--require-pvd") == 0)
    {
        Check(PhysicsWorld::SupportsPvd(), "PVD requires Debug/Profile");
        physx::PxRigidDynamic* box = world.CreateDynamicBox(PxTransform(PxVec3(0, 4, 0)), PxVec3(0.5f), 10, "PVD test box");
        Check(box != nullptr, "Body exists before connecting to PVD");
        Check(world.ConnectPvd() && world.IsPvdConnected(), "Connect to running PVD");
        Check(world.ConnectPvd(), "Repeated PVD connect is harmless");
        Check(world.ConnectPvd(true) && world.IsPvdConnected(), "Reconnect to PVD with an existing paused scene");
        Check(world.GetStepCount() == 0 && box->getGlobalPose().p.y == 4.0f,
            "Connecting/reconnecting does not advance paused physics");
        for (int step = 0; step < 60; ++step)
            world.Step();
        Check(world.CreateStaticBox(PxTransform(PxVec3(0, -0.25f, 0)),
            PxVec3(6, 0.25f, 6), "PVD ground"), "PVD ground");
        PhysicsQueryCapsule* capsule = world.CreateQueryCapsule(PxVec3(2, 1, 0), 0.3f, 0.9f);
        Check(capsule != nullptr, "PVD character capsule");
        world.SetPvdSceneQueries(true);
        const std::function<void(float)> moveCapsule = [&world, capsule](float) -> void
        {
            world.MoveQueryCapsule(*capsule, PxVec3(0.02f, -0.1f, 0));
        };
        for (int reconnect = 0; reconnect < 3; ++reconnect)
        {
            // Movement sweeps leave query geometry in the previous PVD session.
            world.Step(moveCapsule);
            const uint64_t steps = world.GetStepCount();
            const physx::PxVec3 position = capsule->centre;
            Check(world.ConnectPvd(true) && world.IsPvdConnected(),
                "Reconnect after recording character scene queries");
            Check(world.GetStepCount() == steps && capsule->centre == position,
                "Reconnect preserves paused character state");
            world.Step(moveCapsule);
            Check(capsule->centre.x > position.x, "Character moves after reconnect");
        }
        world.DestroyQueryCapsule(capsule);
        std::puts("PASS: live PVD, paused-scene reconnect and character queries after reconnect");
    }
    world.Shutdown();
    Check(world.GetErrorCount() == 0, "No errors after reinitialize/PVD shutdown");
    std::puts("PASS: PhysicsWorld timing, collision, reset and lifecycle");
    return 0;
}
