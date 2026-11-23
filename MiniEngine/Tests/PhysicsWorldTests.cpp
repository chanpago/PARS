#include "PhysicsWorld.h"
#include "../Model/FallArmBrace.h"
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
        std::vector<PhysicsContactImpulse> contacts;
        float strength = 0;
        for (int step = 0; step < 6 && strength <= 1e-4f; ++step)
        {
            world.Step();
            strength = world.GetExternalImpactImpulse(*target, nullptr, &contacts);
        }
        Check(!contacts.empty(), "Fallback captures a real kinematic contact");
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
            }
        }
        Check(sustained, "The pressed ball continues transferring impulse");
        std::puts("PASS: approaching collision, threshold, departing/missing source, ownership and collision policy");
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
        goal.weight = 1;
        goal.approach = 1;
        for (int side = 0; side < 2; ++side)
        {
            goal.hand = -1;
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
        goal.hitNode = 0;
        goal.point = arms[1].actualHand.transform(PxVec3(0, -.8f, .05f));
        PARS::FallArmSnapshot result;
        Check(PARS::ApplyImpactPalmTarget(goal, arms, parents, pose, result) && goal.hand == 1 &&
            result.arms[goal.hand].reachLimited &&
            std::abs((pose[6].p - pose[5].p).magnitude() - arms[1].setup.lowerLength) < 1e-4f,
            "An unreachable body hit selects the closer hand and preserves arm length");
        std::puts("PASS: struck arm, moving surface, palm orientation, hand choice and reach limit");
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
        const PARS::FootSupportBalance center = EvaluateFootSupportBalance(hull, PxVec3(0));
        Check(center.zone == FootSupportZone::Inside && center.weight == 1 && !center.hasDirection,
            "COM at the area centroid has full weight without normalizing a zero vector");
        Check(center.bufferHull.size() == 4 && center.bufferHull[0] == PxVec3(-2, .4f, -4),
            "B doubles XZ distances while preserving surface height");
        for (const PxVec3 direction : {PxVec3(1, 0, 0), PxVec3(0, 0, 2), PxVec3(1, 0, 2)})
        {
            const PARS::FootSupportBalance edgeA = EvaluateFootSupportBalance(hull, direction);
            const PARS::FootSupportBalance middle = EvaluateFootSupportBalance(hull, direction * 1.5f);
            const PARS::FootSupportBalance edgeB = EvaluateFootSupportBalance(hull, direction * 2);
            const PARS::FootSupportBalance outside = EvaluateFootSupportBalance(hull, direction * 3);
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
            const float weight = EvaluateFootSupportBalance(hull, PxVec3(1 + i * .01f, 50, 0)).weight;
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
        const PARS::FootSupportBalance movedBalance = EvaluateFootSupportBalance(moved, transform(PxVec3(1.5f, 0, 0)));
        Check(std::abs(movedBalance.weight - .5f) < 1e-6f &&
                (movedBalance.center - transform(center.center)).magnitude() < 1e-5f,
            "Translation and rotation move A, B and centroid without changing weight");
        Check(EvaluateFootSupportBalance(hull, PxVec3(2, 0, 0), 3).weight == .5f,
            "Changing B scale changes the fade interval");
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
    if (argc == 2 && std::strcmp(argv[1], "--impact-hand-only") == 0)
    {
        TestApproachingImpact();
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
