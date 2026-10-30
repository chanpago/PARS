#include "PhysicsWorld.h"
#include "../Model/FootSupportHull.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits>

using namespace physx;

void TestFootstepPlanner();
void TestFootstepIK();

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
        auto* floor = world.CreateStaticBox(PxTransform(PxVec3(0, -.25f, 0)),
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
        auto* ownBody = world.CreateDynamicBox(PxTransform(PxVec3(0, .1f, 0)),
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

    void TestFootSupportHull()
    {
        const std::vector<PxVec3> left = {
            {-.15f, .4f, -.1f}, {-.05f, .4f, -.1f},
            {-.05f, .4f, .1f}, {-.15f, .4f, .1f}};
        auto points = left;
        points.insert(points.end(), {{.05f, .4f, 0}, {.15f, .4f, 0}, {.15f, .4f, .2f}, {.05f, .4f, .2f}});
        points.push_back(left[0]);
        points.emplace_back(0.0f, .4f, .05f); // Interior point must not affect the boundary.
        const auto hull = PARS::BuildFootSupportHull(points);
        Check(hull.size() == 6, "Offset feet form a six-vertex support hull");
        double area = 0;
        for (size_t i = 0; i < hull.size(); ++i)
        {
            const auto& a = hull[i];
            const auto& b = hull[(i + 1) % hull.size()];
            Check(a.y == .4f, "Hull preserves the real hit height");
            area += double(a.x) * b.z - double(a.z) * b.x;
            for (const auto& point : points)
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
        const auto oneFoot = PARS::BuildFootSupportHull(left);
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
        const auto hull = BuildFootSupportHull({{-1, .4f, -2}, {1, .4f, -2}, {1, .4f, 2}, {-1, .4f, 2}});
        const auto center = EvaluateFootSupportBalance(hull, PxVec3(0));
        Check(center.zone == FootSupportZone::Inside && center.weight == 1 && !center.hasDirection,
            "COM at the area centroid has full weight without normalizing a zero vector");
        Check(center.bufferHull.size() == 4 && center.bufferHull[0] == PxVec3(-2, .4f, -4),
            "B doubles XZ distances while preserving surface height");
        for (const PxVec3 direction : {PxVec3(1, 0, 0), PxVec3(0, 0, 2), PxVec3(1, 0, 2)})
        {
            const auto edgeA = EvaluateFootSupportBalance(hull, direction);
            const auto middle = EvaluateFootSupportBalance(hull, direction * 1.5f);
            const auto edgeB = EvaluateFootSupportBalance(hull, direction * 2);
            const auto outside = EvaluateFootSupportBalance(hull, direction * 3);
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
        const auto trapezoid = BuildFootSupportHull({{0, 0, 0}, {4, 0, 0}, {2, 0, 2}, {0, 0, 2}});
        const auto centroid = EvaluateFootSupportBalance(trapezoid, PxVec3(0)).center;
        Check(std::abs(centroid.x - 14.0f / 9) < 1e-6f && std::abs(centroid.z - 8.0f / 9) < 1e-6f,
            "Expansion uses the polygon area centroid, not the vertex average");
        auto moved = hull;
        const auto transform = [](PxVec3 v)
        {
            return PxVec3(100 - v.z, v.y + 7, 200 + v.x);
        };
        for (auto& vertex : moved)
        {
            vertex = transform(vertex);
        }
        const auto movedBalance = EvaluateFootSupportBalance(moved, transform(PxVec3(1.5f, 0, 0)));
        Check(std::abs(movedBalance.weight - .5f) < 1e-6f &&
                (movedBalance.center - transform(center.center)).magnitude() < 1e-5f,
            "Translation and rotation move A, B and centroid without changing weight");
        Check(EvaluateFootSupportBalance(hull, PxVec3(2, 0, 0), 3).weight == .5f,
            "Changing B scale changes the fade interval");
        const float nan = std::numeric_limits<float>::quiet_NaN();
        for (const auto& invalid : {std::vector<PxVec3>{}, std::vector<PxVec3>{{0, 0, 0}, {1, 0, 0}},
            std::vector<PxVec3>{{0, 0, 0}, {1, 0, 0}, {2, 0, 0}},
            std::vector<PxVec3>{{nan, 0, 0}, {1, 0, 0}, {0, 0, 1}}})
        {
            const auto result = EvaluateFootSupportBalance(invalid, PxVec3(0));
            Check(result.weight == 0 && result.zone == FootSupportZone::Invalid && result.bufferHull.empty(),
                "Missing, collinear and nonfinite support cannot produce a buffer or positive weight");
        }
        Check(EvaluateFootSupportBalance(hull, PxVec3(nan)).zone == FootSupportZone::Invalid &&
                EvaluateFootSupportBalance(hull, PxVec3(0), 1).weight == 0 &&
                EvaluateFootSupportBalance(hull, PxVec3(0), nan).weight == 0,
            "Invalid COM and expansion scales fail safely");
        std::puts("PASS: A/B centroid, expansion, radial weight, boundaries, transforms and invalid input");
    }

    void TestFootSupportCandidates()
    {
        using namespace PARS;
        std::array<std::vector<PxVec3>, 2> feet{{
            {{-.2f, 0, -.1f}, {-.1f, 0, -.1f}, {-.1f, 0, .1f}, {-.2f, 0, .1f}},
            {{.1f, 0, -.1f}, {.2f, 0, -.1f}, {.2f, 0, .1f}, {.1f, 0, .1f}}}};
        std::vector<PxVec3> points = feet[0];
        points.insert(points.end(), feet[1].begin(), feet[1].end());
        const auto hull = BuildFootSupportHull(points);
        const auto left = EvaluateFootSupportCandidates(hull, PxVec3(-.12f, 3, 0), feet);
        const auto right = EvaluateFootSupportCandidates(hull, PxVec3(.12f, -3, 0), feet);
        Check(left.support == 0 && left.step == 1 && right.support == 1 && right.step == 0,
            "Candidate uses horizontal COM distance and assigns opposite feet");
        Check(EvaluateFootSupportCandidates(hull, PxVec3(0), feet).support == 0 &&
                EvaluateFootSupportCandidates(hull, PxVec3(0), feet, 1).support == 1,
            "Initial tie is deterministic and an existing candidate survives a tie");
        Check(EvaluateFootSupportCandidates(hull, PxVec3(.003f, 0, 0), feet, 0).support == 0 &&
                EvaluateFootSupportCandidates(hull, PxVec3(.02f, 0, 0), feet, 0).support == 1,
            "Small COM noise retains the candidate; clear transfer switches it");
        Check(EvaluateFootSupportCandidates(hull, PxVec3(.25f, 0, 0), feet, 0).support == -1 &&
                EvaluateFootSupportCandidates(hull, PxVec3(std::numeric_limits<float>::quiet_NaN()), feet).step == -1,
            "A exterior and invalid COM clear both candidates");
        auto missing = feet;
        missing[0].resize(2);
        Check(EvaluateFootSupportCandidates(hull, PxVec3(0), missing).support == -1,
            "Two valid foot polygons are required; a missing foot has no candidate pair");
        const auto transform = [](PxVec3 point)
        {
            return PxVec3(10 - point.z, point.y + 4, -7 + point.x);
        };
        auto moved = feet;
        for (auto& foot : moved)
        {
            for (auto& point : foot)
            {
                point = transform(point);
            }
        }
        auto movedHull = hull;
        for (auto& point : movedHull)
        {
            point = transform(point);
        }
        const auto relocated = EvaluateFootSupportCandidates(movedHull, transform(PxVec3(-.12f, 0, 0)), moved);
        Check(relocated.support == left.support && std::abs(relocated.distances[0] - left.distances[0]) < 1e-5f,
            "World translation and yaw preserve foot identity and COM distance");
        std::puts("PASS: COM foot candidates, switching hysteresis, validity and world transforms");
    }

    float SimulateAtRate(int fps, double seconds)
    {
        PhysicsWorld world;
        Check(world.Initialize(false), "Initialize");
        Check(world.CreateStaticBox(PxTransform(PxVec3(0, -0.25f, 0)), PxVec3(6, 0.25f, 6), "Test ground"), "Ground");
        auto box = world.CreateDynamicBox(PxTransform(PxVec3(0, 4, 0)), PxVec3(0.5f), 10, "Test box");
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

int main(int argc, char** argv)
{
    if (argc == 2 && std::strcmp(argv[1], "--footstep-only") == 0)
    {
        TestFootstepPlanner();
        TestFootstepIK();
        return 0;
    }
    if (argc == 2 && std::strcmp(argv[1], "--foot-candidates-only") == 0)
    {
        TestFootSupportCandidates();
        return 0;
    }
    TestFootProjectionRaycast();
    TestFootSupportHull();
    TestFootSupportBalance();
    TestFootSupportCandidates();
    TestFootstepPlanner();
    TestFootstepIK();
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
        auto box = world.CreateDynamicBox(PxTransform(PxVec3(0, 4, 0)), PxVec3(0.5f), 10, "Reset box");
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
        auto box = world.CreateDynamicBox(PxTransform(PxVec3(0, 4, 0)), PxVec3(0.5f), 10, "PVD test box");
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
        auto capsule = world.CreateQueryCapsule(PxVec3(2, 1, 0), 0.3f, 0.9f);
        Check(capsule != nullptr, "PVD character capsule");
        world.SetPvdSceneQueries(true);
        const auto moveCapsule = [&world, capsule](float)
        {
            world.MoveQueryCapsule(*capsule, PxVec3(0.02f, -0.1f, 0));
        };
        for (int reconnect = 0; reconnect < 3; ++reconnect)
        {
            // Movement sweeps leave query geometry in the previous PVD session.
            world.Step(moveCapsule);
            const auto steps = world.GetStepCount();
            const auto position = capsule->centre;
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
