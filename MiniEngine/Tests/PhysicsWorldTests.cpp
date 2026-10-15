#include "PhysicsWorld.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits>

using namespace physx;

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
        auto capsule = world.CreateCapsuleController(PxExtendedVec3(2, 1, 0), 0.3f, 0.9f);
        Check(capsule != nullptr, "PVD character capsule");
        const auto moveCapsule = [capsule](float dt)
        {
            capsule->move(PxVec3(0.02f, -0.1f, 0), 0.001f, dt, PxControllerFilters());
        };
        for (int reconnect = 0; reconnect < 3; ++reconnect)
        {
            // Controller sweeps leave query geometry in the previous PVD session.
            world.Step(moveCapsule);
            const auto steps = world.GetStepCount();
            const auto position = capsule->getPosition();
            Check(world.ConnectPvd(true) && world.IsPvdConnected(),
                "Reconnect after recording character scene queries");
            Check(world.GetStepCount() == steps && capsule->getPosition() == position,
                "Reconnect preserves paused character state");
            world.Step(moveCapsule);
            Check(capsule->getPosition().x > position.x, "Character moves after reconnect");
        }
        capsule->release();
        std::puts("PASS: live PVD, paused-scene reconnect and character queries after reconnect");
    }
    world.Shutdown();
    Check(world.GetErrorCount() == 0, "No errors after reinitialize/PVD shutdown");
    std::puts("PASS: PhysicsWorld timing, collision, reset and lifecycle");
    return 0;
}
