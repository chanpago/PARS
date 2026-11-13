#include "../Model/FootstepPlanner.h"
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>

using namespace PARS;
using namespace physx;

namespace
{
    unsigned checks = 0;

    void Check(bool condition, const char* message)
    {
        ++checks;
        if (!condition)
        {
            std::fprintf(stderr, "FAIL: %s\n", message);
            std::exit(1);
        }
    }

    bool Near(const PxVec3& a, const PxVec3& b)
    {
        return (a - b).magnitude() < 1e-4f;
    }

    FootstepInput Standing()
    {
        FootstepInput input;
        input.active = true;
        input.automaticSupport = true;
        input.hasCom = true;
        input.hasVelocity = true;
        input.com = PxVec3(0, 1, 0);
        input.velocity = PxVec3(0, 0, 2);
        input.hull = BuildFootSupportHull({{-.3f, 0, -.2f}, {.3f, 0, -.2f},
            {.3f, 0, .2f}, {-.3f, 0, .2f}});
        input.zone = FootSupportZone::Inside;
        input.fixedDeltaTime = 1.0f / 60.0f;
        input.bodyWeight = 700;
        input.completedStep = 1;
        input.revision = 1;
        for (size_t side = 0; side < input.feet.size(); ++side)
        {
            FootstepFootInput& foot = input.feet[side];
            foot.position = PxVec3(side == 0 ? -.12f : .12f, 0, 0);
            foot.contactPosition = foot.position;
            foot.normal = PxVec3(0, 1, 0);
            foot.calfLength = .45f;
            foot.normalImpulse = input.bodyWeight * input.fixedDeltaTime * .5f;
            foot.available = true;
            foot.hasContactSample = true;
            foot.grounded = true;
        }
        // 플래너의 계약만 분리해 검사하며 실제 물리 가능성은 별도 IK 입력이 담당한다.
        input.evaluateReachability = [](size_t, const PxVec3&, const PxVec3&, float, float) -> FootstepReachability
        {
            FootstepReachability result;
            result.reachable = true;
            result.requiredDuration = .2f;
            return result;
        };
        return input;
    }

    void Advance(FootstepPlanner& planner, FootstepInput& input, const FootstepSettings& settings, unsigned count)
    {
        for (unsigned sample = 0; sample < count; ++sample)
        {
            ++input.completedStep;
            planner.Update(input, settings);
        }
    }

    void TestPredictionAndFixedTarget()
    {
        FootstepPlanner planner;
        FootstepSettings settings;
        FootstepInput input = Standing();
        unsigned evaluations = 0;
        input.evaluateReachability = [&evaluations](size_t, const PxVec3&, const PxVec3&, float, float) -> FootstepReachability
        {
            ++evaluations;
            FootstepReachability result;
            result.reachable = true;
            result.requiredDuration = .2f;
            return result;
        };
        planner.Update(input, settings);
        const FootstepSnapshot start = planner.GetSnapshot();
        Check(start.justStarted && start.stepFoot == 1 && start.supportFoot == 0, "A contact foot supports the first step");
        Check(Near(start.target, PxVec3(0, 0, 1.2f)), "Landing is the 0.6 second COM projection without an offset");
        Check(Near(start.roleSelectionCom, PxVec3(0, 0, .6f)), "Role selection uses the COM midpoint");
        Check(std::abs(start.motion.duration - .3f) < 1e-6f && start.motion.landingTargetLocked,
            "The 0.3 second step starts with a locked landing target");
        input.com = PxVec3(1, 1, 0);
        input.velocity = PxVec3(-4, 0, 0);
        input.feet[1].grounded = false;
        input.feet[0].normal = PxVec3(.1f, 1, 0).getNormalized();
        settings.landingPredictionTime = 1;
        settings.stepDuration = 1;
        Advance(planner, input, settings, 6);
        const FootstepSnapshot& moving = planner.GetSnapshot();
        Check(Near(moving.target, start.target) && Near(moving.targetNormal, start.targetNormal),
            "COM, terrain measurements and tuning cannot retarget an active step");
        Check(moving.stepFoot == start.stepFoot && moving.supportFoot == start.supportFoot,
            "Roles stay locked until contact");
        Check(evaluations == 1, "Active steps do not evaluate new candidates");
        Check(std::abs(moving.motion.duration - .3f) < 1e-6f, "An active step keeps its original duration");
        const uint64_t count = moving.startCount;
        planner.Update(input, settings);
        Check(planner.GetSnapshot().startCount == count, "Repeated physics snapshots do not restart the step");
    }

    void TestRoleMidpoint()
    {
        FootstepPlanner planner;
        FootstepSettings settings;
        settings.minimumStanceDistance = 0;
        FootstepInput input = Standing();
        input.com.x = -.1f;
        input.velocity = PxVec3(1, 0, 0);
        planner.Update(input, settings);
        Check(planner.GetSnapshot().supportFoot == 1 && planner.GetSnapshot().stepFoot == 0,
            "Future COM midpoint can select a different support foot than current COM");
    }

    void TestMinimumDistanceAndSegment()
    {
        FootstepPlanner planner;
        FootstepSettings settings;
        FootstepInput input = Standing();
        input.velocity.z = .5f;
        planner.Update(input, settings);
        Check(!planner.GetSnapshot().IsInProgress() &&
            planner.GetSnapshot().reason == FootstepReason::MinimumDistanceConflict,
            "No target is invented when the whole segment is inside the minimum stance distance");
        settings.minimumStanceDistance = 0;
        Advance(planner, input, settings, 1);
        Check(planner.GetSnapshot().justStarted, "Minimum stance distance can be disabled");

        planner.Reset();
        settings = FootstepSettings();
        input = Standing();
        input.feet[1].grounded = false;
        input.velocity.z = .5f;
        planner.Update(input, settings);
        Check(planner.GetSnapshot().justStarted && !planner.GetSnapshot().candidates[1].minimumDistanceRequired,
            "Minimum stance distance is only activated in actual double support");

        planner.Reset();
        input = Standing();
        input.evaluateReachability = [](size_t, const PxVec3& position, const PxVec3&, float, float) -> FootstepReachability
        {
            FootstepReachability result;
            result.reachable = position.z <= .8f;
            result.requiredDuration = .2f;
            return result;
        };
        planner.Update(input, settings);
        const FootstepSnapshot& state = planner.GetSnapshot();
        const PxVec3 path = PxVec3(0, 0, 1.2f) - input.feet[1].position;
        const PxVec3 delta = state.target - input.feet[1].position;
        Check(state.justStarted && state.candidates[1].reachLimited, "An unreachable COM target is shortened");
        Check(delta.cross(path).magnitude() < 1e-4f && state.target.z <= .8001f && state.target.z > .79f,
            "The selected reachable point stays on the segment and approaches its feasible boundary");
        Check(state.candidates[1].stanceDistance >= settings.minimumStanceDistance - 1e-4f,
            "Reach limiting preserves the required stance distance");
    }

    void TestTimeBudgetAndGroundPlane()
    {
        FootstepPlanner planner;
        FootstepSettings settings;
        FootstepInput input = Standing();
        input.evaluateReachability = [](size_t, const PxVec3& position, const PxVec3&, float, float) -> FootstepReachability
        {
            FootstepReachability result;
            result.reachable = true;
            result.requiredDuration = position.z * .4f;
            return result;
        };
        planner.Update(input, settings);
        Check(planner.GetSnapshot().justStarted && planner.GetSnapshot().target.z <= .7501f &&
            planner.GetSnapshot().target.z > .74f, "Time feasibility participates in target selection");

        planner.Reset();
        input = Standing();
        input.evaluateReachability = [](size_t, const PxVec3&, const PxVec3&, float, float) -> FootstepReachability
        {
            FootstepReachability result;
            result.reachable = true;
            result.requiredDuration = 1;
            return result;
        };
        planner.Update(input, settings);
        Check(planner.GetSnapshot().reason == FootstepReason::InsufficientTime &&
            !planner.GetSnapshot().IsInProgress(), "No step starts with an over-budget plan");

        planner.Reset();
        input = Standing();
        input.feet[0].normal = PxVec3(0, 1, .2f).getNormalized();
        input.feet[0].contactPosition.y = -.02f;
        planner.Update(input, settings);
        const FootstepSnapshot& slope = planner.GetSnapshot();
        Check(slope.justStarted && std::abs(slope.target.x) < 1e-5f && std::abs(slope.target.z - 1.2f) < 1e-5f,
            "Ground projection preserves predicted COM XZ");
        Check(std::abs((slope.target - input.feet[0].contactPosition).dot(input.feet[0].normal)) < 1e-5f,
            "Landing lies on the actual stance contact plane");
    }

    void TestContactAndTransfer()
    {
        FootstepPlanner planner;
        const FootstepSettings settings;
        FootstepInput input = Standing();
        planner.Update(input, settings);
        const PxVec3 target = planner.GetSnapshot().target;
        input.feet[1].grounded = false;
        Advance(planner, input, settings, 18);
        Check(planner.GetSnapshot().IsInProgress() && planner.GetSnapshot().motion.deadlineMissed,
            "The deadline cannot manufacture ground contact");
        Check(Near(planner.GetSnapshot().target, target), "A missed deadline does not change the landing point");
        input.feet[1].position = target;
        input.feet[1].contactPosition = target;
        input.feet[1].grounded = true;
        Advance(planner, input, settings, 1);
        Check(planner.GetSnapshot().justLanded && planner.GetSnapshot().transfer.active,
            "Actual contact ends the step and starts load confirmation");
        input.com.z = .4f;
        input.feet[1].normalImpulse = input.bodyWeight * input.fixedDeltaTime * .6f;
        input.feet[0].normalImpulse = input.bodyWeight * input.fixedDeltaTime * .4f;
        Advance(planner, input, settings, 2);
        Check(!planner.GetSnapshot().IsInProgress(), "One or two load samples cannot transfer support");
        Advance(planner, input, settings, 1);
        Check(planner.GetSnapshot().justStarted && planner.GetSnapshot().supportFoot == 1 &&
            planner.GetSnapshot().stepFoot == 0, "Continuous support load permits the next midpoint-selected step");
    }

    void TestInvalidInputs()
    {
        FootstepPlanner planner;
        FootstepSettings settings;
        FootstepInput input = Standing();
        input.evaluateReachability = {};
        planner.Update(input, settings);
        Check(planner.GetSnapshot().reason == FootstepReason::NoReachability,
            "Missing physical planning data cannot silently allow a step");
        planner.Reset();
        input = Standing();
        settings.stepDuration = std::numeric_limits<float>::quiet_NaN();
        planner.Update(input, settings);
        Check(planner.GetSnapshot().reason == FootstepReason::InvalidSettings, "Invalid duration is rejected");
        settings = FootstepSettings();
        Advance(planner, input, settings, 1);
        input.outsideBufferReleased = true;
        Advance(planner, input, settings, 1);
        Check(planner.GetSnapshot().phase == FootstepPhase::Aborted, "Support release aborts the step");
        input.outsideBufferReleased = false;
        Advance(planner, input, settings, 1);
        Check(planner.GetSnapshot().phase == FootstepPhase::Aborted, "Support release stays latched");
        ++input.revision;
        Advance(planner, input, settings, 1);
        Check(planner.GetSnapshot().justStarted, "A new contact revision clears the abort latch");
    }
}

void TestFootstepPlanner()
{
    TestPredictionAndFixedTarget();
    TestRoleMidpoint();
    TestMinimumDistanceAndSegment();
    TestTimeBudgetAndGroundPlane();
    TestContactAndTransfer();
    TestInvalidInputs();
    std::printf("PASS: %u physical footstep planning checks\n", checks);
}
