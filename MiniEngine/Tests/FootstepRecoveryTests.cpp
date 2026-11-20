#include "../Model/FootstepPlanner.h"
#include "foundation/PxQuat.h"
#include <chrono>
#include <cstdio>
#include <cstdlib>

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
            std::fprintf(stderr, "FAIL: recovery regions: %s\n", message);
            std::exit(1);
        }
    }

    FootstepPolygon Rectangle(float minX, float maxX, float minZ, float maxZ)
    {
        return {{minX, 0, minZ}, {maxX, 0, minZ}, {maxX, 0, maxZ}, {minX, 0, maxZ}};
    }

    FootstepRecoveryInput Input()
    {
        FootstepRecoveryInput input;
        input.com = PxVec3(0, 1, 0);
        input.firstStepTime = .25f;
        input.followingStepTime = .3f;
        input.movingFoot = 1;
        input.stepping = true;
        input.normalTarget = PxVec3(.12f, 0, .3f);
        input.preferred = input.normalTarget;
        for (size_t side = 0; side < 2; ++side)
        {
            FootstepReachInput& reach = input.feet[side];
            const float x = side == 0 ? -.12f : .12f;
            reach.position = PxVec3(x, 0, 0);
            reach.hip = PxVec3(x, .65f, 0);
            reach.stepOrigin = reach.position;
            reach.legLength = 1;
            reach.minimumWidth = .168f;
            reach.outward = PxVec3(side == 0 ? -1.0f : 1.0f, 0, 0);
            reach.sole = Rectangle(x - .04f, x + .04f, -.1f, .1f);
            reach.available = true;
        }
        return input;
    }

    bool InAny(const std::vector<FootstepPolygon>& pieces, const PxVec3& point)
    {
        for (const FootstepPolygon& piece : pieces)
        {
            if (IsInsideFootSupportHull(piece, point))
            {
                return true;
            }
        }
        return false;
    }

    void TestPolygonGeometry()
    {
        const FootstepPolygon a = Rectangle(0, 2, 0, 2);
        const FootstepPolygon b = Rectangle(1, 3, 1, 3);
        Check(std::abs(FootstepPolygonArea(IntersectFootstepPolygons(a, b)) - 1) < 1e-5f,
            "intersection has the expected area");
        Check(std::abs(FootstepUnionArea({a, b}) - 7) < 1e-5f,
            "overlap is counted once in union area");
        Check(std::abs(FootstepUnionArea({a, a, a}) - 4) < 1e-5f,
            "duplicate pieces cannot bias mode selection");
        Check(std::abs(FootstepUnionArea({a, Rectangle(.5f, 1, .5f, 1)}) - 4) < 1e-5f,
            "contained pieces cannot inflate area");
        Check(std::abs(FootstepUnionArea({Rectangle(0, 1, 0, 1), Rectangle(1, 2, 0, 1)}) - 2) < 1e-5f,
            "opposite shared edges disappear from union boundary");
        Check(std::abs(FootstepUnionArea({Rectangle(0, 1, 0, 1), Rectangle(2, 3, 0, 1)}) - 2) < 1e-5f,
            "disconnected pieces retain the forbidden gap");
        Check((ClosestFootstepPoint(a, PxVec3(3, 0, 1)) - PxVec3(2, 0, 1)).magnitude() < 1e-5f,
            "preferred target projects to the closest edge");
        Check(IntersectFootstepPolygons(a, Rectangle(5, 6, 5, 6)).empty(),
            "disjoint polygons have no feasible intersection");
        for (uint32_t index = 0; index < 30; ++index)
        {
            const float offset = static_cast<float>(index) * .13f;
            FootstepPolygon first = Rectangle(0, 1, 0, 1);
            FootstepPolygon second = Rectangle(offset, offset + 1, .3f, 1.3f);
            const float expected = 2 - (std::max)(0.0f, 1 - offset) * .7f;
            const PxQuat rotation(.37f, PxVec3(0, 1, 0));
            for (PxVec3& point : first)
            {
                point = rotation.rotate(point) + PxVec3(20, 0, -15);
            }
            for (PxVec3& point : second)
            {
                point = rotation.rotate(point) + PxVec3(20, 0, -15);
            }
            Check(std::abs(FootstepUnionArea({first, second, first}) - expected) < 3e-5f,
                "rotated and translated overlapping polygons preserve exact union area");
        }
    }

    void TestModeChoice()
    {
        FootstepRecoverySettings settings;
        settings.maximumSteps = 1;
        FootstepRecoveryInput input = Input();
        input.velocity = PxVec3(0, 0, .6f);
        FootstepRecoverySnapshot result = SelectFootstepRecovery(input, settings);
        Check(result.hasTarget && result.mode == FootstepMode::Normal && result.modelFeasible,
            "ordinary forward recovery keeps the normal mode");
        Check(InAny(result.intersections[0], result.target), "normal target lies in both required regions");
        input.velocity = PxVec3(-.8f, 0, 0);
        input.com.z = .15f;
        result = SelectFootstepRecovery(input, settings);
        Check(result.overlapAreas[0] == 0 && result.hasTarget && result.mode == FootstepMode::FrontCross,
            "inward and forward demand selects front crossover");
        Check(result.target.x < input.feet[0].position.x && InAny(result.intersections[1], result.target),
            "front crossover can land beyond the stance foot instead of being clamped outward");
        const FootstepRecoverySnapshot front = result;
        input.normalTarget = PxVec3(10, 0, -10);
        input.preferred = PxVec3(-10, 0, -10);
        result = SelectFootstepRecovery(input, settings);
        Check(result.mode == front.mode && std::abs(result.overlapAreas[1] - front.overlapAreas[1]) < 1e-6f,
            "corrected and preferred targets cannot change physical recovery regions");
        input = Input();
        input.velocity = PxVec3(-.8f, 0, 0);
        input.com.z = -.15f;
        result = SelectFootstepRecovery(input, settings);
        Check(result.hasTarget && result.mode == FootstepMode::BackCross && result.target.z < 0,
            "inward and backward demand selects rear crossover");
        input.velocity = PxVec3(-10, 0, 0);
        result = SelectFootstepRecovery(input, settings);
        Check(result.hasTarget && !result.modelFeasible && result.status == FootstepRecoveryStatus::BestEffort,
            "unreachable recovery is reported as best effort rather than feasible");
        input = Input();
        input.feet[1].hip.y = 2;
        result = SelectFootstepRecovery(input, settings);
        Check(!result.hasTarget && result.status == FootstepRecoveryStatus::NoReach,
            "a fully unreachable fixed hip cannot fabricate a crossover region");
        input = Input();
        input.velocity = PxVec3(-.8f, 0, 0);
        input.com.z = .15f;
        input.acceptsTarget = [](const PxVec3&) -> bool
        {
            return false;
        };
        result = SelectFootstepRecovery(input, settings);
        Check(!result.hasTarget && result.poseRejected && result.poseChecks <= settings.maximumPoseChecks,
            "PA rejection cannot be promoted to an accepted target");
        input.locked = true;
        input.lockedTarget = PxVec3(.1f, 0, .2f);
        input.previousMode = FootstepMode::Normal;
        result = SelectFootstepRecovery(input, settings);
        Check(result.status == FootstepRecoveryStatus::Locked && result.target == input.lockedTarget && result.poseChecks == 0,
            "landing lock preserves the target and avoids a second pose search");
    }

    void TestCaptureAndTiming()
    {
        FootstepRecoverySettings settings;
        settings.maximumSteps = 1;
        FootstepRecoveryInput input = Input();
        input.velocity = PxVec3(.3f, 0, .5f);
        const FootstepRecoverySnapshot first = SelectFootstepRecovery(input, settings);
        const float omega = std::sqrt(9.81f);
        Check(std::abs(first.icp.x - .3f / omega) < 1e-6f && std::abs(first.icp.z - .5f / omega) < 1e-6f,
            "ICP uses actual COM and horizontal velocity");
        Check(!first.hasTarget || !first.modelFeasible || first.requiredSteps == 1,
            "one-step feasible candidates do not request unnecessary followup steps");
        const float growth = std::exp(omega * input.firstStepTime);
        for (const PxVec3& cop : input.feet[0].sole)
        {
            const PxVec3 future = first.icp * growth + cop * (1 - growth);
            for (const PxVec3& sole : input.feet[1].sole)
            {
                const PxVec3 local = sole - input.feet[1].position;
                Check(InAny(first.capture, future - local),
                    "one-step region contains analytically computed future ICP and finite-foot offsets");
            }
        }
        settings.maximumSteps = 3;
        const FootstepRecoverySnapshot multi = SelectFootstepRecovery(input, settings);
        Check(multi.steps == 3 && multi.capturePieceCount <= 13 && multi.usesFutureReachApproximation,
            "multistep computation is bounded and explicitly identifies the future reach approximation");
        for (const PxVec3& point : first.capture[0])
        {
            Check(InAny(multi.capture, point), "multistep region includes one-step recovery");
        }
        input.firstStepTime = .5f;
        settings.maximumSteps = 1;
        const FootstepRecoverySnapshot delayed = SelectFootstepRecovery(input, settings);
        Check((ClosestFootstepPoint(delayed.capture[0], input.preferred) -
            ClosestFootstepPoint(first.capture[0], input.preferred)).magnitude() > .01f,
            "time spent transferring load changes required landing positions");
        input.stepping = false;
        const FootstepRecoverySnapshot preview = SelectFootstepRecovery(input, settings);
        Check(preview.status == FootstepRecoveryStatus::Preview && !preview.hasTarget && preview.poseChecks == 0,
            "idle preview does not run PA pose search or issue a command");
        input = Input();
        input.com = PxVec3(0, 1, .15f);
        input.velocity = PxVec3(-.8f, 0, 0);
        const FootstepRecoverySnapshot beforeTurn = SelectFootstepRecovery(input, settings);
        const PxQuat yaw(PxHalfPi, PxVec3(0, 1, 0));
        input.com = yaw.rotate(input.com);
        input.velocity = yaw.rotate(input.velocity);
        input.forward = yaw.rotate(input.forward);
        input.preferred = yaw.rotate(input.preferred);
        input.normalTarget = yaw.rotate(input.normalTarget);
        for (FootstepReachInput& foot : input.feet)
        {
            foot.position = yaw.rotate(foot.position);
            foot.hip = yaw.rotate(foot.hip);
            foot.ankleOffset = yaw.rotate(foot.ankleOffset);
            foot.outward = yaw.rotate(foot.outward);
            foot.stepOrigin = yaw.rotate(foot.stepOrigin);
            for (PxVec3& corner : foot.sole)
            {
                corner = yaw.rotate(corner);
            }
        }
        const FootstepRecoverySnapshot turned = SelectFootstepRecovery(input, settings);
        Check(turned.mode == beforeTurn.mode && std::abs(turned.overlapAreas[1] - beforeTurn.overlapAreas[1]) < 1e-5f,
            "crossover selection follows character forward rather than world axes");
    }

    FootstepInput PlannerInput()
    {
        FootstepInput input;
        input.active = true;
        input.automaticSupport = true;
        input.idleAnimation = true;
        input.hasCom = true;
        input.hasVelocity = true;
        input.hasReference = true;
        input.fixedDeltaTime = 1.0f / 60;
        input.bodyWeight = 700;
        input.requestedFoot = 1;
        input.com = PxVec3(0, 1, .15f);
        input.velocity = PxVec3(-.8f, 0, 0);
        input.hull = Rectangle(-.16f, .16f, -.1f, .1f);
        input.zone = FootSupportZone::Buffer;
        input.revision = 1;
        const FootstepRecoveryInput geometry = Input();
        for (size_t side = 0; side < 2; ++side)
        {
            FootstepFootInput& foot = input.feet[side];
            foot.available = true;
            foot.hasContactSample = true;
            foot.grounded = true;
            foot.normal = PxVec3(0, 1, 0);
            foot.position = geometry.feet[side].position;
            foot.contactPosition = foot.position;
            foot.stanceOffset = foot.position;
            foot.hasSoleRectangle = true;
            std::copy(geometry.feet[side].sole.begin(), geometry.feet[side].sole.end(), foot.corners.begin());
            foot.reach = geometry.feet[side];
            foot.calfLength = .5f;
            foot.normalImpulse = input.bodyWeight * input.fixedDeltaTime * .5f;
        }
        input.acceptsLanding = [](size_t, const PxVec3&) -> bool
        {
            return true;
        };
        return input;
    }

    void TestTakeoffModeSelection()
    {
        FootstepSettings settings;
        FootstepInput input = PlannerInput();
        input.com.z = 0;
        input.velocity = PxVec3(-.04f, 0, 0);
        FootstepPlanner planner;
        planner.Update(input, settings);
        Check(planner.GetSnapshot().startCom.limited && !planner.GetSnapshot().needsStep && planner.GetSnapshot().startCount == 0,
            "A short prediction inside the support hull cannot start a crossover just by entering the cone");

        input.velocity = PxVec3(-.30f, 0, .12f);
        settings.landingComClampEnabled = false;
        planner.Reset();
        planner.Update(input, settings);
        Check(planner.GetSnapshot().feet[1].justStarted && planner.GetSnapshot().feet[1].target.recovery.mode == FootstepMode::Normal,
            "Without the cone preference the same recoverable demand starts a normal step");
        settings.landingComClampEnabled = true;
        for (const FootstepMode mode : {FootstepMode::FrontCross, FootstepMode::BackCross})
        {
            input = PlannerInput();
            input.com.z = 0;
            input.velocity = PxVec3(-.30f, 0, mode == FootstepMode::FrontCross ? .12f : -.12f);
            planner.Reset();
            planner.Update(input, settings);
            const FootstepSnapshot& step = planner.GetSnapshot();
            const FootstepCommand& command = step.feet[1];
            const size_t modeIndex = static_cast<size_t>(mode);
            Check(step.startCom.limited && !IsInsideFootSupportHull(input.hull, step.startCom.raw) &&
                step.candidates[1].recovery.overlapAreas[0] > 0 && step.candidates[1].recovery.overlapAreas[modeIndex] > 0 &&
                command.justStarted && command.target.recovery.hasTarget && command.target.recovery.mode == mode,
                "A valid crossover beats an available normal target on the first grounded cone sample");

            const PxVec3 firstTarget = command.target.position;
            input.feet[1].grounded = false;
            input.com.z = mode == FootstepMode::FrontCross ? -.15f : .15f;
            input.velocity = PxVec3(-.8f, 0, 0);
            ++input.completedStep;
            planner.Update(input, settings);
            Check(command.hasTakenOff && !command.motion.landingTargetLocked && command.target.recovery.modeLocked &&
                command.target.recovery.mode == mode && command.target.recovery.hasTarget &&
                IsInsideFootSupportHull(step.candidates[1].recovery.reachable[modeIndex], command.target.position) &&
                (command.target.position - firstTarget).magnitude() > 1e-4f,
                "Takeoff keeps the grounded cross direction while adjusting the landing within its reach");

            input.requestedFoot = 0;
            ++input.completedStep;
            planner.Update(input, settings);
            Check(command.target.recovery.mode == mode && step.candidates[1].recovery.hasTarget &&
                step.candidates[1].recovery.mode == mode, "A request for the other foot still replans the airborne foot in its own mode");
            const PxVec3 accepted = command.target.position;
            input.feet[0].reach.available = false;
            ++input.completedStep;
            planner.Update(input, settings);
            Check(command.target.recovery.mode == mode && command.target.position == accepted && !step.candidates[1].valid &&
                step.candidates[1].recovery.modeLocked && step.candidates[1].recovery.mode == mode,
                "Unavailable recovery geometry holds the last cross target and reports its committed mode");
        }

        input = PlannerInput();
        input.com.z = 0;
        input.velocity = PxVec3(-.30f, 0, .12f);
        input.acceptsLanding = [](size_t, const PxVec3& target) -> bool
        {
            return std::abs(target.z) < .10f;
        };
        planner.Reset();
        planner.Update(input, settings);
        const FootstepCommand& fallback = planner.GetSnapshot().feet[1];
        Check(planner.GetSnapshot().startCom.limited && fallback.justStarted && fallback.target.recovery.hasTarget &&
            fallback.target.recovery.poseRejected && fallback.target.recovery.mode == FootstepMode::Normal,
            "Rejected cross poses leave a pose check for a valid normal fallback");
    }

    void TestAirborneNormalStep()
    {
        const FootstepSettings settings;
        FootstepInput input = PlannerInput();
        input.com.z = 0;
        input.velocity = PxVec3(0, 0, .6f);
        bool acceptLanding = true;
        input.acceptsLanding = [&acceptLanding](size_t, const PxVec3&) -> bool
        {
            return acceptLanding;
        };
        unsigned searches = 0;
        input.findCrossStep = [&searches]() -> MotionMatchResult
        {
            ++searches;
            return {};
        };
        FootstepPlanner planner;
        planner.Update(input, settings);
        const FootstepSnapshot& step = planner.GetSnapshot();
        const FootstepCommand& command = step.feet[1];
        Check(command.justStarted && command.target.recovery.mode == FootstepMode::Normal, "The test step starts normal");
        input.feet[1].grounded = false;
        ++input.completedStep;
        planner.Update(input, settings);
        const PxVec3 takeoffTarget = command.target.position;
        input.velocity = PxVec3(-.8f, 0, 0);
        input.com.z = -.15f;
        ++input.completedStep;
        planner.Update(input, settings);
        Check(command.hasTakenOff && command.motion.progress > 0 && !command.motion.landingTargetLocked &&
            step.candidates[1].recovery.overlapAreas[2] > 0 && step.candidates[1].recovery.overlapAreas[0] == 0 &&
            command.target.recovery.mode == FootstepMode::Normal && command.target.recovery.status == FootstepRecoveryStatus::BestEffort &&
            IsInsideFootSupportHull(step.candidates[1].recovery.reachable[0], command.target.position) &&
            (command.target.position - takeoffTarget).magnitude() > 1e-4f && searches == 0,
            "Late back-cross demand adjusts only the normal landing and cannot start a cross motion search");
        const PxVec3 accepted = command.target.position;
        acceptLanding = false;
        input.feet[1].grounded = true;
        ++input.completedStep;
        planner.Update(input, settings);
        Check(command.phase == FootstepPhase::Airborne && command.target.position == accepted &&
            !step.candidates[1].recovery.hasTarget && step.candidates[1].recovery.modeLocked &&
            step.candidates[1].recovery.mode == FootstepMode::Normal,
            "Brief contact and rejected poses still report the airborne normal mode and hold its target");
        acceptLanding = true;
        input.feet[1].grounded = false;
        const uint32_t swingSamples = static_cast<uint32_t>(std::ceil(settings.stepDuration / input.fixedDeltaTime));
        for (uint32_t sample = 0; sample < swingSamples; ++sample)
        {
            ++input.completedStep;
            planner.Update(input, settings);
        }
        Check(command.motion.landingTargetLocked && command.motion.progress == 1, "The airborne step reaches its landing lock");
        input.feet[1].grounded = true;
        ++input.completedStep;
        planner.Update(input, settings);
        Check(command.justLanded && command.target.recovery.mode == FootstepMode::Normal, "The normal step lands in its takeoff mode");
        ++input.completedStep;
        planner.Update(input, settings);
        Check(command.justStarted && command.target.recovery.mode == FootstepMode::BackCross && searches == 1,
            "The next step can select the still-needed crossover after landing and load confirmation");
    }

    void TestPlannerIntegration()
    {
        FootstepInput input = PlannerInput();
        FootstepSettings settings;
        Check(settings.recovery.maximumSteps == 1 && !settings.pathAvoidanceEnabled,
            "Default recovery plans one step with path avoidance off");
        settings.pathAvoidanceEnabled = true;
        settings.recovery.transferTime = .025f;
        FootstepPlanner planner;
        planner.Update(input, settings);
        const FootstepCommand first = planner.GetSnapshot().feet[1];
        Check(first.justStarted && first.target.recovery.hasTarget && first.target.recovery.mode == FootstepMode::FrontCross,
            "a raw inward demand branches before normal COM and minimum-width corrections");
        Check(first.target.position.x < input.feet[0].position.x,
            "the legacy crossing clamp is not applied to the selected crossover target");
        Check(!first.target.landingCom.enabled && !first.target.landingCom.limited,
            "crossover debug metadata does not claim that the bypassed normal COM clamp was applied");
        Check(first.target.recovery.capture.empty() && !planner.GetSnapshot().candidates[1].recovery.capture.empty(),
            "prediction copies keep only decision metadata while the debug candidate keeps region geometry");
        planner.ReportRecoveryPath(1, first.target.position, false);
        ++input.completedStep;
        planner.Update(input, settings);
        const FootstepCommand replanned = planner.GetSnapshot().feet[1];
        Check((replanned.target.position - first.target.position).magnitude() >= .0099f ||
            !planner.GetSnapshot().candidates[1].recovery.hasTarget,
            "a failed path requests an alternative landing instead of repeating the same candidate");
        input.acceptsLanding = [](size_t, const PxVec3&) -> bool
        {
            return false;
        };
        ++input.completedStep;
        planner.Update(input, settings);
        Check((planner.GetSnapshot().feet[1].target.position - replanned.target.position).magnitude() < 1e-6f,
            "candidate rejection preserves an existing crossover goal instead of applying a normal clamp");
        input.acceptsLanding = [](size_t, const PxVec3&) -> bool
        {
            return true;
        };
        input.feet[1].grounded = false;
        for (uint32_t frame = 0; frame < 20; ++frame)
        {
            ++input.completedStep;
            planner.Update(input, settings);
        }
        const PxVec3 locked = planner.GetSnapshot().feet[1].target.position;
        Check(planner.GetSnapshot().feet[1].motion.landingTargetLocked, "integration keeps the existing 75 percent lock");
        planner.ReportRecoveryPath(1, locked, false);
        Check(planner.GetSnapshot().feet[1].target.recovery.pathRejected, "The locked path records the last rejection");
        settings.pathAvoidanceEnabled = false;
        input.velocity = PxVec3(1, 0, 2);
        ++input.completedStep;
        planner.Update(input, settings);
        Check((planner.GetSnapshot().feet[1].target.position - locked).magnitude() < 1e-6f,
            "new recovery choices cannot move the locked horizontal target");
        Check(planner.GetSnapshot().feet[1].motion.valid && !planner.GetSnapshot().feet[1].target.recovery.pathRejected,
            "Disabling avoidance clears the previous rejection while the swing remains active");
        input.active = false;
        input.velocity = PxVec3(0);
        input.feet[1].grounded = true;
        ++input.completedStep;
        planner.Update(input, settings);
        Check(!planner.GetSnapshot().IsInProgress() && planner.GetSnapshot().candidates[0].recovery.status ==
            FootstepRecoveryStatus::Preview && planner.GetSnapshot().candidates[0].recovery.poseChecks == 0,
            "initial kinematic idle can draw regions without commanding a step or searching poses");
    }
}

void TestFootstepRecovery()
{
    checks = 0;
    TestPolygonGeometry();
    TestModeChoice();
    TestCaptureAndTiming();
    TestTakeoffModeSelection();
    TestAirborneNormalStep();
    TestPlannerIntegration();
    FootstepRecoveryInput input = Input();
    input.velocity = PxVec3(-1.5f, 0, .5f);
    FootstepRecoverySettings settings;
    settings.maximumSteps = 3;
    const std::chrono::steady_clock::time_point started = std::chrono::steady_clock::now();
    double maximum = 0;
    for (uint32_t frame = 0; frame < 120; ++frame)
    {
        input.com.z = static_cast<float>(frame % 10) * .01f;
        const FootstepRecoverySnapshot result = SelectFootstepRecovery(input, settings);
        Check(result.capturePieceCount <= 13 && result.poseChecks <= settings.maximumPoseChecks,
            "runtime work stays within configured region and pose bounds");
        maximum = (std::max)(maximum, result.elapsedMilliseconds);
    }
    const double total = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - started).count();
    std::printf("PASS: %u recovery-region checks; 120 three-step decisions avg %.3f ms, max %.3f ms (geometry only)\n",
        checks, total / 120, maximum);
}
