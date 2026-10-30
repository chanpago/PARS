#include "../Model/FootstepPlanner.h"
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

    bool Near(float a, float b)
    {
        return std::abs(a - b) < 1e-5f;
    }

    FootstepInput Standing()
    {
        FootstepInput input;
        input.active = input.automaticSupport = input.idleAnimation = true;
        input.hasCom = input.hasVelocity = input.hasReference = true;
        input.com = PxVec3(0, 1, 0);
        input.velocity = PxVec3(0, 0, 1);
        input.hull = BuildFootSupportHull({{-.25f, .4f, -.15f}, {.25f, .4f, -.15f},
            {.25f, .4f, .15f}, {-.25f, .4f, .15f}});
        input.zone = FootSupportZone::Inside;
        input.projectionHeight = .4f;
        input.blueFoot = 1;
        input.completedStep = 10;
        input.revision = 1;
        for (size_t side = 0; side < input.feet.size(); ++side)
        {
            auto& foot = input.feet[side];
            foot.available = foot.hasContactSample = foot.grounded = true;
            const float x = side == 0 ? -.12f : .12f;
            foot.position = PxVec3(x, .4f, 0);
            foot.normal = PxVec3(0, 1, 0);
            foot.stanceOffset = PxVec3(x, 0, 0);
        }
        return input;
    }

    void TestStartConditions()
    {
        FootstepPlanner planner;
        FootstepSettings settings;
        auto input = Standing();
        input.velocity = PxVec3(0);
        planner.Update(input, settings);
        Check(!planner.GetSnapshot().needsStep && planner.GetSnapshot().startCount == 0,
            "Stationary COM inside A does not start a step");
        input.velocity.z = .5f;
        ++input.completedStep;
        planner.Update(input, settings);
        Check(!planner.GetSnapshot().needsStep, "Predicted COM on A boundary counts as inside");
        input.velocity.z = 1;
        ++input.completedStep;
        planner.Update(input, settings);
        const auto started = planner.GetSnapshot();
        Check(started.justStarted && started.stepFoot == 1 && started.supportFoot == 0 && started.startCount == 1,
            "Predictive exit starts the blue foot while actual COM is still inside A");
        Check(Near(started.predictedCom.z, .3f) && Near(started.predictedCom.y, .4f),
            "Start prediction uses 0.3 seconds and support projection height");
        Check(Near(started.target.x, .096f) && Near(started.target.z, .3f) && Near(started.target.y, .4f),
            "Target adds 0.8 times the reference foot offset at the ground plane height");
        Check(started.phase == FootstepPhase::AwaitingTakeoff && started.landingCount == 0,
            "Initial standing contact is not a completed landing");

        for (int blocked = 0; blocked < 6; ++blocked)
        {
            input = Standing();
            planner.Reset();
            if (blocked == 0)
            {
                input.feet[0].grounded = false;
            }
            else if (blocked == 1)
            {
                input.blueFoot = -1;
            }
            else if (blocked == 2)
            {
                input.hasReference = false;
            }
            else if (blocked == 3)
            {
                input.feet[1].hasContactSample = false;
            }
            else if (blocked == 4)
            {
                input.automaticSupport = false;
            }
            else
            {
                input.idleAnimation = false;
            }
            planner.Update(input, settings);
            Check(planner.GetSnapshot().startCount == 0 && !planner.GetSnapshot().hasTarget,
                "Missing actual support, blue foot, reference, contact sample or supported mode blocks start");
        }
        input = Standing();
        input.feet[1].grounded = false;
        planner.Reset();
        planner.Update(input, settings);
        Check(planner.GetSnapshot().phase == FootstepPhase::Airborne && planner.GetSnapshot().hasTarget,
            "A blue foot already airborne may start when the opposite foot has actual contact");
    }

    void TestStepLifecycle()
    {
        FootstepPlanner planner;
        const FootstepSettings settings;
        auto input = Standing();
        planner.Update(input, settings);
        const auto start = planner.GetSnapshot();
        input.blueFoot = 0;
        planner.Update(input, settings);
        Check(planner.GetSnapshot().startCount == 1 && planner.GetSnapshot().stepFoot == 1,
            "Repeated reads of one completed physics step cannot start again");
        ++input.completedStep;
        input.velocity.z = 2;
        planner.Update(input, settings);
        Check(planner.GetSnapshot().stepFoot == 1 && planner.GetSnapshot().startCount == 1 &&
            Near(planner.GetSnapshot().target.z, .6f), "Candidate flip keeps the selected foot and updates its target");
        Check((planner.GetSnapshot().initialTarget - start.initialTarget).magnitude() < 1e-6f &&
            Near(planner.GetSnapshot().initialDistance, start.initialDistance),
            "Live target updates retain the initial target and travel distance for the future trajectory");
        input.com.z = .2f;
        input.zone = FootSupportZone::Buffer;
        input.blueFoot = -1;
        ++input.completedStep;
        planner.Update(input, settings);
        Check(planner.GetSnapshot().hasTarget && planner.GetSnapshot().stepFoot == 1,
            "Losing the raw blue candidate outside A does not discard a step already in progress");
        input.feet[1].grounded = false;
        ++input.completedStep;
        planner.Update(input, settings);
        Check(planner.GetSnapshot().phase == FootstepPhase::Airborne, "Actual loss of contact records takeoff");
        input.feet[0].grounded = false;
        ++input.completedStep;
        planner.Update(input, settings);
        Check(!planner.GetSnapshot().hasTarget && planner.GetSnapshot().stepFoot == 1 &&
            planner.GetSnapshot().reason == FootstepReason::NoStanceContact,
            "Lost stance contact suspends output without reselecting the moving foot");
        input.feet[0].grounded = true;
        ++input.completedStep;
        planner.Update(input, settings);
        Check(planner.GetSnapshot().hasTarget && planner.GetSnapshot().startCount == 1,
            "Regained stance contact resumes the existing target without another start");
        input.feet[1].grounded = true;
        ++input.completedStep;
        planner.Update(input, settings);
        Check(planner.GetSnapshot().justLanded && planner.GetSnapshot().landingCount == 1 &&
            !planner.GetSnapshot().hasTarget && planner.GetSnapshot().startCount == 1,
            "Recontact after takeoff completes once and does not start again in the same step");
        input.zone = FootSupportZone::Inside;
        input.com.z = 0;
        input.blueFoot = 0;
        ++input.completedStep;
        planner.Update(input, settings);
        Check(planner.GetSnapshot().justStarted && planner.GetSnapshot().startCount == 2 &&
            planner.GetSnapshot().stepFoot == 0, "The following physics step evaluates the latest blue foot");
    }

    void TestAbortAndInvalidInputs()
    {
        FootstepPlanner planner;
        const FootstepSettings settings;
        auto input = Standing();
        planner.Update(input, settings);
        input.feet[1].grounded = false;
        input.hasVelocity = false;
        ++input.completedStep;
        planner.Update(input, settings);
        Check(planner.GetSnapshot().phase == FootstepPhase::Airborne && !planner.GetSnapshot().hasTarget,
            "Invalid prediction input does not erase actual takeoff history");
        input.feet[1].grounded = true;
        input.hasVelocity = true;
        input.zone = FootSupportZone::Outside;
        ++input.completedStep;
        planner.Update(input, settings);
        Check(planner.GetSnapshot().phase == FootstepPhase::Aborted && planner.GetSnapshot().abortCount == 1 &&
            planner.GetSnapshot().landingCount == 0 && !planner.GetSnapshot().hasTarget,
            "Actual B exit takes priority over landing and clears the active target");
        Check(!planner.GetSnapshot().candidates[0].valid && !planner.GetSnapshot().candidates[1].valid,
            "Aborted output cannot reuse a formerly valid candidate");
        input.zone = FootSupportZone::Inside;
        ++input.completedStep;
        planner.Update(input, settings);
        Check(planner.GetSnapshot().phase == FootstepPhase::Aborted && planner.GetSnapshot().startCount == 1,
            "Returning inside B does not rearm stepping");
        ++input.revision;
        planner.Update(input, settings);
        Check(planner.GetSnapshot().justStarted && planner.GetSnapshot().abortCount == 0,
            "A reset or PA generation change removes stale state even at the same step number");
        input.rootReleased = true;
        ++input.completedStep;
        planner.Update(input, settings);
        Check(planner.GetSnapshot().phase == FootstepPhase::Aborted && !planner.GetSnapshot().hasTarget,
            "Existing root support release also blocks step output");
        planner.Reset();
        input = Standing();
        input.velocity.x = std::numeric_limits<float>::quiet_NaN();
        planner.Update(input, settings);
        Check(!planner.GetSnapshot().hasPrediction && !planner.GetSnapshot().hasTarget,
            "Nonfinite input cannot produce prediction or target");
        planner.Reset();
        input = Standing();
        input.zone = FootSupportZone::Invalid;
        input.hull.clear();
        planner.Update(input, settings);
        Check(planner.GetSnapshot().phase != FootstepPhase::Aborted && !planner.GetSnapshot().hasTarget,
            "Missing A/B is not fabricated as a geometric B exit");
    }

    void TestTargetGeometry()
    {
        FootstepPlanner planner;
        FootstepSettings settings;
        auto input = Standing();
        input.velocity = PxVec3(100, 100, 0);
        input.feet[1].position.x = 10;
        planner.Update(input, settings);
        const auto capped = planner.GetSnapshot().candidates[1];
        Check(capped.valid && capped.lengthLimited && Near(capped.stanceDistance, 2),
            "Maximum step length is two meters from the opposite foot");
        Check(capped.travelDistance > 2 && Near(capped.position.y, .4f),
            "The cap is not measured from the moving foot and ignores vertical COM velocity");
        planner.Reset();
        input = Standing();
        input.feet[0].normal = PxVec3(0, .8f, .6f);
        planner.Update(input, settings);
        const auto slope = planner.GetSnapshot().candidates[1];
        Check(slope.valid && std::abs((slope.position - input.feet[0].position).dot(input.feet[0].normal)) < 1e-5f,
            "Landing lies on the actual stance-normal plane through the stance sole");
        input.feet[0].normal = PxVec3(1, 0, 0);
        ++input.completedStep;
        planner.Update(input, settings);
        Check(!planner.GetSnapshot().hasTarget, "A wall normal cannot create a landing plane");

        planner.Reset();
        input = Standing();
        input.velocity = PxVec3(-100, 0, 100);
        planner.Update(input, settings);
        Check(planner.GetSnapshot().target.x < input.feet[0].position.x,
            "Crossing suppression is off by default");
        settings.preventCrossing = true;
        ++input.completedStep;
        planner.Update(input, settings);
        const auto crossed = planner.GetSnapshot().candidates[1];
        Check(crossed.valid && crossed.crossingLimited &&
            Near(crossed.position.x - input.feet[0].position.x, .1f) && crossed.stanceDistance <= 2.00001f,
            "Optional crossing boundary and maximum radius remain satisfied together");

        planner.Reset();
        input = Standing();
        settings = {};
        // 기준 방향을 90도 돌려도 발 간격 보정이 월드 X축에 고정되지 않아야 한다.
        input.supportRight = PxVec3(0, 0, 1);
        input.feet[0].stanceOffset = PxVec3(0, 0, -.12f);
        input.feet[1].stanceOffset = PxVec3(0, 0, .12f);
        planner.Update(input, settings);
        Check(Near(planner.GetSnapshot().target.x, 0) && Near(planner.GetSnapshot().target.z, .396f),
            "Reference stance offset follows the supplied support heading");
        settings.maximumStepLength = -1;
        ++input.completedStep;
        planner.Update(input, settings);
        Check(!planner.GetSnapshot().hasTarget && planner.GetSnapshot().reason == FootstepReason::InvalidSettings,
            "Invalid settings clear output rather than retaining a stale target");
    }
}

void TestFootstepPlanner()
{
    TestStartConditions();
    TestStepLifecycle();
    TestAbortAndInvalidInputs();
    TestTargetGeometry();
    std::printf("PASS: %u procedural footstep checks (no PhysX simulation)\n", checks);
}
