#include "../Model/FootstepPlanner.h"
#include <foundation/PxQuat.h>
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

    FootstepSettings TestSettings()
    {
        FootstepSettings settings;
        settings.startPredictionTime = .3f;
        settings.landingPredictionTime = .3f;
        settings.stepDuration = .3f;
        settings.liftHeightScale = .15f;
        settings.minimumStanceWidthRatio = .7f;
        // 연속 하중 판정 검사는 기본 튜닝과 분리해 하중 기준과 확인 시간을 명시한다.
        settings.supportLoadRatio = .5f;
        settings.supportConfirmationTime = .05f;
        return settings;
    }

    FootstepInput Standing()
    {
        FootstepInput input;
        input.active = true;
        input.automaticSupport = true;
        input.idleAnimation = true;
        input.hasCom = true;
        input.hasVelocity = true;
        input.hasReference = true;
        input.com = PxVec3(0, 1, 0);
        input.velocity = PxVec3(0, 0, 1);
        input.hull = BuildFootSupportHull({{-.25f, .4f, -.15f}, {.25f, .4f, -.15f},
            {.25f, .4f, .15f}, {-.25f, .4f, .15f}});
        input.zone = FootSupportZone::Inside;
        input.projectionHeight = .4f;
        input.fixedDeltaTime = 1.0f / 60;
        input.bodyWeight = 1000;
        input.requestedFoot = 1;
        input.completedStep = 10;
        input.revision = 1;
        for (size_t side = 0; side < input.feet.size(); ++side)
        {
            FootstepFootInput& foot = input.feet[side];
            foot.available = true;
            foot.hasContactSample = true;
            foot.grounded = true;
            const float x = side == 0 ? -.12f : .12f;
            foot.position = PxVec3(x, .4f, 0);
            foot.contactPosition = foot.position;
            foot.normal = PxVec3(0, 1, 0);
            foot.stanceOffset = PxVec3(x, 0, 0);
            foot.hasGroundProjection = true;
            foot.calfLength = .45f;
            foot.normalImpulse = input.bodyWeight * .5f * input.fixedDeltaTime;
        }
        return input;
    }

    void Advance(FootstepPlanner& planner, FootstepInput& input, const FootstepSettings& settings, unsigned count = 1)
    {
        for (unsigned sample = 0; sample < count; ++sample)
        {
            ++input.completedStep;
            planner.Update(input, settings);
            for (const FootstepCommand& command : planner.GetSnapshot().feet)
            {
                if (command.IsInProgress() && command.motion.valid)
                {
                    Check((SampleFootstepPosition(command, 0) - command.motion.position).magnitude() < 1e-5f,
                        "Collision trajectory sampling at the current time matches the live drive target");
                }
            }
        }
    }

    void TestWalkingMotionSearch()
    {
        FootstepSettings settings = TestSettings();
        settings.stepDuration = .15f;
        settings.recovery.maximumSteps = 1;
        FootstepInput input = Standing();
        input.com.y = 1.4f;
        input.acceptsLanding = [](size_t, const PxVec3&) -> bool
        {
            return true;
        };
        for (size_t side = 0; side < input.feet.size(); ++side)
        {
            FootstepFootInput& foot = input.feet[side];
            foot.reach.available = true;
            foot.reach.position = foot.position;
            foot.reach.hip = foot.position + PxVec3(0, .65f, 0);
            foot.reach.legLength = 1.0f;
            foot.reach.sole = {foot.position + PxVec3(-.04f, 0, -.1f), foot.position + PxVec3(.04f, 0, -.1f),
                foot.position + PxVec3(.04f, 0, .1f), foot.position + PxVec3(-.04f, 0, .1f)};
        }
        uint32_t searches = 0;
        WalkingSwing lastSwing;
        input.findWalkingStep = [&searches, &lastSwing](const WalkingSwing& swing) -> MotionMatchResult
        {
            lastSwing = swing;
            ++searches;
            return {3, .5f, .125f};
        };
        FootstepPlanner planner;
        planner.Update(input, settings);
        Check(planner.GetSnapshot().feet[1].justStarted && planner.GetSnapshot().feet[1].target.recovery.mode == FootstepMode::Normal &&
                searches == 1 && lastSwing.movingFoot == 1 && lastSwing.mode == FootstepMode::Normal,
            "A committed normal step searches Walking for the requested foot");
        input.velocity = PxVec3(-1.0f, 0, 0);
        input.com.z = .15f;
        Advance(planner, input, settings);
        const FootstepCommand& command = planner.GetSnapshot().feet[1];
        Check(!command.justStarted && command.target.recovery.mode == FootstepMode::FrontCross && searches == 2 &&
                lastSwing.movingFoot == 1 && lastSwing.mode == FootstepMode::FrontCross &&
                command.motionSearchStep == input.completedStep && command.motionMatch.clipIndex == 3 &&
                command.motionMatch.clipTimeSeconds == .5f && command.motionMatch.score == .125f,
            "A grounded change of step mode selects the matching swing from Walking");
        const uint64_t searchStep = command.motionSearchStep;
        planner.Update(input, settings);
        input.feet[1].grounded = false;
        input.com.z = -.15f;
        Advance(planner, input, settings, 3);
        Check(searches == 2 && command.motionSearchStep == searchStep, "Repeated ticks and airborne retargets keep the takeoff match");
        planner.Reset();
        input.feet[1].grounded = true;
        Advance(planner, input, settings);
        Check(command.justStarted && command.target.recovery.mode == FootstepMode::BackCross &&
                command.motionSearchStep == input.completedStep && searches == 3 &&
                lastSwing.movingFoot == 1 && lastSwing.mode == FootstepMode::BackCross,
            "A new back-cross command starts its own motion search");
        planner.Reset();
        input.findWalkingStep = {};
        Advance(planner, input, settings);
        Check(command.justStarted && command.target.recovery.mode == FootstepMode::BackCross && command.motionSearchStep == UINT64_MAX,
            "A cross step waits for a usable motion query after a reset");
        input.findWalkingStep = [&searches](const WalkingSwing&) -> MotionMatchResult
        {
            ++searches;
            return {};
        };
        Advance(planner, input, settings);
        const uint64_t firstReadyStep = input.completedStep;
        Advance(planner, input, settings, 2);
        Check(searches == 4 && command.motionSearchStep == firstReadyStep && command.motionMatch.clipIndex == UINT32_MAX,
            "A cross step records one unsuccessful search on the first usable sample");
    }

    void TestPredictionAndTrajectory()
    {
        FootstepPlanner planner;
        const FootstepSettings settings = TestSettings();
        FootstepInput input = Standing();
        input.velocity = PxVec3(0);
        planner.Update(input, settings);
        Check(!planner.GetSnapshot().needsStep && planner.GetSnapshot().startCount == 0,
            "Stationary COM inside the hull does not request motion");
        input.velocity.z = .5f;
        Advance(planner, input, settings);
        Check(!planner.GetSnapshot().needsStep, "The hull boundary counts as inside");
        input.velocity.z = 1;
        Advance(planner, input, settings);
        const FootstepSnapshot started = planner.GetSnapshot();
        Check(started.feet[1].justStarted && started.feet[0].phase == FootstepPhase::Idle &&
                started.startCount == 1, "Only the requested foot receives a command");
        Check(Near(started.predictedCom.z, .3f) && Near(started.predictedCom.y, .4f),
            "Start prediction uses its own horizon and projected height");
        Check((started.feet[1].target.position - PxVec3(.12f, .4f, .3f)).magnitude() < 1e-5f,
            "Landing prediction keeps the Idle offset on the observed plane");
        Check(started.feet[1].phase == FootstepPhase::AwaitingTakeoff &&
                started.feet[1].motion.valid && started.landingCount == 0,
            "An initial grounded sample remains an active command");
        Check((started.feet[1].motion.position - input.feet[1].position).magnitude() < 1e-6f &&
                Near(started.feet[1].motion.initialHeight, .045f),
            "Trajectory starts at the actual sole with distance-scaled lift");

        input.feet[1].grounded = false;
        input.feet[1].position.z = .15f;
        Advance(planner, input, settings);
        Advance(planner, input, settings, 9);
        const FootstepCommand halfway = planner.GetSnapshot().feet[1];
        Check(halfway.phase == FootstepPhase::Airborne && Near(halfway.motion.progress, .5f) &&
                Near(halfway.motion.measuredProgress, .5f), "Time and measured progress are independent");
        Check(Near(halfway.motion.position.z, .15f) && Near(halfway.motion.position.y, .445f),
            "The smooth horizontal trajectory reaches its maximum lift halfway");
        input.velocity.z = 1.5f;
        Advance(planner, input, settings);
        Check(Near(planner.GetSnapshot().feet[1].target.position.z, .45f),
            "An active foot retargets before the landing lock");
        Advance(planner, input, settings, 4);
        const PxVec3 locked = planner.GetSnapshot().feet[1].target.position;
        Check(planner.GetSnapshot().feet[1].motion.landingTargetLocked,
            "Landing locks according to that foot's own clock");
        input.velocity.z = 2;
        Advance(planner, input, settings, 4);
        Check(Near(planner.GetSnapshot().feet[1].target.position.z, locked.z) &&
                planner.GetSnapshot().feet[1].IsInProgress(),
            "Time expiry preserves the locked goal and cannot fabricate a landing");
        Advance(planner, input, settings, 100);
        Check(Near(planner.GetSnapshot().feet[1].motion.landingLowering, settings.maximumLandingLowering) &&
                planner.GetSnapshot().feet[1].motion.valid, "Extra lowering stays bounded while awaiting contact");
        input.feet[1].position = locked + PxVec3(.02f, 0, 0);
        input.feet[1].grounded = true;
        input.feet[0].grounded = false;
        Advance(planner, input, settings);
        const FootstepCommand landed = planner.GetSnapshot().feet[1];
        Check(landed.justLanded && landed.phase == FootstepPhase::Landed &&
                planner.GetSnapshot().landingCount == 1 && Near(landed.motion.landingError, .02f),
            "The foot finishes on its own contact and records actual error");
        Advance(planner, input, settings);
        Check(!planner.GetSnapshot().feet[1].justStarted && planner.GetSnapshot().startCount == 1 &&
                planner.GetSnapshot().feet[1].reason == FootstepReason::WaitingSupportLoad,
            "The next command waits when its remaining foot has no actual contact");
        input.feet[0].grounded = true;
        input.feet[0].normalImpulse = input.bodyWeight * .6f * input.fixedDeltaTime;
        Advance(planner, input, settings, 2);
        Check(planner.GetSnapshot().startCount == 1, "Two load samples cannot satisfy the three-sample confirmation");
        Advance(planner, input, settings);
        Check(planner.GetSnapshot().feet[1].justStarted && planner.GetSnapshot().startCount == 2,
            "The same foot remains selectable after its other contact confirms the required load");
    }

    void TestImpactSelection()
    {
        const FootstepSettings settings = TestSettings();
        for (unsigned direction = 0; direction < 2; ++direction)
        {
            for (unsigned angle = 0; angle < 12; ++angle)
            {
                FootstepPlanner planner;
                FootstepInput input = Standing();
                input.requestedFoot = -1;
                input.hasImpact = true;
                const int selected = static_cast<int>(direction);
                input.com.x = direction == 0 ? .1f : -.1f;
                input.impactPosition = PxVec3(direction == 0 ? -.2f : .2f, 8, 0);
                const PxQuat rotation(PxTwoPi * static_cast<float>(angle) / 12, PxVec3(0, 1, 0));
                const PxVec3 translation(3, 0, -7);
                input.com = rotation.rotate(input.com) + translation;
                input.velocity = rotation.rotate(input.velocity);
                input.supportRight = rotation.rotate(input.supportRight);
                input.impactPosition = rotation.rotate(input.impactPosition) + translation;
                for (PxVec3& point : input.hull)
                {
                    point = rotation.rotate(point) + translation;
                }
                for (FootstepFootInput& foot : input.feet)
                {
                    foot.position = rotation.rotate(foot.position) + translation;
                    foot.contactPosition = rotation.rotate(foot.contactPosition) + translation;
                    foot.stanceOffset = rotation.rotate(foot.stanceOffset);
                }
                planner.Update(input, settings);
                Check(planner.GetSnapshot().requestedFoot == selected &&
                        planner.GetSnapshot().feet[selected].justStarted &&
                        !planner.GetSnapshot().feet[1 - selected].IsInProgress(),
                    "Impact-nearest selection is independent of COM side, heading and world translation");
            }
        }

        FootstepPlanner planner;
        FootstepInput input = Standing();
        input.requestedFoot = -1;
        input.hasImpact = true;
        input.impactPosition = PxVec3(-.2f, 1, 0);
        planner.Update(input, settings);
        input.hasImpact = false;
        input.feet[0].grounded = false;
        input.feet[0].position.z = 1;
        Advance(planner, input, settings);
        Check(planner.GetSnapshot().requestedFoot == 0 && planner.GetSnapshot().startCount == 1,
            "Motion alone cannot reselect the other foot from a stale impact");
        input.hasImpact = true;
        input.impactPosition = input.feet[1].position + PxVec3(0, 1, 0);
        Advance(planner, input, settings);
        Check(planner.GetSnapshot().feet[0].IsInProgress() && !planner.GetSnapshot().feet[1].IsInProgress() &&
                planner.GetSnapshot().requestedFoot == 1 && planner.GetSnapshot().startCount == 1,
            "A new impact changes the requested foot while the unfinished step still blocks a new command");

        planner.Reset();
        input = Standing();
        input.requestedFoot = -1;
        planner.Update(input, settings);
        Check(planner.GetSnapshot().IsInProgress(), "An unobserved initial disturbance uses landing error as a fallback");
    }

    void TestIndependentCommands()
    {
        const FootstepSettings settings = TestSettings();
        FootstepPlanner planner;
        FootstepInput input = Standing();
        input.requestedFoot = 0;
        planner.Update(input, settings);
        input.feet[0].grounded = false;
        Advance(planner, input, settings, 4);
        input.requestedFoot = 1;
        input.velocity.z = 1.5f;
        Advance(planner, input, settings);
        Check(planner.GetSnapshot().feet[0].IsInProgress() && !planner.GetSnapshot().feet[1].IsInProgress() &&
                Near(planner.GetSnapshot().feet[0].motion.elapsedTime, 4 * input.fixedDeltaTime) &&
                Near(planner.GetSnapshot().feet[1].motion.elapsedTime, 0),
            "The requested next foot waits while the existing command keeps its own flight clock");
        const uint64_t firstStarted = planner.GetSnapshot().feet[0].startedStep;
        input.feet[1].grounded = false;
        input.hull.clear();
        input.zone = FootSupportZone::Invalid;
        Advance(planner, input, settings);
        Check(planner.GetSnapshot().feet[0].motion.valid && !planner.GetSnapshot().feet[1].motion.valid &&
                planner.GetSnapshot().hasGroundPlane && !planner.GetSnapshot().needsStep,
            "The current command continues against the cached plane while a new command remains blocked");
        input.feet[0].hasContactSample = false;
        Advance(planner, input, settings);
        Check(!planner.GetSnapshot().feet[0].motion.valid && !planner.GetSnapshot().feet[1].motion.valid &&
                planner.GetSnapshot().feet[0].startedStep == firstStarted,
            "A missing sample holds the existing command without restarting it or releasing the next one");
        input.feet[0].hasContactSample = true;
        Advance(planner, input, settings, 20);
        input.feet[0].grounded = true;
        input.feet[1].grounded = true;
        Advance(planner, input, settings);
        Check(planner.GetSnapshot().feet[0].justLanded && !planner.GetSnapshot().feet[1].IsInProgress(),
            "Landing alone cannot release the next requested command");
        input.hull = Standing().hull;
        input.zone = FootSupportZone::Inside;
        input.feet[0].normalImpulse = input.bodyWeight * .6f * input.fixedDeltaTime;
        Advance(planner, input, settings, 3);
        Check(planner.GetSnapshot().feet[1].justStarted && planner.GetSnapshot().feet[0].phase == FootstepPhase::Landed &&
                Near(planner.GetSnapshot().feet[1].motion.elapsedTime, 0),
            "Confirmed actual load releases the requested foot without overwriting the preceding foot's history");
        Check(Near(planner.GetSnapshot().GetMaximumInitialDistance(),
                planner.GetSnapshot().feet[1].initialDistance), "Standing lowering measures the new command only");

        for (unsigned missing = 0; missing < 3; ++missing)
        {
            planner.Reset();
            input = Standing();
            if (missing == 0)
            {
                input.feet[0].grounded = false;
            }
            else if (missing == 1)
            {
                input.feet[0].hasContactSample = false;
            }
            else
            {
                input.feet[0].available = false;
            }
            planner.Update(input, settings);
            Check(planner.GetSnapshot().feet[1].justStarted && planner.GetSnapshot().feet[1].motion.valid,
                "Neither opposite-foot contact, availability nor a body-weight ratio gates a command");
        }
    }

    void TestGeometry()
    {
        FootstepSettings settings = TestSettings();
        for (unsigned slope = 0; slope < 3; ++slope)
        {
            FootstepPlanner planner;
            FootstepInput input = Standing();
            input.velocity = PxVec3(0, 0, 8);
            const PxVec3 normal = PxVec3(.1f * static_cast<float>(slope), 1, .1f).getNormalized();
            for (FootstepFootInput& foot : input.feet)
            {
                foot.normal = normal;
                foot.contactPosition.y = .4f - (normal.x * foot.position.x) / normal.y;
                foot.position.y = foot.contactPosition.y;
            }
            planner.Update(input, settings);
            const FootstepSnapshot state = planner.GetSnapshot();
            const FootstepCommand& command = state.feet[1];
            Check(command.target.valid && command.target.lengthLimited &&
                    (command.target.position - command.startPosition).magnitude() <= settings.maximumStepLength + 1e-5f,
                "Maximum step length is measured from this command's own departure point");
            Check(std::abs(normal.dot(command.target.position - state.groundPosition)) < 1e-5f,
                "The candidate lies on the actual observed slope");
            input.feet[1].position += PxVec3(0, .1f, .4f);
            input.feet[1].grounded = false;
            Advance(planner, input, settings);
            Check((planner.GetSnapshot().feet[1].target.position - command.startPosition).magnitude() <=
                    settings.maximumStepLength + 1e-5f, "Retargeting keeps the original departure-point bound");
        }

        FootstepPlanner planner;
        FootstepInput input = Standing();
        input.velocity = PxVec3(-2, 0, 1);
        input.feet[0].grounded = false;
        planner.Update(input, settings);
        const FootstepTarget narrowed = planner.GetSnapshot().feet[1].target;
        Check(narrowed.valid && narrowed.stanceWidthLimited &&
                Near(narrowed.position.x - input.feet[0].position.x, .24f * settings.minimumStanceWidthRatio),
            "Width limiting uses the other foot as geometry without requiring its contact");
        planner.Reset();
        settings.preventCrossing = true;
        settings.crossingOffset = .3f;
        planner.Update(input, settings);
        Check(planner.GetSnapshot().feet[1].target.crossingLimited &&
                Near(planner.GetSnapshot().feet[1].target.position.x - input.feet[0].position.x, .3f),
            "Optional crossing prevention remains a geometric target constraint");
    }

    void TestGatesAndInvalidData()
    {
        const FootstepSettings settings = TestSettings();
        for (unsigned gate = 0; gate < 12; ++gate)
        {
            FootstepPlanner planner;
            FootstepInput input = Standing();
            switch (gate)
            {
            case 0:
                input.active = false;
                break;
            case 1:
                input.automaticSupport = false;
                break;
            case 2:
                input.idleAnimation = false;
                break;
            case 3:
                input.hasCom = false;
                break;
            case 4:
                input.hasVelocity = false;
                break;
            case 5:
                input.hasReference = false;
                break;
            case 6:
                input.feet[1].calfLength = 0;
                break;
            case 7:
                input.feet[1].hasContactSample = false;
                break;
            case 8:
                input.feet[0].grounded = false;
                input.feet[1].grounded = false;
                break;
            case 9:
                input.hull.clear();
                break;
            case 10:
                input.com.x = std::numeric_limits<float>::quiet_NaN();
                break;
            case 11:
                input.feet[1].position.z = std::numeric_limits<float>::infinity();
                break;
            }
            planner.Update(input, settings);
            Check(!planner.GetSnapshot().IsInProgress() && !planner.GetSnapshot().feet[1].motion.valid,
                "Missing required observations and invalid inputs cannot emit a motion command");
        }

        const std::array<float FootstepSettings::*, 14> fields{{
            &FootstepSettings::startPredictionTime, &FootstepSettings::landingPredictionTime,
            &FootstepSettings::stanceScale, &FootstepSettings::minimumStanceWidthRatio,
            &FootstepSettings::maximumStepLength, &FootstepSettings::crossingOffset,
            &FootstepSettings::liftHeightScale, &FootstepSettings::stepDuration,
            &FootstepSettings::minimumLiftHeight, &FootstepSettings::landingLockProgress,
            &FootstepSettings::landingLoweringSpeed, &FootstepSettings::maximumLandingLowering,
            &FootstepSettings::supportLoadRatio, &FootstepSettings::supportConfirmationTime}};
        for (float FootstepSettings::* field : fields)
        {
            FootstepPlanner planner;
            FootstepSettings invalid = settings;
            invalid.*field = std::numeric_limits<float>::quiet_NaN();
            planner.Update(Standing(), invalid);
            Check(planner.GetSnapshot().reason == FootstepReason::InvalidSettings &&
                    !planner.GetSnapshot().IsInProgress(), "Nonfinite settings are rejected before motion");
        }
    }

    void TestClockAndAbort()
    {
        FootstepPlanner planner;
        const FootstepSettings settings = TestSettings();
        FootstepInput input = Standing();
        planner.Update(input, settings);
        planner.Update(input, settings);
        Check(planner.GetSnapshot().startCount == 1 && Near(planner.GetSnapshot().feet[1].motion.progress, 0),
            "Reading the same physics step cannot advance a command twice");
        input.outsideBufferReleased = true;
        Advance(planner, input, settings);
        Check(planner.GetSnapshot().aborted && planner.GetSnapshot().abortCount == 1 &&
                !planner.GetSnapshot().IsInProgress() && !planner.GetSnapshot().feet[1].motion.valid,
            "Buffer expiry cancels both foot commands");
        input.outsideBufferReleased = false;
        Advance(planner, input, settings, 4);
        Check(planner.GetSnapshot().aborted && planner.GetSnapshot().startCount == 1,
            "An abort remains latched until reset");
        planner.Reset();
        planner.Update(input, settings);
        Check(!planner.GetSnapshot().aborted && planner.GetSnapshot().feet[1].justStarted,
            "Explicit reset starts a new independent command history");
        input.rootReleased = true;
        Advance(planner, input, settings);
        Check(planner.GetSnapshot().reason == FootstepReason::RootSupportReleased &&
                planner.GetSnapshot().aborted, "Root release cancels commands");
        input.rootReleased = false;
        ++input.revision;
        Advance(planner, input, settings);
        Check(planner.GetSnapshot().startCount == 1 && planner.GetSnapshot().feet[1].justStarted &&
                !planner.GetSnapshot().aborted, "A new contact revision clears stale commands and aborts");
        input.completedStep = 1;
        planner.Update(input, settings);
        Check(planner.GetSnapshot().startCount == 1 && Near(planner.GetSnapshot().feet[1].motion.progress, 0),
            "A rewound physics clock resets departure times");
        input.active = false;
        Advance(planner, input, settings);
        Check(!planner.GetSnapshot().hasGroundPlane && !planner.GetSnapshot().IsInProgress(),
            "Leaving active mode clears commands and cached observations");
    }

    void TestFixedStepRates()
    {
        const std::array<unsigned, 4> rates{{30, 60, 120, 240}};
        for (unsigned rate : rates)
        {
            FootstepPlanner planner;
            FootstepSettings settings = TestSettings();
            settings.stepDuration = .4f;
            FootstepInput input = Standing();
            input.fixedDeltaTime = 1.0f / static_cast<float>(rate);
            planner.Update(input, settings);
            input.feet[1].grounded = false;
            Advance(planner, input, settings);
            Advance(planner, input, settings, rate / 5);
            const FootstepMotion motion = planner.GetSnapshot().feet[1].motion;
            Check(Near(motion.progress, .5f) && Near(motion.position.z, .15f) && Near(motion.height, .045f),
                "Separate command timing is consistent across fixed physics rates");
            Advance(planner, input, settings, rate / 5);
            Check(planner.GetSnapshot().feet[1].IsInProgress() &&
                    Near(planner.GetSnapshot().feet[1].motion.progress, 1),
                "Elapsed time alone never fabricates a grounded sample");
            input.feet[1].grounded = true;
            Advance(planner, input, settings);
            Check(planner.GetSnapshot().feet[1].justLanded, "Actual contact closes an expired command at every rate");
        }
    }
    void FinishInitialStep(FootstepPlanner& planner, FootstepInput& input, const FootstepSettings& settings)
    {
        planner.Update(input, settings);
        const size_t side = static_cast<size_t>(planner.GetSnapshot().requestedFoot);
        input.feet[side].grounded = false;
        Advance(planner, input, settings);
        const unsigned flightSamples = static_cast<unsigned>(std::ceil(settings.stepDuration / input.fixedDeltaTime));
        Advance(planner, input, settings, flightSamples);
        input.feet[side].grounded = true;
        input.feet[side].position = planner.GetSnapshot().feet[side].target.position;
        Advance(planner, input, settings);
        Check(planner.GetSnapshot().feet[side].justLanded && planner.GetSnapshot().landingCount == 1,
            "Prepare a completed physical contact sequence before testing the next command");
        input.com.z += .1f;
    }

    void TestLoadTransferAtRates()
    {
        const FootstepSettings settings = TestSettings();
        const FootstepSettings defaults;
        Check(Near(defaults.supportLoadRatio, .25f) && Near(defaults.supportConfirmationTime, .01f),
            "Default next-command confirmation is one quarter body weight sustained for 0.01 seconds");
        const std::array<unsigned, 4> rates{{30, 60, 120, 240}};
        for (unsigned rate : rates)
        {
            FootstepPlanner planner;
            FootstepInput input = Standing();
            input.fixedDeltaTime = 1.0f / static_cast<float>(rate);
            for (FootstepFootInput& foot : input.feet)
            {
                foot.normalImpulse = input.bodyWeight * .6f * input.fixedDeltaTime;
            }
            FinishInitialStep(planner, input, settings);
            Check(planner.GetSnapshot().loadTransfer.required &&
                    Near(planner.GetSnapshot().loadTransfer.confirmationTimes[1], 0),
                "The landing impulse is excluded from the following load confirmation");
            input.requestedFoot = 0;
            const unsigned samples = static_cast<unsigned>(std::ceil(settings.supportConfirmationTime / input.fixedDeltaTime - 1e-5f));
            Advance(planner, input, settings, samples - 1);
            Check(planner.GetSnapshot().startCount == 1 &&
                    planner.GetSnapshot().feet[0].reason == FootstepReason::WaitingSupportLoad,
                "The next foot cannot start before the full physical confirmation interval");
            const float before = planner.GetSnapshot().loadTransfer.confirmationTimes[1];
            for (unsigned duplicate = 0; duplicate < 5; ++duplicate)
            {
                planner.Update(input, settings);
            }
            Check(Near(planner.GetSnapshot().loadTransfer.confirmationTimes[1], before),
                "Repeated reads of the same physics result cannot count as extra supported time");
            Advance(planner, input, settings);
            Check(planner.GetSnapshot().feet[0].justStarted && planner.GetSnapshot().startCount == 2 &&
                    planner.GetSnapshot().requestedFoot == 0,
                "The selected next foot starts after enough load samples at every physics rate");
        }
    }

    void TestLoadTransferInterruptions()
    {
        const FootstepSettings settings = TestSettings();
        for (unsigned interruption = 0; interruption < 7; ++interruption)
        {
            FootstepPlanner planner;
            FootstepInput input = Standing();
            FinishInitialStep(planner, input, settings);
            input.requestedFoot = 0;
            input.feet[1].normalImpulse = input.bodyWeight * .6f * input.fixedDeltaTime;
            Advance(planner, input, settings, 2);
            Check(planner.GetSnapshot().startCount == 1 &&
                    planner.GetSnapshot().loadTransfer.confirmationTimes[1] > 0,
                "Accumulate a partial confirmation before interrupting the measured load");
            switch (interruption)
            {
            case 0:
                input.feet[1].grounded = false;
                break;
            case 1:
                input.feet[1].hasContactSample = false;
                break;
            case 2:
                input.feet[1].normalImpulse = input.bodyWeight * .49f * input.fixedDeltaTime;
                break;
            case 3:
                input.feet[1].normalImpulse = std::numeric_limits<float>::quiet_NaN();
                break;
            case 4:
                input.bodyWeight = 0;
                break;
            case 5:
                input.feet[1].normal = PxVec3(0);
                break;
            case 6:
                input.completedStep += 5;
                break;
            }
            Advance(planner, input, settings);
            Check(planner.GetSnapshot().startCount == 1 &&
                    planner.GetSnapshot().loadTransfer.confirmationTimes[1] <= input.fixedDeltaTime + 1e-6f,
                "Lost contact, insufficient load, invalid data and unobserved clock gaps break continuity");
            input.feet[1].grounded = true;
            input.feet[1].hasContactSample = true;
            input.feet[1].normal = PxVec3(0, 1, 0);
            input.bodyWeight = 1000;
            input.feet[1].normalImpulse = input.bodyWeight * .6f * input.fixedDeltaTime;
            Advance(planner, input, settings);
            Check(planner.GetSnapshot().startCount == 1, "Old confirmation time cannot survive an interruption");
            Advance(planner, input, settings, 2);
            Check(planner.GetSnapshot().feet[0].IsInProgress() && planner.GetSnapshot().startCount == 2,
                "Fresh uninterrupted load releases the same requested foot after a reset");
        }

        FootstepPlanner planner;
        FootstepInput input = Standing();
        FinishInitialStep(planner, input, settings);
        input.requestedFoot = 0;
        input.feet[1].normalImpulse = input.bodyWeight * 2 * input.fixedDeltaTime;
        Advance(planner, input, settings);
        input.feet[1].normalImpulse = input.bodyWeight * .4f * input.fixedDeltaTime;
        Advance(planner, input, settings, 6);
        Check(planner.GetSnapshot().startCount == 1 && Near(planner.GetSnapshot().loadTransfer.confirmationTimes[1], 0),
            "A large one-frame contact spike cannot substitute for sustained supported weight");
        input.feet[1].normalImpulse = input.bodyWeight * .5f * input.fixedDeltaTime;
        Advance(planner, input, settings, 3);
        Check(planner.GetSnapshot().feet[0].justStarted, "Exactly half the body weight satisfies the inclusive threshold");

        planner.Reset();
        input = Standing();
        FinishInitialStep(planner, input, settings);
        input.requestedFoot = 0;
        input.feet[1].normal = PxVec3(.6f, .8f, 0);
        input.feet[1].normalImpulse = input.bodyWeight * .6f * input.fixedDeltaTime;
        Advance(planner, input, settings, 6);
        Check(planner.GetSnapshot().startCount == 1 && Near(planner.GetSnapshot().loadTransfer.loadRatios[1], .48f),
            "The weight threshold uses vertical reaction rather than the full normal force on a slope");
        input.feet[1].normalImpulse = input.bodyWeight * .7f * input.fixedDeltaTime;
        Advance(planner, input, settings, 3);
        Check(planner.GetSnapshot().feet[0].justStarted, "Sufficient vertical reaction on a slope releases the next command");
    }

    void TestCrossRegionGeometry()
    {
        const FootstepSettings settings;
        for (unsigned heading = 0; heading < 12; ++heading)
        {
            const PxQuat yaw(PxTwoPi * static_cast<float>(heading) / 12, PxVec3(0, 1, 0));
            const PxVec3 translation(14, 0, -11);
            FootstepInput input = Standing();
            input.supportRight = yaw.rotate(PxVec3(1, 0, 0));
            input.com = translation + yaw.rotate(PxVec3(-.19f, 1, .3f));
            input.velocity = yaw.rotate(PxVec3(-.4f, 0, .2f));
            for (FootstepFootInput& foot : input.feet)
            {
                foot.position = translation + yaw.rotate(foot.position);
                foot.contactPosition = foot.position;
                foot.stanceOffset = yaw.rotate(foot.stanceOffset);
            }
            FootstepPlanner planner;
            planner.Update(input, settings);
            const FootstepSnapshot& step = planner.GetSnapshot();
            for (size_t side = 0; side < input.feet.size(); ++side)
            {
                const FootstepCrossRegion& region = step.crossRegions[side];
                const PxVec3 outward = side == 0 ? -input.supportRight : input.supportRight;
                const PxVec3 forward = yaw.rotate(PxVec3(0, 0, 1));
                Check(region.valid && region.center == input.feet[side].position && region.forward.dot(forward) > .99999f,
                    "The support sole center anchors a line parallel to character forward at every heading");
                for (float distance : {-3.0f, 0.0f, 3.0f})
                {
                    const PxVec3 onLine = region.center + forward * distance + PxVec3(0, 1, 0);
                    Check(!region.Contains(onLine) && region.Contains(onLine + outward * .001f) &&
                        !region.Contains(onLine - outward * .001f), "Only the exterior of each support line is a crossover region");
                }
            }
            const PxVec3 expectedStart = step.comProjection + input.velocity * settings.startPredictionTime;
            const PxVec3 expectedLanding = step.comProjection + input.velocity * settings.landingPredictionTime;
            Check(step.hasPrediction && (step.predictedCom - expectedStart).magnitude() < 1e-5f &&
                (step.candidates[1].landingCom - expectedLanding).magnitude() < 1e-5f && step.crossRegions[0].Contains(step.predictedCom),
                "Both COM predictions travel freely when the current COM is already beyond the support foot");
        }

        FootstepInput input = Standing();
        FootstepPlanner planner;
        input.feet[0].grounded = false;
        planner.Update(input, settings);
        Check(!planner.GetSnapshot().crossRegions[0].valid, "An airborne foot cannot supply an active support line");
        input.active = false;
        Advance(planner, input, settings);
        Check(planner.GetSnapshot().crossRegions[0].valid && planner.GetSnapshot().GetCrossRegionSupportFoot() < 0,
            "Idle ground projections preview both possible support lines before selecting a foot");
        input.feet[0].hasGroundProjection = false;
        Advance(planner, input, settings);
        Check(!planner.GetSnapshot().crossRegions[0].valid, "Idle preview requires contact or a ground projection for that foot");
        input.supportRight = PxVec3(0);
        Advance(planner, input, settings);
        Check(!planner.GetSnapshot().crossRegions[1].valid, "A missing horizontal character heading cannot create a region");
    }

    void TestTakeoffConfirmation()
    {
        FootstepPlanner planner;
        const FootstepSettings settings = TestSettings();
        FootstepInput input = Standing();
        planner.Update(input, settings);
        Advance(planner, input, settings, 60);
        const FootstepCommand waiting = planner.GetSnapshot().feet[1];
        Check(waiting.phase == FootstepPhase::AwaitingTakeoff && !waiting.hasTakenOff &&
                planner.GetSnapshot().landingCount == 0 && planner.GetSnapshot().startCount == 1,
            "A foot that never left the floor cannot complete or repeatedly restart a step");
        Check(waiting.motion.valid && Near(waiting.motion.progress, 0) &&
                !waiting.motion.landingTargetLocked && Near(waiting.motion.height, waiting.motion.initialHeight),
            "Lift stays active until actual takeoff instead of expiring on the nominal deadline");
        Check(Near(waiting.motion.position.x, waiting.startPosition.x) &&
                Near(waiting.motion.position.z, waiting.startPosition.z),
            "The takeoff command lifts vertically without dragging the contacting foot horizontally");
        input.feet[1].grounded = false;
        Advance(planner, input, settings);
        const FootstepCommand airborne = planner.GetSnapshot().feet[1];
        Check(airborne.hasTakenOff && airborne.phase == FootstepPhase::Airborne &&
                Near(airborne.motion.elapsedTime, 0) && Near(airborne.motion.progress, 0),
            "A late physical takeoff starts its own flight clock at zero");
        Check((airborne.motion.position - waiting.motion.position).magnitude() < 1e-6f,
            "Takeoff carries the held lift goal into flight without a positional jump");
        Advance(planner, input, settings, 9);
        Check(Near(planner.GetSnapshot().feet[1].motion.progress, .5f) &&
                planner.GetSnapshot().feet[1].motion.position.y <= waiting.startPosition.y + waiting.motion.initialHeight + 1e-5f,
            "Flight progresses after takeoff and preserves the total lift-height bound");
        Advance(planner, input, settings, 20);
        Check(planner.GetSnapshot().feet[1].IsInProgress() && planner.GetSnapshot().landingCount == 0,
            "An expired airborne command still requires a real return contact");
        input.feet[1].grounded = true;
        Advance(planner, input, settings);
        Check(planner.GetSnapshot().feet[1].justLanded && planner.GetSnapshot().landingCount == 1,
            "Only a taken-off foot with a subsequent real contact can finish");
    }
}

void TestFootstepRecovery();

void TestFootstepPlanner()
{
    TestFootstepRecovery();
    checks = 0;
    TestWalkingMotionSearch();
    TestPredictionAndTrajectory();
    TestImpactSelection();
    TestIndependentCommands();
    TestGeometry();
    TestGatesAndInvalidData();
    TestClockAndAbort();
    TestFixedStepRates();
    TestTakeoffConfirmation();
    TestLoadTransferAtRates();
    TestLoadTransferInterruptions();
    TestCrossRegionGeometry();
    std::printf("PASS: %u independent foot-command checks\n", checks);
}
