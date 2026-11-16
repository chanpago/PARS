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
        // 기존 간격 보정의 회귀 검사는 새 COM 제한과 분리한다.
        settings.minimumStanceWidthRatio = .7f;
        settings.landingComClampEnabled = false;
        // 연속 하중 판정 검사는 기본 튜닝과 분리해 하중 기준과 확인 시간을 명시한다.
        settings.supportLoadRatio = .5f;
        settings.supportConfirmationTime = .05f;
        return settings;
    }

    std::array<PxVec3, 4> SoleRectangle(const PxVec3& center, float halfWidth, float halfLength, float yaw = 0)
    {
        const std::array<PxVec3, 4> local{{PxVec3(-halfWidth, 0, -halfLength), PxVec3(halfWidth, 0, -halfLength),
            PxVec3(halfWidth, 0, halfLength), PxVec3(-halfWidth, 0, halfLength)}};
        const PxQuat rotation(yaw, PxVec3(0, 1, 0));
        std::array<PxVec3, 4> result;
        for (size_t corner = 0; corner < result.size(); ++corner)
        {
            result[corner] = center + rotation.rotate(local[corner]);
        }
        return result;
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
            foot.corners = SoleRectangle(foot.position, .04f, .1f);
            foot.hasSoleRectangle = true;
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

    void TestLandingComGeometry()
    {
        const FootstepSettings settings;
        const std::array<PxVec3, 9> displacements{{PxVec3(0), PxVec3(0, 0, .4f),
            PxVec3(-.2f, 0, -.4f), PxVec3(.4f, 0, 0), PxVec3(-.4f, 0, -.2f),
            PxVec3(-.4f, 0, -.1f), PxVec3(-.4f, 0, 0), PxVec3(-.8f, 0, -.2f), PxVec3(-.1f, 0, -.025f)}};
        const std::array<PxVec3, 9> expected{{PxVec3(0), PxVec3(0, 0, .4f),
            PxVec3(-.2f, 0, -.4f), PxVec3(.4f, 0, 0), PxVec3(-.4f, 0, -.2f),
            PxVec3(-.36f, 0, -.18f), PxVec3(-.32f, 0, .16f), PxVec3(-.72f, 0, -.36f), PxVec3(-.09f, 0, -.045f)}};
        Check(settings.landingComClampEnabled && Near(settings.minimumStanceWidthRatio, 0) && !settings.preventCrossing,
            "Default placement uses supporting-foot geometry with independent width and crossing constraints disabled");
        for (unsigned heading = 0; heading < 12; ++heading)
        {
            const PxQuat rotation(PxTwoPi * static_cast<float>(heading) / 12, PxVec3(0, 1, 0));
            const PxVec3 translation(2, 0, -3);
            for (unsigned mirror = 0; mirror < 2; ++mirror)
            {
                const float sign = mirror == 0 ? 1.0f : -1.0f;
                const size_t selected = mirror == 0 ? 1 : 0;
                for (size_t sample = 0; sample < displacements.size(); ++sample)
                {
                    FootstepPlanner planner;
                    FootstepInput input = Standing();
                    input.requestedFoot = -1;
                    input.hasImpact = true;
                    for (size_t side = 0; side < input.feet.size(); ++side)
                    {
                        FootstepFootInput& foot = input.feet[side];
                        const float x = side == 0 ? -.3f : .3f;
                        foot.position = PxVec3(x, .4f, 0);
                        foot.contactPosition = foot.position;
                        foot.stanceOffset = PxVec3(x, 0, 0);
                        foot.corners = SoleRectangle(foot.position, .1f, .1f);
                        for (PxVec3& corner : foot.corners)
                        {
                            corner = rotation.rotate(corner) + translation;
                        }
                        foot.position = rotation.rotate(foot.position) + translation;
                        foot.contactPosition = rotation.rotate(foot.contactPosition) + translation;
                        foot.stanceOffset = rotation.rotate(foot.stanceOffset);
                    }
                    input.impactPosition = input.feet[selected].position;
                    input.com = rotation.rotate(input.com) + translation;
                    const PxVec3 displacement(displacements[sample].x * sign, 0, displacements[sample].z);
                    const PxVec3 corrected(expected[sample].x * sign, 0, expected[sample].z);
                    input.velocity = rotation.rotate(displacement / settings.landingPredictionTime);
                    input.velocity.y = 7;
                    input.supportRight = rotation.rotate(input.supportRight);
                    for (PxVec3& point : input.hull)
                    {
                        point = rotation.rotate(point) + translation;
                    }
                    const PxVec3 originalCom = input.com;
                    const PxVec3 originalVelocity = input.velocity;
                    planner.Update(input, settings);
                    const FootstepSnapshot& state = planner.GetSnapshot();
                    const FootstepTarget& target = state.candidates[selected];
                    const FootstepLandingPrediction& landing = target.landingCom;
                    const PxVec3 origin(input.com.x, .4f, input.com.z);
                    Check(target.valid && landing.valid && landing.region.valid &&
                            (landing.raw - origin - rotation.rotate(displacement)).magnitude() < 2e-5f &&
                            (landing.clamped - origin - rotation.rotate(corrected)).magnitude() < 2e-5f,
                        "Known foot-corner ray projections remain symmetric under heading, reflection and translation");
                    Check(landing.limited == (sample >= 5) && landing.region.supportFoot == static_cast<int>(1 - selected) &&
                            (target.raw - landing.clamped - input.feet[selected].stanceOffset).magnitude() < 2e-5f,
                        "Only the opposite supporting rectangle restricts COM before the moving foot offset is added");
                    Check(!IsInsideFootstepLandingRegion(landing.region, landing.clamped),
                        "Corrected COM lies outside the open forbidden sector including projections before and beyond the foot");
                    Check(sample != 0 || landing.direction == FootstepLandingDirection::None,
                        "Zero displacement remains at the current COM without selecting a boundary");
                    Check((input.com - originalCom).magnitude() == 0 &&
                            (input.velocity - originalVelocity).magnitude() == 0 && state.requestedFoot == static_cast<int>(selected),
                        "Ray correction preserves measured physical state and impact-nearest foot selection");
                    if (sample >= 5)
                    {
                        Check(!state.candidates[1 - selected].landingCom.limited,
                            "The other moving-foot candidate does not inherit the wrong supporting rectangle");
                    }
                }
            }
        }

        const std::array<float, 3> lengths{{.06f, .1f, .18f}};
        for (float halfLength : lengths)
        {
            for (unsigned direction = 0; direction < 72; ++direction)
            {
                FootstepPlanner planner;
                FootstepInput input = Standing();
                const float angle = PxTwoPi * static_cast<float>(direction) / 72;
                input.velocity = PxVec3(std::sin(angle), 0, std::cos(angle));
                input.feet[0].corners = SoleRectangle(input.feet[0].position, .04f, halfLength, .31f);
                planner.Update(input, settings);
                const FootstepLandingPrediction& landing = planner.GetSnapshot().candidates[1].landingCom;
                Check(landing.valid && landing.region.valid && !IsInsideFootstepLandingRegion(landing.region, landing.clamped) &&
                        (landing.clamped - landing.origin).magnitude() <= (landing.raw - landing.origin).magnitude() + 1e-6f,
                    "Sampled predictions use rotated rectangles and enter the complement without increasing prediction distance");
                Check(landing.limited || (landing.raw - landing.clamped).magnitude() == 0,
                    "Allowed predictions and true corner-ray boundaries retain their original points");
            }
        }

        const PxVec3 origin(0, .4f, 0);
        const PxVec3 forward(0, 0, 1);
        const std::array<PxVec3, 4> rectangle = SoleRectangle(PxVec3(-.3f, .4f, 0), .1f, .1f);
        const FootstepLandingRegion initial = BuildFootstepLandingRegion(origin, rectangle, forward);
        const FootstepLandingRegion shifted = BuildFootstepLandingRegion(PxVec3(.2f, .4f, .08f), rectangle, forward);
        const FootstepLandingRegion longer = BuildFootstepLandingRegion(origin,
            SoleRectangle(PxVec3(-.3f, .4f, 0), .1f, .2f), forward);
        const FootstepLandingRegion rotated = BuildFootstepLandingRegion(origin,
            SoleRectangle(PxVec3(-.3f, .4f, 0), .1f, .1f, .35f), forward);
        Check(initial.valid && shifted.valid && longer.valid && rotated.valid &&
                shifted.halfAngleRadians < initial.halfAngleRadians && longer.halfAngleRadians > initial.halfAngleRadians &&
                (rotated.frontRay - initial.frontRay).magnitude() > .01f,
            "COM position, actual foot dimensions and actual sole rotation determine changing boundaries without a fixed angle");
        const std::array<FootstepLandingRegion, 4> regions{{initial, shifted, longer, rotated}};
        for (const FootstepLandingRegion& region : regions)
        {
            const std::array<PxVec3, 2> rays{{region.frontRay, region.backRay}};
            for (const PxVec3& ray : rays)
            {
                bool throughCorner = false;
                for (const PxVec3& corner : region.corners)
                {
                    const PxVec3 delta = corner - region.origin;
                    throughCorner = throughCorner || (std::abs(ray.x * delta.z - ray.z * delta.x) < 1e-5f && delta.dot(ray) > 0);
                }
                Check(throughCorner, "Every forbidden boundary is a ray through an actual supporting-foot corner");
            }
        }
    }

    void TestLandingComWorldPrecision()
    {
        const FootstepSettings settings;
        const std::array<PxVec3, 3> translations{{PxVec3(0), PxVec3(3.5f, 0, -2.5f), PxVec3(-40, 0, 64)}};
        const std::array<float, 5> distances{{.0001f, .000173f, .00137f, .02f, .4f}};
        for (const PxVec3& translation : translations)
        {
            for (float distance : distances)
            {
                FootstepPlanner planner;
                FootstepInput input = Standing();
                input.com += translation;
                input.velocity = PxVec3(-distance, 0, -distance * .17f) / settings.landingPredictionTime;
                for (FootstepFootInput& foot : input.feet)
                {
                    foot.position += translation;
                    foot.contactPosition += translation;
                    for (PxVec3& corner : foot.corners)
                    {
                        corner += translation;
                    }
                }
                planner.Update(input, settings);
                const FootstepLandingPrediction& landing = planner.GetSnapshot().candidates[1].landingCom;
                Check(landing.valid && landing.region.valid && !IsInsideFootstepLandingRegion(landing.region, landing.clamped),
                    "Sub-millimetre predictions translated in world coordinates cannot round back inside a forbidden boundary");
            }
        }
    }

    void TestLandingComDisabledAndStart()
    {
        FootstepSettings enabled;
        FootstepSettings disabled = enabled;
        disabled.landingComClampEnabled = false;
        FootstepPlanner clamped;
        FootstepPlanner original;
        FootstepInput input = Standing();
        input.velocity = PxVec3(-.6f, 0, 0);
        clamped.Update(input, enabled);
        original.Update(input, disabled);
        const FootstepSnapshot& a = clamped.GetSnapshot();
        const FootstepSnapshot& b = original.GetSnapshot();
        Check(a.needsStep && b.needsStep && a.feet[1].justStarted && b.feet[1].justStarted &&
                (a.predictedCom - b.predictedCom).magnitude() == 0 && Near(a.predictedCom.x, -.36f) &&
                IsInsideFootSupportHull(input.hull, a.candidates[1].landingCom.clamped),
            "Start risk still uses raw 0.6-second prediction when the ray-corrected landing prediction is inside support");
        const PxVec3 expected(-.12f, .4f, 0);
        Check((b.feet[1].target.position - expected).magnitude() < 1e-6f && !b.candidates[1].landingCom.limited,
            "Disabling corner-ray restriction restores original zero-width prediction plus Idle offset");
        original.Reset();
        input.supportRight.x = std::numeric_limits<float>::quiet_NaN();
        disabled.landingComClampHysteresisDegrees = std::numeric_limits<float>::quiet_NaN();
        original.Update(input, disabled);
        Check(original.GetSnapshot().feet[1].justStarted &&
                (original.GetSnapshot().feet[1].target.position - expected).magnitude() < 1e-6f,
            "Disabled restriction cannot depend on a corner-ray basis or its hysteresis setting");

        const std::array<float, 4> invalidHysteresis{{-1, 90, std::numeric_limits<float>::infinity(),
            std::numeric_limits<float>::quiet_NaN()}};
        for (float angle : invalidHysteresis)
        {
            FootstepPlanner planner;
            enabled.landingComClampHysteresisDegrees = angle;
            planner.Update(Standing(), enabled);
            Check(planner.GetSnapshot().reason == FootstepReason::InvalidSettings,
                "Boundary-choice hysteresis rejects negative, nonfinite and unbounded angles");
        }
        enabled = FootstepSettings();
        for (unsigned missing = 0; missing < 5; ++missing)
        {
            FootstepPlanner planner;
            input = Standing();
            input.velocity = PxVec3(-.6f, 0, 0);
            if (missing == 0)
            {
                input.feet[0].grounded = false;
            }
            else if (missing == 1)
            {
                input.feet[0].hasSoleRectangle = false;
            }
            else if (missing == 2)
            {
                input.feet[0].corners[0].x = std::numeric_limits<float>::quiet_NaN();
            }
            else if (missing == 3)
            {
                input.feet[0].corners.fill(input.feet[0].position);
            }
            else
            {
                input.com.x = input.feet[0].position.x;
            }
            planner.Update(input, enabled);
            const FootstepLandingPrediction& landing = planner.GetSnapshot().candidates[1].landingCom;
            Check(landing.valid && !landing.region.valid && !landing.limited &&
                    (landing.clamped - landing.raw).magnitude() == 0,
                "Missing ground, missing or degenerate geometry and COM inside the foot cannot fabricate a forbidden angle");
            Check(missing != 4 || landing.region.containsCom,
                "COM-inside-foot degeneracy remains explicitly distinguishable from a valid outside tangent sector");
        }
        clamped.Reset();
        input = Standing();
        input.supportRight.x = std::numeric_limits<float>::quiet_NaN();
        clamped.Update(input, enabled);
        Check(!clamped.GetSnapshot().IsInProgress(), "An invalid character basis cannot emit a corrected target");
    }

    void TestLandingComHistoryAndLock()
    {
        FootstepSettings settings;
        settings.stepDuration = .4f;
        FootstepPlanner planner;
        FootstepInput input = Standing();
        input.velocity = PxVec3(-1, 0, 0);
        planner.Update(input, settings);
        Check(planner.GetSnapshot().feet[1].landingComDirection == FootstepLandingDirection::Front,
            "The first exact lateral tie deterministically chooses the front boundary");
        for (unsigned jitter = 0; jitter < 8; ++jitter)
        {
            input.velocity.z = jitter % 2 == 0 ? -.01f : .01f;
            Advance(planner, input, settings);
            Check(planner.GetSnapshot().feet[1].target.landingCom.direction == FootstepLandingDirection::Front,
                "Small side-axis jitter cannot alternate the active command between front and back");
        }
        input.velocity.z = -.2f;
        Advance(planner, input, settings);
        Check(planner.GetSnapshot().feet[1].landingComDirection == FootstepLandingDirection::Back,
            "A clear rearward input outside the hysteresis band changes to the nearer back boundary");
        input.velocity.z = 0;
        Advance(planner, input, settings);
        Check(planner.GetSnapshot().feet[1].target.landingCom.direction == FootstepLandingDirection::Back &&
                planner.GetSnapshot().candidates[0].landingCom.direction == FootstepLandingDirection::None,
            "Direction history belongs to the moving foot command and cannot leak into an idle foot candidate");
        input.velocity = PxVec3(0);
        Advance(planner, input, settings);
        Check(planner.GetSnapshot().feet[1].target.landingCom.direction == FootstepLandingDirection::None &&
                planner.GetSnapshot().feet[1].landingComDirection == FootstepLandingDirection::Back,
            "Zero displacement introduces no direction while preserving the active command's prior choice");
        input.velocity = PxVec3(-1, 0, 0);
        input.feet[1].grounded = false;
        Advance(planner, input, settings);
        Advance(planner, input, settings, 17);
        Check(!planner.GetSnapshot().feet[1].motion.landingTargetLocked,
            "The corrected landing target stays editable before 75 percent flight progress");
        const FootstepTarget lastEditable = planner.GetSnapshot().feet[1].target;
        input.velocity = PxVec3(1, 0, .4f);
        input.com.x = .1f;
        Advance(planner, input, settings);
        Check(planner.GetSnapshot().feet[1].motion.landingTargetLocked &&
                (planner.GetSnapshot().feet[1].target.position - lastEditable.position).magnitude() == 0,
            "Crossing the 75 percent threshold locks the previously accepted horizontal landing point");
        input.com.z = .2f;
        input.supportRight = PxVec3(0, 0, -1);
        Advance(planner, input, settings, 7);
        const FootstepCommand locked = planner.GetSnapshot().feet[1];
        Check(Near(locked.target.position.x, lastEditable.position.x) &&
                Near(locked.target.position.z, lastEditable.position.z) &&
                (planner.GetSnapshot().candidates[1].landingCom.origin - lastEditable.landingCom.origin).magnitude() > .1f,
            "Latest COM and character heading update debug candidates without moving the locked landing point");
        input.feet[1].grounded = true;
        Advance(planner, input, settings);
        Check(planner.GetSnapshot().feet[1].justLanded &&
                planner.GetSnapshot().feet[1].landingComDirection == FootstepLandingDirection::None,
            "Actual landing clears direction history at command completion");
        input.supportRight = PxVec3(1, 0, 0);
        input.com = PxVec3(0, 1, 0);
        input.velocity = PxVec3(-1, 0, 0);
        Advance(planner, input, settings);
        Check(planner.GetSnapshot().feet[1].justStarted &&
                planner.GetSnapshot().feet[1].landingComDirection == FootstepLandingDirection::Front,
            "A new command after landing starts a fresh front-back choice instead of inheriting the previous back choice");

        for (unsigned interruption = 0; interruption < 6; ++interruption)
        {
            planner.Reset();
            input = Standing();
            input.velocity = PxVec3(-1, 0, -.2f);
            planner.Update(input, settings);
            Check(planner.GetSnapshot().feet[1].landingComDirection == FootstepLandingDirection::Back,
                "Prepare a rearward command before testing history cleanup");
            input.velocity.z = 0;
            if (interruption == 0)
            {
                input.outsideBufferReleased = true;
            }
            else if (interruption == 1)
            {
                ++input.revision;
            }
            else if (interruption == 2)
            {
                input.active = false;
            }
            else if (interruption == 3)
            {
                settings.landingComClampEnabled = false;
            }
            else if (interruption == 4)
            {
                planner.Reset();
            }
            else
            {
                input.completedStep = 0;
            }
            Advance(planner, input, settings);
            Check(planner.GetSnapshot().feet[1].landingComDirection ==
                    (interruption == 1 || interruption >= 4 ? FootstepLandingDirection::Front : FootstepLandingDirection::None),
                "Abort, contact revision, inactive mode, disabling, reset and clock rewind cannot retain an old direction choice");
            settings.landingComClampEnabled = true;
        }
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

void TestFootstepPlanner()
{
    checks = 0;
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
    TestLandingComGeometry();
    TestLandingComWorldPrecision();
    TestLandingComDisabledAndStart();
    TestLandingComHistoryAndLock();
    std::printf("PASS: %u independent foot-command checks\n", checks);
}
