#include "../Model/FootstepPlanner.h"
#include <foundation/PxQuat.h>
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
        // 기존 간격 보정의 회귀 검사는 새 COM 제한과 분리한다.
        settings.minimumStanceWidthRatio = .7f;
        settings.landingComClampEnabled = false;
        // 연속 하중 판정 검사는 기본 튜닝과 분리해 하중 기준과 확인 시간을 명시한다.
        settings.supportLoadRatio = .5f;
        settings.supportConfirmationTime = .05f;
        return settings;
    }

    FootstepSettings ComTestSettings()
    {
        FootstepSettings settings;
        // COM 보정 자체의 검사는 기본 발 간격·교차 억제 튜닝과 분리한다.
        settings.minimumStanceWidthRatio = 0;
        settings.preventCrossing = false;
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
        input.bufferHull = EvaluateFootSupportBalance(input.hull, input.com).bufferHull;
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
        Advance(planner, input, settings, 7);
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

        const std::array<float FootstepSettings::*, 15> fields{{
            &FootstepSettings::startPredictionTime, &FootstepSettings::landingPredictionTime,
            &FootstepSettings::stanceScale, &FootstepSettings::minimumStanceWidthRatio,
            &FootstepSettings::maximumStepLength, &FootstepSettings::crossingOffset,
            &FootstepSettings::liftHeightScale, &FootstepSettings::backwardLiftTimeRatio, &FootstepSettings::stepDuration,
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

    void TestActualComBufferTimeout()
    {
        FootstepPlanner planner;
        const FootstepSettings settings = TestSettings();
        FootstepInput input = Standing();
        input.velocity.z = 2.0f;
        planner.Update(input, settings);
        const FootstepSnapshot& state = planner.GetSnapshot();
        Advance(planner, input, settings, 20);
        Check(!state.actualComOutsideBuffer && state.outsideBufferTime == 0.0 && !state.aborted,
            "A prediction outside B does not start the timer while the actual COM is inside");
        input.com.z = .45f;
        input.velocity.z = -.9f;
        input.zone = EvaluateFootSupportBalance(input.hull, input.com).zone;
        Advance(planner, input, settings);
        Check(state.actualComOutsideBuffer && state.outsideBufferTime == 0.0 && !state.aborted,
            "Actual COM outside B starts the timer even when the prediction is inside");
        Advance(planner, input, settings, 17);
        const double beforeReentry = state.outsideBufferTime;
        planner.Update(input, settings);
        Check(!state.aborted && state.IsInProgress() && std::abs(beforeReentry - 17.0 / 60.0) < 1e-6 &&
                state.outsideBufferTime == beforeReentry, "Seventeen elapsed ticks and repeated samples keep stepping");
        input.com.z = .3f;
        input.zone = EvaluateFootSupportBalance(input.hull, input.com).zone;
        Advance(planner, input, settings);
        Check(!state.actualComOutsideBuffer && state.outsideBufferTime == 0.0 && !state.aborted,
            "Returning to the B boundary resets the actual COM timer");
        input.com.z = .45f;
        input.zone = EvaluateFootSupportBalance(input.hull, input.com).zone;
        Advance(planner, input, settings, 5);
        input.hasCom = false;
        Advance(planner, input, settings);
        Check(!state.actualComOutsideBuffer && state.outsideBufferTime == 0.0, "A missing actual COM breaks the outside interval");
        input.hasCom = true;
        input.hasVelocity = false;
        Advance(planner, input, settings, 5);
        input.completedStep += 2;
        planner.Update(input, settings);
        Check(state.actualComOutsideBuffer && state.outsideBufferTime == 0.0, "A missing physics tick restarts the outside interval");
        Advance(planner, input, settings, 17);
        Check(!state.aborted, "A new interval under 0.30 seconds keeps stepping");
        Advance(planner, input, settings);
        Check(state.aborted && state.reason == FootstepReason::OutsideBuffer && !state.IsInProgress() &&
                std::abs(state.outsideBufferTime - 0.3) < 1e-8, "Eighteen elapsed ticks at 60 Hz abort at 0.30 seconds");
        input.com.z = 0.0f;
        input.zone = FootSupportZone::Inside;
        Advance(planner, input, settings);
        Check(state.aborted && state.abortCount == 1, "Re-entry after timeout keeps the fall latched");
    }

    void TestBackwardFallSteps()
    {
        FootstepPlanner planner;
        const FootstepSettings settings = TestSettings();
        FootstepInput input = Standing();
        input.fallingBackward = true;
        input.com.z = -.45f;
        input.velocity.z = -.5f;
        input.zone = EvaluateFootSupportBalance(input.hull, input.com).zone;
        planner.Update(input, settings);
        const FootstepSnapshot& state = planner.GetSnapshot();
        input.feet[1].grounded = false;
        Advance(planner, input, settings, 18);
        Check(state.backwardFallStepsRemaining == 2 && state.feet[1].IsInProgress() && state.startCount == 1,
            "A backward fall keeps the airborne first step at the normal fall threshold");
        input.rootReleased = true;
        input.idleAnimation = false;
        input.fallingBackward = false;
        Advance(planner, input, settings);
        Check(state.feet[1].motion.valid && state.landingCount == 0 && !state.aborted,
            "Releasing the pelvis and switching to a fall clip preserve the swing");

        input.feet[1].grounded = true;
        Advance(planner, input, settings);
        planner.Update(input, settings);
        Check(state.backwardFallStepsRemaining == 1 && state.landingCount == 1 && state.startCount == 1,
            "The first physical landing counts once and does not immediately lift the other foot");
        input.com.z = -.05f;
        input.zone = FootSupportZone::Inside;
        input.velocity.z = 0;
        input.feet[1].normalImpulse = 0;
        Advance(planner, input, settings, 4);
        Check(state.startCount == 1 && state.backwardFallStepsRemaining == 1,
            "The opposite leg waits for the foot that landed to take the load");
        input.feet[1].normalImpulse = input.bodyWeight * input.fixedDeltaTime;
        Advance(planner, input, settings, 4);
        Check(state.startCount == 2 && state.feet[0].IsInProgress() && state.feet[0].motion.valid,
            "The other leg can step during the fall even after the COM re-enters the hull");
        input.feet[0].grounded = false;
        Advance(planner, input, settings);
        input.feet[0].grounded = true;
        Advance(planner, input, settings);
        Check(state.feet[0].justLanded && state.landingCount == 2 && state.backwardFallStepsRemaining == 0 &&
                state.aborted && !state.IsInProgress(), "The second real landing ends backward fall stepping");
        Advance(planner, input, settings, 8);
        Check(state.startCount == 2 && state.abortCount == 1, "A latched fall cannot start a third step");

        planner.Reset();
        input = Standing();
        input.requestedFoot = 1;
        input.feet[1].grounded = false;
        planner.Update(input, settings);
        input.feet[1].grounded = true;
        Advance(planner, input, settings);
        Check(state.followupFoot == 0, "Ordinary recovery queues the foot opposite its last landing");
        input.zone = FootSupportZone::Invalid;
        input.fallingBackward = true;
        input.feet[1].grounded = false;
        input.feet[0].timeToReachLimit = .25f;
        Advance(planner, input, settings);
        Check(state.feet[1].justStarted && state.feet[1].motion.hasSupportDeadline,
            "A backward fall takes over the free foot even when ordinary recovery had queued the planted foot");
        input.rootReleased = true;
        input.feet[1].grounded = true;
        input.feet[1].landed = true;
        input.feet[1].normalImpulse = .01f;
        input.feet[0].grounded = false;
        input.feet[0].position.y += .1f;
        input.velocity = PxVec3(0);
        Advance(planner, input, settings);
        Check(state.feet[0].justStarted && state.feet[0].hasTakenOff && state.feet[0].motion.valid &&
                Near(state.feet[0].motion.initialHeight, 0) && state.feet[1].justLanded && state.backwardFallStepsRemaining == 1,
            "An already airborne foot can reach straight down without another takeoff or a standing load");
    }

    void TestBackwardFallInterruptions()
    {
        FootstepPlanner planner;
        const FootstepSettings settings = TestSettings();
        FootstepInput input = Standing();
        input.zone = FootSupportZone::Invalid;
        input.hull.clear();
        input.fallingBackward = true;
        planner.Update(input, settings);
        const FootstepSnapshot& state = planner.GetSnapshot();
        Check(state.backwardFallStepsRemaining == 2, "A grounded backward fall can start when the support hull is lost");
        input.rootReleased = true;
        input.pelvisGrounded = true;
        Advance(planner, input, settings);
        Check(state.aborted && state.landingCount == 0, "Pelvis contact ends the attempt without crediting a step");

        planner.Reset();
        input.rootReleased = false;
        input.pelvisGrounded = false;
        planner.Update(input, settings);
        input.feet[0].grounded = false;
        input.feet[1].grounded = false;
        input.hasFloorProjection = true;
        Advance(planner, input, settings);
        Check(!state.aborted, "A remaining floor ray keeps the falling swing active without a complete support hull");
        input.hasFloorProjection = false;
        Advance(planner, input, settings);
        Check(state.aborted && state.backwardFallStepsRemaining == 0, "Losing the floor ends the remaining attempts");

        planner.Reset();
        input.feet[0].grounded = true;
        input.feet[1].grounded = true;
        planner.Update(input, settings);
        input.rootReleased = true;
        Advance(planner, input, settings, 38);
        Check(state.aborted && state.landingCount == 0 && !state.IsInProgress(),
            "A blocked foot times out without inventing a touchdown");
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
        input.com.z = .45f;
        input.velocity.z = -.9f;
        input.zone = EvaluateFootSupportBalance(input.hull, input.com).zone;
        input.feet[1].grounded = false;
        Advance(planner, input, settings, 12);
        Check(input.zone == FootSupportZone::Outside && planner.GetSnapshot().feet[1].phase == FootstepPhase::Airborne &&
                planner.GetSnapshot().feet[1].motion.valid && planner.GetSnapshot().abortCount == 0,
            "A brief actual COM exit keeps a recovery step active before the timeout");
        input.feet[1].grounded = true;
        Advance(planner, input, settings);
        Check(planner.GetSnapshot().feet[1].justLanded && planner.GetSnapshot().landingCount == 1,
            "Ground contact completes the same step outside B");
        input.rootReleased = true;
        Advance(planner, input, settings);
        Check(planner.GetSnapshot().aborted && planner.GetSnapshot().abortCount == 1 &&
                !planner.GetSnapshot().IsInProgress() && !planner.GetSnapshot().feet[1].motion.valid,
            "Loss of root support cancels both foot commands");
        input.rootReleased = false;
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

    void TestBackwardSwing()
    {
        FootstepSettings settings = TestSettings();
        settings.backwardLiftTimeRatio = 1.5f;
        for (const float direction : {-1.0f, 1.0f})
        {
            FootstepInput input = Standing();
            input.velocity.z = direction;
            input.feet[0].timeToReachLimit = .25f;
            FootstepPlanner planner;
            planner.Update(input, settings);
            const FootstepCommand& command = planner.GetSnapshot().feet[1];
            const float peak = direction < 0 ? .6f : .5f;
            const float top = SampleFootstepPosition(command, peak * command.motion.duration).y;
            Check(command.justStarted && Near(top, command.startPosition.y + command.motion.initialHeight) &&
                    Near(SampleFootstepPosition(command, command.motion.duration).y, command.target.position.y),
                "The backward swing shifts peak time while preserving its clearance and deadline");
            Check(Near(SampleFootstepPosition(command, command.motion.duration * .5f).z,
                    .5f * (command.startPosition.z + command.target.position.z)),
                "Vertical swing timing preserves horizontal progress in both directions");
        }
    }

    void TestSwingSoleSeparation()
    {
        FootstepSettings settings = TestSettings();
        settings.minimumStanceWidthRatio = 0;
        settings.preventCrossing = false;
        for (size_t side = 0; side < 2; ++side)
        {
            FootstepInput input = Standing();
            input.requestedFoot = static_cast<int>(side);
            input.velocity.z = -1;
            FootstepFootInput& moving = input.feet[side];
            FootstepFootInput& support = input.feet[1 - side];
            const float sign = side == 0 ? -1.0f : 1.0f;
            moving.corners = SoleRectangle(moving.position, .085f, .1f);
            support.corners = SoleRectangle(support.position, .09f, .1f);
            moving.soleVelocity.x = -sign * 2.4f;
            support.soleVelocity.x = sign * 2.4f;
            support.timeToReachLimit = MaximumFootstepDuration;

            FootstepPlanner planner;
            planner.Update(input, settings);
            const FootstepCommand& step = planner.GetSnapshot().feet[side];
            const PxVec3 landing = step.target.position;
            Check(step.justStarted && step.motion.swingOutwardOffset.isZero(), "A planted foot stays on its takeoff path");
            moving.grounded = false;
            Advance(planner, input, settings, 9);
            // 현재 24cm에서 다음 틱에 16cm로 좁아지므로, 두 반폭과 여유의 합 18cm까지 2cm를 보정한다.
            Check(Near(step.motion.progress, .5f) && Near((step.motion.position.x - moving.position.x) * sign, .02f) &&
                Near(step.motion.position.z, (moving.position.z + landing.z) * .5f) &&
                Near(step.motion.position.y, moving.position.y + step.motion.height),
                "Relative sole motion offsets only the outward axis at mid-swing on either foot");
            Check(step.target.position == landing &&
                (SampleFootstepPosition(step, GetFootstepRemainingTime(step)) - landing).magnitude() < 1e-5f,
                "The corrected swing still reaches the selected landing point");

            support.soleVelocity = moving.soleVelocity;
            Advance(planner, input, settings);
            Check(step.motion.swingOutwardOffset.isZero(), "Feet translating together retain their lateral spacing");
            support.soleVelocity.x = -moving.soleVelocity.x;
            moving.position.z += .25f;
            moving.corners = SoleRectangle(moving.position, .085f, .1f);
            Advance(planner, input, settings);
            Check(step.motion.swingOutwardOffset.isZero(), "Fore-aft separation leaves the normal swing unchanged");
            moving.soleVelocity.z = -.1f / input.fixedDeltaTime;
            Advance(planner, input, settings);
            Check(Near(step.motion.swingOutwardOffset.x * sign, .02f), "A fore-aft overlap on the next tick triggers clearance");
            moving.soleVelocity.z = 0;
            moving.position.z -= .25f;
            moving.corners = SoleRectangle(moving.position, .085f, .1f);
            support.grounded = false;
            Advance(planner, input, settings);
            Check(step.motion.swingOutwardOffset.isZero(), "An airborne opposite foot does not supply a stance boundary");
        }
    }

    void TestLoadedTouchdown()
    {
        FootstepPlanner planner;
        const FootstepSettings settings = TestSettings();
        FootstepInput input = Standing();
        input.velocity.z = -1.0f;
        input.feet[0].timeToReachLimit = MaximumFootstepDuration;
        planner.Update(input, settings);
        FootstepFootInput& foot = input.feet[1];
        const FootstepCommand& command = planner.GetSnapshot().feet[1];
        foot.grounded = false;
        Advance(planner, input, settings, 13);
        foot.grounded = true;
        foot.normalImpulse = input.bodyWeight * input.fixedDeltaTime;
        Advance(planner, input, settings);
        Check(command.IsInProgress(), "A single impact away from the landing target does not plant the foot");
        foot.normalImpulse *= .1f;
        Advance(planner, input, settings);
        Check(command.IsInProgress(), "A light brush during descent does not stop the swing");
        foot.normalImpulse = input.bodyWeight * input.fixedDeltaTime;
        Advance(planner, input, settings);
        Check(command.justLanded && command.motion.progress < 1 && command.motion.landingError > .1f,
            "Continued ground contact bearing body weight ends a low descending swing at the actual foot");
    }

    void TestFixedStepRates()
    {
        const std::array<unsigned, 4> rates{{30, 60, 120, 240}};
        for (unsigned rate : rates)
        {
            FootstepPlanner planner;
            FootstepSettings settings = TestSettings();
            settings.stepDuration = .2f;
            FootstepInput input = Standing();
            input.fixedDeltaTime = 1.0f / static_cast<float>(rate);
            planner.Update(input, settings);
            input.feet[1].grounded = false;
            Advance(planner, input, settings);
            Advance(planner, input, settings, rate / 10);
            const FootstepMotion motion = planner.GetSnapshot().feet[1].motion;
            Check(Near(motion.progress, .5f) && Near(motion.position.z, .15f) && Near(motion.height, .045f),
                "Separate command timing is consistent across fixed physics rates");
            Advance(planner, input, settings, rate / 10);
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
        const unsigned flightSamples = static_cast<unsigned>(std::ceil(.5f * settings.stepDuration / input.fixedDeltaTime));
        Advance(planner, input, settings, flightSamples);
        input.feet[side].grounded = true;
        input.feet[side].position = planner.GetSnapshot().feet[side].target.position;
        Advance(planner, input, settings);
        const FootstepCommand& landed = planner.GetSnapshot().feet[side];
        Check(landed.justLanded && landed.motion.elapsedTime < landed.motion.duration &&
                Near(GetFootstepRemainingTime(landed), 0) && planner.GetSnapshot().landingCount == 1,
            "Ground contact after takeoff completes the step before the planned swing duration expires");
        input.com.z += .1f;
    }

    void TestLoadTransferAtRates()
    {
        const FootstepSettings defaults;
        FootstepSettings settings = TestSettings();
        settings.stepDuration = defaults.stepDuration;
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
        const FootstepSettings settings = ComTestSettings();
        const std::array<PxVec3, 9> displacements{{PxVec3(0), PxVec3(0, 0, .4f),
            PxVec3(-.2f, 0, -.4f), PxVec3(.4f, 0, 0), PxVec3(-.4f, 0, -.2f),
            PxVec3(-.4f, 0, -.1f), PxVec3(-.4f, 0, 0), PxVec3(-.8f, 0, -.2f), PxVec3(-.1f, 0, -.025f)}};
        const std::array<PxVec3, 9> expected{{PxVec3(0), PxVec3(0, 0, .4f),
            PxVec3(-.2f, 0, -.4f), PxVec3(.4f, 0, 0), PxVec3(-.4f, 0, -.2f),
            PxVec3(-.36f, 0, -.18f), PxVec3(-.32f, 0, .16f), PxVec3(-.72f, 0, -.36f), PxVec3(-.09f, 0, -.045f)}};
        Check(FootstepSettings().landingComClampEnabled,
            "Supporting-foot COM restriction is enabled by default");
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
                    const FootstepComPrediction& landing = target.landingCom;
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
                    const float horizonRatio = settings.startPredictionTime / settings.landingPredictionTime;
                    Check(state.hasPrediction && state.startCom.region.supportFoot == static_cast<int>(1 - selected) &&
                            (state.predictedCom - origin - rotation.rotate(corrected) * horizonRatio).magnitude() < 3e-5f &&
                            !IsInsideFootstepLandingRegion(state.startCom.region, state.predictedCom),
                        "Cyan start prediction uses the same supporting boundary with its own prediction horizon");
                    Check(state.needsStep == !IsInsideFootSupportHull(input.hull, state.predictedCom),
                        "The actual start decision uses the same corrected COM point as the cyan marker");
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
                const FootstepComPrediction& landing = planner.GetSnapshot().candidates[1].landingCom;
                Check(landing.valid && landing.region.valid && !IsInsideFootstepLandingRegion(landing.region, landing.clamped) &&
                        (landing.clamped - landing.origin).magnitude() <= (landing.raw - landing.origin).magnitude() + 1e-6f,
                    "Sampled predictions use rotated rectangles and enter the complement without increasing prediction distance");
                Check(landing.limited || (landing.raw - landing.clamped).magnitude() == 0,
                    "Allowed predictions and true corner-ray boundaries retain their original points");
                const FootstepSnapshot& state = planner.GetSnapshot();
                Check(state.hasPrediction && !IsInsideFootstepLandingRegion(state.startCom.region, state.predictedCom),
                    "Cyan predictions avoid the supporting-foot sector for every sampled sole rotation and velocity");
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
                const FootstepComPrediction& landing = planner.GetSnapshot().candidates[1].landingCom;
                Check(landing.valid && landing.region.valid && !IsInsideFootstepLandingRegion(landing.region, landing.clamped),
                    "Sub-millimetre predictions translated in world coordinates cannot round back inside a forbidden boundary");
                const FootstepSnapshot& state = planner.GetSnapshot();
                Check(state.hasPrediction && state.startCom.region.valid &&
                        !IsInsideFootstepLandingRegion(state.startCom.region, state.predictedCom),
                    "Translated cyan predictions retain the same world-space boundary tolerance");
            }
        }
    }

    void TestLandingComDisabledAndStart()
    {
        FootstepSettings enabled = ComTestSettings();
        FootstepSettings disabled = enabled;
        disabled.landingComClampEnabled = false;
        FootstepPlanner clamped;
        FootstepPlanner original;
        FootstepInput input = Standing();
        input.velocity = PxVec3(-.5f, 0, 0);
        clamped.Update(input, enabled);
        original.Update(input, disabled);
        const FootstepSnapshot& a = clamped.GetSnapshot();
        const FootstepSnapshot& b = original.GetSnapshot();
        Check(!a.needsStep && !a.IsInProgress() && a.startCount == 0 && b.needsStep && b.feet[1].justStarted &&
                a.startCom.limited && Near(a.startCom.raw.x, -.3f) && Near(b.predictedCom.x, -.3f) &&
                IsInsideFootSupportHull(input.hull, a.predictedCom) &&
                !IsInsideFootstepLandingRegion(a.startCom.region, a.predictedCom),
            "Correcting the cyan prediction inside support prevents a step that the original prediction would start");
        Check((b.predictedCom - b.startCom.raw).magnitude() == 0 && !b.startCom.limited,
            "Disabling restriction restores the raw start prediction as well as the raw landing prediction");
        const PxVec3 expected(-.08f, .4f, 0);
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
        enabled = ComTestSettings();
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
            const FootstepComPrediction& landing = planner.GetSnapshot().candidates[1].landingCom;
            Check(landing.valid && !landing.region.valid && !landing.limited &&
                    (landing.clamped - landing.raw).magnitude() == 0,
                "Missing ground, missing or degenerate geometry and COM inside the foot cannot fabricate a forbidden angle");
            Check(missing != 4 || landing.region.containsCom,
                "COM-inside-foot degeneracy remains explicitly distinguishable from a valid outside tangent sector");
            const FootstepSnapshot& state = planner.GetSnapshot();
            Check(state.hasPrediction && !state.startCom.region.valid && !state.startCom.limited &&
                    (state.predictedCom - state.startCom.raw).magnitude() == 0,
                "Unavailable supporting geometry preserves the cyan prediction without inventing a forbidden sector");
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
        settings.stepDuration = .25f;
        FootstepPlanner planner;
        FootstepInput input = Standing();
        // 이륙 후 9틱째에 진행률이 정확히 90%가 되도록 한 걸음을 10틱으로 나눈다.
        input.fixedDeltaTime = settings.stepDuration / 10.0f;
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
            Check(planner.GetSnapshot().startCom.direction == FootstepLandingDirection::Front,
                "The cyan prediction shares the active command's boundary history under small velocity jitter");
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
        Advance(planner, input, settings, 7);
        const FootstepCommand& moving = planner.GetSnapshot().feet[1];
        const PxVec3 previousTarget = moving.target.position;
        input.com.z = .1f;
        Advance(planner, input, settings);
        Check(!moving.motion.landingTargetLocked && (moving.target.position - previousTarget).magnitude() > .001f,
            "The corrected landing target still follows the COM on the last tick before 90 percent");
        const FootstepTarget lastEditable = moving.target;
        input.velocity = PxVec3(1, 0, .4f);
        input.com.x = .1f;
        Advance(planner, input, settings);
        Check(Near(moving.motion.progress, .90f) && moving.motion.landingTargetLocked &&
                moving.target.position == lastEditable.position,
            "Reaching 90 percent locks the previously accepted horizontal landing point");
        input.com.z = .2f;
        input.supportRight = PxVec3(0, 0, -1);
        Advance(planner, input, settings, 7);
        const FootstepCommand locked = planner.GetSnapshot().feet[1];
        Check(Near(locked.target.position.x, lastEditable.position.x) &&
                Near(locked.target.position.z, lastEditable.position.z) &&
                (planner.GetSnapshot().candidates[1].landingCom.origin - lastEditable.landingCom.origin).magnitude() > .1f,
            "Latest COM and character heading update debug candidates without moving the locked landing point");
        Check(planner.GetSnapshot().startCom.origin.x == input.com.x &&
                planner.GetSnapshot().startCom.origin.z == input.com.z &&
                !IsInsideFootstepLandingRegion(planner.GetSnapshot().startCom.region, planner.GetSnapshot().predictedCom),
            "Locking the landing target does not freeze or bypass the cyan prediction correction");
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
                input.rootReleased = true;
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

    void TestSupportLandingDeadline()
    {
        FootstepSettings settings = TestSettings();
        settings.stepDuration = .15f;
        for (const unsigned rate : {60u, 120u})
        {
            FootstepPlanner planner;
            FootstepInput input = Standing();
            input.fixedDeltaTime = 1.0f / static_cast<float>(rate);
            input.feet[0].timeToReachLimit = 2.0f;
            input.feet[1].timeToReachLimit = 0;
            planner.Update(input, settings);
            const FootstepCommand& step = planner.GetSnapshot().feet[1];
            Check(step.justStarted && step.motion.hasSupportDeadline && Near(GetFootstepRemainingTime(step), .3f),
                "A long support forecast caps the whole step at 0.3 seconds");
            Advance(planner, input, settings, rate / 20);
            Check(step.phase == FootstepPhase::AwaitingTakeoff && Near(GetFootstepRemainingTime(step), .25f) &&
                step.motion.position.y > step.startPosition.y && step.motion.position.z > step.startPosition.z,
                "Lifting and moving toward the target spend the same budget before contact breaks");
            input.feet[1].grounded = false;
            Advance(planner, input, settings);
            Check(step.hasTakenOff && Near(GetFootstepRemainingTime(step), .25f - input.fixedDeltaTime),
                "Physical takeoff does not grant a fresh flight duration");
            const PxVec3 expectedNext = SampleFootstepPosition(step, input.fixedDeltaTime);
            const float shorterForecast = .15f;
            input.feet[1].grounded = true;
            input.feet[0].timeToReachLimit = shorterForecast;
            Advance(planner, input, settings);
            Check(step.IsInProgress() && !step.justLanded && Near(GetFootstepRemainingTime(step), shorterForecast),
                "A distant early contact keeps the target and an earlier support limit advances touchdown");
            Check((step.motion.position - expectedNext).magnitude() < 1e-5f,
                "A changed deadline retimes the remaining arc without jumping the current drive target");
            input.feet[1].grounded = false;
            const PxVec3 nextBeforeExtension = SampleFootstepPosition(step, input.fixedDeltaTime);
            input.feet[0].timeToReachLimit = .5f;
            Advance(planner, input, settings);
            Check(Near(step.motion.duration, MaximumFootstepDuration) &&
                (step.motion.position - nextBeforeExtension).magnitude() < 1e-5f &&
                (SampleFootstepPosition(step, GetFootstepRemainingTime(step)) - step.target.position).magnitude() < 1e-5f,
                "A longer support forecast extends the same swing from its current position up to the whole-step cap");
            input.feet[0].timeToReachLimit = shorterForecast;
            Advance(planner, input, settings);
            Check(Near(GetFootstepRemainingTime(step), shorterForecast),
                "The same swing can shorten again after an extension");
            input.feet[0].grounded = false;
            Advance(planner, input, settings);
            const float remaining = shorterForecast - input.fixedDeltaTime;
            Check(Near(GetFootstepRemainingTime(step), remaining) &&
                (SampleFootstepPosition(step, remaining) - step.target.position).magnitude() < 1e-5f,
                "Losing support preserves the promised time and the drive trajectory reaches the landing point");
            Advance(planner, input, settings, static_cast<unsigned>(std::ceil(shorterForecast / input.fixedDeltaTime)));
            Check(step.IsInProgress() && Near(step.motion.progress, 1) && !step.justLanded,
                "The deadline alone cannot manufacture a ground contact");
            const float expiredDuration = step.motion.duration;
            input.feet[0].grounded = true;
            input.feet[0].timeToReachLimit = .5f;
            Advance(planner, input, settings);
            Check(Near(step.motion.duration, expiredDuration) && Near(GetFootstepRemainingTime(step), 0) &&
                step.motion.landingLowering > 0 && !step.justLanded,
                "A later forecast preserves an expired deadline while the airborne foot lowers for contact");
            input.feet[1].position = step.target.position;
            input.feet[1].grounded = true;
            Advance(planner, input, settings);
            Check(step.justLanded && Near(step.motion.landingError, 0) && step.motion.landingTimeError > 0,
                "A late physical touchdown completes the step and reports the missed deadline");

            FootstepPlanner descentPlanner;
            FootstepInput descent = Standing();
            descent.fixedDeltaTime = input.fixedDeltaTime;
            descent.feet[0].timeToReachLimit = MinimumFootstepDuration;
            descentPlanner.Update(descent, settings);
            const FootstepCommand& descendingStep = descentPlanner.GetSnapshot().feet[1];
            descent.feet[1].grounded = false;
            descent.feet[0].timeToReachLimit = MinimumFootstepDuration - .5f * descent.fixedDeltaTime;
            Advance(descentPlanner, descent, settings);
            Check(Near(descendingStep.motion.duration, MinimumFootstepDuration),
                "A sub-tick increase keeps the current deadline during lift");
            descent.feet[0].timeToReachLimit = 0;
            const unsigned ticksToApex = static_cast<unsigned>(std::ceil(.5f * MinimumFootstepDuration / descent.fixedDeltaTime));
            Advance(descentPlanner, descent, settings, ticksToApex);
            descent.feet[0].timeToReachLimit = .5f;
            Advance(descentPlanner, descent, settings);
            Check(descendingStep.motion.progress > .5f && descendingStep.motion.progress < 1 &&
                Near(descendingStep.motion.duration, MinimumFootstepDuration),
                "A longer support forecast during descent keeps the committed touchdown");
        }

        FootstepPlanner planner;
        FootstepInput input = Standing();
        planner.Update(input, settings);
        Advance(planner, input, settings, 2);
        const FootstepCommand& step = planner.GetSnapshot().feet[1];
        const PxVec3 lastLift = step.motion.position;
        input.feet[0].timeToReachLimit = .3f;
        Advance(planner, input, settings);
        const float remainingBudget = .3f - 3.0f * input.fixedDeltaTime;
        Check(step.motion.hasSupportDeadline && Near(GetFootstepRemainingTime(step), remainingBudget) &&
            (step.motion.position - lastLift).magnitude() < 1e-6f,
            "A first usable support measurement during lift preserves the current foot pose");
        input.feet[1].grounded = false;
        Advance(planner, input, settings);
        input.feet[0].timeToReachLimit = 2.0f;
        input.feet[1].grounded = true;
        input.feet[1].position = step.target.position;
        input.feet[1].corners = SoleRectangle(input.feet[1].position, .04f, .1f);
        Advance(planner, input, settings);
        Check(step.justLanded && Near(step.motion.duration, remainingBudget) && step.motion.progress < 1 && step.motion.landingTimeError < 0,
            "Reaching the target on real contact finishes early without postponing the promised touchdown");
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

void TestImpactWarmupPlanning()
{
    const FootstepInput standing = Standing();
    const float dt = standing.fixedDeltaTime;
    const PxVec3 shove(1, 0, 0);
    const float reach = .18f;
    const ImpactWarmupStep centered = PlanImpactWarmupStep(standing.hull, standing.com, shove, reach, dt);
    const ImpactWarmupStep nearEdge = PlanImpactWarmupStep(standing.hull, PxVec3(.23f, 1, 0), shove, reach, dt);
    Check(nearEdge.duration < centered.duration && nearEdge.stride.x > centered.stride.x,
        "The same shove from near the support edge needs a sooner, longer warmup placement");
    const ImpactWarmupStep held = PlanImpactWarmupStep(standing.hull, standing.com, shove * 4, reach, dt);
    Check(held.duration < centered.duration && held.stride.x > centered.stride.x && held.stride.magnitude() <= reach,
        "Greater remaining momentum increases the step rate and stride within reach");
    const ImpactWarmupStep leaning = PlanImpactWarmupStep(standing.hull, PxVec3(0, 1, .22f), shove, reach, dt);
    const ImpactWarmupStep opposite = PlanImpactWarmupStep(standing.hull, PxVec3(0, 1, -.22f), shove, reach, dt);
    Check(leaning.stride.x > 0 && leaning.stride.z > 0 && Near(leaning.stride.z, -opposite.stride.z),
        "The predicted COM adds the missing support direction to the same incoming impulse");
    const PxQuat rotation(.83f, PxVec3(0, 1, 0));
    const PxVec3 shift(13, 3, -8);
    FootstepPolygon transformed;
    for (const PxVec3& corner : standing.hull)
    {
        transformed.push_back(shift + rotation.rotate(corner));
    }
    const ImpactWarmupStep rotated = PlanImpactWarmupStep(transformed, shift + rotation.rotate(standing.com),
        rotation.rotate(shove), reach, dt);
    Check(Near(rotated.duration, centered.duration) && (rotated.stride - rotation.rotate(centered.stride)).magnitude() < 1e-5f,
        "Sliding and turning the entire stance preserves pelvis-relative planning");
    const ImpactWarmupStep settled = PlanImpactWarmupStep(standing.hull, standing.com, PxVec3(0), reach, dt);
    Check(settled.stride.isZero() && settled.duration >= centered.duration && Near(held.duration / dt, std::round(held.duration / dt)),
        "A settled COM finishes in place and landing times use whole physics ticks");

    const PxVec3 forward(0, 0, 1);
    const PxVec3 swingOffset(.15f, 0, .05f);
    const PxVec3 supportOffset(-.15f, 0, -.05f);
    const PxVec3 stride(.1f, 0, 0);
    const PxVec3 diagonal(2, 0, 1);
    const float angle = std::atan2(2.0f, 1.0f);
    Check(Near(SelectImpactWarmupYaw(forward, diagonal, -1, swingOffset, supportOffset, stride), angle) &&
        Near(SelectImpactWarmupYaw(forward, PxVec3(2, 0, -1), 1, swingOffset, supportOffset, stride), -angle) &&
        Near(SelectImpactWarmupYaw(forward, -diagonal, -1, swingOffset, supportOffset, stride), angle),
        "Oblique side shoves turn toward the nearer forward or backward step");
    Check(SelectImpactWarmupYaw(forward, forward, 1, swingOffset, supportOffset, stride) == 0 &&
        SelectImpactWarmupYaw(forward, -forward, -1, swingOffset, supportOffset, stride) == 0 &&
        SelectImpactWarmupYaw(forward, PxVec3(0), 1, swingOffset, supportOffset, stride) == 0,
        "Axial shoves and a stopped COM retain the heading");
    const PxVec3 foreOffset(0, 0, .12f);
    Check(Near(SelectImpactWarmupYaw(forward, shove, 1, swingOffset, supportOffset, stride), PxHalfPi) &&
        Near(SelectImpactWarmupYaw(forward, shove, -1, swingOffset, supportOffset, stride), -PxHalfPi) &&
        Near(SelectImpactWarmupYaw(forward, shove, 0, swingOffset, supportOffset, stride), -PxHalfPi) &&
        Near(SelectImpactWarmupYaw(forward, shove, 0, swingOffset + foreOffset, supportOffset + foreOffset, stride), PxHalfPi),
        "A lateral tie follows pelvis yaw first, then the heading that preserves foot order");
    Check(Near(SelectImpactWarmupYaw(rotation.rotate(forward), rotation.rotate(shove), 0, rotation.rotate(swingOffset),
        rotation.rotate(supportOffset), rotation.rotate(stride)), -PxHalfPi),
        "The warmup turn is independent of the character's starting heading");
    std::puts("PASS: COM-based warmup cadence, reach and lateral turn into forward or backward steps");
}

void TestFootstepPlanner()
{
    TestFootstepRecovery();
    checks = 0;
    TestImpactWarmupPlanning();
    TestPredictionAndTrajectory();
    TestImpactSelection();
    TestIndependentCommands();
    TestGeometry();
    TestGatesAndInvalidData();
    TestActualComBufferTimeout();
    TestBackwardFallSteps();
    TestBackwardFallInterruptions();
    TestClockAndAbort();
    TestBackwardSwing();
    TestSwingSoleSeparation();
    TestLoadedTouchdown();
    TestFixedStepRates();
    TestTakeoffConfirmation();
    TestSupportLandingDeadline();
    TestLoadTransferAtRates();
    TestLoadTransferInterruptions();
    TestLandingComGeometry();
    TestLandingComWorldPrecision();
    TestLandingComDisabledAndStart();
    TestLandingComHistoryAndLock();
    std::printf("PASS: %u independent foot-command checks\n", checks);
}
