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

    FootstepSettings TestSettings()
    {
        // 궤적 검증의 시간 조건을 고정해 기본 튜닝 값의 변경과 분리한다.
        FootstepSettings settings;
        settings.startPredictionTime = .3f;
        settings.landingPredictionTime = .3f;
        settings.stepDuration = .3f;
        return settings;
    }

    void Advance(FootstepPlanner& planner, FootstepInput& input, const FootstepSettings& settings, unsigned count)
    {
        for (unsigned sample = 0; sample < count; ++sample)
        {
            ++input.completedStep;
            planner.Update(input, settings);
        }
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
        input.fixedDeltaTime = 1.0f / 60;
        input.bodyWeight = 1000;
        input.blueFoot = 1;
        input.completedStep = 10;
        input.revision = 1;
        for (size_t side = 0; side < input.feet.size(); ++side)
        {
            FootstepFootInput& foot = input.feet[side];
            foot.available = foot.hasContactSample = foot.grounded = true;
            const float x = side == 0 ? -.12f : .12f;
            foot.position = PxVec3(x, .4f, 0);
            foot.normal = PxVec3(0, 1, 0);
            foot.stanceOffset = PxVec3(x, 0, 0);
            foot.calfLength = .45f;
            foot.normalImpulse = input.bodyWeight * .5f * input.fixedDeltaTime;
        }
        return input;
    }

    void TestStartConditions()
    {
        FootstepPlanner planner;
        FootstepSettings settings = TestSettings();
        PARS::FootstepInput input = Standing();
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
        const PARS::FootstepSnapshot started = planner.GetSnapshot();
        Check(started.justStarted && started.stepFoot == 1 && started.supportFoot == 0 && started.startCount == 1,
            "Predictive exit starts the blue foot while actual COM is still inside A");
        Check(Near(started.predictedCom.z, .3f) && Near(started.predictedCom.y, .4f),
            "Start prediction uses 0.3 seconds and support projection height");
        Check(Near(settings.stanceScale, 1.0f) && Near(started.target.x, .12f) &&
                Near(started.target.z, .3f) && Near(started.target.y, .4f),
            "Target adds the full reference foot offset at the ground plane height");
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

    void TestBufferStartCandidates()
    {
        const FootstepSettings settings;
        for (const float direction : {-1.0f, 1.0f})
        {
            FootstepInput input = Standing();
            std::array<std::vector<PxVec3>, 2> feet;
            std::vector<PxVec3> points;
            for (size_t side = 0; side < feet.size(); ++side)
            {
                const PxVec3 center = input.feet[side].position;
                feet[side] = {center + PxVec3(-.05f, 0, -.15f), center + PxVec3(.05f, 0, -.15f),
                    center + PxVec3(.05f, 0, .15f), center + PxVec3(-.05f, 0, .15f)};
                points.insert(points.end(), feet[side].begin(), feet[side].end());
            }
            input.hull = BuildFootSupportHull(points);
            input.com.x = .25f * direction;
            input.velocity = PxVec3(direction, 0, 0);
            input.zone = EvaluateFootSupportBalance(input.hull, input.com).zone;
            // 후보 번호를 직접 지정하지 않고 실제 영역 판정과 후보 선택을 플래너에 연결한다.
            input.blueFoot = EvaluateFootSupportCandidates(input.hull, input.com, feet, input.zone).step;
            Check(input.zone == FootSupportZone::Buffer && !IsInsideFootSupportHull(input.hull, input.com),
                "Buffer-start fixture places actual COM outside A but inside B");
            FootstepPlanner planner;
            planner.Update(input, settings);
            const FootstepSnapshot& started = planner.GetSnapshot();
            Check(started.needsStep && started.justStarted && started.startCount == 1 &&
                    started.stepFoot == (direction < 0 ? 1 : 0) && started.hasTarget && started.motion.valid,
                "Actual COM in the buffer starts a fresh step through the real candidate selector");

            // 후보가 생겨도 반대 발의 실제 지면 접촉 조건은 계속 필요하다.
            const int supportFoot = 1 - input.blueFoot;
            input.feet[supportFoot].grounded = false;
            planner.Reset();
            planner.Update(input, settings);
            Check(planner.GetSnapshot().startCount == 0 && !planner.GetSnapshot().hasTarget &&
                    planner.GetSnapshot().reason == FootstepReason::NoStanceContact,
                "Buffer start still requires actual contact on the opposite foot");
            input.feet[supportFoot].grounded = true;

            input.com.x = .5f * direction;
            input.zone = EvaluateFootSupportBalance(input.hull, input.com).zone;
            input.blueFoot = EvaluateFootSupportCandidates(input.hull, input.com, feet, input.zone).step;
            planner.Reset();
            planner.Update(input, settings);
            Check(input.zone == FootSupportZone::Outside && input.blueFoot == -1 &&
                    planner.GetSnapshot().phase == FootstepPhase::Aborted && planner.GetSnapshot().startCount == 0,
                "Actual COM beyond B still aborts without starting a step");
            input.com.x = .25f * direction;
            input.zone = EvaluateFootSupportBalance(input.hull, input.com).zone;
            input.blueFoot = EvaluateFootSupportCandidates(input.hull, input.com, feet, input.zone).step;
            ++input.completedStep;
            planner.Update(input, settings);
            Check(planner.GetSnapshot().phase == FootstepPhase::Aborted && planner.GetSnapshot().startCount == 0,
                "Returning to the buffer does not bypass the existing abort latch");
        }
    }

    void TestStepLifecycle()
    {
        FootstepPlanner planner;
        const FootstepSettings settings = TestSettings();
        PARS::FootstepInput input = Standing();
        planner.Update(input, settings);
        const PARS::FootstepSnapshot start = planner.GetSnapshot();
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
            !planner.GetSnapshot().motion.valid && planner.GetSnapshot().reason == FootstepReason::NoStanceContact,
            "Lost stance contact suspends output without reselecting the moving foot");
        input.feet[0].grounded = true;
        ++input.completedStep;
        planner.Update(input, settings);
        Check(planner.GetSnapshot().hasTarget && planner.GetSnapshot().startCount == 1,
            "Regained stance contact resumes the existing target without another start");
        input.feet[1].grounded = true;
        input.completedStep = start.startedStep + 18;
        planner.Update(input, settings);
        Check(planner.GetSnapshot().justLanded && planner.GetSnapshot().landingCount == 1 &&
            !planner.GetSnapshot().hasTarget && !planner.GetSnapshot().motion.valid && planner.GetSnapshot().startCount == 1,
            "Recontact after takeoff completes once and does not start again in the same step");
        input.zone = FootSupportZone::Inside;
        input.com.z = 0;
        input.com.x = input.feet[1].position.x;
        input.blueFoot = 0;
        ++input.completedStep;
        planner.Update(input, settings);
        Check(!planner.GetSnapshot().justStarted && planner.GetSnapshot().reason == FootstepReason::WaitingSupportLoad,
            "The first post-landing sample waits for sustained load");
        Advance(planner, input, settings, 2);
        Check(planner.GetSnapshot().justStarted && planner.GetSnapshot().startCount == 2 &&
            planner.GetSnapshot().stepFoot == 0, "Sustained predicted-side support permits the next step");
    }

    void TestFootRolesFollowCommittedStep()
    {
        const FootstepSettings settings = TestSettings();
        for (const int movingFoot : {0, 1})
        {
            FootstepPlanner planner;
            FootstepInput input = Standing();
            const int supportFoot = 1 - movingFoot;
            std::array<std::vector<PxVec3>, 2> feet;
            for (size_t side = 0; side < feet.size(); ++side)
            {
                const PxVec3 center = input.feet[side].position;
                feet[side] = {center + PxVec3(-.05f, 0, -.1f), center + PxVec3(.05f, 0, -.1f),
                    center + PxVec3(.05f, 0, .1f), center + PxVec3(-.05f, 0, .1f)};
            }
            input.com.x = input.feet[supportFoot].position.x;
            input.blueFoot = EvaluateFootSupportCandidates(input.hull, input.com, feet, input.zone).step;
            input.velocity = PxVec3(0);
            planner.Update(input, settings);
            FootstepRoles roles = planner.GetSnapshot().GetRoles();
            Check(roles.IsValid() && roles.movableFoot == movingFoot && roles.supportFoot == supportFoot && !roles.locked,
                "Standing roles identify the eligible moving foot and its actual supporting opposite foot");

            input.velocity.z = 1;
            ++input.completedStep;
            planner.Update(input, settings);
            const uint64_t startedStep = planner.GetSnapshot().startedStep;
            roles = planner.GetSnapshot().GetRoles();
            Check(planner.GetSnapshot().justStarted && roles.locked && roles.movableFoot == movingFoot,
                "Starting a step commits the same moving foot that was displayed while standing");

            // 실제 거리 판정으로 후보를 뒤집되 이륙 대기와 공중 이동의 역할은 바꾸지 않는다.
            input.com.x = input.feet[movingFoot].position.x;
            input.blueFoot = EvaluateFootSupportCandidates(input.hull, input.com, feet, input.zone).step;
            Check(input.blueFoot == supportFoot, "COM crossing really flips the geometric candidate");
            for (const bool grounded : {true, false})
            {
                input.feet[movingFoot].grounded = grounded;
                ++input.completedStep;
                planner.Update(input, settings);
                const FootstepSnapshot& state = planner.GetSnapshot();
                roles = state.GetRoles();
                Check(state.rawBlueFoot != state.stepFoot && roles.locked && roles.movableFoot == state.stepFoot &&
                        roles.supportFoot == state.supportFoot && roles.movableFoot == movingFoot,
                    "Awaiting takeoff and airborne colors follow committed roles despite COM candidate reversal");
            }

            input.blueFoot = -1;
            input.feet[supportFoot].grounded = false;
            ++input.completedStep;
            planner.Update(input, settings);
            roles = planner.GetSnapshot().GetRoles();
            Check(!planner.GetSnapshot().motion.valid && roles.locked && roles.movableFoot == movingFoot &&
                    roles.supportFoot == supportFoot,
                "Missing candidate and lost stance contact suspend motion without dropping or swapping roles");
            input.feet[supportFoot].grounded = true;
            input.feet[movingFoot].hasContactSample = false;
            ++input.completedStep;
            planner.Update(input, settings);
            Check(planner.GetSnapshot().GetRoles().movableFoot == movingFoot,
                "Missing contact samples retain the committed moving role");
            input.feet[movingFoot].hasContactSample = true;
            input.feet[movingFoot].grounded = true;
            input.blueFoot = supportFoot;
            ++input.completedStep;
            planner.Update(input, settings);
            Check(!planner.GetSnapshot().justLanded && planner.GetSnapshot().GetRoles().movableFoot == movingFoot,
                "Early recontact cannot unlock roles before the existing landing rule accepts it");
            input.completedStep = startedStep + 18;
            planner.Update(input, settings);
            roles = planner.GetSnapshot().GetRoles();
            Check(planner.GetSnapshot().justLanded && roles.locked && roles.movableFoot == movingFoot,
                "The landing event retains the roles used by recovery in that same physics tick");
            ++input.completedStep;
            planner.Update(input, settings);
            Check(!planner.GetSnapshot().justStarted && planner.GetSnapshot().GetRoles().movableFoot == movingFoot,
                "The landing impulse alone cannot swap the committed support roles");
            Advance(planner, input, settings, 2);
            roles = planner.GetSnapshot().GetRoles();
            Check(planner.GetSnapshot().justStarted && roles.movableFoot == supportFoot && roles.supportFoot == movingFoot,
                "Sustained support after landing permits the predicted role change");

            input.zone = FootSupportZone::Outside;
            ++input.completedStep;
            planner.Update(input, settings);
            Check(!planner.GetSnapshot().GetRoles().IsValid(), "Aborting clears visible roles even when stored foot IDs remain");
            input.zone = FootSupportZone::Inside;
            ++input.completedStep;
            planner.Update(input, settings);
            Check(!planner.GetSnapshot().GetRoles().IsValid(), "Returning inside B cannot display a movable role while abort is latched");
            planner.Reset();
            Check(!planner.GetSnapshot().GetRoles().IsValid(), "Reset clears committed and candidate roles together");
        }

        for (int blocked = 0; blocked < 4; ++blocked)
        {
            FootstepPlanner planner;
            FootstepInput input = Standing();
            if (blocked == 0)
            {
                input.active = false;
            }
            else if (blocked == 1)
            {
                input.automaticSupport = false;
            }
            else if (blocked == 2)
            {
                input.idleAnimation = false;
            }
            else
            {
                input.feet[0].grounded = false;
            }
            planner.Update(input, settings);
            Check(!planner.GetSnapshot().GetRoles().IsValid(),
                "Unavailable stepping modes and missing actual support cannot advertise movable foot roles");
        }
    }

    void TestAbortAndInvalidInputs()
    {
        FootstepPlanner planner;
        const FootstepSettings settings = TestSettings();
        PARS::FootstepInput input = Standing();
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
            planner.GetSnapshot().landingCount == 0 && !planner.GetSnapshot().hasTarget && !planner.GetSnapshot().motion.valid,
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
        FootstepSettings settings = TestSettings();
        PARS::FootstepInput input = Standing();
        input.velocity = PxVec3(100, 100, 0);
        input.feet[1].position.x = 10;
        planner.Update(input, settings);
        const PARS::FootstepTarget capped = planner.GetSnapshot().candidates[1];
        Check(capped.valid && capped.lengthLimited && Near(capped.stanceDistance, 2),
            "Maximum step length is two meters from the opposite foot");
        Check(capped.travelDistance > 2 && Near(capped.position.y, .4f),
            "The cap is not measured from the moving foot and ignores vertical COM velocity");
        planner.Reset();
        input = Standing();
        input.feet[0].normal = PxVec3(0, .8f, .6f);
        planner.Update(input, settings);
        const PARS::FootstepTarget slope = planner.GetSnapshot().candidates[1];
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
        const PARS::FootstepTarget crossed = planner.GetSnapshot().candidates[1];
        Check(crossed.valid && crossed.crossingLimited &&
            Near(crossed.position.x - input.feet[0].position.x, .1f) && crossed.stanceDistance <= 2.00001f,
            "Optional crossing boundary and maximum radius remain satisfied together");

        planner.Reset();
        input = Standing();
        settings = TestSettings();
        // 기준 방향을 90도 돌려도 발 간격 보정이 월드 X축에 고정되지 않아야 한다.
        input.supportRight = PxVec3(0, 0, 1);
        input.feet[0].stanceOffset = PxVec3(0, 0, -.12f);
        input.feet[1].stanceOffset = PxVec3(0, 0, .12f);
        planner.Update(input, settings);
        Check(Near(planner.GetSnapshot().target.x, 0) && Near(planner.GetSnapshot().target.z, .42f),
            "Reference stance offset follows the supplied support heading");
        settings.maximumStepLength = -1;
        ++input.completedStep;
        planner.Update(input, settings);
        Check(!planner.GetSnapshot().hasTarget && planner.GetSnapshot().reason == FootstepReason::InvalidSettings,
            "Invalid settings clear output rather than retaining a stale target");
    }

    void TestMotionHeightAndProgress()
    {
        FootstepPlanner planner;
        const FootstepSettings settings = TestSettings();
        FootstepInput input = Standing();
        planner.Update(input, settings);
        const FootstepSnapshot start = planner.GetSnapshot();
        Check(start.motion.valid && (start.motion.position - start.startPosition).magnitude() < 1e-6f &&
            Near(start.motion.height, 0) && Near(start.motion.elapsedTime, 0),
            "Movement begins at the actual foot instead of jumping to the landing target");
        Check(Near(start.motion.duration, .3f) && Near(settings.minimumLiftHeight, .03f) &&
                start.motion.initialHeight >= settings.minimumLiftHeight,
            "Movement has its own duration and minimum peak clearance");
        planner.Update(input, settings);
        Check(Near(planner.GetSnapshot().motion.elapsedTime, 0),
            "Duplicate physics samples cannot advance movement time");

        // ?? ?? ???? ??? ?? ??? ?? ??? ????.
        input.feet[1].grounded = false;
        input.completedStep = start.startedStep + 9;
        planner.Update(input, settings);
        const FootstepSnapshot middle = planner.GetSnapshot();
        const PxVec3 midpoint = (start.startPosition + middle.target) * .5f;
        Check(Near(middle.motion.progress, .5f) && Near(middle.motion.measuredProgress, 0) &&
            Near(middle.motion.height, start.motion.initialHeight) &&
            Near(middle.motion.position.x, midpoint.x) && Near(middle.motion.position.z, midpoint.z),
            "Half the duration reaches the path midpoint and peak without relying on actual foot progress");
        input.feet[1].position = middle.target + PxVec3(0, .2f, 0);
        ++input.completedStep;
        planner.Update(input, settings);
        Check(Near(planner.GetSnapshot().motion.measuredProgress, 1) &&
            planner.GetSnapshot().motion.progress < 1 && planner.GetSnapshot().motion.height > 0,
            "Early actual arrival does not skip the timed lift");
        input.completedStep = start.startedStep + 18;
        planner.Update(input, settings);
        Check(Near(planner.GetSnapshot().motion.progress, 1) && Near(planner.GetSnapshot().motion.height, 0) &&
            (planner.GetSnapshot().motion.position - planner.GetSnapshot().target).magnitude() < 1e-6f &&
            planner.GetSnapshot().phase == FootstepPhase::Airborne && planner.GetSnapshot().landingCount == 0,
            "The duration ends at the target but still requires actual contact");
        input.feet[1].position = planner.GetSnapshot().target;
        input.feet[1].grounded = true;
        ++input.completedStep;
        planner.Update(input, settings);
        Check(planner.GetSnapshot().justLanded && !planner.GetSnapshot().motion.valid &&
            Near(planner.GetSnapshot().motion.landingError, 0),
            "Contact after the duration finishes the step and records landing error");
        input.blueFoot = 0;
        Advance(planner, input, settings, 3);
        Check(planner.GetSnapshot().justStarted && Near(planner.GetSnapshot().motion.elapsedTime, 0),
            "A subsequent step starts a fresh clock");

        // ?? ?? ??? ??? ?? ?? ??? ??? ??? ??.
        for (const float dt : {1.0f / 60, 1.0f / 120})
        {
            planner.Reset();
            input = Standing();
            input.fixedDeltaTime = dt;
            planner.Update(input, settings);
            input.feet[1].grounded = false;
            input.completedStep += static_cast<uint64_t>(std::lround(.15f / dt));
            planner.Update(input, settings);
            Check((planner.GetSnapshot().motion.position - middle.motion.position).magnitude() < 1e-5f,
                "Equal elapsed physics time produces the same path target at 60 and 120 Hz");
        }
    }

    void TestMotionLimitsAndRetargeting()
    {
        FootstepPlanner planner;
        FootstepSettings settings = TestSettings();
        FootstepInput input = Standing();
        input.velocity.z = 100;
        input.feet[1].calfLength = .18f;
        planner.Update(input, settings);
        Check(Near(planner.GetSnapshot().motion.initialHeight, .18f),
            "Long movement caps the peak at the reference calf length");
        for (const float length : {.02f, .04f, .45f})
        {
            planner.Reset();
            input = Standing();
            input.feet[1].position = PxVec3(.12f, .4f, .29f);
            input.feet[1].calfLength = length;
            planner.Update(input, settings);
            Check(Near(planner.GetSnapshot().motion.initialHeight, (std::min)(settings.minimumLiftHeight, length)),
                "Short steps get minimum clearance without exceeding the calf cap");
        }
        for (const float length : {0.0f, -1.0f, std::numeric_limits<float>::quiet_NaN()})
        {
            planner.Reset();
            input = Standing();
            input.feet[1].calfLength = length;
            planner.Update(input, settings);
            Check(planner.GetSnapshot().reason == FootstepReason::NoCalfLength &&
                !planner.GetSnapshot().motion.valid, "Invalid calf length rejects the trajectory");
        }
        planner.Reset();
        input = Standing();
        input.feet[1].position = PxVec3(.12f, .4f, .3f);
        planner.Update(input, settings);
        Check(planner.GetSnapshot().reason == FootstepReason::NoStepDistance,
            "A zero-distance target still does not start a step");

        planner.Reset();
        input = Standing();
        input.feet[0].normal = PxVec3(0, .8f, .6f);
        planner.Update(input, settings);
        const FootstepSnapshot slopeStart = planner.GetSnapshot();
        input.feet[1].grounded = false;
        input.completedStep += 9;
        planner.Update(input, settings);
        const FootstepSnapshot slopeMiddle = planner.GetSnapshot();
        Check(Near(slopeMiddle.motion.position.y,
            (slopeStart.startPosition.y + slopeMiddle.target.y) * .5f + slopeMiddle.motion.initialHeight),
            "Sloped endpoints interpolate their height before adding vertical clearance");

        planner.Reset();
        input = Standing();
        planner.Update(input, settings);
        const FootstepSnapshot start = planner.GetSnapshot();
        input.feet[1].grounded = false;
        input.velocity.z = 2;
        settings.stepDuration = .8f;
        settings.minimumLiftHeight = .2f;
        input.completedStep += 9;
        planner.Update(input, settings);
        Check(Near(planner.GetSnapshot().target.z, .6f) && Near(planner.GetSnapshot().motion.progress, .5f) &&
            Near(planner.GetSnapshot().motion.duration, .3f) &&
            Near(planner.GetSnapshot().motion.initialHeight, start.motion.initialHeight),
            "Live candidates update while movement duration and peak height stay latched");

        // ?? ??? ?? ?? ????? ??? ??? ???.
        planner.Reset();
        settings = TestSettings();
        input = Standing();
        planner.Update(input, settings);
        input.feet[1].grounded = false;
        ++input.completedStep;
        planner.Update(input, settings);
        input.feet[1].grounded = true;
        ++input.completedStep;
        planner.Update(input, settings);
        Check(!planner.GetSnapshot().justLanded && planner.GetSnapshot().motion.valid &&
            planner.GetSnapshot().motion.height > 0, "Early recontact does not cut off the lift");
        input.completedStep += 16;
        planner.Update(input, settings);
        Check(planner.GetSnapshot().justLanded && planner.GetSnapshot().motion.landingError > .29f,
            "End-of-duration contact finishes while retaining an undershot landing error");
        ++input.revision;
        planner.Update(input, settings);
        Check(planner.GetSnapshot().motion.valid && Near(planner.GetSnapshot().motion.elapsedTime, 0),
            "A physics reset clears elapsed time and landing history");
        input.feet[1].grounded = false;
        input.feet[1].position.x = std::numeric_limits<float>::quiet_NaN();
        ++input.completedStep;
        planner.Update(input, settings);
        Check(!planner.GetSnapshot().motion.valid, "Nonfinite actual foot position invalidates output");

        for (int invalid = 0; invalid < 3; ++invalid)
        {
            planner.Reset();
            input = Standing();
            settings = TestSettings();
            if (invalid == 0)
            {
                settings.stepDuration = 0;
            }
            else if (invalid == 1)
            {
                settings.minimumLiftHeight = std::numeric_limits<float>::quiet_NaN();
            }
            else
            {
                input.fixedDeltaTime = 0;
            }
            planner.Update(input, settings);
            Check(planner.GetSnapshot().reason == FootstepReason::InvalidSettings &&
                !planner.GetSnapshot().motion.valid, "Invalid movement parameters reject the trajectory");
        }
        planner.Reset();
        input = Standing();
        settings = TestSettings();
        planner.Update(input, settings);
        input.completedStep += 18;
        planner.Update(input, settings);
        Check(planner.GetSnapshot().justLanded && planner.GetSnapshot().motion.landingError > .29f,
            "A foot that never lifted cannot stay stuck after the trajectory duration");
    }

    void TestSupportTransfer()
    {
        const FootstepSettings settings;
        Check(Near(settings.stepDuration, .15f) && Near(settings.landingPredictionTime, .3f),
            "The tuned movement and landing prediction defaults remain unchanged");
        for (const float dt : {1.0f / 60, 1.0f / 120})
        {
            FootstepPlanner planner;
            FootstepInput input = Standing();
            input.fixedDeltaTime = dt;
            planner.Update(input, settings);
            input.feet[1].grounded = false;
            Advance(planner, input, settings, 1);
            // 현재 COM은 왼발에 가깝지만 다음 이동 시간 뒤에는 오른발에 더 가깝다.
            input.com.x = -.06f;
            input.velocity.x = 1;
            input.blueFoot = 1;
            input.feet[1].grounded = true;
            input.feet[1].normalImpulse = input.bodyWeight * dt * 12;
            input.completedStep += static_cast<uint64_t>(std::ceil(settings.stepDuration / dt));
            planner.Update(input, settings);
            Check(planner.GetSnapshot().justLanded && Near(planner.GetSnapshot().transfer.confirmationTime, 0),
                "Landing spikes are excluded from transfer confirmation");
            Advance(planner, input, settings, 1);
            Check(planner.GetSnapshot().transfer.candidateFoot == 1 && !planner.GetSnapshot().justStarted,
                "Predicted COM selects the future support but a single peak cannot release the old foot");
            planner.Update(input, settings);
            Check(Near(planner.GetSnapshot().transfer.confirmationTime, dt),
                "Duplicate observations do not accumulate support time");
            input.feet[1].normalImpulse = 0;
            Advance(planner, input, settings, 1);
            Check(Near(planner.GetSnapshot().transfer.confirmationTime, 0) && !planner.GetSnapshot().transfer.ready,
                "A landing spike followed by unloaded contact cannot transfer support");
            input.feet[1].normalImpulse = input.bodyWeight * dt * .6f;
            const unsigned samples = static_cast<unsigned>(std::ceil(settings.supportConfirmationTime / dt - 1e-5f));
            Advance(planner, input, settings, samples - 1);
            Check(!planner.GetSnapshot().justStarted, "Support remains locked until the full confirmation time");
            Advance(planner, input, settings, 1);
            Check(planner.GetSnapshot().justStarted && planner.GetSnapshot().stepFoot == 0 &&
                planner.GetSnapshot().supportFoot == 1 && planner.GetSnapshot().rawBlueFoot == 1,
                "Predicted COM with sustained actual load overrides the old instantaneous COM candidate");
            input.velocity.x = -10;
            input.feet[1].normalImpulse = 0;
            Advance(planner, input, settings, 1);
            Check(planner.GetSnapshot().stepFoot == 0 && planner.GetSnapshot().supportFoot == 1,
                "Prediction and load changes cannot swap roles during the committed swing");
        }

        for (unsigned interruption = 0; interruption < 8; ++interruption)
        {
            FootstepPlanner planner;
            FootstepInput input = Standing();
            planner.Update(input, settings);
            input.completedStep += 9;
            planner.Update(input, settings);
            input.com.x = .1f;
            input.feet[1].normalImpulse = input.bodyWeight * input.fixedDeltaTime * .6f;
            Advance(planner, input, settings, 2);
            const FootstepInput valid = input;
            if (interruption == 0)
            {
                input.feet[1].grounded = false;
            }
            else if (interruption == 1)
            {
                input.feet[1].hasContactSample = false;
            }
            else if (interruption == 2)
            {
                input.feet[1].normal = PxVec3(1, 0, 0);
            }
            else if (interruption == 3)
            {
                input.feet[1].normalImpulse = std::numeric_limits<float>::quiet_NaN();
            }
            else if (interruption == 4)
            {
                input.bodyWeight = 0;
            }
            else if (interruption == 5)
            {
                ++input.completedStep;
            }
            else if (interruption == 6)
            {
                input.hasVelocity = false;
            }
            else
            {
                input.com.x = -.1f;
            }
            Advance(planner, input, settings, 1);
            Check(!planner.GetSnapshot().justStarted && !planner.GetSnapshot().transfer.ready,
                "Invalid contact, missing samples, changed candidate or missing input breaks confirmation");
            const uint64_t completedStep = input.completedStep;
            input = valid;
            input.completedStep = completedStep;
            // 샘플 누락은 현재의 유효 샘플부터 다시 센다. 나머지 중단은 다음 입력부터 확인한다.
            Advance(planner, input, settings, interruption == 5 ? 2 : 3);
            Check(planner.GetSnapshot().justStarted && planner.GetSnapshot().stepFoot == 0,
                "A fresh uninterrupted load interval recovers from a rejected transfer");
        }

        FootstepPlanner planner;
        FootstepInput input = Standing();
        input.com.x = -.1f;
        planner.Update(input, settings);
        input.completedStep += 9;
        planner.Update(input, settings);
        Advance(planner, input, settings, 3);
        Check(planner.GetSnapshot().justStarted && planner.GetSnapshot().stepFoot == 1,
            "Repeated use of the same swing foot remains possible when prediction and support load justify it");
        ++input.revision;
        planner.Update(input, settings);
        Check(!planner.GetSnapshot().transfer.active,
            "A contact revision clears post-landing load history");
    }

}

void TestFootstepPlanner()
{
    TestStartConditions();
    TestBufferStartCandidates();
    TestStepLifecycle();
    TestFootRolesFollowCommittedStep();
    TestAbortAndInvalidInputs();
    TestTargetGeometry();
    TestMotionHeightAndProgress();
    TestMotionLimitsAndRetargeting();
    TestSupportTransfer();
    std::printf("PASS: %u procedural footstep checks (no PhysX simulation)\n", checks);
}
