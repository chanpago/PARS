#include "../Model/FootstepCollision.h"
#include <cmath>
#include "../Model/FootstepCollisionBounds.h"
#include "../Model/FootstepAvoidance.h"
#include "FramePerformance.h"
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <limits>

using namespace PARS;
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

    bool Near(const PxVec3& first, const PxVec3& second)
    {
        return (first - second).magnitude() < 1e-5f;
    }

    FootstepCollisionLeg MakeLeg(float x)
    {
        FootstepCollisionLeg leg;
        const float bend = std::atan2(.2f, .45f);
        leg.pose.hip = PxTransform(PxVec3(x, 1, 0), PxQuat(-bend, PxVec3(1, 0, 0)));
        leg.pose.knee = PxTransform(PxVec3(x, .55f, .2f), PxQuat(bend, PxVec3(1, 0, 0)));
        leg.pose.ankle = PxTransform(PxVec3(x, .1f, 0));
        leg.pose.thighLength = std::sqrt(.45f * .45f + .2f * .2f);
        leg.pose.calfLength = leg.pose.thighLength;
        leg.pose.soleInBone = PxTransform(PxVec3(0, -.05f, 0));
        leg.pose.targetSole = leg.pose.ankle * leg.pose.soleInBone;
        leg.pose.referenceBend = PxVec3(0, 0, 1);

        leg.command.phase = FootstepPhase::Airborne;
        leg.command.hasTakenOff = true;
        leg.command.hasTarget = true;
        leg.command.startPosition = leg.pose.targetSole.p;
        leg.command.takeoffPosition = leg.command.startPosition;
        leg.command.target.position = PxVec3(x + .3f, .12f, 0);
        leg.command.target.normal = PxVec3(0, 1, 0);
        leg.command.target.valid = true;
        leg.command.motion.position = leg.command.startPosition;
        leg.command.motion.duration = .3f;
        leg.command.motion.initialHeight = .08f;
        leg.command.motion.valid = true;

        const std::array<PxTransform, 3> bones{leg.pose.hip, leg.pose.knee, leg.pose.ankle};
        for (size_t part = 0; part < leg.shapes.size(); ++part)
        {
            FootstepCollisionShape& shape = leg.shapes[part];
            shape.available = true;
            if (part < 2)
            {
                shape.geometry.storeAny(PxCapsuleGeometry(.04f, .18f));
                shape.inBone = PxTransform(PxVec3(0, -.22f, 0), PxQuat(-PxHalfPi, PxVec3(0, 0, 1)));
            }
            else
            {
                shape.geometry.storeAny(PxBoxGeometry(.06f, .06f, .1f));
                shape.inBone = PxTransform(PxVec3(0, 0, .1f), PxQuat(.2f, PxVec3(0, 1, 0)));
            }
            shape.actualPose = bones[part] * shape.inBone;
        }
        return leg;
    }

    FootstepCollisionInput MakeInput()
    {
        FootstepCollisionInput input;
        input.completedStep = 42;
        input.revision = 7;
        input.fixedDeltaTime = 1.0f / 120.0f;
        input.legs[0] = MakeLeg(0);
        input.legs[1] = MakeLeg(3);
        input.legs[1].command.phase = FootstepPhase::Idle;
        return input;
    }

    void TestFuturePathAndRetarget()
    {
        FootstepCollisionInput input = MakeInput();
        FootstepCollisionSnapshot result = PredictFootstepCollisions(input);
        Check(result.status == FootstepCollisionStatus::Clear && result.sampleCount == 6 &&
            result.queryCount >= 63 && result.overlapPairs == 0,
            "An unobstructed swing certifies intervals as well as six display poses");
        Check(result.completedStep == 42 && result.revision == 7 &&
            std::abs(result.samples[0].timeAhead - input.fixedDeltaTime) < 1e-6f &&
            std::abs(result.samples[5].timeAhead - .3f) < 1e-6f,
            "The snapshot identifies its source tick and includes the next tick and landing");

        // 현재 발은 떨어져 있지만 도착할 발의 충돌체 중심에는 반대 발이 있다.
        input.legs[1].shapes[2].actualPose = PxTransform(PxVec3(.3f, .17f, .1f));
        result = PredictFootstepCollisions(input);
        Check((result.overlapPairs & (1u << 8)) != 0 && result.samples[0].overlapPairs == 0 &&
            (result.samples[5].overlapPairs & (1u << 8)) != 0,
            "A future foot collision is detected before the current feet overlap");
        Check(Near(input.legs[0].pose.ankle.p, PxVec3(0, .1f, 0)) &&
            input.legs[0].command.motion.elapsedTime == 0,
            "Prediction does not advance or move the input controller");

        input.legs[0].command.target.position.x = -.3f;
        ++input.completedStep;
        result = PredictFootstepCollisions(input);
        Check(result.overlapPairs == 0 && result.completedStep == 43,
            "Retargeting uses the new path instead of cached collision results");

        input.legs[0].command.phase = FootstepPhase::Landed;
        result = PredictFootstepCollisions(input);
        Check(result.status == FootstepCollisionStatus::Inactive && result.queryCount == 0 && result.overlapPairs == 0,
            "A finished step clears the prediction and performs no geometry queries");
    }

    void TestAllPairsAndOffsets()
    {
        for (uint32_t left = 0; left < 3; ++left)
        {
            for (uint32_t right = 0; right < 3; ++right)
            {
                FootstepCollisionInput input = MakeInput();
                input.legs[0].command.target.position = input.legs[0].command.startPosition;
                input.legs[0].command.motion.initialHeight = 0;
                for (uint32_t part = 0; part < 3; ++part)
                {
                    if (part != left)
                    {
                        input.legs[0].shapes[part].inBone.p.x += 20 + part;
                    }
                }
                input.legs[1].shapes[right].actualPose = input.legs[0].shapes[left].actualPose;
                const FootstepCollisionSnapshot result = PredictFootstepCollisions(input);
                Check(result.status == FootstepCollisionStatus::Collision &&
                    result.overlapPairs == (1u << (left * 3 + right)),
                    "Every capsule/capsule, capsule/box and box/box pair keeps its PA offset and canonical bit");
            }
        }
    }

    void TestBothMoving()
    {
        FootstepCollisionInput input = MakeInput();
        input.legs[1] = MakeLeg(.3f);
        const FootstepCollisionSnapshot moving = PredictFootstepCollisions(input);
        Check(moving.status == FootstepCollisionStatus::Clear && moving.overlapPairs == 0,
            "Two parallel moving legs are compared over the same future time intervals");
        input.legs[1].command.phase = FootstepPhase::Idle;
        const FootstepCollisionSnapshot stationary = PredictFootstepCollisions(input);
        Check((stationary.overlapPairs & (1u << 8)) != 0,
            "The same opposite leg obstructs the landing when it stays at its actual position");
    }

    void TestTakeoffAndLowering()
    {
        FootstepCollisionInput input = MakeInput();
        FootstepCommand& command = input.legs[0].command;
        command.hasTakenOff = false;
        command.phase = FootstepPhase::AwaitingTakeoff;
        const PxVec3 lifted = SampleFootstepPosition(command, .075f);
        Check(Near(lifted, command.startPosition + PxVec3(0, .04f, 0)),
            "Before takeoff the trajectory raises the foot without horizontal dragging");
        const FootstepCollisionSnapshot result = PredictFootstepCollisions(input);
        Check(result.assumesTakeoff && std::abs(result.samples[5].timeAhead - .45f) < 1e-6f &&
            Near(result.samples[5].solePositions[0], command.target.position),
            "Pre-takeoff sampling includes the remaining lift and marks the assumed takeoff");

        command.hasTakenOff = true;
        command.phase = FootstepPhase::Airborne;
        command.motion.elapsedTime = command.motion.duration;
        command.motion.progress = 1;
        command.motion.landingTargetLocked = true;
        command.motion.landingLowering = .01f;
        command.motion.landingLoweringSpeed = .05f;
        command.motion.maximumLandingLowering = .03f;
        Check(Near(SampleFootstepPosition(command, .2f), command.target.position - PxVec3(0, .02f, 0)) &&
            Near(SampleFootstepPosition(command, 1), command.target.position - PxVec3(0, .03f, 0)),
            "After the nominal duration, prediction continues bounded landing lowering");
        const FootstepCollisionSnapshot lowering = PredictFootstepCollisions(input);
        Check(lowering.status == FootstepCollisionStatus::Clear && lowering.samples[5].timeAhead > 0,
            "An overdue landing still checks its next control target without dividing by zero");
    }

    void TestUnavailableAndReachLimit()
    {
        FootstepCollisionInput input = MakeInput();
        input.legs[0].command.motion.duration = 0;
        FootstepCollisionSnapshot result = PredictFootstepCollisions(input);
        Check(result.status == FootstepCollisionStatus::Unavailable && result.queryCount == 0,
            "An invalid duration is unknown, not a collision-free path");
        input = MakeInput();
        input.legs[1].shapes[0].available = false;
        result = PredictFootstepCollisions(input);
        Check(result.status == FootstepCollisionStatus::Unavailable && result.queryCount == 0,
            "A missing collider does not silently reduce the coverage");
        input = MakeInput();
        input.legs[0].shapes[2].actualPose.p.x = std::numeric_limits<float>::quiet_NaN();
        result = PredictFootstepCollisions(input);
        Check(result.status == FootstepCollisionStatus::Unavailable && result.queryCount == 0,
            "Invalid transforms never enter PhysX geometry queries");
        input = MakeInput();
        input.legs[0].command.target.position.x = 10;
        result = PredictFootstepCollisions(input);
        Check(result.sampleCount == 6 && result.reachLimited,
            "Unreachable targets explicitly report that the solved leg was reach-limited");
    }
}

namespace
{
    void TestIntervalsAndTracking()
    {
        FootstepCollisionInput input = MakeInput();
        input.maximumIntervals = 0;
        FootstepCollisionSnapshot result = PredictFootstepCollisions(input);
        Check(result.status == FootstepCollisionStatus::Uncertain && result.budgetExhausted,
            "Exhausted interval budget never certifies clear from six empty samples");
        input = MakeInput();
        input.includeTrackingError = false;
        input.legs[0].shapes[2].geometry.storeAny(PxSphereGeometry(.002f));
        input.legs[0].shapes[2].inBone = PxTransform(PxIdentity);
        input.legs[1].shapes[2].geometry.storeAny(PxSphereGeometry(.002f));
        std::array<PxTransform, 3> future;
        LegIKResult solved;
        Check(PredictFootstepLeg(input.legs[0], .037f, future, solved), "Build between-sample obstacle");
        input.legs[1].shapes[2].actualPose = future[2];
        result = PredictFootstepCollisions(input);
        for (const FootstepCollisionSample& sample : result.samples)
        {
            Check(sample.overlapPairs == 0, "Thin obstacle lies between all six display poses");
        }
        Check(result.status != FootstepCollisionStatus::Clear, "Thin between-sample collision cannot be accepted");
        std::printf("between-sample: status=%d intervals=%u queries=%u\n",
            int(result.status), result.intervalCount, result.queryCount);
        input = MakeInput();
        input.legs[0].shapes[2].actualPose.p.x += .1f;
        result = PredictFootstepCollisions(input);
        Check(result.maximumTrackingError >= .099f, "Actual-target tracking displacement expands the enclosure");
        input.legs[0].path.enabled = true;
        input.legs[0].path.duration = std::numeric_limits<float>::quiet_NaN();
        Check(PredictFootstepCollisions(input).status == FootstepCollisionStatus::Unavailable,
            "Invalid detour cannot be marked clear");
    }

    void TestBoundsContainCurvedIK()
    {
        uint32_t checked = 0;
        for (int fixture = 0; fixture < 24; ++fixture)
        {
            FootstepCollisionLeg leg = MakeLeg(.12f);
            leg.command.motion.liftTimeRatio = fixture % 2 == 0 ? 1.5f : 1.0f;
            leg.command.motion.swingOutwardOffset.x = fixture % 3 == 0 ? .06f : 0.0f;
            leg.command.target.position.z = .12f * std::sin(float(fixture));
            leg.command.target.position.x += .08f * std::cos(float(fixture));
            leg.path = MakeFootstepPath(PxVec3(0), PxVec3(0),
                PxVec3(.01f, .015f, .06f * std::sin(float(fixture) + .4f)), 0, .3f);
            for (int interval = 0; interval < 30; ++interval)
            {
                const float begin = .01f * interval;
                const float end = begin + .002f;
                std::array<FootstepInterval::Transform, 3> bounds;
                if (!BoundFootstepLeg(leg, begin, end, bounds))
                {
                    continue;
                }
                ++checked;
                for (int sample = 0; sample <= 4; ++sample)
                {
                    const float time = begin + (end - begin) * sample / 4;
                    std::array<PxTransform, 3> actual;
                    LegIKResult solved;
                    Check(PredictFootstepLeg(leg, time, actual, solved), "Dense curved IK reference remains valid");
                    for (int part = 0; part < 3; ++part)
                    {
                        for (int point = 0; point < 8; ++point)
                        {
                            const PxVec3 local(point & 1 ? .2f : -.2f, point & 2 ? .06f : -.06f,
                                point & 4 ? .1f : -.1f);
                            const PxVec3 world = actual[part].transform(local);
                            const FootstepInterval::Vector box = bounds[part].position +
                                FootstepInterval::Rotate(bounds[part].rotation, FootstepInterval::Vector(local));
                            Check(world.x >= box.x.lower && world.x <= box.x.upper &&
                                world.y >= box.y.lower && world.y <= box.y.upper &&
                                world.z >= box.z.lower && world.z <= box.z.upper,
                                "Conservative transform contains dense rotating geometry on the actual nonlinear IK path");
                        }
                    }
                }
            }
        }
        Check(checked >= 300, "Interval regression exercises a substantial set of nontrivial valid bounds");
        std::printf("PASS: %u curved IK intervals contain dense transformed geometry\n", checked);
    }

    FootstepCollisionInput MakeAvoidanceFixture()
    {
        FootstepCollisionInput input = MakeInput();
        input.includeTrackingError = false;
        input.legs[0].shapes[2].geometry.storeAny(PxSphereGeometry(.012f));
        input.legs[0].shapes[2].inBone = PxTransform(PxIdentity);
        input.legs[1].shapes[2].geometry.storeAny(PxSphereGeometry(.012f));
        std::array<PxTransform, 3> middle;
        LegIKResult solved;
        PredictFootstepLeg(input.legs[0], .15f, middle, solved);
        input.legs[1].shapes[2].actualPose = middle[2];
        return input;
    }

    void TestAvoidanceAndContinuity()
    {
        using FramePerformance::Category;
        FootstepCollisionInput input = MakeAvoidanceFixture();
        Check(PredictFootstepCollisions(input).status == FootstepCollisionStatus::Collision,
            "Avoidance fixture obstructs the direct swing");
        FootstepAvoidance avoidance;
        const std::chrono::steady_clock::time_point started = std::chrono::steady_clock::now();
        FramePerformance::BeginFrame();
        avoidance.Update(input, PxVec3(0, 0, 1));
        FramePerformance::EndFrame();
        const FootstepAvoidanceSnapshot first = avoidance.GetSnapshot();
        const double milliseconds = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - started).count();
        std::printf("avoidance: status=%d candidates=%u intervals=%u queries=%u %.3fms\n",
            int(first.status), first.candidatesChecked, first.totalIntervals, first.totalQueries, milliseconds);
        const FramePerformance::Recorder& timings = FramePerformance::GetState().recorder;
        Check(timings.GetStatistics(Category::CurrentPathSamples).calls == 1 &&
            timings.GetStatistics(Category::DetourRanking).calls == 1 &&
            timings.GetStatistics(Category::DetourSamples).calls == first.candidatesChecked &&
            timings.GetStatistics(Category::DetourIntervals).calls > 0,
            "A blocked path records the current check, candidate ranking and each detour check");
        Check(timings.GetStatistics(Category::CurrentPathMidpoints).calls +
            timings.GetStatistics(Category::DetourMidpoints).calls == first.totalIntervals,
            "Every visited interval records one midpoint check for its path");
        Check(timings.GetStatistics(Category::TrajectoryTotal).last <= milliseconds,
            "Nested candidate and interval timings fit within the elapsed avoidance update");
        std::printf("trajectory ms | current samples %.3f, midpoints %.3f, bounds/split %.3f | "
            "detour ranking %.3f, samples %.3f, midpoints %.3f, bounds/split %.3f\n",
            timings.GetStatistics(Category::CurrentPathSamples).last, timings.GetStatistics(Category::CurrentPathMidpoints).last,
            timings.GetStatistics(Category::CurrentPathIntervals).last, timings.GetStatistics(Category::DetourRanking).last,
            timings.GetStatistics(Category::DetourSamples).last, timings.GetStatistics(Category::DetourMidpoints).last,
            timings.GetStatistics(Category::DetourIntervals).last);
        Check(first.status == FootstepAvoidanceStatus::Detouring && first.collision.status == FootstepCollisionStatus::Clear,
            "A blocked direct swing selects a fully checked detour");
        Check(Near(SampleFootstepPath(first.paths[0], 0), PxVec3(0)) &&
            Near(SampleFootstepPath(first.paths[0], .3f), PxVec3(0)) &&
            Near(SampleFootstepPathVelocity(first.paths[0], 0), PxVec3(0)) &&
            Near(SampleFootstepPathVelocity(first.paths[0], .3f), PxVec3(0)),
            "Detour preserves takeoff and landing position and velocity");
        input.legs[0].command.motion.elapsedTime = .03f;
        ++input.completedStep;
        FramePerformance::BeginFrame();
        avoidance.Update(input, PxVec3(0, 0, 1));
        FramePerformance::EndFrame();
        Check(timings.GetStatistics(Category::CurrentPathIntervals).calls == 1 &&
            timings.GetStatistics(Category::DetourRanking).calls == 0 &&
            timings.GetStatistics(Category::DetourSamples).calls == 0 && timings.GetStatistics(Category::DetourMidpoints).calls == 0 &&
            timings.GetStatistics(Category::DetourIntervals).calls == 0,
            "Rechecking a safe path only records the current path stages");
        Check(avoidance.GetSnapshot().directions == first.directions &&
            avoidance.GetSnapshot().candidatesChecked == 0, "A safe selected route is kept on the next tick");
        const PxVec3 offset = SampleFootstepPath(first.paths[0], .08f);
        const PxVec3 velocity = SampleFootstepPathVelocity(first.paths[0], .08f);
        const FootstepPath replacement = MakeFootstepPath(offset, velocity, PxVec3(0, .02f, -.04f), .08f, .22f);
        Check(Near(SampleFootstepPath(replacement, .08f), offset) &&
            Near(SampleFootstepPathVelocity(replacement, .08f), velocity),
            "Replanning joins the old position and derivative at the current flight clock");
        input = MakeAvoidanceFixture();
        input.acceptsPose = [](size_t, const LegIKResult&, float) -> bool
        {
            return false;
        };
        avoidance.Reset();
        avoidance.Update(input, PxVec3(0, 0, 1));
        Check(avoidance.GetSnapshot().status == FootstepAvoidanceStatus::Failed &&
            !avoidance.GetSnapshot().paths[0].enabled, "Joint-rejected candidates are not applied");
        input.legs[1].shapes[2].actualPose.p.x += 3;
        avoidance.Update(input, PxVec3(0, 0, 1));
        Check(avoidance.GetSnapshot().status == FootstepAvoidanceStatus::Direct &&
            avoidance.GetSnapshot().candidatesChecked == 0,
            "Existing joint clamping alone does not trigger collision detour search on a clear direct path");
        input = MakeAvoidanceFixture();
        input.legs[1].shapes[2].geometry.storeAny(PxSphereGeometry(1));
        const PxVec3 landing = input.legs[0].command.target.position;
        avoidance.Update(input, PxVec3(0, 0, 1));
        Check(avoidance.GetSnapshot().status == FootstepAvoidanceStatus::Failed &&
            Near(input.legs[0].command.target.position, landing), "No route reports failure without changing the landing point");
        input.legs[0].command.phase = FootstepPhase::Landed;
        avoidance.Update(input, PxVec3(0, 0, 1));
        Check(avoidance.GetSnapshot().status == FootstepAvoidanceStatus::Inactive &&
            !avoidance.GetSnapshot().paths[0].enabled, "Landing clears the detour state");
    }
}

void TestPelvisTurnTrajectory()
{
    for (size_t side = 0; side < 2; ++side)
    {
        FootstepCollisionInput input = MakeInput();
        input.legs[0] = MakeLeg(-.18f);
        input.legs[1] = MakeLeg(.18f);
        const size_t support = 1 - side;
        input.legs[support].command.phase = FootstepPhase::Idle;
        FootstepCommand& command = input.legs[side].command;
        const float sign = side == 0 ? -1.0f : 1.0f;
        command.target.position = command.startPosition + PxVec3(sign * .075f, 0, -.18f);
        FootstepPelvisTurn turn;
        turn.angle = -sign * PxPi / 6;
        turn.duration = command.motion.duration;
        turn.pelvis = PxTransform(PxVec3(0, 1, 0));
        for (size_t foot = 0; foot < 2; ++foot)
        {
            FootstepCollisionLeg& leg = input.legs[foot];
            turn.legs[foot] = leg.pose;
            leg.pelvisTurn = &turn;
            leg.side = foot;
            leg.pelvisPivot = turn.pelvis.p;
        }
        PxTransform landing(PxIdentity);
        Check(MakeFootSoleTarget(command.target.position, command.target.normal, input.legs[side].pose.targetSole,
            landing, command.target.position - command.startPosition), "Build the unchanged stride-aligned sole");
        turn.SetLandingSole(side, landing);
        Check(std::abs(turn.SoleRotationAt(0).dot(input.legs[side].pose.targetSole.q)) > .99999f &&
            std::abs(turn.SoleRotationAt(turn.duration).dot(landing.q)) > .99999f, "Rotate from the measured foot to the required stride heading");
        input.rejectInvalidPoses = true;
        const FootstepCollisionSnapshot path = PredictFootstepCollisions(input);
        Check(path.status == FootstepCollisionStatus::Clear && !path.reachLimited, "Certify both signs of the moving-pelvis swing");
        for (size_t foot = 0; foot < input.legs.size(); ++foot)
        {
            const FootstepCollisionLeg& leg = input.legs[foot];
            const float begin = turn.duration * .4f;
            const float end = turn.duration * .41f;
            std::array<FootstepInterval::Transform, 3> bounds;
            Check(BoundFootstepLeg(leg, begin, end, bounds), "Bound the mid-turn geometry of each leg");
            for (int sample = 0; sample <= 8; ++sample)
            {
                const float time = begin + (end - begin) * static_cast<float>(sample) / 8;
                std::array<PxTransform, 3> shapes;
                LegIKResult solved;
                Check(PredictFootstepLeg(leg, time, shapes, solved), "Sample both legs within the turn interval");
                for (size_t part = 0; part < shapes.size(); ++part)
                {
                    const PxVec3 local(.07f, -.11f, .19f);
                    const PxVec3 point = shapes[part].transform(local);
                    const FootstepInterval::Vector bound = bounds[part].position +
                        FootstepInterval::Rotate(bounds[part].rotation, FootstepInterval::Vector(local));
                    Check(point.x >= bound.x.lower && point.x <= bound.x.upper && point.y >= bound.y.lower &&
                        point.y <= bound.y.upper && point.z >= bound.z.lower && point.z <= bound.z.upper,
                        "The interval contains the turning leg shape including the rotating toe");
                }
            }
        }
        for (int index = 0; index <= 10; ++index)
        {
            const float time = turn.duration * static_cast<float>(index) / 10;
            LegIKResult planted;
            std::array<PxTransform, 3> shapes;
            Check(PredictFootstepLeg(input.legs[support], time, shapes, planted) && Near(planted.solvedSole.p, turn.legs[support].targetSole.p) &&
                std::abs(planted.solvedSole.q.dot(turn.legs[support].targetSole.q)) > .99999f, "Support sole stays planted throughout the turn");
            LegIKResult moving;
            Check(PredictFootstepLeg(input.legs[side], time, shapes, moving) &&
                Near(moving.solvedSole.p, SampleFootstepPosition(command, time)) &&
                std::abs(moving.solvedSole.q.dot(turn.SoleRotationAt(time))) > .99999f, "Moving sole follows the tested swing and toe rotation");
        }
        input.acceptsPose = [support](size_t foot, const LegIKResult&, float time) -> bool
        {
            return foot != support || time <= .1f;
        };
        const FootstepCollisionSnapshot blocked = PredictFootstepCollisions(input);
        Check(blocked.status != FootstepCollisionStatus::Clear && blocked.jointLimited, "A mid-turn support joint vetoes the entire path");
    }
}

void TestFootstepCollision()
{
    TestPelvisTurnTrajectory();
    TestFuturePathAndRetarget();
    TestAllPairsAndOffsets();
    TestBothMoving();
    TestTakeoffAndLowering();
    TestUnavailableAndReachLimit();
    TestIntervalsAndTracking();
    TestBoundsContainCurvedIK();
    TestAvoidanceAndContinuity();
    std::puts("PASS: FootstepCollision continuous bounds, tracking, primitives, avoidance and failure handling");
}
