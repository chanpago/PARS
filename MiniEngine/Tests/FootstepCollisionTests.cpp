#include "../Model/FootstepCollision.h"
#include <cmath>
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
        Check(result.status == FootstepCollisionStatus::Sampled && result.sampleCount == 6 &&
            result.queryCount == 54 && result.overlapPairs == 0,
            "An unobstructed swing checks six poses and nine primitive pairs per pose");
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
                Check(result.status == FootstepCollisionStatus::Sampled &&
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
        Check(moving.status == FootstepCollisionStatus::Sampled && moving.queryCount == 54 && moving.overlapPairs == 0,
            "Two parallel moving legs are compared at the same future time with only 54 queries");
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
        Check(lowering.status == FootstepCollisionStatus::Sampled && lowering.samples[5].timeAhead > 0,
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
        Check(result.status == FootstepCollisionStatus::Sampled && result.reachLimited,
            "Unreachable targets explicitly report that the solved leg was reach-limited");
    }
}

void TestFootstepCollision()
{
    TestFuturePathAndRetarget();
    TestAllPairsAndOffsets();
    TestBothMoving();
    TestTakeoffAndLowering();
    TestUnavailableAndReachLimit();
    std::puts("PASS: FootstepCollision sampling, retargeting, primitive pairs, moving legs, takeoff and validation");
}
