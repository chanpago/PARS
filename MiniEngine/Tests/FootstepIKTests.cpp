#include "../Model/FootstepIK.h"
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

    bool Near(PxVec3 a, PxVec3 b, float tolerance = 8e-5f)
    {
        return (a - b).magnitude() < tolerance;
    }

    bool Near(PxTransform a, PxTransform b)
    {
        return Near(a.p, b.p) && PxAbs(a.q.dot(b.q)) > 1 - 1e-5f;
    }

    TwoBoneIKInput BentLeg()
    {
        TwoBoneIKInput input;
        const float bend = std::atan2(.2f, .45f);
        input.hip = PxTransform(PxVec3(0, 1, 0), PxQuat(-bend, PxVec3(1, 0, 0)));
        input.knee = PxTransform(PxVec3(0, .55f, .2f), PxQuat(bend, PxVec3(1, 0, 0)));
        input.ankle = PxTransform(PxVec3(0, .1f, 0));
        input.thighLength = input.calfLength = std::sqrt(.45f * .45f + .2f * .2f);
        input.soleInBone = PxTransform(PxVec3(.06f, -.1f, .12f), PxQuat(.17f, PxVec3(1, 0, 0)));
        input.targetSole = input.ankle * input.soleInBone;
        input.targetSole.p += PxVec3(.2f, .18f, .25f);
        input.referenceBend = PxVec3(0, 0, 1);
        return input;
    }

    void CheckSegments(const TwoBoneIKInput& input, const LegIKResult& result)
    {
        Check(result.valid, "The leg solution is finite");
        Check(PxAbs((result.knee.p - result.hip.p).magnitude() - input.thighLength) < 8e-5f &&
            PxAbs((result.ankle.p - result.knee.p).magnitude() - input.calfLength) < 8e-5f,
            "IK preserves both reference bone lengths");
        const PxVec3 upperLocal = input.hip.q.rotateInv((input.knee.p - input.hip.p).getNormalized()) * input.thighLength;
        const PxVec3 lowerLocal = input.knee.q.rotateInv((input.ankle.p - input.knee.p).getNormalized()) * input.calfLength;
        Check(Near(result.hip.transform(upperLocal), result.knee.p) &&
            Near(result.knee.transform(lowerLocal), result.ankle.p),
            "Forward kinematics of the output rotations reproduces the solved knee and ankle");
        Check(Near(result.ankle * input.soleInBone, result.solvedSole),
            "The solved ankle recomposes the foot sole including its PA offset and rotation");
    }

    void TestGeometryAndOffsets()
    {
        auto input = BentLeg();
        auto result = SolveTwoBoneIK(input);
        CheckSegments(input, result);
        Check(!result.reachLimited && Near(result.solvedSole, input.targetSole) && result.reachError < 1e-5f,
            "A reachable sole position and orientation are reached together");
        Check(!Near(result.requestedAnkle, input.targetSole.p) &&
            Near(result.requestedAnkle, (input.targetSole * input.soleInBone.getInverse()).p),
            "The foot sole target is not mistaken for the ankle position");
        Check(result.hintSource == KneeHintSource::Animation && result.bendDirection.dot(PxVec3(0, 0, 1)) > 0,
            "A bent animation supplies the knee direction");

        for (int index = 0; index < 24; ++index)
        {
            input = BentLeg();
            const float angle = float(index) * PxTwoPi / 24;
            const PxTransform ankle(PxVec3(std::sin(angle) * .3f, .35f, std::cos(angle) * .3f),
                PxQuat(angle, PxVec3(0, 1, 0)) * PxQuat(.3f, PxVec3(0, 0, 1)));
            input.soleInBone = PxTransform(PxVec3(.04f, -.08f, .14f),
                PxQuat(-.23f, PxVec3(1, 0, 0)) * PxQuat(.31f, PxVec3(0, 1, 0)));
            input.targetSole = ankle * input.soleInBone;
            result = SolveTwoBoneIK(input);
            CheckSegments(input, result);
            Check(Near(result.ankle, ankle), "Changing foot orientation rotates the ankle offset correctly in every direction");
            const PxVec3 sourceNormal = (input.knee.p - input.hip.p).cross(input.ankle.p - input.knee.p).getNormalized();
            const PxVec3 upperNormal = result.hip.q.rotate(input.hip.q.rotateInv(sourceNormal));
            const PxVec3 lowerNormal = result.knee.q.rotate(input.knee.q.rotateInv(sourceNormal));
            Check(Near(upperNormal, lowerNormal), "Changing the bend plane does not add relative knee twist");
        }

        input = BentLeg();
        const auto original = SolveTwoBoneIK(input);
        const PxTransform world(PxVec3(3, -2, 1), PxQuat(.8f, PxVec3(0, 1, 0)) * PxQuat(.4f, PxVec3(1, 0, 0)));
        input.hip = world * input.hip;
        input.knee = world * input.knee;
        input.ankle = world * input.ankle;
        input.targetSole = world * input.targetSole;
        input.referenceBend = world.q.rotate(input.referenceBend);
        result = SolveTwoBoneIK(input);
        CheckSegments(input, result);
        Check(Near(result.hip, world * original.hip) && Near(result.knee, world * original.knee) &&
            Near(result.ankle, world * original.ankle),
            "The IK solution is unchanged by translating and rotating the whole coordinate frame");
    }

    void TestReachAndHints()
    {
        auto input = BentLeg();
        input.targetSole.p = PxVec3(0, -5, 3);
        auto result = SolveTwoBoneIK(input);
        CheckSegments(input, result);
        Check(result.reachLimited && result.reachError > 4 && Near(result.requestedSole, input.targetSole),
            "An unreachable goal retains the requested target and reports geometric reach error");
        input.thighLength = .6f;
        input.calfLength = .3f;
        input.targetSole = PxTransform(input.hip.p) * input.soleInBone;
        result = SolveTwoBoneIK(input);
        CheckSegments(input, result);
        Check(result.reachLimited && result.reachError > .299f,
            "A target at the hip stays outside the unequal-leg folding singularity");

        input = {};
        input.hip = PxTransform(PxVec3(0, 1, 0));
        input.knee = PxTransform(PxVec3(0, .5f, 0));
        input.ankle = PxTransform(PxVec3(0));
        input.thighLength = input.calfLength = .5f;
        input.targetSole = PxTransform(PxVec3(0, .3f, .2f));
        input.referenceBend = PxVec3(0, 0, 1);
        result = SolveTwoBoneIK(input);
        CheckSegments(input, result);
        Check(result.hintSource == KneeHintSource::Reference && result.knee.p.z > 0,
            "A straight animation uses the reference bending direction");
        input.hasPreviousBend = true;
        input.previousBend = PxVec3(1, 0, 0);
        result = SolveTwoBoneIK(input);
        CheckSegments(input, result);
        Check(result.hintSource == KneeHintSource::Previous && result.knee.p.x > 0,
            "A previous valid direction takes priority over the reference at a straight knee");
        input.hasPreviousBend = false;
        input.referenceBend = PxVec3(0);
        result = SolveTwoBoneIK(input);
        CheckSegments(input, result);
        Check(result.hintSource == KneeHintSource::Fallback, "Missing hints use a finite perpendicular fallback");
        input.targetSole = PxTransform(PxVec3(0, 1.8f, 0));
        result = SolveTwoBoneIK(input);
        CheckSegments(input, result);
        Check(Near(result.solvedSole, input.targetSole), "A reversed target direction remains finite");
        for (float length : {0.0f, -1.0f, std::numeric_limits<float>::quiet_NaN()})
        {
            input.thighLength = length;
            Check(!SolveTwoBoneIK(input).valid, "Invalid bone lengths are rejected");
        }
        input = BentLeg();
        input.targetSole.p.x = std::numeric_limits<float>::infinity();
        Check(!SolveTwoBoneIK(input).valid, "Nonfinite foot targets do not become joint goals");
    }

    void TestSoleOrientation()
    {
        PxTransform target(PxIdentity);
        const PxTransform animated(PxVec3(0), PxQuat(.7f, PxVec3(0, 1, 0)));
        const PxVec3 normal = PxVec3(.2f, 1, .3f).getNormalized();
        Check(MakeFootSoleTarget(PxVec3(1, 2, 3), normal, animated, target), "A sloped landing plane produces a sole orientation");
        Check(Near(target.p, PxVec3(1, 2, 3)) && Near(target.q.rotate(PxVec3(0, 1, 0)), normal),
            "The sole keeps its requested position and aligns its up direction to the plane");
        PxVec3 forward = animated.q.rotate(PxVec3(0, 0, 1));
        forward = (forward - normal * forward.dot(normal)).getNormalized();
        Check(Near(target.q.rotate(PxVec3(0, 0, 1)), forward), "Foot yaw follows the animation projected onto the plane");
        Check(MakeFootSoleTarget(PxVec3(0), PxVec3(0, 1, 0),
            PxTransform(PxVec3(0), PxQuat(-PxHalfPi, PxVec3(1, 0, 0))), target) && target.isSane(),
            "An animated forward direction parallel to the normal uses its side direction");
        Check(!MakeFootSoleTarget(PxVec3(0), PxVec3(0), animated, target), "A missing ground normal is rejected");
    }

    struct PoseFixture
    {
        std::vector<int32_t> parents{-1, 0, 1, 2, 3, 0, 5, 6, 7, 0};
        std::vector<PxTransform> pose;
        FootstepIKInput input;

        PoseFixture()
        {
            const auto leg = BentLeg();
            pose.resize(parents.size(), PxTransform(PxIdentity));
            pose[0] = PxTransform(PxVec3(0, 1.1f, 0));
            pose[9] = PxTransform(PxVec3(0, 1.4f, 0));
            input.enabled = true;
            input.stepFoot = 1;
            input.revision = 1;
            const PxTransform physical(PxVec3(.2f, 1.07f, .15f),
                PxQuat(.3f, PxVec3(0, 1, 0)) * PxQuat(.1f, PxVec3(0, 0, 1)));
            const auto toPhysical = physical * pose[0].getInverse();
            for (size_t side = 0; side < 2; ++side)
            {
                const int hip = side == 0 ? 1 : 5;
                const PxTransform offset(PxVec3(side == 0 ? -.12f : .12f, 0, 0));
                pose[hip] = offset * leg.hip;
                pose[hip + 1] = offset * leg.knee;
                pose[hip + 2] = offset * leg.ankle;
                pose[hip + 3] = pose[hip + 2] * PxTransform(PxVec3(0, 0, .2f), PxQuat(.1f, PxVec3(1, 0, 0)));
                auto& setup = input.legs[side];
                setup.available = true;
                setup.anchorNode = 0;
                setup.hipNode = hip;
                setup.kneeNode = hip + 1;
                setup.ankleNode = hip + 2;
                setup.thighLength = leg.thighLength;
                setup.calfLength = leg.calfLength;
                setup.referenceBendInHip = leg.hip.q.rotateInv(PxVec3(0, 0, 1));
                setup.actualAnchor = physical;
                setup.soleInBone = leg.soleInBone;
                setup.targetSole = toPhysical * pose[hip + 2] * leg.soleInBone;
            }
            input.legs[1].targetSole.p += PxVec3(.15f, .12f, .2f);
        }
    };

    void TestPoseAndPhysicalAnchor()
    {
        PoseFixture fixture;
        FootstepIK ik;
        const auto original = fixture.pose;
        Check(ik.Apply(fixture.input, fixture.parents, fixture.pose), "Both legs produce corrected joint targets");
        Check(ik.GetSnapshot().status == FootstepIKStatus::Applied && ik.GetSnapshot().correctedNodes.size() == 8,
            "Only both leg chains and their toe descendants are marked for correction");
        Check(Near(original[0], fixture.pose[0]) && Near(original[9], fixture.pose[9]),
            "The pelvis and upper-body animation goals remain unchanged");
        for (size_t side = 0; side < 2; ++side)
        {
            const auto& leg = fixture.input.legs[side];
            const PxTransform physical = leg.actualAnchor * fixture.pose[leg.anchorNode].getInverse();
            const PxTransform sole = physical * fixture.pose[leg.ankleNode] * leg.soleInBone;
            Check(Near(sole, leg.targetSole), "Joint-relative targets reach the world sole from the actual translated and tilted pelvis");
            Check(Near(fixture.pose[leg.hipNode].p, original[leg.hipNode].p), "The hip attachment position is not moved");
            const int toe = leg.ankleNode + 1;
            Check(Near(fixture.pose[leg.ankleNode].getInverse() * fixture.pose[toe],
                original[leg.ankleNode].getInverse() * original[toe]), "Toe local position and rotation survive the IK override");
        }
        Check(ik.GetSnapshot().legs[0].reachError < 1e-5f, "The support leg retains its current contacted sole goal");

        fixture.pose = original;
        fixture.input.enabled = false;
        Check(!ik.Apply(fixture.input, fixture.parents, fixture.pose) && ik.GetSnapshot().correctedNodes.empty() &&
            ik.GetSnapshot().status == FootstepIKStatus::Inactive && Near(fixture.pose[7], original[7]),
            "Disabling after landing, B exit or contact loss leaves the original animation pose");
        fixture.input.enabled = true;
        fixture.input.legs[1].available = false;
        Check(!ik.Apply(fixture.input, fixture.parents, fixture.pose) &&
            ik.GetSnapshot().status == FootstepIKStatus::InvalidLeg && Near(fixture.pose[3], original[3]),
            "A missing leg does not partially apply the other leg");
        fixture.input.legs[1].available = true;
        fixture.parents[2] = 9;
        Check(!ik.Apply(fixture.input, fixture.parents, fixture.pose) && ik.GetSnapshot().status == FootstepIKStatus::InvalidPose,
            "An invalid hierarchy is rejected before traversing ancestors");
        fixture.parents[2] = 1;
        fixture.input.legs[1].targetSole.p.x = std::numeric_limits<float>::quiet_NaN();
        Check(!ik.Apply(fixture.input, fixture.parents, fixture.pose) &&
            ik.GetSnapshot().status == FootstepIKStatus::InvalidTarget && Near(fixture.pose[1], original[1]),
            "A nonfinite second goal does not leak a first-leg correction");
    }

    void TestHintHistoryReset()
    {
        PoseFixture fixture;
        FootstepIK ik;
        const auto original = fixture.pose;
        Check(ik.Apply(fixture.input, fixture.parents, fixture.pose), "Prepare a previous valid bending direction");
        fixture.pose = original;
        for (const auto& leg : fixture.input.legs)
        {
            fixture.pose[leg.kneeNode].p = (fixture.pose[leg.hipNode].p + fixture.pose[leg.ankleNode].p) * .5f;
        }
        const auto straight = fixture.pose;
        Check(ik.Apply(fixture.input, fixture.parents, fixture.pose) &&
            ik.GetSnapshot().legs[1].hintSource == KneeHintSource::Previous,
            "The next straight animation sample reuses the preceding valid knee direction");
        fixture.pose = straight;
        ++fixture.input.revision;
        Check(ik.Apply(fixture.input, fixture.parents, fixture.pose) &&
            ik.GetSnapshot().legs[1].hintSource == KneeHintSource::Reference,
            "PA and physics revision changes discard the previous knee hint");
    }
}

void TestFootstepIK()
{
    TestGeometryAndOffsets();
    TestReachAndHints();
    TestSoleOrientation();
    TestPoseAndPhysicalAnchor();
    TestHintHistoryReset();
    std::printf("PASS: %u footstep IK checks (no PhysX simulation)\n", checks);
}
