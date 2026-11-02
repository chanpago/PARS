#include "../Model/StandingSupport.h"
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

    StandingSupportInput StandingPose()
    {
        StandingSupportInput input;
        input.enabled = true;
        input.deltaTime = .1f;
        input.revision = 1;
        input.completedStep = 1;
        input.pelvis = PxTransform(PxVec3(0, 1, 0));
        input.animatedPelvis = input.pelvis;
        for (size_t side = 0; side < input.feet.size(); ++side)
        {
            StandingSupportFoot& foot = input.feet[side];
            foot.available = foot.grounded = true;
            foot.sole = PxTransform(PxVec3(side == 0 ? -.1f : .1f, 0, 0));
            foot.animatedSole = foot.sole;
            foot.length = 1.1f;
        }
        return input;
    }

    void TestHeightAndSwing()
    {
        StandingSupport controller;
        const StandingSupportSettings settings;
        StandingSupportInput input = StandingPose();
        input.pelvis.p.y = .9f;
        Check(controller.Update(input, settings), "Standing height target is valid");
        const StandingSupportSnapshot raised = controller.GetSnapshot();
        Check(raised.targetHeight > input.pelvis.p.y && raised.targetHeight < 1,
            "A low pelvis requests a bounded rise");
        Check(raised.soleTargets[0].p.y < 0 && raised.soleTargets[1].p.y < 0,
            "Stance legs extend to raise the pelvis through floor contact");
        input = StandingPose();
        controller.Reset();
        input.stepping = true;
        input.stepDistance = .4f;
        input.feet[1].supporting = false;
        input.feet[1].grounded = false;
        input.feet[1].sole.p = PxVec3(.2f, .2f, .3f);
        Check(controller.Update(input, settings), "Single stance supports height control");
        const StandingSupportSnapshot lowered = controller.GetSnapshot();
        Check(lowered.targetHeight < 1 && lowered.soleTargets[0].p.y > 0,
            "A step requests stance-leg flexion and a lower pelvis");
        Check((lowered.soleTargets[1].p - input.feet[1].sole.p).magnitude() < 1e-6f &&
            lowered.contactLowering == 0, "Airborne swing target is preserved and is not a lost stance contact");
        controller.Reset();
        input = StandingPose();
        Check(controller.Update(input, settings), "Initialize a horizontal support reference");
        input.stepping = true;
        input.feet[1].supporting = false;
        input.feet[1].sole.p.z += .3f;
        ++input.completedStep;
        Check(controller.Update(input, settings) && controller.GetSnapshot().horizontalCorrection.magnitude() < 1e-6f,
            "Swing trajectory cannot pull the pelvis horizontal reference toward itself");
        input.stepping = false;
        input.feet[1].supporting = true;
        ++input.completedStep;
        Check(controller.Update(input, settings) && controller.GetSnapshot().horizontalCorrection.magnitude() < .02f,
            "Landing updates the horizontal reference continuously rather than jumping to the new foot midpoint");
    }

    void TestContactLossAndRecovery()
    {
        StandingSupport controller;
        const StandingSupportSettings settings;
        StandingSupportInput input = StandingPose();
        Check(controller.Update(input, settings), "Initialize both-foot stance");
        input.feet[0].grounded = false;
        for (unsigned step = 2; step < 100; ++step)
        {
            input.completedStep = step;
            Check(controller.Update(input, settings), "Missing contact does not stop the height controller");
        }
        const float low = controller.GetSnapshot().targetHeight;
        Check(low < .95f && low > .9f && !input.feet[0].grounded,
            "Persistent contact loss lowers within a bound without forging contact");
        Check((controller.GetSnapshot().soleTargets[0].p - input.feet[0].sole.p).magnitude() < 1e-6f,
            "A lost stance foot keeps reaching the floor instead of retracting with the lowered pelvis");
        Check(controller.Update(input, settings) && controller.GetSnapshot().targetHeight == low,
            "The same physics sample does not advance lowering twice");
        input.feet[0].grounded = true;
        ++input.completedStep;
        Check(controller.Update(input, settings) && controller.GetSnapshot().targetHeight > low &&
            controller.GetSnapshot().targetHeight <= low + settings.raisingSpeed * input.deltaTime + 1e-6f,
            "Recontact restores height at the upward rate limit");
    }

    void TestReachAndReset()
    {
        StandingSupport controller;
        const StandingSupportSettings settings;
        StandingSupportInput input = StandingPose();
        input.pelvis.p.x = .6f;
        for (unsigned step = 1; step < 30; ++step)
        {
            input.completedStep = step;
            Check(controller.Update(input, settings), "Reach-aware stance target remains valid");
        }
        const StandingSupportSnapshot reachable = controller.GetSnapshot();
        Check(reachable.reachLowering > .1f && !reachable.reachLimited,
            "Horizontal pelvis displacement requests reachable leg height");
        for (const StandingSupportFoot& foot : input.feet)
        {
            const PxVec3 pelvis(input.pelvis.p.x, reachable.targetHeight, input.pelvis.p.z);
            Check((pelvis - foot.sole.p).magnitude() <= foot.length - settings.reachReserve + 1e-5f,
                "Settled pelvis height fits both stance leg lengths");
        }
        input.pelvis.p.x = 2;
        ++input.completedStep;
        Check(controller.Update(input, settings) && controller.GetSnapshot().reachLimited,
            "An impossible horizontal span is reported instead of hidden");
        input.enabled = false;
        Check(!controller.Update(input, settings) && !controller.GetSnapshot().valid,
            "Disabling clears support state");
        input = StandingPose();
        input.deltaTime = std::numeric_limits<float>::quiet_NaN();
        Check(!controller.Update(input, settings), "Invalid timing cannot create height targets");
        input = StandingPose();
        Check(controller.Update(input, settings), "A fresh pose can initialize after invalid input");
        ++input.revision;
        input.pelvis.p.y = .8f;
        Check(controller.Update(input, settings) && controller.GetSnapshot().targetHeight < .85f,
            "A physics revision initializes from the new pelvis rather than stale height");
    }
}

void TestStandingSupport()
{
    TestHeightAndSwing();
    TestContactLossAndRecovery();
    TestReachAndReset();
    std::puts("PASS: standing height, swing isolation, contact loss, reach and reset");
}
