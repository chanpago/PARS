#include "../Model/StandingSupport.h"
#include "../Model/FootSupport.h"
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

    void TestSupportContactGrace()
    {
        FootSupportContact contact;
        const PxVec3 position(.1f, 0, .2f);
        const PxVec3 normal(0, 1, 0);
        contact.Update(100, true, false, PxVec3(0), PxVec3(0));
        Check(!contact.IsValid(), "Grace cannot create support before a real contact");
        contact.Update(100, true, true, position, normal);
        for (uint64_t step = 101; step <= 103; ++step)
        {
            contact.Update(step, true, false, PxVec3(0), PxVec3(0));
            contact.Update(step, true, false, PxVec3(0), PxVec3(0));
            Check(contact.IsValid() && contact.GetLastContactStep() == 100 &&
                    (contact.GetPosition() - position).magnitude() < 1e-6f &&
                    (contact.GetNormal() - normal).magnitude() < 1e-6f,
                "Three missing physics steps retain the support plane without duplicate-query aging");
        }
        contact.Update(104, true, false, PxVec3(0), PxVec3(0));
        Check(!contact.IsValid(), "The fourth missing step expires support");
        contact.Update(105, true, true, position, normal);
        contact.Update(108, true, false, PxVec3(0), PxVec3(0));
        Check(contact.IsValid() && contact.GetLastContactStep() == 105,
            "A real recontact refreshes the three-step deadline");
        contact.Update(110, true, false, PxVec3(0), PxVec3(0));
        Check(!contact.IsValid(), "Skipped physics steps still consume the grace interval");
        contact.Update(111, true, true, position, normal);
        contact.Update(112, false, false, PxVec3(0), PxVec3(0));
        Check(!contact.IsValid(), "An unavailable contact sample invalidates support immediately");
        contact.Update(113, true, true, position, normal);
        contact.Update(114, true, true, position, PxVec3(1, 0, 0));
        Check(!contact.IsValid(), "A non-supporting contact normal cannot retain the old floor");
        contact.Update(115, true, true, position, normal);
        contact.Update(1, true, false, PxVec3(0), PxVec3(0));
        Check(!contact.IsValid(), "A rewound clock cannot reuse the previous support plane");
        contact.Update(116, true, true, position, normal);
        contact.Reset();
        Check(!contact.IsValid(), "Reset removes the retained support plane");
    }

    void TestGracePreservesStandingCorrection()
    {
        StandingSupport held;
        StandingSupport continuous;
        FootSupportContact contact;
        const StandingSupportSettings settings;
        StandingSupportInput input = StandingPose();
        input.deltaTime = 1.0f / 60.0f;
        input.stepping = true;
        input.stepDistance = .4f;
        input.feet[1].supporting = false;
        input.feet[1].grounded = false;
        contact.Update(1, true, true, input.feet[0].sole.p, PxVec3(0, 1, 0));
        Check(held.Update(input, settings) && continuous.Update(input, settings),
            "Initialize the same single-support correction");
        input.feet[0].grounded = false;
        for (uint64_t step = 2; step <= 4; ++step)
        {
            input.completedStep = step;
            contact.Update(step, true, false, PxVec3(0), PxVec3(0));
            input.feet[0].contactGraceActive = contact.IsValid();
            StandingSupportInput reference = input;
            reference.feet[0].grounded = true;
            reference.feet[0].contactGraceActive = false;
            Check(held.Update(input, settings) && continuous.Update(reference, settings),
                "Temporary loss keeps a valid standing correction");
            Check(!input.feet[0].grounded && held.GetSnapshot().contactLowering == 0 &&
                    (held.GetSnapshot().soleTargets[0].p - continuous.GetSnapshot().soleTargets[0].p).magnitude() < 1e-6f,
                "Grace preserves the correction without changing the raw contact result");
        }
        input.completedStep = 5;
        contact.Update(5, true, false, PxVec3(0), PxVec3(0));
        input.feet[0].contactGraceActive = contact.IsValid();
        Check(held.Update(input, settings) && held.GetSnapshot().contactLowering > 0 &&
                (held.GetSnapshot().soleTargets[0].p - input.feet[0].sole.p).magnitude() < 1e-6f,
            "Persistent loss resumes floor seeking after the third missing step");
        Check((held.GetSnapshot().soleTargets[1].p - input.feet[1].sole.p).magnitude() < 1e-6f,
            "Support grace never changes the swing-foot target");
    }

    void TestDisplayContactGrace()
    {
        FootSupportSnapshot sample;
        sample.footstep.phase = FootstepPhase::AwaitingTakeoff;
        sample.footstep.supportFoot = 0;
        sample.footstep.stepFoot = 1;
        sample.feet[0].available = true;
        FootContactSample& contact = sample.feet[0].contact;
        contact.hasSample = true;
        contact.grounded = true;
        contact.normal = PxVec3(0, 1, 0);
        contact.support.Update(10, contact.hasSample, contact.grounded, contact.position, contact.normal);
        Check(sample.GetDisplayRoles().IsValid(), "A real stance contact enables role colors");
        contact.grounded = false;
        contact.normal = PxVec3(0);
        for (uint64_t step = 11; step <= 13; ++step)
        {
            contact.support.Update(step, contact.hasSample, contact.grounded, contact.position, contact.normal);
            Check(sample.GetDisplayRoles().supportFoot == 0 && sample.GetDisplayRoles().movableFoot == 1,
                "Role colors retain the committed feet during three missing contact steps");
        }
        sample.footstep.phase = FootstepPhase::Aborted;
        Check(!sample.GetDisplayRoles().IsValid(), "Aborting stepping still hides roles during the grace interval");
        sample.footstep.phase = FootstepPhase::Airborne;
        contact.support.Update(14, contact.hasSample, contact.grounded, contact.position, contact.normal);
        Check(!sample.GetDisplayRoles().IsValid(), "Role colors hide after support grace expires");
    }
}

void TestStandingSupport()
{
    TestHeightAndSwing();
    TestContactLossAndRecovery();
    TestReachAndReset();
    TestSupportContactGrace();
    TestGracePreservesStandingCorrection();
    TestDisplayContactGrace();
    std::puts("PASS: standing height, swing isolation, contact loss, reach and reset");
}
