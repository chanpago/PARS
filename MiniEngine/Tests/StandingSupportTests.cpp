#include "../Model/StandingSupport.h"
#include "../Model/FootSupport.h"
#include "../Model/FootSupportGeometry.h"
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

    void TestCombinedFootContact()
    {
        PhysicsGroundContact foot;
        foot.position = PxVec3(0);
        foot.normal = PxVec3(0, 1, 0);
        foot.pointCount = 4;
        foot.normalImpulse = 8;
        foot.normalImpulseVector = PxVec3(0, 8, 0);
        PhysicsGroundContact toe;
        toe.position = PxVec3(0, 0, .3f);
        toe.normal = PxVec3(0, 1, 0);
        toe.pointCount = 2;
        toe.normalImpulse = 3;
        toe.normalImpulseVector = PxVec3(0, 3, 0);

        FootContactSample contact;
        contact.Update(1, true, {}, {});
        contact.Update(2, true, {}, toe);
        Check(contact.grounded && contact.landed && contact.landingCount == 1 && contact.lastLandingStep == 2 &&
                contact.pointCount == 2 && contact.normalImpulse == 3 && contact.support.IsValid() &&
                (contact.position - toe.position).magnitude() < 1e-6f && contact.IsToeOnlySupport() &&
                contact.parts[1].landed && contact.supportFractions[1] == 1,
            "Toe-only contact lands the whole foot and supplies its real support load");

        StandingSupport controller;
        StandingSupportInput input = StandingPose();
        input.feet[0].grounded = contact.grounded;
        input.feet[0].supportNormal = contact.normal;
        input.feet[1].supporting = false;
        Check(controller.Update(input, StandingSupportSettings()) && controller.GetSnapshot().contactLowering == 0,
            "A toe-supported stance does not keep seeking the floor");

        contact.Update(3, true, foot, toe);
        Check(contact.grounded && !contact.leftGround && !contact.landed && contact.pointCount == 6 &&
                contact.normalImpulse == 11 && (contact.position - PxVec3(0, 0, .3f * 3 / 11)).magnitude() < 1e-6f &&
                (contact.normal - PxVec3(0, 1, 0)).magnitude() < 1e-6f,
            "Foot and toe use their vertical loads for the support center without another landing");
        Check(contact.parts[0].landed && !contact.parts[1].landed && !contact.IsToeOnlySupport() &&
                std::abs(contact.supportFractions[0] - 8.0f / 11) < 1e-6f &&
                std::abs(contact.supportFractions[1] - 3.0f / 11) < 1e-6f &&
                contact.parts[0].ground.normalImpulse == 8 && contact.parts[1].ground.normalImpulse == 3,
            "Per-part landing, raw impulses and load fractions remain separate");
        contact.Update(4, true, foot, {});
        Check(contact.grounded && !contact.leftGround && !contact.landed && contact.landingCount == 1 &&
                contact.takeoffCount == 0 && contact.pointCount == 4 && contact.normalImpulse == 8 &&
                (contact.position - foot.position).magnitude() < 1e-6f,
            "Rolling from toe to foot preserves continuous support and the foot-only result");
        contact.Update(5, true, {}, toe);
        Check(contact.grounded && !contact.leftGround && !contact.landed && contact.landingCount == 1,
            "Rolling from foot back to toe also preserves continuous support");

        contact.Update(6, true, {}, {});
        Check(!contact.grounded && contact.leftGround && contact.takeoffCount == 1 && contact.lastTakeoffStep == 6 &&
                contact.pointCount == 0 && contact.normalImpulse == 0 && contact.support.IsValid() &&
                contact.normalImpulseVector.isZero() && contact.parts[1].ground.pointCount == 0 &&
                contact.IsToeOnlySupport() && contact.supportFractions[1] == 1,
            "Takeoff requires both bodies to leave the floor and grace never fabricates an impulse");
        contact.Update(8, true, {}, {});
        Check(!contact.grounded && !contact.leftGround && contact.support.IsValid(),
            "The last real toe contact retains the existing three-step support grace");
        contact.Update(9, true, {}, {});
        Check(!contact.grounded && !contact.support.IsValid() && !contact.IsToeOnlySupport() &&
                contact.supportFractions[1] == 0, "Toe support expires after the same grace interval");
        contact.Update(10, true, {}, toe);
        Check(contact.grounded && contact.landed && contact.landingCount == 2 && contact.lastLandingStep == 10,
            "A real toe recontact after flight records one new landing");
        contact.Update(11, false, foot, toe);
        Check(!contact.hasSample && !contact.grounded && !contact.leftGround && !contact.landed &&
                contact.pointCount == 0 && contact.normalImpulse == 0 && !contact.support.IsValid(),
            "An unavailable foot cannot reuse contact data from either body");

        foot.normalImpulse = 0;
        foot.normalImpulseVector = PxVec3(0);
        toe.normalImpulse = 0;
        toe.normalImpulseVector = PxVec3(0);
        contact.Update(12, true, foot, toe);
        Check(contact.grounded && contact.normalImpulse == 0 &&
                (contact.position - PxVec3(0, 0, .1f)).magnitude() < 1e-6f &&
                contact.IsPartSupporting(0) && contact.IsPartSupporting(1),
            "Real zero-load contacts use point counts without fabricating a force");
        foot.normal = PxVec3(.6f, .8f, 0);
        foot.normalImpulseVector = foot.normal * 10;
        foot.normalImpulse = 10;
        toe.normalImpulseVector = toe.normal * 2;
        toe.normalImpulse = 2;
        contact.Update(13, true, foot, toe);
        Check(std::abs(contact.normalImpulseVector.magnitude() * contact.normal.y - 10) < 1e-5f &&
                std::abs(contact.supportFractions[1] - .2f) < 1e-6f &&
                (contact.position - PxVec3(0, 0, .06f)).magnitude() < 1e-6f,
            "Different contact normals preserve the exact summed vertical load");
    }

    void TestToeGeometryAndHeelPivot()
    {
        PhysicsBody toe;
        toe.shape = PhysicsShapeType::Capsule;
        toe.radius = .02f;
        toe.halfLength = .017f;
        const PxTransform body(PxVec3(0, .02f, .2f));
        const PxTransform sole(PxVec3(0, 0, .2f));
        const std::array<PxVec3, 8> outline = BuildFootBodyOutline(toe, body, sole);
        Check(std::abs(outline[0].x - .037f) < 1e-6f && std::abs(outline[4].x + .037f) < 1e-6f &&
                std::abs(outline[2].z - .22f) < 1e-6f && std::abs(outline[6].z - .18f) < 1e-6f,
            "Toe projection follows the capsule radius and half length");
        const PxTransform world(PxVec3(2, 1, -3), PxQuat(.7f, PxVec3(0, 1, 0)) * PxQuat(.3f, PxVec3(1, 0, 0)));
        const std::array<PxVec3, 8> moved = BuildFootBodyOutline(toe, world * body, world * sole);
        for (size_t point = 0; point < outline.size(); ++point)
        {
            Check((moved[point] - world.transform(outline[point])).magnitude() < 1e-5f,
                "A rotated toe outline does not jump between capsule endpoints from rounding error");
        }
        std::vector<PxVec3> points{PxVec3(-.05f, 0, -.1f), PxVec3(.05f, 0, -.1f),
            PxVec3(.05f, 0, .1f), PxVec3(-.05f, 0, .1f)};
        Check(!IsInsideFootSupportHull(BuildFootSupportHull(points), PxVec3(0, 0, .2f)),
            "The original foot-only hull does not contain the toe region");
        points.insert(points.end(), outline.begin(), outline.end());
        Check(IsInsideFootSupportHull(BuildFootSupportHull(points), PxVec3(0, 0, .2f)),
            "Adding the toe projection extends the same foot support hull");

        const PxTransform foot(PxVec3(0, .1f, 0), PxQuat(-.4f, PxVec3(1, 0, 0)));
        const PxTransform displacedToe = world * sole;
        const PxTransform displacedFoot = world * foot;
        const PxTransform target = MakeToeSupportedFootTarget(displacedFoot, displacedToe, sole);
        Check((target.p - foot.p).magnitude() < 1e-5f && std::abs(target.q.dot(foot.q)) > .999999f &&
                target.q.rotate(PxVec3(0, 1, 0)).y < .95f,
            "The toe anchor restores the foot position while preserving heel lift");
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

    void TestSupportFootTilt()
    {
        StandingSupport controller;
        StandingSupportSettings settings;
        StandingSupportInput input = StandingPose();
        const PxVec3 pitchAxis(1, 0, 0);
        input.feet[0].actualSole.q = PxQuat(PxPi / 18.0f, pitchAxis);
        input.feet[0].sole.q = PxQuat(PxPi / 9.0f, pitchAxis);
        input.feet[1].supporting = false;
        input.feet[1].actualSole.q = input.feet[0].actualSole.q;
        input.feet[1].sole.q = input.feet[0].sole.q;
        Check(controller.Update(input, settings), "Tilted support has a valid target");
        const PxVec3 limitedUp = controller.GetSnapshot().soleTargets[0].q.rotate(PxVec3(0, 1, 0));
        Check(controller.GetSnapshot().footTiltLimited[0] &&
                std::abs(std::acos(PxClamp(limitedUp.y, -1.0f, 1.0f)) - PxPi / 36.0f) < 1e-5f,
            "A tilted stance foot caps an excessive target at five degrees");
        Check(!controller.GetSnapshot().footTiltLimited[1] &&
                std::abs(controller.GetSnapshot().soleTargets[1].q.dot(input.feet[1].sole.q)) > .999999f,
            "The same tilt never alters the swing-foot orientation");
        ++input.completedStep;
        input.feet[0].allowHeelLift = true;
        Check(controller.Update(input, settings) && !controller.GetSnapshot().footTiltLimited[0] &&
                std::abs(controller.GetSnapshot().soleTargets[0].q.dot(input.feet[0].sole.q)) > .999999f,
            "Toe-only support permits fore-aft heel lift");
        ++input.completedStep;
        input.feet[0].actualSole.q = PxQuat(PxPi / 18.0f, PxVec3(0, 0, 1));
        input.feet[0].sole.q = PxQuat(PxPi / 9.0f, PxVec3(0, 0, 1));
        Check(controller.Update(input, settings) && controller.GetSnapshot().footTiltLimited[0],
            "Toe-only support still limits sideways collapse");
        input.feet[0].allowHeelLift = false;
        input.feet[0].actualSole.q = PxQuat(PxPi / 18.0f, pitchAxis);
        ++input.completedStep;
        input.feet[0].sole.q = PxQuat(-PxPi / 9.0f, pitchAxis);
        Check(controller.Update(input, settings) && !controller.GetSnapshot().footTiltLimited[0],
            "A restoring target in the opposite direction remains unchanged");
        ++input.completedStep;
        input.feet[0].sole.q = PxQuat(PxPi, PxVec3(0, 1, 0));
        Check(controller.Update(input, settings) && !controller.GetSnapshot().footTiltLimited[0],
            "Yaw alone is not treated as sole tilt");
        ++input.completedStep;
        const PxQuat slope(PxPi / 12.0f, PxVec3(0, 0, 1));
        input.feet[0].supportNormal = slope.rotate(PxVec3(0, 1, 0));
        input.feet[0].actualSole.q = slope;
        input.feet[0].sole.q = slope;
        Check(controller.Update(input, settings) && !controller.GetSnapshot().footTiltLimited[0],
            "A foot flat on a slope is not flattened toward world up");
        ++input.completedStep;
        input.feet[0].supportNormal = PxVec3(0, 1, 0);
        input.feet[0].actualSole.q = PxQuat(3.5f * PxPi / 180.0f, pitchAxis);
        input.feet[0].sole.q = PxQuat(PxPi / 9.0f, pitchAxis);
        Check(controller.Update(input, settings), "Intermediate tilt remains valid");
        const PxVec3 blendedUp = controller.GetSnapshot().soleTargets[0].q.rotate(PxVec3(0, 1, 0));
        Check(std::abs(std::acos(PxClamp(blendedUp.y, -1.0f, 1.0f)) - 12.5f * PxPi / 180.0f) < 1e-5f,
            "The limit blends continuously between its onset and full strength");
        ++input.completedStep;
        input.feet[0].grounded = false;
        Check(controller.Update(input, settings) && !controller.GetSnapshot().footTiltLimited[0],
            "No tilt restriction remains after support contact expires");
        ++input.completedStep;
        input.feet[0].contactGraceActive = true;
        Check(controller.Update(input, settings) && controller.GetSnapshot().footTiltLimited[0],
            "Brief support contact grace preserves the tilt restriction");
        ++input.completedStep;
        settings.limitFootTilt = false;
        Check(controller.Update(input, settings) && !controller.GetSnapshot().footTiltLimited[0] &&
                std::abs(controller.GetSnapshot().soleTargets[0].q.dot(input.feet[0].sole.q)) > .999999f,
            "Disabling the limiter restores the original rotation target");
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
    TestCombinedFootContact();
    TestToeGeometryAndHeelPivot();
    TestSupportContactGrace();
    TestGracePreservesStandingCorrection();
    TestDisplayContactGrace();
    TestSupportFootTilt();
    std::puts("PASS: standing height, swing isolation, contact loss, reach and reset");
}
