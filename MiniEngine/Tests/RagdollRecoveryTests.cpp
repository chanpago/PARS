#include "../Model/RagdollRecovery.h"
#include "../Model/Animation.h"
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>

using namespace PARS;
using namespace Math;

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

    bool Near(const Matrix4& a, const Matrix4& b)
    {
        return float(Length(Vector3(a.GetX()) - Vector3(b.GetX()))) < 1e-5f &&
            float(Length(Vector3(a.GetY()) - Vector3(b.GetY()))) < 1e-5f &&
            float(Length(Vector3(a.GetZ()) - Vector3(b.GetZ()))) < 1e-5f &&
            float(Length(Vector3(a.GetW()) - Vector3(b.GetW()))) < 1e-5f;
    }

    RecoveryInput Balanced()
    {
        RecoveryInput input;
        input.enabled = input.bothGrounded = input.hasReference = true;
        input.insideReference = input.predictionInsideReference = true;
        input.insideSupport = input.predictionInsideSupport = true;
        input.deltaTime = 1.0f / 60.0f;
        input.completedStep = 10;
        input.revision = 1;
        return input;
    }

    void TestTransitions()
    {
        RagdollRecovery control;
        RecoverySettings settings;
        auto input = Balanced();
        control.Update(input, settings);
        Check(control.GetSnapshot().state == RecoveryState::Animation && !control.GetSnapshot().blending,
            "Balanced idle starts in Animation without a transition");
        ++input.completedStep;
        input.impactVelocity = settings.impactVelocity * .9f;
        control.Update(input, settings);
        Check(control.GetSnapshot().recoveryCount == 0, "Subthreshold contact impulses do not enter recovery");
        ++input.completedStep;
        input.impactVelocity = settings.impactVelocity;
        control.Update(input, settings);
        Check(control.GetSnapshot().state == RecoveryState::Recovery && control.GetSnapshot().enteredRecovery &&
            control.GetSnapshot().holdingTargets && control.GetSnapshot().recoveryCount == 1,
            "An external impact enters Recovery before a footstep is needed");
        input.impactVelocity = 0;
        input.bothGrounded = false;
        ++input.completedStep;
        control.Update(input, settings);
        Check(control.GetSnapshot().holdingTargets && control.GetSnapshot().reason == RecoveryReason::WaitingContact,
            "Losing contact keeps recovery targets instead of falling back to Idle");
        input.bothGrounded = true;
        input.stepping = true;
        ++input.completedStep;
        control.Update(input, settings);
        Check(control.GetSnapshot().reason == RecoveryReason::Stepping && control.GetSnapshot().stableTime == 0,
            "A foot still in flight prevents animation return even with balanced COM");
        input.stepping = false;
        input.predictionInsideReference = false;
        ++input.completedStep;
        control.Update(input, settings);
        Check(control.GetSnapshot().reason == RecoveryReason::OutsideReference,
            "Current COM alone cannot authorize return when predicted COM leaves the reference");
        input.predictionInsideReference = true;
        input.horizontalSpeed = settings.maximumReturnSpeed + .01f;
        ++input.completedStep;
        control.Update(input, settings);
        Check(control.GetSnapshot().reason == RecoveryReason::Moving, "Return also requires COM speed to settle");
        input.horizontalSpeed = 0;
        for (int i = 0; i < 6; ++i)
        {
            ++input.completedStep;
            control.Update(input, settings);
        }
        Check(Near(control.GetSnapshot().stableTime, .1f), "Continuous stability accumulates fixed-step time");
        control.Update(input, settings);
        Check(Near(control.GetSnapshot().stableTime, .1f), "Reading the same completed step cannot advance the timer");
        input.bothGrounded = false;
        ++input.completedStep;
        control.Update(input, settings);
        Check(control.GetSnapshot().stableTime == 0 && control.GetSnapshot().holdingTargets,
            "Contact flicker resets stability time but retains Recovery ownership");
        input.bothGrounded = true;
        for (int i = 0; i < 11; ++i)
        {
            ++input.completedStep;
            control.Update(input, settings);
            Check(control.GetSnapshot().state == RecoveryState::Recovery, "Return waits the entire stable duration");
        }
        ++input.completedStep;
        control.Update(input, settings);
        Check(control.GetSnapshot().state == RecoveryState::Animation && control.GetSnapshot().startedBlend &&
            control.GetSnapshot().blending && control.GetSnapshot().blendAlpha == 0 && control.GetSnapshot().returnCount == 1,
            "Stable recovery returns to Animation starting at the captured physical pose");
        for (int i = 0; i < 15; ++i)
        {
            ++input.completedStep;
            control.Update(input, settings);
        }
        Check(Near(control.GetSnapshot().blendAlpha, .5f), "The return pose reaches its midpoint after 0.25 seconds");
        for (int i = 0; i < 16; ++i)
        {
            ++input.completedStep;
            control.Update(input, settings);
        }
        Check(!control.GetSnapshot().blending && control.GetSnapshot().blendAlpha == 1,
            "Return completes after the configured blend duration");
    }

    void TestInterruptsAndReset()
    {
        RagdollRecovery control;
        RecoverySettings settings;
        auto input = Balanced();
        input.needsStep = true;
        control.Update(input, settings);
        Check(control.GetSnapshot().state == RecoveryState::Recovery,
            "A predicted balance loss can enter Recovery without a recorded collision impulse");
        input.needsStep = false;
        for (int i = 0; i < 12; ++i)
        {
            ++input.completedStep;
            control.Update(input, settings);
        }
        Check(control.GetSnapshot().blending, "Prepare a return in progress");
        ++input.completedStep;
        input.impactVelocity = settings.impactVelocity;
        control.Update(input, settings);
        Check(control.GetSnapshot().state == RecoveryState::Recovery && !control.GetSnapshot().blending &&
            control.GetSnapshot().enteredRecovery && control.GetSnapshot().recoveryCount == 2,
            "A fresh impact interrupts the return and recaptures recovery feet");
        input.impactVelocity = 0;
        input.fallen = true;
        for (int i = 0; i < 60; ++i)
        {
            ++input.completedStep;
            control.Update(input, settings);
        }
        Check(control.GetSnapshot().reason == RecoveryReason::Fallen && !control.GetSnapshot().holdingTargets &&
            !control.GetSnapshot().blending, "A latched B exit disables foot correction and blocks return");
        input.fallen = false;
        ++input.revision;
        control.Update(input, settings);
        Check(control.GetSnapshot().state == RecoveryState::Animation && control.GetSnapshot().recoveryCount == 0,
            "PA, teleport or mode revision resets recovery history even on the same step");
        ++input.completedStep;
        input.impactVelocity = settings.impactVelocity;
        control.Update(input, settings);
        ++input.completedStep;
        input.impactVelocity = 0;
        input.deltaTime = std::numeric_limits<float>::quiet_NaN();
        control.Update(input, settings);
        Check(control.GetSnapshot().reason == RecoveryReason::InvalidSample && control.GetSnapshot().stableTime == 0,
            "Invalid time cannot accidentally authorize a return");
        input.enabled = false;
        control.Update(input, settings);
        Check(control.GetSnapshot().reason == RecoveryReason::Disabled && control.GetSnapshot().recoveryCount == 0,
            "Disabling the controller clears its state and counters");
    }

    void TestPhysicalPoseBlend()
    {
        GraphNode source{}, target{};
        source.rotation = Quaternion(kIdentity);
        source.scale = XMFLOAT3(1, 1, 1);
        source.staleMatrix = false;
        target = source;
        target.matrixIdx = 17;
        target.skeletonRoot = true;
        const Matrix3 stretch(Vector3(1, 0, 0), Vector3(.2f, 2, 0), Vector3(.1f, .15f, .7f));
        source.xform = Matrix4(Matrix3(Quaternion(Vector3(kYUnitVector), DirectX::XM_PIDIV2)) * stretch,
            Vector3(1, 2, 3));
        target.xform = Matrix4(stretch, Vector3(3, 2, 1));
        const std::vector<GraphNode> from{source};
        std::vector<GraphNode> to{target};
        Check(BlendRecoveryPose(from, to, 0) && Near(to[0].xform, source.xform),
            "Alpha zero uses the actual physical matrix despite a stale animation rotation cache");
        Check(to[0].matrixIdx == 17 && to[0].skeletonRoot && !to[0].staleMatrix,
            "Physical blending preserves the destination skeleton metadata");
        to = {target};
        Check(BlendRecoveryPose(from, to, .5f), "Blend physical matrices with scale and shear");
        const Matrix4 halfway(Matrix3(Quaternion(Vector3(kYUnitVector), DirectX::XM_PIDIV4)) * stretch, Vector3(2, 2, 2));
        Check(Near(to[0].xform, halfway), "Rotation follows a spherical midpoint while stretch and shear survive");
        Check(Near(from[0].xform, source.xform), "The captured physical source remains frozen through the blend");
        to = {target};
        Check(BlendRecoveryPose(from, to, 1) && Near(to[0].xform, target.xform),
            "Alpha one leaves the current advancing animation pose intact");
        target.staleMatrix = true;
        target.rotation = Quaternion(Vector3(kYUnitVector), DirectX::XM_PI);
        target.scale = XMFLOAT3(1, 1, 1);
        to = {target};
        Check(BlendRecoveryPose(from, to, .5f) && !to[0].staleMatrix,
            "An animation destination is evaluated from its TRS cache when its matrix is stale");
        const auto before = to;
        Check(!BlendRecoveryPose({}, to, .5f) && Near(to[0].xform, before[0].xform),
            "A different skeleton size does not partially mutate the destination");
        Check(!BlendRecoveryPose(from, to, std::numeric_limits<float>::quiet_NaN()), "Reject a nonfinite blend weight");
    }
}

void RunRagdollRecoveryTests()
{
    TestTransitions();
    TestInterruptsAndReset();
    TestPhysicalPoseBlend();
    std::printf("PASS: %u recovery state and physical-pose blend checks (no simulation)\n", checks);
}
