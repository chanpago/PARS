#include "../Model/HorizontalBalance.h"
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>

using namespace PARS;
using namespace physx;

namespace
{
    void CheckBalance(bool value, const char* message)
    {
        if (!value)
        {
            std::fprintf(stderr, "FAIL: %s\n", message);
            std::exit(1);
        }
    }

    HorizontalBalanceInput StandingBalance()
    {
        HorizontalBalanceInput input;
        input.enabled = true;
        input.revision = 1;
        input.completedStep = 1;
        input.mass = 60;
        input.deltaTime = 1.0f / 60.0f;
        input.supportWeight = 1;
        input.com = PxVec3(0, 1, 0);
        for (size_t side = 0; side < input.feet.size(); ++side)
        {
            HorizontalBalanceFoot& foot = input.feet[side];
            foot.available = true;
            foot.grounded = true;
            foot.position = PxVec3(side == 0 ? -.15f : .15f, 0, 0);
            foot.normalImpulse = input.mass * 9.81f * input.deltaTime * .5f;
        }
        return input;
    }
}

void TestHorizontalBalance()
{
    HorizontalBalance controller;
    HorizontalBalanceSettings settings;
    settings.contactBlendTime = 0;
    settings.maximumAccelerationRate = 10000;
    HorizontalBalanceInput input = StandingBalance();
    CheckBalance(controller.Update(input, settings) && controller.GetSnapshot().force.isZero(),
        "Initial standing pose has no artificial horizontal pull");
    ++input.completedStep;
    input.com.x = -.1f;
    input.velocity = PxVec3(-.5f, 20, -.3f);
    CheckBalance(controller.Update(input, settings) && controller.GetSnapshot().force.x > 0 &&
        controller.GetSnapshot().force.z > 0 && controller.GetSnapshot().force.y == 0 &&
        controller.GetSnapshot().force.magnitude() <= input.mass * settings.maximumAcceleration + 1e-4f,
        "Backward and sideways COM motion receives bounded restoring force without a Y force");
    const PxVec3 force = controller.GetSnapshot().force;
    input.velocity = -input.velocity;
    CheckBalance(controller.Update(input, settings) && (controller.GetSnapshot().force - force).isZero(),
        "Repeated queries cannot advance the controller twice in one physics step");
    ++input.completedStep;
    input.velocity = PxVec3(2, 0, 0);
    CheckBalance(controller.Update(input, settings) && controller.GetSnapshot().force.x < 0,
        "Fast return toward the target is braked instead of amplified");
    ++input.completedStep;
    input.stepping = true;
    input.feet[1].moving = true;
    input.feet[1].position = PxVec3(20, 1, 20);
    CheckBalance(controller.Update(input, settings) && controller.GetSnapshot().targetCom.magnitude() < 1e-6f &&
        controller.GetSnapshot().contactWeights[1] == 0,
        "An airborne motion target cannot drag the COM reference or claim support");
    ++input.completedStep;
    input.feet[0].grounded = false;
    CheckBalance(controller.Update(input, settings) && controller.GetSnapshot().force.isZero(),
        "Actual loss of all contact clears external force immediately without contact grace");
    ++input.completedStep;
    input.feet[0].grounded = true;
    input.feet[0].normalImpulse = 0;
    CheckBalance(controller.Update(input, settings) && controller.GetSnapshot().force.isZero(),
        "A contact with no measured load cannot create assistance");
    input = StandingBalance();
    ++input.revision;
    settings.contactBlendTime = .1f;
    settings.maximumAccelerationRate = 2;
    CheckBalance(controller.Update(input, settings), "Reset the reference on a contact revision");
    const float firstContact = controller.GetSnapshot().contactWeights[0];
    ++input.completedStep;
    input.velocity = PxVec3(10, 0, 10);
    CheckBalance(controller.Update(input, settings) && controller.GetSnapshot().contactWeights[0] > firstContact &&
        controller.GetSnapshot().acceleration.magnitude() <= settings.maximumAccelerationRate * input.deltaTime + 1e-5f,
        "Landing contact and acceleration changes ramp progressively");
    ++input.completedStep;
    settings.maximumAcceleration = 0;
    CheckBalance(controller.Update(input, settings) && controller.GetSnapshot().force.isZero(),
        "A zero acceleration ceiling disables force without leaving a smoothed residual");
    ++input.completedStep;
    settings.maximumAcceleration = 2;
    input.supportWeight = 0;
    CheckBalance(controller.Update(input, settings) && controller.GetSnapshot().force.isZero(),
        "Released root support cannot be held up by horizontal assistance");
    ++input.completedStep;
    input.supportWeight = 1;
    input.velocity.x = std::numeric_limits<float>::quiet_NaN();
    CheckBalance(!controller.Update(input, settings) && controller.GetSnapshot().force.isZero(),
        "Invalid COM samples cannot reach the force accumulator");
    input = StandingBalance();
    controller.Reset();
    CheckBalance(controller.Update(input, settings), "Initialize a fresh landing reference");
    ++input.completedStep;
    input.feet[0].position.x += .4f;
    input.feet[1].position.x += .4f;
    CheckBalance(controller.Update(input, settings) && controller.GetSnapshot().targetCom.x > 0 &&
        controller.GetSnapshot().targetCom.x < .4f && controller.GetSnapshot().force.x > 0,
        "A confirmed new stance updates the COM reference continuously rather than snapping");
    input = StandingBalance();
    settings.enabled = false;
    CheckBalance(!controller.Update(input, settings) && controller.GetSnapshot().force.isZero(),
        "Disabling assistance resets the controller and clears force");
    std::puts("PASS: horizontal PD direction, limits, contact gating, landing blend, reset and disable");
}
