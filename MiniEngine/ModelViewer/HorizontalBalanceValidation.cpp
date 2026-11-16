#include "RagdollImpactTest.h"
#include "GameFramework/Character.h"
#include "GameFramework/World.h"
#include "GameFramework/PlayerController.h"
#include "PhysicsWorld.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <stdexcept>

using namespace PARS;
using namespace Math;
using namespace physx;

namespace
{
    struct HorizontalTrial
    {
    public:
        const char* name;
        PxVec3 velocity;
    };
}

bool RunHorizontalBalanceValidation(RagdollImpactTest& test, World& world, Character& player,
    const std::wstring& path)
{
    std::ofstream report(path);
    std::ofstream samples(path + L".csv");
    report << std::fixed << std::setprecision(6);
    samples << std::fixed << std::setprecision(8);
    Character* dummy = test.GetDummy();
    if (!dummy)
    {
        report << "FAIL: missing dummy\n";
        return false;
    }
    SkeletalMeshComponent& mesh = dummy->GetMesh();
    const HorizontalBalanceSettings originalSettings = mesh.GetHorizontalBalanceSettings();
    bool passed = true;
    try
    {
        PlayerController* controller = dynamic_cast<PlayerController*>(player.GetController());
        if (controller)
        {
            controller->SetMoveInput(0, 0, false);
        }
        test.SetContactCheckEnabled(false);
        world.SetPhysicsTimeScale(1);
        world.SetPhysicsPaused(false);
        const std::array<HorizontalTrial, 6> trials{{
            {"idle", PxVec3(0)},
            {"backward", PxVec3(0, 0, -.5f)},
            {"forward", PxVec3(0, 0, .5f)},
            {"sideways", PxVec3(.5f, 0, 0)},
            {"diagonal", PxVec3(.35f, 0, -.35f)},
            {"strong", PxVec3(0, 0, -2.0f)}
        }};
        samples << "case,enabled,frame,com_x,com_y,com_z,v_x,v_y,v_z,force_x,force_y,force_z,target_x,target_z,support,load_left,load_right,steps,landings,release,limited\n";
        report << "Identical whole-body velocity impulses; same saved PA; continuous active Idle; both modes warm up with PD for 180 steps, then toggle before 360 recorded steps.\n";
        report << "frequency=" << originalSettings.frequency << " damping_ratio=" << originalSettings.dampingRatio
            << " max_acceleration=" << originalSettings.maximumAcceleration << " max_acceleration_rate="
            << originalSettings.maximumAccelerationRate << " stepping_position_scale=" << originalSettings.steppingPositionScale << '\n';
        report << "case,enabled,first_second_mean_speed,final_second_mean_speed,displacement,peak_speed,peak_force,steps,landings,released,initial_com_x,initial_com_z\n";
        for (const HorizontalTrial& trial : trials)
        {
            for (unsigned mode = 0; mode < 2; ++mode)
            {
                if (!test.SetAutomaticSupport(true))
                {
                    throw std::runtime_error("Resetting the actual character failed");
                }
                mesh.SetAutomaticRagdoll(false);
                mesh.GetHorizontalBalanceSettings() = originalSettings;
                // 보조 ON/OFF 모두 같은 안정 자세에서 충격을 시작한다.
                mesh.GetHorizontalBalanceSettings().enabled = true;
                std::string error;
                if (!dummy->SetPhysicsMode(PhysicsAnimationMode::ActiveRagdoll, error) ||
                    !mesh.GetRagdoll().SetRootSupport(1, false, error) || !mesh.SetAutomaticRootSupport(true, error))
                {
                    throw std::runtime_error(error);
                }
                for (unsigned frame = 0; frame < 180; ++frame)
                {
                    world.Tick(1.0f / 60.0f);
                    test.Update();
                }
                const FootSupportSnapshot initial = mesh.GetFootSupport().GetSnapshot();
                mesh.GetHorizontalBalanceSettings().enabled = mode != 0;
                mesh.GetRagdoll().AddVelocity(Vector3(trial.velocity.x, trial.velocity.y, trial.velocity.z));
                uint64_t previousStarts = initial.footstep.startCount;
                uint64_t previousLandings = initial.footstep.landingCount;
                uint64_t starts = 0;
                uint64_t landings = 0;
                double firstSpeed = 0;
                double finalSpeed = 0;
                float peakSpeed = 0;
                float peakForce = 0;
                bool released = false;
                for (unsigned frame = 0; frame < 360; ++frame)
                {
                    world.Tick(1.0f / 60.0f);
                    test.Update();
                    const FootSupportSnapshot& sample = mesh.GetFootSupport().GetSnapshot();
                    const HorizontalBalanceSnapshot& horizontal = mesh.GetHorizontalBalanceSnapshot();
                    // 낙상으로 플래너 카운터가 초기화돼도 부호 없는 정수의 차이가 넘치지 않게 누적한다.
                    starts += sample.footstep.startCount >= previousStarts ?
                        sample.footstep.startCount - previousStarts : sample.footstep.startCount;
                    landings += sample.footstep.landingCount >= previousLandings ?
                        sample.footstep.landingCount - previousLandings : sample.footstep.landingCount;
                    previousStarts = sample.footstep.startCount;
                    previousLandings = sample.footstep.landingCount;
                    const float speed = std::sqrt(sample.centerOfMassVelocity.x * sample.centerOfMassVelocity.x +
                        sample.centerOfMassVelocity.z * sample.centerOfMassVelocity.z);
                    if (!mesh.GetRagdoll().IsFinite() || !horizontal.force.isFinite() || horizontal.force.y != 0 ||
                        horizontal.force.magnitude() > sample.totalMass * originalSettings.maximumAcceleration + .01f ||
                        (mode == 0 && !horizontal.force.isZero()) ||
                        (horizontal.supportWeight == 0 && !horizontal.force.isZero()))
                    {
                        throw std::runtime_error("Horizontal force invariant failed");
                    }
                    if (frame < 60)
                    {
                        firstSpeed += speed / 60;
                    }
                    if (frame >= 300)
                    {
                        finalSpeed += speed / 60;
                    }
                    peakSpeed = (std::max)(peakSpeed, speed);
                    peakForce = (std::max)(peakForce, horizontal.force.magnitude());
                    released = released || mesh.GetRootSupportRelease() != RootSupportRelease::None;
                    samples << trial.name << ',' << mode << ',' << frame << ',' << sample.centerOfMass.x << ','
                        << sample.centerOfMass.y << ',' << sample.centerOfMass.z << ',' << sample.centerOfMassVelocity.x << ','
                        << sample.centerOfMassVelocity.y << ',' << sample.centerOfMassVelocity.z << ',' << horizontal.force.x << ','
                        << horizontal.force.y << ',' << horizontal.force.z << ',' << horizontal.targetCom.x << ','
                        << horizontal.targetCom.z << ',' << horizontal.supportWeight << ',' << horizontal.loadRatios[0] << ','
                        << horizontal.loadRatios[1] << ',' << sample.footstep.startCount << ',' << sample.footstep.landingCount << ','
                        << static_cast<unsigned>(mesh.GetRootSupportRelease()) << ',' << horizontal.limited << '\n';
                }
                const FootSupportSnapshot& final = mesh.GetFootSupport().GetSnapshot();
                const PxVec3 displacement = final.centerOfMass - initial.centerOfMass;
                report << trial.name << ',' << mode << ',' << firstSpeed << ',' << finalSpeed << ','
                    << std::sqrt(displacement.x * displacement.x + displacement.z * displacement.z) << ',' << peakSpeed << ','
                    << peakForce << ',' << starts << ',' << landings << ',' << released << ','
                    << initial.centerOfMass.x << ',' << initial.centerOfMass.z << std::endl;
            }
        }
        if (world.GetPhysicsWorld()->GetErrorCount() != 0)
        {
            throw std::runtime_error("PhysX reported errors during horizontal assistance validation");
        }
        report << "PASS: actual character A/B and finite, XZ-only, acceleration limit, disabled and no-load invariants\n";
    }
    catch (const std::exception& error)
    {
        report << "FAIL: " << error.what() << std::endl;
        passed = false;
    }
    mesh.GetHorizontalBalanceSettings() = originalSettings;
    world.SetPhysicsPaused(true);
    return passed;
}
