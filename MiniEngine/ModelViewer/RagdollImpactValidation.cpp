#include "RagdollImpactTest.h"
#include "GameFramework/Character.h"
#include "GameFramework/World.h"
#include "GameFramework/PlayerController.h"
#include "PreviewAnimInstance.h"
#include "Util/CommandLineArg.h"
#include <fstream>
#include <stdexcept>
#include <cmath>
#include <functional>

// 기준과 후보에서 같은 바닥·애니메이션·입력을 사용하는 실제 캐릭터 검증이다.
bool RunKinematicRecoveryValidation(RagdollImpactTest& test, PARS::World& world, PARS::Character& player,
    const std::wstring& path, uint32_t scenario)
{
    using namespace PARS;
    using namespace Math;
    std::ofstream report(path);
    std::ofstream samples(path + L".csv");
    std::ofstream feet(path + L".feet.csv");
    std::ofstream flails(path + L".flail.csv");
    uint32_t validateFootAlignment = 0;
    uint32_t validateFlail = 0;
    uint32_t flailDirection = 0;
    CommandLineArgs::GetInteger(L"validatefootalignment", validateFootAlignment);
    CommandLineArgs::GetInteger(L"validateflail", validateFlail);
    CommandLineArgs::GetInteger(L"flaildirection", flailDirection);
    const std::function<void(bool, const char*)> check = [&](bool condition, const char* message) -> void
    {
        report << (condition ? "PASS: " : "FAIL: ") << message << std::endl;
        if (!condition)
        {
            throw std::runtime_error(message);
        }
    };
    try
    {
        check(report.good() && samples.good() && feet.good() && scenario <= 5 && test.SetAutomaticSupport(true),
            "Prepare grounded automatic kinematic dummy");
        SkeletalMeshComponent& mesh = test.GetDummy()->GetMesh();
        if (validateFlail)
        {
            check(mesh.GetRecoveryFlail().IsConfigured() && flails.good() && flailDirection <= 2,
                "Both real flail clips and the Spine mask are configured");
        }
        check(!mesh.IsRagdoll() && mesh.GetRagdoll().GetKinematicBodyCount() == mesh.GetRagdoll().GetBodyCount(),
            "Idle starts with every PA body kinematic");
        check(mesh.GetKinematicStableDuration() == 2.0f, "Default recovery duration is exactly two seconds");
        std::string error;
        check(player.StopRagdoll(error), "Place projectile source away from the dummy");
        const Vector3 source = flailDirection == 1 ? Vector3(3.5f, 0, 0) :
            flailDirection == 2 ? Vector3(2.0f, 0, -2.5f) : Vector3(3.5f, 0, -5.0f);
        player.SetActorLocation(source + Vector3(0, player.GetCapsuleComponent().GetHalfHeight(), 0));
        PlayerController* controller = dynamic_cast<PlayerController*>(player.GetController());
        if (controller)
        {
            controller->SetMoveInput(0, 0, false);
            controller->SetInputEnabled(false);
        }
        check(test.SetProjectileSettings(scenario == 3 ? 20.0f : 2.0f, 7.5f), "Set explicit projectile test input");
        PhysicsSkeleton skeleton;
        check(mesh.BuildPhysicsSkeleton(skeleton, error), "Read actual dummy skeleton");
        world.SetPhysicsPaused(false);
        world.SetPhysicsTimeScale(1);
        samples << "frame,time,mode,blending,zone,stable_time,impact,step_count,bone,x,y,z,position_error,rotation_error\n";
        feet << "frame,time,mode,blending,side,grounded,root_x,root_y,root_z,sole_x,sole_y,sole_z,target_x,target_y,target_z\n";
        flails << "frame,mode,recovery,layer_weight,front_weight,target_front_weight,phase,ik_status,clip\n";
        report << "fixed_dt=" << PhysicsWorld::FixedDeltaTime << " stable_duration=2 blend_duration="
            << mesh.GetRecoverySettings().blendDuration << " scenario=" << scenario << std::endl;
        bool seenActive = false;
        bool seenReturn = false;
        bool secondShot = false;
        unsigned entries = 0;
        unsigned returns = 0;
        unsigned flailSteps = 0;
        unsigned directionChecks = 0;
        float frontWeightSum = 0;
        PhysicsAnimationMode previousMode = mesh.GetPhysicsMode();
        for (uint32_t frame = 0; frame < 900; ++frame)
        {
            if ((scenario != 0 && frame == 120) ||
                (scenario == 2 && seenReturn && !secondShot && !mesh.IsBlendingToKinematic()) ||
                (scenario == 4 && seenReturn && !secondShot && mesh.IsBlendingToKinematic()) ||
                (scenario == 5 && seenActive && !secondShot && mesh.GetKinematicStableTime() >= .8f))
            {
                check(test.FireFrom(player), "Fire a real colliding projectile");
                secondShot = seenActive;
            }
            const float beforeTimer = mesh.GetKinematicStableTime();
            const bool beforeFalling = mesh.IsFollowingFallingAnimation();
            const Vector3 beforeRoot = test.GetDummy()->GetActorLocation();
            Vector3 expectedRootOffset(kZero);
            bool beforeBothGrounded = true;
            if (mesh.IsActiveRagdoll())
            {
                // 전환 전에 마지막 완료 물리 스텝의 실제 발과 현재 애니메이션을 함께 기록한다.
                std::vector<Matrix4> beforeTargets;
                check(EvaluatePhysicsPose(skeleton, mesh.GetAnimInstance().GetPose(),
                    Matrix4(mesh.GetComponentTransform()), beforeTargets, error), "Evaluate pre-return animation");
                for (const FootSoleSample& foot : mesh.GetFootSupport().GetSnapshot().feet)
                {
                    const std::vector<int>& nodes = mesh.GetRagdoll().GetBoneNodes();
                    size_t body = 0;
                    while (body < nodes.size() && nodes[body] != foot.boneNode)
                    {
                        ++body;
                    }
                    check(foot.available && foot.boneNode >= 0 && body < nodes.size(), "Resolve physical foot body");
                    PhysicsGroundContact contact;
                    beforeBothGrounded = mesh.GetRagdoll().GetBodyGroundContact(body, contact) && beforeBothGrounded;
                    const Matrix4 actualSole = mesh.GetRagdoll().GetBoneWorld(body) * foot.soleInBone;
                    const Matrix4 targetSole = PhysicsBoneFrame(beforeTargets[foot.boneNode]) * foot.soleInBone;
                    expectedRootOffset += (Vector3(actualSole.GetW()) - Vector3(targetSole.GetW())) * .5f;
                }
            }
            float expectedFrontWeight = -1;
            if (validateFlail)
            {
                Vector3 center(kZero);
                float mass = 0;
                if (mesh.GetRagdoll().GetCenterOfMass(center, mass) && mass > 0 &&
                    mesh.GetRagdoll().GetExternalImpactImpulse() >= mass * mesh.GetRecoverySettings().impactVelocity)
                {
                    std::vector<RagdollBodyImpulse> impacts;
                    mesh.GetRagdoll().CaptureExternalImpacts(impacts);
                    physx::PxVec3 impulse(0);
                    for (const RagdollBodyImpulse& impact : impacts)
                    {
                        impulse += impact.impulse;
                    }
                    const Vector3 forward = mesh.GetComponentTransform().GetZ();
                    const float impulseLength = std::hypot(impulse.x, impulse.z);
                    const float forwardLength = std::hypot(static_cast<float>(forward.GetX()), static_cast<float>(forward.GetZ()));
                    if (impulseLength > 1e-5f && forwardLength > 1e-5f)
                    {
                        expectedFrontWeight = .5f + .5f * Clamp((impulse.x / impulseLength) *
                            (static_cast<float>(forward.GetX()) / forwardLength) + (impulse.z / impulseLength) *
                            (static_cast<float>(forward.GetZ()) / forwardLength), -1.0f, 1.0f);
                    }
                }
            }
            world.Tick(1.0f / 60.0f);
            test.Update();
            check(mesh.GetRagdoll().IsFinite() && mesh.GetPhysicsError().empty(), "Finite physical state and no controller error");
            const RecoveryFlail& flail = mesh.GetRecoveryFlail();
            flails << frame << ',' << static_cast<int>(mesh.GetPhysicsMode()) << ','
                << static_cast<int>(mesh.GetRecoverySnapshot().state) << ',' << flail.GetWeight() << ','
                << flail.GetFrontWeight() << ',' << flail.GetTargetFrontWeight() << ',' << flail.GetPhase() << ','
                << static_cast<int>(mesh.GetFootstepIKSnapshot().status) << ','
                << mesh.GetAnimInstance().GetCurrentAnimationIndex() << '\n';
            if (flail.GetWeight() > 0)
            {
                ++flailSteps;
                frontWeightSum += flail.GetFrontWeight();
                if (validateFlail)
                {
                    check(mesh.IsActiveRagdoll() && mesh.GetRecoverySnapshot().state == RecoveryState::Recovery &&
                        mesh.GetAnimInstance().GetCurrentAnimationIndex() == 0 &&
                        mesh.GetFootstepIKSnapshot().status == FootstepIKStatus::Applied,
                        "Upper-body flail keeps Active Ragdoll, Idle selection, Recovery, and leg IK active together");
                    if (expectedFrontWeight >= 0)
                    {
                        ++directionChecks;
                        check(std::abs(flail.GetTargetFrontWeight() - expectedFrontWeight) < 1e-4f,
                            "The blend target matches the actual received horizontal contact impulse");
                    }
                }
            }
            if (mesh.GetPhysicsMode() != previousMode)
            {
                report << "transition_frame=" << frame << " from=" << static_cast<int>(previousMode)
                    << " to=" << static_cast<int>(mesh.GetPhysicsMode()) << " previous_stable=" << beforeTimer
                    << " stable=" << mesh.GetKinematicStableTime() << std::endl;
                if (mesh.IsActiveRagdoll())
                {
                    ++entries;
                    seenActive = true;
                }
                else if (!mesh.IsRagdoll())
                {
                    ++returns;
                    seenReturn = true;
                    if (beforeFalling)
                    {
                        check(mesh.IsFollowingStandingUpAnimation(), "A fallen character returns through the separate standing-up animation");
                    }
                    else
                    {
                        check(beforeTimer >= 2.0f - 1.0f / 60.0f - .00001f, "Return requires the complete two-second interval");
                        const Vector3 rootOffset = test.GetDummy()->GetActorLocation() - beforeRoot;
                        const float alignmentError = float(Length(rootOffset - expectedRootOffset));
                        report << "return_both_grounded=" << beforeBothGrounded
                            << " root_dx=" << float(rootOffset.GetX()) << " root_dy=" << float(rootOffset.GetY())
                            << " root_dz=" << float(rootOffset.GetZ()) << " sole_alignment_error=" << alignmentError << std::endl;
                        if (validateFootAlignment)
                        {
                            check(beforeBothGrounded, "Kinematic handoff requires both actual ground contacts");
                            check(alignmentError < .0001f, "Root translation matches both sole midpoints on all three axes");
                        }
                    }
                }
                previousMode = mesh.GetPhysicsMode();
            }
            if (frame == 180)
            {
                const float stable = mesh.GetKinematicStableTime();
                const uint64_t step = world.GetPhysicsWorld()->GetStepCount();
                world.SetPhysicsPaused(true);
                for (int pauseFrame = 0; pauseFrame < 120; ++pauseFrame)
                {
                    world.Tick(1.0f / 60.0f);
                }
                check(mesh.GetKinematicStableTime() == stable && world.GetPhysicsWorld()->GetStepCount() == step,
                    "Paused wall-clock time does not advance recovery");
                world.SetPhysicsPaused(false);
            }
            const FootSupportSnapshot& support = mesh.GetFootSupport().GetSnapshot();
            std::vector<Matrix4> targets;
            check(EvaluatePhysicsPose(skeleton, mesh.GetAnimInstance().GetPose(), Matrix4(mesh.GetComponentTransform()), targets, error),
                "Evaluate animation targets for measured bone errors");
            for (size_t side = 0; side < support.feet.size(); ++side)
            {
                const FootSoleSample& foot = support.feet[side];
                const std::vector<int>& nodes = mesh.GetRagdoll().GetBoneNodes();
                size_t body = 0;
                while (body < nodes.size() && nodes[body] != foot.boneNode)
                {
                    ++body;
                }
                check(foot.available && foot.boneNode >= 0 && body < nodes.size(), "Measure actual foot sole");
                PhysicsGroundContact contact;
                const bool grounded = mesh.GetRagdoll().GetBodyGroundContact(body, contact);
                const Matrix4 actualSole = mesh.GetRagdoll().GetBoneWorld(body) * foot.soleInBone;
                const Matrix4 targetSole = PhysicsBoneFrame(targets[foot.boneNode]) * foot.soleInBone;
                const Vector3 actualPosition(actualSole.GetW());
                const Vector3 targetPosition(targetSole.GetW());
                const Vector3 rootPosition = test.GetDummy()->GetActorLocation();
                feet << frame << ',' << (frame + 1) / 60.0f << ',' << static_cast<int>(mesh.GetPhysicsMode())
                    << ',' << mesh.IsBlendingToKinematic() << ',' << side << ',' << grounded
                    << ',' << float(rootPosition.GetX()) << ',' << float(rootPosition.GetY()) << ',' << float(rootPosition.GetZ())
                    << ',' << float(actualPosition.GetX()) << ',' << float(actualPosition.GetY()) << ',' << float(actualPosition.GetZ())
                    << ',' << float(targetPosition.GetX()) << ',' << float(targetPosition.GetY()) << ',' << float(targetPosition.GetZ()) << '\n';
            }
            for (size_t body = 0; body < mesh.GetRagdoll().GetBodyCount(); ++body)
            {
                const Matrix4 actual = mesh.GetRagdoll().GetBoneWorld(body);
                const Matrix4 target = PhysicsBoneFrame(targets[mesh.GetRagdoll().GetBoneNodes()[body]]);
                const Vector3 position(actual.GetW());
                const float dot = std::abs(DirectX::XMVectorGetX(DirectX::XMQuaternionDot(Quaternion(actual), Quaternion(target))));
                samples << frame << ',' << (frame + 1) / 60.0f << ',' << static_cast<int>(mesh.GetPhysicsMode())
                    << ',' << mesh.IsBlendingToKinematic() << ',' << static_cast<int>(support.balance.zone)
                    << ',' << mesh.GetKinematicStableTime() << ',' << mesh.GetRagdoll().GetExternalImpactImpulse()
                    << ',' << support.footstep.startCount << ',' << mesh.GetPhysicsAsset()->bodies[body].bone
                    << ',' << float(position.GetX()) << ',' << float(position.GetY()) << ',' << float(position.GetZ())
                    << ',' << float(Length(position - Vector3(target.GetW())))
                    << ',' << 2 * std::acos(Clamp(dot, 0.0f, 1.0f)) << '\n';
            }
        }
        world.SetPhysicsPaused(true);
        report << "entries=" << entries << " returns=" << returns << std::endl;
        check(world.GetPhysicsWorld()->GetErrorCount() == 0, "No PhysX errors");
        check(scenario == 0 ? !seenActive : seenActive, "Impact is required to enter active ragdoll");
        if (validateFlail)
        {
            check(scenario == 0 ? flailSteps == 0 : flailSteps > 0,
                "Upper-body flail starts only when recovery is needed");
            const float meanFrontWeight = flailSteps > 0 ? frontWeightSum / static_cast<float>(flailSteps) : .5f;
            report << "flail_steps=" << flailSteps << " mean_front_weight=" << meanFrontWeight
                << " direction=" << flailDirection << " measured_direction_checks=" << directionChecks << std::endl;
            if (scenario != 0)
            {
                check(directionChecks > 0, "Validate the blend against real contact vectors rather than projectile launch direction");
            }
        }
        if (scenario == 1 || scenario == 2 || scenario == 4 || scenario == 5)
        {
            check(seenReturn, "Grounded projectile recovery returns to kinematic mode");
        }
        if (scenario == 2 || scenario == 4)
        {
            check(entries >= 2 && returns >= 2, "A second collision can activate and recover the same dummy again");
        }
        report << "PASS: kinematic recovery validation" << std::endl;
        return true;
    }
    catch (const std::exception& error)
    {
        report << "FAIL: " << error.what() << std::endl;
        return false;
    }
}

bool RunCharacterBodyImpactValidation(RagdollImpactTest& test, PARS::World& world, PARS::Character& player,
    const std::wstring& path, uint32_t scenario, uint32_t frames)
{
    using namespace PARS;
    using namespace Math;
    using namespace physx;
    uint32_t kinematicCase = 0;
    if (CommandLineArgs::GetInteger(L"kinematiccase", kinematicCase))
    {
        return RunKinematicRecoveryValidation(test, world, player, path, kinematicCase);
    }
    std::ofstream report(path);
    std::ofstream samples(path + L".csv");
    std::ofstream events(path + L".events.csv");
    std::ofstream states(path + L".states.csv");
    std::ofstream capsules(path + L".capsules.csv");
    uint32_t kinematicPlayer = 0;
    uint32_t modeCycles = 0;
    uint32_t incoming = 0;
    CommandLineArgs::GetInteger(L"bodyimpactkinematic", kinematicPlayer);
    CommandLineArgs::GetInteger(L"bodyimpactmodes", modeCycles);
    CommandLineArgs::GetInteger(L"bodyimpactincoming", incoming);
    const std::function<void(bool, const char*)> check = [&](bool condition, const char* message) -> void
    {
        report << (condition ? "PASS: " : "FAIL: ") << message << std::endl;
        if (!condition)
        {
            throw std::runtime_error(message);
        }
    };
    try
    {
        Character* dummy = test.GetDummy();
        check(report.good() && samples.good() && events.good() && dummy && scenario <= 6 && frames > 0 && frames <= 720,
            "Open actual-character body impact validation");
        std::string error;
        check(test.Reset(), "Reset grounded Idle dummy with original automatic support");
        PlayerController* controller = dynamic_cast<PlayerController*>(player.GetController());
        check(controller != nullptr, "Use the normal player movement controller");
        controller->SetInputEnabled(true);
        controller->SetMoveInput(0, 0, false);
        check(player.StopRagdoll(error), "Reset player simulation");
        player.GetCharacterMovement().StopMovementImmediately();
        const float yaw = scenario == 3 || scenario == 5 ? DirectX::XM_PIDIV2 :
            scenario == 4 ? -DirectX::XM_PIDIV2 : scenario == 6 ? DirectX::XM_PIDIV4 : 0.0f;
        const Vector3 direction(std::sin(yaw), 0, std::cos(yaw));
        const Vector3 origin = dummy->GetActorLocation() - direction * 2.0f;
        player.SetActorTransform(AffineTransform(Quaternion(Vector3(kYUnitVector), yaw),
            Vector3(origin.GetX(), player.GetCapsuleComponent().GetHalfHeight(), origin.GetZ())));
        controller->SetControlYaw(yaw);
        PreviewAnimInstance& animation = static_cast<PreviewAnimInstance&>(player.GetMesh().GetAnimInstance());
        animation.SetAutoCycleEnabled(false);
        animation.SetLocomotionEnabled(true);
        animation.PlayAnimation(0);
        world.Tick(0);
        if (!kinematicPlayer)
        {
            check(player.SetPhysicsMode(PhysicsAnimationMode::ActiveRagdoll, error) &&
                player.GetMesh().GetRagdoll().SetRootSupported(true, error), "Enable original supported player");
        }
        CharacterImpactSettings impactSettings = player.GetCharacterImpact().GetSettings();
        CommandLineArgs::GetFloat(L"bodyimpactmass", impactSettings.collisionMass);
        CommandLineArgs::GetFloat(L"bodyimpactradius", impactSettings.capsuleRadiusScale);
        CommandLineArgs::GetFloat(L"bodyimpactheight", impactSettings.capsuleHeightScale);
        uint32_t enabled = impactSettings.enabled ? 1 : 0;
        CommandLineArgs::GetInteger(L"bodyimpactenabled", enabled);
        impactSettings.enabled = enabled != 0;
        check(player.GetCharacterImpact().SetSettings(impactSettings, error), "Configure independent capsule mass, geometry and enable state");
        float expectedTotalMass = 0;
        for (const PhysicsBody& body : player.GetMesh().GetPhysicsAsset()->bodies)
        {
            expectedTotalMass += body.mass;
        }
        uint32_t toggle = 0;
        CommandLineArgs::GetInteger(L"bodyimpacttoggle", toggle);
        PhysicsSkeleton playerSkeleton;
        PhysicsSkeleton dummySkeleton;
        check(player.GetMesh().BuildPhysicsSkeleton(playerSkeleton, error) &&
            dummy->GetMesh().BuildPhysicsSkeleton(dummySkeleton, error), "Load real skeletons for animation-relative measurements");
        world.SetPhysicsPaused(false);
        world.SetPhysicsTimeScale(1);
        samples << "frame,time,actor,bone,x,y,z,qx,qy,qz,qw,animation_position_error,animation_rotation_error,point_speed,root_supported,grounded_left,grounded_right,drive_error\n";
        events << "frame,time,event,hit,receiver,requested,applied,closing,px,py,pz,ix,iy,iz,capped\n";
        states << "frame,mode,blending,zone,stable_time,step_count,impact\n";
        capsules << "frame,time,mode,root_x,root_y,root_z,present,mass,radius,half_length,x,y,z,cyan_lines\n";
        report << "scenario=" << scenario << " frames=" << frames << " fixed_dt=" << PhysicsWorld::FixedDeltaTime
            << " player_auto_support=" << player.GetMesh().UsesAutomaticRootSupport()
            << " dummy_auto_support=" << dummy->GetMesh().UsesAutomaticRootSupport()
            << " kinematic_player=" << kinematicPlayer << " mode_cycles=" << modeCycles << " incoming=" << incoming << std::endl;
        report << "pa_root_mass_kg=" << player.GetMesh().GetRagdoll().GetImpactRootMass()
            << " pa_total_mass_kg=" << expectedTotalMass << " capsule_collision_mass_kg=" << impactSettings.collisionMass
            << " capsule_added_pa_mass_kg=0 capsule_enabled=" << impactSettings.enabled
            << " repeated_toggle=" << toggle << std::endl;
        report << "capsule_radius_scale=" << impactSettings.capsuleRadiusScale
            << " capsule_height_scale=" << impactSettings.capsuleHeightScale << std::endl;
        uint64_t eventCount = 0;
        for (uint32_t frame = 0; frame < frames; ++frame)
        {
            if (modeCycles && frame % 60 == 0)
            {
                const PhysicsAnimationMode mode = frame / 60 % 3 == 0 ? PhysicsAnimationMode::Animation :
                    frame / 60 % 3 == 1 ? PhysicsAnimationMode::ActiveRagdoll : PhysicsAnimationMode::PassiveRagdoll;
                check(player.SetPhysicsMode(mode, error), "Cycle kinematic, active and passive player configurations");
            }
            if (toggle && frame < 100)
            {
                // 기존 대기 구간에서 설정만 반복 전환해 충돌 형상과 PA 질량 보존을 확인한다.
                impactSettings.enabled = frame % 2 != 0;
                check(player.GetCharacterImpact().SetSettings(impactSettings, error), "Toggle physical capsule at a fixed-step boundary");
                check(player.GetMesh().GetRagdoll().HasCharacterCapsule() == impactSettings.enabled,
                    "Simulation shape follows the enable state");
                const Ragdoll& ragdoll = player.GetMesh().GetRagdoll();
                const float expectedMass = player.GetMesh().GetPhysicsAsset()->bodies[ragdoll.GetImpactRootBody()].mass;
                check(std::abs(ragdoll.GetImpactRootMass() - expectedMass) < .0001f,
                    "Runtime toggle preserves the PA root mass in both enable states");
                check(std::abs(ragdoll.GetCharacterCapsuleMass() - (impactSettings.enabled ? impactSettings.collisionMass : 0)) < .0001f,
                    "Runtime toggle changes only the independent collision mass");
            }
            // 기본 입력: 2초 안정화, 3초 접근, 2초 후퇴, 2초 재접근, 나머지는 정지한다.
            const float input = scenario == 0 || frame < 120 || frame >= 540 ? 0.0f :
                frame < 300 ? 1.0f : frame < 420 ? -1.0f : 1.0f;
            controller->SetMoveInput(input, 0, scenario != 1);
            if (scenario == 5 && frame == 120)
            {
                // 토크 상한과 밀침 설정을 그대로 둔 고속 충격 시험 입력이다.
                player.GetMesh().GetRagdoll().AddVelocity(direction * 15.0f);
            }
            if (incoming && frame == 120)
            {
                // 플레이어 입력을 주지 않고 실제 더미가 다가와 충돌하는 조건을 검사한다.
                dummy->GetMesh().SetAutomaticRagdoll(false);
                check(dummy->SetPhysicsMode(PhysicsAnimationMode::PassiveRagdoll, error), "Launch passive receiver toward stationary player");
                dummy->GetMesh().GetRagdoll().AddVelocity(-direction * 6.0f);
            }
            world.Tick(1.0f / 60.0f);
            test.Update();
            const Ragdoll& playerRagdoll = player.GetMesh().GetRagdoll();
            const Vector3 playerRoot = player.GetActorLocation();
            PxTransform capsulePose(PxIdentity);
            PxVec3 targetOffset(0);
            float radius = 0;
            float halfLength = 0;
            playerRagdoll.GetCharacterImpactCapsule(impactSettings.capsuleRadiusScale, capsulePose, radius,
                halfLength, targetOffset, impactSettings.capsuleHeightScale);
            std::vector<PxDebugLine> debugLines;
            test.BuildLines(debugLines);
            size_t cyanLines = 0;
            for (const PxDebugLine& line : debugLines)
            {
                if (line.color0 == 0xff00ffff && line.color1 == 0xff00ffff)
                {
                    ++cyanLines;
                }
            }
            capsules << frame << ',' << (frame + 1) / 60.0f << ',' << static_cast<int>(player.GetMesh().GetPhysicsMode())
                << ',' << float(playerRoot.GetX()) << ',' << float(playerRoot.GetY()) << ',' << float(playerRoot.GetZ())
                << ',' << playerRagdoll.HasCharacterCapsule() << ',' << playerRagdoll.GetCharacterCapsuleMass()
                << ',' << radius << ',' << halfLength << ',' << capsulePose.p.x << ',' << capsulePose.p.y << ',' << capsulePose.p.z
                << ',' << cyanLines << '\n';
            Vector3 center(kZero);
            float totalMass = 0;
            check(player.GetMesh().GetRagdoll().GetCenterOfMass(center, totalMass) &&
                std::abs(totalMass - expectedTotalMass) < .0001f,
                "Independent collision mass is excluded from total PA mass throughout actual-character simulation");
            const float time = (frame + 1) / 60.0f;
            const SkeletalMeshComponent& dummyMesh = dummy->GetMesh();
            const FootSupportSnapshot& dummySupport = dummyMesh.GetFootSupport().GetSnapshot();
            states << frame << ',' << static_cast<int>(dummyMesh.GetPhysicsMode()) << ',' << dummyMesh.IsBlendingToKinematic()
                << ',' << static_cast<int>(dummySupport.balance.zone) << ',' << dummyMesh.GetKinematicStableTime()
                << ',' << dummySupport.footstep.startCount << ',' << dummy->GetMesh().GetRagdoll().GetExternalImpactImpulse() << '\n';
            for (const CharacterImpactSnapshot& impact : player.GetCharacterImpact().GetAppliedImpacts())
            {
                ++eventCount;
                for (const BodyImpactShare& share : impact.shares)
                {
                    events << frame << ',' << time << ',' << impact.eventCount << ',' << impact.hitBone << ',' << share.bone
                        << ',' << 0 << ',' << impact.normalImpulse << ',' << impact.closingSpeed
                        << ',' << impact.point.x << ',' << impact.point.y << ',' << impact.point.z
                        << ',' << share.impulse.x << ',' << share.impulse.y << ',' << share.impulse.z << ',' << 0 << '\n';
                }
            }
            for (Character* character : {&player, dummy})
            {
                SkeletalMeshComponent& mesh = character->GetMesh();
                Ragdoll& ragdoll = mesh.GetRagdoll();
                const PhysicsSkeleton& skeleton = character == &player ? playerSkeleton : dummySkeleton;
                std::vector<Matrix4> targets;
                check(ragdoll.IsFinite() && EvaluatePhysicsPose(skeleton, mesh.GetAnimInstance().GetPose(),
                    Matrix4(mesh.GetComponentTransform()), targets, error), "Finite bodies and valid animation targets");
                const FootSupportSnapshot& feet = mesh.GetFootSupport().GetSnapshot();
                for (size_t body = 0; body < ragdoll.GetBodyCount(); ++body)
                {
                    const Matrix4 actual = ragdoll.GetBoneWorld(body);
                    const Matrix4 target = PhysicsBoneFrame(targets[ragdoll.GetBoneNodes()[body]]);
                    const Vector3 position(actual.GetW());
                    DirectX::XMFLOAT4 rotation;
                    DirectX::XMStoreFloat4(&rotation, Quaternion(actual));
                    const float dot = std::abs(DirectX::XMVectorGetX(DirectX::XMQuaternionDot(Quaternion(actual), Quaternion(target))));
                    const float angle = 2 * std::acos(Clamp(dot, 0.0f, 1.0f));
                    const PxVec3 point(float(position.GetX()), float(position.GetY()), float(position.GetZ()));
                    samples << frame << ',' << time << ',' << (character == &player ? "player" : "dummy") << ','
                        << mesh.GetPhysicsAsset()->bodies[body].bone << ',' << point.x << ',' << point.y << ',' << point.z
                        << ',' << rotation.x << ',' << rotation.y << ',' << rotation.z << ',' << rotation.w
                        << ',' << float(Length(position - Vector3(target.GetW()))) << ',' << angle
                        << ',' << ragdoll.GetBodyPointVelocity(body, point).magnitude() << ',' << ragdoll.IsRootSupported()
                        << ',' << feet.feet[0].contact.grounded << ',' << feet.feet[1].contact.grounded
                        << ',' << ragdoll.GetMaxDriveError() << '\n';
                }
            }
        }
        controller->SetMoveInput(0, 0, false);
        world.SetPhysicsPaused(true);
        report << "events=" << eventCount << std::endl;
        check(world.GetPhysicsWorld()->GetErrorCount() == 0, "No PhysX errors");
        report << "PASS: actual-character body impact recording" << std::endl;
        return true;
    }
    catch (const std::exception& error)
    {
        report << "FAIL: " << error.what() << std::endl;
        return false;
    }
}

using namespace PARS;
using namespace Math;
using namespace physx;

bool RunRagdollImpactValidation(RagdollImpactTest& test, World& world, Character& player, const std::wstring& path)
{
    std::ofstream report(path);
    const std::function<void(bool, const char*)> check = [&](bool condition, const char* message) -> void
    {
        report << (condition ? "PASS: " : "FAIL: ") << message << " | " << test.GetStatusMessage() << std::endl;
        if (!condition)
        {
            throw std::runtime_error(message);
        }
    };
    try
    {
        // 저장된 사용자 설정과 관계없이 기존 충격 시험의 초기 조건을 유지한다.
        check(test.SetProjectileSettings(2.0f, 7.5f), "Use the original 2 kg / 7.5 m/s fixture independently of saved UI settings");
        check(test.SetAutomaticSupport(false), "Use manual support for the existing projection and projectile baseline");
        PARS::Character* dummy = test.GetDummy();
        check(dummy && dummy != &player && dummy->GetController() == nullptr,
            "Idle dummy is independent of the possessed player");
        check(dummy->GetMesh().IsActiveRagdoll() && dummy->GetMesh().GetRagdoll().IsRootSupported(),
            "Dummy is dynamic with angular drives and pelvis support enabled");
        const std::function<void()> tick = [&]() -> void
        {
            world.Tick(1.0f / 60);
            test.Update();
        };
        PARS::FootSupport& support = dummy->GetMesh().GetFootSupport();
        check(support.GetSnapshot().hasCenterOfMass && !support.GetSnapshot().hull.empty() &&
                !support.GetSnapshot().balance.bufferHull.empty(),
            "Reset prepares COM and A/B before the first physics step or render");
        const std::function<bool()> useFootOnlyProjectionFixture = [&]() -> bool
        {
            // 아래 사각형 오프셋 시험은 Toe가 없는 PA의 호환성을 검사한다.
            // Toe가 있으면 Foot 사각형을 옮겨도 Toe 투영은 실제 바디 아래에 남는 것이 정상이다.
            PhysicsAsset footOnly = *dummy->GetMesh().GetPhysicsAsset();
            for (size_t body = footOnly.bodies.size(); body-- > 0;)
            {
                const std::string& name = footOnly.bodies[body].bone;
                const size_t separator = name.find_last_of(':');
                const std::string leaf = name.substr(separator == std::string::npos ? 0 : separator + 1);
                if (leaf == "LeftToeBase" || leaf == "RightToeBase")
                {
                    footOnly.RemoveBody(body);
                }
            }
            std::string error;
            return dummy->GetMesh().SetPhysicsAsset(footOnly, error);
        };
        check(useFootOnlyProjectionFixture(), "Use an explicitly toe-less PA only for legacy rectangle projection checks");
        world.SetPhysicsPaused(true);
        FootSoleDebug debug;
        debug.SetVisible(false);
        std::vector<PxDebugLine> lines;
        const uint64_t pausedSample = support.GetSnapshot().sampleNumber;
        debug.BuildLines(dummy->GetMesh(), lines);
        tick();
        check(lines.empty() && support.GetSnapshot().sampleNumber == pausedSample,
            "Hidden debug and paused rendering do not change the physics snapshot");
        const uint64_t previousStep = world.GetPhysicsWorld()->GetStepCount();
        world.RequestPhysicsStep();
        tick();
        check(support.GetSnapshot().sampleNumber == pausedSample + 1 &&
                support.GetSnapshot().completedPhysicsStep == previousStep,
            "Single stepping samples the completed body state once even with debug hidden");
        debug.SetVisible(true);
        const uint64_t renderedSample = support.GetSnapshot().sampleNumber;
        debug.BuildLines(dummy->GetMesh(), lines);
        debug.BuildLines(dummy->GetMesh(), lines);
        bool hasBufferLines = false;
        for (const physx::PxDebugLine& line : lines)
        {
            hasBufferLines |= line.color0 == 0xffffffff;
        }
        check(hasBufferLines && support.GetSnapshot().sampleNumber == renderedSample,
            "Drawing B's white dashes reads the snapshot without running any queries");
        const PARS::FootSoleSettings leftSettings = support.GetSoleSettings(0);
        const PARS::FootSoleSettings rightSettings = support.GetSoleSettings(1);
        support.GetSoleSettings(0).offset.y += 1;
        world.RequestPhysicsStep();
        tick();
        check(support.GetSnapshot().projectedPointCount > 0 && support.GetSnapshot().projectedPointCount <= 4 &&
                !support.GetSnapshot().hull.empty(),
            "One valid foot still produces its own A/B region");
        support.GetSoleSettings(1).offset.y += 1;
        world.RequestPhysicsStep();
        tick();
        check(support.GetSnapshot().hull.empty() && support.GetSnapshot().balance.bufferHull.empty() &&
                support.GetSnapshot().balance.weight == 0 && !support.GetSnapshot().hasComProjection &&
                dummy->GetMesh().GetRagdoll().IsRootSupported(),
            "Missing support clears stale A/B and weight without changing pelvis support in observation mode");
        support.GetSoleSettings(0) = leftSettings;
        support.GetSoleSettings(1) = rightSettings;
        check(test.Reset() && !support.GetSnapshot().hull.empty(), "Reset immediately replaces the invalid snapshot");
        world.SetPhysicsPaused(false);
        for (int fps : {30, 120})
        {
            const uint64_t initialSample = support.GetSnapshot().sampleNumber;
            const uint64_t initialStep = world.GetPhysicsWorld()->GetStepCount();
            for (int frame = 0; frame < fps; ++frame)
            {
                world.Tick(1.0f / fps);
            }
            check(support.GetSnapshot().sampleNumber - initialSample == 60 &&
                    world.GetPhysicsWorld()->GetStepCount() - initialStep == 60,
                "30/120 FPS each produce exactly 60 support samples per simulated second");
        }
        float travel[2] = {};
        for (int trial = 0; trial < 2; ++trial)
        {
            check(test.Reset(), "Reset dummy to the same grounded Idle state");
            for (int frame = 0; frame < 120; ++frame)
            {
                tick();
            }
            const Math::Vector3 origin = dummy->GetMesh().GetRagdoll().GetPosition();
            if (trial == 1)
            {
                check(test.FireFrom(player), "Fire through the same command used by the R key");
                const std::vector<physx::PxRigidDynamic*> shots = test.GetProjectiles();
                check(shots.size() == 1 && std::abs(shots[0]->getMass() - 2.0f) < 1e-5f &&
                        std::abs(shots[0]->getLinearVelocity().magnitude() - 7.5f) < 1e-4f,
                    "Projectile has the configured 2 kg mass and 7.5 m/s initial speed");
                PxShape* shape = nullptr;
                check(shots[0]->getShapes(&shape, 1) == 1 && shape->getGeometry().getType() == PxGeometryType::eSPHERE &&
                        std::abs(static_cast<const PxSphereGeometry&>(shape->getGeometry()).radius - .15f) < 1e-5f,
                    "Projectile collision radius is 15 cm");
                check(shots[0]->getRigidBodyFlags().isSet(PxRigidBodyFlag::eENABLE_SPECULATIVE_CCD),
                    "Fast projectile uses speculative collision detection");
                // A weak interactive default need not exceed the supported Idle sway.
                // Keep this collision-response regression at its original, explicit strength.
                test.ClearProjectiles();
                check(test.SetProjectileSettings(50, 75) && test.FireFrom(player),
                    "Use a fixed 50 kg / 75 m/s collision-response fixture after checking the defaults");
            }
            for (int frame = 0; frame < 180; ++frame)
            {
                tick();
                travel[trial] = Max(travel[trial], float(Length(dummy->GetMesh().GetRagdoll().GetPosition() - origin)));
            }
        }
        report << "idle_travel_m=" << travel[0] << " projectile_travel_m=" << travel[1] << std::endl;
        check(travel[1] > travel[0] + .0001f, "Physical sphere impact moves the actual Hips beyond the Idle baseline");
        check(dummy->GetMesh().GetAnimInstance().GetAnimationCount() == 1 &&
                dummy->GetMesh().GetAnimInstance().GetCurrentAnimationIndex() == 0,
            "Dummy keeps the Idle animation after being hit");

        check(test.SetProjectileSettings(.5f, 7.5f) && test.Reset(), "Use the existing 0.5 kg fixture for camera aiming checks");
        PhysicsWorld* physics = world.GetPhysicsWorld();
        physx::PxVec3 (*const toPx)(Math::Vector3) = [](Vector3 v) -> physx::PxVec3
        {
            return PxVec3(float(v.GetX()), float(v.GetY()), float(v.GetZ()));
        };
        const Vector3 shotOrigin = player.GetActorLocation() + Vector3(0, .15f, 0);
        const Vector3 camera = shotOrigin + Vector3(0, 1, -4);
        const Vector3 target = shotOrigin + Vector3(6, 2, 6);
        const Vector3 aim = Normalize(target - camera);
        void (*const release)(physx::PxRigidStatic*) = [](PxRigidStatic* body) -> void
        {
            if (body)
            {
                body->release();
            }
        };
        std::unique_ptr<PxRigidStatic, decltype(release)> aimTarget(
            physics->CreateStaticBox(PxTransform(toPx(target)), PxVec3(.35f), "Aim validation target"), release);
        std::unique_ptr<PxRigidStatic, decltype(release)> ownBody(
            physics->CreateStaticBox(PxTransform(toPx(camera + aim * 2)), PxVec3(.2f), "Ignored player body"), release);
        check(aimTarget && ownBody, "Create aiming targets");
        ownBody->userData = &player;
        check(test.FireAimed(player, camera, aim), "Fire using an offset third-person camera ray");
        physx::PxRigidDynamic* aimedShot = test.GetProjectiles().back();
        const PxVec3 start = aimedShot->getGlobalPose().p;
        const PxVec3 direction = aimedShot->getLinearVelocity().getNormalized();
        const PxVec3 toTarget = toPx(target) - start;
        check((toTarget - direction * toTarget.dot(direction)).magnitude() < .35f && toTarget.dot(direction) > 0,
            "Shot converges on the crosshair target and ignores the player's own body");
        check((start - toPx(shotOrigin)).magnitude() < 1.0f &&
                std::abs(aimedShot->getLinearVelocity().magnitude() - 7.5f) < 1e-4f,
            "Aimed shot starts in front of the player at the configured speed");
        test.ClearProjectiles();
        ownBody.reset();
        aimTarget.reset();

        const Vector3 skyAim = Normalize(Vector3(-1, 1, 0));
        check(test.FireAimed(player, camera, skyAim), "Fire into empty space without snapping to the dummy");
        check(test.GetProjectiles().back()->getLinearVelocity().getNormalized().dot(toPx(skyAim)) > .999f,
            "Empty-space shot follows the camera heading, including upward pitch");
        test.ClearProjectiles();
        check(!test.FireAimed(player, camera, Vector3(kZero)) && test.GetProjectiles().empty(),
            "Invalid aim direction does not create a projectile");

        std::unique_ptr<PxRigidStatic, decltype(release)> muzzleWall(
            physics->CreateStaticBox(PxTransform(toPx(shotOrigin + Vector3(0, 0, .4f))),
                PxVec3(.3f, .25f, .05f), "Muzzle obstruction"), release);
        check(muzzleWall != nullptr, "Create a nearby muzzle obstruction");
        check(!test.FireAimed(player, camera, Vector3(kZUnitVector)) && test.GetProjectiles().empty(),
            "A visible camera target cannot fire a sphere through a blocked muzzle");
        muzzleWall.reset();

        test.ClearProjectiles();
        const uint32_t bodies = world.GetPhysicsWorld()->GetDynamicBodyCount();
        for (int shot = 0; shot < 13; ++shot)
        {
            check(test.FireFrom(player), "Repeat firing");
        }
        check(test.GetProjectiles().size() == 12 && world.GetPhysicsWorld()->GetDynamicBodyCount() == bodies + 12,
            "Repeated firing caps live projectile bodies at twelve");
        check(test.Reset() && test.GetProjectiles().empty() && world.GetPhysicsWorld()->GetDynamicBodyCount() == bodies,
            "Reset releases projectiles and restores the dynamic dummy without body leaks");
        check(test.SetAutomaticSupport(true) && dummy->GetMesh().UsesAutomaticRootSupport() &&
                !player.GetMesh().UsesAutomaticRootSupport(),
            "Enable COM support on the dummy only");
        Ragdoll& ragdoll = dummy->GetMesh().GetRagdoll();
        check(!dummy->GetMesh().IsRagdoll() && dummy->GetMesh().UsesAutomaticRagdoll(),
            "Automatic dummy now waits in kinematic mode before an impact");
        // 아래 기존 검증은 항상 동적인 다리 지지 제어의 계약을 같은 입력으로 검사한다.
        // 새 자동 모드의 기본 동작과 복귀는 별도의 실제 캐릭터 전환 검증에서 확인한다.
        const std::function<bool()> resetActive = [&]() -> bool
        {
            if (!test.Reset())
            {
                return false;
            }
            SkeletalMeshComponent& mesh = dummy->GetMesh();
            mesh.SetAutomaticRagdoll(false);
            std::string error;
            return dummy->SetPhysicsMode(PhysicsAnimationMode::ActiveRagdoll, error) &&
                ragdoll.SetRootSupported(true, error) && mesh.SetAutomaticRootSupport(true, error);
        };
        check(resetActive(), "Select continuous active mode for the original support-controller regression");
        check(ragdoll.UsesFootDrivenSupport() && !ragdoll.IsRootAxisSupported(0) && !ragdoll.IsRootAxisSupported(1) &&
                !ragdoll.IsRootAxisSupported(2) && ragdoll.IsRootAxisSupported(3),
            "Automatic Idle uses leg support, frees root translation and keeps rotation assistance");
        check(useFootOnlyProjectionFixture(), "Use the toe-less projection fixture for synthetic rectangle displacement");

        // Deterministically exercise both release causes, and re-entry without rearming.
        world.SetPhysicsPaused(true);
        const std::function<void()> singleStep = [&]() -> void
        {
            world.RequestPhysicsStep();
            tick();
        };
        const PARS::FootSoleSettings originalLeft = support.GetSoleSettings(0);
        const PARS::FootSoleSettings originalRight = support.GetSoleSettings(1);
        support.GetSoleSettings(0).offset.x += 2;
        support.GetSoleSettings(1).offset.x += 2;
        singleStep();
        check(support.GetSnapshot().balance.zone == FootSupportZone::Outside &&
                dummy->GetMesh().GetRootSupportRelease() == RootSupportRelease::OutsideBuffer && !ragdoll.IsRootSupported(),
            "Reaching B releases support in the same fixed step");
        support.GetSoleSettings(0) = originalLeft;
        support.GetSoleSettings(1) = originalRight;
        singleStep();
        check(support.GetSnapshot().balance.weight > 0 && ragdoll.GetRootSupportWeight() == 0 &&
                !ragdoll.IsRootSupported() && ragdoll.AreDrivesEnabled(),
            "COM re-entry cannot re-enable pelvis support; ordinary joint PD remains enabled");
        std::string supportError;
        check(dummy->GetMesh().SetAutomaticRootSupport(true, supportError) && !ragdoll.IsRootSupported(),
            "Repeating automatic mode enable does not clear the release latch");
        check(resetActive() && ragdoll.GetRootSupportWeight() == 1 &&
                dummy->GetMesh().GetRootSupportRelease() == RootSupportRelease::None,
            "Dummy reset rearms support from its newly sampled grounded pose");
        check(useFootOnlyProjectionFixture(), "Use the toe-less projection fixture for synthetic missing rays");
        support.GetSoleSettings(0).offset.y += 1;
        support.GetSoleSettings(1).offset.y += 1;
        singleStep();
        check(dummy->GetMesh().GetRootSupportRelease() == RootSupportRelease::NoSupport && !ragdoll.IsRootSupported(),
            "Invalid foot support fully releases Y/rotation support");
        support.GetSoleSettings(0) = originalLeft;
        support.GetSoleSettings(1) = originalRight;
        check(resetActive(), "Restore the automatic impact fixture");
        world.SetPhysicsPaused(false);

        float idleMinimumWeight = 1;
        float idleFootForce = 0;
        for (int frame = 0; frame < 600; ++frame)
        {
            tick();
            idleMinimumWeight = PxMin(idleMinimumWeight, ragdoll.GetRootSupportWeight());
            if (frame >= 480)
            {
                const FootSupportSnapshot& feet = support.GetSnapshot();
                idleFootForce += (feet.feet[0].contact.normalImpulse + feet.feet[1].contact.normalImpulse) * 60.0f / 120.0f;
                check(PxAbs(float(ragdoll.GetRootSupportForce().GetY())) < 1e-5f,
                    "Automatic leg support supplies no external pelvis Y force");
            }
        }
        report << "automatic_idle_min_weight=" << idleMinimumWeight << std::endl;
        report << "automatic_idle_mean_foot_force=" << idleFootForce << std::endl;
        check(idleMinimumWeight > .99f && ragdoll.IsRootSupported(), "Automatic Idle stays supported for ten seconds");
        const FootSupportSnapshot& idleSupport = support.GetSnapshot();
        check(dummy->GetMesh().GetStandingSupportSnapshot().valid && idleSupport.feet[0].contact.grounded &&
            idleSupport.feet[1].contact.grounded && std::abs(idleFootForce - idleSupport.totalMass * 9.81f) < idleSupport.totalMass,
            "Both feet support body weight through floor contact in automatic Idle");
        const float masses[] = {.1f, 2.0f, 50.0f}, speeds[] = {30.0f, 30.0f, 75.0f};
        bool sawBand = false, sawFall = false;
        for (int trial = 0; trial < 3; ++trial)
        {
            check(resetActive() && test.SetProjectileSettings(masses[trial], speeds[trial]), "Prepare a reproducible automatic impact");
            for (int frame = 0; frame < 120; ++frame)
            {
                tick();
            }
            const float initialHeight = float(ragdoll.GetPosition().GetY());
            check(test.FireFrom(player), "Shoot the automatic support dummy");
            float minimumWeight = 1, minimumHeight = initialHeight;
            int bandFrames = 0, releaseFrame = -1;
            bool matched = true, stayedReleased = true, finiteAndDriven = true;
            for (int frame = 0; frame < 300; ++frame)
            {
                tick();
                const float raw = support.GetSnapshot().balance.weight, applied = ragdoll.GetRootSupportWeight();
                minimumWeight = PxMin(minimumWeight, applied);
                minimumHeight = PxMin(minimumHeight, float(ragdoll.GetPosition().GetY()));
                if (applied > 0 && applied < 1)
                {
                    ++bandFrames;
                    matched &= std::abs(raw - applied) < 1e-6f;
                }
                if (!ragdoll.IsRootSupported() && releaseFrame < 0)
                {
                    releaseFrame = frame;
                }
                if (releaseFrame >= 0)
                {
                    stayedReleased &= applied == 0;
                }
                finiteAndDriven &= ragdoll.IsFinite() && ragdoll.AreDrivesEnabled();
            }
            report << "automatic_impact mass=" << masses[trial] << " speed=" << speeds[trial]
                << " min_weight=" << minimumWeight << " band_frames=" << bandFrames << " release_frame=" << releaseFrame
                << " min_hips_y=" << minimumHeight << " max_displacement=" << test.GetMaxHipsDisplacement() << std::endl;
            check(matched && stayedReleased && finiteAndDriven,
                "Impact follows the weight, preserves the release latch and keeps finite driven bodies");
            if (trial == 0)
            {
                check(releaseFrame < 0 && minimumWeight > .99f,
                    "A weak physical impact stays inside A without releasing support");
            }
            sawBand |= bandFrames > 0;
            sawFall |= releaseFrame >= 0 && minimumHeight < initialHeight - .35f;
        }
        check(sawBand && sawFall, "Physical impacts exercise gradual support loss and a subsequent fall");
        check(test.SetProjectileSettings(.5f, 7.5f) && resetActive() && ragdoll.GetRootSupportWeight() == 1,
            "Restore the 0.5 kg capture fixture and clear the fallen state");
        check(world.GetPhysicsWorld()->GetErrorCount() == 0, "No PhysX errors in the impact test");
        check(test.FireFrom(player), "Leave one sphere visible for the optional GPU capture");
        world.SetPhysicsPaused(true);
        report << "PASS: all dummy projectile integration checks" << std::endl;
        return true;
    }
    catch (const std::exception& error)
    {
        report << "FAIL: " << error.what() << std::endl;
        return false;
    }
}
