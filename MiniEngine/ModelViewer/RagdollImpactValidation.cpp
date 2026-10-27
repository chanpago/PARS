#include "RagdollImpactTest.h"
#include "GameFramework/Character.h"
#include "GameFramework/World.h"
#include <fstream>
#include <stdexcept>
#include <cmath>

using namespace PARS;
using namespace Math;
using namespace physx;

bool RunRagdollImpactValidation(RagdollImpactTest& test, World& world, Character& player, const std::wstring& path)
{
    std::ofstream report(path);
    const auto check = [&](bool condition, const char* message)
    {
        report << (condition ? "PASS: " : "FAIL: ") << message << " | " << test.GetStatusMessage() << std::endl;
        if (!condition) throw std::runtime_error(message);
    };
    try
    {
        check(test.SetAutomaticSupport(false), "Use manual support for the existing projection and projectile baseline");
        auto* dummy = test.GetDummy();
        check(dummy && dummy != &player && dummy->GetController() == nullptr,
            "Idle dummy is independent of the possessed player");
        check(dummy->GetMesh().IsActiveRagdoll() && dummy->GetMesh().GetRagdoll().IsRootSupported(),
            "Dummy is dynamic with angular drives and pelvis support enabled");
        const auto tick = [&]() { world.Tick(1.0f / 60); test.Update(); };
        auto& support = dummy->GetMesh().GetFootSupport();
        check(support.GetSnapshot().hasCenterOfMass && !support.GetSnapshot().hull.empty() &&
            !support.GetSnapshot().balance.bufferHull.empty(),
            "Reset prepares COM and A/B before the first physics step or render");
        world.SetPhysicsPaused(true);
        FootSoleDebug debug;
        debug.SetVisible(false);
        std::vector<PxDebugLine> lines;
        const auto pausedSample = support.GetSnapshot().sampleNumber;
        debug.BuildLines(dummy->GetMesh(), lines);
        tick();
        check(lines.empty() && support.GetSnapshot().sampleNumber == pausedSample,
            "Hidden debug and paused rendering do not change the physics snapshot");
        const auto previousStep = world.GetPhysicsWorld()->GetStepCount();
        world.RequestPhysicsStep();
        tick();
        check(support.GetSnapshot().sampleNumber == pausedSample + 1 &&
            support.GetSnapshot().completedPhysicsStep == previousStep,
            "Single stepping samples the completed body state once even with debug hidden");
        debug.SetVisible(true);
        const auto renderedSample = support.GetSnapshot().sampleNumber;
        debug.BuildLines(dummy->GetMesh(), lines);
        debug.BuildLines(dummy->GetMesh(), lines);
        bool hasBufferLines = false;
        for (const auto& line : lines) hasBufferLines |= line.color0 == 0xffffffff;
        check(hasBufferLines && support.GetSnapshot().sampleNumber == renderedSample,
            "Drawing B's white dashes reads the snapshot without running any queries");
        const auto leftSettings = support.GetSoleSettings(0), rightSettings = support.GetSoleSettings(1);
        support.GetSoleSettings(0).offset.y += 1;
        world.RequestPhysicsStep();
        tick();
        check(support.GetSnapshot().projectedPointCount > 0 && support.GetSnapshot().projectedPointCount <= 4 &&
            !support.GetSnapshot().hull.empty(), "One valid foot still produces its own A/B region");
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
            const auto initialSample = support.GetSnapshot().sampleNumber;
            const auto initialStep = world.GetPhysicsWorld()->GetStepCount();
            for (int frame = 0; frame < fps; ++frame) world.Tick(1.0f / fps);
            check(support.GetSnapshot().sampleNumber - initialSample == 60 &&
                world.GetPhysicsWorld()->GetStepCount() - initialStep == 60,
                "30/120 FPS each produce exactly 60 support samples per simulated second");
        }
        float travel[2] = {};
        for (int trial = 0; trial < 2; ++trial)
        {
            check(test.Reset(), "Reset dummy to the same grounded Idle state");
            for (int frame = 0; frame < 120; ++frame) tick();
            const auto origin = dummy->GetMesh().GetRagdoll().GetPosition();
            if (trial == 1)
            {
                check(test.FireFrom(player), "Fire through the same command used by the R key");
                const auto shots = test.GetProjectiles();
                check(shots.size() == 1 && std::abs(shots[0]->getMass() - .5f) < 1e-5f &&
                    std::abs(shots[0]->getLinearVelocity().magnitude() - 7.5f) < 1e-4f,
                    "Projectile has the configured 0.5 kg mass and 7.5 m/s initial speed");
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

        check(test.SetProjectileSettings(.5f, 7.5f) && test.Reset(), "Restore defaults before camera aiming checks");
        auto* physics = world.GetPhysicsWorld();
        const auto toPx = [](Vector3 v) { return PxVec3(float(v.GetX()), float(v.GetY()), float(v.GetZ())); };
        const Vector3 shotOrigin = player.GetActorLocation() + Vector3(0, .15f, 0);
        const Vector3 camera = shotOrigin + Vector3(0, 1, -4);
        const Vector3 target = shotOrigin + Vector3(6, 2, 6);
        const Vector3 aim = Normalize(target - camera);
        const auto release = [](PxRigidStatic* body) { if (body) body->release(); };
        std::unique_ptr<PxRigidStatic, decltype(release)> aimTarget(
            physics->CreateStaticBox(PxTransform(toPx(target)), PxVec3(.35f), "Aim validation target"), release);
        std::unique_ptr<PxRigidStatic, decltype(release)> ownBody(
            physics->CreateStaticBox(PxTransform(toPx(camera + aim * 2)), PxVec3(.2f), "Ignored player body"), release);
        check(aimTarget && ownBody, "Create aiming targets");
        ownBody->userData = &player;
        check(test.FireAimed(player, camera, aim), "Fire using an offset third-person camera ray");
        auto* aimedShot = test.GetProjectiles().back();
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
        const auto bodies = world.GetPhysicsWorld()->GetDynamicBodyCount();
        for (int shot = 0; shot < 13; ++shot) check(test.FireFrom(player), "Repeat firing");
        check(test.GetProjectiles().size() == 12 && world.GetPhysicsWorld()->GetDynamicBodyCount() == bodies + 12,
            "Repeated firing caps live projectile bodies at twelve");
        check(test.Reset() && test.GetProjectiles().empty() && world.GetPhysicsWorld()->GetDynamicBodyCount() == bodies,
            "Reset releases projectiles and restores the dynamic dummy without body leaks");
        check(test.SetAutomaticSupport(true) && dummy->GetMesh().UsesAutomaticRootSupport() &&
            !player.GetMesh().UsesAutomaticRootSupport(), "Enable COM support on the dummy only");
        auto& ragdoll = dummy->GetMesh().GetRagdoll();
        check(!ragdoll.IsRootAxisSupported(0) && ragdoll.IsRootAxisSupported(1) &&
            !ragdoll.IsRootAxisSupported(2) && ragdoll.IsRootAxisSupported(3),
            "Automatic support releases X/Z and keeps Y/rotation available");

        // Deterministically exercise both release causes, and re-entry without rearming.
        world.SetPhysicsPaused(true);
        const auto singleStep = [&]() { world.RequestPhysicsStep(); tick(); };
        const auto originalLeft = support.GetSoleSettings(0), originalRight = support.GetSoleSettings(1);
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
        check(test.Reset() && ragdoll.GetRootSupportWeight() == 1 &&
            dummy->GetMesh().GetRootSupportRelease() == RootSupportRelease::None,
            "Dummy reset rearms support from its newly sampled grounded pose");
        support.GetSoleSettings(0).offset.y += 1;
        support.GetSoleSettings(1).offset.y += 1;
        singleStep();
        check(dummy->GetMesh().GetRootSupportRelease() == RootSupportRelease::NoSupport && !ragdoll.IsRootSupported(),
            "Invalid foot support fully releases Y/rotation support");
        support.GetSoleSettings(0) = originalLeft;
        support.GetSoleSettings(1) = originalRight;
        check(test.Reset(), "Restore the automatic impact fixture");
        world.SetPhysicsPaused(false);

        float idleMinimumWeight = 1;
        for (int frame = 0; frame < 600; ++frame)
        {
            tick();
            idleMinimumWeight = PxMin(idleMinimumWeight, ragdoll.GetRootSupportWeight());
        }
        report << "automatic_idle_min_weight=" << idleMinimumWeight << std::endl;
        check(idleMinimumWeight > .99f && ragdoll.IsRootSupported(), "Automatic Idle stays supported for ten seconds");
        const float masses[] = {.1f, 2.0f, 50.0f}, speeds[] = {30.0f, 30.0f, 75.0f};
        bool sawBand = false, sawFall = false;
        for (int trial = 0; trial < 3; ++trial)
        {
            check(test.Reset() && test.SetProjectileSettings(masses[trial], speeds[trial]), "Prepare a reproducible automatic impact");
            for (int frame = 0; frame < 120; ++frame) tick();
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
                if (!ragdoll.IsRootSupported() && releaseFrame < 0) releaseFrame = frame;
                if (releaseFrame >= 0) stayedReleased &= applied == 0;
                finiteAndDriven &= ragdoll.IsFinite() && ragdoll.AreDrivesEnabled();
            }
            report << "automatic_impact mass=" << masses[trial] << " speed=" << speeds[trial]
                << " min_weight=" << minimumWeight << " band_frames=" << bandFrames << " release_frame=" << releaseFrame
                << " min_hips_y=" << minimumHeight << " max_displacement=" << test.GetMaxHipsDisplacement() << std::endl;
            check(matched && stayedReleased && finiteAndDriven,
                "Impact follows the weight, preserves the release latch and keeps finite driven bodies");
            if (trial == 0) check(releaseFrame < 0 && minimumWeight > .99f,
                "A weak physical impact stays inside A without releasing support");
            sawBand |= bandFrames > 0;
            sawFall |= releaseFrame >= 0 && minimumHeight < initialHeight - .35f;
        }
        check(sawBand && sawFall, "Physical impacts exercise gradual support loss and a subsequent fall");
        check(test.SetProjectileSettings(.5f, 7.5f) && test.Reset() && ragdoll.GetRootSupportWeight() == 1,
            "Restore defaults and clear the fallen state for interactive testing");
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
