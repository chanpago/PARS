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
        auto* dummy = test.GetDummy();
        check(dummy && dummy != &player && dummy->GetController() == nullptr,
            "Idle dummy is independent of the possessed player");
        check(dummy->GetMesh().IsActiveRagdoll() && dummy->GetMesh().GetRagdoll().IsRootSupported(),
            "Dummy is dynamic with angular drives and pelvis support enabled");
        const auto tick = [&]() { world.Tick(1.0f / 60); test.Update(); };
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
                check(shots.size() == 1 && std::abs(shots[0]->getMass() - 50) < 1e-5f &&
                    std::abs(shots[0]->getLinearVelocity().magnitude() - 75) < 1e-4f,
                    "Projectile has the configured 50 kg mass and 75 m/s initial speed");
                PxShape* shape = nullptr;
                check(shots[0]->getShapes(&shape, 1) == 1 && shape->getGeometry().getType() == PxGeometryType::eSPHERE &&
                    std::abs(static_cast<const PxSphereGeometry&>(shape->getGeometry()).radius - .15f) < 1e-5f,
                    "Projectile collision radius is 15 cm");
                check(shots[0]->getRigidBodyFlags().isSet(PxRigidBodyFlag::eENABLE_SPECULATIVE_CCD),
                    "Fast projectile uses speculative collision detection");
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

        check(test.Reset(), "Reset before camera aiming checks");
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
            std::abs(aimedShot->getLinearVelocity().magnitude() - 75) < 1e-4f,
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
