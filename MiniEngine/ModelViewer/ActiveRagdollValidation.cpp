// Included in the opt-in -ragdolltest path; no synthetic skeleton replaces the imported character.
#include "GameFramework/World.h"
#include "GameFramework/Character.h"
#include "PhysicsWorld.h"
#include "PreviewAnimInstance.h"
#include <ostream>
#include <stdexcept>
#include <chrono>
#include <cmath>

using namespace Math;
using namespace PARS;

void RunActiveRagdollValidation(World& world, Character& character, std::ostream& report)
{
    std::string error;
    const auto check = [&](bool value, const char* message) {
        report << (value ? "PASS: " : "FAIL: ") << message << " | " << error << std::endl;
        if (!value) throw std::runtime_error(message);
    };
    auto& mesh = character.GetMesh(); auto& ragdoll = mesh.GetRagdoll();
    auto& animation = static_cast<PreviewAnimInstance&>(mesh.GetAnimInstance());
    auto& physics = *world.GetPhysicsWorld();
    const auto originalAsset = *mesh.GetPhysicsAsset();
    const auto start = character.GetActorLocation();
    const auto bodyCount = physics.GetDynamicBodyCount(), jointCount = physics.GetConstraintCount();
    PhysicsSkeleton skeleton;
    check(mesh.BuildPhysicsSkeleton(skeleton, error), "Active: imported skeleton bindings");
    const auto samePose = [](const Matrix4& a, const Matrix4& b) {
        return float(Length(Vector3(a.GetW()) - Vector3(b.GetW()))) < .0001f &&
            float(Length(Vector3(a.GetX()) - Vector3(b.GetX()))) < .0001f &&
            float(Length(Vector3(a.GetY()) - Vector3(b.GetY()))) < .0001f;
    };
    const auto pelvisMatchesAnimation = [&]() {
        std::vector<Matrix4> target;
        if (!EvaluatePhysicsPose(skeleton, animation.GetPose(), Matrix4(mesh.GetComponentTransform()), target, error)) return false;
        return samePose(ragdoll.GetBoneWorld(0), PhysicsBoneFrame(target[ragdoll.GetBoneNodes()[0]]));
    };
    const auto prepare = [&](uint32_t clip) {
        check(character.StopRagdoll(error), "Active: return to animation");
        character.SetActorLocation(start);
        animation.SetLocomotionEnabled(false); animation.SetAutoCycleEnabled(false);
        animation.PlayAnimation(clip);
        physics.ResetClock();
        for (int i = 0; i < 24; ++i) world.Tick(1.0f / 60);
        // Isolate joint tracking from floor obstruction: a supported, hanging
        // character can reach the animation targets. Test floor contact separately.
        character.SetActorLocation(character.GetActorLocation() + Vector3(0, 1, 0));
        world.Tick(0);
        check(character.SetPhysicsMode(PhysicsAnimationMode::ActiveRagdoll, error) && ragdoll.SetRootSupported(true, error),
            "Active: enter supported dynamic drive mode");
        check(ragdoll.GetDrivenJointCount() == 19 && ragdoll.GetKinematicBodyCount() == 1,
            "Active: 19 driven joints and one independently supported pelvis");
    };
    for (uint32_t clip = 0; clip < 3; ++clip)
    {
        prepare(clip);
        const auto initialPelvis = ragdoll.GetBoneWorld(0);
        const float initialTime = animation.GetAnimationTime();
        world.Tick(1.0f / 60);
        check(animation.GetAnimationTime() != initialTime, "Active: target animation continues during physics");
        float maxError = 0, maxGap = 0, maxLimit = 0;
        float pelvisTravel = 0, pelvisRotation = 0;
        size_t worst = 0; int worstStep = 0;
        double elapsedMs = 0;
        for (int step = 0; step < 360; ++step)
        {
            const auto before = std::chrono::steady_clock::now();
            world.Tick(1.0f / 60);
            elapsedMs += std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - before).count();
            if (!ragdoll.IsFinite()) check(false, "Active: finite dynamic state");
            if (!pelvisMatchesAnimation()) check(false, "Active: supported pelvis matches sampled animation after every fixed step");
            const auto pelvis = ragdoll.GetBoneWorld(0);
            pelvisTravel = Max(pelvisTravel, float(Length(Vector3(pelvis.GetW()) - Vector3(initialPelvis.GetW()))));
            pelvisRotation = Max(pelvisRotation, Max(float(Length(Vector3(pelvis.GetX()) - Vector3(initialPelvis.GetX()))),
                float(Length(Vector3(pelvis.GetY()) - Vector3(initialPelvis.GetY())))));
            if (step > 60)
            {
                size_t current = 0; const float currentError = ragdoll.GetMaxDriveError(&current);
                if (currentError > maxError) { maxError = currentError; worst = current; worstStep = step; }
                maxGap = Max(maxGap, ragdoll.GetMaxAnchorError());
                maxLimit = Max(maxLimit, ragdoll.GetMaxAngularError());
            }
        }
        report << "active_clip=" << clip << " tick_avg_ms=" << elapsedMs / 360 << " max_target_error_rad=" << maxError
            << " max_gap_m=" << maxGap << " max_limit_error_rad=" << maxLimit
            << " target_clamp_rad=" << ragdoll.GetMaxTargetClampAngle() << " PVD=" << physics.IsPvdConnected() << std::endl;
        report << "active_worst=" << originalAsset.constraints[worst].childBone << " step=" << worstStep << std::endl;
        report << "active_pelvis=" << clip << " travel_m=" << pelvisTravel << " rotation_basis_delta=" << pelvisRotation << std::endl;
        check(pelvisTravel > .0001f && pelvisRotation > .0001f && ragdoll.GetKinematicBodyCount() == 1,
            "Active: only pelvis is kinematic and follows animated translation and rotation instead of holding the entry pose");
        check(maxError < .35f && maxGap < .08f && maxLimit < .35f, "Active: animated tracking, anchors and limits within declared tolerances");
        // Hold a real sampled pose, disturb the physical body, and let motors recover.
        animation.SetPlaying(false);
        for (int i = 0; i < 180; ++i) world.Tick(1.0f / 60);
        const float restError = ragdoll.GetMaxDriveError();
        const float frozenTime = animation.GetAnimationTime();
        const auto targetBeforePush = animation.GetPose();
        ragdoll.AddVelocity(Vector3(0, 1, 2));
        float peak = 0;
        for (int i = 0; i < 240; ++i)
        {
            world.Tick(1.0f / 60);
            peak = Max(peak, ragdoll.GetMaxDriveError());
        }
        report << "active_recovery=" << clip << " rest=" << restError << " peak=" << peak << " final=" << ragdoll.GetMaxDriveError() << std::endl;
        check(peak > restError + .01f && ragdoll.GetMaxDriveError() < .35f, "Active: impulse response and static target recovery");
        check(animation.GetAnimationTime() == frozenTime && targetBeforePush.size() == animation.GetPose().size(), "Active: physics does not advance or replace a frozen target");
        for (size_t i = 0; i < targetBeforePush.size(); ++i)
            check(float(Length(Vector3(targetBeforePush[i].xform.GetW()) - Vector3(animation.GetPose()[i].xform.GetW()))) < 1e-6f &&
                std::abs(XMVectorGetX(XMVector4Dot(targetBeforePush[i].rotation, animation.GetPose()[i].rotation))) > .99999f,
                "Active: target bone is independent of physical disturbance");
        auto& render = GraphicsContext::Begin(L"Validate active physical render pose");
        check(mesh.UpdateRenderData(render), "Active: DX12 pose upload"); render.Finish(true);
        for (size_t i = 0; i < ragdoll.GetBodyCount(); ++i)
        {
            const auto node = ragdoll.GetBoneNodes()[i]; Matrix4 skin, owner;
            mesh.GetModelInstance().GetNodeTransform(node, skin);
            mesh.GetModelInstance().GetNodeTransform(skeleton.meshNodes[node], owner);
            const auto actual = PhysicsBoneFrame(owner * skin), physical = ragdoll.GetBoneWorld(i);
            check(float(Length(Vector3(actual.GetW()) - Vector3(physical.GetW()))) < .001f &&
                float(Length(Vector3(actual.GetX()) - Vector3(physical.GetX()))) < .001f,
                "Active: renderer uses physical bone transforms");
        }
        std::vector<Matrix4> beforeSwitch;
        for (size_t i = 0; i < ragdoll.GetBodyCount(); ++i) beforeSwitch.push_back(ragdoll.GetBoneWorld(i));
        check(character.SetPhysicsMode(PhysicsAnimationMode::PassiveRagdoll, error) && !ragdoll.AreDrivesEnabled() &&
            !ragdoll.IsRootSupported() && ragdoll.GetKinematicBodyCount() == 0,
            "Active: passive switch disables motors and releases the pelvis into full-body dynamics");
        for (size_t i = 0; i < beforeSwitch.size(); ++i)
            check(samePose(beforeSwitch[i], ragdoll.GetBoneWorld(i)), "Active: passive switch preserves every body pose, including pelvis");
        check(character.SetPhysicsMode(PhysicsAnimationMode::ActiveRagdoll, error) && !ragdoll.IsRootSupported() &&
            physics.GetDynamicBodyCount() == bodyCount && physics.GetConstraintCount() == jointCount,
            "Active: repeated mode switches preserve object counts without automatically reattaching pelvis support");
        for (size_t i = 0; i < beforeSwitch.size(); ++i)
            check(samePose(beforeSwitch[i], ragdoll.GetBoneWorld(i)), "Active: reactivation preserves every body pose");
        check(ragdoll.SetRootSupported(true, error), "Active: explicitly restore pelvis support for animated-pelvis checks");
        animation.SetPlaying(true);
        world.SetPhysicsPaused(true);
        const auto pausedPelvis = ragdoll.GetBoneWorld(0);
        const float time = animation.GetAnimationTime(); const auto steps = physics.GetStepCount();
        auto disabled = originalAsset;
        for (auto& joint : disabled.constraints) for (auto& drive : joint.drives) drive.enabled = false;
        check(mesh.QueueDriveSettings(disabled, error), "Active: queue drive edits while paused");
        world.Tick(1);
        check(physics.GetStepCount() == steps && animation.GetAnimationTime() == time && mesh.HasPendingDriveSettings() &&
            samePose(pausedPelvis, ragdoll.GetBoneWorld(0)), "Active: pause holds animation, pelvis, physics and queued drive settings");
        world.RequestPhysicsStep(); world.Tick(0);
        check(physics.GetStepCount() == steps + 1 && animation.GetAnimationTime() != time &&
            !mesh.HasPendingDriveSettings() && ragdoll.GetDrivenJointCount() == 0 && pelvisMatchesAnimation(),
            "Active: single step advances the animated pelvis even with angular drives disabled");
        check(mesh.QueueDriveSettings(originalAsset, error), "Active: queue original drives");
        world.SetPhysicsPaused(false); world.Tick(1.0f / 60);
        check(ragdoll.GetDrivenJointCount() == 19, "Active: runtime drive settings restore without recreation");
        const float releaseHeight = float(ragdoll.GetBoneWorld(0).GetW().GetY());
        check(character.SetPhysicsMode(PhysicsAnimationMode::PassiveRagdoll, error) && ragdoll.GetKinematicBodyCount() == 0,
            "Active: passive switch releases the animated pelvis");
        for (int i = 0; i < 20; ++i)
        {
            world.Tick(1.0f / 60);
            if (!ragdoll.IsFinite() || ragdoll.GetMaxAnchorError() >= .08f) check(false, "Passive: released character remains finite and connected");
        }
        check(float(ragdoll.GetBoneWorld(0).GetW().GetY()) < releaseHeight - .05f &&
            !ragdoll.IsRootSupported() && ragdoll.GetKinematicBodyCount() == 0,
            "Passive: pelvis falls under gravity instead of staying at the last animated pose");
        check(character.SetPhysicsMode(PhysicsAnimationMode::ActiveRagdoll, error) && !ragdoll.IsRootSupported(),
            "Active: enable motors on the falling character without restoring support");
        for (int i = 0; i < 180; ++i)
        {
            world.Tick(1.0f / 60);
            if (!ragdoll.IsFinite() || ragdoll.GetMaxAnchorError() >= .08f) check(false, "Active: unsupported contact remains finite and connected");
        }
        check(ragdoll.GetKinematicBodyCount() == 0 && ragdoll.GetLowestPoint() > -.04f,
            "Active: unsupported character collides with floor");
        check(character.SetPhysicsMode(PhysicsAnimationMode::PassiveRagdoll, error), "Active: disable motors after floor contact");
        for (int i = 0; i < 60; ++i) world.Tick(1.0f / 60);
        const auto fallen = ragdoll.GetBoneWorld(0);
        check(character.SetPhysicsMode(PhysicsAnimationMode::ActiveRagdoll, error) && !ragdoll.IsRootSupported() &&
            float(Length(Vector3(fallen.GetW()) - Vector3(ragdoll.GetBoneWorld(0).GetW()))) < 1e-6f,
            "Active: enable motors from a fallen pose without teleporting or adding support");
        for (int i = 0; i < 120; ++i)
        {
            world.Tick(1.0f / 60);
            if (!ragdoll.IsFinite() || ragdoll.GetMaxAnchorError() >= .08f)
                check(false, "Active: fallen-pose activation stays finite and connected");
        }
    }

    std::vector<Matrix4> reference;
    float referenceTime = 0;
    for (int fps : {30, 60, 144})
    {
        prepare(1); physics.ResetClock();
        for (int frame = 0; frame < fps * 2; ++frame) world.Tick(1.0f / fps);
        check(physics.GetStepCount() == 120, "Active: equal fixed-step count at 30/60/144 FPS");
        if (reference.empty())
        {
            referenceTime = animation.GetAnimationTime();
            for (size_t i = 0; i < ragdoll.GetBodyCount(); ++i) reference.push_back(ragdoll.GetBoneWorld(i));
        }
        else
        {
            check(std::abs(referenceTime - animation.GetAnimationTime()) < .0001f, "Active: target time is render-FPS independent");
            for (size_t i = 0; i < reference.size(); ++i)
                check(float(Length(Vector3(reference[i].GetW()) - Vector3(ragdoll.GetBoneWorld(i).GetW()))) < .03f &&
                    float(Length(Vector3(reference[i].GetX()) - Vector3(ragdoll.GetBoneWorld(i).GetX()))) < .05f,
                    "Active: physical tracking is render-FPS independent");
        }
    }
    prepare(0);
    for (uint32_t next : {1u, 2u, 0u})
    {
        check(animation.SelectAnimation(next), "Active: select a new target clip while simulated");
        float settledError = 0;
        for (int i = 0; i < 180; ++i)
        {
            world.Tick(1.0f / 60);
            if (!ragdoll.IsFinite() || ragdoll.GetMaxAnchorError() >= .08f)
                check(false, "Active: target clip transition stays finite and connected");
            if (i >= 150) settledError = Max(settledError, ragdoll.GetMaxDriveError());
        }
        check(animation.GetCurrentAnimationIndex() == next && settledError < .35f,
            "Active: blended target clip switch settles within tracking tolerance");
    }
    prepare(1); physics.ResetClock();
    const float slowStart = animation.GetAnimationTime();
    world.SetPhysicsTimeScale(.5f);
    for (int i = 0; i < 60; ++i) world.Tick(1.0f / 60);
    check(physics.GetStepCount() == 30, "Active: half speed runs exactly 30 fixed steps per render second");
    const float slowTime = animation.GetAnimationTime();
    world.SetPhysicsTimeScale(1);
    prepare(1); physics.ResetClock();
    check(std::abs(animation.GetAnimationTime() - slowStart) < .0001f, "Active: repeatable slow-motion baseline");
    for (int i = 0; i < 30; ++i) world.Tick(1.0f / 60);
    check(std::abs(animation.GetAnimationTime() - slowTime) < .0001f, "Active: target animation uses the same slowed physics clock");
    check(character.StopRagdoll(error) && physics.GetErrorCount() == 0, "Active: restore animation without PhysX errors");
    report << "PASS: all active ragdoll integration checks" << std::endl;
}
