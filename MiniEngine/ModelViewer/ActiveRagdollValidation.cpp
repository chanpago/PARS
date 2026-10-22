// Included in the opt-in -ragdolltest path; no synthetic skeleton replaces the imported character.
#include "GameFramework/World.h"
#include "GameFramework/Character.h"
#include "GameFramework/PlayerController.h"
#include "PhysicsWorld.h"
#include "PreviewAnimInstance.h"
#include <ostream>
#include <stdexcept>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>

using namespace Math;
using namespace PARS;

// 실제 캐릭터에 같은 회전 목표를 재생해 골반과 각 관절의 추종 오차를 기록한다.
bool RunPelvisDriveValidation(World& world, Character& character, const std::wstring& reportPath,
    uint32_t profile, uint32_t clip)
{
    std::ofstream report(reportPath);
    std::ofstream samples(reportPath + L".csv");
    try
    {
        if (!report || !samples || profile > 2 || clip > 2)
        {
            throw std::runtime_error("Invalid pelvis validation output or configuration");
        }
        auto& mesh = character.GetMesh();
        auto& ragdoll = mesh.GetRagdoll();
        auto& animation = static_cast<PreviewAnimInstance&>(mesh.GetAnimInstance());
        auto& physics = *world.GetPhysicsWorld();
        const auto asset = *mesh.GetPhysicsAsset();
        PhysicsSkeleton skeleton;
        std::string error;
        if (!mesh.BuildPhysicsSkeleton(skeleton, error) || !character.StopRagdoll(error))
        {
            throw std::runtime_error(error);
        }
        character.GetCharacterMovement().StopMovementImmediately();
        animation.SetLocomotionEnabled(false);
        animation.SetAutoCycleEnabled(false);
        animation.PlayAnimation(clip);
        physics.ResetClock();
        world.SetPhysicsPaused(false);
        world.SetPhysicsTimeScale(1.0f);
        constexpr float deltaTime = 1.0f / 60.0f;
        for (unsigned frame = 0; frame < 24; ++frame)
        {
            world.Tick(deltaTime);
        }
        character.GetCharacterMovement().SetTickEnabled(false);
        const auto base = character.GetActorTransform();
        if (!character.SetPhysicsMode(PhysicsAnimationMode::ActiveRagdoll, error) ||
            !ragdoll.SetRootSupported(true, error) || !ragdoll.UsesArticulation())
        {
            throw std::runtime_error("Could not start supported articulation: " + error);
        }
        for (unsigned frame = 0; frame < 120; ++frame)
        {
            world.Tick(deltaTime);
        }
        samples << std::setprecision(9);
        samples << "frame,time_s,yaw_rad,joint,joint_error_rad,world_position_error_m,"
            "root_rotation_error_rad,root_position_error_m,target_clamp_rad,lowest_point_m\n";
        float maxRootRotation = 0;
        float maxRootPosition = 0;
        float maxJointError = 0;
        for (unsigned frame = 0; frame < 360; ++frame)
        {
            const float time = frame * deltaTime;
            float yaw = 0;
            if (profile == 1)
            {
                const float phase = Clamp((time - 1.0f) / 3.0f, 0.0f, 1.0f);
                const float wave = std::sin(phase * XM_PI);
                yaw = XM_PI * wave * wave;
            }
            else if (profile == 2)
            {
                yaw = XM_PI * (Clamp((time - 1.0f) / .25f, 0.0f, 1.0f) -
                    Clamp((time - 3.0f) / .25f, 0.0f, 1.0f));
            }
            auto transform = base;
            transform.SetBasis(Matrix3::MakeYRotation(yaw) * base.GetBasis());
            character.SetActorTransform(transform);
            world.Tick(deltaTime);
            if (physics.GetLastSubsteps() != 1 || !ragdoll.IsFinite() || ragdoll.GetKinematicBodyCount() != 0)
            {
                throw std::runtime_error("Expected one finite, fully dynamic step per sample");
            }
            std::vector<Matrix4> target;
            if (!EvaluatePhysicsPose(skeleton, animation.GetPose(), Matrix4(mesh.GetComponentTransform()), target, error))
            {
                throw std::runtime_error(error);
            }
            const float rootRotation = ragdoll.GetRootRotationError();
            const float rootPosition = ragdoll.GetRootPositionError();
            maxRootRotation = Max(maxRootRotation, rootRotation);
            maxRootPosition = Max(maxRootPosition, rootPosition);
            for (const auto& joint : asset.constraints)
            {
                const int parent = asset.FindBody(joint.parentBone);
                const int child = asset.FindBody(joint.childBone);
                const auto parentLocal = asset.bodies[parent].GetLocalTransform();
                const auto childLocal = asset.bodies[child].GetLocalTransform();
                auto desired = EvaluateJointDriveTarget(joint,
                    PhysicsBoneFrame(target[ragdoll.GetBoneNodes()[parent]]) * parentLocal,
                    PhysicsBoneFrame(target[ragdoll.GetBoneNodes()[child]]) * childLocal, false);
                desired = ClampArticulationDriveTarget(joint, desired);
                const auto actual = EvaluateJointDriveTarget(joint, ragdoll.GetBoneWorld(parent) * parentLocal,
                    ragdoll.GetBoneWorld(child) * childLocal, false);
                const float dot = std::abs(XMVectorGetX(XMVector4Dot(desired, actual)));
                const float angleError = 2.0f * std::acos(Clamp(dot, 0.0f, 1.0f));
                const float positionError = Length(Vector3(ragdoll.GetBoneWorld(child).GetW()) -
                    Vector3(PhysicsBoneFrame(target[ragdoll.GetBoneNodes()[child]]).GetW()));
                maxJointError = Max(maxJointError, angleError);
                samples << frame << ',' << time << ',' << yaw << ',' << joint.childBone << ',' << angleError << ','
                    << positionError << ',' << rootRotation << ',' << rootPosition << ','
                    << ragdoll.GetMaxTargetClampAngle() << ',' << ragdoll.GetLowestPoint() << '\n';
            }
        }
        if (physics.GetErrorCount() != 0)
        {
            throw std::runtime_error("PhysX reported an error during pelvis validation");
        }
        report << "profile=" << profile << " clip=" << clip << " dt=" << deltaTime << '\n';
        report << "max_root_rotation_rad=" << maxRootRotation << " max_root_position_m=" << maxRootPosition
            << " max_joint_error_rad=" << maxJointError << '\n';
        // 오차는 비교용 측정치다. 유한 상태라는 사실만으로 추종 성능이 좋아졌다고 판정하지 않는다.
        report << "PASS: pelvis validation completed 360 dynamic fixed steps\n";
        return true;
    }
    catch (const std::exception& exception)
    {
        report << "FAIL: " << exception.what() << std::endl;
        return false;
    }
}

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
    auto start = character.GetActorLocation();
    const auto bodyCount = physics.GetDynamicBodyCount(), jointCount = physics.GetConstraintCount();
    PhysicsSkeleton skeleton;
    check(mesh.BuildPhysicsSkeleton(skeleton, error), "Active: imported skeleton bindings");
    const auto samePose = [](const Matrix4& a, const Matrix4& b) {
        return float(Length(Vector3(a.GetW()) - Vector3(b.GetW()))) < .0001f &&
            float(Length(Vector3(a.GetX()) - Vector3(b.GetX()))) < .0001f &&
            float(Length(Vector3(a.GetY()) - Vector3(b.GetY()))) < .0001f;
    };
    const auto pelvisTracksAnimation = [&]() {
        std::vector<Matrix4> target;
        if (!EvaluatePhysicsPose(skeleton, animation.GetPose(), Matrix4(mesh.GetComponentTransform()), target, error)) return false;
        const auto actual = ragdoll.GetBoneWorld(0), desired = PhysicsBoneFrame(target[ragdoll.GetBoneNodes()[0]]);
        return float(Length(Vector3(actual.GetW()) - Vector3(desired.GetW()))) < .08f &&
            std::abs(XMVectorGetX(XMVector4Dot(Quaternion(actual), Quaternion(desired)))) > std::cos(.35f * .5f);
    };
    const auto prepare = [&](uint32_t clip, bool suspended = true) {
        check(character.StopRagdoll(error), "Active: return to animation");
        character.GetCharacterMovement().SetTickEnabled(true);
        character.SetActorLocation(start);
        animation.SetLocomotionEnabled(false); animation.SetAutoCycleEnabled(false);
        animation.PlayAnimation(clip);
        physics.ResetClock();
        for (int i = 0; i < 24; ++i) world.Tick(1.0f / 60);
        // Isolate joint tracking from floor obstruction: a supported, hanging
        // character can reach the animation targets. Test floor contact separately.
        if (suspended) character.SetActorLocation(character.GetActorLocation() + Vector3(0, 1, 0));
        world.Tick(0);
        if (suspended) character.GetCharacterMovement().SetTickEnabled(false);
        check(character.SetPhysicsMode(PhysicsAnimationMode::ActiveRagdoll, error) && ragdoll.SetRootSupported(true, error),
            "Active: enter supported dynamic drive mode");
        check(ragdoll.GetDrivenJointCount() == 19 && ragdoll.GetKinematicBodyCount() == 0 &&
            physics.GetConstraintCount() == jointCount + 1,
            "Active: 20 dynamic bodies, 19 driven joints and one pelvis force/torque support constraint");
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
        float pelvisPositionError = 0, pelvisRotationError = 0;
        size_t worst = 0; int worstStep = 0;
        double elapsedMs = 0;
        for (int step = 0; step < 360; ++step)
        {
            const auto before = std::chrono::steady_clock::now();
            world.Tick(1.0f / 60);
            elapsedMs += std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - before).count();
            if (!ragdoll.IsFinite()) check(false, "Active: finite dynamic state");
            if (!pelvisTracksAnimation()) check(false, "Active: dynamic pelvis tracks animation within 8 cm / 0.35 rad");
            pelvisPositionError = Max(pelvisPositionError, ragdoll.GetRootPositionError());
            pelvisRotationError = Max(pelvisRotationError, ragdoll.GetRootRotationError());
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
        report << "active_pelvis=" << clip << " travel_m=" << pelvisTravel << " rotation_basis_delta=" << pelvisRotation
            << " position_error_m=" << pelvisPositionError << " rotation_error_rad=" << pelvisRotationError << std::endl;
        check(pelvisTravel > .0001f && pelvisRotation > .0001f && ragdoll.GetKinematicBodyCount() == 0,
            "Active: pelvis follows animated translation and rotation while every body remains dynamic");
        check(maxError < .35f && maxGap < .08f && maxLimit < .35f, "Active: animated tracking, anchors and limits within declared tolerances");
        // Hold a real sampled pose, disturb the physical body, and let motors recover.
        animation.SetPlaying(false);
        for (int i = 0; i < 180; ++i) world.Tick(1.0f / 60);
        const float restError = ragdoll.GetMaxDriveError();
        const float restPelvisError = ragdoll.GetRootPositionError();
        const float frozenTime = animation.GetAnimationTime();
        const auto targetBeforePush = animation.GetPose();
        ragdoll.AddVelocity(Vector3(0, 1, 2));
        float peak = 0, pelvisPeak = 0;
        for (int i = 0; i < 240; ++i)
        {
            world.Tick(1.0f / 60);
            peak = Max(peak, ragdoll.GetMaxDriveError());
            pelvisPeak = Max(pelvisPeak, ragdoll.GetRootPositionError());
        }
        report << "active_recovery=" << clip << " rest=" << restError << " peak=" << peak << " final=" << ragdoll.GetMaxDriveError() << std::endl;
        check(peak > restError + .01f && ragdoll.GetMaxDriveError() < .35f, "Active: impulse response and static target recovery");
        check(pelvisPeak > restPelvisError + .005f && pelvisTracksAnimation(),
            "Active: impulse displaces the pelvis and force/torque support recovers");
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
            !mesh.HasPendingDriveSettings() && ragdoll.GetDrivenJointCount() == 0 && pelvisTracksAnimation() &&
            ragdoll.GetKinematicBodyCount() == 0 && ragdoll.IsRootSupported() && !samePose(pausedPelvis, ragdoll.GetBoneWorld(0)),
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

    // The viewer normally enables support while the feet contact the floor.
    // Exercise this separately from the suspended tracking measurements.
    for (uint32_t clip = 0; clip < 3; ++clip)
    {
        prepare(clip, false);
        float positionError = 0, rotationError = 0;
        for (int step = 0; step < 180; ++step)
        {
            world.Tick(1.0f / 60);
            if (!ragdoll.IsFinite() || ragdoll.GetMaxAnchorError() >= .08f || ragdoll.GetLowestPoint() < -.04f)
                check(false, "Active: supported floor contact remains finite, connected and above the floor");
            positionError = Max(positionError, ragdoll.GetRootPositionError());
            rotationError = Max(rotationError, ragdoll.GetRootRotationError());
        }
        report << "active_floor=" << clip << " pelvis_position_error_m=" << positionError
            << " pelvis_rotation_error_rad=" << rotationError << std::endl;
        check(ragdoll.GetKinematicBodyCount() == 0 && positionError < .12f && rotationError < .5f,
            "Active: all-dynamic pelvis support remains bounded during animated floor contact");
    }

    // 사용자에게 보였던 제자리 회전을 실제 Mixamo Idle 포즈와 바닥 접촉 상태에서 재현한다.
    prepare(0, false);
    character.GetCharacterMovement().SetTickEnabled(false);
    const auto turnBase = character.GetActorTransform();
    float armTurnError = 0;
    for (int step = 0; step < 360; ++step)
    {
        const float phase = Clamp((step / 60.0f - 1.0f) / 3.0f, 0.0f, 1.0f);
        const float wave = std::sin(phase * XM_PI);
        auto transform = turnBase;
        transform.SetBasis(Matrix3::MakeYRotation(XM_PI * wave * wave) * turnBase.GetBasis());
        character.SetActorTransform(transform);
        world.Tick(1.0f / 60);
        std::vector<Matrix4> target;
        check(EvaluatePhysicsPose(skeleton, animation.GetPose(), Matrix4(mesh.GetComponentTransform()), target, error),
            "Active turn: sample imported animation target");
        for (const auto& joint : originalAsset.constraints)
        {
            const auto& name = joint.childBone;
            if (name.find("Arm") == std::string::npos && name.find("Hand") == std::string::npos && name.find("Shoulder") == std::string::npos)
            {
                continue;
            }
            const int parent = originalAsset.FindBody(joint.parentBone), child = originalAsset.FindBody(joint.childBone);
            const auto parentLocal = originalAsset.bodies[parent].GetLocalTransform();
            const auto childLocal = originalAsset.bodies[child].GetLocalTransform();
            auto desired = EvaluateJointDriveTarget(joint,
                PhysicsBoneFrame(target[ragdoll.GetBoneNodes()[parent]]) * parentLocal,
                PhysicsBoneFrame(target[ragdoll.GetBoneNodes()[child]]) * childLocal, false);
            desired = ClampArticulationDriveTarget(joint, desired);
            const auto actual = EvaluateJointDriveTarget(joint, ragdoll.GetBoneWorld(parent) * parentLocal,
                ragdoll.GetBoneWorld(child) * childLocal, false);
            const float dot = std::abs(XMVectorGetX(XMVector4Dot(desired, actual)));
            if (step > 60)
            {
                armTurnError = Max(armTurnError, 2 * std::acos(Clamp(dot, 0.0f, 1.0f)));
            }
        }
        if (!ragdoll.IsFinite())
        {
            check(false, "Active turn: finite articulated character");
        }
    }
    character.SetActorTransform(turnBase);
    report << "active_idle_turn_arm_error_rad=" << armTurnError << std::endl;
    check(armTurnError < .35f, "Active turn: Idle arms follow a start/reverse/stop yaw on the floor at frequency 16");

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

    auto* player = dynamic_cast<PlayerController*>(character.GetController());
    check(player != nullptr, "Active movement: preview has a player controller");
    // Earlier regressions already walked forward. Leave enough room to measure
    // unrestricted walk/run before deliberately driving into the +Z wall.
    start = Vector3(2, .95f, -1);
    const auto prepareMovement = [&]() {
        player->SetInputEnabled(false);
        prepare(0, false);
        animation.SetLocomotionEnabled(true);
        player->SetControlYaw(0); player->SetInputEnabled(true);
    };
    const auto moveFrames = [&](int count, float dt = 1.0f / 60) {
        for (int i = 0; i < count; ++i)
        {
            world.Tick(dt);
            if (!ragdoll.IsFinite() || ragdoll.GetKinematicBodyCount() != 0 || ragdoll.GetMaxAnchorError() > .08f)
                check(false, "Active movement: dynamic body state and joint anchors remain valid");
        }
    };
    prepareMovement();
    const auto walkStart = character.GetActorLocation();
    const auto pelvisStart = ragdoll.GetPosition();
    player->SetMoveInput(1, 0, false); moveFrames(60);
    report << "active_walk_travel_m=" << float(character.GetActorLocation().GetZ() - walkStart.GetZ())
        << " pelvis_error_m=" << ragdoll.GetRootPositionError() << " speed_mps=" << character.GetCharacterMovement().GetGroundSpeed()
        << " clip=" << animation.GetCurrentAnimationIndex() << std::endl;
    check(character.CanControlMovement() && character.GetCapsuleComponent().HasPhysicsCollision() &&
        float(character.GetActorLocation().GetZ() - walkStart.GetZ()) > 1.4f &&
        float(ragdoll.GetPosition().GetZ() - pelvisStart.GetZ()) > 1.2f && ragdoll.GetRootPositionError() < .2f &&
        animation.GetCurrentAnimationIndex() == 1, "Active movement: WASD walks the physical character and selects Walk");
    auto& movingRender = GraphicsContext::Begin(L"Validate moving active ragdoll pose");
    check(mesh.UpdateRenderData(movingRender), "Active movement: upload moving physical pose"); movingRender.Finish(true);
    Matrix4 movingSkin, movingOwner;
    const int pelvisNode = ragdoll.GetBoneNodes()[0];
    mesh.GetModelInstance().GetNodeTransform(pelvisNode, movingSkin);
    mesh.GetModelInstance().GetNodeTransform(skeleton.meshNodes[pelvisNode], movingOwner);
    check(float(Length(Vector3(PhysicsBoneFrame(movingOwner * movingSkin).GetW()) - Vector3(ragdoll.GetBoneWorld(0).GetW()))) < .001f,
        "Active movement: moving actor transform does not double-transform the rendered physical pelvis");
    const auto runStart = character.GetActorLocation();
    player->SetMoveInput(1, 0, true); moveFrames(60);
    report << "active_run_travel_m=" << float(character.GetActorLocation().GetZ() - runStart.GetZ())
        << " pelvis_error_m=" << ragdoll.GetRootPositionError() << " speed_mps=" << character.GetCharacterMovement().GetGroundSpeed()
        << " clip=" << animation.GetCurrentAnimationIndex() << std::endl;
    check(float(character.GetActorLocation().GetZ() - runStart.GetZ()) > 3 &&
        ragdoll.GetRootPositionError() < .25f && animation.GetCurrentAnimationIndex() == 2,
        "Active movement: Shift increases travel speed and selects Run");
    player->SetMoveInput(0, 0, false); moveFrames(60);
    check(character.GetCharacterMovement().GetGroundSpeed() < .01f && animation.GetCurrentAnimationIndex() == 0,
        "Active movement: releasing input brakes the character and returns to Idle");
    player->SetControlYaw(XM_PI); player->SetMoveInput(1, 0, false); moveFrames(90);
    check(float(character.GetActorTransform().GetZ().GetZ()) < -.99f && ragdoll.GetRootRotationError() < .35f,
        "Active movement: dynamic pelvis torque follows a change in travel direction");
    player->SetControlYaw(0); player->SetMoveInput(1, 0, true); moveFrames(150);
    check(float(character.GetActorLocation().GetZ()) < 5.5f && character.GetCharacterMovement().GetGroundSpeed() < .05f &&
        animation.GetCurrentAnimationIndex() == 0 && float(ragdoll.GetPosition().GetZ()) < 5.8f,
        "Active movement: scene wall stops the capsule and physical pelvis, and animation returns to Idle");

    prepareMovement();
    player->SetMoveInput(1, 0, false); moveFrames(30);
    const auto pausedPosition = character.GetActorLocation();
    const float pausedTime = animation.GetAnimationTime();
    world.SetPhysicsPaused(true); world.Tick(.5f);
    check(float(Length(character.GetActorLocation() - pausedPosition)) < 1e-6f && animation.GetAnimationTime() == pausedTime,
        "Active movement: pause freezes movement and target animation together");
    world.RequestPhysicsStep(); world.Tick(0);
    check(float(character.GetActorLocation().GetZ() - pausedPosition.GetZ()) > .01f && animation.GetAnimationTime() != pausedTime,
        "Active movement: a single step advances capsule, target and physical body together");
    world.SetPhysicsPaused(false);
    check(ragdoll.SetRootSupported(false, error), "Active movement: disable pelvis support");
    const auto unsupportedPosition = character.GetActorLocation();
    moveFrames(6);
    check(!character.CanControlMovement() && !character.GetCapsuleComponent().HasPhysicsCollision() &&
        float(Length(character.GetActorLocation() - unsupportedPosition)) < 1e-6f,
        "Active movement: releasing support stops movement and removes the capsule");
    check(ragdoll.SetRootSupported(true, error), "Active movement: re-enable pelvis support");
    moveFrames(30);
    check(character.CanControlMovement() && character.GetCapsuleComponent().HasPhysicsCollision() &&
        float(character.GetActorLocation().GetZ() - unsupportedPosition.GetZ()) > .4f,
        "Active movement: support restoration resumes movement without self-collision");
    const auto passivePosition = character.GetActorLocation();
    check(character.SetPhysicsMode(PhysicsAnimationMode::PassiveRagdoll, error) && !character.CanControlMovement() &&
        !character.GetCharacterMovement().IsTickEnabled() && !character.GetCapsuleComponent().HasPhysicsCollision(),
        "Active movement: Passive immediately disables movement and capsule collision");
    moveFrames(12);
    check(float(Length(character.GetActorLocation() - passivePosition)) < 1e-6f,
        "Active movement: held input cannot move a passive character target");
    player->SetInputEnabled(false);
    check(character.StopRagdoll(error) && character.CanControlMovement() &&
        float(Length(character.GetActorLocation() - passivePosition)) < 1e-6f,
        "Active movement: return to animation preserves the latest movement position and restores control");

    float referenceTravel = 0;
    for (int fps : {30, 60, 144})
    {
        prepareMovement(); physics.ResetClock();
        const auto begin = character.GetActorLocation();
        player->SetMoveInput(1, 0, false); moveFrames(fps, 1.0f / fps);
        const float travel = float(character.GetActorLocation().GetZ() - begin.GetZ());
        if (fps == 30) referenceTravel = travel;
        check(physics.GetStepCount() == 60 && std::abs(travel - referenceTravel) < .005f && ragdoll.GetRootPositionError() < .2f,
            "Active movement: actual travel is independent of 30/60/144 render FPS");
    }
    prepareMovement();
    player->SetMoveInput(1, 0, false); moveFrames(12);
    const auto validTransform = mesh.GetRelativeTransform();
    auto invalidTransform = validTransform; invalidTransform.SetBasis(Matrix3::MakeScale(2.0f));
    mesh.SetRelativeTransform(invalidTransform); world.Tick(1.0f / 60);
    check(mesh.GetPhysicsMode() == PhysicsAnimationMode::PassiveRagdoll && !ragdoll.IsRootSupported() &&
        !character.GetCharacterMovement().IsTickEnabled() && !character.GetCapsuleComponent().HasPhysicsCollision() &&
        !mesh.GetPhysicsError().empty(), "Active movement: target failure disables movement and capsule in the same fixed step");
    mesh.SetRelativeTransform(validTransform);
    player->SetInputEnabled(false);
    check(character.StopRagdoll(error) && physics.GetErrorCount() == 0, "Active movement: restore animation without PhysX errors");
    report << "PASS: all active ragdoll movement checks" << std::endl;
    report << "PASS: all active ragdoll integration checks" << std::endl;
}
