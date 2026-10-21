// Opt-in integration check: ModelViewer.exe -ragdolltest <report path>
// Runs with the actual imported model, component lifecycle, PhysX and DX12 pose upload.
#include "GameFramework/World.h"
#include "GameFramework/Character.h"
#include "PhysicsWorld.h"
#include "PreviewAnimInstance.h"
#include "CommandListManager.h"
#include <fstream>
#include <stdexcept>
#include <cmath>
#include <algorithm>
#include <cstdio>

using namespace Math;
void RunActiveRagdollValidation(PARS::World& world, PARS::Character& character, std::ostream& report);
bool RunRagdollValidation(PARS::World& world, PARS::Character& character, const std::wstring& reportPath)
{
    // SDK 단언으로 종료되어도 원인을 리다이렉트한 테스트 로그에 남긴다.
    std::setvbuf(stdout, nullptr, _IONBF, 0);
    std::setvbuf(stderr, nullptr, _IONBF, 0);
    std::ofstream report(reportPath);
    const auto check = [&](bool condition, const char* message) {
        report << (condition ? "PASS: " : "FAIL: ") << message << std::endl;
        if (!condition) throw std::runtime_error(message);
    };
    std::string error;
    auto& mesh = character.GetMesh();
    auto* physics = world.GetPhysicsWorld();
    try
    {
        check(report.good() && physics && mesh.GetPhysicsAsset(), "Play character has a default PA and physics world");
        constexpr size_t bodyCount = 20, jointCount = 19;
        const auto& asset = *mesh.GetPhysicsAsset();
        check(asset.bodies.size() == bodyCount && asset.constraints.size() == jointCount,
            "Default PA has 20 bodies / 19 joints including all spine, neck and shoulder bones");
        check(!mesh.IsRagdoll() && mesh.GetRagdoll().GetBodyCount() == bodyCount && mesh.GetRagdoll().GetJointCount() == jointCount &&
            mesh.GetRagdoll().GetKinematicBodyCount() == bodyCount, "PA bodies and joints are already registered as kinematic before starting ragdoll");
        if (physics->IsPvdConnected()) check(physics->ConnectPvd(true), "Reconnect PVD with animated kinematic bodies and D6 joints");
        auto& animation = static_cast<PreviewAnimInstance&>(mesh.GetAnimInstance());
        animation.SetLocomotionEnabled(false); animation.SetAutoCycleEnabled(false);
        const auto initialCapsuleCount = physics->GetQueryCapsuleCount();
        check(initialCapsuleCount > 0, "Query-only movement capsule exists before ragdoll");
        PARS::PhysicsSkeleton skeleton;
        check(mesh.BuildPhysicsSkeleton(skeleton, error), "Build imported mesh skin bindings");
        // Verify against the imported skeleton, independently of the preset's
        // list: every skinned bone must either have a body or be an explicit
        // finger/toe/end marker. This catches silently skipped major bones.
        for (size_t i = 0; i < skeleton.names.size(); ++i)
        {
            if (skeleton.meshNodes[i] < 0) continue;
            const auto& name = skeleton.names[i];
            bool excluded = name.find("Toe") != std::string::npos || name.find("_End") != std::string::npos;
            for (const char* digit : {"HandThumb", "HandIndex", "HandMiddle", "HandRing", "HandPinky"})
                excluded |= name.find(digit) != std::string::npos;
            const bool hasBody = asset.FindBody(name) >= 0;
            report << "coverage: " << name << (hasBody ? " -> body" : " -> excluded") << std::endl;
            check(hasBody != excluded, "Imported non-digit, non-terminal bone coverage");
        }
        float totalMass = 0;
        for (const auto& body : asset.bodies)
        {
            check(std::isfinite(body.mass) && body.mass > 0, "Every imported body has a finite positive mass");
            totalMass += body.mass;
            report << "mass: " << body.bone << " = " << body.mass << " kg" << std::endl;
        }
        check(std::abs(totalMass - 73.0f) < .001f, "Adult male preset preserves total 73 kg mass");
        const auto massFor = [&](const std::string& suffix) {
            for (const auto& body : asset.bodies)
            {
                const auto colon = body.bone.find_last_of(':');
                if (body.bone.substr(colon == std::string::npos ? 0 : colon + 1) == suffix) return body.mass;
            }
            check(false, "Mass validation can find every anatomical body");
            return 0.0f;
        };
        // Check published anatomical totals independently of the implementation's
        // volume-based split. In particular, never double-count neck or shoulders.
        check(std::abs(massFor("Hips") - 8.1541f) < .001f, "Pelvis is 11.17 percent of body mass");
        check(std::abs(massFor("Spine") + massFor("Spine1") - 11.9209f) < .001f,
            "Middle trunk is 16.33 percent of body mass");
        check(std::abs(massFor("Spine2") + massFor("LeftShoulder") + massFor("RightShoulder") - 11.6508f) < .001f,
            "Upper trunk including shoulders is 15.96 percent of body mass");
        check(std::abs(massFor("Head") + massFor("Neck") - 5.0662f) < .001f,
            "Head including neck is 6.94 percent of body mass");
        check(std::abs(massFor("Neck") / massFor("Head") - 1.66f / 4.20f) < .0001f,
            "Neck/head split follows the male anthropometric model, independent of collision shape size");
        for (const char* side : {"Left", "Right"})
        {
            const std::string prefix(side);
            check(std::abs(massFor(prefix + "Arm") - 1.9783f) < .001f &&
                std::abs(massFor(prefix + "ForeArm") - 1.1826f) < .001f &&
                std::abs(massFor(prefix + "Hand") - .4453f) < .001f,
                "Each arm uses per-side male upper-arm, forearm and hand proportions");
            check(std::abs(massFor(prefix + "UpLeg") - 10.3368f) < .001f &&
                std::abs(massFor(prefix + "Leg") - 3.1609f) < .001f &&
                std::abs(massFor(prefix + "Foot") - 1.0001f) < .001f,
                "Each leg uses per-side male thigh, shank and foot proportions");
        }
        for (const auto& joint : asset.constraints)
        {
            const auto child = std::find(skeleton.names.begin(), skeleton.names.end(), joint.childBone);
            check(child != skeleton.names.end(), "Constraint child exists in imported skeleton");
            int parent = skeleton.parents[child - skeleton.names.begin()];
            while (parent >= 0 && asset.FindBody(skeleton.names[parent]) < 0) parent = skeleton.parents[parent];
            check(parent >= 0 && joint.parentBone == skeleton.names[parent], "Every constraint follows the nearest parent body without skipping spine/neck/shoulders");
        }
        const auto checkKinematicPose = [&]() {
            std::vector<Matrix4> expected;
            if (!PARS::EvaluatePhysicsPose(skeleton, animation.GetPose(), Matrix4(mesh.GetComponentTransform()), expected, error)) return false;
            const auto& runtime = mesh.GetRagdoll();
            if (mesh.IsRagdoll() || runtime.GetKinematicBodyCount() != bodyCount || !mesh.GetPhysicsError().empty()) return false;
            for (size_t body = 0; body < runtime.GetBodyCount(); ++body)
            {
                const Matrix4 target = PARS::PhysicsBoneFrame(expected[runtime.GetBoneNodes()[body]]), actual = runtime.GetBoneWorld(body);
                if (float(Length(Vector3(target.GetW()) - Vector3(actual.GetW()))) > .001f ||
                    float(Length(Vector3(target.GetX()) - Vector3(actual.GetX()))) > .001f ||
                    float(Length(Vector3(target.GetY()) - Vector3(actual.GetY()))) > .001f) return false;
            }
            return true;
        };
        const auto animatedBodies = physics->GetDynamicBodyCount(), animatedJoints = physics->GetConstraintCount();
        check(animatedBodies == mesh.GetRagdoll().GetBodyCount() + 1,
            "Scene contains only PA bodies and the demo box, with no solid movement capsule");
        for (uint32_t clip = 0; clip < 3; ++clip)
        {
            animation.PlayAnimation(clip);
            for (int i = 0; i < 120; ++i)
            {
                world.Tick(1.0f / 60);
                if (!checkKinematicPose()) check(false, "Every kinematic PA body follows sampled animation position and orientation");
            }
            check(true, "120 animated steps keep all 20 kinematic bodies aligned with the mesh");
            // Keep the established ragdoll regression poses, independently of
            // the longer animation-following check above.
            animation.PlayAnimation(clip);
            for (int i = 0; i < 24; ++i) world.Tick(1.0f / 60);
            std::vector<Matrix4> before;
            check(PARS::EvaluatePhysicsPose(skeleton, animation.GetPose(), Matrix4(mesh.GetComponentTransform()), before, error), "Evaluate actual animated pose");
            // Compare the CPU adapter to ModelInstance's renderer transform cache.
            auto& context = GraphicsContext::Begin(L"Validate ragdoll initial pose");
            check(mesh.UpdateRenderData(context), "Upload animation pose to DX12");
            context.Finish(true);
            float renderError = 0;
            for (size_t node = 0; node < before.size(); ++node)
                if (skeleton.meshNodes[node] >= 0)
                {
                    Matrix4 skin, owner;
                    check(mesh.GetModelInstance().GetNodeTransform(static_cast<uint32_t>(node), skin) &&
                        mesh.GetModelInstance().GetNodeTransform(skeleton.meshNodes[node], owner), "Read cached render transforms");
                    renderError = Max(renderError, float(Length(Vector3((owner * skin).GetW()) - Vector3(before[node].GetW()))));
                }
            check(renderError < .0001f, "CPU physics pose agrees with actual renderer (no doubled import scale)");
            const float animationTime = animation.GetAnimationTime();
            const bool started = character.StartRagdoll(error);
            check(started, "Start from animated pose");
            check(!character.GetCapsuleComponent().HasPhysicsCollision() && !character.GetCharacterMovement().IsTickEnabled() &&
                physics->GetQueryCapsuleCount() + 1 == initialCapsuleCount, "Ragdoll disables movement and removes capsule collision");
            auto& ragdoll = mesh.GetRagdoll();
            check(ragdoll.GetBodyCount() == bodyCount && ragdoll.GetJointCount() == jointCount, "Instantiate actual PA articulation links and joints");
            check(ragdoll.GetKinematicBodyCount() == 0 && physics->GetDynamicBodyCount() == animatedBodies &&
                physics->GetConstraintCount() == animatedJoints, "Ragdoll switches registered bodies to dynamic without duplicating the PA");
            std::vector<Matrix4> after;
            PARS::EvaluatePhysicsPose(skeleton, mesh.GetFinalPose(), Matrix4(mesh.GetComponentTransform()), after, error);
            float transitionError = 0;
            for (size_t node = 0; node < before.size(); ++node)
                transitionError = Max(transitionError, float(Length(Vector3(before[node].GetW()) - Vector3(after[node].GetW()))));
            check(transitionError < .001f, "Animation to physics transition preserves visible bone positions");
            const float startHeight = ragdoll.GetPosition().GetY();
            float maxGap = 0, maxAngular = 0;
            size_t worstJoint = 0;
            int worstStep = 0;
            for (int step = 0; step < 600; ++step)
            {
                world.Tick(1.0f / 60);
                if (!ragdoll.IsFinite()) check(false, "Finite ragdoll state");
                if (step > 60)
                {
                    maxGap = Max(maxGap, ragdoll.GetMaxAnchorError());
                    size_t joint = 0;
                    const float angular = ragdoll.GetMaxAngularError(&joint);
                    if (angular > maxAngular) { maxAngular = angular; worstJoint = joint; worstStep = step; }
                }
            }
            check(true, "600 simulation steps remain finite");
            report << "clip=" << clip << " initialY=" << startHeight << " finalY=" << float(ragdoll.GetPosition().GetY())
                << " lowest=" << ragdoll.GetLowestPoint() << " maxGap=" << maxGap << " maxAngular=" << maxAngular
                << " finalAngular=" << ragdoll.GetMaxAngularError()
                << " worstJoint=" << asset.constraints[worstJoint].childBone << " step=" << worstStep << std::endl;
            check(maxGap < .08f && maxAngular < .35f, "Joint anchors and angular limits remain bounded");
            check(float(ragdoll.GetPosition().GetY()) < startHeight - .2f && ragdoll.GetLowestPoint() > -.04f, "Character falls and colliders stay above the floor");
            check(animation.GetAnimationTime() == animationTime, "Animation cannot overwrite the simulated pose");
            auto& render = GraphicsContext::Begin(L"Validate ragdoll final pose");
            check(mesh.UpdateRenderData(render), "Upload final physical pose to DX12");
            render.Finish(true);
            for (size_t body = 0; body < ragdoll.GetBodyCount(); ++body)
            {
                const int node = ragdoll.GetBoneNodes()[body]; Matrix4 skin, owner;
                mesh.GetModelInstance().GetNodeTransform(node, skin);
                mesh.GetModelInstance().GetNodeTransform(skeleton.meshNodes[node], owner);
                const Matrix4 rendered = PARS::PhysicsBoneFrame(owner * skin), physical = ragdoll.GetBoneWorld(body);
                check(float(Length(Vector3(rendered.GetW()) - Vector3(physical.GetW()))) < .001f &&
                    float(Length(Vector3(rendered.GetX()) - Vector3(physical.GetX()))) < .001f &&
                    float(Length(Vector3(rendered.GetY()) - Vector3(physical.GetY()))) < .001f,
                    "Rendered bone position and orientation follow its physical body");
            }
            world.SetPhysicsPaused(true);
            const auto steps = physics->GetStepCount(); world.Tick(1);
            check(steps == physics->GetStepCount(), "Pause freezes physics");
            world.RequestPhysicsStep(); world.Tick(0);
            check(steps + 1 == physics->GetStepCount(), "Single-step advances exactly one fixed step");
            world.SetPhysicsPaused(false);
            if (clip == 0 && physics->IsPvdConnected())
                check(physics->ConnectPvd(true), "Reconnect PVD with live D6 joints");
            check(character.StopRagdoll(error) && !mesh.IsRagdoll() && character.GetCapsuleComponent().HasPhysicsCollision() &&
                character.GetCharacterMovement().IsTickEnabled() && physics->GetQueryCapsuleCount() == initialCapsuleCount,
                "Stop returns to animation and restores query-only movement collision");
            check(checkKinematicPose() && physics->GetDynamicBodyCount() == animatedBodies && physics->GetConstraintCount() == animatedJoints,
                "Stopping keeps all PA objects registered and immediately restores kinematic animation poses");
            world.Tick(1.0f / 60);
            check(animation.GetAnimationTime() != animationTime, "Animation resumes after stopping ragdoll");
        }
        const Vector3 start = character.GetActorLocation();
        for (int step = 0; step < 30; ++step)
        {
            character.AddMovementInput(Vector3(0, 0, 1)); world.Tick(1.0f / 60);
            if (!checkKinematicPose()) check(false, "Kinematic PA follows the moving character root");
        }
        check(float(Length(character.GetActorLocation() - start)) > .3f, "Character can walk again after ragdoll");
        world.SetPhysicsPaused(true);
        const float pausedTime = animation.GetAnimationTime(); world.Tick(1);
        check(animation.GetAnimationTime() == pausedTime && checkKinematicPose(), "Pause keeps animation and kinematic PA synchronized");
        world.RequestPhysicsStep(); world.Tick(0);
        check(animation.GetAnimationTime() > pausedTime && checkKinematicPose(), "Single-step advances animation and kinematic PA together");
        world.SetPhysicsPaused(false);
        const auto bodiesBefore = physics->GetDynamicBodyCount(), jointsBefore = physics->GetConstraintCount();
        auto* disposable = world.SpawnActor<PARS::Character>("RagdollLifetimeTest");
        disposable->SetActorLocation(Vector3(6, .95f, 0));
        disposable->GetMesh().SetSkeletalMesh(mesh.GetSkeletalMesh());
        world.Tick(0);
        check(!disposable->StartRagdoll(error) && disposable->GetCapsuleComponent().HasPhysicsCollision() &&
            disposable->GetCharacterMovement().IsTickEnabled(), "Failed start without PA preserves character control");
        auto& otherMesh = disposable->GetMesh();
        check(otherMesh.SetPhysicsAsset(*mesh.GetPhysicsAsset(), error) && otherMesh.GetRagdoll().GetKinematicBodyCount() == bodyCount,
            "Assigning PA after BeginPlay registers kinematic objects immediately");
        auto invalid = *mesh.GetPhysicsAsset(); invalid.bodies[0].radius = -1;
        check(!otherMesh.SetPhysicsAsset(invalid, error) && otherMesh.GetRagdoll().GetKinematicBodyCount() == bodyCount,
            "Rejected PA replacement preserves live kinematic objects");
        check(otherMesh.SetPhysicsAsset(*mesh.GetPhysicsAsset(), error) && physics->GetConstraintCount() == jointsBefore + jointCount,
            "Valid PA replacement does not duplicate joints");
        check(otherMesh.ClearPhysicsAsset() && otherMesh.GetRagdoll().GetBodyCount() == 0 && physics->GetConstraintCount() == jointsBefore,
            "Clearing PA releases kinematic objects");
        check(otherMesh.SetPhysicsAsset(*mesh.GetPhysicsAsset(), error) &&
            disposable->SetPhysicsMode(PARS::PhysicsAnimationMode::ActiveRagdoll, error), "Start a second independent active ragdoll");
        check(!disposable->GetMesh().SetPhysicsAsset(*mesh.GetPhysicsAsset(), error), "Reject PA replacement while bodies are active");
        disposable->Destroy();
        Graphics::g_CommandManager.IdleGPU();
        world.CollectGarbage();
        check(physics->GetDynamicBodyCount() == bodiesBefore && physics->GetConstraintCount() == jointsBefore &&
            physics->GetQueryCapsuleCount() == initialCapsuleCount, "Destroying an active ragdoll releases every body and joint");
        check(physics->GetErrorCount() == 0, "No PhysX errors through three animation/ragdoll cycles");
        RunActiveRagdollValidation(world, character, report);
        report << "PVD connected=" << physics->IsPvdConnected() << std::endl;
        report << "PASS: all character ragdoll integration checks" << std::endl;
        return true;
    }
    catch (const std::exception& exception)
    {
        report << "FAILED: " << exception.what() << " | " << error << std::endl;
        character.StopRagdoll(error); world.SetPhysicsPaused(false);
        return false;
    }
}
