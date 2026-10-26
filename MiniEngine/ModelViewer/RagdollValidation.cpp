// Opt-in integration check: ModelViewer.exe -ragdolltest <report path>
// Runs with the actual imported model, component lifecycle, PhysX and DX12 pose upload.
#include "GameFramework/World.h"
#include "GameFramework/Character.h"
#include "PhysicsWorld.h"
#include "PreviewAnimInstance.h"
#include "CommandListManager.h"
#include "Util/CommandLineArg.h"
#include <fstream>
#include <stdexcept>
#include <cmath>
#include <algorithm>
#include <cstdio>

using namespace Math;

bool RunCharacterWallValidation(PARS::World& world, PARS::Character& character, const std::wstring& reportPath)
{
    using namespace PARS;
    using namespace physx;
    std::ofstream report(reportPath);
    const auto check = [&](bool condition, const char* message) {
        report << (condition ? "PASS: " : "FAIL: ") << message << std::endl;
        if (!condition) throw std::runtime_error(message);
    };
    auto* physics = world.GetPhysicsWorld();
    PxRigidStatic* wall = nullptr;
    try
    {
        check(report.good() && physics && character.GetMesh().GetPhysicsAsset(), "Load actual character and saved PA");
        const auto errors = physics->GetErrorCount();
        wall = physics->CreateStaticBox(PxTransform(PxVec3(2, 2, 3.1f)), PxVec3(2, 2, .1f), "PA wall validation");
        check(wall != nullptr, "Create wall with its front face at Z=3m");
        const auto bodyFront = [&]() {
            auto* scene = wall->getScene();
            float front = -100;
            const auto consider = [&](const PxRigidActor* body) {
                if (body->userData == &character) front = PxMax(front, body->getWorldBounds(1.0f).maximum.z);
            };
            std::vector<PxActor*> actors(scene->getNbActors(PxActorTypeFlag::eRIGID_DYNAMIC));
            scene->getActors(PxActorTypeFlag::eRIGID_DYNAMIC, actors.data(), static_cast<PxU32>(actors.size()));
            for (auto* actor : actors) consider(actor->is<PxRigidActor>());
            std::vector<PxArticulationReducedCoordinate*> rigs(scene->getNbArticulations());
            scene->getArticulations(rigs.data(), static_cast<PxU32>(rigs.size()));
            for (auto* rig : rigs)
            {
                std::vector<PxArticulationLink*> links(rig->getNbLinks());
                rig->getLinks(links.data(), static_cast<PxU32>(links.size()));
                for (auto* link : links) consider(link);
            }
            return front;
        };
        auto& mesh = character.GetMesh();
        auto& animation = static_cast<PreviewAnimInstance&>(mesh.GetAnimInstance());
        auto& movement = character.GetCharacterMovement();
        auto& capsule = character.GetCapsuleComponent();
        world.SetPhysicsPaused(false);
        world.SetPhysicsTimeScale(1);
        for (bool active : {false, true})
        {
            std::string error;
            check(character.StopRagdoll(error), "Reset mode before wall approach");
            movement.StopMovementImmediately();
            character.SetActorTransform(AffineTransform(Matrix3(kIdentity), Vector3(2, capsule.GetHalfHeight(), 0)));
            animation.SetAutoCycleEnabled(false);
            animation.SetLocomotionEnabled(true);
            animation.SetPlaying(true);
            animation.PlayAnimation(0);
            world.Tick(0);
            if (active)
            {
                check(character.SetPhysicsMode(PhysicsAnimationMode::ActiveRagdoll, error) &&
                        mesh.GetRagdoll().SetRootSupported(true, error), "Enable actual articulated active ragdoll");
            }
            for (int i = 0; i < 60; ++i) world.Tick(1.0f / 60);
            check(capsule.IsHorizontalBlockingEnabled() == !active,
                "Animation uses capsule blocking and active ragdoll disables capsule wall blocking");
            float maximumFront = bodyFront(), maximumRoot = 0;
            for (int i = 0; i < 360; ++i)
            {
                character.SetRunRequested(true);
                character.AddMovementInput(Vector3(0, 0, 1));
                world.Tick(1.0f / 60);
                maximumFront = Max(maximumFront, bodyFront());
                maximumRoot = Max(maximumRoot, float(character.GetActorLocation().GetZ()));
                if (!mesh.GetRagdoll().IsFinite()) check(false, "Bodies remain finite during sustained wall input");
            }
            report << "active=" << active << " root_max_z=" << maximumRoot << " body_front_max_z=" << maximumFront
                   << " final_speed=" << movement.GetGroundSpeed() << std::endl;
            check(maximumRoot > 2 && maximumRoot < 3 && movement.GetGroundSpeed() < .1f,
                "Sustained run input reaches the wall and stops the movement target");
            if (active)
            {
                check(maximumFront < 3.02f && mesh.GetRagdoll().GetKinematicBodyCount() == 0,
                    "Actual dynamic PA bodies stay on the near side of the wall throughout the approach");
            }
            else
            {
                check(maximumRoot < 2.70f, "Kinematic movement stops at the original capsule radius and skin");
            }
            const float stoppedAt = character.GetActorLocation().GetZ();
            for (int i = 0; i < 60; ++i)
            {
                character.AddMovementInput(Vector3(0, 0, -1));
                world.Tick(1.0f / 60);
            }
            check(float(character.GetActorLocation().GetZ()) < stoppedAt - .5f, "Character can retreat from the wall");
        }
        std::string error;
        check(character.StopRagdoll(error) && capsule.IsHorizontalBlockingEnabled(),
            "Returning to animation immediately restores capsule wall blocking");
        check(physics->GetErrorCount() == errors, "No PhysX errors in mode-specific wall collision");
        wall->release();
        report << "PASS: imported character capsule/PA wall blocking" << std::endl;
        return true;
    }
    catch (const std::exception& exception)
    {
        if (wall) wall->release();
        report << "FAILED: " << exception.what() << std::endl;
        return false;
    }
}

// Exercise the saved PA on the imported animations, including the switch before
// the first physics step. Contacts and drive response cannot explain a jump here.
bool RunPATransitionValidation(PARS::World& world, PARS::Character& character, const std::wstring& reportPath)
{
    std::ofstream report(reportPath);
    std::ofstream samples(reportPath + L".csv");
    std::string error;
    const auto check = [&](bool condition, const char* message) {
        report << (condition ? "PASS: " : "FAIL: ") << message << std::endl;
        if (!condition) throw std::runtime_error(message);
    };
    try
    {
        auto& mesh = character.GetMesh();
        auto& runtime = mesh.GetRagdoll();
        auto& animation = static_cast<PreviewAnimInstance&>(mesh.GetAnimInstance());
        auto* physics = world.GetPhysicsWorld();
        check(report.good() && samples.good() && physics && mesh.GetPhysicsAsset(), "Loaded character PA and physics world");
        std::wstring testAsset;
        if (CommandLineArgs::GetString(L"patestasset", testAsset))
        {
            const int length = WideCharToMultiByte(CP_UTF8, 0, testAsset.data(), static_cast<int>(testAsset.size()), nullptr, 0, nullptr, nullptr);
            check(length > 0, "Comparison PA path is valid UTF-8");
            std::string path(length, '\0');
            WideCharToMultiByte(CP_UTF8, 0, testAsset.data(), static_cast<int>(testAsset.size()), &path[0], length, nullptr, nullptr);
            PARS::PhysicsAsset candidate;
            check(candidate.Load(path, error) && mesh.SetPhysicsAsset(candidate, error), "Load comparison PA");
        }
        uint32_t contacts = 1, diagnostic = 0;
        CommandLineArgs::GetInteger(L"patestcontacts", contacts);
        CommandLineArgs::GetInteger(L"patestdiagnostic", diagnostic);
        runtime.SetCollisionEnabled(contacts != 0);
        samples << "clip,step,bone,world_rotation_error_rad,joint_error_rad,clamp_rad,position_error_m,actual_x,actual_y,actual_z,target_x,target_y,target_z\n";
        const auto& asset = *mesh.GetPhysicsAsset();
        PARS::PhysicsSkeleton skeleton;
        check(mesh.BuildPhysicsSkeleton(skeleton, error), "Imported skeleton is available");
        animation.SetLocomotionEnabled(false);
        animation.SetAutoCycleEnabled(false);
        world.SetPhysicsPaused(false);
        world.SetPhysicsTimeScale(1);
        constexpr float dt = 1.0f / 60;
        for (uint32_t clip = 0; clip < 3; ++clip)
        {
            check(character.StopRagdoll(error), "Restore kinematic animation");
            character.GetCharacterMovement().StopMovementImmediately();
            animation.PlayAnimation(clip);
            physics->ResetClock();
            for (unsigned step = 0; step < 24; ++step) world.Tick(dt);
            std::vector<Matrix4> before, after;
            check(PARS::EvaluatePhysicsPose(skeleton, animation.GetPose(), Matrix4(mesh.GetComponentTransform()), before, error),
                "Evaluate animated pose before switching");
            const auto steps = physics->GetStepCount();
            check(character.SetPhysicsMode(PARS::PhysicsAnimationMode::ActiveRagdoll, error) && runtime.AreDrivesEnabled(),
                "Switch kinematic PA to active ragdoll");
            check(physics->GetStepCount() == steps, "No physics step has run during the switch");
            check(PARS::EvaluatePhysicsPose(skeleton, mesh.GetFinalPose(), Matrix4(mesh.GetComponentTransform()), after, error),
                "Evaluate skinning pose after switching");
            float maxPosition = 0, maxBasis = 0;
            for (size_t node = 0; node < before.size(); ++node)
            {
                maxPosition = Max(maxPosition, float(Length(Vector3(after[node].GetW()) - Vector3(before[node].GetW()))));
                const auto a = PARS::PhysicsBoneFrame(before[node]), b = PARS::PhysicsBoneFrame(after[node]);
                maxBasis = Max(maxBasis, Max(float(Length(Vector3(a.GetX()) - Vector3(b.GetX()))),
                    float(Length(Vector3(a.GetY()) - Vector3(b.GetY())))));
            }
            for (size_t body = 0; body < asset.bodies.size(); ++body)
            {
                if (asset.bodies[body].bone.find("Foot") == std::string::npos) continue;
                const int node = runtime.GetBoneNodes()[body];
                const Vector3 delta = Vector3(runtime.GetBoneWorld(body).GetW()) - Vector3(before[node].GetW());
                report << "clip=" << clip << " bone=" << asset.bodies[body].bone
                    << " switch_delta_m=" << float(delta.GetX()) << ',' << float(delta.GetY()) << ',' << float(delta.GetZ())
                    << " distance_m=" << float(Length(delta)) << std::endl;
            }
            report << "clip=" << clip << " max_skin_position_delta_m=" << maxPosition << " max_basis_delta=" << maxBasis << std::endl;
            if (!diagnostic) check(maxPosition < .001f && maxBasis < .001f, "Active transition preserves all visible bones within 1 mm");
            check(runtime.SetRootSupported(true, error), "Enable active pelvis support");
            float maxAnchor = 0;
            float maxHeadAngle = 0, maxFootAngle = 0, maxFootClamp = 0;
            for (unsigned step = 0; step < 480; ++step)
            {
                world.Tick(dt);
                maxAnchor = Max(maxAnchor, runtime.GetMaxAnchorError());
                if (!runtime.IsFinite()) check(false, "Active simulation remains finite");
                std::vector<Matrix4> target;
                if (!PARS::EvaluatePhysicsPose(skeleton, animation.GetPose(), Matrix4(mesh.GetComponentTransform()), target, error))
                    check(false, "Evaluate active animation target");
                const auto angle = [](Quaternion a, Quaternion b) {
                    return 2.0f * std::acos(Clamp(std::abs(XMVectorGetX(XMVector4Dot(Normalize(a), Normalize(b)))), 0.0f, 1.0f));
                };
                for (const auto& joint : asset.constraints)
                {
                    const int parent = asset.FindBody(joint.parentBone), child = asset.FindBody(joint.childBone);
                    const auto parentLocal = asset.bodies[parent].GetLocalTransform(), childLocal = asset.bodies[child].GetLocalTransform();
                    const auto desiredBone = PARS::PhysicsBoneFrame(target[runtime.GetBoneNodes()[child]]);
                    const auto actualBone = runtime.GetBoneWorld(child);
                    const auto desired = PARS::EvaluateJointDriveTarget(joint,
                        PARS::PhysicsBoneFrame(target[runtime.GetBoneNodes()[parent]]) * parentLocal, desiredBone * childLocal, false);
                    const auto limited = PARS::ClampArticulationDriveTarget(joint, desired);
                    const auto actual = PARS::EvaluateJointDriveTarget(joint, runtime.GetBoneWorld(parent) * parentLocal, actualBone * childLocal, false);
                    const auto actualVector = PARS::JointRotationVector(actual), targetVector = PARS::JointRotationVector(desired);
                    const float worldAngle = angle(Quaternion(Matrix3(desiredBone)), Quaternion(Matrix3(actualBone)));
                    const float clampAngle = angle(desired, limited);
                    if (step >= 120)
                    {
                        if (joint.childBone.find("Head") != std::string::npos) maxHeadAngle = Max(maxHeadAngle, worldAngle);
                        if (joint.childBone.find("Foot") != std::string::npos)
                        {
                            maxFootAngle = Max(maxFootAngle, worldAngle);
                            maxFootClamp = Max(maxFootClamp, clampAngle);
                        }
                    }
                    samples << clip << ',' << step << ',' << joint.childBone << ','
                        << worldAngle << ',' << angle(limited, actual) << ',' << clampAngle << ','
                        << float(Length(Vector3(desiredBone.GetW()) - Vector3(actualBone.GetW()))) << ','
                        << float(actualVector.GetX()) << ',' << float(actualVector.GetY()) << ',' << float(actualVector.GetZ()) << ','
                        << float(targetVector.GetX()) << ',' << float(targetVector.GetY()) << ',' << float(targetVector.GetZ()) << '\n';
                }
            }
            report << "clip=" << clip << " max_anchor_gap_m=" << maxAnchor
                << " target_clamp_rad=" << runtime.GetMaxTargetClampAngle() << std::endl;
            check(maxAnchor < .001f, "480 active steps preserve connected joint anchors");
            report << "clip=" << clip << " max_head_error_deg=" << XMConvertToDegrees(maxHeadAngle)
                << " max_foot_error_deg=" << XMConvertToDegrees(maxFootAngle)
                << " max_foot_clamp_deg=" << XMConvertToDegrees(maxFootClamp) << std::endl;
            if (!diagnostic)
            {
                check(maxHeadAngle < XMConvertToRadians(8), "Head follows gait without large contact-induced oscillation");
                check(maxFootAngle < XMConvertToRadians(15), "Feet follow gait with bounded contact response");
                check(maxFootClamp < XMConvertToRadians(1), "Ankle limits accommodate the imported gait");
            }
        }
        check(character.StopRagdoll(error) && physics->GetErrorCount() == 0, "Return to animation without PhysX errors");
        report << (diagnostic ? "DIAGNOSTIC: " : "PASS: ") << "saved PA active transitions and gait on Idle, Walking and SlowRun" << std::endl;
        return true;
    }
    catch (const std::exception& exception)
    {
        report << "FAIL: " << exception.what() << " | " << error << std::endl;
        character.StopRagdoll(error);
        return false;
    }
}

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
