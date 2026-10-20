#include "RagdollComparisonRunner.h"
#include "Components/SkeletalMeshComponent.h"
#include "PhysicsWorld.h"
#include "WorldRenderer.h"
#include "CommandContext.h"
#include <fstream>
#include <stdexcept>
#include <cmath>
#include <algorithm>

using namespace PARS;

bool RunRagdollComparisonValidation(RagdollComparisonRunner& runner, const PhysicsAsset& asset,
    PhysicsWorld& owner, const std::wstring& reportPath)
{
    std::ofstream report(reportPath);
    const auto check = [&](bool condition, const char* message) {
        report << (condition ? "PASS: " : "FAIL: ") << message << " | " << runner.GetStatusMessage() << std::endl;
        if (!condition)
        {
            throw std::runtime_error(message);
        }
    };
    const auto ownerBodies = owner.GetDynamicBodyCount(), ownerJoints = owner.GetConstraintCount();
    const auto ownerSteps = owner.GetStepCount();
    const auto initialErrors = owner.GetErrorCount();
    try
    {
        check(report.good(), "Open imported-character comparison validation");
        runner.RequestLocomotionComparison(&asset, nullptr);
        check(runner.GetState() == RagdollComparisonRunner::State::Empty && runner.GetWorld().GetActorCount() == 0,
            "Editor button queues creation without changing actors during UI rendering");
        runner.ProcessRequests();
        check(runner.GetState() == RagdollComparisonRunner::State::Running && runner.GetWorld().GetActorCount() == 2,
            "Editor button starts the Idle/Walking/Slow Run sequence at the next safe update");
        runner.SetPaused(true);
        check(runner.Step(), "Editor-launched comparison can pause and step");
        runner.RequestLocomotionComparison(&asset, nullptr);
        runner.ProcessRequests();
        check(runner.GetState() == RagdollComparisonRunner::State::Paused && runner.GetResults().GetFrames().size() == 1,
            "Repeated launch button keeps an existing recording");
        for (int i = 1; i < 120; ++i)
        {
            if (!runner.Step())
            {
                check(false, "Step through the initial Idle stage");
            }
        }
        check(runner.GetResults().GetFrames().back().clipName == "Idle" &&
                std::abs(float(runner.GetViewCenter().GetZ())) < 1e-6f,
            "Idle lasts two seconds without prescribed forward travel");
        check(runner.Step() && runner.GetResults().GetFrames().back().clipName == "Walking" &&
                std::abs(runner.GetResults().GetFrames().back().blendAlpha - 1.0f / 12) < 1e-5f,
            "The first Walking sample uses the existing 0.2-second pose blend");
        const auto pausedBlend = runner.GetResults().GetFrames().back().blendAlpha;
        runner.Advance(1);
        check(runner.GetResults().GetFrames().size() == 121 &&
                runner.GetResults().GetFrames().back().blendAlpha == pausedBlend,
            "Pausing during a transition preserves the recording and blend clock");
        check(runner.Step() && runner.GetResults().GetFrames().back().blendAlpha > pausedBlend,
            "Single stepping advances the transition by one physics step");
        runner.Reset();
        std::vector<ComparisonFrame> reference;
        for (int fps : {30, 60, 144})
        {
            RagdollComparisonRunner::Settings settings;
            check(runner.Start(asset, settings), "Start independent locomotion-sequence dummies");
            check(runner.GetWorld().GetActorCount() == 2 &&
                    runner.GetWorld().GetPhysicsWorld()->GetDynamicBodyCount() == 2 * asset.bodies.size(),
                "Two non-player actors with complete rigid body sets");
            size_t kinematic = 0, dynamic = 0;
            runner.GetWorld().ForEachActor([&](Actor& actor) {
                for (const auto& component : actor.GetComponents())
                {
                    if (auto* mesh = dynamic_cast<SkeletalMeshComponent*>(component.get()))
                    {
                        kinematic += mesh->GetRagdoll().GetKinematicBodyCount();
                        dynamic += mesh->GetRagdoll().GetBodyCount() - mesh->GetRagdoll().GetKinematicBodyCount();
                    }
                }
            });
            check(kinematic == asset.bodies.size() && dynamic == asset.bodies.size(), "One kinematic and one fully dynamic dummy");
            runner.SetPaused(true);
            runner.Advance(1);
            check(runner.GetResults().GetFrames().empty(), "Pause produces no samples");
            check(runner.Step() && runner.GetResults().GetFrames().size() == 1, "Single step captures once after physics");
            check(runner.Start(asset, settings), "Restart resets clocks and measurements");
            for (int frame = 0; frame < fps * 10; ++frame)
            {
                runner.Advance(1.0f / fps);
            }
            const auto& results = runner.GetResults();
            check(runner.GetState() == RagdollComparisonRunner::State::Complete && results.GetFrames().size() == 600,
                "Exactly 600 synchronized samples at 30/60/144 render FPS");
            bool correctStages = true;
            for (size_t i = 0; i < results.GetFrames().size(); ++i)
            {
                const auto& sample = results.GetFrames()[i];
                const auto* name = i < 120 ? "Idle" : i < 360 ? "Walking"
                                                              : "Slow Run";
                const size_t start = i < 120 ? 0 : i < 360 ? 120
                                                           : 360;
                const float alpha = i < 120 ? 1 : std::min(1.0f, float(i - start + 1) / 12);
                correctStages &= sample.clipName == name && std::abs(sample.blendAlpha - alpha) < 1e-5f;
            }
            check(correctStages, "Record Idle for 120 steps, Walking for 240, Slow Run for 240, including both complete blends");
            check(std::abs(results.GetFrames()[120].clipTime - results.GetStep()) < 1e-6f &&
                    std::abs(results.GetFrames()[360].clipTime - results.GetStep()) < 1e-6f,
                "Each new animation begins at its first physics sample");
            check(std::abs(float(runner.GetViewCenter().GetZ()) - 18.1f) < 1e-4f,
                "Camera follows the integrated 0/1.6/3 m/s path with continuous speed ramps");
            float maxClamp = 0;
            for (const auto& joint : results.GetJoints())
            {
                maxClamp = std::max(maxClamp, joint.clamp.maximum);
            }
            report << "fps=" << fps << " pelvis_max_m=" << results.GetBodies()[0].position.maximum
                   << " target_clamp_max_rad=" << maxClamp << std::endl;
            check(results.GetBodies()[0].position.maximum < .5f, "Initial spacing is removed while finite tracking errors remain");
            if (reference.empty())
            {
                reference = results.GetFrames();
            }
            else
            {
                float maxPositionDelta = 0, maxAngleDelta = 0, maxClockDelta = 0;
                bool samePlayback = true;
                for (size_t frame = 0; frame < reference.size(); ++frame)
                {
                    const auto& before = reference[frame];
                    const auto& after = results.GetFrames()[frame];
                    maxClockDelta = std::max(maxClockDelta, std::abs(before.clipTime - after.clipTime));
                    samePlayback &= before.clipName == after.clipName && std::abs(before.blendAlpha - after.blendAlpha) < 1e-6f;
                    for (size_t body = 0; body < before.bodies.size(); ++body)
                    {
                        maxPositionDelta = std::max(maxPositionDelta, std::abs(before.bodies[body].positionError - after.bodies[body].positionError));
                        maxPositionDelta = std::max(maxPositionDelta, std::abs(before.bodies[body].movementError - after.bodies[body].movementError));
                    }
                    for (size_t joint = 0; joint < before.joints.size(); ++joint)
                    {
                        maxAngleDelta = std::max(maxAngleDelta, std::abs(before.joints[joint].clampError - after.joints[joint].clampError));
                        maxAngleDelta = std::max(maxAngleDelta, std::abs(before.joints[joint].trackingError - after.joints[joint].trackingError));
                    }
                }
                report << "repeat_max_position_delta_m=" << maxPositionDelta << " repeat_max_angle_delta_rad=" << maxAngleDelta
                       << " repeat_max_clip_delta_s=" << maxClockDelta << std::endl;
                check(samePlayback && maxPositionDelta < 1e-4f && maxAngleDelta < 1e-4f && maxClockDelta < 1e-6f,
                    "Every frame's playback, blends, movement, limits and tracking are independent of render FPS");
            }
        }
        check(owner.GetDynamicBodyCount() == ownerBodies && owner.GetConstraintCount() == ownerJoints &&
                owner.GetStepCount() == ownerSteps,
            "Player physics scene and clock remain unchanged");
        auto& context = GraphicsContext::Begin(L"Validate comparison dummy rendering");
        Renderer::UpdateWorldRenderData(runner.GetWorld(), context);
        context.Finish(true);
        check(runner.Export("../../.tools/comparison-imported.html"), "Export actual imported locomotion-sequence results");
        const char* clipReports[] = {"../../.tools/comparison-pa-idle.html",
            "../../.tools/comparison-pa-walking.html", "../../.tools/comparison-pa-slowrun.html"};
        const float clipSpeeds[] = {0, 1.6f, 3.0f};
        for (int clip = 0; clip < 3; ++clip)
        {
            RagdollComparisonRunner::Settings settings;
            settings.sequence = false;
            settings.clip = clip;
            settings.speed = clipSpeeds[clip];
            check(runner.Start(asset, settings), "Start six-second locomotion PA validation");
            for (int i = 0; i < 360; ++i)
            {
                if (!runner.Step())
                {
                    check(false, "Capture locomotion PA sample");
                }
            }
            const auto& results = runner.GetResults();
            check(runner.GetState() == RagdollComparisonRunner::State::Complete &&
                    results.GetFrames().size() == 360,
                "Record the complete locomotion clip comparison");
            check(runner.Export(clipReports[clip]), "Export six-second locomotion PA results");
            for (size_t i = 0; i < asset.constraints.size(); ++i)
            {
                const auto& bone = asset.constraints[i].childBone;
                const auto separator = bone.find_last_of(':');
                const auto name = separator == std::string::npos ? bone : bone.substr(separator + 1);
                const auto& joint = results.GetJoints()[i];
                report << "clip=" << clip << " joint=" << bone
                       << " clamp_mean_deg=" << DirectX::XMConvertToDegrees(joint.clamp.Mean())
                       << " clamp_max_deg=" << DirectX::XMConvertToDegrees(joint.clamp.maximum)
                       << " tracking_mean_deg=" << DirectX::XMConvertToDegrees(joint.tracking.Mean())
                       << " tracking_max_deg=" << DirectX::XMConvertToDegrees(joint.tracking.maximum) << std::endl;
                check(joint.clamp.maximum < 1e-4f && joint.singularFrames == 0,
                    "Every locomotion joint target fits the finite PA limits without clamping or singularities");
                for (const auto& violation : joint.violations)
                {
                    check(violation.frames == 0, "No raw twist, swing or coupled-cone limit violations");
                }
                for (const auto& violation : joint.actualViolations)
                {
                    check(violation.frames == 0,
                        "The current physical tracking trajectory also stays inside the finite PA limits");
                }
                if (name == "LeftForeArm" || name == "RightForeArm" || name == "LeftLeg" || name == "RightLeg")
                {
                    check(joint.tracking.Mean() < DirectX::XMConvertToRadians(5) &&
                            joint.tracking.maximum < DirectX::XMConvertToRadians(15),
                        "Elbow/knee drives track the admitted auxiliary rotations");
                }
            }
            if (clip == 1)
            {
                for (size_t i = 0; i < asset.bodies.size(); ++i)
                {
                    if (asset.bodies[i].bone.find("Hand") != std::string::npos)
                    {
                        report << "walking_hand=" << asset.bodies[i].bone
                               << " position_mean_m=" << results.GetBodies()[i].position.Mean() << std::endl;
                        check(results.GetBodies()[i].position.Mean() < .06f,
                            "Walking hand position error improves beyond the locked-elbow baseline");
                    }
                }
            }
        }
        std::string assetError;
        check(asset.Save("../../.tools/comparison-pa.physics.json", assetError), "Save the validated physics asset");
        RagdollComparisonRunner::Settings ground;
        ground.sequence = false;
        ground.duration = 1;
        ground.suspended = false;
        check(runner.Start(asset, ground), "Start floor-contact comparison");
        for (int i = 0; i < 60; ++i)
        {
            runner.Step();
        }
        check(runner.GetState() == RagdollComparisonRunner::State::Complete, "Floor-contact recording completes");
        runner.Reset();
        check(runner.GetWorld().GetActorCount() == 0 && runner.GetWorld().GetPhysicsWorld()->GetDynamicBodyCount() == 0,
            "Reset releases both dummies and all joints");
        check(owner.GetErrorCount() == initialErrors, "No PhysX errors");
        report << "PASS: all ragdoll comparison integration checks" << std::endl;
        return true;
    }
    catch (const std::exception& exception)
    {
        report << "FAILED: " << exception.what() << std::endl;
        return false;
    }
}
