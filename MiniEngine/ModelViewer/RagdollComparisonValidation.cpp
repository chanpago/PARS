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
        std::vector<ComparisonFrame> reference;
        for (int fps : {30, 60, 144})
        {
            RagdollComparisonRunner::Settings settings;
            settings.duration = 2;
            check(runner.Start(asset, settings), "Start independent Walking dummies");
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
            for (int frame = 0; frame < fps * 2; ++frame)
            {
                runner.Advance(1.0f / fps);
            }
            const auto& results = runner.GetResults();
            check(runner.GetState() == RagdollComparisonRunner::State::Complete && results.GetFrames().size() == 120,
                "Exactly 120 synchronized samples at 30/60/144 render FPS");
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
                for (size_t frame = 0; frame < reference.size(); ++frame)
                {
                    const auto& before = reference[frame];
                    const auto& after = results.GetFrames()[frame];
                    maxClockDelta = std::max(maxClockDelta, std::abs(before.clipTime - after.clipTime));
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
                check(maxPositionDelta < 1e-4f && maxAngleDelta < 1e-4f && maxClockDelta < 1e-6f,
                    "Every frame's movement, position, limit and tracking measurements are independent of render FPS");
            }
        }
        check(owner.GetDynamicBodyCount() == ownerBodies && owner.GetConstraintCount() == ownerJoints &&
                owner.GetStepCount() == ownerSteps,
            "Player physics scene and clock remain unchanged");
        auto& context = GraphicsContext::Begin(L"Validate comparison dummy rendering");
        Renderer::UpdateWorldRenderData(runner.GetWorld(), context);
        context.Finish(true);
        check(runner.Export("../../.tools/comparison-imported.html"), "Export actual imported Walking results");
        RagdollComparisonRunner::Settings ground;
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
