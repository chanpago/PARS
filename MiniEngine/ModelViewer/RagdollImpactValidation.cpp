#include "RagdollImpactTest.h"
#include "GameFramework/Character.h"
#include "GameFramework/World.h"
#include "GameFramework/PlayerController.h"
#include "PreviewAnimInstance.h"
#include "Util/CommandLineArg.h"
#include <algorithm>
#include <fstream>
#include <stdexcept>
#include <cmath>
#include <cstdio>
#include <functional>
#include <limits>
#include <array>
#include <cstring>

namespace
{
    struct ExpectedMotionClip
    {
    public:
        const char* name;
        size_t frameCount;
        PARS::WalkingSwing crossedLanding{};
    };

    constexpr ExpectedMotionClip kIdleClip = {"Idle", 501};
    constexpr ExpectedMotionClip kWalkingClips[] =
    {
        {"M_Neutral_Walk_Loop_F", 243},
        {"M_Neutral_Walk_Loop_B", 245},
        {"M_Neutral_Walk_Loop_FL", 243, {1, PARS::FootstepMode::FrontCross}},
        {"M_Neutral_Walk_Loop_FR", 243, {0, PARS::FootstepMode::FrontCross}},
        {"M_Neutral_Walk_Loop_BL", 245, {1, PARS::FootstepMode::BackCross}},
        {"M_Neutral_Walk_Loop_BR", 245, {0, PARS::FootstepMode::BackCross}},
        {"M_Neutral_Walk_Loop_LL", 225, {1, PARS::FootstepMode::BackCross}},
        {"M_Neutral_Walk_Loop_LR", 241, {1, PARS::FootstepMode::FrontCross}},
        {"M_Neutral_Walk_Loop_RL", 241, {0, PARS::FootstepMode::FrontCross}},
        {"M_Neutral_Walk_Loop_RR", 225, {0, PARS::FootstepMode::BackCross}}
    };

    template <size_t ClipCount>
    void CheckMotionFrameRanges(const PARS::AnimInstance& animation, const ExpectedMotionClip (&walkingClips)[ClipCount],
        const std::function<void(bool, const char*)>& check)
    {
        const std::vector<PARS::MotionFrame>& frames = animation.GetMotionFrames();
        size_t walkingFrameCount = 0;
        for (const ExpectedMotionClip& clip : walkingClips)
        {
            walkingFrameCount += clip.frameCount;
        }
        size_t total = kIdleClip.frameCount + walkingFrameCount;
        for (PARS::MotionFrameUsage usage : {PARS::MotionFrameUsage::BalanceUpperBody, PARS::MotionFrameUsage::ShoveFall})
        {
            const PARS::MotionFrameRange range = animation.GetMotionFrameRange(usage);
            check(range.firstFrameIndex == total, "Impact frames follow the previous usage without a gap");
            total += range.frameCount;
        }
        check(frames.size() == total, "Only the selected clips contribute motion candidates");
        for (PARS::MotionFrameUsage usage : {PARS::MotionFrameUsage::Idle, PARS::MotionFrameUsage::Walking})
        {
            const PARS::MotionFrameRange range = animation.GetMotionFrameRange(usage);
            const bool walking = usage == PARS::MotionFrameUsage::Walking;
            check(range.firstFrameIndex == (walking ? kIdleClip.frameCount : 0) &&
                range.frameCount == (walking ? walkingFrameCount : kIdleClip.frameCount),
                "The usage range points to the expected first frame and count");
            size_t clipStart = range.firstFrameIndex;
            const size_t clipCount = walking ? ClipCount : 1;
            for (size_t i = 0; i < clipCount; ++i)
            {
                const ExpectedMotionClip& clip = walking ? walkingClips[i] : kIdleClip;
                bool valid = true;
                bool hasNormal = false;
                bool hasCross = false;
                for (size_t offset = 0; offset < clip.frameCount; ++offset)
                {
                    const PARS::MotionFrame& frame = frames[clipStart + offset];
                    valid = frame.usage == usage && animation.GetAnimationName(frame.clipIndex) == clip.name &&
                        std::abs(frame.clipTimeSeconds - static_cast<float>(offset) / 60.0f) < 2e-6f;
                    if (walking)
                    {
                        valid = valid && frame.swing.movingFoot < 2;
                        if (frame.swing.mode == PARS::FootstepMode::Normal)
                        {
                            hasNormal = true;
                        }
                        else
                        {
                            hasCross = true;
                            valid = valid && frame.swing.movingFoot == clip.crossedLanding.movingFoot &&
                                frame.swing.mode == clip.crossedLanding.mode;
                        }
                    }
                    if (!valid)
                    {
                        const std::string message = std::string(clip.name) + " frame " + std::to_string(offset) +
                            ", moving foot " + std::to_string(frame.swing.movingFoot) +
                            ", mode " + std::to_string(static_cast<int>(frame.swing.mode));
                        check(false, message.c_str());
                    }
                }
                check(valid, clip.name);
                if (walking)
                {
                    check(hasNormal && hasCross == (clip.crossedLanding.movingFoot < 2),
                        "Walking swing labels reproduce the ordinary and crossed landings in the source FBX");
                }
                const PARS::MotionFrame& source = frames[clipStart + 30];
                const PARS::MotionMatchResult match = animation.FindMotionMatch(usage, source.features);
                check(match.clipIndex == source.clipIndex && match.clipTimeSeconds == source.clipTimeSeconds && match.score == 0.0f,
                    "An exact motion query finds the sampled clip and time with zero error");
                clipStart += clip.frameCount;
            }
        }
    }

    void CheckWalkingTravelDirection(const PARS::AnimInstance& animation, const PARS::MotionFrame& backwardFrame,
        const std::function<void(bool, const char*)>& check)
    {
        const float lateral = PARS::GetWalkingTravelAngleDegrees({0, 1}, {2, 0});
        const float rotated = PARS::GetWalkingTravelAngleDegrees({1, 0}, {0, -2});
        check(std::abs(lateral - 90.0f) < 1e-4f && std::abs(rotated - lateral) < 1e-4f,
            "Rotating the character and its COM motion together preserves the signed walking direction");
        check(!std::isfinite(PARS::GetWalkingTravelAngleDegrees({0, 0}, {2, 0})),
            "An undefined character forward supplies no direction preference");

        struct DirectionCase
        {
        public:
            const char* expectedClip;
            PARS::WalkingSwing swing;
            float requestedDegrees;
        };

        const DirectionCase cases[] =
        {
            {"M_Neutral_Walk_Loop_F", {1, PARS::FootstepMode::Normal}, 0.0f},
            {"M_Neutral_Walk_Loop_F", {1, PARS::FootstepMode::Normal}, 88.4f},
            {"M_Neutral_Walk_Loop_B", {1, PARS::FootstepMode::Normal}, -179.8f},
            {"M_Neutral_Walk_Loop_B", {0, PARS::FootstepMode::Normal}, 179.8f},
            {"M_Neutral_Walk_Loop_LR", {1, PARS::FootstepMode::FrontCross}, 90.0f},
            {"M_Neutral_Walk_Loop_RL", {0, PARS::FootstepMode::FrontCross}, -90.0f},
            {"M_Neutral_Walk_Loop_LL", {1, PARS::FootstepMode::BackCross}, 90.0f},
            {"M_Neutral_Walk_Loop_RR", {0, PARS::FootstepMode::BackCross}, -90.0f},
            {"M_Neutral_Walk_Loop_FL", {1, PARS::FootstepMode::FrontCross}, 45.0f},
            {"M_Neutral_Walk_Loop_FR", {0, PARS::FootstepMode::FrontCross}, -45.0f},
            {"M_Neutral_Walk_Loop_BL", {1, PARS::FootstepMode::BackCross}, 135.0f},
            {"M_Neutral_Walk_Loop_BR", {0, PARS::FootstepMode::BackCross}, -135.0f}
        };
        PARS::MotionFeatures query = backwardFrame.features;
        for (const DirectionCase& test : cases)
        {
            query.walkingAngleDegrees = test.requestedDegrees;
            const PARS::MotionMatchResult match = animation.FindMotionMatch(PARS::MotionFrameUsage::Walking, query, test.swing);
            check(animation.GetAnimationName(match.clipIndex) == test.expectedClip &&
                    match.requestedAngleDegrees == test.requestedDegrees && std::isfinite(match.clipAngleDegrees),
                test.expectedClip);
        }
        query.walkingAngleDegrees = PARS::GetWalkingTravelAngleDegrees({0, 1}, {0.001f, 0.001f});
        const PARS::MotionMatchResult resting = animation.FindMotionMatch(PARS::MotionFrameUsage::Walking, query, backwardFrame.swing);
        check(!std::isfinite(resting.requestedAngleDegrees) && resting.clipIndex == backwardFrame.clipIndex &&
                resting.clipTimeSeconds == backwardFrame.clipTimeSeconds && resting.score == 0.0f,
            "Near-stationary COM falls back to the existing pose and displacement score");
    }

    void ConfigureLandingComValidation(PARS::SkeletalMeshComponent& mesh, std::ostream& report)
    {
        PARS::FootstepSettings& settings = mesh.GetFootSupport().GetFootstepSettings();
        uint32_t enabled = settings.landingComClampEnabled ? 1 : 0;
        CommandLineArgs::GetInteger(L"landingcomclamp", enabled);
        settings.landingComClampEnabled = enabled != 0;
        uint32_t recoveryEnabled = settings.recovery.enabled ? 1 : 0;
        CommandLineArgs::GetInteger(L"footsteprecovery", recoveryEnabled);
        settings.recovery.enabled = recoveryEnabled != 0;
        uint32_t recoverySteps = settings.recovery.maximumSteps;
        CommandLineArgs::GetInteger(L"footsteprecoverysteps", recoverySteps);
        settings.recovery.maximumSteps = physx::PxClamp(recoverySteps, uint32_t(1), uint32_t(3));
        CommandLineArgs::GetFloat(L"footstepcapturescale", settings.recovery.oneStepCaptureScale);
        CommandLineArgs::GetFloat(L"backwardliftratio", settings.backwardLiftTimeRatio);
        CommandLineArgs::GetFloat(L"landingcomhysteresis", settings.landingComClampHysteresisDegrees);
        report << "landing_com_clamp=" << settings.landingComClampEnabled
            << " region=opposite_grounded_sole"
            << " hysteresis=" << settings.landingComClampHysteresisDegrees
            << " minimum_stance_ratio=" << settings.minimumStanceWidthRatio
            << " prevent_crossing=" << settings.preventCrossing << std::endl;
        report << "footstep_recovery=" << settings.recovery.enabled << " maximum_steps=" << settings.recovery.maximumSteps
            << " one_step_capture_scale=" << settings.recovery.oneStepCaptureScale
            << " backward_lift_time_ratio=" << settings.backwardLiftTimeRatio << std::endl;
    }

    void CheckImpactMotionClips(const PARS::SkeletalMeshComponent& mesh, const std::function<void(bool, const char*)>& check)
    {
        const PARS::AnimInstance& animation = mesh.GetAnimInstance();
        PARS::PhysicsSkeleton skeleton;
        std::string error;
        check(mesh.BuildPhysicsSkeleton(skeleton, error), "Resolve the live dummy skeleton for the upper-body mask");
        const std::array<const char*, 5> names = {"Slow Jog Backwards", "ShoveFallForward", "ShoveFallBackward", "ShoveFallLeft", "ShoveFallRight"};
        size_t slot = 0;
        for (PARS::MotionFrameUsage usage : {PARS::MotionFrameUsage::BalanceUpperBody, PARS::MotionFrameUsage::ShoveFall})
        {
            const PARS::MotionFrameRange range = animation.GetMotionFrameRange(usage);
            if (usage == PARS::MotionFrameUsage::BalanceUpperBody)
            {
                check(range.frameCount == 44, "All 44 backward jog samples are indexed for upper-body balance recovery");
            }
            uint32_t previous = UINT32_MAX;
            for (size_t index = range.firstFrameIndex; index < range.firstFrameIndex + range.frameCount; ++index)
            {
                const PARS::MotionFrame& frame = animation.GetMotionFrames()[index];
                if (frame.clipIndex == previous)
                {
                    continue;
                }
                previous = frame.clipIndex;
                check(slot < names.size() && animation.GetAnimationName(previous) == names[slot], "Registered clip matches its balance or fall usage");
                ++slot;
                const PARS::MotionMatchResult match = animation.FindImpactPoseMatch(usage, previous);
                check(match.clipIndex == previous && std::isfinite(match.score) && match.clipTimeSeconds < animation.GetAnimationDuration(previous),
                    "Live physical pose matches a playable frame of the registered clip");
                if (usage == PARS::MotionFrameUsage::ShoveFall)
                {
                    continue;
                }
                std::vector<GraphNode> layer;
                std::vector<GraphNode> authored;
                check(animation.SampleAnimationPose(previous, .3f, authored) && animation.SampleUpperBodyPose(previous, .3f, 1, layer),
                    "Sample the authored and upper-body impact poses");
                bool movesUpperBody = false;
                bool matchesMask = true;
                for (size_t node = 0; node < layer.size(); ++node)
                {
                    int ancestor = static_cast<int>(node);
                    while (ancestor >= 0 && skeleton.names[ancestor].substr(skeleton.names[ancestor].rfind(':') + 1) != "Spine")
                    {
                        ancestor = skeleton.parents[ancestor];
                    }
                    const GraphNode& base = animation.GetPose()[node];
                    const GraphNode& expected = ancestor >= 0 ? authored[node] : base;
                    movesUpperBody |= ancestor >= 0 && std::memcmp(&layer[node].rotation, &base.rotation, sizeof(base.rotation)) != 0;
                    matchesMask &= std::memcmp(&layer[node].rotation, &expected.rotation, sizeof(expected.rotation)) == 0 &&
                        std::memcmp(&layer[node].scale, &expected.scale, sizeof(expected.scale)) == 0 &&
                        Math::LengthSquare(Math::Vector3(layer[node].xform.GetW()) - Math::Vector3(expected.xform.GetW())) < 1e-10f;
                }
                check(matchesMask && movesUpperBody, "Spine and descendants follow Slow Jog Backwards while the pelvis and legs keep the base pose");
            }
        }
        check(slot == names.size(), "The balance clip and all four shove falls are indexed on the shared dummy database");
    }

    void WriteLandingComHeader(std::ostream& stream)
    {
        stream << "frame,physics_step,side,mode,root_release,recovery,phase,started,landed,moving,grounded,locked,"
            "enabled,limited,direction,prediction_valid,com_x,com_z,start_x,start_z,raw_x,raw_z,clamped_x,clamped_z,"
            "target_x,target_z,sole_x,sole_y,sole_z,leg_contacts,recovery_count,return_count,landing_error,falling,standing_up,"
            "region_valid,support_foot,region_origin_x,region_origin_z,front_ray_x,front_ray_z,back_ray_x,back_ray_z,region_width_degrees,"
            "start_raw_x,start_raw_z,start_limited,start_region_valid,start_support_foot,start_inside_forbidden,needs_step,"
            "decision_mode,decision_status,decision_has_target,decision_feasible,decision_pose_checks,decision_ms,"
            "normal_area,front_area,back_area,capture_pieces,decision_steps,path_rejected,"
            "animation_clip,animation_time_s,com_velocity_x,com_velocity_z,target_in_capture_margin,balance_state,"
            "actual_com_outside_b,outside_b_time_s,"
            "has_support_deadline,trajectory_duration_s,trajectory_elapsed_s,landing_time_error_s,swing_target_y,backward_fall_steps_remaining,upper_body_clip,upper_body_time_s\n";
    }

    void RecordLandingComFrame(std::ostream& stream, PARS::SkeletalMeshComponent& mesh, uint32_t frame)
    {
        const PARS::FootSupportSnapshot& support = mesh.GetFootSupport().GetSnapshot();
        const PARS::RecoverySnapshot& recovery = mesh.GetRecoverySnapshot();
        const PARS::AnimInstance& animation = mesh.GetAnimInstance();
        const PARS::FootstepComPrediction& start = support.footstep.startCom;
        FootSoleDebug debug;
        std::string contactText;
        const uint32_t contacts = debug.GetLegContacts(&mesh, contactText);
        for (size_t side = 0; side < support.feet.size(); ++side)
        {
            const PARS::FootstepCommand& command = support.footstep.feet[side];
            const PARS::FootstepComPrediction& landing = support.footstep.candidates[side].landingCom;
            const PARS::FootstepRecoverySnapshot& decision = support.footstep.candidates[side].recovery;
            const Math::Vector3 sole(support.feet[side].soleWorld.GetW());
            // 현재 예측, 잠긴 명령 목표, 실제 발바닥과 회복 결과를 서로 다른 열로 기록한다.
            stream << frame << ',' << support.completedPhysicsStep << ',' << side
                << ',' << static_cast<int>(mesh.GetPhysicsMode()) << ',' << static_cast<int>(mesh.GetRootSupportRelease())
                << ',' << static_cast<int>(recovery.state) << ',' << static_cast<int>(command.phase)
                << ',' << command.justStarted << ',' << command.justLanded << ',' << command.IsInProgress()
                << ',' << support.feet[side].contact.grounded << ',' << command.motion.landingTargetLocked
                << ',' << landing.enabled << ',' << landing.limited << ',' << static_cast<int>(landing.direction)
                << ',' << landing.valid << ',' << support.centerOfMass.x << ',' << support.centerOfMass.z
                << ',' << support.footstep.predictedCom.x << ',' << support.footstep.predictedCom.z
                << ',' << landing.raw.x << ',' << landing.raw.z << ',' << landing.clamped.x << ',' << landing.clamped.z
                << ',' << command.target.position.x << ',' << command.target.position.z
                << ',' << float(sole.GetX()) << ',' << float(sole.GetY()) << ',' << float(sole.GetZ())
                << ',' << contacts << ',' << recovery.recoveryCount << ',' << recovery.returnCount
                << ',' << command.motion.landingError << ',' << mesh.IsFollowingFallingAnimation()
                << ',' << mesh.IsFollowingStandingUpAnimation() << ',' << landing.region.valid << ',' << landing.region.supportFoot
                << ',' << landing.region.origin.x << ',' << landing.region.origin.z
                << ',' << landing.region.frontRay.x << ',' << landing.region.frontRay.z
                << ',' << landing.region.backRay.x << ',' << landing.region.backRay.z
                << ',' << landing.region.halfAngleRadians * 360.0f / physx::PxPi
                << ',' << start.raw.x << ',' << start.raw.z << ',' << start.limited
                << ',' << start.region.valid << ',' << start.region.supportFoot
                << ',' << PARS::IsInsideFootstepLandingRegion(start.region, support.footstep.predictedCom)
                << ',' << support.footstep.needsStep
                << ',' << static_cast<int>(decision.mode) << ',' << static_cast<int>(decision.status)
                << ',' << decision.hasTarget << ',' << decision.modelFeasible << ',' << decision.poseChecks
                << ',' << decision.elapsedMilliseconds << ',' << decision.overlapAreas[0]
                << ',' << decision.overlapAreas[1] << ',' << decision.overlapAreas[2]
                << ',' << decision.capturePieceCount << ',' << decision.steps << ',' << decision.pathRejected
                << ',' << animation.GetAnimationName(animation.GetCurrentAnimationIndex()) << ',' << animation.GetAnimationTime()
                << ',' << support.centerOfMassVelocity.x << ',' << support.centerOfMassVelocity.z
                << ',' << decision.targetInCaptureMargin << ',' << PARS::CharacterBalanceStateLabel(mesh.GetBalanceState())
                << ',' << support.footstep.actualComOutsideBuffer << ',' << support.footstep.outsideBufferTime
                << ',' << command.motion.hasSupportDeadline << ',' << command.motion.duration << ',' << command.motion.elapsedTime
                << ',' << command.motion.landingTimeError << ',';
            if (command.motion.valid)
            {
                const float ahead = command.motion.hasSupportDeadline ? static_cast<float>(PhysicsWorld::FixedDeltaTime) : 0.0f;
                stream << PARS::SampleFootstepPosition(command, ahead).y;
            }
            const uint32_t upper = mesh.GetImpactFlailClip();
            stream << ',' << support.footstep.backwardFallStepsRemaining << ',';
            if (upper < animation.GetAnimationCount())
            {
                stream << animation.GetAnimationName(upper);
            }
            stream << ',' << mesh.GetImpactFlailTime() << '\n';
        }
    }

    bool LandingComDebugMatchesPrediction(const PARS::SkeletalMeshComponent& mesh)
    {
        Math::Vector3 center(Math::kZero);
        float mass = 0;
        if (!mesh.GetRagdoll().GetCenterOfMass(center, mass))
        {
            return false;
        }
        const PARS::FootstepSnapshot& step = mesh.GetFootSupport().GetSnapshot().footstep;
        if (step.hasPrediction && step.requestedFoot >= 0 && step.requestedFoot < static_cast<int>(step.feet.size()))
        {
            center = Math::Vector3(step.startCom.origin.x, step.startCom.origin.y, step.startCom.origin.z);
            if (step.startCom.enabled && PARS::IsInsideFootstepLandingRegion(step.startCom.region, step.predictedCom))
            {
                return false;
            }
        }
        FootSoleDebug debug;
        std::vector<physx::PxDebugLine> lines;
        debug.BuildLines(mesh, lines);
        const physx::PxU32 regionColor = mesh.GetFootSupport().GetFootstepSettings().landingComClampEnabled ?
            0xff55cc77 : 0xff889099;
        float minimumX = PX_MAX_F32;
        float maximumX = -PX_MAX_F32;
        float minimumZ = PX_MAX_F32;
        float maximumZ = -PX_MAX_F32;
        unsigned regionLines = 0;
        for (const physx::PxDebugLine& line : lines)
        {
            if (line.color0 != regionColor || line.color1 != regionColor)
            {
                continue;
            }
            minimumX = (std::min)(minimumX, (std::min)(line.pos0.x, line.pos1.x));
            maximumX = (std::max)(maximumX, (std::max)(line.pos0.x, line.pos1.x));
            minimumZ = (std::min)(minimumZ, (std::min)(line.pos0.z, line.pos1.z));
            maximumZ = (std::max)(maximumZ, (std::max)(line.pos0.z, line.pos1.z));
            ++regionLines;
        }
        // 예측이 있으면 같은 스텝의 원점을, 없으면 현재 COM의 미리보기 원점을 검증한다.
        const float tolerance = (std::max)(2e-5f, (maximumX - minimumX + maximumZ - minimumZ) * .001f);
        return regionLines >= 64 && std::abs((minimumX + maximumX) * .5f - float(center.GetX())) <= tolerance &&
            std::abs((minimumZ + maximumZ) * .5f - float(center.GetZ())) <= tolerance;
    }
}

// 기준과 후보에서 같은 바닥·애니메이션·입력을 사용하는 실제 캐릭터 검증이다.
bool RunKinematicRecoveryValidation(RagdollImpactTest& test, PARS::World& world, PARS::Character& player,
    const std::wstring& path, uint32_t scenario)
{
    using namespace PARS;
    using namespace Math;
    std::ofstream report(path);
    std::ofstream samples(path + L".csv");
    std::ofstream feet(path + L".feet.csv");
    std::ofstream landingCom(path + L".landing-com.csv");
    landingCom.precision(9);
    uint32_t validateFootAlignment = 0;
    uint32_t validateLandingRegion = 0;
    CommandLineArgs::GetInteger(L"validatefootalignment", validateFootAlignment);
    CommandLineArgs::GetInteger(L"validatelandingregion", validateLandingRegion);
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
        check(report.good() && samples.good() && feet.good() && landingCom.good() && scenario <= 5 && test.SetAutomaticSupport(true),
            "Prepare grounded automatic kinematic dummy");
        SkeletalMeshComponent& mesh = test.GetDummy()->GetMesh();
        ConfigureLandingComValidation(mesh, report);
        WriteLandingComHeader(landingCom);
        check(!mesh.IsRagdoll() && mesh.GetRagdoll().GetKinematicBodyCount() == mesh.GetRagdoll().GetBodyCount(),
            "Idle starts with every PA body kinematic");
        check(mesh.GetKinematicStableDuration() == 2.0f, "Default recovery duration is exactly two seconds");
        std::string error;
        check(player.StopRagdoll(error), "Place projectile source away from the dummy");
        float sourceX = 3.5f;
        float sourceZ = -5.0f;
        CommandLineArgs::GetFloat(L"impactsourcex", sourceX);
        CommandLineArgs::GetFloat(L"impactsourcez", sourceZ);
        player.SetActorLocation(Vector3(sourceX, player.GetCapsuleComponent().GetHalfHeight(), sourceZ));
        PlayerController* controller = dynamic_cast<PlayerController*>(player.GetController());
        if (controller)
        {
            controller->SetMoveInput(0, 0, false);
            controller->SetInputEnabled(false);
        }
        float projectileMass = scenario == 3 ? 20.0f : 2.0f;
        float projectileSpeed = 7.5f;
        float projectileRadius = RagdollImpactTest::DefaultProjectileRadius;
        float partImpulse = 0;
        float bodyImpulse = 0;
        float torqueMultiplier = 0;
        CommandLineArgs::GetFloat(L"impactmass", projectileMass);
        CommandLineArgs::GetFloat(L"impactspeed", projectileSpeed);
        CommandLineArgs::GetFloat(L"impactradius", projectileRadius);
        CommandLineArgs::GetFloat(L"impactpartimpulse", partImpulse);
        CommandLineArgs::GetFloat(L"impactbodyimpulse", bodyImpulse);
        CommandLineArgs::GetFloat(L"impacttorque", torqueMultiplier);
        check(test.SetProjectileSettings(projectileMass, projectileSpeed, bodyImpulse, torqueMultiplier, partImpulse, projectileRadius),
            "Set explicit projectile test input");
        uint32_t aimBody = 0;
        const bool aimed = CommandLineArgs::GetInteger(L"impactaimbody", aimBody);
        uint32_t physicalContactOnly = 0;
        CommandLineArgs::GetInteger(L"impactcontactonly", physicalContactOnly);
        if (aimed)
        {
            check(aimBody < mesh.GetRagdoll().GetBodyCount(), "Resolve the requested projectile aim body");
            report << "aim_bone=" << mesh.GetPhysicsAsset()->bodies[aimBody].bone << " contact_only=" << physicalContactOnly << std::endl;
        }
        report << "projectile_mass=" << projectileMass << " projectile_speed=" << projectileSpeed << " projectile_radius_m=" << projectileRadius
            << " part_impulse_ns=" << partImpulse << " body_impulse_ns=" << bodyImpulse << " body_torque_multiplier=" << torqueMultiplier
            << " source_x=" << sourceX << " source_z=" << sourceZ << std::endl;
        PhysicsSkeleton skeleton;
        check(mesh.BuildPhysicsSkeleton(skeleton, error), "Read actual dummy skeleton");
        world.SetPhysicsPaused(false);
        world.SetPhysicsTimeScale(1);
        samples << "frame,time,mode,blending,zone,stable_time,impact,step_count,bone,x,y,z,position_error,rotation_error\n";
        feet << "frame,time,mode,blending,side,grounded,root_x,root_y,root_z,sole_x,sole_y,sole_z,target_x,target_y,target_z\n";
        report << "fixed_dt=" << PhysicsWorld::FixedDeltaTime << " stable_duration=2 blend_duration="
            << mesh.GetRecoverySettings().blendDuration << " scenario=" << scenario << std::endl;
        bool seenActive = false;
        bool seenReturn = false;
        bool secondShot = false;
        unsigned entries = 0;
        unsigned returns = 0;
        unsigned footCommands = 0;
        unsigned confirmedTakeoffs = 0;
        unsigned confirmedLandings = 0;
        unsigned validatedDebugFrames = 0;
        bool observedProjectileContact = false;
        std::array<bool, 2> commandObserved{};
        std::array<bool, 2> takeoffObserved{};
        PhysicsAnimationMode previousMode = mesh.GetPhysicsMode();
        for (uint32_t frame = 0; frame < 900; ++frame)
        {
            if ((scenario != 0 && frame == 120) ||
                (scenario == 2 && seenReturn && !secondShot && !mesh.IsBlendingToKinematic()) ||
                (scenario == 4 && seenReturn && !secondShot && mesh.IsBlendingToKinematic()) ||
                (scenario == 5 && seenActive && !secondShot && mesh.GetKinematicStableTime() >= .8f))
            {
                if (aimed)
                {
                    physx::PxGeometryHolder geometry;
                    physx::PxTransform inBone;
                    physx::PxTransform shapePose;
                    check(mesh.GetRagdoll().GetBodyCollisionShape(aimBody, geometry, inBone, shapePose),
                        "Resolve the current target shape");
                    const Vector3 origin = player.GetActorLocation() + Vector3(0, .15f, 0);
                    const Vector3 target(shapePose.p.x, shapePose.p.y, shapePose.p.z);
                    check(test.FireAimed(player, origin, target - origin), "Fire through the normal aim path");
                    if (physicalContactOnly)
                    {
                        // 대조군은 같은 공을 발사하되 관절 반응용 표식만 제거한다.
                        physx::PxShape* shape = nullptr;
                        check(test.GetProjectiles().back()->getShapes(&shape, 1) == 1, "Resolve control projectile shape");
                        physx::PxFilterData filter = shape->getSimulationFilterData();
                        filter.word3 &= ~PhysicsWorld::ProjectileFilterFlag;
                        shape->setSimulationFilterData(filter);
                    }
                }
                else
                {
                    check(test.FireFrom(player), "Fire a real colliding projectile");
                }
                secondShot = seenActive;
            }
            const float beforeTimer = mesh.GetKinematicStableTime();
            const bool beforeFalling = mesh.IsFollowingFallingAnimation();
            const RootSupportRelease beforeRelease = mesh.GetRootSupportRelease();
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
            world.Tick(1.0f / 60.0f);
            test.Update();
            RecordLandingComFrame(landingCom, mesh, frame);
            if (aimed)
            {
                for (const RagdollBodyImpulse& impact : mesh.GetRagdoll().GetExternalImpacts())
                {
                    if (!impact.fromProjectile || impact.impulse.magnitudeSquared() < 1e-8f)
                    {
                        continue;
                    }
                    observedProjectileContact = true;
                    report << "contact_frame=" << frame << " bone=" << mesh.GetPhysicsAsset()->bodies[impact.body].bone
                        << " impulse_ns=" << impact.impulse.magnitude() << '\n';
                }
            }
            if (beforeRelease == RootSupportRelease::None && mesh.GetRootSupportRelease() == RootSupportRelease::ProjectileImpact)
            {
                check(mesh.GetBalanceState() == CharacterBalanceState::GroundFall && !mesh.GetRagdoll().IsRootSupported() &&
                    !mesh.GetRagdoll().UsesFootDrivenSupport(), "Projectile toppling releases support and starts the ground fall in the same tick");
            }
            if (beforeRelease == RootSupportRelease::None && mesh.GetRootSupportRelease() == RootSupportRelease::OutsideBuffer)
            {
                const FootSupportSnapshot& support = mesh.GetFootSupport().GetSnapshot();
                check(support.footstep.actualComOutsideBuffer &&
                        support.footstep.outsideBufferTime + 1e-9 >= FootstepPlanner::OutsideBufferDuration &&
                        (support.footstep.aborted || (mesh.GetGroundFallDirection() == FallDirection::Backward &&
                        support.footstep.backwardFallStepsRemaining > 0)) &&
                        !mesh.GetRagdoll().UsesFootDrivenSupport() && !mesh.GetRagdoll().IsRootSupported() &&
                        mesh.GetBalanceState() == CharacterBalanceState::GroundFall &&
                        (!mesh.IsFallingAnimationEnabled() || mesh.IsFollowingFallingAnimation()),
                    "Continuous actual COM exit from B releases the pelvis and starts the ground fall in the same tick");
            }
            if (validateLandingRegion && (mesh.GetFootSupport().GetSnapshot().hasComProjection ||
                mesh.GetFootSupport().GetSnapshot().footstep.hasGroundPlane))
            {
                check(LandingComDebugMatchesPrediction(mesh), "Rendered region matches the cyan prediction without forbidden-sector penetration");
                ++validatedDebugFrames;
            }
            check(mesh.GetRagdoll().IsFinite() && mesh.GetPhysicsError().empty(), "Finite physical state and no controller error");
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
                    const bool balanceReturn = !beforeFalling;
                    if (balanceReturn)
                    {
                        check(beforeTimer >= 2.0f - 1.0f / 60.0f - .00001f, "Return requires the complete two-second interval");
                    }
                    else
                    {
                        // 낙상 후 기상은 액티브 모드를 유지해도 균형 회복 대기 시간과 다른 전환이다.
                        check(scenario == 3 && mesh.IsFollowingStandingUpAnimation(),
                            "Large-impact fixture permits the existing fall and stand-up transition");
                    }
                    const Vector3 rootOffset = test.GetDummy()->GetActorLocation() - beforeRoot;
                    const float alignmentError = float(Length(rootOffset - expectedRootOffset));
                    report << "return_from_fall=" << !balanceReturn << " return_both_grounded=" << beforeBothGrounded
                        << " root_dx=" << float(rootOffset.GetX()) << " root_dy=" << float(rootOffset.GetY())
                        << " root_dz=" << float(rootOffset.GetZ()) << " sole_alignment_error=" << alignmentError << std::endl;
                    if (validateFootAlignment && balanceReturn)
                    {
                        check(beforeBothGrounded, "Kinematic handoff requires both actual ground contacts");
                        check(alignmentError < .0001f, "Root translation matches both sole midpoints on all three axes");
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
                const FootstepCommand& command = support.footstep.feet[side];
                const FootstepRecoverySnapshot& search = support.footstep.candidates[side].recovery;
                const bool rejected = !command.justStarted && search.status == FootstepRecoveryStatus::PoseRejected;
                const bool crossover = search.hasTarget && search.mode != FootstepMode::Normal;
                if (command.IsInProgress() && support.footstep.requestedFoot == static_cast<int>(side) && (rejected || crossover))
                {
                    const FootSoleDebug overlay;
                    const std::string text = overlay.GetLegPredictionText(mesh);
                    std::string expectedCommand = side == 0 ? u8"왼발 " : u8"오른발 ";
                    expectedCommand += FootstepModeLabel(command.target.recovery.mode);
                    if (rejected)
                    {
                        expectedCommand += u8" (기존 목표 유지)";
                    }
                    const std::string expectedSearch = std::string(u8"\n[착지 탐색] ") +
                        (rejected ? u8"착지 자세 검사 실패" : FootstepModeLabel(search.mode));
                    const size_t searchLine = text.find(expectedSearch);
                    check(text.find(u8"[딛기 명령] ") == 0 && searchLine != std::string::npos &&
                        text.find(expectedCommand) < searchLine, rejected ?
                        "Top overlay keeps the executing mode when landing poses fail" :
                        "Top overlay reports the executing mode when a crossover target is selected");
                }
                if (command.justStarted)
                {
                    ++footCommands;
                    commandObserved[side] = true;
                    takeoffObserved[side] = false;
                }
                if (commandObserved[side] && !takeoffObserved[side] &&
                    (!grounded || (foot.contact.hasSample && !foot.contact.grounded)))
                {
                    takeoffObserved[side] = true;
                    ++confirmedTakeoffs;
                }
                if (command.justLanded)
                {
                    check(takeoffObserved[side], "Completed footstep includes an observed physical takeoff");
                    ++confirmedLandings;
                    commandObserved[side] = false;
                }
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
        if (aimed)
        {
            check(observedProjectileContact == (scenario != 0 && !physicalContactOnly),
                "Aimed shot reports a tagged impact only for the projectile reaction");
        }
        report << "entries=" << entries << " returns=" << returns << std::endl;
        report << "foot_commands=" << footCommands << " physical_takeoffs=" << confirmedTakeoffs
            << " confirmed_landings=" << confirmedLandings << std::endl;
        report << "validated_debug_frames=" << validatedDebugFrames << std::endl;
        if (scenario == 1 && footCommands > 0)
        {
            check(confirmedTakeoffs > 0 && confirmedLandings > 0,
                "A requested recovery step physically lifts and lands the foot");
        }
        check(world.GetPhysicsWorld()->GetErrorCount() == 0, "No PhysX errors");
        check(scenario == 0 ? !seenActive : seenActive, "Impact is required to enter active ragdoll");
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
    std::ofstream landingCom(path + L".landing-com.csv");
    landingCom.precision(9);
    uint32_t kinematicPlayer = 0;
    uint32_t modeCycles = 0;
    uint32_t incoming = 0;
    uint32_t contactCheck = 0;
    CommandLineArgs::GetInteger(L"bodyimpactkinematic", kinematicPlayer);
    CommandLineArgs::GetInteger(L"bodyimpactmodes", modeCycles);
    CommandLineArgs::GetInteger(L"bodyimpactincoming", incoming);
    CommandLineArgs::GetInteger(L"bodyimpactcontactcheck", contactCheck);
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
        check(report.good() && samples.good() && events.good() && landingCom.good() && dummy && scenario <= 10 && frames > 0 && frames <= 720,
            "Open actual-character body impact validation");
        std::string error;
        check(test.Reset(), "Reset grounded Idle dummy with original automatic support");
        const FootSoleDebug overlay;
        check(overlay.GetLegPredictionText(dummy->GetMesh()).find(u8"[추종 애니메이션] Idle (0.00 / ") != std::string::npos,
            "Top overlay shows the idle clip and time before a step");
        ConfigureLandingComValidation(dummy->GetMesh(), report);
        WriteLandingComHeader(landingCom);
        if (contactCheck)
        {
            check(!test.IsContactCheckEnabled(), "Contact pause starts disabled");
            dummy->GetMesh().GetFootSupport().GetFootstepSettings().pathAvoidanceEnabled = true;
            // 검사 전용 복사본에서 허벅지 접촉을 유도하며 저장된 PA 파일은 수정하지 않는다.
            PhysicsAsset contactAsset = *dummy->GetMesh().GetPhysicsAsset();
            for (PhysicsBody& body : contactAsset.bodies)
            {
                if (body.bone.find("UpLeg") != std::string::npos)
                {
                    body.radius = .2f;
                }
            }
            check(dummy->GetMesh().SetPhysicsAsset(contactAsset, error), "Prepare a repeatable physical thigh-contact fixture");
            test.SetContactCheckEnabled(true);
        }
        PlayerController* controller = dynamic_cast<PlayerController*>(player.GetController());
        check(controller != nullptr, "Use the normal player movement controller");
        controller->SetInputEnabled(true);
        controller->SetMoveInput(0, 0, false);
        check(player.StopRagdoll(error), "Reset player simulation");
        player.GetCharacterMovement().StopMovementImmediately();
        const bool checkWarmup = scenario == 9;
        const bool releaseOnContact = scenario == 10;
        const bool continuousFrontPush = scenario == 7 || scenario == 8 || checkWarmup || releaseOnContact;
        float yaw = scenario == 3 || scenario == 5 ? DirectX::XM_PIDIV2 :
            scenario == 4 ? -DirectX::XM_PIDIV2 : scenario == 6 ? DirectX::XM_PIDIV4 : 0.0f;
        float lateralOffset = 0.0f;
        if (continuousFrontPush)
        {
            // 일반 전진 입력으로 더미의 정면에서 밀어 뒷걸음이 발생하는 상황을 재현한다.
            const Vector3 facing = dummy->GetMesh().GetComponentTransform().GetBasis() * Vector3(kZUnitVector);
            yaw = std::atan2(-float(facing.GetX()), -float(facing.GetZ()));
            CommandLineArgs::GetFloat(L"bodyimpactoffset", lateralOffset);
        }
        const Vector3 direction(std::sin(yaw), 0, std::cos(yaw));
        const Vector3 right(std::cos(yaw), 0, -std::sin(yaw));
        const Vector3 origin = dummy->GetActorLocation() - direction * 2.0f + right * lateralOffset;
        player.SetActorTransform(AffineTransform(Quaternion(Vector3(kYUnitVector), yaw),
            Vector3(origin.GetX(), player.GetCapsuleComponent().GetHalfHeight(), origin.GetZ())));
        controller->SetControlYaw(yaw);
        PreviewAnimInstance& animation = static_cast<PreviewAnimInstance&>(player.GetMesh().GetAnimInstance());
        animation.SetAutoCycleEnabled(false);
        animation.SetLocomotionEnabled(true);
        animation.PlayAnimation(0);
        if (checkWarmup)
        {
            AnimInstance& standing = dummy->GetMesh().GetAnimInstance();
            const uint32_t clip = static_cast<uint32_t>(standing.GetAnimationCount());
            check(standing.AddAnimation("QuietStanding", dummy->GetMesh().GetSkeletalMesh()) && standing.PlayAnimation(clip),
                "Start the receiver in a stationary pose with a different clip name");
        }
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
        std::wstring impactPreset;
        if (CommandLineArgs::GetString(L"bodyimpactsettings", impactPreset))
        {
            // 검증용 복사본에서 실제 UI의 저장과 재로딩을 거친 값으로 충돌을 재현한다.
            const std::wstring saved = path + L".impact-settings.json";
            const int length = static_cast<int>(saved.size());
            const int bytes = WideCharToMultiByte(CP_UTF8, 0, saved.data(), length, nullptr, 0, nullptr, nullptr);
            check(bytes > 0, "Encode the validation preset path");
            std::string settingsPath(bytes, '\0');
            WideCharToMultiByte(CP_UTF8, 0, saved.data(), length, &settingsPath[0], bytes, nullptr, nullptr);
            check(CopyFileW(impactPreset.c_str(), saved.c_str(), FALSE) && test.LoadImpactSettings(player, settingsPath),
                "Load the supplied impact preset");
            CharacterImpactComponent& source = player.GetCharacterImpact();
            const ImpactImpulseSettings expected = source.GetImpactImpulseSettings();
            check(test.SaveImpactSettings(player) && source.SetImpactImpulseSettings({}) && test.LoadImpactSettings(player, settingsPath),
                "Round-trip all impact presets through the same settings file as the editor");
            const ImpactImpulseSettings& actual = source.GetImpactImpulseSettings();
            check(actual.partImpulse == expected.partImpulse && actual.bodyImpulse == expected.bodyImpulse &&
                actual.torqueMultiplier == expected.torqueMultiplier, "Restore capsule impulse settings on the player");
            report << "part_impulse_ns=" << actual.partImpulse << " body_impulse_ns=" << actual.bodyImpulse
                << " torque_multiplier=" << actual.torqueMultiplier << std::endl;
        }
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
        states << "frame,mode,blending,zone,stable_time,step_count,impact,physics_step,requested_foot,load_left,load_right,confirm_left,confirm_right,impulse_left,impulse_right,normal_y_left,normal_y_right,grounded_left,grounded_right,started_left,started_right,landed_left,landed_right,avoidance,prediction,candidates,intervals,queries,prediction_ms,tracking_error,reach_limited,joint_limited,fixed_pose_blocked,warmup,warmup_landings,com_vx,com_vz\n";
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
        uint32_t warmupTicks = 0;
        uint32_t warmupLandings = 0;
        uint32_t impactFrame = UINT32_MAX;
        Vector3 impactOrigin(kZero);
        std::array<ImpactWarmupStep, 2> initialSteps;
        ImpactWarmupStep secondStep;
        uint32_t firstLandingTicks = 0;
        const int handBody = dummy->GetMesh().GetPhysicsAsset()->FindBody("mixamorig1:LeftHand");
        const int chestBody = dummy->GetMesh().GetPhysicsAsset()->FindBody("mixamorig1:Spine2");
        const bool centeredWarmup = checkWarmup && kinematicPlayer && lateralOffset == 0 &&
            std::abs(impactSettings.collisionMass - 20.0f) < .001f;
        if (checkWarmup)
        {
            check(handBody >= 0 && chestBody >= 0, "Locate the hand and chest for the upper-body response measurement");
        }
        Vector3 firstHandOffset(kZero);
        float earlyPushDistance = 0;
        float halfSecondPushDistance = 0;
        float firstStepHandTravel = 0;
        float previousUpperBodyTime = 0;
        float peakChestDeflection = 0;
        bool chestYieldsDuringWarmup = true;
        float startPelvisY = 0;
        std::array<float, 2> startSoleY{};
        std::array<int, 2> swingFeet{-1, -1};
        Vector3 lastRoot(kZero);
        PxVec3 slideVelocity(0);
        bool handedOff = false;
        bool physicalStep = false;
        bool expectSecondStepContact = kinematicPlayer != 0;
        bool contactDuringSecondStep = false;
        bool contactAfterHandoff = false;
        bool pushedLastStep = false;
        uint32_t coastingTicks = 0;
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
            float input = scenario == 0 || frame < 120 || frame >= 540 ? 0.0f :
                frame < 300 ? 1.0f : frame < 420 ? -1.0f : 1.0f;
            if (continuousFrontPush)
            {
                input = frame >= 120 && frame < 540 && !(releaseOnContact && eventCount > 0) ? 1.0f : 0.0f;
            }
            controller->SetMoveInput(input, 0, scenario != 1 && scenario != 7);
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
            RecordLandingComFrame(landingCom, dummy->GetMesh(), frame);
            if (contactCheck && world.IsPhysicsPaused())
            {
                FootSoleDebug debug;
                std::string contactMessage;
                const uint32_t contacts = debug.GetLegContacts(&dummy->GetMesh(), contactMessage);
                check(contacts != 0 && !contactMessage.empty(), "Actual dummy leg contact triggers the pause");
                const FootstepCollisionSnapshot prediction = dummy->GetMesh().GetFootstepCollisionSnapshot();
                const FootstepSnapshot& step = dummy->GetMesh().GetFootSupport().GetSnapshot().footstep;
                check(prediction.status == FootstepCollisionStatus::Collision && prediction.sampleCount == 6 &&
                    prediction.queryCount >= 63 && prediction.completedStep == step.completedStep &&
                    prediction.revision == step.revision,
                    "The real dummy checks its runtime PA shapes on the current control tick");
                check(prediction.overlapPairs != 0, "The enlarged-thigh fixture also overlaps on the predicted path");
                report << "prediction_samples=" << prediction.sampleCount << " prediction_queries=" << prediction.queryCount
                    << " prediction_pairs=" << prediction.overlapPairs << " prediction_ms=" << prediction.elapsedMilliseconds
                    << std::endl;
                const uint64_t pausedStep = world.GetPhysicsWorld()->GetStepCount();
                const Vector3 pausedPosition = dummy->GetMesh().GetRagdoll().GetPosition();
                for (uint32_t pausedFrame = 0; pausedFrame < 60; ++pausedFrame)
                {
                    world.Tick(1.0f / 60.0f);
                    test.Update();
                }
                check(world.IsPhysicsPaused() && world.GetPhysicsWorld()->GetStepCount() == pausedStep &&
                    float(Length(dummy->GetMesh().GetRagdoll().GetPosition() - pausedPosition)) < .00001f,
                    "Contact pause preserves the physics step and dummy pose");
                std::string retainedMessage;
                check(debug.GetLegContacts(&dummy->GetMesh(), retainedMessage) == contacts && retainedMessage == contactMessage,
                    "The contact description remains available while paused");
                const FootstepCollisionSnapshot& retainedPrediction = dummy->GetMesh().GetFootstepCollisionSnapshot();
                check(retainedPrediction.completedStep == prediction.completedStep &&
                    retainedPrediction.overlapPairs == prediction.overlapPairs &&
                    debug.GetLegPredictionText(dummy->GetMesh()).find(u8"겹침 예상") != std::string::npos,
                    "The separate predicted-contact description remains inspectable while paused");
                test.ResumeSimulation();
                test.Update();
                check(!world.IsPhysicsPaused(), "Resume does not immediately pause on the same contact");
                test.SetContactCheckEnabled(false);
                test.Update();
                check(!world.IsPhysicsPaused(), "Disabling contact check allows an existing contact");
                test.SetContactCheckEnabled(true);
                test.Update();
                check(world.IsPhysicsPaused(), "Enabling contact check rearms contact detection");
                check(world.GetPhysicsWorld()->GetErrorCount() == 0, "Contact pause introduces no PhysX errors");
                // 캡처 옵션에서는 마지막으로 멈춘 실제 접촉 장면을 표시한다.
                controller->SetMoveInput(0, 0, false);
                report << "PASS: dummy contact pause and resume" << std::endl;
                return true;
            }
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
            const bool swingingSecondFoot = dummyMesh.IsImpactWarmupActive() && warmupLandings == 1;
            if ((checkWarmup || releaseOnContact) && dummyMesh.IsImpactWarmupActive())
            {
                const std::array<ImpactWarmupStep, 2>& steps = dummyMesh.GetImpactWarmupSteps();
                if (warmupTicks == 0)
                {
                    initialSteps = steps;
                    check(steps[0].duration > 0 && steps[1].duration > 0, "Both warmup landings are planned at impact");
                }
                check(steps[0].duration == initialSteps[0].duration && steps[0].stride == initialSteps[0].stride,
                    "The launched first step retains its landing and cadence");
                if (warmupLandings == 0)
                {
                    check(steps[1].duration == initialSteps[1].duration && steps[1].stride == initialSteps[1].stride,
                        "The second landing forecast is held until the first foot lands");
                    if (dummyMesh.GetImpactWarmupLandings() == 1)
                    {
                        firstLandingTicks = warmupTicks + 1;
                    }
                }
                else if (secondStep.duration == 0)
                {
                    secondStep = steps[1];
                }
                else
                {
                    check(steps[1].duration == secondStep.duration && steps[1].stride == secondStep.stride,
                        "The second target is fixed during its swing");
                }
            }
            if (checkWarmup && (dummyMesh.IsImpactWarmupActive() || (warmupTicks > 0 && !physicalStep)))
            {
                // 관통한 캡슐도 입구에서 한 번은 충격을 기록하므로 두 몸통의 순서까지 확인한다.
                const Ragdoll& receiver = dummyMesh.GetRagdoll();
                const Vector3 receiverPelvis(receiver.GetBoneWorld(receiver.GetImpactRootBody()).GetW());
                const Vector3 playerPelvis(playerRagdoll.GetBoneWorld(playerRagdoll.GetImpactRootBody()).GetW());
                check(float(Dot(receiverPelvis - playerPelvis, direction)) > radius * .5f,
                    "The running player stays on the approach side of the dummy through the handoff");
            }
            if (checkWarmup && dummyMesh.IsImpactWarmupActive())
            {
                const Ragdoll& body = dummyMesh.GetRagdoll();
                chestYieldsDuringWarmup = chestYieldsDuringWarmup && body.IsWarmupDynamicBody(chestBody) &&
                    body.GetImpactReaction().spineDriveScale < 1.0f;
                const Vector3 pelvis(body.GetBoneWorld(body.GetImpactRootBody()).GetW());
                const float pelvisY = pelvis.GetY();
                const Vector3 root = dummy->GetActorLocation();
                const Vector3 handOffset = Vector3(body.GetBoneWorld(handBody).GetW()) - pelvis;
                if (centeredWarmup && warmupTicks * PhysicsWorld::FixedDeltaTime >= .1)
                {
                    // 블렌딩이 끝난 같은 시각의 원본 상체와 비교해 골반 이동과 조깅 애니메이션을 측정에서 제외한다.
                    std::vector<GraphNode> authored;
                    std::vector<Matrix4> authoredWorld;
                    check(dummyMesh.GetAnimInstance().SampleUpperBodyPose(dummyMesh.GetImpactFlailClip(),
                        previousUpperBodyTime, 1.0f, authored, &dummyMesh.GetFinalPose()) &&
                        EvaluatePhysicsPose(dummySkeleton, authored, Matrix4(dummyMesh.GetComponentTransform()), authoredWorld, error),
                        "Sample the matching upper-body frame with the same pelvis");
                    const int chestNode = body.GetBoneNodes()[chestBody];
                    const Vector3 chest(body.GetBoneWorld(chestBody).GetW());
                    const float deflection = Dot(chest - Vector3(authoredWorld[chestNode].GetW()), direction);
                    peakChestDeflection = Max(peakChestDeflection, std::abs(deflection));
                }
                previousUpperBodyTime = dummyMesh.GetImpactFlailTime();
                if (warmupTicks == 0)
                {
                    startPelvisY = pelvisY;
                    firstHandOffset = handOffset;
                    for (size_t side = 0; side < startSoleY.size(); ++side)
                    {
                        startSoleY[side] = dummySupport.feet[side].soleWorld.GetW().GetY();
                    }
                }
                else
                {
                    const Vector3 speed = (root - lastRoot) * 60.0f;
                    slideVelocity = PxVec3(speed.GetX(), 0, speed.GetZ());
                }
                const uint32_t landings = dummyMesh.GetImpactWarmupLandings();
                // 물리 플레이어가 튕겨 나간 동안은 짧은 두 번째 스윙 전체가 재접촉 전에 끝날 수 있다.
                if (swingingSecondFoot && !expectSecondStepContact)
                {
                    const PxCapsuleGeometry capsule(radius, halfLength);
                    for (size_t index = 0; index < body.GetBodyCount(); ++index)
                    {
                        if (!body.IsCharacterImpactBody(index))
                        {
                            continue;
                        }
                        PxGeometryHolder geometry;
                        PxTransform inBone(PxIdentity);
                        PxTransform actualPose(PxIdentity);
                        check(body.GetBodyCollisionShape(index, geometry, inBone, actualPose), "Read the simulated torso collider");
                        if (PxGeometryQuery::overlap(capsule, capsulePose, geometry.any(), actualPose))
                        {
                            expectSecondStepContact = true;
                            break;
                        }
                    }
                }
                if (landings == 0)
                {
                    firstStepHandTravel = Max(firstStepHandTravel, float(Length(handOffset - firstHandOffset)));
                }
                check(!dummyMesh.IsRagdoll() && dummySupport.footstep.startCount == 0 && std::abs(pelvisY - startPelvisY) < .002f,
                    "Pelvis stays level while only the visual steps run");
                for (size_t side = 0; side < startSoleY.size() && landings < swingFeet.size(); ++side)
                {
                    if (float(dummySupport.feet[side].soleWorld.GetW().GetY()) > startSoleY[side] + .035f)
                    {
                        swingFeet[landings] = static_cast<int>(side);
                    }
                }
                warmupLandings = landings;
                lastRoot = root;
                ++warmupTicks;
            }
            else if (checkWarmup && warmupTicks > 0 && !physicalStep)
            {
                if (!handedOff)
                {
                    const PxVec3 velocity(dummySupport.centerOfMassVelocity.x, 0, dummySupport.centerOfMassVelocity.z);
                    check(dummyMesh.IsActiveRagdoll() && warmupLandings == 2 && swingFeet[0] >= 0 &&
                        swingFeet[1] == 1 - swingFeet[0], "Two opposite feet swing and land before the physical handoff");
                    check(slideVelocity.magnitude() > .25f && (velocity - slideVelocity).magnitude() < .025f,
                        "The active body inherits the last horizontal slide velocity");
                    handedOff = true;
                }
                if (dummySupport.footstep.startCount > 0)
                {
                    check(dummySupport.footstep.feet[swingFeet[0]].justStarted,
                        "The first physical step uses the foot opposite the last warmup landing");
                    physicalStep = true;
                }
            }
            if (releaseOnContact && dummyMesh.IsImpactWarmupActive())
            {
                const Vector3 root = dummy->GetActorLocation();
                const Vector3 speed = (root - lastRoot) * 60.0f;
                const PxVec3 planar(speed.GetX(), 0, speed.GetZ());
                if (warmupTicks >= 2 && !pushedLastStep && slideVelocity.magnitude() > .05f)
                {
                    check(planar.magnitude() <= slideVelocity.magnitude() + .003f,
                        "A separated dummy coasts without gaining speed from the original impact again");
                    ++coastingTicks;
                }
                lastRoot = root;
                slideVelocity = planar;
                warmupLandings = dummyMesh.GetImpactWarmupLandings();
                ++warmupTicks;
            }
            const FootstepAvoidanceSnapshot& avoidance = dummyMesh.GetFootstepAvoidanceSnapshot();
            const FootstepCollisionSnapshot& prediction = dummyMesh.GetFootstepCollisionSnapshot();
            states << frame << ',' << static_cast<int>(dummyMesh.GetPhysicsMode()) << ',' << dummyMesh.IsBlendingToKinematic()
                << ',' << static_cast<int>(dummySupport.balance.zone) << ',' << dummyMesh.GetKinematicStableTime()
                << ',' << dummySupport.footstep.startCount << ',' << dummy->GetMesh().GetRagdoll().GetExternalImpactImpulse()
                << ',' << dummySupport.completedPhysicsStep << ',' << dummySupport.footstep.requestedFoot
                << ',' << dummySupport.footstep.loadTransfer.loadRatios[0] << ',' << dummySupport.footstep.loadTransfer.loadRatios[1]
                << ',' << dummySupport.footstep.loadTransfer.confirmationTimes[0] << ',' << dummySupport.footstep.loadTransfer.confirmationTimes[1]
                << ',' << dummySupport.feet[0].contact.normalImpulse << ',' << dummySupport.feet[1].contact.normalImpulse
                << ',' << dummySupport.feet[0].contact.normal.y << ',' << dummySupport.feet[1].contact.normal.y
                << ',' << dummySupport.feet[0].contact.grounded << ',' << dummySupport.feet[1].contact.grounded
                << ',' << dummySupport.footstep.feet[0].justStarted << ',' << dummySupport.footstep.feet[1].justStarted
                << ',' << dummySupport.footstep.feet[0].justLanded << ',' << dummySupport.footstep.feet[1].justLanded
                << ',' << static_cast<int>(avoidance.status) << ',' << static_cast<int>(prediction.status)
                << ',' << avoidance.candidatesChecked << ',' << avoidance.totalIntervals << ',' << avoidance.totalQueries
                << ',' << prediction.elapsedMilliseconds << ',' << prediction.maximumTrackingError
                << ',' << prediction.reachLimited << ',' << prediction.jointLimited << ',' << prediction.fixedPoseBlocked
                << ',' << dummyMesh.IsImpactWarmupActive() << ',' << dummyMesh.GetImpactWarmupLandings()
                << ',' << dummySupport.centerOfMassVelocity.x << ',' << dummySupport.centerOfMassVelocity.z << '\n';
            pushedLastStep = false;
            for (const CharacterImpactSnapshot& impact : player.GetCharacterImpact().GetAppliedImpacts())
            {
                if (impact.target == dummy->GetHandle() && impact.normalImpulse > 0)
                {
                    if (impactFrame == UINT32_MAX)
                    {
                        impactFrame = frame;
                    }
                    pushedLastStep = true;
                    contactDuringSecondStep = contactDuringSecondStep || swingingSecondFoot;
                    contactAfterHandoff = contactAfterHandoff || handedOff;
                }
                ++eventCount;
                for (const BodyImpactShare& share : impact.shares)
                {
                    events << frame << ',' << time << ',' << impact.eventCount << ',' << impact.hitBone << ',' << share.bone
                        << ',' << 0 << ',' << impact.normalImpulse << ',' << impact.closingSpeed
                        << ',' << impact.point.x << ',' << impact.point.y << ',' << impact.point.z
                        << ',' << share.impulse.x << ',' << share.impulse.y << ',' << share.impulse.z << ',' << 0 << '\n';
                }
            }
            if (checkWarmup)
            {
                // 보폭에 따라 예열이 먼저 끝날 수 있으므로 실제 접촉 틱과 골반으로 같은 0.1초·0.5초를 잰다.
                const Ragdoll& receiver = dummyMesh.GetRagdoll();
                const Vector3 pelvis(receiver.GetBoneWorld(receiver.GetImpactRootBody()).GetW());
                if (impactFrame == UINT32_MAX)
                {
                    impactOrigin = pelvis;
                }
                else if (frame - impactFrame == 5)
                {
                    earlyPushDistance = Dot(pelvis - impactOrigin, direction);
                }
                else if (frame - impactFrame == 29)
                {
                    halfSecondPushDistance = Dot(pelvis - impactOrigin, direction);
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
        if (contactCheck)
        {
            check(false, "Expected an actual leg contact before the frame limit");
        }
        report << "events=" << eventCount << std::endl;
        if (checkWarmup || releaseOnContact)
        {
            const float dt = static_cast<float>(PhysicsWorld::FixedDeltaTime);
            check(firstLandingTicks > 0 && secondStep.duration > 0 &&
                std::abs(dt * firstLandingTicks - initialSteps[0].duration) < .001f &&
                std::abs(dt * (warmupTicks - firstLandingTicks) - secondStep.duration) < .001f,
                "Each foot lands on its planned physics tick");
            if (lateralOffset == 0 && std::abs(impactSettings.collisionMass - 20.0f) < .001f)
            {
                check((secondStep.stride - initialSteps[1].stride).magnitude() > .005f ||
                    std::abs(secondStep.duration - initialSteps[1].duration) >= .9f * dt,
                    "The real motion at the first landing changes the second step from the impact-time forecast");
            }
            report << "first_step_s=" << initialSteps[0].duration << " first_stride_m=" << initialSteps[0].stride.magnitude()
                << " second_forecast_s=" << initialSteps[1].duration << " second_forecast_stride_m=" << initialSteps[1].stride.magnitude()
                << " second_step_s=" << secondStep.duration << " second_stride_m=" << secondStep.stride.magnitude()
                << " second_swing_travel_m=" << (initialSteps[0].stride + secondStep.stride).magnitude() << std::endl;
        }
        if (checkWarmup)
        {
            report << "push_0.1s_m=" << earlyPushDistance << " push_0.5s_m=" << halfSecondPushDistance
                << " first_step_hand_travel_m=" << firstStepHandTravel << " peak_chest_deflection_m=" << peakChestDeflection << std::endl;
            check(firstStepHandTravel > .05f, "The upper body reacts relative to the pelvis during the first step");
            if (centeredWarmup)
            {
                check(chestYieldsDuringWarmup && peakChestDeflection > .005f,
                    "Chest moves relative to the animation with reduced physical drives throughout the warmup steps");
                // 기본 20 kg 플레이어로 측정한 예열 전 골반 변위는 각각 0.20 m, 1.65 m다.
                check(earlyPushDistance >= .15f && earlyPushDistance <= .27f &&
                    halfSecondPushDistance >= 1.40f && halfSecondPushDistance <= 1.90f,
                    "The animated player's running shove preserves the original onset and half-second travel");
            }
            check(handedOff && physicalStep, "A running player produces two warmup steps followed by a physical step");
            check(!expectSecondStepContact || contactDuringSecondStep,
                "The animated shove or a simulated capsule overlapping the torso produces contact during the second step");
            check(contactAfterHandoff, "Continued input produces physical capsule contact after the handoff");
        }
        if (releaseOnContact)
        {
            check(eventCount > 0 && player.GetCharacterMovement().GetLocomotionSpeed() < .01f &&
                warmupLandings == 2 && coastingTicks > 0,
                "Releasing input after contact lets the dummy finish two warmup steps under the remaining momentum");
        }
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
        uint32_t woundBody = 0;
        if (CommandLineArgs::GetInteger(L"woundholdbone", woundBody))
        {
            std::string error;
            check(test.SetAutomaticSupport(true) && player.StopRagdoll(error), "Prepare the idle impact dummy");
            SkeletalMeshComponent& mesh = test.GetDummy()->GetMesh();
            check(woundBody < mesh.GetRagdoll().GetBodyCount(), "Resolve the upper-body hit shape");
            player.GetCharacterMovement().SetTickEnabled(false);
            test.SetContactCheckEnabled(false);
            world.SetPhysicsPaused(false);
            world.SetPhysicsTimeScale(1.0f);
            for (unsigned frame = 0; frame < 30; ++frame)
            {
                world.Tick(static_cast<float>(PhysicsWorld::FixedDeltaTime));
                test.Update();
            }
            PxGeometryHolder geometry;
            PxTransform inBone;
            PxTransform shape;
            check(mesh.GetRagdoll().GetBodyCollisionShape(woundBody, geometry, inBone, shape), "Measure the physical target");
            const PhysicsAsset& asset = *mesh.GetPhysicsAsset();
            const std::string& bone = asset.bodies[woundBody].bone;
            const int joint = mesh.GetRagdoll().FindWoundJoint(woundBody);
            check(joint >= 0, "Resolve the local wound joint");
            const int left = asset.FindBody("mixamorig1:LeftArm");
            const int right = asset.FindBody("mixamorig1:RightArm");
            check(left >= 0 && right >= 0, "Find the dummy's shoulders");
            Vector3 across = Vector3(mesh.GetRagdoll().GetBoneWorld(right).GetW()) - Vector3(mesh.GetRagdoll().GetBoneWorld(left).GetW());
            across = Normalize(Vector3(across.GetX(), 0, across.GetZ()));
            const Vector3 front = Cross(Vector3(kYUnitVector), across);
            const float side = bone.find("Right") != std::string::npos ? .6f : bone.find("Left") != std::string::npos ? -.6f : 0;
            const Vector3 target(shape.p.x, shape.p.y, shape.p.z);
            const Vector3 eye = target + (front + across * side) * 2.0f;
            player.SetActorLocation(eye - Vector3(0, .15f, 0));
            uint32_t arrow = 0;
            CommandLineArgs::GetInteger(L"woundarrow", arrow);
            ImpactProjectileSettings projectile = ImpactProjectileSettings::ArrowDefaults();
            projectile.radius = arrow ? .01f : .018f;
            projectile.bodyImpulse = 0;
            projectile.partImpulse = 0;
            projectile.torqueMultiplier = 0;
            const ImpactProjectileType type = arrow ? ImpactProjectileType::Arrow : ImpactProjectileType::Sphere;
            check(test.SetProjectileSettings(type, projectile) && test.FireAimed(player, eye, target - eye),
                "Fire a projectile through the normal aim path");
            unsigned longest = 0;
            unsigned consecutive = 0;
            int lastHit = -1;
            float nearest = PX_MAX_F32;
            float gripAge = -1;
            float fiveSecondMaximum = 0;
            unsigned heldSteps = 0;
            bool missingContact = false;
            bool weakened = true;
            for (unsigned frame = 0; frame < 690; ++frame)
            {
                world.Tick(static_cast<float>(PhysicsWorld::FixedDeltaTime));
                test.Update();
                int hit = -1;
                float distance = 0;
                const bool sampled = mesh.GetImpactPalmDistance(hit, distance);
                if (sampled)
                {
                    lastHit = hit;
                    nearest = PxMin(nearest, distance);
                }
                if (sampled && hit == static_cast<int>(woundBody) && distance <= .05f)
                {
                    ++consecutive;
                    longest = (std::max)(longest, consecutive);
                }
                else
                {
                    consecutive = 0;
                }
                if (gripAge < 0)
                {
                    gripAge = mesh.GetRagdoll().GetWoundGripAge(joint);
                }
                if (gripAge >= 0 && heldSteps < 300)
                {
                    missingContact = missingContact || !sampled || hit != static_cast<int>(woundBody);
                    weakened = weakened && mesh.GetRagdoll().GetWoundStrength(joint) <= .101f;
                    fiveSecondMaximum = PxMax(fiveSecondMaximum, distance);
                    ++heldSteps;
                }
            }
            report << "aim=" << bone << " hit=" << lastHit << " closest_m=" << nearest
                << " grip_age_s=" << gripAge << " continuous_steps=" << longest
                << " first_five_seconds_max_m=" << fiveSecondMaximum << std::endl;
            check(heldSteps == 300 && !missingContact && fiveSecondMaximum <= .05f && longest >= 300,
                "Actual palm stays at the moving wound for five seconds from physical contact");
            check(weakened && !mesh.GetRagdoll().HasWound(), "Only release and recover after the full hold");
            check(mesh.GetPhysicsError().empty(), "Wound hold finishes without controller errors");
            return true;
        }
        // 저장된 사용자 설정과 관계없이 기존 충격 시험의 초기 조건을 유지한다.
        check(test.SetProjectileSettings(2.0f, 7.5f), "Use the original 2 kg / 7.5 m/s fixture independently of saved UI settings");
        check(test.SetAutomaticSupport(false), "Use manual support for the existing projection and projectile baseline");
        PARS::Character* dummy = test.GetDummy();
        check(dummy && dummy != &player && dummy->GetController() == nullptr,
            "Idle dummy is independent of the possessed player");
        const PARS::AnimInstance& dummyAnimation = dummy->GetMesh().GetAnimInstance();
        const PARS::AnimInstance& playerAnimation = player.GetMesh().GetAnimInstance();
        const std::vector<PARS::MotionFrame>& candidates = dummyAnimation.GetMotionFrames();
        check(&candidates == &playerAnimation.GetMotionFrames(),
            "Player and impact dummy share the initialized motion frames");
        CheckMotionFrameRanges(dummyAnimation, kWalkingClips, check);
        CheckImpactMotionClips(dummy->GetMesh(), check);

        PARS::AnimInstance mixedClips(player.GetMesh());
        mixedClips.Initialize(player.GetMesh().GetSkeletalMesh());
        const PARS::MotionFrameRange walkingRange = dummyAnimation.GetMotionFrameRange(PARS::MotionFrameUsage::Walking);
        const uint32_t idleClip = candidates.front().clipIndex;
        const uint32_t forwardClip = candidates[walkingRange.firstFrameIndex].clipIndex;
        const uint32_t backwardClip = candidates[walkingRange.firstFrameIndex + kWalkingClips[0].frameCount].clipIndex;
        CheckWalkingTravelDirection(dummyAnimation, candidates[walkingRange.firstFrameIndex + kWalkingClips[0].frameCount + 30], check);
        const float fixedStep = PhysicsWorld::FixedDeltaTime;
        const float duration = dummyAnimation.GetAnimationDuration(forwardClip);
        const PARS::MotionMatchResult wrapped{forwardClip, duration - fixedStep, 0.0f};
        check(std::abs(dummyAnimation.GetWalkingPlaybackTime(wrapped, 0) - wrapped.clipTimeSeconds) < 1e-6f &&
                std::abs(dummyAnimation.GetWalkingPlaybackTime(wrapped, duration + 2.0f * fixedStep) - fixedStep) < 1e-6f,
            "A walking match keeps its start time and can advance across multiple loop boundaries");
        const PARS::MotionFeatures& query = candidates.front().features;
        for (uint32_t side = 0; side < 2; ++side)
        {
            for (PARS::FootstepMode mode : {PARS::FootstepMode::Normal, PARS::FootstepMode::FrontCross, PARS::FootstepMode::BackCross})
            {
                const PARS::MotionMatchResult match = dummyAnimation.FindMotionMatch(PARS::MotionFrameUsage::Walking, query, {side, mode});
                const std::vector<PARS::MotionFrame>::const_iterator found = std::find_if(candidates.begin(), candidates.end(),
                    [&match](const PARS::MotionFrame& frame) -> bool
                    {
                        return frame.clipIndex == match.clipIndex && frame.clipTimeSeconds == match.clipTimeSeconds;
                    });
                check(found != candidates.end() && found->usage == PARS::MotionFrameUsage::Walking &&
                        found->swing.movingFoot == side && found->swing.mode == mode,
                    "Walking can match either foot for an ordinary, front-cross or back-cross step from the same query");
            }
        }
        PARS::MotionFeatures invalidQuery = query;
        invalidQuery.footVelocities[1].y = std::numeric_limits<float>::quiet_NaN();
        check(dummyAnimation.FindMotionMatch(PARS::MotionFrameUsage::Walking, invalidQuery).clipIndex == UINT32_MAX,
            "A non-finite motion query cannot select a clip");
        std::string motionError;
        check(mixedClips.ShareMotionFramesFrom(playerAnimation, motionError) &&
                mixedClips.BuildMotionFrames({{backwardClip, PARS::MotionFrameUsage::Walking}, {idleClip, PARS::MotionFrameUsage::Idle},
                    {forwardClip, PARS::MotionFrameUsage::Walking}}, motionError),
            "Build interleaved usages on a separate animation instance");
        CheckMotionFrameRanges(mixedClips, {kWalkingClips[1], kWalkingClips[0]}, check);
        check(!mixedClips.BuildMotionFrames({{idleClip, PARS::MotionFrameUsage::Count}}, motionError) &&
                mixedClips.GetMotionFrameRange(PARS::MotionFrameUsage::Walking).frameCount ==
                    kWalkingClips[0].frameCount + kWalkingClips[1].frameCount,
            "An invalid usage leaves the published range intact");
        check(mixedClips.BuildMotionFrames({{idleClip, PARS::MotionFrameUsage::Idle}}, motionError),
            "Rebuild the index with only Idle candidates");
        const PARS::MotionFrameRange emptyRange = mixedClips.GetMotionFrameRange(PARS::MotionFrameUsage::Walking);
        check(emptyRange.firstFrameIndex == kIdleClip.frameCount && emptyRange.frameCount == 0,
            "An unused Walking range is empty at the end of the candidate array");
        check(mixedClips.FindMotionMatch(PARS::MotionFrameUsage::Walking, query).clipIndex == UINT32_MAX,
            "An empty Walking group produces no match");
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
        check(dummyAnimation.GetAnimationName(dummyAnimation.GetCurrentAnimationIndex()) == "Idle",
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
        PxRaycastHit wallSight;
        check(physics->Raycast(toPx(camera), toPx(aim) * 100, wallSight, &player) && wallSight.actor == aimTarget.get(),
            "Crosshair sees the front surface of the wall fixture");
        check(test.SetProjectileSettings(.5f, 7.5f, 0, 0, 0, .08f) && test.FireAimed(player, camera, aim),
            "Fire an aimed sphere with an 8 cm radius");
        const PxRigidDynamic* smallBall = test.GetProjectiles().back();
        const PxVec3 expectedDirection = (wallSight.position + wallSight.normal * .08f - toPx(shotOrigin)).getNormalized();
        check((smallBall->getLinearVelocity().getNormalized() - expectedDirection).magnitude() < 1e-4f,
            "The camera target offsets the sphere center by its selected radius");
        check(test.SetProjectileSettings(.5f, 7.5f, 0, 0, 0, .30f) && test.FireFrom(player),
            "Fire a 30 cm sphere while the smaller shot is still in flight");
        const PxRigidDynamic* largeBall = test.GetProjectiles().back();
        PxShape* smallShape = nullptr;
        PxShape* largeShape = nullptr;
        check(smallBall->getShapes(&smallShape, 1) == 1 && largeBall->getShapes(&largeShape, 1) == 1 &&
                smallShape->getGeometry().getType() == PxGeometryType::eSPHERE && largeShape->getGeometry().getType() == PxGeometryType::eSPHERE &&
                std::abs(static_cast<const PxSphereGeometry&>(smallShape->getGeometry()).radius - .08f) < 1e-5f &&
                std::abs(static_cast<const PxSphereGeometry&>(largeShape->getGeometry()).radius - .30f) < 1e-5f &&
                std::abs(largeBall->getMass() - .5f) < 1e-5f,
            "Live shots keep their individual collision radii and the selected mass");
        test.ClearProjectiles();
        bool wallAccurate = true;
        bool gravityRestored = true;
        for (const float speed : {7.5f, 100.0f})
        {
            check(test.SetProjectileSettings(.5f, speed) && test.FireAimed(player, camera, aim), "Fire at the visible wall surface");
            const PxRigidDynamic* ball = test.GetProjectiles().back();
            float error = std::numeric_limits<float>::infinity();
            for (int frame = 0; frame < 120 && !std::isfinite(error); ++frame)
            {
                physics->Step();
                std::vector<PhysicsContactImpulse> contacts;
                physics->GetExternalImpactImpulse(*aimTarget, nullptr, &contacts);
                for (const PhysicsContactImpulse& contact : contacts)
                {
                    if (contact.fromProjectile && contact.impulse.magnitudeSquared() > 0)
                    {
                        error = (contact.position - wallSight.position).magnitude();
                        report << "wall_sight=(" << wallSight.position.x << ',' << wallSight.position.y << ',' << wallSight.position.z
                            << ") contact=(" << contact.position.x << ',' << contact.position.y << ',' << contact.position.z << ')' << std::endl;
                        break;
                    }
                }
            }
            report << "wall_first_contact_error_m=" << error << " speed_mps=" << speed << std::endl;
            wallAccurate = wallAccurate && error < .025f;
            gravityRestored = gravityRestored && !ball->getActorFlags().isSet(PxActorFlag::eDISABLE_GRAVITY);
            test.ClearProjectiles();
        }
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
        const std::vector<PhysicsBody>& targetBodies = dummy->GetMesh().GetPhysicsAsset()->bodies;
        const std::vector<PhysicsBody>::const_iterator head = std::find_if(targetBodies.begin(), targetBodies.end(),
            [](const PhysicsBody& body) -> bool
            {
                return body.bone.find("Head") != std::string::npos;
            });
        PxGeometryHolder headGeometry;
        PxTransform inBone;
        PxTransform headPose;
        const size_t headIndex = static_cast<size_t>(head - targetBodies.begin());
        check(ragdoll.GetBodyCollisionShape(headIndex, headGeometry, inBone, headPose), "Locate the idle head collider");
        const PxVec3 towardsHead = (headPose.p - toPx(shotOrigin)).getNormalized();
        const PxVec3 shoulder = toPx(shotOrigin) - towardsHead * 1.65f +
            towardsHead.cross(PxVec3(0, 1, 0)).getNormalized() * .55f + PxVec3(0, .56f, 0);
        const PxVec3 headAim = (headPose.p - shoulder).getNormalized();
        PxGeomRaycastHit headSight;
        check(PxGeometryQuery::raycast(shoulder, headAim, headGeometry.any(), headPose, 100, PxHitFlag::eDEFAULT, 1, &headSight) == 1,
            "The shoulder camera crosshair points at the idle head surface");
        check(test.SetProjectileSettings(.5f, 100.0f) &&
                test.FireAimed(player, Vector3(shoulder.x, shoulder.y, shoulder.z), Vector3(headAim.x, headAim.y, headAim.z)),
            "Fire from the player's muzzle at the idle head");
        float headError = std::numeric_limits<float>::infinity();
        size_t hitBody = targetBodies.size();
        std::vector<RagdollBodyImpulse> headContacts;
        for (int frame = 0; frame < 30 && hitBody == targetBodies.size(); ++frame)
        {
            physics->Step();
            ragdoll.CaptureExternalImpacts(headContacts);
            for (const RagdollBodyImpulse& contact : headContacts)
            {
                if (contact.fromProjectile && contact.impulse.magnitudeSquared() > 0)
                {
                    hitBody = contact.body;
                    headError = (contact.position - headSight.position).magnitude();
                    break;
                }
            }
        }
        report << "head_first_contact_error_m=" << headError << " body=" << hitBody << " expected=" << headIndex << std::endl;
        check(wallAccurate, "First physical wall contact lands within 2.5 cm of the crosshair at both speeds");
        check(gravityRestored, "Gravity acts on the ball after impact at both speeds");
        check(hitBody == headIndex && headError < .025f, "First physical dummy contact hits the sighted head surface within 2.5 cm");
        check(test.SetProjectileSettings(.5f, 7.5f), "Restore the support-controller projectile fixture");
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

        // 유효한 B 밖에서의 지지 유지와 지지면 자체가 사라졌을 때의 해제를 각각 확인한다.
        world.SetPhysicsPaused(true);
        const std::function<void()> singleStep = [&]() -> void
        {
            world.RequestPhysicsStep();
            tick();
        };
        const PARS::FootSoleSettings originalLeft = support.GetSoleSettings(0);
        const PARS::FootSoleSettings originalRight = support.GetSoleSettings(1);
        const float previousWeight = ragdoll.GetRootSupportWeight();
        support.GetSoleSettings(0).offset.x += 2;
        support.GetSoleSettings(1).offset.x += 2;
        singleStep();
        check(support.GetSnapshot().balance.zone == FootSupportZone::Outside &&
                dummy->GetMesh().GetRootSupportRelease() == RootSupportRelease::None &&
                ragdoll.UsesFootDrivenSupport() && ragdoll.GetRootSupportWeight() == previousWeight,
            "Leaving B preserves the previous assistance and the foot controller");
        support.GetSoleSettings(0) = originalLeft;
        support.GetSoleSettings(1) = originalRight;
        singleStep();
        check(support.GetSnapshot().balance.weight > 0 && ragdoll.GetRootSupportWeight() == support.GetSnapshot().balance.weight &&
                dummy->GetMesh().GetRootSupportRelease() == RootSupportRelease::None && ragdoll.AreDrivesEnabled(),
            "Re-entering B applies the current support weight");
        check(resetActive() && ragdoll.GetRootSupportWeight() == 1 &&
                dummy->GetMesh().GetRootSupportRelease() == RootSupportRelease::None,
            "Dummy reset rearms support from its newly sampled grounded pose");
        support.GetSoleSettings(0).offset.y += 1;
        support.GetSoleSettings(1).offset.y += 1;
        singleStep();
        check(dummy->GetMesh().GetRootSupportRelease() == RootSupportRelease::NoSupport && !ragdoll.IsRootSupported(),
            "Invalid foot support fully releases Y/rotation support");
        support.GetSoleSettings(0) = originalLeft;
        support.GetSoleSettings(1) = originalRight;
        singleStep();
        std::string supportError;
        check(dummy->GetMesh().SetAutomaticRootSupport(true, supportError) && !ragdoll.IsRootSupported(),
            "Restoring a valid support area does not clear a genuine loss-of-support latch");
        check(resetActive(), "Restore the automatic impact fixture");
        world.SetPhysicsPaused(false);

        float idleMinimumWeight = 1;
        float idleFootForce = 0;
        for (int frame = 0; frame < 600; ++frame)
        {
            tick();
            if (idleMinimumWeight > 0.0f && !ragdoll.IsRootSupported())
            {
                const FootSupportSnapshot& feet = support.GetSnapshot();
                report << "idle_release_frame=" << frame << " reason=" << static_cast<int>(dummy->GetMesh().GetRootSupportRelease())
                    << " zone=" << static_cast<int>(feet.balance.zone)
                    << " outside_b_time_s=" << feet.footstep.outsideBufferTime << std::endl;
            }
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
            float minimumWeight = 1;
            float minimumHeight = initialHeight;
            int bandFrames = 0;
            int releaseFrame = -1;
            bool matched = true;
            bool stayedReleased = true;
            bool finiteAndDriven = true;
            for (int frame = 0; frame < 300; ++frame)
            {
                tick();
                const FootSupportBalance& balance = support.GetSnapshot().balance;
                const float applied = ragdoll.GetRootSupportWeight();
                minimumWeight = PxMin(minimumWeight, applied);
                minimumHeight = PxMin(minimumHeight, float(ragdoll.GetPosition().GetY()));
                if (balance.zone == FootSupportZone::Buffer && ragdoll.IsRootSupported())
                {
                    ++bandFrames;
                    matched &= std::abs(balance.weight - applied) < 1e-6f;
                }
                if (!ragdoll.IsRootSupported() && releaseFrame < 0)
                {
                    releaseFrame = frame;
                }
                if (releaseFrame >= 0)
                {
                    stayedReleased &= applied == 0;
                }
                finiteAndDriven &= ragdoll.IsFinite() && (ragdoll.AreDrivesEnabled() == dummy->GetMesh().IsActiveRagdoll());
            }
            report << "automatic_impact mass=" << masses[trial] << " speed=" << speeds[trial]
                << " min_weight=" << minimumWeight << " band_frames=" << bandFrames << " release_frame=" << releaseFrame
                << " min_hips_y=" << minimumHeight << " max_displacement=" << test.GetMaxHipsDisplacement() << std::endl;
            check(matched && stayedReleased && finiteAndDriven,
                "Impact preserves the support weight, release latch and drives for the current physics mode");
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
