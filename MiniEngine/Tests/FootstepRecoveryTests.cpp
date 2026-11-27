#include "../Model/FootstepPlanner.h"
#include "foundation/PxQuat.h"
#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstdlib>

using namespace PARS;
using namespace physx;

namespace
{
    unsigned checks = 0;

    void Check(bool condition, const char* message)
    {
        ++checks;
        if (!condition)
        {
            std::fprintf(stderr, "FAIL: recovery regions: %s\n", message);
            std::exit(1);
        }
    }

    FootstepPolygon Rectangle(float minX, float maxX, float minZ, float maxZ)
    {
        return {{minX, 0, minZ}, {maxX, 0, minZ}, {maxX, 0, maxZ}, {minX, 0, maxZ}};
    }

    FootstepRecoveryInput Input()
    {
        FootstepRecoveryInput input;
        input.com = PxVec3(0, 1, 0);
        input.firstStepTime = .25f;
        input.followingStepTime = .3f;
        input.movingFoot = 1;
        input.stepping = true;
        input.normalTarget = PxVec3(.12f, 0, .3f);
        input.preferred = input.normalTarget;
        for (size_t side = 0; side < 2; ++side)
        {
            FootstepReachInput& reach = input.feet[side];
            const float x = side == 0 ? -.12f : .12f;
            reach.position = PxVec3(x, 0, 0);
            reach.hip = PxVec3(x, .65f, 0);
            reach.stepOrigin = reach.position;
            reach.legLength = 1;
            reach.minimumWidth = .168f;
            reach.outward = PxVec3(side == 0 ? -1.0f : 1.0f, 0, 0);
            reach.sole = Rectangle(x - .04f, x + .04f, -.1f, .1f);
            reach.available = true;
        }
        return input;
    }

    bool InAny(const std::vector<FootstepPolygon>& pieces, const PxVec3& point)
    {
        for (const FootstepPolygon& piece : pieces)
        {
            if (IsInsideFootSupportHull(piece, point))
            {
                return true;
            }
        }
        return false;
    }

    void TestPolygonGeometry()
    {
        const FootstepPolygon a = Rectangle(0, 2, 0, 2);
        const FootstepPolygon b = Rectangle(1, 3, 1, 3);
        Check(std::abs(FootstepPolygonArea(IntersectFootstepPolygons(a, b)) - 1) < 1e-5f,
            "intersection has the expected area");
        Check(std::abs(FootstepUnionArea({a, b}) - 7) < 1e-5f,
            "overlap is counted once in union area");
        Check(std::abs(FootstepUnionArea({a, a, a}) - 4) < 1e-5f,
            "duplicate pieces cannot bias mode selection");
        Check(std::abs(FootstepUnionArea({a, Rectangle(.5f, 1, .5f, 1)}) - 4) < 1e-5f,
            "contained pieces cannot inflate area");
        Check(std::abs(FootstepUnionArea({Rectangle(0, 1, 0, 1), Rectangle(1, 2, 0, 1)}) - 2) < 1e-5f,
            "opposite shared edges disappear from union boundary");
        Check(std::abs(FootstepUnionArea({Rectangle(0, 1, 0, 1), Rectangle(2, 3, 0, 1)}) - 2) < 1e-5f,
            "disconnected pieces retain the forbidden gap");
        PxVec3 centroid(0);
        const FootstepPolygon wing = Rectangle(1, 4, 0, 1);
        const FootstepPolygon island = Rectangle(6, 7, 0, 1);
        Check(std::abs(FootstepUnionArea({a, wing, a, island}, &centroid) - 7) < 1e-5f &&
            (centroid - PxVec3(16.5f / 7, 0, 5.5f / 7)).magnitude() < 1e-5f,
            "the capture center weights disconnected area and counts overlapping pieces only once");
        Check((ClosestFootstepPoint(a, PxVec3(3, 0, 1)) - PxVec3(2, 0, 1)).magnitude() < 1e-5f,
            "preferred target projects to the closest edge");
        Check(IntersectFootstepPolygons(a, Rectangle(5, 6, 5, 6)).empty(),
            "disjoint polygons have no feasible intersection");
        for (uint32_t index = 0; index < 30; ++index)
        {
            const float offset = static_cast<float>(index) * .13f;
            FootstepPolygon first = Rectangle(0, 1, 0, 1);
            FootstepPolygon second = Rectangle(offset, offset + 1, .3f, 1.3f);
            const float expected = 2 - (std::max)(0.0f, 1 - offset) * .7f;
            const PxQuat rotation(.37f, PxVec3(0, 1, 0));
            for (PxVec3& point : first)
            {
                point = rotation.rotate(point) + PxVec3(20, 0, -15);
            }
            for (PxVec3& point : second)
            {
                point = rotation.rotate(point) + PxVec3(20, 0, -15);
            }
            Check(std::abs(FootstepUnionArea({first, second, first}) - expected) < 3e-5f,
                "rotated and translated overlapping polygons preserve exact union area");
        }
    }

    void TestGroundFallFoothold()
    {
        FootstepRecoveryInput input = Input();
        input.fallingStep = true;
        input.velocity.z = -3.0f;
        input.preferred.z = -1.0f;
        FootstepRecoverySettings settings;
        settings.maximumSteps = 1;
        settings.oneStepCaptureScale = 1.0f;
        settings.maximumPoseChecks = 2;
        const PxVec3 underHip(.12f, 0, 0);
        input.checkTargetPose = [underHip](const PxVec3& position) -> FootstepPoseFailures
        {
            return (position - underHip).magnitudeSquared() < 1e-6f ? FootstepPoseFailure::None : FootstepPoseFailure::Reach;
        };
        const FootstepRecoverySnapshot result = SelectFootstepRecovery(input, settings);
        Check(result.hasTarget && result.status == FootstepRecoveryStatus::BestEffort &&
            result.poseChecks == 2 && (result.target - underHip).magnitudeSquared() < 1e-6f,
            "A falling character can plant under the hip within the usual pose budget when balance is out of reach");

        input.checkTargetPose = [](const PxVec3& position) -> FootstepPoseCheck
        {
            FootstepPoseCheck check(FootstepPoseFailure::Reach);
            check.solvedHipAnkleDistance = .6f;
            check.soleError = .01f + std::abs(position.z);
            return check;
        };
        const FootstepRecoverySnapshot attempt = SelectFootstepRecovery(input, settings);
        Check(attempt.hasTarget && !attempt.modelFeasible && attempt.status == FootstepRecoveryStatus::BestEffort &&
            (attempt.target - underHip).magnitudeSquared() < 1e-6f,
            "A fall can aim for the closest joint-limited pose when no sole fits the ground exactly");
        input.fallingStep = false;
        Check(!SelectFootstepRecovery(input, settings).hasTarget, "Ordinary recovery still requires an attainable foothold");
        input.fallingStep = true;
        input.checkTargetPose = [](const PxVec3&) -> FootstepPoseCheck
        {
            FootstepPoseCheck check(FootstepPoseFailure::Reach);
            check.failures |= FootstepPoseFailure::OppositeLegOverlap;
            check.solvedHipAnkleDistance = .6f;
            check.soleError = .01f;
            return check;
        };
        Check(!SelectFootstepRecovery(input, settings).hasTarget, "A fall attempt cannot use a pose that intersects the other leg");
    }

    void TestNormalReachDisplay()
    {
        FootstepRecoverySettings settings;
        settings.maximumSteps = 1;
        settings.oneStepCaptureScale = 1.0f;
        FootstepRecoveryInput input = Input();
        input.velocity = PxVec3(-.8f, 0, 0);
        input.com.z = .15f;
        const FootstepRecoverySnapshot cross = SelectFootstepRecovery(input, settings);
        const FootstepPolygon geometry = cross.normalReach.BuildPolygon();
        const PxVec3 beyondShortStep(.45f, 0, 0);
        Check(cross.mode == FootstepMode::FrontCross && cross.overlapAreas[0] == 0 &&
            IsInsideFootSupportHull(geometry, beyondShortStep) && !IsInsideFootSupportHull(geometry, PxVec3(.02f, 0, 0)),
            "black reach exists during a crossover and excludes the stance-foot clearance");

        input.velocity = PxVec3(0);
        input.feet[input.movingFoot].maximumDistance = .08f;
        input.checkTargetPose = [](const PxVec3&) -> FootstepPoseFailures
        {
            return FootstepPoseFailure::Reach;
        };
        const FootstepRecoverySnapshot rejected = SelectFootstepRecovery(input, settings);
        Check(rejected.status == FootstepRecoveryStatus::PoseRejected && rejected.normalReach.BuildPolygon() == geometry &&
            !IsInsideFootSupportHull(rejected.reachable[0], beyondShortStep),
            "pose rejection and the command stride cap preserve the geometric overlay");
        input.stepping = false;
        const FootstepRecoverySnapshot preview = SelectFootstepRecovery(input, settings);
        Check(preview.status == FootstepRecoveryStatus::Preview && preview.normalReach.BuildPolygon() == geometry,
            "preview and swing use the same geometric overlay");
        input.firstStepTime = 0;
        const FootstepRecoverySnapshot noPrediction = SelectFootstepRecovery(input, settings);
        Check(noPrediction.status == FootstepRecoveryStatus::Unavailable && noPrediction.normalReach.BuildPolygon() == geometry,
            "the geometric overlay can be drawn before a capture prediction is available");
        input.feet[input.movingFoot].hip.y = 2;
        const FootstepRecoverySnapshot unreachable = SelectFootstepRecovery(input, settings);
        Check(unreachable.normalReach.BuildPolygon().empty(), "black reach ends when the leg cannot reach the ground");
    }

    void TestModeChoice()
    {
        FootstepRecoverySettings settings;
        settings.maximumSteps = 1;
        FootstepRecoveryInput input = Input();
        input.velocity = PxVec3(0, 0, .6f);
        input.normalTarget.x += .025f;
        FootstepRecoverySnapshot result = SelectFootstepRecovery(input, settings);
        Check(result.hasTarget && result.mode == FootstepMode::Normal && result.modelFeasible,
            "ordinary forward recovery keeps the normal mode");
        Check(InAny(result.intersections[0], result.target) && (result.target - input.normalTarget).magnitudeSquared() < 1e-8f,
            "normal selection uses the corrected stance target even when the crossover preference differs");
        input.velocity = PxVec3(-.8f, 0, 0);
        input.com.z = .15f;
        result = SelectFootstepRecovery(input, settings);
        Check(result.overlapAreas[0] == 0 && result.hasTarget && result.mode == FootstepMode::FrontCross,
            "inward and forward demand selects front crossover");
        Check(result.target.x < input.feet[0].position.x && InAny(result.intersections[1], result.target),
            "front crossover can land beyond the stance foot instead of being clamped outward");
        const FootstepRecoverySnapshot front = result;
        input.normalTarget = PxVec3(10, 0, -10);
        input.preferred = PxVec3(-10, 0, -10);
        result = SelectFootstepRecovery(input, settings);
        Check(result.mode == front.mode && std::abs(result.overlapAreas[1] - front.overlapAreas[1]) < 1e-6f,
            "corrected and preferred targets cannot change physical recovery regions");
        input = Input();
        input.velocity = PxVec3(-.8f, 0, 0);
        input.com.z = -.15f;
        result = SelectFootstepRecovery(input, settings);
        Check(result.hasTarget && result.mode == FootstepMode::BackCross && result.target.z < 0,
            "inward and backward demand selects rear crossover");
        input.velocity = PxVec3(-10, 0, 0);
        result = SelectFootstepRecovery(input, settings);
        Check(result.hasTarget && !result.modelFeasible && result.status == FootstepRecoveryStatus::BestEffort,
            "unreachable recovery is reported as best effort rather than feasible");
        const FootstepRecoverySettings threeSteps;
        input.modeSelection = FootstepRecoveryInput::ModeSelection::KeepCurrent;
        for (FootstepMode mode : {FootstepMode::Normal, FootstepMode::FrontCross, FootstepMode::BackCross})
        {
            input.previousMode = mode;
            result = SelectFootstepRecovery(input, threeSteps);
            Check(result.hasTarget && result.status == FootstepRecoveryStatus::BestEffort && result.mode == mode,
                "three-step best-effort search stays within the mode chosen for the step");
        }
        input = Input();
        input.feet[1].hip.y = 2;
        result = SelectFootstepRecovery(input, settings);
        Check(!result.hasTarget && result.status == FootstepRecoveryStatus::NoReach,
            "a fully unreachable fixed hip cannot fabricate a crossover region");
        input = Input();
        input.velocity = PxVec3(-.8f, 0, 0);
        input.com.z = .15f;
        input.checkTargetPose = [](const PxVec3&) -> FootstepPoseCheck
        {
            FootstepPoseCheck check;
            check.joints[0].correctionRadians = .2f;
            check.joints[1].correctionRadians = .3f;
            check.joints[2].correctionRadians = .82f;
            return check;
        };
        result = SelectFootstepRecovery(input, settings);
        Check(result.hasTarget && !result.poseRejected && result.poseChecks == 1 &&
            result.poseCheckResults[0].joints[2].correctionRadians == .82f,
            "A reachable first candidate is accepted even when all three joint targets need clamping");
        input.locked = true;
        input.lockedTarget = PxVec3(.1f, 0, .2f);
        input.previousMode = FootstepMode::Normal;
        result = SelectFootstepRecovery(input, settings);
        Check(result.status == FootstepRecoveryStatus::Locked && result.target == input.lockedTarget && result.poseChecks == 0,
            "landing lock preserves the target and avoids a second pose search");
    }

    void TestCaptureCenterCrossing()
    {
        FootstepRecoverySettings settings;
        settings.maximumSteps = 1;
        FootstepRecoveryInput input = Input();
        input.firstStepTime = .4f;
        const float growth = std::exp(std::sqrt(9.81f) * input.firstStepTime);
        for (size_t moving = 0; moving < input.feet.size(); ++moving)
        {
            input.movingFoot = moving;
            input.feet[moving].minimumWidth = 0;
            const FootstepReachInput& support = input.feet[1 - moving];
            for (float forward : {-.32f, .32f})
            {
                for (float outward : {-.01f, 0.0f, .01f})
                {
                    const PxVec3 offset = support.outward * outward + input.forward * forward;
                    input.com = support.position + offset / growth + PxVec3(0, 1, 0);
                    const FootstepRecoverySnapshot result = SelectFootstepRecovery(input, settings);
                    const FootstepMode crossMode = forward > 0 ? FootstepMode::FrontCross : FootstepMode::BackCross;
                    Check((result.captureCenter - support.position - offset).magnitude() < 1e-5f &&
                        result.overlapAreas[0] > 0 && result.overlapAreas[static_cast<size_t>(crossMode)] > 0,
                        "the capture straddles the support foot with feasible normal and cross landings");
                    Check(result.hasTarget && result.modelFeasible && result.captureBeyondSupport == (outward > 0) &&
                        result.mode == (outward > 0 ? crossMode : FootstepMode::Normal),
                        "either stance foot crosses only when the center passes its lateral position");
                }
            }
        }
        input = Input();
        input.com = PxVec3(input.feet[0].position.x + .01f, 1, .15f);
        const FootstepRecoverySnapshot inside = SelectFootstepRecovery(input, settings);
        Check(inside.overlapAreas[0] == 0 && inside.overlapAreas[1] > 0 && inside.hasTarget &&
            inside.mode == FootstepMode::Normal && inside.status == FootstepRecoveryStatus::BestEffort,
            "cross overlap alone cannot turn an inside center into a crossover");
        input.com.x = input.feet[0].position.x - .01f;
        input.fallingStep = true;
        const FootstepRecoverySnapshot outside = SelectFootstepRecovery(input, settings);
        Check(outside.hasTarget && outside.mode == FootstepMode::FrontCross,
            "a fall cannot inject a normal under-hip target into the selected cross modes");

        input.fallingStep = false;
        input.com.z = 0;
        input.firstStepTime = .45f;
        settings.maximumPoseChecks = 2;
        const FootstepRecoverySnapshot firstChoice = SelectFootstepRecovery(input, settings);
        const bool frontFirst = firstChoice.mode == FootstepMode::FrontCross;
        input.checkTargetPose = [frontFirst](const PxVec3& point) -> FootstepPoseFailures
        {
            return (point.z > 0) == frontFirst ? FootstepPoseFailure::OppositeLegOverlap : FootstepPoseFailure::None;
        };
        const FootstepRecoverySnapshot alternative = SelectFootstepRecovery(input, settings);
        Check(firstChoice.overlapAreas[1] > 0 && firstChoice.overlapAreas[2] > 0 && alternative.hasTarget &&
            alternative.modelFeasible && alternative.poseChecks == 2 &&
            alternative.mode == (frontFirst ? FootstepMode::BackCross : FootstepMode::FrontCross),
            "the other cross direction gets a pose check when the preferred direction is blocked");
    }

    void TestNearestNormalLanding()
    {
        FootstepRecoverySettings settings;
        settings.maximumSteps = 1;
        settings.oneStepCaptureScale = 1.0f;
        FootstepRecoveryInput input = Input();
        input.feet[1].maximumDistance = .4f;
        for (float direction : {-1.0f, 1.0f})
        {
            input.velocity.z = direction * 4.0f;
            const FootstepRecoverySnapshot step = SelectFootstepRecovery(input, settings);
            const PxVec3 expected = input.feet[1].position + PxVec3(0, 0, direction * .4f);
            Check(step.hasTarget && step.mode == FootstepMode::Normal && step.status == FootstepRecoveryStatus::BestEffort &&
                (step.target - expected).magnitude() < 1e-5f,
                "a distant forward or backward capture chooses the closest normal landing within the stride limit");
        }

        // 회복 사각형의 가까운 꼭짓점을 일반 도달 다각형의 사선 변 중점 바깥에 둔다.
        input.velocity = PxVec3(0);
        const FootstepRecoverySnapshot initial = SelectFootstepRecovery(input, settings);
        const PxVec3 corner = ClosestFootstepPoint(initial.capture.front(), PxVec3(-10, 0, -10));
        const float offset = .5f * input.feet[1].maximumDistance * (.5f + std::sqrt(.75f));
        const PxVec3 edgeMidpoint = input.feet[1].position + PxVec3(offset, 0, offset);
        const float growth = std::exp(std::sqrt(9.81f / input.com.y) * input.firstStepTime);
        input.com += (edgeMidpoint + PxVec3(.25f, 0, .25f) - corner) / growth;
        FootstepPolygon checked;
        input.checkTargetPose = [&checked](const PxVec3& point) -> FootstepPoseFailures
        {
            checked.push_back(point);
            return checked.size() == 3 ? FootstepPoseFailure::None : FootstepPoseFailure::OppositeLegOverlap;
        };
        const FootstepRecoverySnapshot step = SelectFootstepRecovery(input, settings);
        Check(checked.size() == 3 && step.hasTarget && step.mode == FootstepMode::Normal &&
            (checked.front() - edgeMidpoint).magnitude() < 1e-5f && step.target == checked.back(),
            "a rejected nearest point on an edge is followed by another validated normal candidate");
        float previousGap = 0;
        for (const PxVec3& point : checked)
        {
            const float gap = (point - ClosestFootstepPoint(step.capture.front(), point)).magnitude();
            Check(IsInsideFootSupportHull(step.reachable[0], point) && gap + 1e-6f >= previousGap,
                "all normal candidates are tested in order of distance to capture");
            previousGap = gap;
        }
        input.tryShorterFirstStep = true;
        checked.clear();
        const FootstepRecoverySnapshot handoff = SelectFootstepRecovery(input, settings);
        const PxVec3 start = ClosestFootstepPoint(handoff.reachable[0], input.feet[input.movingFoot].stepOrigin);
        Check(checked.size() == 3 && handoff.poseChecks == 3 && handoff.hasTarget && handoff.target == checked.back() &&
            (checked.front() - edgeMidpoint).magnitude() < 1e-5f && IsInsideFootSupportHull(handoff.reachable[0], handoff.target) &&
            (checked[2] - start).magnitude() < (checked[1] - start).magnitude() &&
            (checked[1] - start).magnitude() < (checked[0] - start).magnitude(),
            "A warmup handoff tests progressively shorter reachable strides after the nearest landing is rejected");
    }

    void TestCaptureAndTiming()
    {
        FootstepRecoverySettings settings;
        settings.maximumSteps = 1;
        settings.oneStepCaptureScale = 1.0f;
        FootstepRecoveryInput input = Input();
        input.velocity = PxVec3(.3f, 0, .5f);
        const FootstepRecoverySnapshot first = SelectFootstepRecovery(input, settings);
        const float omega = std::sqrt(9.81f);
        Check(std::abs(first.icp.x - .3f / omega) < 1e-6f && std::abs(first.icp.z - .5f / omega) < 1e-6f,
            "ICP uses actual COM and horizontal velocity");
        Check(!first.hasTarget || !first.modelFeasible || first.requiredSteps == 1,
            "one-step feasible candidates do not request unnecessary followup steps");
        const float growth = std::exp(omega * input.firstStepTime);
        for (const PxVec3& cop : input.feet[0].sole)
        {
            const PxVec3 future = first.icp * growth + cop * (1 - growth);
            for (const PxVec3& sole : input.feet[1].sole)
            {
                const PxVec3 local = sole - input.feet[1].position;
                Check(InAny(first.capture, future - local),
                    "one-step region contains analytically computed future ICP and finite-foot offsets");
            }
        }
        settings.maximumSteps = 3;
        const FootstepRecoverySnapshot multi = SelectFootstepRecovery(input, settings);
        Check(multi.steps == 3 && multi.capturePieceCount <= 13 && multi.usesFutureReachApproximation,
            "multistep computation is bounded and explicitly identifies the future reach approximation");
        for (const PxVec3& point : first.capture[0])
        {
            Check(InAny(multi.capture, point), "multistep region includes one-step recovery");
        }
        input.firstStepTime = .5f;
        settings.maximumSteps = 1;
        const FootstepRecoverySnapshot delayed = SelectFootstepRecovery(input, settings);
        Check((ClosestFootstepPoint(delayed.capture[0], input.preferred) -
            ClosestFootstepPoint(first.capture[0], input.preferred)).magnitude() > .01f,
            "time spent transferring load changes required landing positions");
        input.stepping = false;
        const FootstepRecoverySnapshot preview = SelectFootstepRecovery(input, settings);
        Check(preview.status == FootstepRecoveryStatus::Preview && !preview.hasTarget && preview.poseChecks == 0,
            "idle preview does not run PA pose search or issue a command");
        input = Input();
        input.com = PxVec3(0, 1, .15f);
        input.velocity = PxVec3(-.8f, 0, 0);
        const FootstepRecoverySnapshot beforeTurn = SelectFootstepRecovery(input, settings);
        const PxQuat yaw(PxHalfPi, PxVec3(0, 1, 0));
        input.com = yaw.rotate(input.com);
        input.velocity = yaw.rotate(input.velocity);
        input.forward = yaw.rotate(input.forward);
        input.preferred = yaw.rotate(input.preferred);
        input.normalTarget = yaw.rotate(input.normalTarget);
        for (FootstepReachInput& foot : input.feet)
        {
            foot.position = yaw.rotate(foot.position);
            foot.hip = yaw.rotate(foot.hip);
            foot.ankleOffset = yaw.rotate(foot.ankleOffset);
            foot.outward = yaw.rotate(foot.outward);
            foot.stepOrigin = yaw.rotate(foot.stepOrigin);
            for (PxVec3& corner : foot.sole)
            {
                corner = yaw.rotate(corner);
            }
        }
        const FootstepRecoverySnapshot turned = SelectFootstepRecovery(input, settings);
        Check(turned.mode == beforeTurn.mode && std::abs(turned.overlapAreas[1] - beforeTurn.overlapAreas[1]) < 1e-5f,
            "crossover selection follows character forward rather than world axes");
    }

    void TestThreeStepReachForBothFeet()
    {
        const FootstepRecoverySettings threeSteps;
        Check(threeSteps.maximumSteps == 3, "the default planner considers three recovery steps");
        FootstepRecoverySettings twoSteps = threeSteps;
        twoSteps.maximumSteps = 2;
        for (size_t side = 0; side < 2; ++side)
        {
            FootstepRecoveryInput input = Input();
            input.movingFoot = side;
            input.velocity = input.feet[side].outward * -1.3f + input.forward * 1.4f;
            input.normalTarget.x = input.feet[side].position.x;
            input.preferred = input.normalTarget;
            const FootstepRecoverySnapshot two = SelectFootstepRecovery(input, twoSteps);
            const FootstepRecoverySnapshot three = SelectFootstepRecovery(input, threeSteps);
            Check(three.capture.size() > 1 && three.captureBeyondSupport && three.mode == FootstepMode::FrontCross,
                "the center of the combined three-step capture can request a crossover for either stance foot");
            Check(!two.modelFeasible && three.hasTarget && three.modelFeasible && three.requiredSteps == 3,
                "either moving foot can use a landing that requires all three steps");
        }
    }

    void TestOneStepCaptureMargin()
    {
        FootstepRecoverySettings settings;
        settings.maximumSteps = 1;
        FootstepRecoverySettings exact = settings;
        exact.oneStepCaptureScale = 1.0f;
        FootstepRecoveryInput input = Input();
        const FootstepRecoverySnapshot original = SelectFootstepRecovery(input, exact);
        const FootstepRecoverySnapshot expanded = SelectFootstepRecovery(input, settings);
        Check(expanded.steps == 1 && expanded.capture.size() == 1 && original.capture.size() == 1,
            "one-step expansion keeps a single capture polygon");
        Check(std::abs(FootstepPolygonArea(expanded.capture.front()) / FootstepPolygonArea(original.capture.front()) -
            settings.oneStepCaptureScale * settings.oneStepCaptureScale) < 1e-4f && expanded.reachable == original.reachable,
            "only the capture polygon grows by the configured linear scale");
        Check(expanded.hasTarget && expanded.modelFeasible && expanded.requiredSteps == 1 && !expanded.targetInCaptureMargin &&
            expanded.mode == original.mode && (expanded.target - original.target).magnitude() < 1e-6f,
            "a model-feasible landing keeps priority over the margin");

        const PxVec3 edge = ClosestFootstepPoint(original.capture.front(), PxVec3(.14f, 0, 1));
        const PxVec3 marginTarget = edge + PxVec3(0, 0, .02f);
        input.feet[1].stepOrigin = marginTarget;
        input.feet[1].maximumDistance = .003f;
        input.normalTarget = marginTarget;
        const FootstepRecoverySnapshot margin = SelectFootstepRecovery(input, settings);
        Check(margin.hasTarget && margin.status == FootstepRecoveryStatus::BestEffort && margin.targetInCaptureMargin &&
            !margin.modelFeasible && margin.requiredSteps == 0 && !InAny(original.capture, margin.target) &&
            InAny(margin.intersections[static_cast<size_t>(margin.mode)], margin.target),
            "a margin-only reachable target is accepted without claiming one-step capture");
        input.locked = true;
        input.previousMode = margin.mode;
        input.lockedTarget = margin.target;
        const FootstepRecoverySnapshot locked = SelectFootstepRecovery(input, settings);
        Check(locked.status == FootstepRecoveryStatus::Locked && locked.target == margin.target &&
            locked.targetInCaptureMargin && !locked.modelFeasible && locked.poseChecks == 0,
            "a locked margin target stays fixed and retains its classification");
        input = Input();
        FootstepPolygon checked;
        input.checkTargetPose = [&checked](const PxVec3& point) -> FootstepPoseCheck
        {
            checked.push_back(point);
            FootstepPoseCheck check;
            if (checked.size() == 1)
            {
                check.failures = FootstepPoseFailure::Reach;
                check.failures |= FootstepPoseFailure::InvalidInput;
                check.joints[0].inputIssue = FootstepJointInputIssue::Constraint;
                check.soleError = .015f;
                check.reachLimited = true;
                check.joints[1].correctionRadians = .21f;
                check.joints[2].correctionRadians = .08f;
            }
            else
            {
                check.failures = FootstepPoseFailure::OppositeLegOverlap;
                check.overlapPairs = (1u << 2) | (1u << 7);
            }
            return check;
        };
        const FootstepRecoverySnapshot rejected = SelectFootstepRecovery(input, settings);
        Check(!rejected.hasTarget && rejected.poseRejected && checked.size() == rejected.poseChecks &&
            checked.size() <= settings.maximumPoseChecks, "both regions share a single pose-check budget");
        Check(std::equal(checked.begin(), checked.end(), rejected.poseCheckPositions.begin()),
            "the snapshot retains every rejected landing position in pose-check order");
        const FootstepPoseCheck& firstCheck = rejected.poseCheckResults[0];
        const FootstepPoseCheck& secondCheck = rejected.poseCheckResults[1];
        Check(rejected.poseChecks > 1 && firstCheck.failures.isSet(FootstepPoseFailure::Reach) &&
            firstCheck.failures.isSet(FootstepPoseFailure::InvalidInput) && !firstCheck.failures.isSet(FootstepPoseFailure::OppositeLegOverlap) &&
            secondCheck.failures == FootstepPoseFailure::OppositeLegOverlap,
            "simultaneous and distinct failures stay attached to the correct candidates across the search");
        Check(rejected.movingFoot == input.movingFoot && firstCheck.reachLimited && firstCheck.soleError == .015f &&
            firstCheck.joints[0].correctionRadians == 0 && firstCheck.joints[1].correctionRadians == .21f &&
            firstCheck.joints[2].correctionRadians == .08f && firstCheck.overlapPairs == 0 &&
            secondCheck.overlapPairs == ((1u << 2) | (1u << 7)) && !secondCheck.reachLimited && secondCheck.soleError == 0,
            "joint corrections, reach error and contact pairs stay with their tested foot and candidate");
    }

    FootstepInput PlannerInput()
    {
        FootstepInput input;
        input.active = true;
        input.automaticSupport = true;
        input.idleAnimation = true;
        input.hasCom = true;
        input.hasVelocity = true;
        input.hasReference = true;
        input.fixedDeltaTime = 1.0f / 60;
        input.bodyWeight = 700;
        input.requestedFoot = 1;
        input.com = PxVec3(0, 1, .15f);
        input.velocity = PxVec3(-.8f, 0, 0);
        input.hull = Rectangle(-.16f, .16f, -.1f, .1f);
        input.zone = FootSupportZone::Buffer;
        input.revision = 1;
        const FootstepRecoveryInput geometry = Input();
        for (size_t side = 0; side < 2; ++side)
        {
            FootstepFootInput& foot = input.feet[side];
            foot.available = true;
            foot.hasContactSample = true;
            foot.grounded = true;
            foot.normal = PxVec3(0, 1, 0);
            foot.position = geometry.feet[side].position;
            foot.contactPosition = foot.position;
            foot.stanceOffset = foot.position;
            foot.hasSoleRectangle = true;
            std::copy(geometry.feet[side].sole.begin(), geometry.feet[side].sole.end(), foot.corners.begin());
            foot.reach = geometry.feet[side];
            foot.calfLength = .5f;
            foot.normalImpulse = input.bodyWeight * input.fixedDeltaTime * .5f;
        }
        input.checkLandingPose = [](size_t, const PxVec3&, const PxVec3&, const PxVec3&) -> FootstepPoseFailures
        {
            return FootstepPoseFailure::None;
        };
        return input;
    }

    void TestLandingHeadingFromComLine()
    {
        FootstepSettings settings;
        settings.recovery.enabled = false;
        settings.landingComClampEnabled = false;
        for (size_t side = 0; side < 2; ++side)
        {
            const float sign = side == 0 ? -1.0f : 1.0f;
            FootstepInput input = PlannerInput();
            input.requestedFoot = static_cast<int>(side);
            input.com = PxVec3(sign * .3f, 1, 0);
            input.velocity = PxVec3(sign * .3f, 1.8f, 0);
            const PxVec3 toes = PxVec3(sign * .12f, 0, 1).getNormalized();
            input.feet[side].stanceForwardInComponent = toes;
            FootstepPlanner planner;
            planner.Update(input, settings);
            const FootstepCommand& command = planner.GetSnapshot().feet[side];
            Check(command.justStarted && sign * (command.target.position.x - command.startPosition.x) > .1f &&
                (command.target.soleForward - toes).magnitude() < 1e-5f,
                "A real lateral step at low horizontal COM speed keeps each foot's standing toe heading");

            planner.Reset();
            input.velocity = PxVec3(0, 1.8f, sign * .03f);
            planner.Update(input, settings);
            const FootstepCommand& aligned = planner.GetSnapshot().feet[side];
            const PxVec3 delta = aligned.target.position - aligned.startPosition;
            PxVec3 stride = PxVec3(delta.x, 0, delta.z).getNormalized();
            if (stride.dot(toes) < 0)
            {
                stride = -stride;
            }
            Check(aligned.justStarted && (stride - toes).magnitude() > .1f &&
                (aligned.target.soleForward - stride).magnitude() < 1e-5f,
                "Even a slow forward or backward COM line uses the actual foot trajectory as the heading");
        }

        FootstepInput input = PlannerInput();
        input.com = PxVec3(.18f, 1, .3f);
        input.velocity = PxVec3(-.6f, 0, -.6f);
        const PxVec3 toes = PxVec3(-.35f, 0, 1).getNormalized();
        input.feet[1].stanceForwardInComponent = toes;
        settings.landingComClampEnabled = true;
        settings.landingPredictionTime = settings.startPredictionTime;
        FootstepPlanner planner;
        planner.Update(input, settings);
        const FootstepTarget& target = planner.GetSnapshot().candidates[1];
        const FootstepComPrediction& prediction = target.landingCom;
        const float clampedWeight = GetFootstepStrideWeight(PxVec3(0, 0, 1), prediction.clamped - prediction.origin);
        const PxVec3 stride = target.position - input.feet[1].position;
        Check(prediction.limited && PxAbs(clampedWeight - .5f) > .01f &&
            (target.soleForward - GetFootstepSoleForward(stride, toes, .5f)).magnitude() < 1e-5f,
            "A 45-degree raw COM line keeps the half turn even when the support foot redirects the predicted point");
    }

    void TestNormalStepBoundary()
    {
        FootstepSettings settings;
        settings.recovery.maximumSteps = 1;
        Check(settings.crossingOffset == .3f, "Default crossing clearance is 30 cm");
        for (size_t side = 0; side < 2; ++side)
        {
            const float footSign = side == 0 ? -1.0f : 1.0f;
            for (const float travelSign : {-1.0f, 1.0f})
            {
                FootstepPlanner planner;
                FootstepInput input = PlannerInput();
                input.requestedFoot = static_cast<int>(side);
                input.com.z = 0;
                input.velocity = PxVec3(footSign * .6f, 0, travelSign * .6f);
                planner.Update(input, settings);

                const PxVec3 outward = PxVec3(footSign, 0, -travelSign).getNormalized();
                const PxVec3 support = input.feet[1 - side].position;
                const PxVec3 boundaryPoint = support + outward * .3f + input.velocity * .1f;
                const FootstepRecoverySnapshot& search = planner.GetSnapshot().candidates[side].recovery;
                const FootstepPolygon overlay = search.normalReach.BuildPolygon();
                const FootstepCommand& command = planner.GetSnapshot().feet[side];
                Check(command.justStarted && command.target.recovery.mode == FootstepMode::Normal &&
                    (search.normalReach.outward - outward).magnitude() < 1e-5f &&
                    (command.target.position - support).dot(outward) >= .3f - 1e-5f,
                    "Both feet land at least 30 cm outward across forward and backward diagonal travel");
                const PxVec3 inside = boundaryPoint + outward * .01f;
                const PxVec3 outside = boundaryPoint - outward * .01f;
                Check(IsInsideFootSupportHull(overlay, inside) && !IsInsideFootSupportHull(overlay, outside) &&
                    IsInsideFootSupportHull(search.reachable[0], inside) && !IsInsideFootSupportHull(search.reachable[0], outside),
                    "The black overlay and normal landing search cut at the same perpendicular distance");

                input.feet[side].grounded = false;
                input.velocity.x = -input.velocity.x;
                ++input.completedStep;
                planner.Update(input, settings);
                Check(command.phase == FootstepPhase::Airborne && (search.normalReach.outward - outward).magnitude() < 1e-5f &&
                    (command.target.position - support).dot(outward) >= .3f - 1e-5f,
                    "Changing COM direction during swing preserves the step-start boundary");
                input.feet[side].grounded = true;
                ++input.completedStep;
                planner.Update(input, settings);
                const PxVec3 nextOutward = PxVec3(footSign, 0, travelSign).getNormalized();
                Check(command.justLanded && (planner.GetSnapshot().candidates[side].normalStepOutward - nextOutward).magnitude() < 1e-5f,
                    "The next landing candidate uses the new travel direction after touchdown");
            }
        }

        FootstepRecoveryInput preview = Input();
        preview.stepping = false;
        for (const PxVec3& velocity : {PxVec3(.02f, 2, .02f), PxVec3(-1, 0, .1f), PxVec3(1, 0, 0)})
        {
            preview.velocity = velocity;
            const FootstepRecoverySnapshot result = SelectFootstepRecovery(preview, settings.recovery);
            Check(result.normalReach.outward == preview.feet[preview.movingFoot].outward,
                "Nearly stationary and sideways steps use the body's lateral axis");
        }

        FootstepInput input = PlannerInput();
        input.com = PxVec3(.18f, 1, .3f);
        input.velocity = PxVec3(-.6f, 0, -.6f);
        FootstepPlanner planner;
        planner.Update(input, settings);
        const FootstepComPrediction& prediction = planner.GetSnapshot().startCom;
        const PxVec3 outward = planner.GetSnapshot().candidates[1].recovery.normalReach.outward;
        Check(prediction.limited && std::abs((prediction.raw - prediction.origin).dot(outward)) < 1e-5f &&
            std::abs((prediction.clamped - prediction.origin).dot(outward)) > .01f,
            "The boundary follows the raw COM displacement when support-foot correction changes the prediction");

        planner.Reset();
        input = PlannerInput();
        input.velocity = PxVec3(0, 0, 1);
        planner.Update(input, settings);
        input.feet[1].grounded = false;
        ++input.completedStep;
        planner.Update(input, settings);
        const FootstepCommand& swing = planner.GetSnapshot().feet[1];
        Check(swing.phase == FootstepPhase::Airborne && swing.motion.swingOutwardOffset.isZero() &&
            swing.target.position.x - input.feet[0].position.x >= .3f - 1e-5f,
            "Starting with a narrow stance widens the landing without a sudden lateral offset at takeoff");
    }

    void TestFollowupAfterOneStepLanding()
    {
        FootstepSettings settings;
        settings.recovery.maximumSteps = 1;
        settings.recovery.oneStepCaptureScale = 1.0f;
        FootstepInput input = PlannerInput();
        input.requestedFoot = -1;
        input.hasImpact = true;
        input.impactPosition = input.feet[1].position;
        input.com.z = 0;
        input.velocity = PxVec3(0, 0, .6f);

        FootstepPlanner planner;
        planner.Update(input, settings);
        const FootstepSnapshot& step = planner.GetSnapshot();
        Check(step.feet[1].justStarted && step.feet[1].target.recovery.requiredSteps == 1,
            "the impact selects the right foot with a one-step capture prediction");
        input.feet[1].grounded = false;
        ++input.completedStep;
        planner.Update(input, settings);
        input.feet[1].grounded = true;
        input.feet[1].normalImpulse = 0;
        ++input.completedStep;
        planner.Update(input, settings);
        Check(step.feet[1].justLanded && step.requestedFoot == 0 && step.startCount == 1,
            "a real landing hands the next request to the left foot despite continued impact at the right");
        ++input.completedStep;
        planner.Update(input, settings);
        Check(step.requestedFoot == 0 && step.feet[0].reason == FootstepReason::WaitingSupportLoad && step.startCount == 1,
            "the left foot waits specifically for the landed right foot to bear weight");
        input.feet[1].normalImpulse = input.bodyWeight * settings.supportLoadRatio * input.fixedDeltaTime;
        ++input.completedStep;
        planner.Update(input, settings);
        Check(step.feet[0].justStarted && step.startCount == 2,
            "continued imbalance starts the opposite foot as soon as support load is confirmed");

        input.feet[0].grounded = false;
        ++input.completedStep;
        planner.Update(input, settings);
        input.feet[0].grounded = true;
        input.velocity = PxVec3(0);
        ++input.completedStep;
        planner.Update(input, settings);
        Check(step.feet[0].justLanded && !step.needsStep && step.followupFoot == -1 && step.startCount == 2,
            "regaining balance on landing ends the sequence without forcing another step");
    }

    void TestNormalLandingPrefersStanceTarget()
    {
        FootstepSettings settings;
        settings.recovery.maximumSteps = 1;
        settings.recovery.oneStepCaptureScale = 1.0f;
        FootstepInput input = PlannerInput();
        input.com.z = 0;
        input.velocity = PxVec3(0, 0, .6f);
        FootstepPlanner planner;
        planner.Update(input, settings);
        const FootstepTarget first = planner.GetSnapshot().candidates[1];
        Check(planner.GetSnapshot().feet[1].justStarted && first.recovery.mode == FootstepMode::Normal &&
            InAny(first.recovery.intersections[0], first.raw) && (first.position - first.raw).magnitudeSquared() < 1e-8f,
            "a normal step keeps the stance target when it lies inside the overlap");

        const float reachableTime = (MinimumFootstepDuration + MaximumFootstepDuration) * .5f;
        input.feet[0].timeToReachLimit = reachableTime;
        planner.Reset();
        planner.Update(input, settings);
        Check(std::abs(planner.GetSnapshot().candidates[1].recovery.predictionTime - reachableTime) < 1e-5f &&
            std::abs(GetFootstepRemainingTime(planner.GetSnapshot().feet[1]) - reachableTime) < 1e-5f,
            "Capture region and swing use the same time from the step decision to touchdown");
        input.feet[0].timeToReachLimit = 10.0f;
        planner.Reset();
        planner.Update(input, settings);
        const FootstepTarget& capped = planner.GetSnapshot().candidates[1];
        Check(planner.GetSnapshot().feet[1].justStarted && capped.recovery.hasTarget &&
            std::abs(capped.recovery.predictionTime - MaximumFootstepDuration) < 1e-5f &&
            std::abs(GetFootstepRemainingTime(planner.GetSnapshot().feet[1]) - MaximumFootstepDuration) < 1e-5f,
            "A long support forecast selects a validated target using the maximum step duration");
        input.feet[0].timeToReachLimit = std::numeric_limits<float>::infinity();
        // 기본 보폭 한도 안에서 선호점만 회복 영역 밖으로 옮긴다.
        input.feet[1].stanceOffset.x += .3f;
        FootstepPolygon checked;
        FootstepPolygon checkedHeadings;
        input.feet[1].stanceForwardInComponent = PxVec3(.2f, 0, 1).getNormalized();
        input.checkLandingPose = [&checked, &checkedHeadings, &input](size_t side, const PxVec3& point, const PxVec3& normal,
            const PxVec3& soleForward) -> FootstepPoseCheck
        {
            const PxVec3 expected = GetFootstepSoleForward(point - input.feet[side].position,
                input.feet[side].stanceForwardInComponent, 1.0f);
            Check(normal == PxVec3(0, 1, 0) && (soleForward - expected).magnitude() < 1e-5f,
                "Each landing pose uses the moving foot's standing heading and the candidate stride");
            checked.push_back(point);
            checkedHeadings.push_back(soleForward);
            FootstepPoseCheck check;
            check.joints[2].correctionRadians = .82f;
            if (checked.size() != 2)
            {
                check.failures = FootstepPoseFailure::Reach;
            }
            return check;
        };
        planner.Reset();
        planner.Update(input, settings);
        const FootstepTarget& second = planner.GetSnapshot().candidates[1];
        Check(planner.GetSnapshot().feet[1].justStarted && second.recovery.mode == FootstepMode::Normal &&
            second.recovery.intersections[0].size() == 1 && !InAny(second.recovery.intersections[0], second.raw) &&
            checked.size() == 2, "a wider stance moves the reference outside the overlap and a second pose is accepted");
        Check(second.recovery.poseChecks == checked.size() &&
            std::equal(checked.begin(), checked.end(), second.recovery.poseCheckPositions.begin()),
            "the snapshot retains the checked positions through the first accepted landing");
        Check(second.soleForward == checkedHeadings.back() && planner.GetSnapshot().feet[1].target.soleForward == second.soleForward,
            "The drive command keeps the exact heading accepted by the landing validator");
        Check(second.recovery.poseCheckResults[0].failures == FootstepPoseFailure::Reach &&
            second.recovery.poseCheckResults[1].failures == FootstepPoseFailure::None &&
            second.recovery.poseCheckResults[1].joints[2].correctionRadians == .82f,
            "the planner starts the reachable step with its measured ankle correction");
        const PxVec3 closest = ClosestFootstepPoint(second.recovery.intersections[0].front(), second.raw);
        Check((checked.front() - closest).magnitudeSquared() < 1e-8f && second.position == checked.back() &&
            InAny(second.recovery.intersections[0], second.position),
            "an outside stance target tries the closest overlap point first and keeps the alternative inside");

        input.velocity.z = -4.0f;
        input.checkLandingPose = [](size_t, const PxVec3&, const PxVec3&, const PxVec3&) -> FootstepPoseFailures
        {
            return FootstepPoseFailure::Reach;
        };
        planner.Reset();
        planner.Update(input, settings);
        const FootstepSnapshot& rejected = planner.GetSnapshot();
        Check(rejected.needsStep && rejected.candidates[1].recovery.status == FootstepRecoveryStatus::PoseRejected &&
            !rejected.candidates[1].valid && rejected.startCount == 0,
            "rejecting every region candidate cannot start a step at the idle reference");
    }

    void TestStepModeLock()
    {
        FootstepInput input = PlannerInput();
        FootstepSettings settings;
        settings.recovery.maximumSteps = 1;
        FootstepPlanner planner;
        const FootstepCommand& command = planner.GetSnapshot().feet[1];
        const FootstepRecoverySnapshot& search = planner.GetSnapshot().candidates[1].recovery;

        input.com.z = 0;
        input.velocity = PxVec3(0, 0, .6f);
        planner.Update(input, settings);
        Check(command.justStarted && command.target.recovery.hasTarget && command.target.recovery.mode == FootstepMode::Normal,
            "the forward step starts in normal mode");
        const PxVec3 normalTarget = command.target.position;

        input.feet[1].grounded = false;
        input.com.z = .15f;
        input.velocity = PxVec3(-.8f, 0, 0);
        ++input.completedStep;
        planner.Update(input, settings);
        Check(command.phase == FootstepPhase::Airborne && !command.motion.landingTargetLocked &&
            search.overlapAreas[0] == 0 && search.overlapAreas[1] > 0 && search.hasTarget &&
            search.status == FootstepRecoveryStatus::BestEffort && command.target.recovery.mode == FootstepMode::Normal &&
            IsInsideFootSupportHull(search.reachable[0], command.target.position) && command.target.position == search.target,
            "an airborne normal step uses its own closest reachable landing when only a crossover overlaps");

        input.com.z = 0;
        input.velocity = PxVec3(0, 0, 1.1f);
        ++input.completedStep;
        planner.Update(input, settings);
        Check(search.hasTarget && command.target.recovery.mode == FootstepMode::Normal &&
            (command.target.position - normalTarget).magnitude() > .01f,
            "the same normal step can still adjust its landing position");

        input.com.z = -.15f;
        input.velocity = PxVec3(-.8f, 0, 0);
        input.feet[1].grounded = true;
        input.completedStep += static_cast<uint64_t>(std::ceil(settings.stepDuration / input.fixedDeltaTime));
        planner.Update(input, settings);
        Check(command.justLanded && command.target.recovery.mode == FootstepMode::Normal,
            "the normal step retains its mode through landing");
        ++input.completedStep;
        planner.Update(input, settings);
        Check(command.justStarted && command.target.recovery.mode == FootstepMode::BackCross,
            "the next step of the same foot can select a rear crossover");
        const PxVec3 crossTarget = command.target.position;

        input.feet[1].grounded = false;
        input.feet[1].soleVelocity.x = -10.0f;
        input.requestedFoot = 0;
        ++input.completedStep;
        planner.Update(input, settings);
        Check(command.phase == FootstepPhase::Airborne && command.target.recovery.mode == FootstepMode::BackCross &&
            command.target.position == crossTarget && command.motion.swingOutwardOffset.isZero(),
            "an airborne crossover keeps its path even when the soles converge");
    }

    void TestPlannerIntegration()
    {
        FootstepInput input = PlannerInput();
        FootstepSettings settings;
        settings.recovery.maximumSteps = 1;
        Check(!settings.pathAvoidanceEnabled, "Footstep path avoidance defaults to off");
        settings.pathAvoidanceEnabled = true;
        settings.recovery.transferTime = .025f;
        FootstepPlanner planner;
        planner.Update(input, settings);
        const FootstepCommand first = planner.GetSnapshot().feet[1];
        Check(first.justStarted && first.target.recovery.hasTarget && first.target.recovery.mode == FootstepMode::FrontCross,
            "a raw inward demand branches before normal COM and minimum-width corrections");
        Check(first.target.position.x < input.feet[0].position.x,
            "the legacy crossing clamp is not applied to the selected crossover target");
        Check(!first.target.landingCom.enabled && !first.target.landingCom.limited,
            "crossover debug metadata does not claim that the bypassed normal COM clamp was applied");
        Check(first.target.recovery.capture.empty() && !planner.GetSnapshot().candidates[1].recovery.capture.empty(),
            "prediction copies keep only decision metadata while the debug candidate keeps region geometry");
        planner.ReportRecoveryPath(1, first.target.position, false);
        ++input.completedStep;
        planner.Update(input, settings);
        const FootstepCommand replanned = planner.GetSnapshot().feet[1];
        Check((replanned.target.position - first.target.position).magnitude() >= .0099f ||
            !planner.GetSnapshot().candidates[1].recovery.hasTarget,
            "a failed path requests an alternative landing instead of repeating the same candidate");
        input.checkLandingPose = [](size_t, const PxVec3&, const PxVec3&, const PxVec3&) -> FootstepPoseFailures
        {
            return FootstepPoseFailure::OppositeLegOverlap;
        };
        ++input.completedStep;
        planner.Update(input, settings);
        Check((planner.GetSnapshot().feet[1].target.position - replanned.target.position).magnitude() < 1e-6f,
            "candidate rejection preserves an existing crossover goal instead of applying a normal clamp");
        input.checkLandingPose = [](size_t, const PxVec3&, const PxVec3&, const PxVec3&) -> FootstepPoseFailures
        {
            return FootstepPoseFailure::None;
        };
        input.feet[1].grounded = false;
        const uint32_t lockTick = static_cast<uint32_t>(std::ceil(
            settings.stepDuration * settings.landingLockProgress / input.fixedDeltaTime));
        for (uint32_t frame = 0; frame < lockTick; ++frame)
        {
            ++input.completedStep;
            planner.Update(input, settings);
        }
        const FootstepCommand& command = planner.GetSnapshot().feet[1];
        Check(!command.motion.landingTargetLocked, "the default swing stays editable until the lock tick");
        const PxVec3 locked = command.target.position;
        ++input.completedStep;
        planner.Update(input, settings);
        const FootstepRecoverySnapshot& search = planner.GetSnapshot().candidates[1].recovery;
        Check(command.motion.landingTargetLocked && command.target.position == locked &&
            search.status == FootstepRecoveryStatus::Locked && search.poseChecks == 0,
            "the lock tick keeps the crossover landing target and skips further pose checks");
        planner.ReportRecoveryPath(1, locked, false);
        Check(planner.GetSnapshot().feet[1].target.recovery.pathRejected, "The locked path records the last rejection");
        settings.pathAvoidanceEnabled = false;
        input.velocity = PxVec3(1, 0, 2);
        ++input.completedStep;
        planner.Update(input, settings);
        Check((planner.GetSnapshot().feet[1].target.position - locked).magnitude() < 1e-6f,
            "new recovery choices cannot move the locked horizontal target");
        Check(planner.GetSnapshot().feet[1].motion.valid && !planner.GetSnapshot().feet[1].target.recovery.pathRejected,
            "Disabling path avoidance clears the previous rejection while the swing continues");
        input.active = false;
        input.velocity = PxVec3(0);
        input.feet[1].grounded = true;
        ++input.completedStep;
        planner.Update(input, settings);
        Check(!planner.GetSnapshot().IsInProgress() && planner.GetSnapshot().candidates[0].recovery.status ==
            FootstepRecoveryStatus::Preview && planner.GetSnapshot().candidates[0].recovery.poseChecks == 0,
            "initial kinematic idle can draw regions without commanding a step or searching poses");
    }
}

void TestFootstepRecovery()
{
    checks = 0;
    TestPolygonGeometry();
    TestNormalReachDisplay();
    TestLandingHeadingFromComLine();
    TestNormalStepBoundary();
    TestGroundFallFoothold();
    TestModeChoice();
    TestCaptureCenterCrossing();
    TestNearestNormalLanding();
    TestCaptureAndTiming();
    TestThreeStepReachForBothFeet();
    TestOneStepCaptureMargin();
    TestFollowupAfterOneStepLanding();
    TestNormalLandingPrefersStanceTarget();
    TestStepModeLock();
    TestPlannerIntegration();
    FootstepRecoveryInput input = Input();
    input.velocity = PxVec3(-1.5f, 0, .5f);
    const FootstepRecoverySettings settings;
    const std::chrono::steady_clock::time_point started = std::chrono::steady_clock::now();
    double maximum = 0;
    for (uint32_t frame = 0; frame < 120; ++frame)
    {
        input.com.z = static_cast<float>(frame % 10) * .01f;
        const FootstepRecoverySnapshot result = SelectFootstepRecovery(input, settings);
        Check(result.capturePieceCount <= 13 && result.poseChecks <= settings.maximumPoseChecks,
            "runtime work stays within configured region and pose bounds");
        maximum = (std::max)(maximum, result.elapsedMilliseconds);
    }
    const double total = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - started).count();
    std::printf("PASS: %u recovery-region checks; 120 %u-step decisions avg %.3f ms, max %.3f ms (geometry only)\n",
        checks, settings.maximumSteps, total / 120, maximum);
}
