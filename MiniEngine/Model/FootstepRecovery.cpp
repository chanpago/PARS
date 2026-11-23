#include "FootstepRecovery.h"
#include "FramePerformance.h"
#include <algorithm>
#include <chrono>
#include <limits>
#include <utility>

using namespace PARS;
using namespace physx;

namespace
{
    constexpr uint32_t CircleVertices = 12;
    constexpr float Epsilon = 1e-6f;
    constexpr uint32_t MaximumCapturePieces = 13;

    float Cross(const PxVec3& a, const PxVec3& b)
    {
        return a.x * b.z - a.z * b.x;
    }

    void Clip(const FootstepPolygon& polygon, const PxVec3& normal, float offset, FootstepPolygon& result)
    {
        result.clear();
        if (polygon.empty())
        {
            return;
        }
        result.reserve(polygon.size() + 1);
        PxVec3 previous = polygon.back();
        float previousDistance = previous.dot(normal) - offset;
        for (const PxVec3& current : polygon)
        {
            const float distance = current.dot(normal) - offset;
            if ((distance >= 0) != (previousDistance >= 0))
            {
                result.push_back(previous + (current - previous) *
                    (previousDistance / (previousDistance - distance)));
            }
            if (distance >= 0)
            {
                result.push_back(current);
            }
            previous = current;
            previousDistance = distance;
        }
        if (result.size() < 3)
        {
            result.clear();
        }
    }

    FootstepPolygon Transform(const FootstepPolygon& polygon, float scale, const PxVec3& translation)
    {
        FootstepPolygon result;
        result.reserve(polygon.size());
        for (const PxVec3& point : polygon)
        {
            result.emplace_back(point.x * scale + translation.x, translation.y, point.z * scale + translation.z);
        }
        return result;
    }

    FootstepPolygon Sum(const FootstepPolygon& a, const FootstepPolygon& b)
    {
        if (a.size() < 3 || b.size() < 3)
        {
            return {};
        }
        size_t startA = 0;
        size_t startB = 0;
        for (size_t index = 1; index < a.size(); ++index)
        {
            if (a[index].z < a[startA].z || (a[index].z == a[startA].z && a[index].x < a[startA].x))
            {
                startA = index;
            }
        }
        for (size_t index = 1; index < b.size(); ++index)
        {
            if (b[index].z < b[startB].z || (b[index].z == b[startB].z && b[index].x < b[startB].x))
            {
                startB = index;
            }
        }
        FootstepPolygon result;
        result.reserve(a.size() + b.size());
        size_t first = 0;
        size_t second = 0;
        while (first < a.size() || second < b.size())
        {
            const PxVec3 point = a[(startA + first) % a.size()] + b[(startB + second) % b.size()];
            result.emplace_back(point.x, a[0].y, point.z);
            const PxVec3 edgeA = a[(startA + first + 1) % a.size()] - a[(startA + first) % a.size()];
            const PxVec3 edgeB = b[(startB + second + 1) % b.size()] - b[(startB + second) % b.size()];
            const float turn = Cross(edgeA, edgeB);
            const bool advanceA = second == b.size() || (first < a.size() && turn >= -1e-8f);
            const bool advanceB = first == a.size() || (second < b.size() && turn <= 1e-8f);
            if (advanceA)
            {
                ++first;
            }
            if (advanceB)
            {
                ++second;
            }
        }
        return result;
    }

    void RemoveContained(std::vector<FootstepPolygon>& pieces)
    {
        std::vector<FootstepPolygon> result;
        for (size_t source = 0; source < pieces.size(); ++source)
        {
            bool contained = false;
            for (size_t other = 0; other < pieces.size() && !contained; ++other)
            {
                if (source == other || pieces[other].empty() || pieces[source].empty())
                {
                    continue;
                }
                bool inside = true;
                for (const PxVec3& point : pieces[source])
                {
                    inside = inside && IsInsideFootSupportHull(pieces[other], point);
                }
                if (inside)
                {
                    bool equal = true;
                    for (const PxVec3& point : pieces[other])
                    {
                        equal = equal && IsInsideFootSupportHull(pieces[source], point);
                    }
                    contained = !equal || other < source;
                }
            }
            if (!contained && !pieces[source].empty())
            {
                result.push_back(pieces[source]);
            }
        }
        pieces = std::move(result);
    }

    FootstepPolygon Circle(const PxVec3& center, float radius)
    {
        FootstepPolygon result;
        if (!std::isfinite(radius) || radius <= Epsilon)
        {
            return result;
        }
        result.reserve(CircleVertices);
        for (uint32_t index = 0; index < CircleVertices; ++index)
        {
            const float angle = PxTwoPi * static_cast<float>(index) / CircleVertices;
            result.emplace_back(center.x + radius * std::cos(angle), center.y, center.z + radius * std::sin(angle));
        }
        return result;
    }

    FootstepNormalReach MakeNormalReach(const FootstepRecoveryInput& input,
        const FootstepRecoverySettings& settings, size_t side)
    {
        FootstepNormalReach result;
        const FootstepReachInput& foot = input.feet[side];
        const FootstepReachInput& support = input.feet[1 - side];
        if (!foot.available || !support.available || foot.sole.size() < 3 || support.sole.size() < 3 ||
            !foot.hip.isFinite() || !foot.ankleOffset.isFinite() || !foot.outward.isFinite() ||
            std::abs(foot.outward.magnitudeSquared() - 1.0f) > .01f ||
            !std::isfinite(foot.legLength) || foot.legLength <= 0 ||
            !std::isfinite(foot.minimumWidth) || foot.minimumWidth < 0)
        {
            return result;
        }
        const float height = foot.hip.y - (input.groundHeight + foot.ankleOffset.y);
        const float length = foot.legLength * .9999f;
        const float radiusSquared = length * length - height * height;
        if (!std::isfinite(radiusSquared) || radiusSquared <= 0)
        {
            return result;
        }
        const float physicalWidth = FootstepSoleHalfExtent(foot.sole, foot.position, foot.outward) +
            FootstepSoleHalfExtent(support.sole, support.position, foot.outward) + settings.clearance;
        result.center = PxVec3(foot.hip.x - foot.ankleOffset.x, input.groundHeight, foot.hip.z - foot.ankleOffset.z);
        result.radius = std::sqrt(radiusSquared);
        result.outward = foot.outward;
        result.minimumWidth = (std::max)(foot.minimumWidth, physicalWidth);
        result.minimumProjection = support.position.dot(foot.outward) + result.minimumWidth;
        result.combinedHalfLength = FootstepSoleHalfExtent(foot.sole, foot.position, input.forward) +
            FootstepSoleHalfExtent(support.sole, support.position, input.forward);
        return result;
    }

    std::array<FootstepPolygon, 3> Reach(const FootstepRecoveryInput& input,
        const FootstepRecoverySettings& settings, size_t side, const FootstepNormalReach& normal)
    {
        std::array<FootstepPolygon, 3> result;
        const FootstepReachInput& foot = input.feet[side];
        const FootstepReachInput& support = input.feet[1 - side];
        if (normal.radius <= 0 || !std::isfinite(foot.maximumDistance) || foot.maximumDistance <= 0)
        {
            return result;
        }
        const FootstepPolygon reachable = IntersectFootstepPolygons(Circle(normal.center, normal.radius),
            Circle(PxVec3(foot.stepOrigin.x, input.groundHeight, foot.stepOrigin.z), foot.maximumDistance));
        Clip(reachable, normal.outward, normal.minimumProjection, result[0]);
        const float lengthClearance = normal.combinedHalfLength + settings.clearance;
        const float angles[] = {settings.frontAngleDegrees, settings.backAngleDegrees};
        for (size_t cross = 0; cross < 2; ++cross)
        {
            const PxVec3 forward = input.forward * (cross == 0 ? 1.0f : -1.0f);
            const PxVec3 boundary = forward + foot.outward * std::tan(angles[cross] * PxPi / 180.0f);
            // 발 사각형과 대각선 경계로 지지발 근처를 제외하고, 실제 PA 관절은 후보에서 검사한다.
            Clip(reachable, boundary, support.position.dot(boundary) + lengthClearance, result[cross + 1]);
        }
        return result;
    }

    bool Contains(const std::vector<FootstepPolygon>& pieces, const PxVec3& point)
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

    float DistanceSquared(const PxVec3& a, const PxVec3& b)
    {
        const PxVec3 delta = a - b;
        return delta.x * delta.x + delta.z * delta.z;
    }

    PxVec3 PolygonCenter(const FootstepPolygon& polygon)
    {
        const PxVec3 origin = polygon.front();
        PxVec3 offset(0);
        for (const PxVec3& point : polygon)
        {
            offset += point - origin;
        }
        return origin + offset / static_cast<float>(polygon.size());
    }

    void ExpandOneStepCapture(FootstepPolygon& polygon, float scale)
    {
        // 볼록 다각형의 꼭짓점 평균을 기준으로 수평 치수만 늘린다.
        const PxVec3 center = PolygonCenter(polygon);
        for (PxVec3& point : polygon)
        {
            point += (point - center) * (scale - 1.0f);
        }
    }

    bool EvaluateIntersections(FootstepRecoverySnapshot& result)
    {
        for (size_t mode = 0; mode < 3; ++mode)
        {
            result.intersections[mode].clear();
            for (const FootstepPolygon& capture : result.capture)
            {
                FootstepPolygon intersection = IntersectFootstepPolygons(capture, result.reachable[mode]);
                if (FootstepPolygonArea(intersection) > 1e-9f)
                {
                    result.intersections[mode].push_back(std::move(intersection));
                }
            }
            result.overlapAreas[mode] = FootstepUnionArea(result.intersections[mode]);
        }
        return result.overlapAreas[0] > 0 || result.overlapAreas[1] > 0 || result.overlapAreas[2] > 0;
    }

    bool CaptureReachesNormalSide(const FootstepRecoverySnapshot& result)
    {
        for (const FootstepPolygon& piece : result.capture)
        {
            for (const PxVec3& point : piece)
            {
                if (point.dot(result.normalReach.outward) >= result.normalReach.minimumProjection)
                {
                    return true;
                }
            }
        }
        return false;
    }

    struct LandingCandidate
    {
    public:
        bool operator<(const LandingCandidate& other) const
        {
            return score < other.score;
        }

    public:
        size_t mode = 0;
        PxVec3 position{0};
        float score = 0;
    };

    void AddBestEffortCandidates(const FootstepPolygon& reach, const std::vector<FootstepPolygon>& capture,
        size_t mode, std::vector<LandingCandidate>& candidates)
    {
        if (reach.size() < 3 || capture.empty())
        {
            return;
        }
        // 분리된 두 볼록 조각의 최단점은 한쪽 꼭짓점과 반대쪽 변 사이에 있다.
        FootstepPolygon points = reach;
        points.push_back(PolygonCenter(reach));
        for (const FootstepPolygon& piece : capture)
        {
            for (const PxVec3& vertex : piece)
            {
                const PxVec3 projected = ClosestFootstepPoint(reach, vertex);
                if (std::find(points.begin(), points.end(), projected) == points.end())
                {
                    points.push_back(projected);
                }
            }
        }
        for (const PxVec3& point : points)
        {
            float gapSquared = std::numeric_limits<float>::max();
            for (const FootstepPolygon& piece : capture)
            {
                const PxVec3 closest = ClosestFootstepPoint(piece, point);
                gapSquared = (std::min)(gapSquared, DistanceSquared(point, closest));
            }
            candidates.push_back({mode, point, gapSquared});
        }
    }

    void AddAlternativePoseCandidates(const FootstepPolygon& piece, size_t mode, const PxVec3& preferred,
        std::vector<LandingCandidate>& candidates)
    {
        const PxVec3 center = PolygonCenter(piece);
        candidates.push_back({mode, center, DistanceSquared(center, preferred)});
        for (const PxVec3& vertex : piece)
        {
            candidates.push_back({mode, vertex, DistanceSquared(vertex, preferred)});
        }
    }

    void SelectTarget(const FootstepRecoveryInput& input, const FootstepRecoverySettings& settings,
        bool hasOverlap, bool allowGroundedSwitch, FootstepPolygon& tested, FootstepRecoverySnapshot& result)
    {
        std::vector<LandingCandidate> candidates;
        std::array<size_t, 3> order{{0, 1, 2}};
        if (result.overlapAreas[2] > result.overlapAreas[1])
        {
            std::swap(order[1], order[2]);
        }
        const size_t previous = static_cast<size_t>(input.previousMode);
        if (previous > 0 && previous < 3 && result.overlapAreas[previous] > 0 &&
            result.overlapAreas[previous] * (1 + settings.switchAreaRatio) >= result.overlapAreas[order[1]])
        {
            order[1] = previous;
            order[2] = 3 - previous;
        }
        // 일반 방향의 회복 영역이 보폭 밖에 있으면 일반 도달 영역 안에서 최선점을 찾는다.
        const bool normalOutOfRange = !hasOverlap && input.previousMode == FootstepMode::Normal &&
            !result.reachable[0].empty() && CaptureReachesNormalSide(result);
        const bool checkAlternatives = input.acceptsTarget || input.hasRejectedTarget;
        for (size_t mode : order)
        {
            if ((normalOutOfRange && mode != 0) ||
                (mode != previous && input.modeSelection != FootstepRecoveryInput::ModeSelection::NewStep && !allowGroundedSwitch))
            {
                continue;
            }
            const size_t first = candidates.size();
            if (!hasOverlap)
            {
                AddBestEffortCandidates(result.reachable[mode], result.capture, mode, candidates);
                if (mode != 0 && candidates.size() > first)
                {
                    // 이미 교차가 필요한 방향에서는 기존 교차 목표와 대체 자세의 순서를 유지한다.
                    const LandingCandidate nearest = *std::min_element(candidates.begin() + first, candidates.end());
                    candidates.resize(first);
                    candidates.push_back(nearest);
                    if (checkAlternatives)
                    {
                        AddAlternativePoseCandidates(result.reachable[mode], mode, input.preferred, candidates);
                    }
                }
                continue;
            }
            // 일반 딛기는 Idle 발 배치와 보폭을 반영한 목표에 가까운 교집합 후보부터 검사한다.
            const PxVec3& preferred = mode == 0 && input.normalTarget.isFinite() ? input.normalTarget : input.preferred;
            for (const FootstepPolygon& piece : result.intersections[mode])
            {
                const PxVec3 closest = ClosestFootstepPoint(piece, preferred);
                candidates.push_back({mode, closest, DistanceSquared(closest, preferred)});
                if (checkAlternatives)
                {
                    AddAlternativePoseCandidates(piece, mode, preferred, candidates);
                }
            }
            std::stable_sort(candidates.begin() + first, candidates.end());
        }
        if (!hasOverlap)
        {
            std::stable_sort(candidates.begin(), candidates.end());
        }
        if (input.backwardFall && input.previousMode == FootstepMode::Normal && !result.reachable[0].empty())
        {
            // 이미 넘어지는 중에는 골반 아래에서 짧게 짚을 수 있는 자세를 같은 검사 예산 안에서 먼저 찾는다.
            const FootstepPolygon& reach = result.reachable[0];
            const PxVec3 underHip = ClosestFootstepPoint(reach, result.normalReach.center);
            const PxVec3 towardFall = ClosestFootstepPoint(reach, input.preferred);
            const PxVec3 underFoot = ClosestFootstepPoint(reach, input.feet[input.movingFoot].stepOrigin);
            candidates.insert(candidates.begin(), {{0, (underHip + towardFall) * .5f, 0}, {0, underHip, 0}, {0, underFoot, 0}});
        }
        for (const LandingCandidate& candidate : candidates)
        {
            if (input.hasRejectedTarget && DistanceSquared(candidate.position, input.rejectedTarget) < .0001f)
            {
                continue;
            }
            bool duplicate = false;
            for (const PxVec3& point : tested)
            {
                duplicate = duplicate || DistanceSquared(point, candidate.position) < 1e-8f;
            }
            if (duplicate)
            {
                continue;
            }
            if (result.poseChecks >= settings.maximumPoseChecks)
            {
                result.budgetExhausted = true;
                break;
            }
            tested.push_back(candidate.position);
            ++result.poseChecks;
            if (input.acceptsTarget && !input.acceptsTarget(candidate.position))
            {
                result.poseRejected = true;
                continue;
            }
            result.target = candidate.position;
            result.mode = static_cast<FootstepMode>(candidate.mode);
            result.hasTarget = true;
            result.modelFeasible = hasOverlap && (!input.backwardFall || Contains(result.intersections[candidate.mode], candidate.position));
            result.status = result.modelFeasible ? FootstepRecoveryStatus::Feasible : FootstepRecoveryStatus::BestEffort;
            return;
        }
        result.status = result.poseRejected ? FootstepRecoveryStatus::PoseRejected : FootstepRecoveryStatus::NoReach;
    }
}

FootstepPolygon FootstepNormalReach::BuildPolygon() const
{
    FootstepPolygon polygon;
    Clip(Circle(center, radius), outward, minimumProjection, polygon);
    return polygon;
}

FootstepPolygon PARS::IntersectFootstepPolygons(const FootstepPolygon& first, const FootstepPolygon& second)
{
    if (first.empty() || second.size() < 3)
    {
        return {};
    }
    // 볼록 다각형은 경계 하나로 자를 때 꼭짓점이 최대 하나 늘어난다.
    const size_t vertexCapacity = first.size() + second.size();
    FootstepPolygon result;
    result.reserve(vertexCapacity);
    result.assign(first.begin(), first.end());
    FootstepPolygon clipped;
    clipped.reserve(vertexCapacity);
    for (size_t index = 0; index < second.size() && !result.empty(); ++index)
    {
        const PxVec3& a = second[index];
        const PxVec3 edge = second[(index + 1) % second.size()] - a;
        const PxVec3 inward(-edge.z, 0, edge.x);
        Clip(result, inward, a.dot(inward), clipped);
        result.swap(clipped);
    }
    return result;
}

float PARS::FootstepPolygonArea(const FootstepPolygon& polygon)
{
    double twiceArea = 0;
    for (size_t index = 1; index + 1 < polygon.size(); ++index)
    {
        twiceArea += Cross(polygon[index] - polygon[0], polygon[index + 1] - polygon[0]);
    }
    return static_cast<float>(std::abs(twiceArea) * .5);
}

PxVec3 PARS::ClosestFootstepPoint(const FootstepPolygon& polygon, const PxVec3& point)
{
    if (polygon.empty())
    {
        return point;
    }
    if (IsInsideFootSupportHull(polygon, point))
    {
        return PxVec3(point.x, polygon[0].y, point.z);
    }
    PxVec3 result = polygon[0];
    float best = std::numeric_limits<float>::max();
    for (size_t index = 0; index < polygon.size(); ++index)
    {
        const PxVec3& a = polygon[index];
        PxVec3 edge = polygon[(index + 1) % polygon.size()] - a;
        edge.y = 0;
        const float length = edge.magnitudeSquared();
        const float t = length > Epsilon * Epsilon ? PxClamp((point - a).dot(edge) / length, 0.0f, 1.0f) : 0;
        const PxVec3 closest = a + edge * t;
        const float distance = DistanceSquared(closest, point);
        if (distance < best)
        {
            best = distance;
            result = closest;
        }
    }
    return result;
}

float PARS::FootstepUnionArea(const std::vector<FootstepPolygon>& polygons)
{
    if (polygons.size() == 1)
    {
        return FootstepPolygonArea(polygons.front());
    }
    // 각 다각형의 경계 중 다른 조각 내부에 가려지지 않은 구간만 적분한다.
    // 수직 단면마다 모든 변을 재검색하는 방식보다 비용의 증가를 제한할 수 있다.
    double area2 = 0;
    PxVec3 origin(0);
    for (const FootstepPolygon& polygon : polygons)
    {
        if (!polygon.empty())
        {
            origin = polygon.front();
            break;
        }
    }
    for (size_t source = 0; source < polygons.size(); ++source)
    {
        const FootstepPolygon& polygon = polygons[source];
        if (polygon.size() < 3)
        {
            continue;
        }
        for (size_t index = 0; index < polygon.size(); ++index)
        {
            const PxVec3& a = polygon[index];
            const PxVec3 edge = polygon[(index + 1) % polygon.size()] - a;
            std::vector<std::pair<float, float>> covered;
            for (size_t other = 0; other < polygons.size(); ++other)
            {
                if (other == source || polygons[other].size() < 3)
                {
                    continue;
                }
                float low = 0;
                float high = 1;
                const FootstepPolygon& clip = polygons[other];
                for (size_t boundary = 0; boundary < clip.size() && low < high; ++boundary)
                {
                    const PxVec3& start = clip[boundary];
                    const PxVec3 direction = clip[(boundary + 1) % clip.size()] - start;
                    const float distance = Cross(direction, a - start);
                    const float slope = Cross(direction, edge);
                    if (std::abs(slope) < 1e-9f)
                    {
                        if (distance < -1e-7f || (std::abs(distance) < 1e-7f && direction.dot(edge) > 0 && other > source))
                        {
                            high = low;
                        }
                    }
                    else if (slope > 0)
                    {
                        low = (std::max)(low, -distance / slope);
                    }
                    else
                    {
                        high = (std::min)(high, -distance / slope);
                    }
                }
                if (low < high)
                {
                    covered.emplace_back(PxClamp(low, 0.0f, 1.0f), PxClamp(high, 0.0f, 1.0f));
                }
            }
            std::sort(covered.begin(), covered.end());
            float visible = 1;
            float end = 0;
            for (const std::pair<float, float>& interval : covered)
            {
                if (interval.second > end)
                {
                    visible -= interval.second - (std::max)(end, interval.first);
                    end = interval.second;
                }
            }
            area2 += Cross(a - origin, edge) * (std::max)(0.0f, visible);
        }
    }
    return static_cast<float>((std::max)(0.0, area2 * .5));
}

FootstepRecoverySnapshot PARS::SelectFootstepRecovery(const FootstepRecoveryInput& input,
    const FootstepRecoverySettings& settings)
{
    FramePerformance::Scope landingTiming(FramePerformance::Category::LandingTargetSearch);
    const std::chrono::steady_clock::time_point started = std::chrono::steady_clock::now();
    FootstepRecoverySnapshot result;
    result.preferred = input.preferred;
    if (!settings.enabled || input.movingFoot > 1 || !std::isfinite(input.groundHeight) ||
        !std::isfinite(settings.clearance) || settings.clearance < 0)
    {
        return result;
    }
    const size_t side = input.movingFoot;
    for (const FootstepReachInput& foot : input.feet)
    {
        if (foot.sole.size() < 3 || foot.sole.size() > 8 || !foot.position.isFinite())
        {
            return result;
        }
        for (const PxVec3& corner : foot.sole)
        {
            if (!corner.isFinite())
            {
                return result;
            }
        }
    }
    result.normalReach = MakeNormalReach(input, settings, side);
    const float height = input.com.y - input.groundHeight;
    if (!input.com.isFinite() || !input.velocity.isFinite() || !std::isfinite(height) || height < .05f ||
        !std::isfinite(input.firstStepTime) || input.firstStepTime <= 0 ||
        !std::isfinite(input.followingStepTime) || input.followingStepTime <= 0 ||
        settings.maximumSteps < 1 || settings.maximumSteps > 3 || settings.maximumPoseChecks == 0 || settings.maximumPoseChecks > 6 ||
        !std::isfinite(settings.frontAngleDegrees) || settings.frontAngleDegrees < 0 || settings.frontAngleDegrees >= 80 ||
        !std::isfinite(settings.backAngleDegrees) || settings.backAngleDegrees < 0 || settings.backAngleDegrees >= 80 ||
        !std::isfinite(settings.switchAreaRatio) || settings.switchAreaRatio < 0 ||
        (settings.maximumSteps == 1 && (!std::isfinite(settings.oneStepCaptureScale) || settings.oneStepCaptureScale < 1.0f)) ||
        !input.forward.isFinite() || std::abs(input.forward.magnitudeSquared() - 1) > .01f ||
        !input.preferred.isFinite() || !input.lockedTarget.isFinite() ||
        static_cast<size_t>(input.previousMode) > 2)
    {
        return result;
    }
    const float omega = std::sqrt(9.81f / height);
    const float exponent = omega * input.firstStepTime;
    if (exponent > 12 || input.feet[1 - side].sole.size() < 3)
    {
        return result;
    }
    const float growth = std::exp(exponent);
    const float discount = std::exp(-omega * input.followingStepTime);
    result.icp = PxVec3(input.com.x + input.velocity.x / omega, input.groundHeight, input.com.z + input.velocity.z / omega);
    result.predictionTime = input.firstStepTime;
    const uint32_t maximum = input.stepping ? settings.maximumSteps : 1;
    std::array<std::array<FootstepPolygon, 3>, 2> reaches;
    reaches[side] = Reach(input, settings, side, result.normalReach);
    if (maximum > 1)
    {
        const size_t other = 1 - side;
        reaches[other] = Reach(input, settings, other, MakeNormalReach(input, settings, other));
    }
    result.reachable = reaches[side];
    std::array<FootstepPolygon, 2> soles;
    std::array<std::vector<FootstepPolygon>, 2> errors;
    for (size_t foot = 0; foot < 2; ++foot)
    {
        soles[foot] = Transform(input.feet[foot].sole, 1, -input.feet[foot].position);
        errors[foot].push_back(soles[foot]);
    }
    // 고정 착지 시각의 ICP 집합이다. 압력 중심을 지지발 안에서 선택하는 LIP 모델을 사용한다.
    const FootstepPolygon future = Transform(input.feet[1 - side].sole, 1 - growth,
        PxVec3(growth * result.icp.x, input.groundHeight, growth * result.icp.z));
    std::array<std::vector<FootstepPolygon>, 3> captureBySteps;
    for (uint32_t steps = 1; steps <= maximum; ++steps)
    {
        result.steps = steps;
        for (const FootstepPolygon& error : errors[side])
        {
            captureBySteps[steps - 1].push_back(Sum(future,
                Transform(error, -1, PxVec3(0, input.groundHeight, 0))));
        }
        if (steps == maximum)
        {
            result.capture = captureBySteps[steps - 1];
            break;
        }
        std::array<std::vector<FootstepPolygon>, 2> nextErrors;
        for (size_t stance = 0; stance < 2; ++stance)
        {
            // 마지막 전개에서는 이번 이동발의 회복 영역만 읽는다.
            if (steps + 1 == maximum && stance != side)
            {
                continue;
            }
            nextErrors[stance].push_back(soles[stance]);
            const size_t next = 1 - stance;
            const FootstepPolygon pressure = Transform(soles[stance], 1 - discount, PxVec3(0));
            for (const FootstepPolygon& reachable : reaches[next])
            {
                if (reachable.empty())
                {
                    continue;
                }
                const FootstepPolygon displacement = Transform(reachable, 1, -input.feet[stance].position);
                for (const FootstepPolygon& remainingError : errors[next])
                {
                    nextErrors[stance].push_back(Sum(Transform(Sum(displacement, remainingError), discount, PxVec3(0)), pressure));
                }
            }
            RemoveContained(nextErrors[stance]);
        }
        errors = std::move(nextErrors);
    }
    RemoveContained(result.capture);
    result.capturePieceCount = static_cast<uint32_t>(result.capture.size());
    result.usesFutureReachApproximation = result.steps > 1;
    if (result.capturePieceCount > MaximumCapturePieces)
    {
        result.status = FootstepRecoveryStatus::Unavailable;
        result.budgetExhausted = true;
        return result;
    }
    bool hasOverlap = EvaluateIntersections(result);
    result.modelFeasible = hasOverlap;
    const bool allowGroundedSwitch = input.modeSelection == FootstepRecoveryInput::ModeSelection::GroundedNormal &&
        input.previousMode == FootstepMode::Normal && (result.overlapAreas[1] > 0 || result.overlapAreas[2] > 0);
    const bool expand = settings.maximumSteps == 1 && settings.oneStepCaptureScale > 1.0f &&
        !result.capture.empty() && result.capture.front().size() >= 3;
    FootstepPolygon tested;
    if (expand)
    {
        // 원래 영역의 자세 후보를 먼저 검사한다. 확장 영역도 같은 틱의 검사 예산을 공유한다.
        if (input.stepping && !input.locked && hasOverlap)
        {
            SelectTarget(input, settings, hasOverlap, allowGroundedSwitch, tested, result);
        }
        ExpandOneStepCapture(result.capture.front(), settings.oneStepCaptureScale);
        hasOverlap = EvaluateIntersections(result);
    }
    if (!input.stepping)
    {
        result.status = FootstepRecoveryStatus::Preview;
    }
    else if (input.locked)
    {
        result.mode = input.previousMode;
        result.target = input.lockedTarget;
        result.hasTarget = true;
        result.modelFeasible = Contains(result.intersections[static_cast<size_t>(result.mode)], result.target);
        result.status = FootstepRecoveryStatus::Locked;
    }
    else if (!result.hasTarget)
    {
        // 1보 확장 여유분이나 모델 밖의 최선 시도로는 진행 중인 일반 딛기를 바꾸지 않는다.
        const bool allowSwitch = allowGroundedSwitch && !expand;
        const bool keepNormal = input.previousMode == FootstepMode::Normal &&
            input.modeSelection != FootstepRecoveryInput::ModeSelection::NewStep && !allowSwitch;
        const bool usableOverlap = keepNormal ? !result.intersections[0].empty() : hasOverlap;
        SelectTarget(input, settings, usableOverlap, allowSwitch, tested, result);
    }
    if (result.hasTarget && result.modelFeasible)
    {
        for (uint32_t steps = 1; steps <= maximum; ++steps)
        {
            if (Contains(captureBySteps[steps - 1], result.target))
            {
                result.requiredSteps = steps;
                break;
            }
        }
        if (expand && result.requiredSteps == 0)
        {
            result.modelFeasible = false;
            result.targetInCaptureMargin = true;
            if (!input.locked)
            {
                result.status = FootstepRecoveryStatus::BestEffort;
            }
        }
    }
    result.elapsedMilliseconds = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - started).count();
    return result;
}

const char* PARS::FootstepModeLabel(FootstepMode mode)
{
    switch (mode)
    {
    case FootstepMode::Normal:
        return u8"일반";
    case FootstepMode::FrontCross:
        return u8"앞 교차";
    case FootstepMode::BackCross:
        return u8"뒤 교차";
    }
    return u8"알 수 없음";
}

const char* PARS::FootstepRecoveryStatusLabel(FootstepRecoveryStatus status)
{
    switch (status)
    {
    case FootstepRecoveryStatus::Unavailable:
        return u8"입력 미확인";
    case FootstepRecoveryStatus::Preview:
        return u8"대기 미리보기";
    case FootstepRecoveryStatus::Feasible:
        return u8"모델상 회복 후보";
    case FootstepRecoveryStatus::BestEffort:
        return u8"모델 영역 밖 / 최선 시도";
    case FootstepRecoveryStatus::NoReach:
        return u8"도달 영역 없음";
    case FootstepRecoveryStatus::PoseRejected:
        return u8"착지 자세 검사 실패";
    case FootstepRecoveryStatus::Locked:
        return u8"착지 목표 고정";
    }
    return u8"알 수 없음";
}
