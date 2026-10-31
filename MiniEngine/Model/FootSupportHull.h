#pragma once

#include <foundation/PxVec3.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <vector>

namespace PARS
{
    // XZ convex hull, counterclockwise without a repeated closing vertex.
    // Keep each hit's Y coordinate for drawing on the actual surface.
    inline std::vector<physx::PxVec3> BuildFootSupportHull(std::vector<physx::PxVec3> points)
    {
        for (const auto& point : points)
        {
            if (!point.isFinite())
            {
                return {};
            }
        }
        std::sort(points.begin(), points.end(), [](const auto& a, const auto& b)
        {
            if (a.x != b.x)
            {
                return a.x < b.x;
            }
            if (a.z != b.z)
            {
                return a.z < b.z;
            }
            return a.y < b.y;
        });
        points.erase(std::unique(points.begin(), points.end(), [](const auto& a, const auto& b)
        {
            return a.x == b.x && a.z == b.z;
        }),
            points.end());
        if (points.size() < 3)
        {
            return {};
        }

        const auto turn = [](const auto& a, const auto& b, const auto& c)
        {
            return (double(b.x) - a.x) * (double(c.z) - a.z) -
                (double(b.z) - a.z) * (double(c.x) - a.x);
        };
        std::vector<physx::PxVec3> hull;
        hull.reserve(points.size() * 2);
        for (const auto& point : points)
        {
            while (hull.size() >= 2 && turn(hull[hull.size() - 2], hull.back(), point) <= 0)
            {
                hull.pop_back();
            }
            hull.push_back(point);
        }
        const size_t upperStart = hull.size() + 1;
        for (size_t i = points.size() - 1; i-- > 0;)
        {
            while (hull.size() >= upperStart && turn(hull[hull.size() - 2], hull.back(), points[i]) <= 0)
            {
                hull.pop_back();
            }
            hull.push_back(points[i]);
        }
        hull.pop_back();
        if (hull.size() < 3)
        {
            hull.clear();
        }
        return hull;
    }

    // A point on an edge counts as inside. Y does not affect the XZ test.
    inline bool IsInsideFootSupportHull(const std::vector<physx::PxVec3>& hull, const physx::PxVec3& point)
    {
        if (hull.size() < 3 || !point.isFinite())
        {
            return false;
        }
        for (size_t i = 0; i < hull.size(); ++i)
        {
            const auto& a = hull[i];
            const auto& b = hull[(i + 1) % hull.size()];
            const double dx = double(b.x) - a.x, dz = double(b.z) - a.z;
            const double cross = dx * (double(point.z) - a.z) - dz * (double(point.x) - a.x);
            if (cross < -1e-6 * std::sqrt(dx * dx + dz * dz))
            {
                return false;
            }
        }
        return true;
    }

    enum class FootSupportZone
    {
        Invalid,
        Inside,
        Buffer,
        Outside
    };

    struct FootSupportBalance
    {
        std::vector<physx::PxVec3> bufferHull;
        physx::PxVec3 center{0};
        float weight = 0;
        float comDistance = 0, innerDistance = 0, outerDistance = 0;
        bool hasDirection = false;
        FootSupportZone zone = FootSupportZone::Invalid;
    };

    struct FootSupportCandidates
    {
        int support = -1, step = -1;
        std::array<physx::PxVec3, 2> centers{physx::PxVec3(0), physx::PxVec3(0)};
        std::array<float, 2> distances{};
    };

    // 현재 COM이 A 내부 또는 A~B 사이에 있으면 지지발·움직일 발 후보를 고른다.
    // zone은 호출자가 같은 COM과 현재 B 크기로 계산한 판정이다. B 경계·바깥과 무효 영역은 제외한다.
    // 후보는 COM과 발 중심의 수평 거리로 정한다. 실제 접촉과 착지 가능 여부는 플래너가 별도로 검사한다.
    inline FootSupportCandidates EvaluateFootSupportCandidates(const std::vector<physx::PxVec3>& hull,
        const physx::PxVec3& com, const std::array<std::vector<physx::PxVec3>, 2>& feet,
        FootSupportZone zone, int previous = -1)
    {
        FootSupportCandidates result;
        // A를 벗어났다는 이유만으로 회복 발딛기 후보를 없애지 않는다.
        if (!com.isFinite() || hull.size() < 3 ||
            (zone != FootSupportZone::Inside && zone != FootSupportZone::Buffer))
        {
            return result;
        }
        for (size_t side = 0; side < feet.size(); ++side)
        {
            if (BuildFootSupportHull(feet[side]).size() < 3)
            {
                return {};
            }
            for (const physx::PxVec3& point : feet[side])
            {
                result.centers[side] += point;
            }
            result.centers[side] /= float(feet[side].size());
            const physx::PxVec3 delta = com - result.centers[side];
            result.distances[side] = std::sqrt(delta.x * delta.x + delta.z * delta.z);
        }
        // COM에 가까운 발을 지지발 후보로, 반대 발을 움직일 발 후보로 정한다.
        result.support = result.distances[0] <= result.distances[1] ? 0 : 1;
        // 거리 차이가 1cm 이내면 이전 후보를 유지한다. 처음 동률이면 왼발을 선택한다.
        if (previous >= 0 && previous < 2 &&
            result.distances[previous] <= result.distances[1 - previous] + .01f)
        {
            result.support = previous;
        }
        result.step = 1 - result.support;
        return result;
    }

    // A is a CCW convex hull from BuildFootSupportHull. B scales XZ distances
    // about its AREA centroid; a scale of 2 gives four times the area.
    inline FootSupportBalance EvaluateFootSupportBalance(const std::vector<physx::PxVec3>& hull,
        const physx::PxVec3& point, float scale = 2.0f)
    {
        FootSupportBalance result;
        if (hull.size() < 3 || !std::isfinite(scale) || scale <= 1.0f)
        {
            return result;
        }
        for (const auto& vertex : hull)
        {
            if (!vertex.isFinite())
            {
                return result;
            }
        }

        // Translate before the shoelace sum to avoid cancellation far from the origin.
        const auto& origin = hull[0];
        double area2 = 0, cx = 0, cz = 0, height = 0;
        for (size_t i = 0; i < hull.size(); ++i)
        {
            const auto& a = hull[i];
            const auto& b = hull[(i + 1) % hull.size()];
            const double ax = double(a.x) - origin.x, az = double(a.z) - origin.z;
            const double bx = double(b.x) - origin.x, bz = double(b.z) - origin.z;
            const double cross = ax * bz - az * bx;
            area2 += cross;
            cx += (ax + bx) * cross;
            cz += (az + bz) * cross;
            height += a.y;
        }
        if (area2 <= 1e-10)
        {
            return result;
        }
        cx = origin.x + cx / (3 * area2);
        cz = origin.z + cz / (3 * area2);
        result.center = physx::PxVec3(float(cx), float(height / hull.size()), float(cz));
        for (const auto& vertex : hull)
        {
            const physx::PxVec3 expanded(float(cx + scale * (vertex.x - cx)), vertex.y,
                float(cz + scale * (vertex.z - cz)));
            if (!expanded.isFinite())
            {
                return {};
            }
            result.bufferHull.push_back(expanded);
        }
        if (!point.isFinite())
        {
            return result;
        }

        const double dx = double(point.x) - cx, dz = double(point.z) - cz;
        const double distance = std::sqrt(dx * dx + dz * dz);
        result.comDistance = float(distance);
        if (distance <= 1e-8)
        {
            result.weight = 1;
            result.zone = FootSupportZone::Inside;
            return result;
        }
        const double directionX = dx / distance, directionZ = dz / distance;
        double inner = std::numeric_limits<double>::infinity();
        for (size_t i = 0; i < hull.size(); ++i)
        {
            const auto& a = hull[i];
            const auto& b = hull[(i + 1) % hull.size()];
            const double ex = double(b.x) - a.x, ez = double(b.z) - a.z;
            const double outward = ez * directionX - ex * directionZ;
            if (outward > 0)
            {
                inner = (std::min)(inner, (ex * (cz - a.z) - ez * (cx - a.x)) / outward);
            }
        }
        const double outer = scale * inner;
        if (!std::isfinite(inner) || inner <= 0 || outer - inner <= 1e-8)
        {
            return result;
        }
        result.hasDirection = true;
        result.innerDistance = float(inner);
        result.outerDistance = float(outer);
        if (IsInsideFootSupportHull(hull, point))
        {
            result.weight = 1;
            result.zone = FootSupportZone::Inside;
        }
        else if (distance >= outer - 1e-6)
        {
            result.zone = FootSupportZone::Outside;
        }
        else
        {
            result.weight = float((std::max)(0.0, (std::min)(1.0, (outer - distance) / (outer - inner))));
            result.zone = FootSupportZone::Buffer;
        }
        return result;
    }
}
