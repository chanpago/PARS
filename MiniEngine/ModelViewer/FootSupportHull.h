#pragma once

#include <foundation/PxVec3.h>
#include <algorithm>
#include <cmath>
#include <vector>

namespace PARS
{
    // XZ convex hull, counterclockwise without a repeated closing vertex.
    // Keep each hit's Y coordinate for drawing on the actual surface.
    inline std::vector<physx::PxVec3> BuildFootSupportHull(std::vector<physx::PxVec3> points)
    {
        std::sort(points.begin(), points.end(), [](const auto& a, const auto& b)
        {
            if (a.x != b.x) return a.x < b.x;
            if (a.z != b.z) return a.z < b.z;
            return a.y < b.y;
        });
        points.erase(std::unique(points.begin(), points.end(), [](const auto& a, const auto& b)
        {
            return a.x == b.x && a.z == b.z;
        }), points.end());
        if (points.size() < 3)
            return {};

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
                hull.pop_back();
            hull.push_back(point);
        }
        const size_t upperStart = hull.size() + 1;
        for (size_t i = points.size() - 1; i-- > 0;)
        {
            while (hull.size() >= upperStart && turn(hull[hull.size() - 2], hull.back(), points[i]) <= 0)
                hull.pop_back();
            hull.push_back(points[i]);
        }
        hull.pop_back();
        if (hull.size() < 3)
            hull.clear();
        return hull;
    }

    // A point on an edge counts as inside. Y does not affect the XZ test.
    inline bool IsInsideFootSupportHull(const std::vector<physx::PxVec3>& hull, const physx::PxVec3& point)
    {
        if (hull.size() < 3 || !point.isFinite())
            return false;
        for (size_t i = 0; i < hull.size(); ++i)
        {
            const auto& a = hull[i];
            const auto& b = hull[(i + 1) % hull.size()];
            const double dx = double(b.x) - a.x, dz = double(b.z) - a.z;
            const double cross = dx * (double(point.z) - a.z) - dz * (double(point.x) - a.x);
            if (cross < -1e-6 * std::sqrt(dx * dx + dz * dz))
                return false;
        }
        return true;
    }
}
