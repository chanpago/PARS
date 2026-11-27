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
        for (const physx::PxVec3& point : points)
        {
            if (!point.isFinite())
            {
                return {};
            }
        }
        std::sort(points.begin(), points.end(), [](const physx::PxVec3& a, const physx::PxVec3& b) -> bool
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
        points.erase(std::unique(points.begin(), points.end(), [](const physx::PxVec3& a, const physx::PxVec3& b) -> bool
                         {
            return a.x == b.x && a.z == b.z;
        }),
            points.end());
        if (points.size() < 3)
        {
            return {};
        }

        double (*const turn)(const physx::PxVec3&, const physx::PxVec3&, const physx::PxVec3&) = [](const physx::PxVec3& a, const physx::PxVec3& b, const physx::PxVec3& c) -> double
        {
            return (double(b.x) - a.x) * (double(c.z) - a.z) -
                (double(b.z) - a.z) * (double(c.x) - a.x);
        };
        std::vector<physx::PxVec3> hull;
        hull.reserve(points.size() * 2);
        for (const physx::PxVec3& point : points)
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
            const physx::PxVec3& a = hull[i];
            const physx::PxVec3& b = hull[(i + 1) % hull.size()];
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
    public:
        std::vector<physx::PxVec3> bufferHull;
        physx::PxVec3 center{0};
        float weight = 0;
        float comDistance = 0;
        float innerDistance = 0;
        float outerDistance = 0;
        FootSupportZone zone = FootSupportZone::Invalid;
        bool hasDirection = false;
    };

    // 기준 다리 길이를 구할 수 없는 모델에 사용하는 완충 확장 반경(m).
    constexpr float DefaultFootSupportBufferRadius = 1.0f;

    inline double FootSupportLateralProjection(const physx::PxVec3& point, const physx::PxVec3& right)
    {
        return static_cast<double>(point.x) * right.x + static_cast<double>(point.z) * right.z;
    }

    inline physx::PxVec3 ExpandFootSupportVertex(const physx::PxVec3& vertex, double cx, double cz, double expansion)
    {
        return physx::PxVec3(static_cast<float>(vertex.x + expansion * (vertex.x - cx)), vertex.y,
            static_cast<float>(vertex.z + expansion * (vertex.z - cz)));
    }

    // 각 변을 확대하면서 좌우 경계에서 자른다. 한 변이 두 경계를 모두 지나면 진행 순서대로 교점을 넣는다.
    inline std::vector<physx::PxVec3> BuildFootSupportBufferHull(const std::vector<physx::PxVec3>& hull,
        double cx, double cz, double expansion, const physx::PxVec3& right, double leftLimit, double rightLimit)
    {
        if (hull.empty() || expansion == 0)
        {
            return hull;
        }
        std::vector<physx::PxVec3> clipped;
        clipped.reserve(hull.size() + 2);
        physx::PxVec3 previous = ExpandFootSupportVertex(hull.back(), cx, cz, expansion);
        double previousSide = FootSupportLateralProjection(previous, right);
        for (const physx::PxVec3& support : hull)
        {
            const physx::PxVec3 vertex = ExpandFootSupportVertex(support, cx, cz, expansion);
            if (!vertex.isFinite())
            {
                return {};
            }
            const double side = FootSupportLateralProjection(vertex, right);
            const double firstLimit = side > previousSide ? leftLimit : rightLimit;
            const double secondLimit = side > previousSide ? rightLimit : leftLimit;
            for (const double limit : {firstLimit, secondLimit})
            {
                if ((previousSide < limit && limit < side) || (side < limit && limit < previousSide))
                {
                    const float fraction = static_cast<float>((limit - previousSide) / (side - previousSide));
                    clipped.push_back(previous + (vertex - previous) * fraction);
                }
            }
            if (leftLimit <= side && side <= rightLimit)
            {
                clipped.push_back(vertex);
            }
            previous = vertex;
            previousSide = side;
        }
        return clipped;
    }

    // A의 면적 중심에서 다리 길이 상한까지 확장한 뒤 캐릭터 좌우의 추가 여유만 절반으로 줄인다.
    inline FootSupportBalance EvaluateFootSupportBalance(const std::vector<physx::PxVec3>& hull,
        const physx::PxVec3& point, float scale = 2.0f, float radiusLimit = DefaultFootSupportBufferRadius,
        const physx::PxVec3& characterForward = physx::PxVec3(0, 0, 1))
    {
        FootSupportBalance result;
        if (hull.size() < 3 || !std::isfinite(scale) || scale <= 1.0f ||
            !std::isfinite(radiusLimit) || radiusLimit <= 0.0f)
        {
            return result;
        }
        for (const physx::PxVec3& vertex : hull)
        {
            if (!vertex.isFinite())
            {
                return result;
            }
        }

        // 월드 원점에서 멀 때도 면적과 중심을 안정적으로 구하도록 첫 꼭짓점을 기준으로 옮긴다.
        const physx::PxVec3& origin = hull[0];
        double area2 = 0;
        double cx = 0;
        double cz = 0;
        double height = 0;
        for (size_t i = 0; i < hull.size(); ++i)
        {
            const physx::PxVec3& a = hull[i];
            const physx::PxVec3& b = hull[(i + 1) % hull.size()];
            const double ax = double(a.x) - origin.x;
            const double az = double(a.z) - origin.z;
            const double bx = double(b.x) - origin.x;
            const double bz = double(b.z) - origin.z;
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
        physx::PxVec3 right(characterForward.z, 0, -characterForward.x);
        if (!right.isFinite() || right.isZero())
        {
            right = physx::PxVec3(1, 0, 0);
        }
        const double centerSide = cx * right.x + cz * right.z;
        double leftEdge = centerSide;
        double rightEdge = centerSide;
        double radiusSquared = 0;
        for (const physx::PxVec3& vertex : hull)
        {
            const double x = double(vertex.x) - cx;
            const double z = double(vertex.z) - cz;
            radiusSquared = (std::max)(radiusSquared, x * x + z * z);
            const double side = FootSupportLateralProjection(vertex, right);
            leftEdge = (std::min)(leftEdge, side);
            rightEdge = (std::max)(rightEdge, side);
        }
        // A가 이미 제한 반경을 채웠으면 B를 A와 일치시켜 실제 발 지지 영역을 보존한다.
        const double bufferScale = physx::PxClamp(radiusLimit / std::sqrt(radiusSquared), 1.0, static_cast<double>(scale));
        const double expansion = bufferScale - 1.0;
        const double lateralExpansion = 0.5 * expansion;
        const double leftLimit = leftEdge + lateralExpansion * (leftEdge - centerSide);
        const double rightLimit = rightEdge + lateralExpansion * (rightEdge - centerSide);
        result.bufferHull = BuildFootSupportBufferHull(hull, cx, cz, expansion, right, leftLimit, rightLimit);
        if (result.bufferHull.empty())
        {
            return {};
        }
        if (!point.isFinite())
        {
            return result;
        }

        const double dx = double(point.x) - cx;
        const double dz = double(point.z) - cz;
        const double distance = std::sqrt(dx * dx + dz * dz);
        result.comDistance = float(distance);
        if (distance <= 1e-8)
        {
            result.weight = 1;
            result.zone = FootSupportZone::Inside;
            return result;
        }
        const double directionX = dx / distance;
        const double directionZ = dz / distance;
        double inner = std::numeric_limits<double>::infinity();
        for (size_t i = 0; i < hull.size(); ++i)
        {
            const physx::PxVec3& a = hull[i];
            const physx::PxVec3& b = hull[(i + 1) % hull.size()];
            const double ex = double(b.x) - a.x;
            const double ez = double(b.z) - a.z;
            const double outward = ez * directionX - ex * directionZ;
            if (outward > 0)
            {
                inner = (std::min)(inner, (ex * (cz - a.z) - ez * (cx - a.x)) / outward);
            }
        }
        if (!std::isfinite(inner) || inner <= 0)
        {
            return result;
        }
        // COM 방향에서 확대 다각형과 좌우 절단선 중 먼저 만나는 경계가 실제 B의 끝이다.
        double outer = bufferScale * inner;
        const double lateralDirection = directionX * right.x + directionZ * right.z;
        if (lateralDirection != 0)
        {
            const double limit = lateralDirection > 0 ? rightLimit : leftLimit;
            outer = (std::min)(outer, (limit - centerSide) / lateralDirection);
        }
        result.hasDirection = true;
        result.innerDistance = float(inner);
        result.outerDistance = float(outer);
        if (IsInsideFootSupportHull(hull, point))
        {
            result.weight = 1;
            result.zone = FootSupportZone::Inside;
        }
        else if (distance >= outer - 1e-6 || outer == inner)
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
