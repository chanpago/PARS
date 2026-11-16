#include "FootstepCollisionBounds.h"
#include "foundation/PxMat33.h"
#include <algorithm>
#include <cmath>

// 구간 연산의 반올림 순서를 유지한다. 프로젝트의 빠른 수학 최적화보다 우선한다.
#pragma float_control(precise, on, push)

using namespace PARS;
using namespace physx;
using namespace PARS::FootstepInterval;

namespace
{
    bool Unit(PxVec3& value)
    {
        const float length = value.magnitude();
        if (!value.isFinite() || !std::isfinite(length) || length <= 1e-6f)
        {
            return false;
        }
        value /= length;
        return true;
    }

    Range Blend(const Range& time)
    {
        return time * time * (Range(3) - Range(2) * time);
    }

    Vector BoundOffset(const FootstepPath& path, const Range& flight)
    {
        if (!path.enabled || flight.upper < 0 || flight.lower >= path.startTime + path.duration)
        {
            return Vector();
        }
        const Range u = Clamp((flight - Range(path.startTime)) / Range(path.duration), 0, 1);
        Vector offset(path.coefficients[4]);
        for (int index = 3; index >= 0; --index)
        {
            offset = offset * u + Vector(path.coefficients[index]);
        }
        if (flight.lower < 0 || flight.upper >= path.startTime + path.duration)
        {
            offset = Join(offset, Vector());
        }
        return offset;
    }

    Vector BoundPosition(const FootstepCollisionLeg& leg, const Range& time)
    {
        const FootstepCommand& command = leg.command;
        const FootstepMotion& motion = command.motion;
        Range elapsed = Range(motion.elapsedTime) + time;
        Vector takeoff(command.takeoffPosition);
        Vector lift;
        bool includesLift = false;
        if (!command.hasTakenOff)
        {
            const float remaining = PxMax(0.0f, .5f * motion.duration - motion.takeoffElapsedTime);
            elapsed = time - Range(remaining);
            const Range liftTime = Clamp((Range(motion.takeoffElapsedTime) + time) /
                Range(.5f * motion.duration), 0, 1);
            lift = Vector(command.startPosition) + Vector(Range(0), Range(motion.initialHeight) * Blend(liftTime), Range(0));
            if (time.upper <= remaining)
            {
                return lift + BoundOffset(leg.path, elapsed);
            }
            includesLift = time.lower <= remaining;
            takeoff = Vector(command.startPosition) + Vector(PxVec3(0, motion.initialHeight, 0));
        }
        const Range progress = Clamp(elapsed / Range(motion.duration), 0, 1);
        Vector position = takeoff + (Vector(command.target.position) - takeoff) * Blend(progress);
        position.y = position.y + Range(.5f * motion.initialHeight) * (Range(1) - Cos(Range(PxTwoPi) * progress));
        position.y = Minimum(position.y, PxMax(command.startPosition.y, command.target.position.y) + motion.initialHeight);
        const Range overtime = Maximum(elapsed - Range(motion.duration), 0);
        const Range lowering = Minimum(Range(motion.landingLowering) + overtime * Range(motion.landingLoweringSpeed),
            motion.maximumLandingLowering);
        position.y = position.y - lowering;
        if (includesLift)
        {
            position = Join(position, lift);
        }
        return position + BoundOffset(leg.path, elapsed);
    }

    bool Project(Vector& hint, const Vector& direction)
    {
        hint = hint - direction * Dot(hint, direction);
        return Normalize(hint);
    }

    bool Perpendicular(const Vector& direction, Vector& result)
    {
        Vector basis(PxVec3(1, 0, 0));
        if (direction.x.lower > -.7f && direction.x.upper < .7f)
        {
            basis = Vector(PxVec3(1, 0, 0));
        }
        else if (direction.x.lower >= .7f || direction.x.upper <= -.7f)
        {
            basis = Vector(PxVec3(0, 1, 0));
        }
        else
        {
            return false;
        }
        result = basis;
        return Project(result, direction);
    }

    bool PlaneQuaternion(const Vector& a, const Vector& b, const Vector& c, Quaternion& result)
    {
        // 행렬→쿼터니언 변환에서 가능한 모든 분기를 합친다. 특이한 분기는 안전 판정을 하지 않는다.
        bool found = false;
        Quaternion reference;
        for (int branch = 0; branch < 4; ++branch)
        {
            Range t;
            Quaternion q;
            bool possible = false;
            if (branch == 0)
            {
                possible = c.z.lower < 0 && a.x.upper > b.y.lower;
                t = Range(1) + a.x - b.y - c.z;
                q = Quaternion(t, a.y + b.x, c.x + a.z, b.z - c.y);
            }
            else if (branch == 1)
            {
                possible = c.z.lower < 0 && a.x.lower <= b.y.upper;
                t = Range(1) - a.x + b.y - c.z;
                q = Quaternion(a.y + b.x, t, b.z + c.y, c.x - a.z);
            }
            else if (branch == 2)
            {
                possible = c.z.upper >= 0 && a.x.lower < -b.y.lower;
                t = Range(1) - a.x - b.y + c.z;
                q = Quaternion(c.x + a.z, b.z + c.y, t, a.y - b.x);
            }
            else
            {
                possible = c.z.upper >= 0 && a.x.upper >= -b.y.upper;
                t = Range(1) + a.x + b.y + c.z;
                q = Quaternion(b.z - c.y, c.x - a.z, a.y - b.x, t);
            }
            if (!possible)
            {
                continue;
            }
            if (t.lower <= 0 || !t.IsFinite())
            {
                return false;
            }
            q = q * (Range(.5) / Sqrt(t));
            if (!found)
            {
                reference = Quaternion(Range((q.x.lower + q.x.upper) * .5),
                    Range((q.y.lower + q.y.upper) * .5), Range((q.z.lower + q.z.upper) * .5),
                    Range((q.w.lower + q.w.upper) * .5));
            }
            // q와 -q는 같은 회전이다. 분기 경계의 부호 반전이 구간을 불필요하게 넓히지 않게 한다.
            const Range hemisphere = q.x * reference.x + q.y * reference.y + q.z * reference.z + q.w * reference.w;
            if (hemisphere.upper < 0)
            {
                q = q * Range(-1);
            }
            else if (hemisphere.lower <= 0)
            {
                return false;
            }
            result = found ? Join(result, q) : q;
            found = true;
        }
        return found && result.IsFinite();
    }

    bool FromTo(Vector from, Vector to, Quaternion& result)
    {
        if (!Normalize(from) || !Normalize(to))
        {
            return false;
        }
        const Range dot = Clamp(Dot(from, to), -1, 1);
        if (dot.lower < -.999999f)
        {
            return false;
        }
        const Vector cross = Cross(from, to);
        result = Quaternion(cross.x, cross.y, cross.z, Range(1) + dot);
        return Normalize(result);
    }

    bool BoundIK(const TwoBoneIKInput& input, const Vector& sole, std::array<Transform, 3>& bones)
    {
        const float upper = input.thighLength;
        const float lower = input.calfLength;
        PxVec3 sourceUpper = input.knee.p - input.hip.p;
        PxVec3 sourceLower = input.ankle.p - input.knee.p;
        if (!Unit(sourceUpper) || !Unit(sourceLower))
        {
            return false;
        }
        Transform target(input.targetSole);
        target.position = sole;
        const Transform requested = target * input.soleInBone.getInverse();
        Vector direction = requested.position - Vector(input.hip.p);
        const Range requestedDistance = Length(direction);
        if (!Normalize(direction))
        {
            return false;
        }
        PxVec3 sourceAxis = input.ankle.p - input.hip.p;
        if (!Unit(sourceAxis))
        {
            sourceAxis = sourceUpper;
        }
        const PxVec3 kneeOffset = input.knee.p - input.hip.p;
        PxVec3 sourceBend = kneeOffset - sourceAxis * kneeOffset.dot(sourceAxis);
        const bool animationBent = sourceBend.magnitude() > (upper + lower) * .001f;
        Vector bend;
        // 유효한 선행 힌트의 실패 여부가 구간 안에서 바뀌면 분할하여 다시 검사한다.
        if (animationBent)
        {
            bend = Vector(sourceBend);
            if (!Project(bend, direction))
            {
                return false;
            }
        }
        else if (input.hasPreviousBend && !input.previousBend.isZero())
        {
            bend = Vector(input.previousBend);
            if (!Project(bend, direction))
            {
                return false;
            }
        }
        else if (!input.referenceBend.isZero())
        {
            bend = Vector(input.referenceBend);
            if (!Project(bend, direction))
            {
                return false;
            }
        }
        else if (!Perpendicular(direction, bend))
        {
            return false;
        }
        const float margin = PxMin((upper + lower) * 1e-4f, PxMin(upper, lower) * .1f);
        const Range distance = Clamp(requestedDistance, PxAbs(upper - lower) + margin, upper + lower - margin);
        const Range along = (Range(upper) * Range(upper) - Range(lower) * Range(lower) + Square(distance)) /
            (Range(2) * distance);
        const Range height = Sqrt(Maximum(Range(upper) * Range(upper) - Square(along), 0));
        const Vector knee = Vector(input.hip.p) + direction * along + bend * height;
        const Vector ankle = Vector(input.hip.p) + direction * distance;
        const Vector normal = Cross(direction, bend);
        Quaternion sourcePlane;
        if (animationBent && Unit(sourceBend))
        {
            // 원본 굽힘 평면은 시간에 따라 변하지 않으므로 실제 IK와 같은 상수 회전을 재사용한다.
            sourcePlane = Quaternion(PxQuat(PxMat33(sourceAxis, sourceBend, sourceAxis.cross(sourceBend))));
        }
        else
        {
            Vector sourceBendBounds = bend;
            if (!Project(sourceBendBounds, Vector(sourceAxis)) ||
                !PlaneQuaternion(Vector(sourceAxis), sourceBendBounds, Cross(Vector(sourceAxis), sourceBendBounds), sourcePlane))
            {
                return false;
            }
        }
        Quaternion targetPlane;
        if (!PlaneQuaternion(direction, bend, normal, targetPlane))
        {
            return false;
        }
        sourcePlane.x = -sourcePlane.x;
        sourcePlane.y = -sourcePlane.y;
        sourcePlane.z = -sourcePlane.z;
        Quaternion planeRotation = targetPlane * sourcePlane;
        if (!Normalize(planeRotation))
        {
            return false;
        }
        Quaternion hipRotation;
        Quaternion kneeRotation;
        if (!FromTo(Rotate(planeRotation, Vector(sourceUpper)), knee - Vector(input.hip.p), hipRotation) ||
            !FromTo(Rotate(planeRotation, Vector(sourceLower)), ankle - knee, kneeRotation))
        {
            return false;
        }
        hipRotation = hipRotation * planeRotation * Quaternion(input.hip.q);
        kneeRotation = kneeRotation * planeRotation * Quaternion(input.knee.q);
        if (!Normalize(hipRotation) || !Normalize(kneeRotation))
        {
            return false;
        }
        bones[0].position = Vector(input.hip.p);
        bones[0].rotation = hipRotation;
        bones[1].position = knee;
        bones[1].rotation = kneeRotation;
        bones[2].position = ankle;
        bones[2].rotation = requested.rotation;
        return bones[0].IsFinite() && bones[1].IsFinite() && bones[2].IsFinite();
    }

    Range ProjectShape(const FootstepCollisionShape& shape, const Transform& bounds,
        const std::array<Vector, 3>& rotatedAxes, const PxVec3& axis, float margin)
    {
        const Vector direction(axis);
        const Range axisLength = Length(direction);
        Range center = Dot(bounds.position, direction);
        Range extent = Range(margin) * axisLength;
        if (shape.geometry.getType() == PxGeometryType::eCAPSULE)
        {
            const PxCapsuleGeometry& capsule = shape.geometry.capsule();
            extent = extent + Range(capsule.radius) * axisLength + Range(capsule.halfHeight) *
                Range(MagnitudeUpper(Dot(rotatedAxes[0], direction)));
        }
        else if (shape.geometry.getType() == PxGeometryType::eBOX)
        {
            const PxVec3 half = shape.geometry.box().halfExtents;
            for (int axisIndex = 0; axisIndex < 3; ++axisIndex)
            {
                extent = extent + Range(half[axisIndex]) *
                    Range(MagnitudeUpper(Dot(rotatedAxes[axisIndex], direction)));
            }
        }
        else
        {
            extent = extent + Range(shape.geometry.sphere().radius) * axisLength;
        }
        return center + Range(-extent.upper, extent.upper);
    }
}

bool PARS::PredictFootstepLeg(const FootstepCollisionLeg& leg, float timeAhead,
    std::array<PxTransform, 3>& shapes, LegIKResult& result)
{
    if (!leg.command.IsInProgress())
    {
        for (size_t part = 0; part < shapes.size(); ++part)
        {
            shapes[part] = leg.shapes[part].actualPose;
        }
        return true;
    }
    TwoBoneIKInput pose = leg.pose;
    // 발바닥 회전은 이번 실제 제어 목표와 정확히 같게 유지한다.
    pose.targetSole.p = SampleFootstepPosition(leg.command, leg.path, timeAhead);
    result = SolveTwoBoneIK(pose);
    if (!result.valid)
    {
        return false;
    }
    const std::array<PxTransform, 3> bones{result.hip, result.knee, result.ankle};
    for (size_t part = 0; part < shapes.size(); ++part)
    {
        shapes[part] = bones[part] * leg.shapes[part].inBone;
    }
    return true;
}

bool PARS::BoundFootstepLeg(const FootstepCollisionLeg& leg, float begin, float end,
    std::array<Transform, 3>& shapes)
{
    if (!leg.command.IsInProgress())
    {
        for (size_t part = 0; part < shapes.size(); ++part)
        {
            shapes[part] = Transform(leg.shapes[part].actualPose);
        }
        return true;
    }
    TwoBoneIKInput pose = leg.pose;
    std::array<Transform, 3> bones;
    if (!BoundIK(pose, BoundPosition(leg, Time(begin, end)), bones))
    {
        return false;
    }
    for (size_t part = 0; part < shapes.size(); ++part)
    {
        shapes[part] = bones[part] * leg.shapes[part].inBone;
    }
    return true;
}

bool PARS::SeparateFootstepShapes(const FootstepCollisionShape& first, const Transform& firstBounds,
    const PxTransform& firstMiddle, float firstMargin,
    const FootstepCollisionShape& second, const Transform& secondBounds,
    const PxTransform& secondMiddle, float secondMargin)
{
    if (!firstBounds.IsFinite() || !secondBounds.IsFinite())
    {
        return false;
    }
    std::array<PxVec3, 25> axes;
    std::array<Vector, 3> firstRotatedAxes;
    std::array<Vector, 3> secondRotatedAxes;
    for (int index = 0; index < 3; ++index)
    {
        PxVec3 basis(0);
        basis[index] = 1;
        firstRotatedAxes[index] = Rotate(firstBounds.rotation, Vector(basis));
        secondRotatedAxes[index] = Rotate(secondBounds.rotation, Vector(basis));
    }
    const PxMat33 firstAxes(firstMiddle.q);
    const PxMat33 secondAxes(secondMiddle.q);
    axes[0] = secondMiddle.p - firstMiddle.p;
    uint32_t count = 1;
    for (int index = 0; index < 3; ++index)
    {
        PxVec3 basis(0);
        basis[index] = 1;
        axes[count++] = basis;
        axes[count++] = firstAxes[index];
        axes[count++] = secondAxes[index];
        for (int other = 0; other < 3; ++other)
        {
            axes[count++] = firstAxes[index].cross(secondAxes[other]);
        }
    }
    // 캡슐의 축과 중심 연결선에 수직인 분리축도 검사한다.
    for (int index = 0; index < 3; ++index)
    {
        axes[count++] = axes[0] - firstAxes[index] * axes[0].dot(firstAxes[index]);
        axes[count++] = axes[0] - secondAxes[index] * axes[0].dot(secondAxes[index]);
    }
    for (uint32_t index = 0; index < count; ++index)
    {
        PxVec3 axis = axes[index];
        if (!axis.isFinite() || axis.normalize() < 1e-6f)
        {
            continue;
        }
        const Range a = ProjectShape(first, firstBounds, firstRotatedAxes, axis, firstMargin);
        const Range b = ProjectShape(second, secondBounds, secondRotatedAxes, axis, secondMargin);
        if (a.IsFinite() && b.IsFinite() && (a.upper < b.lower || b.upper < a.lower))
        {
            return true;
        }
    }
    return false;
}

float PARS::FootstepShapeRadius(const FootstepCollisionShape& shape)
{
    switch (shape.geometry.getType())
    {
    case PxGeometryType::eCAPSULE:
        return shape.geometry.capsule().halfHeight + shape.geometry.capsule().radius;
    case PxGeometryType::eBOX:
        return shape.geometry.box().halfExtents.magnitude();
    case PxGeometryType::eSPHERE:
        return shape.geometry.sphere().radius;
    default:
        return 0;
    }
}

#pragma float_control(pop)
