#include "RagdollComparison.h"
#include <algorithm>
#include <cmath>

using namespace Math;

namespace PARS
{
    namespace
    {
        constexpr float ViolationTolerance = 1e-4f;

        float AxisExcess(float angle, PhysicsJointMotion motion, float minimum, float maximum)
        {
            if (motion == PhysicsJointMotion::Free)
            {
                return 0;
            }
            if (motion == PhysicsJointMotion::Locked)
            {
                return std::abs(angle);
            }
            return std::max(0.0f, std::max(minimum - angle, angle - maximum));
        }

        bool Finite(const Matrix4& transform)
        {
            const float* values = reinterpret_cast<const float*>(&transform);
            for (int i = 0; i < 16; ++i)
            {
                if (!std::isfinite(values[i]))
                {
                    return false;
                }
            }
            return true;
        }
    }

    void ComparisonMetric::Add(float value, float time)
    {
        sum += value;
        squareSum += double(value) * value;
        if (count == 0 || value > maximum)
        {
            maximum = value;
            maximumTime = time;
        }
        ++count;
    }

    float ComparisonMetric::Mean() const
    {
        return count ? static_cast<float>(sum / count) : 0;
    }

    float ComparisonMetric::Rms() const
    {
        return count ? static_cast<float>(std::sqrt(squareSum / count)) : 0;
    }

    void ComparisonViolation::Add(float value, float time)
    {
        excess.Add(value, time);
        if (value > ViolationTolerance)
        {
            if (frames == 0)
            {
                firstTime = time;
            }
            lastTime = time;
            ++frames;
            longestFrames = std::max(longestFrames, ++consecutiveFrames);
        }
        else
        {
            consecutiveFrames = 0;
        }
    }

    float ComparisonRotationError(Quaternion a, Quaternion b)
    {
        DirectX::XMFLOAT4 delta;
        XMStoreFloat4(&delta, Normalize(a * ~b));
        return 2 * std::atan2(std::sqrt(delta.x * delta.x + delta.y * delta.y + delta.z * delta.z), std::abs(delta.w));
    }

    ComparisonJointRotation AnalyzeComparisonJoint(const PhysicsConstraint& joint, Quaternion rotation)
    {
        ComparisonJointRotation result;
        Quaternion q = Normalize(rotation);
        DirectX::XMFLOAT4 v;
        XMStoreFloat4(&v, q);
        if (v.w < 0)
        {
            q = -q;
            XMStoreFloat4(&v, q);
        }
        const float norm = std::sqrt(v.x * v.x + v.w * v.w);
        result.singular = norm <= 1e-7f;
        const Quaternion twist = result.singular ? Quaternion(kIdentity) : Quaternion(XMVectorSet(v.x / norm, 0, 0, v.w / norm));
        DirectX::XMFLOAT4 swing;
        XMStoreFloat4(&swing, Normalize(q * ~twist));
        if (swing.w < 0)
        {
            swing.y = -swing.y;
            swing.z = -swing.z;
            swing.w = -swing.w;
        }
        result.angles = {result.singular ? 0 : 2 * std::atan2(v.x, v.w),
            4 * std::atan2(swing.y, 1 + swing.w), 4 * std::atan2(swing.z, 1 + swing.w)};
        result.excess[0] = AxisExcess(result.angles[0], joint.twist, joint.twistMin, joint.twistMax);
        result.excess[1] = AxisExcess(result.angles[1], joint.swingY, -joint.swingYLimit, joint.swingYLimit);
        result.excess[2] = AxisExcess(result.angles[2], joint.swingZ, -joint.swingZLimit, joint.swingZLimit);
        if (joint.swingY == PhysicsJointMotion::Limited && joint.swingZ == PhysicsJointMotion::Limited)
        {
            const float y = result.angles[1], z = result.angles[2];
            const float radius = std::sqrt(y * y / (joint.swingYLimit * joint.swingYLimit) +
                z * z / (joint.swingZLimit * joint.swingZLimit));
            if (radius > 1)
            {
                result.excess[3] = std::sqrt(y * y + z * z) * (1 - 1 / radius);
            }
        }
        return result;
    }

    bool RagdollComparison::Initialize(const PhysicsAsset& asset, const Matrix4& referencePlacement,
        const Matrix4& actualPlacement, float step, std::string& error)
    {
        if (!asset.Validate(error))
        {
            return false;
        }
        if (asset.bodies.empty() || !std::isfinite(step) || step <= 0 ||
            !Finite(referencePlacement) || !Finite(actualPlacement))
        {
            error = "Invalid comparison setup.";
            return false;
        }
        *this = RagdollComparison();
        m_Asset = asset;
        m_ReferenceInverse = Invert(PhysicsBoneFrame(referencePlacement));
        m_ActualInverse = Invert(PhysicsBoneFrame(actualPlacement));
        m_Step = step;
        m_Bodies.resize(asset.bodies.size());
        m_Joints.resize(asset.constraints.size());
        for (const auto& joint : asset.constraints)
        {
            m_JointBodies.push_back({asset.FindBody(joint.parentBone), asset.FindBody(joint.childBone)});
        }
        error.clear();
        return true;
    }

    bool RagdollComparison::Sample(const std::vector<Matrix4>& reference, const std::vector<Matrix4>& actual,
        float clipTime, std::string& error, bool baseline, const std::string& clipName, float blendAlpha)
    {
        if (m_Bodies.empty() || reference.size() != m_Bodies.size() || actual.size() != m_Bodies.size() || !std::isfinite(clipTime) ||
            !std::isfinite(blendAlpha) || blendAlpha < 0 || blendAlpha > 1 ||
            (!baseline && m_PreviousReference.empty()) || (baseline && !m_Frames.empty()) ||
            !std::all_of(reference.begin(), reference.end(), Finite) || !std::all_of(actual.begin(), actual.end(), Finite))
        {
            error = "Invalid comparison sample or missing baseline.";
            return false;
        }
        std::vector<Vector3> referencePositions, actualPositions;
        ComparisonFrame frame;
        frame.time = static_cast<float>(m_Frames.size() + 1) * m_Step;
        frame.clipTime = clipTime;
        frame.clipName = clipName;
        frame.blendAlpha = blendAlpha;
        frame.bodies.resize(m_Bodies.size());
        frame.joints.resize(m_Joints.size());
        for (size_t i = 0; i < m_Bodies.size(); ++i)
        {
            const Matrix4 a = m_ReferenceInverse * reference[i], b = m_ActualInverse * actual[i];
            const Vector3 pa(a.GetW()), pb(b.GetW()), difference = pb - pa;
            referencePositions.push_back(pa);
            actualPositions.push_back(pb);
            if (baseline)
            {
                continue;
            }
            auto& sample = frame.bodies[i];
            const Vector3 movement = (pb - m_PreviousActual[i]) - (pa - m_PreviousReference[i]);
            XMStoreFloat3(&sample.reference, pa);
            XMStoreFloat3(&sample.actual, pb);
            XMStoreFloat3(&sample.difference, difference);
            XMStoreFloat3(&sample.movementDifference, movement);
            sample.positionError = Length(difference);
            sample.movementError = Length(movement);
            sample.rotationError = ComparisonRotationError(Quaternion(a), Quaternion(b));
            m_Bodies[i].position.Add(sample.positionError, frame.time);
            m_Bodies[i].movement.Add(sample.movementError, frame.time);
            m_Bodies[i].rotation.Add(sample.rotationError, frame.time);
        }
        m_PreviousReference = std::move(referencePositions);
        m_PreviousActual = std::move(actualPositions);
        if (baseline)
        {
            error.clear();
            return true;
        }
        for (size_t i = 0; i < m_Joints.size(); ++i)
        {
            const auto& joint = m_Asset.constraints[i];
            const int parent = m_JointBodies[i][0], child = m_JointBodies[i][1];
            const Quaternion raw = EvaluateJointDriveTarget(joint, reference[parent], reference[child], false);
            const Quaternion bounded = EvaluateJointDriveTarget(joint, reference[parent], reference[child]);
            const Quaternion physical = EvaluateJointDriveTarget(joint, actual[parent], actual[child], false);
            auto& sample = frame.joints[i];
            auto& result = m_Joints[i];
            sample.raw = AnalyzeComparisonJoint(joint, raw);
            sample.bounded = AnalyzeComparisonJoint(joint, bounded);
            sample.actual = AnalyzeComparisonJoint(joint, physical);
            sample.clampError = ComparisonRotationError(raw, bounded);
            sample.trackingError = ComparisonRotationError(bounded, physical);
            sample.totalError = ComparisonRotationError(raw, physical);
            result.clamp.Add(sample.clampError, frame.time);
            result.tracking.Add(sample.trackingError, frame.time);
            result.total.Add(sample.totalError, frame.time);
            result.singularFrames += sample.raw.singular ? 1 : 0;
            for (size_t axis = 0; axis < 4; ++axis)
            {
                result.violations[axis].Add(sample.raw.excess[axis], frame.time);
                result.actualViolations[axis].Add(sample.actual.excess[axis], frame.time);
                if (axis < 3)
                {
                    const float angle = sample.raw.angles[axis];
                    result.minimumAngles[axis] = m_Frames.empty() ? angle : std::min(result.minimumAngles[axis], angle);
                    result.maximumAngles[axis] = m_Frames.empty() ? angle : std::max(result.maximumAngles[axis], angle);
                }
            }
        }
        m_Frames.push_back(std::move(frame));
        error.clear();
        return true;
    }
}
