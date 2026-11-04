#pragma once

#include "PhysicsPose.h"
#include <array>

namespace PARS
{
    struct ComparisonMetric
    {
    public:
        void Add(float value, float time);

    public:
        float Mean() const;
        float Rms() const;

    public:
        double sum = 0;
        double squareSum = 0;
        size_t count = 0;
        float maximum = 0;
        float maximumTime = 0;
    };

    struct ComparisonViolation
    {
    public:
        void Add(float value, float time);

    public:
        ComparisonMetric excess;
        size_t frames = 0;
        size_t longestFrames = 0;
        size_t consecutiveFrames = 0;
        float firstTime = -1;
        float lastTime = -1;
    };

    struct ComparisonJointRotation
    {
    public:
        // D6는 twist/quarter-angle swing, Articulation은 회전 벡터다. 단위는 rad다.
        std::array<float, 3> angles{};
        // X/Y/Z 단독 초과량과 두 Limited swing 축의 타원 방사 방향 초과량이다.
        std::array<float, 4> excess{};
        bool singular = false;
    };

    ComparisonJointRotation AnalyzeComparisonJoint(const PhysicsConstraint& joint, Math::Quaternion rotation, bool articulation = false);
    float ComparisonRotationError(Math::Quaternion a, Math::Quaternion b);

    struct ComparisonBodySample
    {
    public:
        DirectX::XMFLOAT3 reference{};
        DirectX::XMFLOAT3 actual{};
        DirectX::XMFLOAT3 difference{};
        DirectX::XMFLOAT3 movementDifference{};
        float positionError = 0;
        float movementError = 0;
        float rotationError = 0;
    };

    struct ComparisonJointSample
    {
    public:
        ComparisonJointRotation raw;
        ComparisonJointRotation bounded;
        ComparisonJointRotation actual;
        float clampError = 0;
        float trackingError = 0;
        float totalError = 0;
    };

    struct ComparisonFrame
    {
    public:
        std::string clipName;
        std::vector<ComparisonBodySample> bodies;
        std::vector<ComparisonJointSample> joints;
        float time = 0;
        float clipTime = 0;
        float blendAlpha = 1;
    };

    struct ComparisonBodyResult
    {
    public:
        ComparisonMetric position;
        ComparisonMetric movement;
        ComparisonMetric rotation;
    };

    struct ComparisonJointResult
    {
    public:
        ComparisonMetric clamp;
        ComparisonMetric tracking;
        ComparisonMetric total;
        std::array<ComparisonViolation, 4> violations;
        std::array<ComparisonViolation, 4> actualViolations;
        size_t singularFrames = 0;
        std::array<float, 3> minimumAngles{};
        std::array<float, 3> maximumAngles{};
    };

    // 렌더링과 PhysX에 의존하지 않는 측정기다. 입력은 콜라이더 중심의 강체 변환이다.
    // 배치 차이는 초기 rigid frame으로 한 번만 보정하며 골반의 실제 드리프트를 지우지 않는다.
    class RagdollComparison
    {
    public:
        bool Initialize(const PhysicsAsset& asset, const Math::Matrix4& referencePlacement,
            const Math::Matrix4& actualPlacement, float step, std::string& error);
        bool Sample(const std::vector<Math::Matrix4>& reference, const std::vector<Math::Matrix4>& actual,
            float clipTime, std::string& error, bool baseline = false,
            const std::string& clipName = {}, float blendAlpha = 1);
        bool ExportHtml(const std::string& path, const std::string& description, std::string& error) const;

    public:
        const PhysicsAsset& GetAsset() const
        {
            return m_Asset;
        }
        const std::vector<ComparisonFrame>& GetFrames() const
        {
            return m_Frames;
        }
        const std::vector<ComparisonBodyResult>& GetBodies() const
        {
            return m_Bodies;
        }
        const std::vector<ComparisonJointResult>& GetJoints() const
        {
            return m_Joints;
        }
        float GetStep() const
        {
            return m_Step;
        }

    private:
        Math::Matrix4 m_ReferenceInverse{Math::kIdentity};
        Math::Matrix4 m_ActualInverse{Math::kIdentity};
        PhysicsAsset m_Asset;
        std::vector<std::array<int, 2>> m_JointBodies;
        std::vector<Math::Vector3> m_PreviousReference;
        std::vector<Math::Vector3> m_PreviousActual;
        std::vector<ComparisonFrame> m_Frames;
        std::vector<ComparisonBodyResult> m_Bodies;
        std::vector<ComparisonJointResult> m_Joints;
        float m_Step = 1.0f / 60;
        bool m_Articulation = false;
    };
}
