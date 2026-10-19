#pragma once

#include "VectorMath.h"
#include <string>
#include <vector>

namespace PARS
{
    enum class PhysicsShapeType
    {
        Capsule,
        Box,
        Sphere
    };

    // 초기 에디터에서는 본마다 기본 형상 바디 하나를 사용한다. 거리는 미터, 질량은 kg 단위다.
    // 오프셋과 회전은 스케일을 제거한 본의 강체 월드 좌표계 기준이다.
    // 가져온 본이나 애니메이션의 스케일은 PhysX 충돌 기본 형상의 크기에 적용하지 않는다.
    // 캡슐 원통은 PxCapsuleGeometry와 동일하게 로컬 X축을 따라 뻗는다.
    struct PhysicsBody
    {
        std::string bone;
        PhysicsShapeType shape = PhysicsShapeType::Capsule;
        DirectX::XMFLOAT3 offset{0, 0, 0};
        DirectX::XMFLOAT4 rotation{0, 0, 0, 1};
        DirectX::XMFLOAT3 halfExtents{.05f, .1f, .05f};
        float radius = .05f;
        float halfLength = .1f; // 원통 부분의 반길이이며, 양 끝의 반구는 제외한다.
        float mass = 1;

        Math::Matrix4 GetLocalTransform() const;
    };

    Math::Matrix4 PhysicsBoneFrame(const Math::Matrix4& boneWorld);

    enum class PhysicsJointMotion
    {
        Locked,
        Limited,
        Free
    };

    // 힘 기반 각도 PD 드라이브다. 토크 제한은 구동 축마다 Nm 단위로 지정한다.
    // 강성/감쇠 계산에는 라디안과 라디안/초를 사용한다. 이전 형식의 PA는 패시브 상태를 유지한다.
    struct PhysicsAngularDrive
    {
        bool enabled = false;
        float stiffness = 0, damping = 0, maxTorque = 0;
    };

    struct PhysicsJointFrame
    {
        DirectX::XMFLOAT3 offset{0, 0, 0};
        DirectX::XMFLOAT4 rotation{0, 0, 0, 1};
        Math::Matrix4 GetTransform() const;
        static PhysicsJointFrame FromTransform(const Math::Matrix4& rigidTransform);
    };

    // D6 조인트 데이터다. 평행이동은 잠그고, X축은 트위스트, Y/Z축은 스윙으로 사용한다.
    // 좌표계는 바디 기준이다(본의 강체 좌표계 * 콜라이더 오프셋).
    // 각도는 라디안, 위치는 미터 단위다. 축이나 좌표계의 좌우 방향은 변환하지 않는다.
    struct PhysicsConstraint
    {
        std::string parentBone, childBone;
        PhysicsJointFrame parentFrame, childFrame;
        PhysicsJointMotion twist = PhysicsJointMotion::Limited;
        PhysicsJointMotion swingY = PhysicsJointMotion::Limited;
        PhysicsJointMotion swingZ = PhysicsJointMotion::Limited;
        float twistMin = -DirectX::XM_PIDIV4, twistMax = DirectX::XM_PIDIV4;
        float swingYLimit = DirectX::XM_PI / 6, swingZLimit = DirectX::XM_PI / 6;
        bool enableCollision = false;
        PhysicsAngularDrive drives[3]; // 순서대로 트위스트 X, 스윙 Y, 스윙 Z다.
        // SLERP는 모든 회전에 drives[0]을 사용하며, 잠긴 회전축이 없어야 한다.
        bool useSlerpDrive = false;

        void SetWorldFrame(const Math::Matrix4& parentBodyWorld,
            const Math::Matrix4& childBodyWorld, const Math::Matrix4& jointWorld);
    };

    struct PhysicsAsset
    {
        std::vector<PhysicsBody> bodies;
        std::vector<PhysicsConstraint> constraints;
        int FindBody(const std::string& bone) const;
        int FindConstraint(const std::string& boneA, const std::string& boneB) const;
        // 바디를 제거하면 연결된 조인트도 모두 제거한다.
        void RemoveBody(size_t index);
        bool Validate(std::string& error) const;
        bool ValidateSkeleton(const std::vector<std::string>& jointNames, std::string& error) const;
        bool Save(const std::string& path, std::string& error) const;
        // 전체 검증에 성공해야 반영한다. 파일을 거부한 경우 기존 에셋은 그대로 유지한다.
        bool Load(const std::string& path, std::string& error);
    };
    // 조정 가능한 초기 제어 게인이며, 실제 인체의 근력을 측정한 값은 아니다.
    void ConfigureDefaultDrives(PhysicsAsset& asset);
    bool HasEnabledAngularDrive(const PhysicsConstraint& joint);
}
