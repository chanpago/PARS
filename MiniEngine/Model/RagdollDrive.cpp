#include "Ragdoll.h"
#include "PhysicsPose.h"
#include <algorithm>
#include <cmath>
#if defined(_M_X64)
#include "PhysicsWorld.h"
using namespace physx;
#endif
using namespace Math;

namespace PARS
{
#if defined(_M_X64)
    namespace
    {
        PxQuat ToPx(Quaternion q)
        {
            DirectX::XMFLOAT4 value;
            DirectX::XMStoreFloat4(&value, q);
            return PxQuat(value.x, value.y, value.z, value.w);
        }
        float Angle(PxQuat a, PxQuat b)
        {
            return 2 * std::acos(PxClamp(PxAbs(a.dot(b)), 0.0f, 1.0f));
        }
        PxVec3 LimitMagnitude(PxVec3 v, float limit)
        {
            const float length = v.magnitude();
            return length > limit ? v * (limit / length) : v;
        }
        bool Enabled(const PhysicsAngularDrive& drive, PhysicsJointMotion motion)
        {
            return drive.enabled && motion != PhysicsJointMotion::Locked && drive.maxTorque > 0 &&
                (drive.GetStiffness() > 0 || drive.GetDamping() > 0);
        }
    }
#endif

    void Ragdoll::SetDrivesEnabled(bool enabled)
    {
        m_DrivesEnabled = enabled && IsSimulating();
#if defined(_M_X64)
        const PxD6Drive::Enum axes[] = {PxD6Drive::eTWIST, PxD6Drive::eSWING1, PxD6Drive::eSWING2};
        for (size_t i = 0; i < m_Joints.size(); ++i)
        {
            const auto& data = m_Asset.constraints[i];
            const PhysicsJointMotion motions[] = {data.twist, data.swingY, data.swingZ};
            m_Joints[i]->setAngularDriveConfig(data.useSlerpDrive ? PxD6AngularDriveConfig::eSLERP : PxD6AngularDriveConfig::eSWING_TWIST);
            for (int axis = 0; axis < (data.useSlerpDrive ? 1 : 3); ++axis)
            {
                const auto& drive = data.drives[axis];
                // 가속도 모드의 게인은 omegaN^2, 2*zeta*omegaN이다. 관성과 토크 제한은 PhysX가 반영한다.
                // 이전 PA의 힘 모드는 기존 게인의 의미를 유지한다.
                const bool on = m_DrivesEnabled && Enabled(drive, motions[axis]);
                PxD6JointDrive motor;
                if (on)
                {
                    motor = PxD6JointDrive(drive.GetStiffness(), drive.GetDamping(), drive.maxTorque, drive.useAcceleration);
                }
                m_Joints[i]->setDrive(data.useSlerpDrive ? PxD6Drive::eSLERP : axes[axis], motor);
            }
        }
        for (auto* body : m_Bodies)
        {
            // 액티브 모터는 애니메이션 추종과 Debug 계산 비용의 균형을 위해 위치 32회 / 속도 8회의
            // 솔버 반복을 사용한다. 패시브 모드는 16/4를 사용한다.
            body->setSolverIterationCounts(m_DrivesEnabled ? 32 : 16, m_DrivesEnabled ? 8 : 4);
            if (!(body->getRigidBodyFlags() & PxRigidBodyFlag::eKINEMATIC))
            {
                body->wakeUp();
            }
        }
#endif
        if (!m_DrivesEnabled)
        {
            m_HasDriveTargets = false;
        }
    }

    bool Ragdoll::ApplyDriveSettings(const PhysicsAsset& asset, std::string& error)
    {
        if (!asset.Validate(error))
        {
            return false;
        }
        if (asset.constraints.size() != m_Asset.constraints.size())
        {
            error = "Drive update requires the same joints.";
            return false;
        }
        for (size_t i = 0; i < asset.constraints.size(); ++i)
        {
            if (asset.constraints[i].parentBone != m_Asset.constraints[i].parentBone ||
                asset.constraints[i].childBone != m_Asset.constraints[i].childBone)
            {
                error = "Drive update requires matching parent/child bodies.";
                return false;
            }
        }
        auto candidate = m_Asset;
        for (size_t i = 0; i < asset.constraints.size(); ++i)
        {
            candidate.constraints[i].useSlerpDrive = asset.constraints[i].useSlerpDrive;
            for (int axis = 0; axis < 3; ++axis)
            {
                candidate.constraints[i].drives[axis] = asset.constraints[i].drives[axis];
            }
        }
        // 다를 수 있는 에디터 제한값 대신 현재 실행 중인 조인트의 운동 설정을 기준으로 검증한다.
        // 이 갱신에서는 의도적으로 조인트 좌표계를 교체하지 않는다.
        if (!candidate.Validate(error))
        {
            return false;
        }
        for (size_t i = 0; i < candidate.constraints.size(); ++i)
        {
            m_Asset.constraints[i].useSlerpDrive = candidate.constraints[i].useSlerpDrive;
            for (int axis = 0; axis < 3; ++axis)
            {
                m_Asset.constraints[i].drives[axis] = candidate.constraints[i].drives[axis];
            }
        }
        SetDrivesEnabled(m_DrivesEnabled);
        error.clear();
        return true;
    }

    bool Ragdoll::UpdateDriveTargets(const std::vector<Matrix4>& boneWorld, float deltaTime, std::string& error)
    {
        if (!IsActive() || !std::isfinite(deltaTime) || deltaTime < 0)
        {
            error = "Invalid drive update.";
            return false;
        }
#if defined(_M_X64)
        std::vector<Matrix4> bodies;
        bodies.reserve(m_Bodies.size());
        PxTransform supportedPose(PxIdentity);
        for (size_t i = 0; i < m_Bodies.size(); ++i)
        {
            const int node = m_BoneNodes[i];
            if (node < 0 || size_t(node) >= boneWorld.size())
            {
                error = "Drive target has no matching bone.";
                return false;
            }

            const auto transform = PhysicsBoneFrame(boneWorld[node]) * m_Asset.bodies[i].GetLocalTransform();
            const auto frame = PhysicsJointFrame::FromTransform(transform);

            if (!ToPx(Quaternion(DirectX::XMLoadFloat4(&frame.rotation))).isSane())
            {
                error = "Invalid drive target rotation.";
                return false;
            }
            if (int(i) == m_SupportBody)
            {
                supportedPose = PxTransform(PxVec3(frame.offset.x, frame.offset.y, frame.offset.z),
                    ToPx(Quaternion(DirectX::XMLoadFloat4(&frame.rotation))));

                if (!supportedPose.isValid())
                {
                    error = "Invalid supported pelvis target.";
                    return false;
                }
            }
            bodies.push_back(transform);
        }
        std::vector<Quaternion> targets;
        targets.reserve(m_Joints.size());
        m_MaxTargetClampAngle = 0;
        for (const auto& data : m_Asset.constraints)
        {
            const auto& a = bodies[m_Asset.FindBody(data.parentBone)];
            const auto& b = bodies[m_Asset.FindBody(data.childBone)];
            const auto target = EvaluateJointDriveTarget(data, a, b);
            if (!ToPx(target).isSane())
            {
                error = "Non-finite joint drive target.";
                return false;
            }
            targets.push_back(target);
            m_MaxTargetClampAngle = PxMax(m_MaxTargetClampAngle, Angle(ToPx(target), ToPx(EvaluateJointDriveTarget(data, a, b, false))));
        }
        if (m_SupportJoint)
        {
            // 골반은 동적 상태를 유지한다. 월드에 고정된 D6 조인트가 제한된 힘/토크를 가하며,
            // 다른 조인트와 접촉 제약도 같은 암시적 풀이 과정에서 함께 계산한다.
            // 선형/SLERP 드라이브의 두 속도는 모두 자식 속도에서 월드 속도를 뺀 값이다.
            PxVec3 linear(0), angular(0);
            if (m_HasSupportTarget && deltaTime > 0)
            {
                const auto previous = m_SupportJoint->getDrivePosition();
                linear = LimitMagnitude((supportedPose.p - previous.p) / deltaTime, 10.0f);
                PxQuat delta = supportedPose.q * previous.q.getConjugate();

                if (delta.w < 0)
                {
                    delta = -delta;
                }
                PxReal angle;
                PxVec3 axis;
                delta.toRadiansAndUnitAxis(angle, axis);
                angular = axis * PxMin(angle / deltaTime, 20.0f);
            }
            m_SupportJoint->setDrivePosition(supportedPose);
            m_SupportJoint->setDriveVelocity(linear, angular);
            m_HasSupportTarget = true;
        }
        for (size_t i = 0; i < targets.size(); ++i)
        {
            const PxQuat q = ToPx(targets[i]);
            PxVec3 angular(0);
            const bool history = m_HasDriveTargets && m_DriveTargets.size() == targets.size() && deltaTime > 0;
            bool changed = !history || Angle(q, ToPx(m_DriveTargets[i])) > 1e-4f;
            const auto& data = m_Asset.constraints[i];
            if (m_DrivesEnabled && HasEnabledAngularDrive(data))
            {
                PxRigidActor* a;
                PxRigidActor* b;
                m_Joints[i]->getActors(a, b);
                const PxQuat parent = a->getGlobalPose().q * m_Joints[i]->getLocalPose(PxJointActorIndex::eACTOR0).q;
                const PxQuat child = b->getGlobalPose().q * m_Joints[i]->getLocalPose(PxJointActorIndex::eACTOR1).q;
                // 작은 팔다리는 관성이 작아 정지된 모터 목표에서 멀리 떨어져 있어도 PhysX의 에너지 기반
                // 수면 임계값을 만족할 수 있다. 이 경우에도 목표 오차를 계속 해결하도록 한다.
                changed |= Angle(parent.getConjugate() * child, q) > .03f;
            }
            if (history)
            {
                PxQuat delta = q * ToPx(m_DriveTargets[i]).getConjugate();
                if (delta.w < 0)
                {
                    delta = -delta;
                }
                PxReal angle;
                PxVec3 axis;
                delta.toRadiansAndUnitAxis(angle, axis);
                angular = axis * PxMin(angle / deltaTime, 20.0f);
                // 이 SDK의 SWING_TWIST 제약 행은 부모 속도에서 자식 속도를 뺀 값을 사용한다.
                // SLERP는 자식 속도에서 부모 속도를 뺀 값을 사용한다(ExtD6JointSolverPrep 참고).
                if (!data.useSlerpDrive)
                {
                    angular = -angular;
                }
            }
            m_Joints[i]->setDrivePosition(PxTransform(PxVec3(0), q), changed);
            m_Joints[i]->setDriveVelocity(PxVec3(0), angular, changed);
        }
        m_DriveTargets = std::move(targets);
        m_HasDriveTargets = true;
        error.clear();
        return true;
#else
        (void)boneWorld;
        error = "Drives require x64 PhysX.";
        return false;
#endif
    }

    bool Ragdoll::SetRootSupported(bool supported, std::string& error)
    {
#if defined(_M_X64)
        if (supported == IsRootSupported())
        {
            error.clear();
            return true;
        }
        int root = -1;
        if (supported)
        {
            if (!IsSimulating())
            {
                error = "Support requires a simulated ragdoll.";
                return false;
            }
            for (size_t i = 0; i < m_Asset.bodies.size(); ++i)
            {
                bool child = false;
                for (const auto& joint : m_Asset.constraints)
                {
                    child |= joint.childBone == m_Asset.bodies[i].bone;
                }
                if (!child)
                {
                    if (root >= 0)
                    {
                        error = "Pelvis support requires one root body.";
                        return false;
                    }
                    root = int(i);
                }
            }
            if (root < 0)
            {
                error = "PA has no root body to support.";
                return false;
            }
        }
        if (!supported)
        {
            // 드라이브를 제거해도 모든 바디의 포즈와 속도는 유지한다.
            m_SupportJoint->release();
            m_SupportJoint = nullptr;
            m_Bodies[m_SupportBody]->wakeUp();
            m_SupportBody = -1;
            m_HasSupportTarget = false;
            error.clear();
            return true;
        }
        auto* body = m_Bodies[root];
        auto* drive = PxD6JointCreate(body->getScene()->getPhysics(), nullptr, PxTransform(PxIdentity), body, PxTransform(PxIdentity));
        if (!drive)
        {
            error = "Could not create pelvis force/torque drive.";
            return false;
        }
        // 6개 축을 모두 자유롭게 둔다. 크기가 제한된 모터의 힘만으로 골반을 지지한다.
        // 이 런타임 조인트는 PA의 인체 관절 제약과 별개다.
        drive->setName("PelvisForceTorque");
        for (int axis = 0; axis < PxD6Axis::eCOUNT; ++axis)
        {
            drive->setMotion(PxD6Axis::Enum(axis), PxD6Motion::eFREE);
        }
        drive->setAngularDriveConfig(PxD6AngularDriveConfig::eSLERP);
        drive->setConstraintFlag(PxConstraintFlag::eDRIVE_LIMITS_ARE_FORCES, true);
        float mass = 0;
        for (const auto* part : m_Bodies)
        {
            mass += part->getMass();
        }
        // 래그돌 전체 질량에 비례해 초기 지지 게인을 설정한다. 선형 힘 제한은 월드 축마다 N 단위,
        // SLERP 토크 제한은 솔버의 각도 제약 행마다 Nm 단위다.
        const PxD6JointDrive position(mass * 900.0f, mass * 60.0f, mass * 60.0f, false);
        for (auto axis : {PxD6Drive::eX, PxD6Drive::eY, PxD6Drive::eZ})
        {
            drive->setDrive(axis, position);
        }
        drive->setDrive(PxD6Drive::eSLERP, PxD6JointDrive(mass * 200.0f, mass * 10.0f, mass * 20.0f, false));
        // 다음 애니메이션 목표가 들어올 때까지 현재 포즈를 유지한다. 지지를 다시 켤 때
        // 포즈가 갑자기 바뀌거나 유한차분 속도가 급증하는 것을 방지한다.
        drive->setDrivePosition(body->getGlobalPose());
        drive->setDriveVelocity(PxVec3(0), PxVec3(0));
        m_SupportJoint = drive;
        m_SupportBody = root;
        m_HasSupportTarget = false;
        body->wakeUp();
        error.clear();
        return true;
#else
        (void)supported;
        error = "Support requires x64 PhysX.";
        return false;
#endif
    }

    float Ragdoll::GetRootPositionError() const
    {
#if defined(_M_X64)
        if (m_SupportJoint)
        {
            return (m_SupportJoint->getDrivePosition().p - m_Bodies[m_SupportBody]->getGlobalPose().p).magnitude();
        }
#endif
        return 0;
    }

    float Ragdoll::GetRootRotationError() const
    {
#if defined(_M_X64)
        if (m_SupportJoint)
        {
            return Angle(m_SupportJoint->getDrivePosition().q, m_Bodies[m_SupportBody]->getGlobalPose().q);
        }
#endif
        return 0;
    }

    size_t Ragdoll::GetDrivenJointCount() const
    {
        size_t count = 0;
#if defined(_M_X64)
        if (m_DrivesEnabled)
        {
            for (const auto& joint : m_Asset.constraints)
            {
                if (HasEnabledAngularDrive(joint))
                {
                    ++count;
                }
            }
        }
#endif
        return count;
    }

    float Ragdoll::GetMaxDriveError(size_t* worstJoint) const
    {
        float result = 0;
        if (worstJoint)
        {
            *worstJoint = 0;
        }
#if defined(_M_X64)
        if (!m_DrivesEnabled || !m_HasDriveTargets)
        {
            return result;
        }
        for (size_t i = 0; i < m_Joints.size(); ++i)
        {
            const auto& data = m_Asset.constraints[i];
            if (!HasEnabledAngularDrive(data))
            {
                continue;
            }
            PxRigidActor* a;
            PxRigidActor* b;
            m_Joints[i]->getActors(a, b);
            const PxQuat parent = a->getGlobalPose().q * m_Joints[i]->getLocalPose(PxJointActorIndex::eACTOR0).q;
            const PxQuat child = b->getGlobalPose().q * m_Joints[i]->getLocalPose(PxJointActorIndex::eACTOR1).q;
            const float error = Angle(parent.getConjugate() * child, ToPx(m_DriveTargets[i]));
            if (error > result)
            {
                result = error;
                if (worstJoint)
                {
                    *worstJoint = i;
                }
            }
        }
#endif
        return result;
    }
}
