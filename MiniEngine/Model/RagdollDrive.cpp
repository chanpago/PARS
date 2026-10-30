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
                // 기존 PA의 힘 모드는 D6에서 유지한다. Articulation 모터는 아래에서 별도로 갱신한다.
                const bool on = !m_Articulation && m_DrivesEnabled && Enabled(drive, motions[axis]);
                PxD6JointDrive motor;
                if (on)
                {
                    motor = PxD6JointDrive(drive.GetStiffness(), drive.GetDamping(), drive.maxTorque, drive.useAcceleration);
                }
                m_Joints[i]->setDrive(data.useSlerpDrive ? PxD6Drive::eSLERP : axes[axis], motor);
            }
        }
        if (m_Articulation)
        {
            m_Articulation->setArticulationFlag(PxArticulationFlag::eDISABLE_SELF_COLLISION, m_DrivesEnabled);
            m_Articulation->setSolverIterationCounts(32, 8);
            UpdateArticulationDrives();
        }
        else
        {
            for (auto* body : m_Bodies)
            {
                body->is<PxRigidDynamic>()->setSolverIterationCounts(m_DrivesEnabled ? 32 : 16, m_DrivesEnabled ? 8 : 4);
            }
        }
        WakeBodies();
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
        const auto previous = m_Asset;
        for (size_t i = 0; i < candidate.constraints.size(); ++i)
        {
            m_Asset.constraints[i].useSlerpDrive = candidate.constraints[i].useSlerpDrive;
            for (int axis = 0; axis < 3; ++axis)
            {
                m_Asset.constraints[i].drives[axis] = candidate.constraints[i].drives[axis];
            }
        }
        if (IsSimulating() && m_Asset.UsesArticulation() != UsesArticulation())
        {
            if (!RebuildSimulation(true, Vector3(kZero), error, false))
            {
                for (size_t i = 0; i < previous.constraints.size(); ++i)
                {
                    m_Asset.constraints[i].useSlerpDrive = previous.constraints[i].useSlerpDrive;
                    for (int axis = 0; axis < 3; ++axis)
                    {
                        m_Asset.constraints[i].drives[axis] = previous.constraints[i].drives[axis];
                    }
                }
                return false;
            }
        }
        if (!PrepareStandby(error))
        {
            for (size_t i = 0; i < previous.constraints.size(); ++i)
            {
                m_Asset.constraints[i].useSlerpDrive = previous.constraints[i].useSlerpDrive;
                for (int axis = 0; axis < 3; ++axis)
                {
                    m_Asset.constraints[i].drives[axis] = previous.constraints[i].drives[axis];
                }
            }
            return false;
        }
        SetDrivesEnabled(m_DrivesEnabled);
        error.clear();
        return true;
    }

    void Ragdoll::ResetDriveTargetHistory()
    {
        m_HasDriveTargets = false;
        m_HasSupportTarget = false;
        m_PreviousArticulationTargetVelocity.clear();
    }

    bool Ragdoll::UpdateDriveTargets(const std::vector<Matrix4>& boneWorld, float deltaTime, std::string& error)
    {
        // 애니메이션 본의 월드 목표 자세를 PA 바디와 관절의 목표로 변환해 PhysX에 전달한다.
        // 실제 몸체의 움직임은 이 함수가 끝난 뒤 PhysicsWorld::Step()의 simulate()에서 계산된다.
        // IsActive()는 물리 바디가 생성되어 있는지 확인한다. 액티브 래그돌 모드 검사와는 다르다.
        if (!IsActive() || !std::isfinite(deltaTime) || deltaTime < 0)
        {
            error = "Invalid drive update.";
            return false;
        }
#if defined(_M_X64)

        // A target replacement/reset has no elapsed time. Unlike a support-weight edit,
        // it must discard old target velocities so the next step cannot invent acceleration.
        if (deltaTime == 0)
        {
            m_PreviousArticulationTargetVelocity.clear();
        }

        // [1] 본의 월드 목표 자세 -> PA 물리 바디의 월드 목표 자세
        // m_Bodies는 실제 PhysX 바디들, 아래 bodies는 이번 스텝에 계산할 목표 행렬 배열이다.
        std::vector<Matrix4> bodies;

        // reserve는 메모리만 확보한다. 각 목표 행렬은 반복문 끝의 push_back으로 추가한다.
        bodies.reserve(m_Bodies.size());

        // 골반 지지 드라이브에 전달할 목표 위치/회전. 지지 바디를 찾으면 아래에서 채운다.
        PxTransform supportedPose(PxIdentity);
        for (size_t i = 0; i < m_Bodies.size(); ++i)
        {
            // i는 물리 바디 번호, node는 그 바디가 연결된 모델의 본 노드 번호다.
            // 모든 본에 물리 바디가 있는 것은 아니므로 두 번호를 구분해야 한다.
            const int node = m_BoneNodes[i];
            if (node < 0 || size_t(node) >= boneWorld.size())
            {
                error = "Drive target has no matching bone.";
                return false;
            }

            // 본의 월드 행렬에서 스케일/전단을 제거하고 위치/회전으로 된 강체 좌표계를 만든다.
            // 여기에 PA의 본 기준 바디 offset/rotation을 곱해 바디의 월드 목표 자세를 구한다.
            // 예: 어깨에 있는 상완 본의 기준점에서 팔 중간에 있는 캡슐 중심으로 옮기는 변환이다.
            const auto transform = PhysicsBoneFrame(boneWorld[node]) * m_Asset.bodies[i].GetLocalTransform();

            // 목표 행렬에서 월드 위치(frame.offset)와 회전(frame.rotation)을 꺼낸다.
            // PhysicsJointFrame 구조체를 사용하는 것이며, 여기서 관절을 생성하지는 않는다.
            const auto frame = PhysicsJointFrame::FromTransform(transform);

            // 목표 회전이 PhysX에서 사용할 수 있는 정상적인 쿼터니언인지 검사한다.
            if (!ToPx(Quaternion(DirectX::XMLoadFloat4(&frame.rotation))).isSane())
            {
                error = "Invalid drive target rotation.";
                return false;
            }
            // 지지 대상으로 선택한 루트 바디(현재 캐릭터의 골반)는 목표를 따로 보관한다.
            // 지지가 꺼져 있으면 m_SupportBody == -1이므로 이 분기를 건너뛴다.
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
            // bodies[i]에는 실제 현재 자세가 아니라 애니메이션을 따라갈 바디 목표 자세가 들어간다.
            bodies.push_back(transform);
        }


        // [2] 부모/자식 바디의 목표 자세 -> 관절 좌표계에서의 목표 상대 회전
        // targets의 인덱스는 바디 번호가 아니라 PA 관절 번호다.
        std::vector<Quaternion> targets;
        targets.reserve(m_Joints.size());
        m_MaxTargetClampAngle = 0;
        for (const auto& data : m_Asset.constraints)
        {
            const auto& a = bodies[m_Asset.FindBody(data.parentBone)];
            const auto& b = bodies[m_Asset.FindBody(data.childBone)];

            // a, b에 PA의 부모/자식 관절 프레임을 반영해 두 관절 프레임 사이의 회전을 구한다.
            // D6 경로는 이 함수에서 제한을 적용하고, Articulation 경로는 아래에서 별도로 제한한다.
            // Articulation이라면 EvaluateJointDriveTarget에서 부모 조인트 좌표계 기준 자식 조인트의 회전 정도를 쿼터니언으로 정규화한다.
            Math::Quaternion target = EvaluateJointDriveTarget(data, a, b, !m_Articulation);
            if (m_Articulation)
            {
                // 잠긴 축과 각 축의 허용 회전 범위를 반영해 구동 목표를 제한한다.
                target = ClampArticulationDriveTarget(data, target);
            }
            if (!ToPx(target).isSane())
            {
                error = "Non-finite joint drive target.";
                return false;
            }
            targets.push_back(target);
            // 원래 애니메이션 목표에서 얼마나 제한됐는지 진단할 최대 각도 차이를 기록한다.
            m_MaxTargetClampAngle = PxMax(m_MaxTargetClampAngle, Angle(ToPx(target), ToPx(EvaluateJointDriveTarget(data, a, b, false))));
        }


        // [3] 골반 지지 드라이브의 월드 목표 자세와 목표 속도 갱신
        if (m_SupportJoint)
        {
            // 골반은 동적 상태를 유지한다. 월드에 연결된 D6 드라이브가 힘/토크를 가하며,
            // 다른 조인트와 접촉 제약도 같은 암시적 풀이 과정에서 함께 계산한다.
            // 선형/SLERP 드라이브의 두 속도는 모두 자식 속도에서 월드 속도를 뺀 값이다.
            PxVec3 linear(0), angular(0);
            // 이전 목표와 이번 목표의 차이 / dt로 목표 선속도와 각속도를 구한다.
            // 실제 골반 속도를 읽는 단계가 아니다. 첫 목표이거나 dt가 0이면 목표 속도는 0이다.
            if (m_HasSupportTarget && deltaTime > 0)
            {
                const auto previous = m_SupportJoint->getDrivePosition();
                // 목표가 급변할 때 속도가 과도하게 커지지 않도록 선속도를 10 m/s로 제한한다.
                linear = LimitMagnitude((supportedPose.p - previous.p) / deltaTime, 10.0f);
                PxQuat delta = supportedPose.q * previous.q.getConjugate();

                // q와 -q는 같은 회전이다. 짧은 방향의 회전 차이를 사용하도록 부호를 맞춘다.
                if (delta.w < 0)
                {
                    delta = -delta;
                }
                PxReal angle;
                PxVec3 axis;
                delta.toRadiansAndUnitAxis(angle, axis);
                // 회전 차이를 축/각도로 풀어 목표 각속도를 구하고 20 rad/s로 제한한다.
                angular = axis * PxMin(angle / deltaTime, 20.0f);
            }
            // 동적 골반이 따라갈 목표를 설정한다. 골반을 이 위치로 순간이동시키는 호출은 아니다.
            m_SupportJoint->setDrivePosition(supportedPose);
            m_SupportJoint->setDriveVelocity(linear, angular);
            m_HasSupportTarget = true;
        }


        // [4] 관절별 목표 회전/각속도를 PhysX 드라이브에 전달
		// targets에는 각 관절의 목표 회전이 들어 있다(쿼터니언, 부모관절 기준 상대적 차이). 목표 각속도는 이전 목표와 이번 목표의 차이를 시간으로 나눠 계산한다.
        for (size_t i = 0; i < targets.size(); ++i)
        {
            const PxQuat q = ToPx(targets[i]);
            PxVec3 angular(0);
            // 이전 관절 목표가 있어야 시간 차분으로 목표 각속도를 계산할 수 있다.
            const bool history = m_HasDriveTargets && m_DriveTargets.size() == targets.size() && deltaTime > 0;
            // 목표가 바뀌었거나 실제 자세가 목표에서 멀면 잠든 물리 객체도 다시 깨운다.
            bool changed = !history || Angle(q, ToPx(m_DriveTargets[i])) > 1e-4f;
            const auto& data = m_Asset.constraints[i];
            if (m_DrivesEnabled && HasEnabledAngularDrive(data))
            {
                PxRigidActor* a;
                PxRigidActor* b;
                m_Joints[i]->getActors(a, b);
                // 목표 바디 배열과 달리 여기서는 실제 물리 바디의 현재 관절 프레임을 읽는다.
                const PxQuat parent = a->getGlobalPose().q * m_Joints[i]->getLocalPose(PxJointActorIndex::eACTOR0).q;
                const PxQuat child = b->getGlobalPose().q * m_Joints[i]->getLocalPose(PxJointActorIndex::eACTOR1).q;
                // 작은 팔다리는 관성이 작아 정지된 모터 목표에서 멀리 떨어져 있어도 PhysX의 에너지 기반
                // 수면 임계값을 만족할 수 있다. 이 경우에도 목표 오차를 계속 해결하도록 한다.
                changed |= Angle(parent.getConjugate() * child, q) > .03f;
            }
            // 두 목표 회전의 차이로 추종할 각속도를 구한다. 현재 자세 오차와는 구분한다.
            if (history)
            {
                // 이 식이 나오는 이유는 생각해보면,
				// 이번목표 = delta * 이전목표 이니까(delta는 차이) 
                // 이번목표 * 이전목표역회전 = delta
				// q가 이번목표 , m_DriveTargets[i]가 이전목표
                PxQuat delta = q * ToPx(m_DriveTargets[i]).getConjugate();

                // 같은 회전 변화라도, 더 짧게 도는 방향으로 각속도를 계산하려는것. 의도적으로 그런것이다.
                // 현재처럼 한 고정 스텝 동안 목표가 조금씩 변하는 상황에서는 짧은 쪽으로 계산하는게 맞다
                // w가 0보다 작으면 각도가 180도 이상이라는 뜻이므로 이때 모든 성분의 부호를 뒤집으면 같은 회전을 반대축으로 180이하만큼 도는 표현으로 바꿀 수 있다.
                // 간단히 쿼터니언에 대해 정리하자면, 어느 축을 중심으로 몇도를 회전할지를 쿼터니언의 네 성분(x,y,z,w)로 표현함.
				// 회전축이 길이 인 벡터 u = (ux, uy, uz)이고 회전각이 θ라면, 쿼터니언은 다음과 같이 표현됨:
				// x = ux * sin(θ/2), y = uy * sin(θ/2), z = uz * sin(θ/2), w = cos(θ/2)
                // 즉, xyz에는 회전축 방향을 담고, xyz의 크기와 w에 회전량을 담는 구조 
                if (delta.w < 0)
                {
                    delta = -delta;
                }
                PxReal angle;
                PxVec3 axis;

                // 이전 목표와 이번 목표 사이의 회전 변화량을 쿼터니언으로 계산
                // 거기서 회전 축과 회전각을 꺼냄
                // 회전각을 경과 시간으로 나눠 xyz 성분의 목표 각속도 벡터를 만듦
                //여기서 변화량은 실제 바디가 회전한 양이 아니라, 애니메이션 목표  회전이 변한 양
                // 이걸 반으로 3축의 각속도에 대한 벡터를 angular에 저장한다
                delta.toRadiansAndUnitAxis(angle, axis);

                // 회전 차이를 축/각도로 풀어 목표 각속도를 구하고 20 rad/s로 제한한다.
                angular = axis * PxMin(angle / deltaTime, 20.0f);

                // 이 SDK의 SWING_TWIST 제약 행은 부모 속도에서 자식 속도를 뺀 값을 사용한다.
                // SLERP는 자식 속도에서 부모 속도를 뺀 값을 사용한다(ExtD6JointSolverPrep 참고).
                if (!m_Articulation && !data.useSlerpDrive)
                {
                    angular = -angular;
                }
            }
            if (m_Articulation)
            {
                // Articulation이 사용하는 좌표계로 변환해 3축 목표 각도/각속도를 설정한다.
                // 위에서 구한 angular를 기준으로 세팅한다

                SetArticulationTarget(i, targets[i], Vector3(angular.x, angular.y, angular.z));
                if (changed)
                {
                    m_Articulation->wakeUp();
                }
            }
            // D6 경로의 관절 목표도 설정한다. Articulation 경로의 D6는 진단 좌표계용이다.
            m_Joints[i]->setDrivePosition(PxTransform(PxVec3(0), q), changed);
            m_Joints[i]->setDriveVelocity(PxVec3(0), angular, changed);
        }
        // 다음 고정 스텝에서 목표 각속도를 계산할 수 있도록 이번 목표를 보관한다.
        m_DriveTargets = std::move(targets);
        m_HasDriveTargets = true;



        //////////////////////////////////////////
        // 1. 각 물리 바디의 월드 목표 위치·회전 계산 → bodies 배열에 저장.
        // 2. 각 관절의 목표 상대 회전 계산·제한 → 쿼터니언으로 targets에 저장.
        // 3. 골반 지지가 켜져 있으면 → 골반 지지 드라이브에 목표 위치·회전과 목표 선속도·각속도 전달.
        // 4. 각 관절 드라이브에 목표 회전과 목표 각속도 전달 → 각속도는 이전 목표와 이번 목표의 차이로 계산한 PxVec3.
        // 단, 아티큘레이션에서는 4번 내부에서 쿼터니언과 각속도 벡터를 해당 관절 좌표계에 맞춰 변환하고, 축별 목표값으로 전달.
        //////////////////////////////////////////

        // [5] Articulation의 PD 구동 설정과 추가 보상 토크 갱신 (D6 전용 경로는 내부에서 반환)
        // 관절 PD의 강성/감쇠 등을 설정한다. 관절 PD 토크 자체는 이후 PhysX가 계산한다.
        // 드라이브와 골반 지지가 켜져 있으면 중력 + 코리올리/원심력 + 목표 가속도 관성 토크를 계산한다.
        // 관절의 추가 보상은 cache.jointForce에 넣고 applyCache로 전달한다. 관절 PD 토크와 직접 합산하지 않는다.
        // 골반 회전 PD는 이 함수 내부에서 목표 가속도에 반영하는 별도 처리가 있다.
        UpdateArticulationDrives(deltaTime);
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
        return SetRootSupport(supported ? 1.0f : 0.0f, true, error);
    }

    bool Ragdoll::SetRootSupport(float weight, bool horizontal, std::string& error)
    {
#if defined(_M_X64)
        if (!std::isfinite(weight) || weight < 0 || weight > 1)
        {
            error = "Support weight must be finite and in [0, 1].";
            return false;
        }
        const bool supported = weight > 0;
        if (supported == IsRootSupported() && (!supported ||
            (weight == m_RootSupportWeight && horizontal == m_SupportHorizontal)))
        {
            error.clear();
            return true;
        }
        if (supported && IsRootSupported())
        {
            if (horizontal != m_SupportHorizontal)
            {
                m_PreviousArticulationTargetVelocity.clear();
                m_HasSupportTarget = false;
            }
            m_RootSupportWeight = weight;
            m_SupportHorizontal = horizontal;
            ConfigureRootSupportDrives();
            UpdateArticulationDrives();
            WakeBodies();
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
            WakeBodies();
            m_SupportBody = -1;
            m_HasSupportTarget = false;
            m_RootSupportWeight = 0;
            m_PreviousArticulationTargetVelocity.clear();
            UpdateArticulationDrives();
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
        // 6개 축을 모두 자유롭게 두고 모터로 골반을 지지한다.
        // 이 런타임 조인트는 PA의 인체 관절 제약과 별개다.
        drive->setName("PelvisForceTorque");
        for (int axis = 0; axis < PxD6Axis::eCOUNT; ++axis)
        {
            drive->setMotion(PxD6Axis::Enum(axis), PxD6Motion::eFREE);
        }
        drive->setAngularDriveConfig(PxD6AngularDriveConfig::eSLERP);
        drive->setConstraintFlag(PxConstraintFlag::eDRIVE_LIMITS_ARE_FORCES, true);
        // 다음 애니메이션 목표가 들어올 때까지 현재 포즈를 유지한다. 지지를 다시 켤 때
        // 포즈가 갑자기 바뀌거나 유한차분 속도가 급증하는 것을 방지한다.
        drive->setDrivePosition(body->getGlobalPose());
        drive->setDriveVelocity(PxVec3(0), PxVec3(0));
        m_SupportJoint = drive;
        m_SupportBody = root;
        m_RootSupportWeight = weight;
        m_SupportHorizontal = horizontal;
        m_HasSupportTarget = false;
        m_PreviousArticulationTargetVelocity.clear();
        ConfigureRootSupportDrives();
        WakeBodies();
        UpdateArticulationDrives();
        error.clear();
        return true;
#else
        (void)weight;
        (void)horizontal;
        error = "Support requires x64 PhysX.";
        return false;
#endif
    }

    void Ragdoll::ConfigureRootSupportDrives()
    {
#if defined(_M_X64)
        if (!m_SupportJoint)
        {
            return;
        }
        float mass = 0;
        for (const auto* part : m_Bodies)
        {
            mass += part->getMass();
        }
        const float w = m_RootSupportWeight;
        // Scale gains and force capacity, never the world-space target position.
        const PxD6JointDrive position(mass * PelvisLinearFrequency * PelvisLinearFrequency * w,
            2.0f * mass * PelvisLinearDampingRatio * PelvisLinearFrequency * w, mass * 60.0f * w, false);
        m_SupportJoint->setDrive(PxD6Drive::eX, m_SupportHorizontal ? position : PxD6JointDrive());
        m_SupportJoint->setDrive(PxD6Drive::eY, position);
        m_SupportJoint->setDrive(PxD6Drive::eZ, m_SupportHorizontal ? position : PxD6JointDrive());
        // Active articulations use the coupled mass-matrix PD in UpdateArticulationDrives.
        const PxD6JointDrive rotation = m_Articulation ? (m_DrivesEnabled ? PxD6JointDrive() :
            PxD6JointDrive(PelvisAngularFrequency * PelvisAngularFrequency * w,
                2.0f * PelvisAngularDampingRatio * PelvisAngularFrequency * w, PX_MAX_F32 * w, true)) :
            PxD6JointDrive(mass * 200.0f * w, mass * 10.0f * w, mass * 20.0f * w, false);
        m_SupportJoint->setDrive(PxD6Drive::eSLERP, rotation);
#endif
    }

    float Ragdoll::GetRootTargetHeight() const
    {
#if defined(_M_X64)
        if (m_SupportJoint)
        {
            return m_SupportJoint->getDrivePosition().p.y;
        }
#endif
        return float(GetPosition().GetY());
    }

    float Ragdoll::GetRootPositionError() const
    {
#if defined(_M_X64)
        if (m_SupportJoint)
        {
            const auto error = m_SupportJoint->getDrivePosition().p - m_Bodies[m_SupportBody]->getGlobalPose().p;
            return m_SupportHorizontal ? error.magnitude() : PxAbs(error.y);
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
