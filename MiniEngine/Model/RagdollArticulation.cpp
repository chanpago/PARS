#include "Ragdoll.h"
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
        PxVec3 ToPx(Vector3 v)
        {
            return PxVec3(float(v.GetX()), float(v.GetY()), float(v.GetZ()));
        }

        PxTransform ToPx(const Matrix4& value)
        {
            const auto frame = PhysicsJointFrame::FromTransform(value);
            return PxTransform(PxVec3(frame.offset.x, frame.offset.y, frame.offset.z),
                PxQuat(frame.rotation.x, frame.rotation.y, frame.rotation.z, frame.rotation.w));
        }

        PxVec3 RotationVector(PxQuat rotation)
        {
            if (rotation.w < 0)
            {
                rotation = -rotation;
            }
            float angle;
            PxVec3 axis;
            rotation.toRadiansAndUnitAxis(angle, axis);
            return axis * angle;
        }

        constexpr PxArticulationAxis::Enum AngularAxes[] = {
            PxArticulationAxis::eTWIST, PxArticulationAxis::eSWING1, PxArticulationAxis::eSWING2};

        // 다른 구동 축은 자세를 유지한다고 근사하고, 자유로운 베이스/무구동 축은 제거한다.
        // I = Mii - MiF * inverse(MFF) * MFi. 질량 배율이 아니라 실제 관절 위치와 바디 관성에서 나온다.
        std::vector<float> ControlInertias(const float* massMatrix, unsigned count, const std::vector<unsigned>& freeAxes)
        {
            const size_t freeCount = freeAxes.size();
            std::vector<double> lower(freeCount * freeCount, 0);
            for (size_t row = 0; row < freeCount; ++row)
            {
                for (size_t col = 0; col <= row; ++col)
                {
                    double value = massMatrix[freeAxes[row] * count + freeAxes[col]];
                    for (size_t k = 0; k < col; ++k)
                    {
                        value -= lower[row * freeCount + k] * lower[col * freeCount + k];
                    }
                    lower[row * freeCount + col] = row == col ? std::sqrt((std::max)(value, 1e-12)) : value / lower[col * freeCount + col];
                }
            }
            std::vector<float> result(count);
            std::vector<double> projected(freeCount);
            for (unsigned axis = 0; axis < count; ++axis)
            {
                double inertia = massMatrix[axis * count + axis];
                for (size_t row = 0; row < freeCount; ++row)
                {
                    double value = massMatrix[freeAxes[row] * count + axis];
                    for (size_t col = 0; col < row; ++col)
                    {
                        value -= lower[row * freeCount + col] * projected[col];
                    }
                    projected[row] = value / lower[row * freeCount + row];
                    inertia -= projected[row] * projected[row];
                }
                result[axis] = static_cast<float>((std::max)(inertia, 0.0));
            }
            return result;
        }
    }
#endif

    bool Ragdoll::CreateArticulation(PhysicsWorld& world, const std::vector<Matrix4>& boneWorld, std::string& error)
    {
#if defined(_M_X64)
        const size_t count = m_Asset.bodies.size();
        std::vector<int> parent(count, -1), inbound(count, -1);
        for (size_t i = 0; i < m_Asset.constraints.size(); ++i)
        {
            const auto& joint = m_Asset.constraints[i];
            const int child = m_Asset.FindBody(joint.childBone);
            if (parent[child] >= 0)
            {
                error = "Articulation requires exactly one parent per body.";
                return false;
            }
            parent[child] = m_Asset.FindBody(joint.parentBone);
            inbound[child] = static_cast<int>(i);
        }
        for (size_t i = 0; i < count; ++i)
        {
            if (parent[i] < 0)
            {
                if (m_ArticulationRoot >= 0)
                {
                    error = "Articulation requires a connected PA with one root.";
                    return false;
                }
                m_ArticulationRoot = static_cast<int>(i);
            }
        }
        if (m_ArticulationRoot < 0 || m_Asset.constraints.size() + 1 != count)
        {
            error = "Articulation requires an acyclic PA tree.";
            return false;
        }
        m_Articulation = world.CreateArticulation();
        if (!m_Articulation)
        {
            error = "Could not create articulated ragdoll.";
            return false;
        }
        m_Articulation->setName("RagdollArticulation");
        m_Articulation->setArticulationFlag(PxArticulationFlag::eDRIVE_LIMITS_ARE_FORCES, true);
        m_Articulation->setSolverIterationCounts(16, 4);
        m_Bodies.resize(count, nullptr);
        m_BoneNodes.resize(count);
        m_ArticulationJoints.resize(m_Asset.constraints.size(), nullptr);
        size_t created = 0;
        while (created < count)
        {
            const size_t previous = created;
            for (size_t i = 0; i < count; ++i)
            {
                if (m_Bodies[i] || (parent[i] >= 0 && !m_Bodies[parent[i]]))
                {
                    continue;
                }
                const auto& body = m_Asset.bodies[i];
                const int node = static_cast<int>(std::find(m_BoneNames.begin(), m_BoneNames.end(), body.bone) - m_BoneNames.begin());
                const auto pose = ToPx(PhysicsBoneFrame(boneWorld[node]) * body.GetLocalTransform());
                PxGeometryHolder geometry;
                switch (body.shape)
                {
                case PhysicsShapeType::Capsule:
                    geometry.storeAny(PxCapsuleGeometry(body.radius, body.halfLength));
                    break;
                case PhysicsShapeType::Sphere:
                    geometry.storeAny(PxSphereGeometry(body.radius));
                    break;
                case PhysicsShapeType::Box:
                    geometry.storeAny(PxBoxGeometry(body.halfExtents.x, body.halfExtents.y, body.halfExtents.z));
                    break;
                }
                auto* parentLink = parent[i] < 0 ? nullptr : m_Bodies[parent[i]]->is<PxArticulationLink>();
                auto* link = world.CreateArticulationLink(*m_Articulation, parentLink, pose, geometry.any(), body.mass, body.bone.c_str());
                if (!link)
                {
                    error = "Could not create articulation link: " + body.bone;
                    return false;
                }
                m_Bodies[i] = link;
                m_BoneNodes[i] = node;
                ++created;
                if (!parentLink)
                {
                    continue;
                }
                auto* joint = link->getInboundJoint();
                const auto& data = m_Asset.constraints[inbound[i]];
                const auto parentFrame = ToPx(data.parentFrame.GetTransform());
                const auto childFrame = ToPx(data.childFrame.GetTransform());
                joint->setParentPose(parentFrame);
                joint->setChildPose(childFrame);
                joint->setJointType(PxArticulationJointType::eSPHERICAL);
                joint->setName(data.childBone.c_str());
                // 이 SDK는 새 마찰값을 0으로 설정해도 구형 기본 마찰(0.05)이 남는다.
                // PA에 없는 정지 마찰을 추가하지 않고, 제동은 명시한 PD 감쇠로 처리한다.
#pragma warning(push)
#pragma warning(disable : 4996)
                joint->setFrictionCoefficient(0);
#pragma warning(pop)
                // 현재 PhysX의 구면 관절은 회전 벡터 좌표와 축별 제한을 쓴다.
                const PxVec3 position = RotationVector((parentLink->getGlobalPose() * parentFrame).q.getConjugate() * (pose * childFrame).q);
                const PhysicsJointMotion motions[] = {data.twist, data.swingY, data.swingZ};
                const float lower[] = {data.twistMin, -data.swingYLimit, -data.swingZLimit};
                const float upper[] = {data.twistMax, data.swingYLimit, data.swingZLimit};
                for (int axis = 0; axis < 3; ++axis)
                {
                    // 잠긴 축도 3축 구면 관절의 내부 제한으로 처리해 회전 좌표의 순서를 유지한다.
                    const auto motion = motions[axis] == PhysicsJointMotion::Free ? PxArticulationMotion::eFREE : PxArticulationMotion::eLIMITED;
                    joint->setMotion(AngularAxes[axis], motion);
                    const bool locked = motions[axis] == PhysicsJointMotion::Locked;
                    joint->setLimitParams(AngularAxes[axis], PxArticulationLimit(locked ? -1e-6f : lower[axis], locked ? 1e-6f : upper[axis]));
                    joint->setMaxJointVelocity(AngularAxes[axis], 20.0f);
                    joint->setFrictionParams(AngularAxes[axis], PxJointFrictionParams(0, 0, 0));
                    joint->setJointPosition(AngularAxes[axis], position[axis]);
                    joint->setDriveTarget(AngularAxes[axis], position[axis]);
                }
                m_ArticulationJoints[inbound[i]] = joint;
            }
            if (created == previous)
            {
                error = "Articulation PA contains a cycle or disconnected bodies.";
                return false;
            }
        }
        if (!world.AddArticulation(*m_Articulation))
        {
            error = "Could not register articulation in the physics world.";
            return false;
        }
        m_Articulation->updateKinematic(PxArticulationKinematicFlag::ePOSITION);
        m_ArticulationCache = m_Articulation->createCache();
        if (!m_ArticulationCache)
        {
            error = "Could not allocate articulation dynamics cache.";
            return false;
        }
        // PhysX의 내부 링크 순서는 PA의 배열 순서와 다를 수 있다.
        std::vector<unsigned> dofs(count), offsets(count);
        for (const auto* body : m_Bodies)
        {
            const auto* link = body->is<PxArticulationLink>();
            dofs[link->getLinkIndex()] = link->getInboundJointDof();
        }
        for (size_t i = 1; i < count; ++i)
        {
            offsets[i] = offsets[i - 1] + dofs[i - 1];
        }
        for (const auto& joint : m_Asset.constraints)
        {
            const auto* link = m_Bodies[m_Asset.FindBody(joint.childBone)]->is<PxArticulationLink>();
            m_ArticulationDofOffsets.push_back(offsets[link->getLinkIndex()]);
        }
        return true;
#else
        (void)world;
        (void)boneWorld;
        error = "Articulation requires x64 PhysX.";
        return false;
#endif
    }

    bool Ragdoll::RebuildSimulation(bool simulate, Vector3 velocity, std::string& error, bool inheritAnimationVelocity)
    {
#if defined(_M_X64)
        if (!ToPx(velocity).isFinite())
        {
            error = "Invalid initial velocity.";
            return false;
        }
        std::vector<Matrix4> pose(m_BoneNames.size(), Matrix4(kIdentity));
        std::vector<Vector3> linear(m_Bodies.size(), velocity), angular(m_Bodies.size(), Vector3(kZero));
        for (size_t i = 0; i < m_Bodies.size(); ++i)
        {
            pose[m_BoneNodes[i]] = GetBoneWorld(i);
            if (IsSimulating())
            {
                const auto v = m_Bodies[i]->getLinearVelocity();
                const auto w = m_Bodies[i]->getAngularVelocity();
                linear[i] = Vector3(v.x, v.y, v.z);
                angular[i] = Vector3(w.x, w.y, w.z);
            }
            else if (inheritAnimationVelocity && m_AnimatedLinearVelocity.size() == m_Bodies.size())
            {
                linear[i] = m_AnimatedLinearVelocity[i];
                angular[i] = m_AnimatedAngularVelocity[i];
            }
        }
        Ragdoll candidate;
        if (!candidate.Create(*m_World, m_Asset, m_BoneNames, pose, velocity, error, simulate))
        {
            return false;
        }
        candidate.SetQueryOwner(m_QueryOwner);
        candidate.SetCollisionEnabled(m_CollisionEnabled);
        if (simulate)
        {
            candidate.SetArticulationVelocities(linear, angular);
            candidate.SetDrivesEnabled(m_DrivesEnabled);
            if (IsRootSupported() && !candidate.SetRootSupported(true, error))
            {
                return false;
            }
        }
        Swap(candidate);
        error.clear();
        return true;
#else
        (void)simulate;
        (void)velocity;
        (void)inheritAnimationVelocity;
        error = "Articulation requires x64 PhysX.";
        return false;
#endif
    }

    void Ragdoll::SetArticulationVelocities(const std::vector<Vector3>& linear, const std::vector<Vector3>& angular)
    {
#if defined(_M_X64)
        if (!m_Articulation)
        {
            for (size_t i = 0; i < m_Bodies.size(); ++i)
            {
                auto* body = m_Bodies[i]->is<PxRigidDynamic>();
                body->setLinearVelocity(ToPx(linear[i]));
                body->setAngularVelocity(ToPx(angular[i]));
            }
            return;
        }
        m_Articulation->setRootLinearVelocity(ToPx(linear[m_ArticulationRoot]));
        m_Articulation->setRootAngularVelocity(ToPx(angular[m_ArticulationRoot]));
        for (size_t i = 0; i < m_ArticulationJoints.size(); ++i)
        {
            const auto& data = m_Asset.constraints[i];
            const int parent = m_Asset.FindBody(data.parentBone), child = m_Asset.FindBody(data.childBone);
            auto* joint = m_ArticulationJoints[i];
            const auto frame = m_Bodies[child]->getGlobalPose() * joint->getChildPose();
            const auto relative = frame.q.rotateInv(ToPx(angular[child] - angular[parent]));
            for (int axis = 0; axis < 3; ++axis)
            {
                joint->setJointVelocity(AngularAxes[axis], relative[axis]);
            }
        }
        m_Articulation->updateKinematic(PxArticulationKinematicFlag::eVELOCITY);
#else
        (void)linear;
        (void)angular;
#endif
    }

    void Ragdoll::WakeBodies()
    {
#if defined(_M_X64)
        if (m_Articulation)
        {
            m_Articulation->wakeUp();
            return;
        }
        for (auto* body : m_Bodies)
        {
            if (!(body->getRigidBodyFlags() & PxRigidBodyFlag::eKINEMATIC))
            {
                body->is<PxRigidDynamic>()->wakeUp();
            }
        }
#endif
    }

    void Ragdoll::SetArticulationTarget(size_t index, Quaternion rotation, Vector3 angularVelocity)
    {
#if defined(_M_X64)
        DirectX::XMFLOAT4 q;
        DirectX::XMStoreFloat4(&q, rotation);
        const auto& data = m_Asset.constraints[index];
        const auto* joint = m_ArticulationJoints[index];
        const auto* childBody = m_Bodies[m_Asset.FindBody(data.childBone)];
        // 이 SDK의 spherical drive는 목표 벡터를 내부 motion matrix로 한 번 더 회전한다.
        // DyFeatherstoneArticulation::setupInternalConstraintsRecursive의 driveAxis 계산을 보정한다.
        // 관절 위치/제한 좌표는 그대로 두고, 드라이브에 전달하는 벡터만 역변환한다.
        const auto childJointInMass = childBody->getCMassLocalPose().getInverse() * joint->getChildPose();
        const auto position = childJointInMass.q.rotateInv(RotationVector(PxQuat(q.x, q.y, q.z, q.w)));
        const auto parent = m_Bodies[m_Asset.FindBody(data.parentBone)]->getGlobalPose() * joint->getParentPose();
        const auto child = m_Bodies[m_Asset.FindBody(data.childBone)]->getGlobalPose() * joint->getChildPose();
        // 목표 차분은 부모 관절 좌표다. Articulation의 속도/토크 축은 현재 자식 관절 좌표다.
        const auto velocity = child.q.rotateInv(parent.q.rotate(ToPx(angularVelocity)));
        for (int axis = 0; axis < 3; ++axis)
        {
            m_ArticulationJoints[index]->setDriveTarget(AngularAxes[axis], position[axis], false);
            m_ArticulationJoints[index]->setDriveVelocity(AngularAxes[axis], velocity[axis], false);
        }
#else
        (void)index;
        (void)rotation;
        (void)angularVelocity;
#endif
    }

    // 추가 보상 토크(중력 + 코리올리/원심력 + 목표 가속도에 필요한 관성 토크)는 우리 코드에서 계산해 PhysX에 전달한다.
    // 일반 관절 PD는 목표 회전/각속도와 강성/감쇠 계수를 전달하고, PD 토크 자체는 PhysX가 simulate()에서 계산한다.
    // 따라서 cache.jointForce에는 관절의 추가 보상 토크만 넣는다. 일반 관절 PD 토크를 여기서 직접 더하지 않는다.
    // 골반 회전 PD는 예외로, 이 함수 안에서 보정을 계산해 목표 가속도와 골반 토크에 반영한다.
    void Ragdoll::UpdateArticulationDrives(float deltaTime)
    {
#if defined(_M_X64)
        if (!m_Articulation || !m_ArticulationCache)
        {
            return;
        }
        auto& cache = *m_ArticulationCache;
        const bool compensate = m_DrivesEnabled && IsRootSupported();
        const unsigned count = 6 + m_Articulation->getDofs();
        std::vector<float> targetVelocity(count, 0), targetAcceleration(count, 0);
        if (compensate)
        {
            PxVec3 linear, angular;
            m_SupportJoint->getDriveVelocity(linear, angular);
            for (int axis = 0; axis < 3; ++axis)
            {
                targetVelocity[axis] = linear[axis];
                targetVelocity[3 + axis] = angular[axis];
            }
            for (size_t i = 0; i < m_ArticulationJoints.size(); ++i)
            {
                for (int axis = 0; axis < 3; ++axis)
                {
                    targetVelocity[6 + m_ArticulationDofOffsets[i] + axis] = m_ArticulationJoints[i]->getDriveVelocity(AngularAxes[axis]);
                }
            }
            if (deltaTime > 0 && m_PreviousArticulationTargetVelocity.size() == count)
            {
                for (unsigned axis = 0; axis < count; ++axis)
                {
                    targetAcceleration[axis] = (targetVelocity[axis] - m_PreviousArticulationTargetVelocity[axis]) / deltaTime;
                }
            }
        }
        m_PreviousArticulationTargetVelocity = compensate && deltaTime > 0 ? targetVelocity : std::vector<float>();
        if (m_SupportJoint)
        {
            // 활성 제어 중에는 아래 질량행렬 PD가 골반 회전을 담당한다. 두 PD를 중복 적용하지 않는다.
            const PxD6JointDrive rotation = compensate ? PxD6JointDrive() :
                PxD6JointDrive(PelvisAngularFrequency * PelvisAngularFrequency,
                    2.0f * PelvisAngularDampingRatio * PelvisAngularFrequency, PX_MAX_F32, true);
            m_SupportJoint->setDrive(PxD6Drive::eSLERP, rotation);
        }
        if (compensate && deltaTime > 0 && m_HasSupportTarget)
        {
            const auto* root = m_Bodies[m_ArticulationRoot];
            const PxVec3 error = RotationVector(m_SupportJoint->getDrivePosition().q * root->getGlobalPose().q.getConjugate());
            const PxVec3 velocity = root->getAngularVelocity();
            const float stiffness = PelvisAngularFrequency * PelvisAngularFrequency;
            const float damping = 2.0f * PelvisAngularDampingRatio * PelvisAngularFrequency;
            const float denominator = 1.0f + damping * deltaTime + stiffness * deltaTime * deltaTime;
            for (unsigned axis = 0; axis < 3; ++axis)
            {
                // 다음 스텝의 각도/속도로 PD를 평가하는 후진 오일러 근사다.
                // 회전 가속도를 먼저 정하고, M 전체에 곱해 축별 관성과 관절 반작용을 함께 반영한다.
                targetAcceleration[3 + axis] = (targetAcceleration[3 + axis] +
                    stiffness * (error[axis] - deltaTime * velocity[axis]) +
                    damping * (targetVelocity[3 + axis] - velocity[axis])) / denominator;
            }
        }
        std::vector<float> inertias;
        if (m_DrivesEnabled)
        {
            m_Articulation->copyInternalStateToCache(cache, PxArticulationCacheFlag::eALL);
            m_Articulation->commonInit();
            m_Articulation->computeMassMatrix(cache);
            std::vector<unsigned> freeAxes;
            if (!IsRootSupported())
            {
                freeAxes = {0, 1, 2, 3, 4, 5};
            }
            for (size_t i = 0; i < m_ArticulationJoints.size(); ++i)
            {
                const auto& data = m_Asset.constraints[i];
                const PhysicsJointMotion motions[] = {data.twist, data.swingY, data.swingZ};
                for (int axis = 0; axis < 3; ++axis)
                {
                    const auto& drive = data.drives[data.useSlerpDrive ? 0 : axis];
                    const bool driven = drive.enabled && drive.maxTorque > 0 &&
                        (drive.GetStiffness() > 0 || drive.GetDamping() > 0);
                    if (motions[axis] != PhysicsJointMotion::Locked && !driven)
                    {
                        freeAxes.push_back(6 + m_ArticulationDofOffsets[i] + axis);
                    }
                }
            }
            inertias = ControlInertias(cache.massMatrix, 6 + m_Articulation->getDofs(), freeAxes);
        }
        if (compensate)
        {
            // 기준 자세와 실제 속도를 사용해 연결된 몸 전체의 중력/회전 하중을 계산한다.
            // 부유 베이스의 앞 6개 항은 골반의 외부 힘/토크이며, 관절 모터에는 그 뒤의 항을 적용한다.
            m_Articulation->computeGravityCompensation(cache);
            m_Articulation->computeCoriolisCompensation(cache);
            if (deltaTime > 0 && m_HasSupportTarget)
            {
                // G + C + M*a의 골반 회전 행(3~5)을 월드 공간 토크로 적용한다.
                // a에는 위에서 구한 골반 PD 보정이 포함된다. 같은 a를 아래 관절 행에도 사용하므로
                // 골반 보정에 필요한 팔다리의 반작용까지 계산된다. 위치 지지는 D6가 담당한다.
                PxVec3 rootTorque(0);
                for (unsigned axis = 0; axis < 3; ++axis)
                {
                    const unsigned row = 3 + axis;
                    rootTorque[axis] = cache.gravityCompensationForce[row] + cache.coriolisForce[row];
                    for (unsigned column = 0; column < count; ++column)
                    {
                        rootTorque[axis] += cache.massMatrix[row * count + column] * targetAcceleration[column];
                    }
                }
                // addTorque는 누적되므로 설정 변경(deltaTime=0)에서는 적용하지 않는다.
                m_Bodies[m_ArticulationRoot]->addTorque(rootTorque, PxForceMode::eFORCE, false);
            }
        }
        for (size_t i = 0; i < m_ArticulationJoints.size(); ++i)
        {
            const auto& data = m_Asset.constraints[i];
            const PhysicsJointMotion motions[] = {data.twist, data.swingY, data.swingZ};
            for (int axis = 0; axis < 3; ++axis)
            {
                const auto& drive = data.drives[data.useSlerpDrive ? 0 : axis];
                const bool enabled = m_DrivesEnabled && drive.enabled && drive.maxTorque > 0 &&
                    motions[axis] != PhysicsJointMotion::Locked;
                const unsigned dof = m_ArticulationDofOffsets[i] + axis;
                float feedForward = 0;
                if (enabled && compensate)
                {
                    feedForward = cache.gravityCompensationForce[6 + dof] + cache.coriolisForce[6 + dof];
                    // 부모와 이웃 관절의 목표 가속도까지 질량 행렬에 곱해 관성 하중을 미리 보상한다.
                    // 예: 무릎이 가속할 때 발목이 감당하는 토크를 발목 오차가 커지기 전에 제공한다.
                    for (unsigned column = 0; column < count; ++column)
                    {
                        feedForward += cache.massMatrix[(6 + dof) * count + column] * targetAcceleration[column];
                    }
                    // 토크 상한의 영향을 비교하기 위해 보상토크 제한을 임시로 해제한다.
                    // feedForward = PxClamp(feedForward, -drive.maxTorque, drive.maxTorque);
                }
                cache.jointForce[dof] = feedForward;
                // 상한 해제 실험: 보상 사용량을 차감하지 않고 PD에 사실상 무제한 토크를 허용한다.
                // const float remainingTorque = enabled ? PxMax(0.0f, drive.maxTorque - PxAbs(feedForward)) : 0;
                const float remainingTorque = enabled ? PX_MAX_F32 : 0;
                // 최신 Articulation에는 구형 internalCompliance가 없다. 다른 모터의 지지 상태를 반영한
                // 관성으로 Kp=I*omegaN^2, Kd=2*zeta*I*omegaN을 만들고 암시적 토크 드라이브에 전달한다.
                const float inertia = enabled && drive.useAcceleration ? inertias[6 + dof] : 1.0f;
                const PxArticulationDrive motor(enabled ? inertia * drive.GetStiffness() : 0,
                    enabled ? inertia * drive.GetDamping() : 0, remainingTorque, PxArticulationDriveType::eFORCE);
                m_ArticulationJoints[i]->setDriveParams(AngularAxes[axis], motor);
            }
        }
        m_Articulation->applyCache(cache, PxArticulationCacheFlag::eFORCE, false);
#else
        (void)deltaTime;
#endif
    }
}
