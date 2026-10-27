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
    void Ragdoll::SetCollisionEnabled(bool enabled)
    {
        m_CollisionEnabled = enabled;
#if defined(_M_X64)
        for (auto* body : m_Bodies)
        {
            std::vector<physx::PxShape*> shapes(body->getNbShapes());
            body->getShapes(shapes.data(), static_cast<physx::PxU32>(shapes.size()));
            for (auto* shape : shapes)
            {
                shape->setFlag(physx::PxShapeFlag::eSIMULATION_SHAPE, enabled);
            }
        }
#else
        (void)enabled;
#endif
    }

#if defined(_M_X64)
    namespace
    {
        PxTransform ToPx(const Matrix4& matrix)
        {
            const auto frame = PhysicsJointFrame::FromTransform(matrix);
            const auto& p = frame.offset;
            const auto& q = frame.rotation;
            return PxTransform(PxVec3(p.x, p.y, p.z), PxQuat(q.x, q.y, q.z, q.w));
        }
        Matrix4 FromPx(const PxTransform& pose)
        {
            return Matrix4(OrthogonalTransform(Quaternion(XMVectorSet(pose.q.x, pose.q.y, pose.q.z, pose.q.w)),
                Vector3(pose.p.x, pose.p.y, pose.p.z)));
        }
        PxD6Motion::Enum Motion(PhysicsJointMotion motion)
        {
            return motion == PhysicsJointMotion::Locked ? PxD6Motion::eLOCKED : motion == PhysicsJointMotion::Limited ? PxD6Motion::eLIMITED
                                                                                                                      : PxD6Motion::eFREE;
        }

        class MovementQueryFilter : public PxQueryFilterCallback
        {
        public:
            MovementQueryFilter(const std::vector<PxRigidBody*>& bodies, const void* owner)
                : m_Bodies(bodies), m_Owner(owner)
            {
            }
            PxVec3 direction{0};
            PxQueryHitType::Enum preFilter(const PxFilterData&, const PxShape* shape,
                const PxRigidActor* actor, PxHitFlags&) override
            {
                return !shape->getFlags().isSet(PxShapeFlag::eSIMULATION_SHAPE) ||
                        shape->getFlags().isSet(PxShapeFlag::eTRIGGER_SHAPE) ||
                        (m_Owner && actor->userData == m_Owner) ||
                        std::find(m_Bodies.begin(), m_Bodies.end(), actor) != m_Bodies.end()
                    ? PxQueryHitType::eNONE : PxQueryHitType::eBLOCK;
            }
            PxQueryHitType::Enum postFilter(const PxFilterData&, const PxQueryHit& hit,
                const PxShape*, const PxRigidActor*) override
            {
                const auto& sweep = static_cast<const PxSweepHit&>(hit);
                // 지면 접촉은 물리 솔버에 맡기고, 벽 안쪽으로 들어가는 수평 이동만 제한한다.
                return sweep.normal.y < .70710678f && sweep.normal.dot(direction) < -1e-5f
                    ? PxQueryHitType::eBLOCK : PxQueryHitType::eNONE;
            }

        private:
            const std::vector<PxRigidBody*>& m_Bodies;
            const void* m_Owner;
        };
    }
#endif
    Vector3 Ragdoll::ConstrainHorizontalMovement(const std::vector<Matrix4>& boneWorld, Vector3 displacement) const
    {
#if defined(_M_X64)
        const PxVec3 requested(float(displacement.GetX()), 0, float(displacement.GetZ()));
        if (!requested.isFinite())
        {
            return Vector3(kZero);
        }
        if (!IsActive() || !m_CollisionEnabled || requested.magnitudeSquared() < 1e-12f)
        {
            return Vector3(requested.x, 0, requested.z);
        }
        std::vector<PxTransform> targets;
        for (size_t i = 0; i < m_Bodies.size(); ++i)
        {
            const int node = m_BoneNodes[i];
            if (node < 0 || size_t(node) >= boneWorld.size())
            {
                return Vector3(kZero);
            }
            const auto target = ToPx(PhysicsBoneFrame(boneWorld[node]) * m_Asset.bodies[i].GetLocalTransform());
            if (!target.isValid())
            {
                return Vector3(kZero);
            }
            targets.push_back(target);
        }
        MovementQueryFilter filter(m_Bodies, m_QueryOwner);
        PxVec3 moved(0), remaining = requested;
        constexpr float margin = .002f;
        for (int iteration = 0; iteration < 3 && remaining.magnitudeSquared() > 1e-12f; ++iteration)
        {
            const float distance = remaining.magnitude();
            filter.direction = remaining / distance;
            float allowed = distance;
            PxVec3 normal(0);
            for (size_t i = 0; i < m_Bodies.size(); ++i)
            {
                const auto* body = m_Bodies[i];
                auto* scene = body->getScene();
                if (!scene)
                {
                    continue;
                }
                std::vector<PxShape*> shapes(body->getNbShapes());
                body->getShapes(shapes.data(), static_cast<PxU32>(shapes.size()));
                for (const auto* shape : shapes)
                {
                    if (!shape->getFlags().isSet(PxShapeFlag::eSIMULATION_SHAPE))
                    {
                        continue;
                    }
                    // 실제 바디뿐 아니라 목표 형상도 검사해 바디가 막혔을 때 루트만 계속 전진하지 않게 한다.
                    for (auto pose : {body->getGlobalPose() * shape->getLocalPose(), targets[i] * shape->getLocalPose()})
                    {
                        pose.p += moved;
                        PxSweepBuffer hit;
                        scene->sweep(shape->getGeometry(), pose, filter.direction, distance + margin, hit,
                            PxHitFlag::eDEFAULT | PxHitFlag::eMTD,
                            PxQueryFilterData(PxQueryFlag::eSTATIC | PxQueryFlag::eDYNAMIC | PxQueryFlag::ePREFILTER | PxQueryFlag::ePOSTFILTER), &filter);
                        if (hit.hasBlock && hit.block.distance - margin < allowed)
                        {
                            allowed = PxMax(0.0f, hit.block.distance - margin);
                            normal = hit.block.normal;
                        }
                    }
                }
            }
            const PxVec3 travel = filter.direction * allowed;
            moved += travel;
            remaining -= travel;
            normal.y = 0;
            if (normal.normalize() < 1e-6f)
            {
                break;
            }
            remaining -= normal * PxMin(0.0f, remaining.dot(normal));
        }
        return Vector3(moved.x, 0, moved.z);
#else
        (void)boneWorld;
        return displacement;
#endif
    }

    Ragdoll::~Ragdoll()
    {
        Destroy();
    }
    void Ragdoll::Destroy()
    {
#if defined(_M_X64)
        if (m_SupportJoint)
        {
            m_SupportJoint->release();
        }
        for (auto* joint : m_Joints)
        {
            joint->release();
        }
        if (m_ArticulationCache)
        {
            m_ArticulationCache->release();
        }
        if (m_Articulation)
        {
            m_Articulation->release();
        }
        else
        {
            for (auto* body : m_Bodies)
            {
                body->release();
            }
        }
#endif
        m_Articulation = nullptr;
        m_ArticulationCache = nullptr;
        m_ArticulationJoints.clear();
        m_ArticulationDofOffsets.clear();
        m_PreviousArticulationTargetVelocity.clear();
        m_ArticulationRoot = -1;
        m_World = nullptr;
        m_BoneNames.clear();
        m_QueryOwner = nullptr;
        m_CollisionEnabled = true;
        m_Joints.clear();
        m_Bodies.clear();
        m_BoneNodes.clear();
        m_Asset = {};
        m_Simulating = false;
        m_DrivesEnabled = m_HasDriveTargets = false;
        m_SupportBody = -1;
        m_MaxTargetClampAngle = 0;
        m_SupportJoint = nullptr;
        m_HasSupportTarget = false;
        m_SupportHorizontal = true;
        m_RootSupportWeight = 0;
        m_DriveTargets.clear();
        m_LastKinematicTargets.clear();
        m_AnimatedLinearVelocity.clear();
        m_AnimatedAngularVelocity.clear();
    }

    void Ragdoll::Swap(Ragdoll& other) noexcept
    {
        // 벡터 저장 공간을 교환해 PhysX/PVD가 보관 중인 이름 포인터를 유지한다.
        m_Asset.bodies.swap(other.m_Asset.bodies);
        m_Asset.constraints.swap(other.m_Asset.constraints);
        m_BoneNodes.swap(other.m_BoneNodes);
        m_Bodies.swap(other.m_Bodies);
        m_Joints.swap(other.m_Joints);
        std::swap(m_World, other.m_World);
        m_BoneNames.swap(other.m_BoneNames);
        std::swap(m_QueryOwner, other.m_QueryOwner);
        std::swap(m_CollisionEnabled, other.m_CollisionEnabled);
        std::swap(m_Articulation, other.m_Articulation);
        std::swap(m_ArticulationCache, other.m_ArticulationCache);
        m_ArticulationJoints.swap(other.m_ArticulationJoints);
        m_ArticulationDofOffsets.swap(other.m_ArticulationDofOffsets);
        m_PreviousArticulationTargetVelocity.swap(other.m_PreviousArticulationTargetVelocity);
        std::swap(m_ArticulationRoot, other.m_ArticulationRoot);
        std::swap(m_Simulating, other.m_Simulating);
        std::swap(m_DrivesEnabled, other.m_DrivesEnabled);
        std::swap(m_HasDriveTargets, other.m_HasDriveTargets);
        std::swap(m_SupportBody, other.m_SupportBody);
        std::swap(m_MaxTargetClampAngle, other.m_MaxTargetClampAngle);
        std::swap(m_SupportJoint, other.m_SupportJoint);
        std::swap(m_HasSupportTarget, other.m_HasSupportTarget);
        std::swap(m_SupportHorizontal, other.m_SupportHorizontal);
        std::swap(m_RootSupportWeight, other.m_RootSupportWeight);
        m_DriveTargets.swap(other.m_DriveTargets);
        m_LastKinematicTargets.swap(other.m_LastKinematicTargets);
        m_AnimatedLinearVelocity.swap(other.m_AnimatedLinearVelocity);
        m_AnimatedAngularVelocity.swap(other.m_AnimatedAngularVelocity);
    }

    bool Ragdoll::Create(PhysicsWorld& world, const PhysicsAsset& asset, const std::vector<std::string>& names,
        const std::vector<Matrix4>& boneWorld, Vector3 velocity, std::string& error, bool simulate)
    {
        if (IsActive())
        {
            error = "Stop the existing ragdoll before recreating it.";
            return false;
        }
        if (!asset.ValidateSkeleton(names, error))
        {
            return false;
        }
        if (asset.bodies.empty() || names.size() != boneWorld.size())
        {
            error = "Ragdoll needs bodies and a matching skeleton pose.";
            return false;
        }
#if defined(_M_X64)
        if (!world.IsInitialized())
        {
            error = "The world has no physics scene.";
            return false;
        }
        const PxVec3 initialVelocity(float(velocity.GetX()), float(velocity.GetY()), float(velocity.GetZ()));
        if (!initialVelocity.isFinite())
        {
            error = "Invalid initial velocity.";
            return false;
        }
        m_Asset = asset;
        m_World = &world;
        m_BoneNames = names;
        if (simulate && asset.UsesArticulation())
        {
            if (!CreateArticulation(world, boneWorld, error))
            {
                Destroy();
                return false;
            }
        }
        else
        {
            for (const auto& body : m_Asset.bodies)
            {
                const int node = static_cast<int>(std::find(names.begin(), names.end(), body.bone) - names.begin());
                const PxTransform transform = ToPx(PhysicsBoneFrame(boneWorld[node]) * body.GetLocalTransform());
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
                auto* actor = world.CreateDynamicBody(transform, geometry.any(), body.mass, body.bone.c_str(), body.material);
                if (!actor)
                {
                    error = "Could not create ragdoll body: " + body.bone;
                    Destroy();
                    return false;
                }
                // 관절을 더 단단하게 유지하도록 참고 구현의 4/1보다 반복 횟수를 적당히 높인다.
                // CreateDynamicBody의 32/8 설정을 명시적으로 덮어쓰되, Debug의 고정 스텝 시뮬레이션을
                // 지연시켰던 이전의 128/32보다는 낮게 유지한다.
                actor->setSolverIterationCounts(16, 4);
                actor->setLinearVelocity(initialVelocity);
                m_Bodies.push_back(actor);
                m_BoneNodes.push_back(node);
            }
        }
        for (const auto& data : m_Asset.constraints)
        {
            auto* joint = world.CreateJoint(*m_Bodies[asset.FindBody(data.parentBone)], ToPx(data.parentFrame.GetTransform()),
                *m_Bodies[asset.FindBody(data.childBone)], ToPx(data.childFrame.GetTransform()));
            if (!joint)
            {
                error = "Could not create ragdoll joint.";
                Destroy();
                return false;
            }
            m_Joints.push_back(joint);
            joint->setName(data.childBone.c_str());
            joint->setAngularDriveConfig(PxD6AngularDriveConfig::eSWING_TWIST);
            joint->setConstraintFlag(PxConstraintFlag::eDRIVE_LIMITS_ARE_FORCES, true);
            // Articulation에서는 연결과 제한을 내부 관절이 담당하며 D6는 진단 좌표계만 보관한다.
            const auto translation = m_Articulation ? PxD6Motion::eFREE : PxD6Motion::eLOCKED;
            joint->setMotion(PxD6Axis::eX, translation);
            joint->setMotion(PxD6Axis::eY, translation);
            joint->setMotion(PxD6Axis::eZ, translation);
            joint->setMotion(PxD6Axis::eTWIST, Motion(data.twist));
            joint->setMotion(PxD6Axis::eSWING1, Motion(data.swingY));
            joint->setMotion(PxD6Axis::eSWING2, Motion(data.swingZ));
            joint->setTwistLimit(PxJointAngularLimitPair(data.twistMin, data.twistMax));
            joint->setSwingLimit(PxJointLimitCone(data.swingYLimit, data.swingZLimit));
            if (data.swingY == PhysicsJointMotion::Limited && data.swingZ == PhysicsJointMotion::Limited)
            {
                // PhysX는 원뿔과 피라미드 제한을 동시에 지원한다(ExtD6Joint.cpp).
                // 바깥쪽 경계는 큰 충격에 대응하는 솔버 제약 행을 추가한다.
                // 타원을 줄이거나 에디터에 표시되는 원뿔을 변경하지는 않는다.
                joint->setPyramidSwingLimit(PxJointLimitPyramid(
                    -data.swingYLimit, data.swingYLimit, -data.swingZLimit, data.swingZLimit));
            }
            joint->setConstraintFlag(PxConstraintFlag::eCOLLISION_ENABLED, data.enableCollision);
            joint->setConstraintFlag(PxConstraintFlag::eVISUALIZATION, true);
            if (m_Articulation)
            {
                // 편집/진단용 좌표계만 남긴다. 같은 링크 사이를 두 솔버 제약으로 중복 연결하지 않는다.
                joint->setConstraintFlag(PxConstraintFlag::eDISABLE_CONSTRAINT, true);
            }
        }
        if (m_Articulation)
        {
            m_Articulation->setRootLinearVelocity(initialVelocity);
            m_Articulation->updateKinematic(PxArticulationKinematicFlag::eVELOCITY);
            m_Simulating = true;
            error.clear();
            return true;
        }
        return SetSimulating(simulate, velocity, error);
#else
        (void)world;
        (void)velocity;
        (void)simulate;
        error = "Ragdoll requires an x64 PhysX build.";
        return false;
#endif
    }

    bool Ragdoll::SetSimulating(bool simulate, Vector3 velocity, std::string& error, bool inheritAnimationVelocity)
    {
        if (!IsActive())
        {
            error = "No PA bodies are registered.";
            return false;
        }
#if defined(_M_X64)
        if (m_Articulation)
        {
            if (simulate)
            {
                error.clear();
                return true;
            }
            return RebuildSimulation(false, velocity, error, inheritAnimationVelocity);
        }
        if (simulate && m_Asset.UsesArticulation())
        {
            return RebuildSimulation(true, velocity, error, inheritAnimationVelocity);
        }
        const PxVec3 initialVelocity(float(velocity.GetX()), float(velocity.GetY()), float(velocity.GetZ()));
        if (!initialVelocity.isFinite())
        {
            error = "Invalid initial velocity.";
            return false;
        }
        if (!simulate)
        {
            SetDrivesEnabled(false);
            SetRootSupported(false, error);
            m_LastKinematicTargets.clear();
            m_AnimatedLinearVelocity.clear();
            m_AnimatedAngularVelocity.clear();
        }
        for (size_t i = 0; i < m_Bodies.size(); ++i)
        {
            auto* body = m_Bodies[i]->is<PxRigidDynamic>();
            body->setRigidBodyFlag(PxRigidBodyFlag::eKINEMATIC, !simulate);
            // 이동 스윕이 캐릭터 자신의 애니메이션 본 형상에 충돌하지 않도록 한다.
            // 시뮬레이션 형상은 계속 활성화하며, 두 모드 모두 PVD에 표시한다.
            PxShape* shape = nullptr;
            body->getShapes(&shape, 1);
            if (shape)
            {
                shape->setFlag(PxShapeFlag::eSCENE_QUERY_SHAPE, simulate);
            }
            if (simulate)
            {
                const auto toPx = [](Vector3 v) {
                    return PxVec3(float(v.GetX()), float(v.GetY()), float(v.GetZ()));
                };
                body->setLinearVelocity(inheritAnimationVelocity && m_AnimatedLinearVelocity.size() == m_Bodies.size() ? toPx(m_AnimatedLinearVelocity[i]) : initialVelocity);
                body->setAngularVelocity(inheritAnimationVelocity && m_AnimatedAngularVelocity.size() == m_Bodies.size() ? toPx(m_AnimatedAngularVelocity[i]) : PxVec3(0));
                body->wakeUp();
            }
        }
        m_Simulating = simulate;
        error.clear();
        return true;
#else
        (void)simulate;
        (void)velocity;
        (void)inheritAnimationVelocity;
        error = "Ragdoll requires an x64 PhysX build.";
        return false;
#endif
    }

    bool Ragdoll::UpdateKinematicPose(const std::vector<Matrix4>& boneWorld, bool teleport, std::string& error)
    {
        if (!IsActive() || IsSimulating())
        {
            error = "PA bodies are not kinematic.";
            return false;
        }
#if defined(_M_X64)
        std::vector<PxTransform> poses;
        poses.reserve(m_Bodies.size());
        for (size_t i = 0; i < m_Bodies.size(); ++i)
        {
            const int node = m_BoneNodes[i];
            if (node < 0 || size_t(node) >= boneWorld.size())
            {
                error = "Invalid animated PA bone index.";
                return false;
            }
            const auto pose = ToPx(PhysicsBoneFrame(boneWorld[node]) * m_Asset.bodies[i].GetLocalTransform());
            if (!pose.isValid())
            {
                error = "Invalid animated PA transform.";
                return false;
            }
            poses.push_back(pose);
        }
        for (size_t i = 0; i < m_Bodies.size(); ++i)
        {
            if (teleport && m_LastKinematicTargets.size() == m_Bodies.size())
            {
                const auto previous = ToPx(m_LastKinematicTargets[i]);
                // 순간이동이나 클립 초기화 시에는 유한차분으로 구한 속도를 무효화한다.
                // 일반적인 틱 이후에 같은 포즈를 갱신할 때는 해당 속도를 보존한다.
                if ((poses[i].p - previous.p).magnitudeSquared() > 1e-8f || PxAbs(poses[i].q.dot(previous.q)) < .999999f)
                {
                    m_AnimatedLinearVelocity.clear();
                    m_AnimatedAngularVelocity.clear();
                }
            }
            if (!teleport && m_LastKinematicTargets.size() == m_Bodies.size())
            {
                const auto previous = ToPx(m_LastKinematicTargets[i]);
                const PxVec3 linear = (poses[i].p - previous.p) / float(PhysicsWorld::FixedDeltaTime);
                PxQuat delta = poses[i].q * previous.q.getConjugate();
                if (delta.w < 0)
                {
                    delta = -delta;
                }
                PxReal angle;
                PxVec3 axis;
                delta.toRadiansAndUnitAxis(angle, axis);
                m_AnimatedLinearVelocity.resize(m_Bodies.size());
                m_AnimatedAngularVelocity.resize(m_Bodies.size());
                m_AnimatedLinearVelocity[i] = Vector3(linear.x, linear.y, linear.z);
                const auto angular = axis * PxMin(angle / float(PhysicsWorld::FixedDeltaTime), 20.0f);
                m_AnimatedAngularVelocity[i] = Vector3(angular.x, angular.y, angular.z);
            }
            if (teleport)
            {
                m_Bodies[i]->setGlobalPose(poses[i]);
            }
            m_Bodies[i]->is<PxRigidDynamic>()->setKinematicTarget(poses[i]);
        }
        m_LastKinematicTargets.resize(m_Bodies.size());
        for (size_t i = 0; i < poses.size(); ++i)
        {
            m_LastKinematicTargets[i] = FromPx(poses[i]);
        }
        error.clear();
        return true;
#else
        (void)boneWorld;
        (void)teleport;
        error = "Ragdoll requires an x64 PhysX build.";
        return false;
#endif
    }

    size_t Ragdoll::GetKinematicBodyCount() const
    {
        size_t count = 0;
#if defined(_M_X64)
        for (const auto* body : m_Bodies)
        {
            if (body->getRigidBodyFlags().isSet(PxRigidBodyFlag::eKINEMATIC))
            {
                ++count;
            }
        }
#endif
        return count;
    }

    Matrix4 Ragdoll::GetBoneWorld(size_t body) const
    {
#if defined(_M_X64)
        return FromPx(m_Bodies[body]->getGlobalPose()) * Invert(m_Asset.bodies[body].GetLocalTransform());
#else
        (void)body;
        return Matrix4(kIdentity);
#endif
    }
    Vector3 Ragdoll::GetPosition() const
    {
        return IsActive() ? Vector3(GetBoneWorld(0).GetW()) : Vector3(kZero);
    }
    bool Ragdoll::GetCenterOfMass(Vector3& center, float& totalMass) const
    {
        center = Vector3(kZero);
        totalMass = 0;
#if defined(_M_X64)
        double massSum = 0, x = 0, y = 0, z = 0;
        for (const auto* body : m_Bodies)
        {
            const float mass = body->getMass();
            const PxVec3 position = (body->getGlobalPose() * body->getCMassLocalPose()).p;
            if (!std::isfinite(mass) || mass <= 0 || !position.isFinite())
                return false;
            massSum += mass;
            x += double(mass) * position.x;
            y += double(mass) * position.y;
            z += double(mass) * position.z;
        }
        if (massSum <= 0)
            return false;
        center = Vector3(float(x / massSum), float(y / massSum), float(z / massSum));
        totalMass = float(massSum);
        return true;
#else
        return false;
#endif
    }

    void Ragdoll::AddVelocity(Vector3 velocity)
    {
        if (!IsSimulating())
        {
            return;
        }
#if defined(_M_X64)
        const PxVec3 delta(float(velocity.GetX()), float(velocity.GetY()), float(velocity.GetZ()));
        if (delta.isFinite())
        {
            if (m_Articulation)
            {
                m_Articulation->setRootLinearVelocity(m_Articulation->getRootLinearVelocity() + delta);
                m_Articulation->updateKinematic(PxArticulationKinematicFlag::eVELOCITY);
                return;
            }
            for (auto* body : m_Bodies)
            {
                if (!(body->getRigidBodyFlags() & PxRigidBodyFlag::eKINEMATIC))
                {
                    body->addForce(delta, PxForceMode::eVELOCITY_CHANGE);
                }
            }
        }
#else
        (void)velocity;
#endif
    }
    void Ragdoll::SetQueryOwner(void* owner)
    {
        m_QueryOwner = owner;
#if defined(_M_X64)
        for (auto* body : m_Bodies)
        {
            body->userData = owner;
        }
#else
        (void)owner;
#endif
    }

    float Ragdoll::GetMaxAnchorError() const
    {
        float result = 0;
#if defined(_M_X64)
        for (auto* joint : m_Joints)
        {
            PxRigidActor* a;
            PxRigidActor* b;
            joint->getActors(a, b);
            result = PxMax(result, ((a->getGlobalPose() * joint->getLocalPose(PxJointActorIndex::eACTOR0)).p - (b->getGlobalPose() * joint->getLocalPose(PxJointActorIndex::eACTOR1)).p).magnitude());
        }
#endif
        return result;
    }
    float Ragdoll::GetMaxAngularError(size_t* worstJoint) const
    {
        float result = 0;
        if (worstJoint)
        {
            *worstJoint = 0;
        }
#if defined(_M_X64)
        for (size_t i = 0; i < m_Joints.size(); ++i)
        {
            const float previous = result;
            const auto& data = m_Asset.constraints[i];
            if (m_Articulation)
            {
                const PhysicsJointMotion motions[] = {data.twist, data.swingY, data.swingZ};
                const float lower[] = {data.twistMin, -data.swingYLimit, -data.swingZLimit};
                const float upper[] = {data.twistMax, data.swingYLimit, data.swingZLimit};
                for (int axis = 0; axis < 3; ++axis)
                {
                    const float position = m_ArticulationJoints[i]->getJointPosition(PxArticulationAxis::Enum(axis));
                    if (motions[axis] == PhysicsJointMotion::Locked)
                    {
                        result = PxMax(result, PxAbs(position));
                    }
                    else if (motions[axis] == PhysicsJointMotion::Limited)
                    {
                        result = PxMax(result, PxMax(lower[axis] - position, position - upper[axis]));
                    }
                }
                if (worstJoint && result > previous)
                {
                    *worstJoint = i;
                }
                continue;
            }
            const auto* joint = m_Joints[i];
            // 상대 쿼터니언의 부호가 바뀌면 PhysX가 +/-2pi 부근의 동등한 각도를 반환할 수 있다.
            // 제한을 검사할 때는 최단 트위스트 각도를 사용한다.
            const float twist = std::remainder(joint->getTwistAngle(), PxTwoPi);
            if (data.twist == PhysicsJointMotion::Locked)
            {
                result = PxMax(result, PxAbs(twist));
            }
            else if (data.twist == PhysicsJointMotion::Limited)
            {
                result = PxMax(result, PxMax(data.twistMin - twist, twist - data.twistMax));
            }
            if (data.swingY == PhysicsJointMotion::Limited && data.swingZ == PhysicsJointMotion::Limited)
            {
                // 각 좌표가 개별 제한 이내여도 대각선 스윙은 실제 타원형 원뿔을 벗어날 수 있다.
                // 이 SDK의 ConeLimitHelperTanLess가 사용하는 각도 타원을 기준으로
                // 방사 방향의 각도 초과량을 측정한다.
                // 이는 오차 지표이며 솔버의 접촉 법선은 아니다.
                const float y = joint->getSwingYAngle(), z = joint->getSwingZAngle();
                const float ny = y / data.swingYLimit, nz = z / data.swingZLimit;
                const float ellipseRadius = std::sqrt(ny * ny + nz * nz);
                if (ellipseRadius > 1)
                {
                    const float ty = std::tan(y * .25f), tz = std::tan(z * .25f);
                    const float py = std::tan(y / ellipseRadius * .25f), pz = std::tan(z / ellipseRadius * .25f);
                    const float r2 = ty * ty + tz * tz, p2 = py * py + pz * pz;
                    // 실제 순수 스윙 쿼터니언과 경계의 순수 스윙 쿼터니언을 내적한다.
                    const float dot = ((1 - r2) * (1 - p2) + 4 * (ty * py + tz * pz)) / ((1 + r2) * (1 + p2));
                    result = PxMax(result, 2 * std::acos(PxClamp(PxAbs(dot), 0.0f, 1.0f)));
                }
            }
            else
            {
                if (data.swingY != PhysicsJointMotion::Free)
                {
                    result = PxMax(result, PxAbs(joint->getSwingYAngle()) - (data.swingY == PhysicsJointMotion::Locked ? 0 : data.swingYLimit));
                }
                if (data.swingZ != PhysicsJointMotion::Free)
                {
                    result = PxMax(result, PxAbs(joint->getSwingZAngle()) - (data.swingZ == PhysicsJointMotion::Locked ? 0 : data.swingZLimit));
                }
            }
            if (worstJoint && result > previous)
            {
                *worstJoint = i;
            }
        }
#endif
        return result;
    }
    float Ragdoll::GetLowestPoint() const
    {
        float result = 1e30f;
#if defined(_M_X64)
        for (const auto* body : m_Bodies)
        {
            result = PxMin(result, body->getWorldBounds(1.0f).minimum.y);
        }
#endif
        return result;
    }
    bool Ragdoll::IsFinite() const
    {
#if defined(_M_X64)
        for (auto* body : m_Bodies)
        {
            if (!body->getGlobalPose().isValid() || !body->getLinearVelocity().isFinite() || !body->getAngularVelocity().isFinite())
            {
                return false;
            }
        }
#endif
        return true;
    }
}
