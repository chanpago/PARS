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
        PxTransform ToPx(const Matrix4& matrix)
        {
            const auto frame = PhysicsJointFrame::FromTransform(matrix);
            const auto& p = frame.offset; const auto& q = frame.rotation;
            return PxTransform(PxVec3(p.x, p.y, p.z), PxQuat(q.x, q.y, q.z, q.w));
        }
        Matrix4 FromPx(const PxTransform& pose)
        {
            return Matrix4(OrthogonalTransform(Quaternion(XMVectorSet(pose.q.x, pose.q.y, pose.q.z, pose.q.w)),
                Vector3(pose.p.x, pose.p.y, pose.p.z)));
        }
        PxD6Motion::Enum Motion(PhysicsJointMotion motion)
        {
            return motion == PhysicsJointMotion::Locked ? PxD6Motion::eLOCKED :
                motion == PhysicsJointMotion::Limited ? PxD6Motion::eLIMITED : PxD6Motion::eFREE;
        }
    }
#endif
    Ragdoll::~Ragdoll() { Destroy(); }
    void Ragdoll::Destroy()
    {
#if defined(_M_X64)
        if (m_SupportJoint) m_SupportJoint->release();
        for (auto* joint : m_Joints) joint->release();
        for (auto* body : m_Bodies) body->release();
#endif
        m_Joints.clear(); m_Bodies.clear(); m_BoneNodes.clear(); m_Asset = {}; m_Simulating = false;
        m_DrivesEnabled = m_HasDriveTargets = false; m_SupportBody = -1; m_MaxTargetClampAngle = 0;
        m_SupportJoint = nullptr; m_HasSupportTarget = false;
        m_DriveTargets.clear(); m_LastKinematicTargets.clear(); m_AnimatedLinearVelocity.clear(); m_AnimatedAngularVelocity.clear();
    }

    void Ragdoll::Swap(Ragdoll& other) noexcept
    {
        // Swap vector storage, preserving the name pointers retained by PhysX/PVD.
        m_Asset.bodies.swap(other.m_Asset.bodies); m_Asset.constraints.swap(other.m_Asset.constraints);
        m_BoneNodes.swap(other.m_BoneNodes); m_Bodies.swap(other.m_Bodies); m_Joints.swap(other.m_Joints);
        std::swap(m_Simulating, other.m_Simulating);
        std::swap(m_DrivesEnabled, other.m_DrivesEnabled); std::swap(m_HasDriveTargets, other.m_HasDriveTargets);
        std::swap(m_SupportBody, other.m_SupportBody); std::swap(m_MaxTargetClampAngle, other.m_MaxTargetClampAngle);
        std::swap(m_SupportJoint, other.m_SupportJoint); std::swap(m_HasSupportTarget, other.m_HasSupportTarget);
        m_DriveTargets.swap(other.m_DriveTargets); m_LastKinematicTargets.swap(other.m_LastKinematicTargets);
        m_AnimatedLinearVelocity.swap(other.m_AnimatedLinearVelocity); m_AnimatedAngularVelocity.swap(other.m_AnimatedAngularVelocity);
    }

    bool Ragdoll::Create(PhysicsWorld& world, const PhysicsAsset& asset, const std::vector<std::string>& names,
        const std::vector<Matrix4>& boneWorld, Vector3 velocity, std::string& error, bool simulate)
    {
        if (IsActive()) { error = "Stop the existing ragdoll before recreating it."; return false; }
        if (!asset.ValidateSkeleton(names, error)) return false;
        if (asset.bodies.empty() || names.size() != boneWorld.size()) { error = "Ragdoll needs bodies and a matching skeleton pose."; return false; }
#if defined(_M_X64)
        if (!world.IsInitialized()) { error = "The world has no physics scene."; return false; }
        const PxVec3 initialVelocity(float(velocity.GetX()), float(velocity.GetY()), float(velocity.GetZ()));
        if (!initialVelocity.isFinite()) { error = "Invalid initial velocity."; return false; }
        m_Asset = asset;
        for (const auto& body : m_Asset.bodies)
        {
            const int node = static_cast<int>(std::find(names.begin(), names.end(), body.bone) - names.begin());
            const PxTransform transform = ToPx(PhysicsBoneFrame(boneWorld[node]) * body.GetLocalTransform());
            PxGeometryHolder geometry;
            switch (body.shape)
            {
            case PhysicsShapeType::Capsule: geometry.storeAny(PxCapsuleGeometry(body.radius, body.halfLength)); break;
            case PhysicsShapeType::Sphere: geometry.storeAny(PxSphereGeometry(body.radius)); break;
            case PhysicsShapeType::Box: geometry.storeAny(PxBoxGeometry(body.halfExtents.x, body.halfExtents.y, body.halfExtents.z)); break;
            }
            auto* actor = world.CreateDynamicBody(transform, geometry.any(), body.mass, body.bone.c_str());
            if (!actor) { error = "Could not create ragdoll body: " + body.bone; Destroy(); return false; }
            // Use a moderate increase over the reference's 4/1 for tighter joints.
            // Override CreateDynamicBody's 32/8 explicitly and stay below the
            // former 128/32 budget, which stalled fixed-step simulation in Debug.
            actor->setSolverIterationCounts(16, 4);
            actor->setLinearVelocity(initialVelocity);
            m_Bodies.push_back(actor); m_BoneNodes.push_back(node);
        }
        for (const auto& data : m_Asset.constraints)
        {
            auto* joint = world.CreateJoint(*m_Bodies[asset.FindBody(data.parentBone)], ToPx(data.parentFrame.GetTransform()),
                *m_Bodies[asset.FindBody(data.childBone)], ToPx(data.childFrame.GetTransform()));
            if (!joint) { error = "Could not create ragdoll joint."; Destroy(); return false; }
            m_Joints.push_back(joint);
            joint->setName(data.childBone.c_str());
            joint->setAngularDriveConfig(PxD6AngularDriveConfig::eSWING_TWIST);
            joint->setConstraintFlag(PxConstraintFlag::eDRIVE_LIMITS_ARE_FORCES, true);
            joint->setMotion(PxD6Axis::eX, PxD6Motion::eLOCKED);
            joint->setMotion(PxD6Axis::eY, PxD6Motion::eLOCKED);
            joint->setMotion(PxD6Axis::eZ, PxD6Motion::eLOCKED);
            joint->setMotion(PxD6Axis::eTWIST, Motion(data.twist));
            joint->setMotion(PxD6Axis::eSWING1, Motion(data.swingY));
            joint->setMotion(PxD6Axis::eSWING2, Motion(data.swingZ));
            joint->setTwistLimit(PxJointAngularLimitPair(data.twistMin, data.twistMax));
            joint->setSwingLimit(PxJointLimitCone(data.swingYLimit, data.swingZLimit));
            if (data.swingY == PhysicsJointMotion::Limited && data.swingZ == PhysicsJointMotion::Limited)
            {
                // PhysX supports cone + pyramid simultaneously (ExtD6Joint.cpp).
                // These enclosing bounds add solver rows for large impacts; they
                // do not shrink the ellipse or change the cone shown in the editor.
                joint->setPyramidSwingLimit(PxJointLimitPyramid(
                    -data.swingYLimit, data.swingYLimit, -data.swingZLimit, data.swingZLimit));
            }
            joint->setConstraintFlag(PxConstraintFlag::eCOLLISION_ENABLED, data.enableCollision);
            joint->setConstraintFlag(PxConstraintFlag::eVISUALIZATION, true);
        }
        return SetSimulating(simulate, velocity, error);
#else
        (void)world; (void)velocity; (void)simulate;
        error = "Ragdoll requires an x64 PhysX build."; return false;
#endif
    }

    bool Ragdoll::SetSimulating(bool simulate, Vector3 velocity, std::string& error, bool inheritAnimationVelocity)
    {
        if (!IsActive()) { error = "No PA bodies are registered."; return false; }
#if defined(_M_X64)
        const PxVec3 initialVelocity(float(velocity.GetX()), float(velocity.GetY()), float(velocity.GetZ()));
        if (!initialVelocity.isFinite()) { error = "Invalid initial velocity."; return false; }
        if (!simulate)
        {
            SetDrivesEnabled(false); SetRootSupported(false, error);
            m_LastKinematicTargets.clear(); m_AnimatedLinearVelocity.clear(); m_AnimatedAngularVelocity.clear();
        }
        for (size_t i = 0; i < m_Bodies.size(); ++i)
        {
            auto* body = m_Bodies[i];
            body->setRigidBodyFlag(PxRigidBodyFlag::eKINEMATIC, !simulate);
            // Movement sweeps must not hit the character's own animated bone shapes.
            // Simulation shapes remain enabled, and stay visible in PVD in both modes.
            PxShape* shape = nullptr;
            body->getShapes(&shape, 1);
            if (shape) shape->setFlag(PxShapeFlag::eSCENE_QUERY_SHAPE, simulate);
            if (simulate)
            {
                const auto toPx = [](Vector3 v) { return PxVec3(float(v.GetX()), float(v.GetY()), float(v.GetZ())); };
                body->setLinearVelocity(inheritAnimationVelocity && m_AnimatedLinearVelocity.size() == m_Bodies.size() ? toPx(m_AnimatedLinearVelocity[i]) : initialVelocity);
                body->setAngularVelocity(inheritAnimationVelocity && m_AnimatedAngularVelocity.size() == m_Bodies.size() ? toPx(m_AnimatedAngularVelocity[i]) : PxVec3(0));
                body->wakeUp();
            }
        }
        m_Simulating = simulate;
        error.clear(); return true;
#else
        (void)simulate; (void)velocity; (void)inheritAnimationVelocity;
        error = "Ragdoll requires an x64 PhysX build."; return false;
#endif
    }

    bool Ragdoll::UpdateKinematicPose(const std::vector<Matrix4>& boneWorld, bool teleport, std::string& error)
    {
        if (!IsActive() || IsSimulating()) { error = "PA bodies are not kinematic."; return false; }
#if defined(_M_X64)
        std::vector<PxTransform> poses;
        poses.reserve(m_Bodies.size());
        for (size_t i = 0; i < m_Bodies.size(); ++i)
        {
            const int node = m_BoneNodes[i];
            if (node < 0 || size_t(node) >= boneWorld.size()) { error = "Invalid animated PA bone index."; return false; }
            const auto pose = ToPx(PhysicsBoneFrame(boneWorld[node]) * m_Asset.bodies[i].GetLocalTransform());
            if (!pose.isValid()) { error = "Invalid animated PA transform."; return false; }
            poses.push_back(pose);
        }
        for (size_t i = 0; i < m_Bodies.size(); ++i)
        {
            if (teleport && m_LastKinematicTargets.size() == m_Bodies.size())
            {
                const auto previous = ToPx(m_LastKinematicTargets[i]);
                // Teleport/clip reset invalidates finite-difference velocities;
                // the ordinary post-tick refresh of the same pose preserves them.
                if ((poses[i].p - previous.p).magnitudeSquared() > 1e-8f || PxAbs(poses[i].q.dot(previous.q)) < .999999f)
                { m_AnimatedLinearVelocity.clear(); m_AnimatedAngularVelocity.clear(); }
            }
            if (!teleport && m_LastKinematicTargets.size() == m_Bodies.size())
            {
                const auto previous = ToPx(m_LastKinematicTargets[i]);
                const PxVec3 linear = (poses[i].p - previous.p) / float(PhysicsWorld::FixedDeltaTime);
                PxQuat delta = poses[i].q * previous.q.getConjugate();
                if (delta.w < 0) delta = -delta;
                PxReal angle; PxVec3 axis; delta.toRadiansAndUnitAxis(angle, axis);
                m_AnimatedLinearVelocity.resize(m_Bodies.size()); m_AnimatedAngularVelocity.resize(m_Bodies.size());
                m_AnimatedLinearVelocity[i] = Vector3(linear.x, linear.y, linear.z);
                const auto angular = axis * PxMin(angle / float(PhysicsWorld::FixedDeltaTime), 20.0f);
                m_AnimatedAngularVelocity[i] = Vector3(angular.x, angular.y, angular.z);
            }
            if (teleport) m_Bodies[i]->setGlobalPose(poses[i]);
            m_Bodies[i]->setKinematicTarget(poses[i]);
        }
        m_LastKinematicTargets.resize(m_Bodies.size());
        for (size_t i = 0; i < poses.size(); ++i) m_LastKinematicTargets[i] = FromPx(poses[i]);
        error.clear(); return true;
#else
        (void)boneWorld; (void)teleport;
        error = "Ragdoll requires an x64 PhysX build."; return false;
#endif
    }

    size_t Ragdoll::GetKinematicBodyCount() const
    {
        size_t count = 0;
#if defined(_M_X64)
        for (const auto* body : m_Bodies)
            if (body->getRigidBodyFlags().isSet(PxRigidBodyFlag::eKINEMATIC)) ++count;
#endif
        return count;
    }

    Matrix4 Ragdoll::GetBoneWorld(size_t body) const
    {
#if defined(_M_X64)
        return FromPx(m_Bodies[body]->getGlobalPose()) * Invert(m_Asset.bodies[body].GetLocalTransform());
#else
        (void)body; return Matrix4(kIdentity);
#endif
    }
    Vector3 Ragdoll::GetPosition() const
    {
        return IsActive() ? Vector3(GetBoneWorld(0).GetW()) : Vector3(kZero);
    }
    void Ragdoll::AddVelocity(Vector3 velocity)
    {
        if (!IsSimulating()) return;
#if defined(_M_X64)
        const PxVec3 delta(float(velocity.GetX()), float(velocity.GetY()), float(velocity.GetZ()));
        if (delta.isFinite()) for (auto* body : m_Bodies)
            if (!(body->getRigidBodyFlags() & PxRigidBodyFlag::eKINEMATIC)) body->addForce(delta, PxForceMode::eVELOCITY_CHANGE);
#else
        (void)velocity;
#endif
    }
    void Ragdoll::SetQueryOwner(void* owner)
    {
#if defined(_M_X64)
        for (auto* body : m_Bodies) body->userData = owner;
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
            PxRigidActor* a; PxRigidActor* b;
            joint->getActors(a, b);
            result = PxMax(result, ((a->getGlobalPose() * joint->getLocalPose(PxJointActorIndex::eACTOR0)).p -
                (b->getGlobalPose() * joint->getLocalPose(PxJointActorIndex::eACTOR1)).p).magnitude());
        }
#endif
        return result;
    }
    float Ragdoll::GetMaxAngularError(size_t* worstJoint) const
    {
        float result = 0;
        if (worstJoint) *worstJoint = 0;
#if defined(_M_X64)
        for (size_t i = 0; i < m_Joints.size(); ++i)
        {
            const float previous = result;
            const auto& data = m_Asset.constraints[i]; const auto* joint = m_Joints[i];
            // PhysX may return the equivalent angle near +/-2pi when the
            // relative quaternion changes sign. Limits use the shortest twist.
            const float twist = std::remainder(joint->getTwistAngle(), PxTwoPi);
            if (data.twist == PhysicsJointMotion::Locked) result = PxMax(result, PxAbs(twist));
            else if (data.twist == PhysicsJointMotion::Limited) result = PxMax(result, PxMax(data.twistMin - twist, twist - data.twistMax));
            if (data.swingY == PhysicsJointMotion::Limited && data.swingZ == PhysicsJointMotion::Limited)
            {
                // Both coordinates can be below their separate limits while a
                // diagonal swing is outside the actual elliptical cone. Measure
                // radial angular excess using this SDK's ConeLimitHelperTanLess
                // angle ellipse (an error metric, not the solver contact normal).
                const float y = joint->getSwingYAngle(), z = joint->getSwingZAngle();
                const float ny = y / data.swingYLimit, nz = z / data.swingZLimit;
                const float ellipseRadius = std::sqrt(ny * ny + nz * nz);
                if (ellipseRadius > 1)
                {
                    const float ty = std::tan(y * .25f), tz = std::tan(z * .25f);
                    const float py = std::tan(y / ellipseRadius * .25f), pz = std::tan(z / ellipseRadius * .25f);
                    const float r2 = ty * ty + tz * tz, p2 = py * py + pz * pz;
                    // Dot product of actual and boundary pure-swing quaternions.
                    const float dot = ((1 - r2) * (1 - p2) + 4 * (ty * py + tz * pz)) / ((1 + r2) * (1 + p2));
                    result = PxMax(result, 2 * std::acos(PxClamp(PxAbs(dot), 0.0f, 1.0f)));
                }
            }
            else
            {
                if (data.swingY != PhysicsJointMotion::Free)
                    result = PxMax(result, PxAbs(joint->getSwingYAngle()) - (data.swingY == PhysicsJointMotion::Locked ? 0 : data.swingYLimit));
                if (data.swingZ != PhysicsJointMotion::Free)
                    result = PxMax(result, PxAbs(joint->getSwingZAngle()) - (data.swingZ == PhysicsJointMotion::Locked ? 0 : data.swingZLimit));
            }
            if (worstJoint && result > previous) *worstJoint = i;
        }
#endif
        return result;
    }
    float Ragdoll::GetLowestPoint() const
    {
        float result = 1e30f;
#if defined(_M_X64)
        for (const auto* body : m_Bodies) result = PxMin(result, body->getWorldBounds(1.0f).minimum.y);
#endif
        return result;
    }
    bool Ragdoll::IsFinite() const
    {
#if defined(_M_X64)
        for (auto* body : m_Bodies)
            if (!body->getGlobalPose().isValid() || !body->getLinearVelocity().isFinite() || !body->getAngularVelocity().isFinite()) return false;
#endif
        return true;
    }
}
