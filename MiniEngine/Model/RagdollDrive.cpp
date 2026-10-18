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
            DirectX::XMFLOAT4 value; DirectX::XMStoreFloat4(&value, q);
            return PxQuat(value.x, value.y, value.z, value.w);
        }
        float Angle(PxQuat a, PxQuat b) { return 2 * std::acos(PxClamp(PxAbs(a.dot(b)), 0.0f, 1.0f)); }
        bool Enabled(const PhysicsAngularDrive& drive, PhysicsJointMotion motion)
        {
            return drive.enabled && motion != PhysicsJointMotion::Locked && drive.maxTorque > 0 &&
                (drive.stiffness > 0 || drive.damping > 0);
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
                // Force-based PD, with a per-axis Nm limit. PhysX performs the
                // solve; do not apply another manual torque controller on top.
                const bool on = m_DrivesEnabled && Enabled(drive, motions[axis]);
                m_Joints[i]->setDrive(data.useSlerpDrive ? PxD6Drive::eSLERP : axes[axis], on ? PxD6JointDrive(drive.stiffness, drive.damping, drive.maxTorque, false) : PxD6JointDrive());
            }
        }
        for (auto* body : m_Bodies)
        {
            // Active motors use 32 position / 8 velocity iterations to balance
            // animated tracking with Debug solve cost. Passive uses 16/4.
            body->setSolverIterationCounts(m_DrivesEnabled ? 32 : 16, m_DrivesEnabled ? 8 : 4);
            if (!(body->getRigidBodyFlags() & PxRigidBodyFlag::eKINEMATIC)) body->wakeUp();
        }
#endif
        if (!m_DrivesEnabled) m_HasDriveTargets = false;
    }

    bool Ragdoll::ApplyDriveSettings(const PhysicsAsset& asset, std::string& error)
    {
        if (!asset.Validate(error)) return false;
        if (asset.constraints.size() != m_Asset.constraints.size()) { error = "Drive update requires the same joints."; return false; }
        for (size_t i = 0; i < asset.constraints.size(); ++i)
            if (asset.constraints[i].parentBone != m_Asset.constraints[i].parentBone ||
                asset.constraints[i].childBone != m_Asset.constraints[i].childBone)
            { error = "Drive update requires matching parent/child bodies."; return false; }
        auto candidate = m_Asset;
        for (size_t i = 0; i < asset.constraints.size(); ++i)
        {
            candidate.constraints[i].useSlerpDrive = asset.constraints[i].useSlerpDrive;
            for (int axis = 0; axis < 3; ++axis) candidate.constraints[i].drives[axis] = asset.constraints[i].drives[axis];
        }
        // Validate against the LIVE joint motions, not the editor's potentially
        // different limits. This update deliberately does not replace frames.
        if (!candidate.Validate(error)) return false;
        for (size_t i = 0; i < candidate.constraints.size(); ++i)
        {
            m_Asset.constraints[i].useSlerpDrive = candidate.constraints[i].useSlerpDrive;
            for (int axis = 0; axis < 3; ++axis) m_Asset.constraints[i].drives[axis] = candidate.constraints[i].drives[axis];
        }
        SetDrivesEnabled(m_DrivesEnabled);
        error.clear(); return true;
    }

    bool Ragdoll::UpdateDriveTargets(const std::vector<Matrix4>& boneWorld, float deltaTime, std::string& error)
    {
        if (!IsActive() || !std::isfinite(deltaTime) || deltaTime < 0) { error = "Invalid drive update."; return false; }
#if defined(_M_X64)
        std::vector<Matrix4> bodies;
        bodies.reserve(m_Bodies.size());
        PxTransform supportedPose(PxIdentity);
        for (size_t i = 0; i < m_Bodies.size(); ++i)
        {
            const int node = m_BoneNodes[i];
            if (node < 0 || size_t(node) >= boneWorld.size()) { error = "Drive target has no matching bone."; return false; }
            const auto transform = PhysicsBoneFrame(boneWorld[node]) * m_Asset.bodies[i].GetLocalTransform();
            const auto frame = PhysicsJointFrame::FromTransform(transform);
            if (!ToPx(Quaternion(DirectX::XMLoadFloat4(&frame.rotation))).isSane())
            { error = "Invalid drive target rotation."; return false; }
            if (int(i) == m_SupportBody)
            {
                supportedPose = PxTransform(PxVec3(frame.offset.x, frame.offset.y, frame.offset.z),
                    ToPx(Quaternion(DirectX::XMLoadFloat4(&frame.rotation))));
                if (!supportedPose.isValid()) { error = "Invalid supported pelvis target."; return false; }
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
            if (!ToPx(target).isSane()) { error = "Non-finite joint drive target."; return false; }
            targets.push_back(target);
            m_MaxTargetClampAngle = PxMax(m_MaxTargetClampAngle, Angle(ToPx(target), ToPx(EvaluateJointDriveTarget(data, a, b, false))));
        }
        // Only the supported pelvis follows the animated world pose directly.
        // Its PA body offset is already included above. Queue the target before
        // simulate(), so PhysX derives the motion that drives connected limbs;
        // the other bodies remain dynamic and follow their joint motors.
        if (m_SupportBody >= 0) m_Bodies[m_SupportBody]->setKinematicTarget(supportedPose);
        for (size_t i = 0; i < targets.size(); ++i)
        {
            const PxQuat q = ToPx(targets[i]);
            PxVec3 angular(0);
            const bool history = m_HasDriveTargets && m_DriveTargets.size() == targets.size() && deltaTime > 0;
            bool changed = !history || Angle(q, ToPx(m_DriveTargets[i])) > 1e-4f;
            const auto& data = m_Asset.constraints[i];
            if (m_DrivesEnabled && HasEnabledAngularDrive(data))
            {
                PxRigidActor* a; PxRigidActor* b; m_Joints[i]->getActors(a, b);
                const PxQuat parent = a->getGlobalPose().q * m_Joints[i]->getLocalPose(PxJointActorIndex::eACTOR0).q;
                const PxQuat child = b->getGlobalPose().q * m_Joints[i]->getLocalPose(PxJointActorIndex::eACTOR1).q;
                // Small limb inertia can satisfy PhysX's energy sleep threshold
                // while still far from a static motor target. Keep solving that error.
                changed |= Angle(parent.getConjugate() * child, q) > .03f;
            }
            if (history)
            {
                PxQuat delta = q * ToPx(m_DriveTargets[i]).getConjugate();
                if (delta.w < 0) delta = -delta;
                PxReal angle; PxVec3 axis; delta.toRadiansAndUnitAxis(angle, axis);
                angular = axis * PxMin(angle / deltaTime, 20.0f);
                // This SDK's SWING_TWIST rows use parent-minus-child velocity;
                // SLERP uses child-minus-parent (see ExtD6JointSolverPrep).
                if (!data.useSlerpDrive) angular = -angular;
            }
            m_Joints[i]->setDrivePosition(PxTransform(PxVec3(0), q), changed);
            m_Joints[i]->setDriveVelocity(PxVec3(0), angular, changed);
        }
        m_DriveTargets = std::move(targets); m_HasDriveTargets = true;
        error.clear(); return true;
#else
        (void)boneWorld; error = "Drives require x64 PhysX."; return false;
#endif
    }

    bool Ragdoll::SetRootSupported(bool supported, std::string& error)
    {
#if defined(_M_X64)
        if (supported == IsRootSupported()) { error.clear(); return true; }
        int root = -1;
        if (supported)
        {
            if (!IsSimulating()) { error = "Support requires a simulated ragdoll."; return false; }
            for (size_t i = 0; i < m_Asset.bodies.size(); ++i)
            {
                bool child = false;
                for (const auto& joint : m_Asset.constraints) child |= joint.childBone == m_Asset.bodies[i].bone;
                if (!child)
                {
                    if (root >= 0) { error = "Pelvis support requires one root body."; return false; }
                    root = int(i);
                }
            }
            if (root < 0) { error = "PA has no root body to support."; return false; }
        }
        if (m_SupportBody >= 0)
        {
            auto* body = m_Bodies[m_SupportBody];
            // Carry the last simulated animation motion into passive dynamics.
            const auto linear = body->getLinearVelocity(), angular = body->getAngularVelocity();
            body->setRigidBodyFlag(PxRigidBodyFlag::eKINEMATIC, false);
            body->setLinearVelocity(linear); body->setAngularVelocity(angular); body->wakeUp();
        }
        m_SupportBody = root;
        if (root >= 0)
        {
            auto* body = m_Bodies[root];
            const auto pose = body->getGlobalPose();
            body->setRigidBodyFlag(PxRigidBodyFlag::eKINEMATIC, true);
            // Hold until the next active animation update supplies a target.
            body->setKinematicTarget(pose);
        }
        error.clear(); return true;
#else
        (void)supported; error = "Support requires x64 PhysX."; return false;
#endif
    }

    size_t Ragdoll::GetDrivenJointCount() const
    {
        size_t count = 0;
#if defined(_M_X64)
        if (m_DrivesEnabled)
            for (const auto& joint : m_Asset.constraints)
                if (HasEnabledAngularDrive(joint)) ++count;
#endif
        return count;
    }

    float Ragdoll::GetMaxDriveError(size_t* worstJoint) const
    {
        float result = 0;
        if (worstJoint) *worstJoint = 0;
#if defined(_M_X64)
        if (!m_DrivesEnabled || !m_HasDriveTargets) return result;
        for (size_t i = 0; i < m_Joints.size(); ++i)
        {
            const auto& data = m_Asset.constraints[i];
            if (!HasEnabledAngularDrive(data)) continue;
            PxRigidActor* a; PxRigidActor* b; m_Joints[i]->getActors(a, b);
            const PxQuat parent = a->getGlobalPose().q * m_Joints[i]->getLocalPose(PxJointActorIndex::eACTOR0).q;
            const PxQuat child = b->getGlobalPose().q * m_Joints[i]->getLocalPose(PxJointActorIndex::eACTOR1).q;
            const float error = Angle(parent.getConjugate() * child, ToPx(m_DriveTargets[i]));
            if (error > result) { result = error; if (worstJoint) *worstJoint = i; }
        }
#endif
        return result;
    }
}
