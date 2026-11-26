#include "Ragdoll.h"
#include "CharacterImpact.h"
#include "FallArmBrace.h"
#include "PhysicsWorld.h"
#include <algorithm>
#include <cctype>
#include <cmath>
#include <utility>

using namespace physx;

namespace PARS
{
    namespace
    {
        PxD6Joint* CreateWoundDrive(PhysicsWorld& world, PxRigidBody& target, const PxTransform& targetAnchor,
            PxRigidBody& hand, const PxTransform& palmAnchor)
        {
            PxD6Joint* joint = world.CreateJoint(target, targetAnchor, hand, palmAnchor);
            if (joint)
            {
                for (unsigned axis = 0; axis < PxD6Axis::eCOUNT; ++axis)
                {
                    joint->setMotion(static_cast<PxD6Axis::Enum>(axis), PxD6Motion::eFREE);
                }
                joint->setConstraintFlag(PxConstraintFlag::eDRIVE_LIMITS_ARE_FORCES, true);
                joint->setConstraintFlag(PxConstraintFlag::eCOLLISION_ENABLED, true);
            }
            return joint;
        }

        void UpdateWoundAnchor(PxD6Joint& joint, const PxTransform& anchor)
        {
            const PxTransform previous = joint.getLocalPose(PxJointActorIndex::eACTOR0);
            if ((previous.p - anchor.p).magnitudeSquared() > 1e-8f ||
                previous.q.getBasisVector0().dot(anchor.q.getBasisVector0()) < .999999f)
            {
                joint.setLocalPose(PxJointActorIndex::eACTOR0, anchor);
            }
        }
    }

    bool Ragdoll::HasApproachingImpact(float minimumDeltaVelocity, float deferCapsuleDeltaVelocity) const
    {
        Math::Vector3 center(Math::kZero);
        float mass = 0;
        if (!m_World || IsSimulating() || !m_CollisionEnabled || !GetCenterOfMass(center, mass))
        {
            return false;
        }
        for (const PxRigidBody* body : m_Bodies)
        {
            if (m_World->HasApproachingImpact(*body, mass, minimumDeltaVelocity, m_QueryOwner, deferCapsuleDeltaVelocity))
            {
                return true;
            }
        }
        return false;
    }

    const std::vector<RagdollBodyImpulse>& Ragdoll::GetExternalImpacts() const
    {
        if (!m_World || !m_CollisionEnabled)
        {
            m_ExternalImpacts.clear();
            return m_ExternalImpacts;
        }
        const uint64_t step = m_World->GetStepCount();
        const uint64_t clock = m_World->GetClockRevision();
        if (m_ImpactSampleStep == step && m_ImpactSampleClock == clock)
        {
            return m_ExternalImpacts;
        }
        // 발 선택과 관절 구동은 같은 완료 스텝의 접촉을 공유한다.
        m_ImpactSampleStep = step;
        m_ImpactSampleClock = clock;
        m_ExternalImpacts.clear();
        std::vector<PhysicsContactImpulse> contacts;
        for (size_t index = 0; index < m_Bodies.size(); ++index)
        {
            m_World->GetExternalImpactImpulse(*m_Bodies[index], m_QueryOwner, &contacts);
            for (const PhysicsContactImpulse& contact : contacts)
            {
                RagdollBodyImpulse impact;
                impact.body = index;
                impact.impactContactId = contact.impactContactId;
                impact.position = contact.position;
                impact.impulse = contact.impulse;
                impact.outwardNormal = contact.outwardNormal;
                impact.fromProjectile = contact.fromProjectile;
                impact.newProjectileHit = contact.newProjectileHit;
                impact.fromCharacterCapsule = contact.fromCharacterCapsule;
                impact.newCapsuleHit = contact.newCapsuleHit;
                impact.fromAnimatedCapsule = contact.fromAnimatedCapsule;
                impact.capsuleVelocity = contact.capsuleVelocity;
                impact.capsuleGroup = contact.capsuleGroup;
                m_ExternalImpacts.push_back(impact);
            }
        }
        return m_ExternalImpacts;
    }

    void Ragdoll::CaptureExternalImpacts(std::vector<RagdollBodyImpulse>& impacts) const
    {
        impacts = GetExternalImpacts();
    }

    namespace
    {
        void ApplyImpulseAtPoint(PxRigidBody& body, const PxVec3& impulse, const PxVec3& position)
        {
            // 관절 링크도 같은 충격량을 받도록 한 고정 스텝 동안 힘으로 적분한다.
            const PxVec3 force = impulse / static_cast<float>(PhysicsWorld::FixedDeltaTime);
            if (force.isFinite())
            {
                PxRigidBodyExt::addForceAtPos(body, force, position, PxForceMode::eFORCE);
            }
        }

        const RagdollBodyImpulse* FindStrongestImpactContact(const std::vector<RagdollBodyImpulse>& impacts,
            uint64_t contactId, size_t bodyCount)
        {
            const RagdollBodyImpulse* strongest = nullptr;
            float largestSquaredImpulse = 0;
            for (const RagdollBodyImpulse& hit : impacts)
            {
                if (hit.impactContactId != contactId || hit.body >= bodyCount || !hit.position.isFinite())
                {
                    continue;
                }
                const float squaredImpulse = hit.impulse.magnitudeSquared();
                if (std::isfinite(squaredImpulse) && squaredImpulse > largestSquaredImpulse)
                {
                    strongest = &hit;
                    largestSquaredImpulse = squaredImpulse;
                }
            }
            return strongest;
        }
    }

    void Ragdoll::ApplyExternalImpacts(const std::vector<RagdollBodyImpulse>& impacts)
    {
        if (!IsSimulating())
        {
            return;
        }
        // 키네마틱 바디가 받지 못했던 충격만 전환 직후 동일한 접촉점에 한 번 적용한다.
        for (const RagdollBodyImpulse& impact : impacts)
        {
            if (impact.body < m_Bodies.size() && impact.position.isFinite() && impact.impulse.isFinite())
            {
                ApplyImpulseAtPoint(*m_Bodies[impact.body], impact.impulse, impact.position);
            }
        }
    }

    PxVec3 Ragdoll::GetImpactAngularVelocity(const PxVec3& center, const PxVec3& angularImpulse) const
    {
        PxMat33 inertia(PxZero);
        for (const PxRigidBody* body : m_Bodies)
        {
            const PxTransform massFrame = body->getGlobalPose() * body->getCMassLocalPose();
            const PxMat33 rotation(massFrame.q);
            const PxVec3 offset = massFrame.p - center;
            // 각 바디의 관성을 월드 축으로 회전하고 평행축 정리로 전신 질량중심에 모은다.
            inertia += rotation * PxMat33::createDiagonal(body->getMassSpaceInertiaTensor()) * rotation.getTranspose();
            inertia += (PxMat33(PxIdentity) * offset.magnitudeSquared() - PxMat33::outer(offset, offset)) * body->getMass();
        }
        const float determinant = inertia.getDeterminant();
        if (!std::isfinite(determinant) || determinant <= 0)
        {
            return PxVec3(0);
        }
        return inertia.getInverse() * angularImpulse;
    }

    namespace
    {
        constexpr float ImpactHoldTime = .08f;
        constexpr float ImpactRecoveryTime = .24f;
        constexpr float ImpactResponseTime = ImpactHoldTime + ImpactRecoveryTime;

        float ImpactYield(float remaining)
        {
            const float t = PxClamp(remaining / ImpactRecoveryTime, 0.0f, 1.0f);
            return t * t * (3.0f - 2.0f * t);
        }

        bool IsSpine(const std::string& name)
        {
            std::string lower = name;
            std::transform(lower.begin(), lower.end(), lower.begin(), [](unsigned char value) -> char
            {
                return static_cast<char>(std::tolower(value));
            });
            return lower.find("spine") != std::string::npos || lower.find("chest") != std::string::npos;
        }

        bool IsLowerBody(const std::string& name)
        {
            std::string lower = name;
            std::transform(lower.begin(), lower.end(), lower.begin(), [](unsigned char value) -> char
            {
                return static_cast<char>(std::tolower(value));
            });
            return lower.find("leg") != std::string::npos || lower.find("thigh") != std::string::npos ||
                lower.find("calf") != std::string::npos || lower.find("knee") != std::string::npos ||
                lower.find("foot") != std::string::npos || lower.find("ankle") != std::string::npos ||
                lower.find("toe") != std::string::npos;
        }
    }

    bool Ragdoll::IsPrimaryProjectileHit(const RagdollBodyImpulse& hit, const std::vector<RagdollBodyImpulse>& impacts) const
    {
        if (!hit.newProjectileHit || !hit.fromProjectile || hit.body >= m_Bodies.size() || !hit.position.isFinite() ||
            !hit.impulse.isFinite() || hit.impulse.magnitudeSquared() <= 1e-8f)
        {
            return false;
        }
        // 한 발이 여러 형상에 걸치면 부위 충격과 같은 접촉점을 상처로 사용한다.
        return hit.impactContactId == 0 || FindStrongestImpactContact(impacts, hit.impactContactId, m_Bodies.size()) == &hit;
    }

    float Ragdoll::GetWoundStrength(size_t joint) const
    {
        for (const JointWound& wound : m_JointWounds)
        {
            if (wound.joint == joint)
            {
                return wound.strength;
            }
        }
        return 1.0f;
    }

    float Ragdoll::GetWoundGripAge(size_t joint) const
    {
        for (const JointWound& wound : m_JointWounds)
        {
            if (wound.joint == joint)
            {
                return wound.gripAge;
            }
        }
        return -1.0f;
    }

    bool Ragdoll::IsHoldingWound() const
    {
        return m_WoundGrip && m_WoundGrip->getMotion(PxD6Axis::eX) == PxD6Motion::eLOCKED;
    }

    void Ragdoll::ReleaseWoundGrip()
    {
        ReleaseWoundSupport();
        m_LastRejectedGripStep = UINT64_MAX;
        m_RejectedGripSteps = 0;
        if (m_WoundGrip)
        {
            m_WoundGrip->release();
            m_WoundGrip = nullptr;
        }
    }

    void Ragdoll::DriveWoundGrip(size_t joint, size_t handBody, size_t hitBody, const PxTransform& palm,
        const PxTransform& target, float weight, bool canHold, bool canAssist)
    {
        if (!m_DrivesEnabled || !m_World || handBody >= m_Bodies.size() || hitBody >= m_Bodies.size() || handBody == hitBody ||
            !palm.isSane() || !target.isSane() || !(weight > 0 && weight <= 1.0f))
        {
            ReleaseWoundGrip();
            return;
        }
        constexpr float lockDistance = .04f;
        constexpr float holdDistance = .06f;
        constexpr uint32_t releaseSteps = 3;
        const bool wasHolding = IsHoldingWound();
        const float distance = (palm.p - target.p).magnitude();
        const bool atContact = PalmFitsWound(palm, target, lockDistance);
        if ((!canHold && !canAssist) || (wasHolding && !PalmFitsWound(palm, target, holdDistance)))
        {
            if (!wasHolding || weight < 1.0f)
            {
                ReleaseWoundGrip();
                return;
            }
            // 고정한 손만 일시적인 흔들림을 허용한다. 같은 물리 틱의 중복 호출은 한 번만 센다.
            const uint64_t step = m_World->GetStepCount();
            if (step != m_LastRejectedGripStep)
            {
                m_RejectedGripSteps = m_RejectedGripSteps > 0 && step - m_LastRejectedGripStep == 1 ? m_RejectedGripSteps + 1 : 1;
                m_LastRejectedGripStep = step;
            }
            if (m_RejectedGripSteps >= releaseSteps)
            {
                ReleaseWoundGrip();
            }
            return;
        }
        m_RejectedGripSteps = 0;
        m_LastRejectedGripStep = UINT64_MAX;
        PxRigidBody& hit = *m_Bodies[hitBody];
        PxRigidBody& hand = *m_Bodies[handBody];
        // D6 스윙의 X축을 손등 법선과 맞춰 손가락 방향으로 도는 비틀림은 자유롭게 둔다.
        const PxTransform normalAxis(PxQuat(PxHalfPi, PxVec3(0, 0, 1)));
        const PxTransform hitAnchor = hit.getGlobalPose().transformInv(target * normalAxis);
        if (!m_WoundGrip)
        {
            const PxTransform palmAnchor = hand.getGlobalPose().transformInv(palm * normalAxis);
            m_WoundGrip = CreateWoundDrive(*m_World, hit, hitAnchor, hand, palmAnchor);
            if (!m_WoundGrip)
            {
                return;
            }
        }
        else
        {
            UpdateWoundAnchor(*m_WoundGrip, hitAnchor);
        }

        const float alignment = weight * PxClamp((.16f - distance) / .12f, 0.0f, 1.0f);
        const PxD6JointDrive swingDrive(95.0f * alignment, 4.0f * alignment, 22.0f * alignment);
        if (swingDrive.forceLimit != m_WoundGrip->getDrive(PxD6Drive::eSWING).forceLimit)
        {
            m_WoundGrip->setDrive(PxD6Drive::eSWING, swingDrive);
        }
        // 실제 손바닥의 거리와 면 방향이 함께 맞아야 위치를 고정한다.
        const bool holding = weight >= 1.0f && (wasHolding || (canHold && atContact));
        if (holding != wasHolding)
        {
            if (!holding)
            {
                ReleaseWoundSupport();
            }
            const PxD6Motion::Enum motion = holding ? PxD6Motion::eLOCKED : PxD6Motion::eFREE;
            m_WoundGrip->setMotion(PxD6Axis::eX, motion);
            m_WoundGrip->setMotion(PxD6Axis::eY, motion);
            m_WoundGrip->setMotion(PxD6Axis::eZ, motion);
            m_WoundGrip->setConstraintFlag(PxConstraintFlag::eCOLLISION_ENABLED, !holding);
        }
        if (holding)
        {
            for (JointWound& wound : m_JointWounds)
            {
                if (wound.joint == joint && wound.gripAge < 0)
                {
                    wound.gripAge = static_cast<float>((m_World->GetStepCount() - wound.hitStep) * PhysicsWorld::FixedDeltaTime);
                    break;
                }
            }
        }
        else
        {
            // 반대편 위팔까지 몸을 가로질러 닿을 수 있게 접근력을 제한하면서 접촉 직전의 오차도 줄인다.
            const PxD6JointDrive drive(9000.0f * weight, 150.0f * weight, 360.0f * weight);
            if (drive.forceLimit != m_WoundGrip->getDrive(PxD6Drive::eX).forceLimit)
            {
                m_WoundGrip->setDrive(PxD6Drive::eX, drive);
                m_WoundGrip->setDrive(PxD6Drive::eY, drive);
                m_WoundGrip->setDrive(PxD6Drive::eZ, drive);
            }
        }
    }

    void Ragdoll::DriveWoundSupport(size_t leadingBody, size_t supportingBody, const PxTransform& palm,
        const PxTransform& target, float weight)
    {
        if (!IsHoldingWound() || !m_World || leadingBody >= m_Bodies.size() || supportingBody >= m_Bodies.size() ||
            leadingBody == supportingBody || !palm.isSane() || !target.isSane() || !(weight > 0 && weight <= 1.0f))
        {
            ReleaseWoundSupport();
            return;
        }
        PxRigidBody& leading = *m_Bodies[leadingBody];
        PxRigidBody& supporting = *m_Bodies[supportingBody];
        const PxTransform normalAxis(PxQuat(PxHalfPi, PxVec3(0, 0, 1)));
        const PxTransform anchor = leading.getGlobalPose().transformInv(target * normalAxis);
        if (!m_WoundSupport)
        {
            const PxTransform palmAnchor = supporting.getGlobalPose().transformInv(palm * normalAxis);
            m_WoundSupport = CreateWoundDrive(*m_World, leading, anchor, supporting, palmAnchor);
            if (!m_WoundSupport)
            {
                return;
            }
        }
        else
        {
            UpdateWoundAnchor(*m_WoundSupport, anchor);
        }
        // 두 손 사이에는 힘이 제한된 스프링만 둬 충돌하거나 팔이 밀리면 자연스럽게 벌어지게 한다.
        const PxD6JointDrive position(2000.0f * weight, 70.0f * weight, 90.0f * weight);
        if (m_WoundSupport->getDrive(PxD6Drive::eX).forceLimit != position.forceLimit)
        {
            m_WoundSupport->setDrive(PxD6Drive::eX, position);
            m_WoundSupport->setDrive(PxD6Drive::eY, position);
            m_WoundSupport->setDrive(PxD6Drive::eZ, position);
            m_WoundSupport->setDrive(PxD6Drive::eSWING, PxD6JointDrive(24.0f * weight, 2.0f * weight, 5.0f * weight));
        }
    }

    void Ragdoll::ReleaseWoundSupport()
    {
        if (m_WoundSupport)
        {
            m_WoundSupport->release();
            m_WoundSupport = nullptr;
        }
    }

    bool Ragdoll::GetWoundSupportDistance(float& meters) const
    {
        if (!m_WoundSupport)
        {
            return false;
        }
        meters = m_WoundSupport->getRelativeTransform().p.magnitude();
        return true;
    }

    void Ragdoll::UpdateWounds(const std::vector<RagdollBodyImpulse>& impacts)
    {
        if (!m_World)
        {
            return;
        }
        const uint64_t clock = m_World->GetClockRevision();
        const uint64_t step = m_World->GetStepCount();
        bool drivesChanged = false;
        if (clock != m_WoundClock)
        {
            ReleaseWoundGrip();
            drivesChanged = !m_JointWounds.empty();
            m_JointWounds.clear();
            m_WoundClock = clock;
        }
        constexpr float approachEnd = ImpactPalmGoal::FullReachTime;
        for (std::vector<JointWound>::iterator wound = m_JointWounds.begin(); wound != m_JointWounds.end();)
        {
            const float age = static_cast<float>((step - wound->hitStep) * PhysicsWorld::FixedDeltaTime);
            const float holdEnd = ImpactPalmGoal::GetWoundReleaseStart(wound->gripAge);
            if (age >= holdEnd + WoundRecoveryTime)
            {
                wound = m_JointWounds.erase(wound);
                drivesChanged = true;
                continue;
            }
            float strength = wound->minimumStrength;
            if (age < approachEnd)
            {
                const float t = age / approachEnd;
                strength = wound->initialStrength + (wound->minimumStrength - wound->initialStrength) * t * t * (3.0f - 2.0f * t);
            }
            else if (age > holdEnd)
            {
                const float t = (age - holdEnd) / WoundRecoveryTime;
                strength += (1.0f - wound->minimumStrength) * t * t * (3.0f - 2.0f * t);
            }
            drivesChanged = drivesChanged || strength != wound->strength;
            wound->strength = strength;
            ++wound;
        }
        for (const RagdollBodyImpulse& hit : impacts)
        {
            if (!IsPrimaryProjectileHit(hit, impacts))
            {
                continue;
            }
            float minimumStrength = 1.0f;
            const int joint = FindWoundJoint(hit.body, &minimumStrength);
            if (joint < 0)
            {
                continue;
            }
            const size_t index = static_cast<size_t>(joint);
            std::vector<JointWound>::iterator wound = std::find_if(m_JointWounds.begin(), m_JointWounds.end(),
                [index](const JointWound& entry) -> bool
                {
                    return entry.joint == index;
                });
            if (wound == m_JointWounds.end())
            {
                m_JointWounds.emplace_back();
                wound = m_JointWounds.end() - 1;
                wound->joint = index;
            }
            // 재피격은 남은 힘에서 다시 약화해 관절이 한 스텝 동안 강해지는 것을 막는다.
            wound->initialStrength = wound->strength;
            wound->minimumStrength = minimumStrength;
            wound->hitStep = step;
            wound->gripAge = -1.0f;
        }
        if (drivesChanged && m_DrivesEnabled)
        {
            if (!m_Articulation)
            {
                ConfigureD6JointDrives();
            }
            WakeBodies();
        }
    }

    std::vector<RagdollImpactResponse> Ragdoll::GetImpactResponses(const std::vector<RagdollBodyImpulse>& impacts) const
    {
        std::vector<RagdollImpactResponse> responses;
        for (const RagdollBodyImpulse& hit : impacts)
        {
            RagdollImpactResponse response;
            if (hit.impactContactId == 0 || !m_World || !m_World->GetImpactResponse(hit.impactContactId, response) ||
                std::any_of(responses.begin(), responses.end(), [&hit](const RagdollImpactResponse& entry) -> bool
                {
                    return entry.contactId == hit.impactContactId;
                }))
            {
                continue;
            }
            const RagdollBodyImpulse* struck = FindStrongestImpactContact(impacts, hit.impactContactId, m_Bodies.size());
            if (struck)
            {
                response.contactId = hit.impactContactId;
                response.body = struck->body;
                response.partPosition = struck->position;
                response.fromCharacterCapsule = struck->fromCharacterCapsule;
                responses.push_back(response);
            }
        }
        return responses;
    }

    PxVec3 Ragdoll::ApplyImpactResponses(const std::vector<RagdollImpactResponse>& responses)
    {
        if (!m_World || !IsSimulating())
        {
            return PxVec3(0);
        }
        PxVec3 center(0);
        float mass = 0;
        PxVec3 linearImpulse(0);
        PxVec3 groundedProjectileImpulse(0);
        PxVec3 angularImpulse(0);
        for (const RagdollImpactResponse& response : responses)
        {
            if (response.body >= m_Bodies.size() || !m_World->ConsumeImpactContact(response.contactId))
            {
                continue;
            }
            if (!response.partImpulse.isZero())
            {
                // 머리와 목처럼 여러 부위가 동시에 맞아도 실제 충격이 가장 컸던 접촉점에 한 번만 가한다.
                ApplyImpulseAtPoint(*m_Bodies[response.body], response.partImpulse, response.partPosition);
            }
            if (response.impulse.isZero())
            {
                continue;
            }
            if (mass == 0)
            {
                Math::Vector3 position(Math::kZero);
                if (!GetCenterOfMass(position, mass) || !std::isfinite(mass) || mass <= 0)
                {
                    return PxVec3(0);
                }
                center = PxVec3(position.GetX(), position.GetY(), position.GetZ());
            }
            // 지상 화살 피격의 전신 이동에서만 상승분을 뺀다. 회전에는 원래 충격량을 사용한다.
            linearImpulse += response.GetLinearImpulse();
            if (response.groundedProjectileHit)
            {
                groundedProjectileImpulse += PxVec3(response.impulse.x, 0, response.impulse.z);
            }
            angularImpulse += response.GetAngularImpulse(center);
        }
        if (mass == 0 || !linearImpulse.isFinite() || !angularImpulse.isFinite())
        {
            return PxVec3(0);
        }
        PxVec3 linear = linearImpulse / mass;
        if (!angularImpulse.isZero())
        {
            const PxVec3 angular = GetImpactAngularVelocity(center, angularImpulse);
            if (!angular.isFinite() || !linear.isFinite())
            {
                return PxVec3(0);
            }
            if (m_Articulation)
            {
                // 루트 질량중심에 같은 강체 운동을 주면 기존 관절 속도를 유지한 채 모든 링크에 전파된다.
                const PxRigidBody* root = m_Bodies[m_ArticulationRoot];
                const PxVec3 rootCenter = (root->getGlobalPose() * root->getCMassLocalPose()).p;
                linear += angular.cross(rootCenter - center);
                m_Articulation->setRootAngularVelocity(m_Articulation->getRootAngularVelocity() + angular);
            }
            else
            {
                for (PxRigidBody* body : m_Bodies)
                {
                    if (body->getRigidBodyFlags().isSet(PxRigidBodyFlag::eKINEMATIC))
                    {
                        continue;
                    }
                    const PxVec3 partCenter = (body->getGlobalPose() * body->getCMassLocalPose()).p;
                    body->addTorque(angular, PxForceMode::eVELOCITY_CHANGE);
                    body->addForce(angular.cross(partCenter - center), PxForceMode::eVELOCITY_CHANGE);
                }
            }
        }
        AddVelocity(Math::Vector3(linear.x, linear.y, linear.z));
        return groundedProjectileImpulse / mass;
    }

    float RagdollImpactReaction::GetJointDriveScale(size_t joint) const
    {
        return joint < jointYieldTimes.size() ? 1.0f - .96f * ImpactYield(jointYieldTimes[joint]) : 1.0f;
    }

    float Ragdoll::GetProjectilePelvisDriveScale() const
    {
        return 1.0f - .85f * ImpactYield(m_ImpactReaction.projectilePelvisYieldTime);
    }

    void Ragdoll::UpdateImpactReaction(const std::vector<RagdollBodyImpulse>& impacts, float deltaTime)
    {
        if (m_DrivesEnabled && AdvanceImpactReaction(m_ImpactReaction, impacts, deltaTime) && !m_Articulation)
        {
            ConfigureD6JointDrives();
            ConfigureRootSupportDrives();
        }
    }

    bool Ragdoll::AdvanceImpactReaction(RagdollImpactReaction& reaction, const std::vector<RagdollBodyImpulse>& impacts,
        float deltaTime) const
    {
        if (!m_World || !std::isfinite(deltaTime) || deltaTime <= 0)
        {
            return false;
        }
        const uint64_t clock = m_World->GetClockRevision();
        const uint64_t step = m_World->GetStepCount();
        const bool wasYielding = reaction.IsYielding();
        if (reaction.clock != clock)
        {
            reaction = {};
        }
        else if (reaction.step == step)
        {
            return false;
        }
        reaction.clock = clock;
        reaction.step = step;
        reaction.projectilePelvisYieldTime = PxMax(0.0f, reaction.projectilePelvisYieldTime - deltaTime);
        for (float& remaining : reaction.jointYieldTimes)
        {
            remaining = PxMax(0.0f, remaining - deltaTime);
        }
        RecordImpactReaction(reaction, impacts);
        if (std::none_of(reaction.jointYieldTimes.begin(), reaction.jointYieldTimes.end(), [](float time) -> bool
            {
                return time > 0;
            }))
        {
            reaction.jointYieldTimes.clear();
        }
        return wasYielding || reaction.IsYielding();
    }

    void Ragdoll::RecordImpactReaction(RagdollImpactReaction& reaction, const std::vector<RagdollBodyImpulse>& impacts) const
    {
        // 한 번의 접촉이 여러 점으로 나뉘어도 같은 바디의 충격량과 골반 기준 각충격량을 합친다.
        std::vector<PxVec3> impulses;
        PxVec3 angularImpulse(0);
        PxVec3 pelvisCenter(0);
        int root = -1;
        for (const RagdollBodyImpulse& hit : impacts)
        {
            if (!hit.fromProjectile || hit.body >= m_Bodies.size() ||
                !hit.position.isFinite() || !hit.impulse.isFinite() || hit.impulse.magnitudeSquared() < 1e-8f)
            {
                continue;
            }
            if (impulses.empty())
            {
                root = GetImpactRootBody();
                if (root < 0)
                {
                    return;
                }
                const PxRigidBody& pelvis = *m_Bodies[root];
                pelvisCenter = (pelvis.getGlobalPose() * pelvis.getCMassLocalPose()).p;
                impulses.resize(m_Bodies.size(), PxVec3(0));
            }
            impulses[hit.body] += hit.impulse;
            angularImpulse += (hit.position - pelvisCenter).cross(hit.impulse);
        }
        if (impulses.empty())
        {
            return;
        }
        float totalMass = 0;
        for (const PxRigidBody* body : m_Bodies)
        {
            totalMass += body->getMass();
        }
        // 투사체가 골반을 중심으로 가한 각충격량에 따라 회전 보조를 잠시 줄인다.
        const float rotation = angularImpulse.magnitude() / (.3f * totalMass);
        reaction.projectilePelvisYieldTime = PxMax(reaction.projectilePelvisYieldTime,
            ImpactResponseTime * PxClamp((rotation - .1f) / .65f, 0.0f, 1.0f));
        for (size_t body = 0; body < impulses.size(); ++body)
        {
            const float mass = PxMax(m_Bodies[body]->getMass(), .06f * totalMass);
            const float speedChange = impulses[body].magnitude() / mass;
            const float amount = PxClamp((speedChange - .4f) / 3.5f, 0.0f, 1.0f);
            if (amount > 0)
            {
                YieldImpactJoints(reaction.jointYieldTimes, body, body == static_cast<size_t>(root), amount);
            }
        }
    }

    void Ragdoll::YieldImpactJoints(std::vector<float>& jointYieldTimes, size_t body, bool isRoot, float amount) const
    {
        jointYieldTimes.resize(m_Asset.constraints.size(), 0);
        const std::string& hitBone = m_Asset.bodies[body].bone;
        if (isRoot || IsSpine(hitBone))
        {
            for (size_t joint = 0; joint < m_Asset.constraints.size(); ++joint)
            {
                const PhysicsConstraint& child = m_Asset.constraints[joint];
                if (child.parentBone == hitBone && IsSpine(child.childBone))
                {
                    jointYieldTimes[joint] = PxMax(jointYieldTimes[joint], ImpactResponseTime * amount * .84f);
                }
            }
        }
        // 맞은 관절에서 부모 두 단계까지 전파해 반대편 팔다리의 자세 구동은 유지한다.
        const std::string* bone = &hitBone;
        for (unsigned depth = 0; depth < 3; ++depth)
        {
            const std::vector<PhysicsConstraint>::const_iterator joint = std::find_if(m_Asset.constraints.begin(),
                m_Asset.constraints.end(), [bone](const PhysicsConstraint& entry) -> bool
                {
                    return entry.childBone == *bone;
                });
            if (joint == m_Asset.constraints.end())
            {
                break;
            }
            const size_t index = static_cast<size_t>(joint - m_Asset.constraints.begin());
            jointYieldTimes[index] = PxMax(jointYieldTimes[index], ImpactResponseTime * amount);
            bone = &joint->parentBone;
            amount *= .80f;
        }
    }

    bool Ragdoll::IsTopplingFromProjectile() const
    {
        if (m_ImpactReaction.projectilePelvisYieldTime <= 0)
        {
            return false;
        }
        const int root = GetImpactRootBody();
        if (root < 0)
        {
            return false;
        }
        const PxRigidBody& pelvis = *m_Bodies[root];
        const PxVec3 base = (pelvis.getGlobalPose() * pelvis.getCMassLocalPose()).p;
        PxVec3 torso(0);
        PxVec3 velocity(0);
        for (size_t body = 0; body < m_Bodies.size(); ++body)
        {
            if (IsSpine(m_Asset.bodies[body].bone))
            {
                const PxRigidBody& spine = *m_Bodies[body];
                const PxVec3 offset = (spine.getGlobalPose() * spine.getCMassLocalPose()).p - base;
                if (offset.magnitudeSquared() > torso.magnitudeSquared())
                {
                    torso = offset;
                    velocity = spine.getLinearVelocity() - pelvis.getLinearVelocity();
                }
            }
        }
        // 위쪽 척추가 이미 35도를 넘고, 0.15초 뒤 기울기가 50도를 넘으며 계속 증가할 때만 지지를 놓는다.
        const PxVec3 predicted = torso + velocity * .15f;
        if (torso.normalize() < 1e-4f || predicted.magnitudeSquared() < 1e-8f)
        {
            return false;
        }
        const float predictedUp = predicted.getNormalized().y;
        return torso.y < std::cos(PxPi * 35.0f / 180.0f) && predictedUp < std::cos(PxPi * 50.0f / 180.0f) && predictedUp < torso.y;
    }

    int Ragdoll::GetImpactRootBody() const
    {
        if (m_ArticulationRoot >= 0)
        {
            return m_ArticulationRoot;
        }
        for (size_t i = 0; i < m_Asset.bodies.size(); ++i)
        {
            const std::string& bone = m_Asset.bodies[i].bone;
            bool child = false;
            for (const PhysicsConstraint& constraint : m_Asset.constraints)
            {
                child = child || constraint.childBone == bone;
            }
            if (!child)
            {
                return static_cast<int>(i);
            }
        }
        return -1;
    }

    int Ragdoll::FindPhysicsBody(const PxRigidActor* actor) const
    {
        for (size_t i = 0; i < m_Bodies.size(); ++i)
        {
            if (m_Bodies[i] == actor)
            {
                return static_cast<int>(i);
            }
        }
        return -1;
    }

    bool Ragdoll::IsCharacterImpactBody(size_t body) const
    {
        const int root = GetImpactRootBody();
        if (body >= m_Asset.bodies.size() || root < 0)
        {
            return false;
        }
        // 플레이어 캡슐은 골반과 척추·가슴 바디에만 직접 충격을 전달한다.
        if (static_cast<int>(body) != root && !IsSpine(m_Asset.bodies[body].bone))
        {
            return false;
        }
        int current = static_cast<int>(body);
        for (size_t depth = 0; depth < m_Asset.bodies.size(); ++depth)
        {
            if (current == root)
            {
                return true;
            }
            const std::string& bone = m_Asset.bodies[current].bone;
            if (IsLowerBody(bone))
            {
                return false;
            }
            int parent = -1;
            for (const PhysicsConstraint& constraint : m_Asset.constraints)
            {
                if (constraint.childBone == bone)
                {
                    parent = m_Asset.FindBody(constraint.parentBone);
                    break;
                }
            }
            if (parent < 0)
            {
                return false;
            }
            // 이름에 하체 표기가 없는 보조 바디도 다리 계층에 속하면 제외한다.
            current = parent;
        }
        return false;
    }

    PxVec3 Ragdoll::GetBodyPointVelocity(size_t body, PxVec3 point) const
    {
        if (body >= m_Bodies.size() || !point.isFinite())
        {
            return PxVec3(0);
        }
        const PxRigidBody& rigid = *m_Bodies[body];
        const PxVec3 center = (rigid.getGlobalPose() * rigid.getCMassLocalPose()).p;
        return rigid.getLinearVelocity() + rigid.getAngularVelocity().cross(point - center);
    }

    float Ragdoll::GetImpactRootYawSpeed() const
    {
        const int root = GetImpactRootBody();
        if (root < 0)
        {
            return 0;
        }
        return m_Bodies[root]->getAngularVelocity().y;
    }

    bool Ragdoll::GetCharacterImpactCapsule(float radiusScale, PxTransform& pose, float& radius,
        float& halfLength, PxVec3& targetOffset, float heightScale) const
    {
        const int root = GetImpactRootBody();
        if (!IsActive() || !m_CollisionEnabled || root < 0 || !std::isfinite(radiusScale) || radiusScale <= 0 ||
            !std::isfinite(heightScale) || heightScale <= 0)
        {
            return false;
        }
        if (m_CharacterCapsuleShape)
        {
            const PxCapsuleGeometry& geometry = static_cast<const PxCapsuleGeometry&>(m_CharacterCapsuleShape->getGeometry());
            pose = m_CharacterCapsuleBody->getGlobalPose();
            radius = geometry.radius;
            halfLength = geometry.halfHeight;
            targetOffset = (m_Bodies[root]->getGlobalPose() * m_CapsuleRootPose).p - pose.p;
            if (m_SupportJoint)
            {
                targetOffset += m_SupportJoint->getDrivePosition().p - m_Bodies[root]->getGlobalPose().p;
            }
            targetOffset.y = 0;
            return pose.isValid();
        }
        PxBounds3 bounds = PxBounds3::empty();
        for (size_t i = 0; i < m_Bodies.size(); ++i)
        {
            if (static_cast<int>(i) != root && !IsSpine(m_Asset.bodies[i].bone))
            {
                continue;
            }
            const PxRigidBody& body = *m_Bodies[i];
            std::vector<PxShape*> shapes(body.getNbShapes());
            body.getShapes(shapes.data(), static_cast<PxU32>(shapes.size()));
            for (const PxShape* shape : shapes)
            {
                PxBounds3 shapeBounds;
                PxGeometryQuery::computeGeomBounds(shapeBounds, shape->getGeometry(), body.getGlobalPose() * shape->getLocalPose());
                bounds.include(shapeBounds);
            }
        }
        if (bounds.isEmpty() || !bounds.isFinite())
        {
            return false;
        }
        const PxVec3 extents = bounds.getExtents();
        radius = PxMax(.05f, PxMax(extents.x, extents.z) * radiusScale);
        halfLength = PxMax(0.0f, extents.y * heightScale - radius);
        pose = PxTransform(bounds.getCenter(), PxQuat(PxHalfPi, PxVec3(0, 0, 1)));
        targetOffset = m_SupportJoint ? m_SupportJoint->getDrivePosition().p - m_Bodies[root]->getGlobalPose().p : PxVec3(0);
        targetOffset.y = 0;
        return pose.isValid() && targetOffset.isFinite();
    }

    void Ragdoll::SetCharacterImpactEnabled(bool enabled, uint32_t ownerGroup, bool sweepKinematicTorso)
    {
        enabled = enabled && IsActive() && m_CollisionEnabled;
        if (!enabled)
        {
            RemoveCharacterCapsule();
        }
        sweepKinematicTorso = enabled && !IsSimulating() && sweepKinematicTorso;
        const bool policyChanged = m_CharacterImpactEnabled != enabled || m_CharacterImpactGroup != ownerGroup;
        if (!policyChanged && m_SweepKinematicTorso == sweepKinematicTorso)
        {
            return;
        }
        // 이동 스윕만 켜는 예열 첫 틱에도 이미 성립한 실제 접촉은 유지한다.
        if (policyChanged)
        {
            // 같은 캐릭터의 충돌 필터를 갱신해도 이미 시작한 관절 회복은 계속한다.
            InvalidateContactState(false);
        }
        m_CharacterImpactEnabled = enabled;
        m_SweepKinematicTorso = sweepKinematicTorso;
        m_CharacterImpactGroup = ownerGroup;
        RefreshCharacterImpactFilters(policyChanged);
    }

    void Ragdoll::RefreshCharacterImpactFilters(bool updateSimulationFilters)
    {
        for (size_t i = 0; i < m_Bodies.size(); ++i)
        {
            PxRigidBody* body = m_Bodies[i];
            std::vector<PxShape*> shapes(body->getNbShapes());
            body->getShapes(shapes.data(), static_cast<PxU32>(shapes.size()));
            for (PxShape* shape : shapes)
            {
                PxFilterData data = shape->getSimulationFilterData();
                data.word2 = m_CharacterImpactGroup;
                data.word3 &= ~(PhysicsWorld::CharacterImpactFilterFlag | PhysicsWorld::CharacterImpactTargetFilterFlag |
                    PhysicsWorld::CharacterCapsuleOwnerFilterFlag);
                if (m_CharacterImpactEnabled)
                {
                    data.word3 |= PhysicsWorld::CharacterImpactFilterFlag;
                    if (IsCharacterImpactBody(i))
                    {
                        data.word3 |= PhysicsWorld::CharacterImpactTargetFilterFlag;
                    }
                    if (m_CharacterCapsuleShape)
                    {
                        data.word3 |= PhysicsWorld::CharacterCapsuleOwnerFilterFlag;
                    }
                }
                if (updateSimulationFilters)
                {
                    shape->setSimulationFilterData(data);
                }
                shape->setFlag(PxShapeFlag::eSCENE_QUERY_SHAPE, IsSimulating() ||
                    (m_SweepKinematicTorso && (data.word3 & PhysicsWorld::CharacterImpactTargetFilterFlag) != 0));
            }
            if (updateSimulationFilters && body->getScene())
            {
                body->getScene()->resetFiltering(*body);
            }
        }
        if (updateSimulationFilters && m_CharacterCapsuleShape)
        {
            PxFilterData capsuleData;
            capsuleData.word2 = m_CharacterImpactGroup;
            capsuleData.word3 = PhysicsWorld::CharacterCapsuleFilterFlag;
            if (!IsSimulating())
            {
                capsuleData.word3 |= PhysicsWorld::AnimatedCapsuleFilterFlag;
            }
            m_CharacterCapsuleShape->setSimulationFilterData(capsuleData);
            if (m_CharacterCapsuleBody->getScene())
            {
                m_CharacterCapsuleBody->getScene()->resetFiltering(*m_CharacterCapsuleBody);
            }
        }
    }

    bool Ragdoll::ConfigureCharacterCapsule(bool enabled, float mass, float radiusScale, float skin, std::string& error,
        float heightScale, const ImpactImpulseSettings& response)
    {
        if (!enabled)
        {
            RemoveCharacterCapsule();
            error.clear();
            return true;
        }
        const int root = GetImpactRootBody();
        if (!m_World || !IsActive() || !m_CollisionEnabled || root < 0 || !std::isfinite(mass) || mass < .01f || mass > 10000 ||
            !std::isfinite(radiusScale) || radiusScale <= 0 || !std::isfinite(heightScale) || heightScale <= 0 ||
            !std::isfinite(skin) || skin < 0 || skin > .1f || !response.IsValid())
        {
            error = u8"몸통 캡슐에는 PA 바디와 유효한 충돌 질량·형상 설정이 필요합니다.";
            return false;
        }
        PxRigidBody& body = *m_Bodies[root];
        if (!body.getScene())
        {
            error = u8"씬에 등록된 몸통에만 독립 충돌 캡슐을 사용할 수 있습니다.";
            return false;
        }
        if (m_CharacterCapsuleShape && (m_CapsuleRadiusScale != radiusScale || m_CapsuleHeightScale != heightScale || m_CapsuleSkin != skin))
        {
            RemoveCharacterCapsule();
        }
        if (!m_CharacterCapsuleShape)
        {
            PxTransform pose(PxIdentity);
            PxVec3 targetOffset(0);
            float radius = 0;
            float halfLength = 0;
            if (!GetCharacterImpactCapsule(radiusScale, pose, radius, halfLength, targetOffset, heightScale))
            {
                error = u8"몸통 캡슐 형상을 준비하지 못했습니다.";
                return false;
            }
            // PA 바디 목록과 Articulation에 넣지 않는다. 유한 질량은 이 독립 바디의 접촉 솔버에만 사용한다.
            m_CharacterCapsuleBody = m_World->CreateDynamicBody(pose, PxCapsuleGeometry(radius, halfLength), mass,
                "Character collision capsule", m_Asset.bodies[root].material);
            if (!m_CharacterCapsuleBody)
            {
                error = u8"독립 동적 몸통 캡슐을 만들지 못했습니다.";
                return false;
            }
            m_CharacterCapsuleBody->getShapes(&m_CharacterCapsuleShape, 1);
            m_CharacterCapsuleBody->setActorFlag(PxActorFlag::eDISABLE_GRAVITY, true);
            m_CharacterCapsuleBody->userData = m_QueryOwner;
            // 이동·카메라 쿼리는 PA를 사용한다. 보조 캡슐은 다른 상체와의 실제 접촉에만 참여한다.
            m_CharacterCapsuleShape->setFlag(PxShapeFlag::eSCENE_QUERY_SHAPE, false);
            m_CapsuleRootPose = body.getGlobalPose().getInverse() * pose;
            m_CapsuleRadiusScale = radiusScale;
            m_CapsuleHeightScale = heightScale;
            m_CapsuleSkin = skin;
            m_CharacterCapsuleShape->setContactOffset(PxMax(.001f, skin));
            m_CharacterCapsuleShape->setRestOffset(0);
            InvalidateContactState(false);
            RefreshCharacterImpactFilters();
            SyncCharacterCapsule();
        }
        if (m_CharacterCapsuleBody->getMass() != mass)
        {
            if (!PxRigidBodyExt::setMassAndUpdateInertia(*m_CharacterCapsuleBody, mass))
            {
                error = u8"독립 캡슐의 충돌 질량·관성을 갱신하지 못했습니다.";
                return false;
            }
            m_World->InvalidateContacts(*m_CharacterCapsuleBody);
            m_CharacterCapsuleBody->wakeUp();
        }
        m_World->SetImpactImpulseSettings(*m_CharacterCapsuleBody, response);
        error.clear();
        return true;
    }

    void Ragdoll::SyncCharacterCapsule(const PxVec3& pushVelocity)
    {
        const int root = GetImpactRootBody();
        if (!m_CharacterCapsuleBody || !IsActive() || root < 0)
        {
            return;
        }
        const PxRigidBody& body = *m_Bodies[root];
        const PxTransform pose = body.getGlobalPose() * m_CapsuleRootPose;
        PxVec3 linear = GetBodyPointVelocity(static_cast<size_t>(root), pose.p);
        PxVec3 angular = body.getAngularVelocity();
        if (!IsSimulating())
        {
            // 키네마틱 속도는 simulate()에서 갱신되므로 현재 스텝의 목표로 직접 계산한다.
            // 캡슐 중심의 회전 이동도 포함하되, 충돌 반작용은 애니메이션 본체로 돌려주지 않는다.
            linear = PxVec3(0);
            angular = PxVec3(0);
            PxTransform target(PxIdentity);
            const PxRigidDynamic* kinematic = body.is<PxRigidDynamic>();
            if (kinematic && kinematic->getKinematicTarget(target))
            {
                const PxTransform capsuleTarget = target * m_CapsuleRootPose;
                const float deltaTime = static_cast<float>(PhysicsWorld::FixedDeltaTime);
                linear = (capsuleTarget.p - pose.p) / deltaTime;
                PxQuat delta = capsuleTarget.q * pose.q.getConjugate();
                if (delta.w < 0)
                {
                    delta = -delta;
                }
                float angle = 0;
                PxVec3 axis(0);
                delta.toRadiansAndUnitAxis(angle, axis);
                angular = axis * (angle / deltaTime);
            }
        }
        // 관통 방지로 이동 목표에서 잘린 법선 속도만 보충한다. 충돌량은 캡슐 질량으로 솔버가 계산한다.
        PxVec3 pushDirection = pushVelocity;
        const float pushSpeed = pushDirection.normalize();
        linear += pushDirection * PxMax(0.0f, pushSpeed - linear.dot(pushDirection));
        if (!pose.isValid() || !linear.isFinite() || !angular.isFinite())
        {
            return;
        }
        // 몸통에서 보조 바디로만 복사한다. 직전 접촉의 반작용을 PA나 PD에 전달하지 않는다.
        m_World->InvalidateContacts(*m_CharacterCapsuleBody);
        m_CharacterCapsuleBody->setGlobalPose(pose);
        m_CharacterCapsuleBody->setLinearVelocity(linear);
        m_CharacterCapsuleBody->setAngularVelocity(angular);
    }

    void Ragdoll::RemoveCharacterCapsule()
    {
        if (!m_CharacterCapsuleBody)
        {
            return;
        }
        InvalidateContactState(false);
        m_CharacterCapsuleBody->release();
        m_CharacterCapsuleBody = nullptr;
        m_CharacterCapsuleShape = nullptr;
        m_CapsuleRootPose = PxTransform(PxIdentity);
        m_CapsuleRadiusScale = 0;
        m_CapsuleHeightScale = 0;
        m_CapsuleSkin = 0;
        RefreshCharacterImpactFilters();
    }

    float Ragdoll::GetImpactRootMass() const
    {
        const int root = GetImpactRootBody();
        return root >= 0 && static_cast<size_t>(root) < m_Bodies.size() ? m_Bodies[root]->getMass() : 0;
    }

    float Ragdoll::GetCharacterCapsuleMass() const
    {
        return m_CharacterCapsuleBody ? m_CharacterCapsuleBody->getMass() : 0;
    }

    void Ragdoll::GetCharacterCapsuleContacts(std::vector<PhysicsCapsuleContact>& contacts) const
    {
        contacts.clear();
        if (m_World && m_CharacterCapsuleBody)
        {
            m_World->GetCharacterCapsuleContacts(*m_CharacterCapsuleBody, contacts);
        }
    }
}
