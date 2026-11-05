#include "Ragdoll.h"
#include "CharacterImpact.h"
#include "PhysicsWorld.h"
#include <algorithm>
#include <cctype>
#include <cmath>

using namespace physx;

namespace PARS
{
    namespace
    {
        bool IsSpine(const std::string& name)
        {
            std::string lower = name;
            std::transform(lower.begin(), lower.end(), lower.begin(), [](unsigned char value) -> char
            {
                return static_cast<char>(std::tolower(value));
            });
            return lower.find("spine") != std::string::npos || lower.find("chest") != std::string::npos;
        }
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

    bool Ragdoll::GetCharacterImpactCapsule(float radiusScale, PxTransform& pose, float& radius,
        float& halfLength, PxVec3& targetOffset) const
    {
        const int root = GetImpactRootBody();
        if (!IsSimulating() || !m_CollisionEnabled || root < 0 || !std::isfinite(radiusScale) || radiusScale <= 0)
        {
            return false;
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
        halfLength = PxMax(0.0f, extents.y - radius);
        pose = PxTransform(bounds.getCenter(), PxQuat(PxHalfPi, PxVec3(0, 0, 1)));
        targetOffset = m_SupportJoint ? m_SupportJoint->getDrivePosition().p - m_Bodies[root]->getGlobalPose().p : PxVec3(0);
        targetOffset.y = 0;
        return pose.isValid() && targetOffset.isFinite();
    }

    void Ragdoll::SetCharacterImpactEnabled(bool enabled, uint32_t ownerGroup)
    {
        enabled = enabled && IsSimulating() && m_CollisionEnabled;
        if (m_CharacterImpactEnabled == enabled && m_CharacterImpactGroup == ownerGroup)
        {
            return;
        }
        InvalidateContactState();
        m_CharacterImpactEnabled = enabled;
        m_CharacterImpactGroup = ownerGroup;
        for (PxRigidBody* body : m_Bodies)
        {
            std::vector<PxShape*> shapes(body->getNbShapes());
            body->getShapes(shapes.data(), static_cast<PxU32>(shapes.size()));
            for (PxShape* shape : shapes)
            {
                PxFilterData data = shape->getSimulationFilterData();
                data.word2 = ownerGroup;
                data.word3 = enabled ? data.word3 | PhysicsWorld::CharacterImpactFilterFlag :
                    data.word3 & ~PhysicsWorld::CharacterImpactFilterFlag;
                shape->setSimulationFilterData(data);
            }
            if (body->getScene())
            {
                body->getScene()->resetFiltering(*body);
            }
        }
    }

    bool Ragdoll::ApplyBodyImpact(PxVec3 impulse, PxVec3 point, float pelvisShare,
        float deltaTime, std::vector<BodyImpactShare>& shares)
    {
        shares.clear();
        const int root = GetImpactRootBody();
        if (!IsSimulating() || root < 0 || !m_World || !impulse.isFinite() || !point.isFinite() ||
            !std::isfinite(deltaTime) || deltaTime <= 0 || !std::isfinite(pelvisShare) || pelvisShare < 0 || pelvisShare > 1 ||
            !(impulse / deltaTime).isFinite() || !std::isfinite(impulse.magnitude()))
        {
            return false;
        }
        float spineMass = 0;
        for (size_t i = 0; i < m_Bodies.size(); ++i)
        {
            if (static_cast<int>(i) != root && IsSpine(m_Asset.bodies[i].bone))
            {
                spineMass += m_Bodies[i]->getMass();
            }
        }
        const float rootShare = spineMass > 0 ? pelvisShare : 1.0f;
        // 모든 바디를 검사한 다음 적용한다. 중간 실패로 일부 바디에만 힘이 남지 않게 한다.
        for (const PxRigidBody* body : m_Bodies)
        {
            const PxVec3 center = (body->getGlobalPose() * body->getCMassLocalPose()).p;
            if (!body->getScene() || body->getRigidBodyFlags().isSet(PxRigidBodyFlag::eKINEMATIC) ||
                !(point - center).cross(impulse / deltaTime).isFinite())
            {
                return false;
            }
        }
        for (size_t i = 0; i < m_Bodies.size(); ++i)
        {
            PxRigidBody& body = *m_Bodies[i];
            const float weight = static_cast<int>(i) == root ? rootShare :
                IsSpine(m_Asset.bodies[i].bone) && spineMass > 0 ? (1 - rootShare) * body.getMass() / spineMass : 0;
            if (weight <= 0)
            {
                continue;
            }
            BodyImpactShare share;
            share.bone = m_Asset.bodies[i].bone;
            share.impulse = impulse * weight;
            const PxVec3 force = share.impulse / deltaTime;
            const PxVec3 center = (body.getGlobalPose() * body.getCMassLocalPose()).p;
            body.addForce(force, PxForceMode::eFORCE);
            body.addTorque((point - center).cross(force), PxForceMode::eFORCE);
            shares.push_back(share);
        }
        const uint64_t step = m_World->GetStepCount() + 1;
        const uint64_t clock = m_World->GetClockRevision();
        if (m_GameplayImpactStep != step || m_GameplayImpactClock != clock)
        {
            m_GameplayImpactImpulse = 0;
        }
        m_GameplayImpactStep = step;
        m_GameplayImpactClock = clock;
        m_GameplayImpactImpulse += impulse.magnitude();
        WakeBodies();
        return true;
    }

    float Ragdoll::GetGameplayImpactImpulse() const
    {
        return m_World && m_GameplayImpactStep == m_World->GetStepCount() &&
            m_GameplayImpactClock == m_World->GetClockRevision() ? m_GameplayImpactImpulse : 0;
    }
}
