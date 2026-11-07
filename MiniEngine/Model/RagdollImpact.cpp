#include "Ragdoll.h"
#include "CharacterImpact.h"
#include "PhysicsWorld.h"
#include <algorithm>
#include <cctype>
#include <cmath>

using namespace physx;

namespace PARS
{
    bool Ragdoll::HasApproachingImpact(float minimumDeltaVelocity) const
    {
        Math::Vector3 center(Math::kZero);
        float mass = 0;
        if (!m_World || IsSimulating() || !m_CollisionEnabled || !GetCenterOfMass(center, mass))
        {
            return false;
        }
        for (const PxRigidBody* body : m_Bodies)
        {
            if (m_World->HasApproachingImpact(*body, mass, minimumDeltaVelocity, m_QueryOwner))
            {
                return true;
            }
        }
        return false;
    }

    void Ragdoll::CaptureExternalImpacts(std::vector<RagdollBodyImpulse>& impacts) const
    {
        impacts.clear();
        if (!m_World || !m_CollisionEnabled)
        {
            return;
        }
        for (size_t index = 0; index < m_Bodies.size(); ++index)
        {
            std::vector<PhysicsContactImpulse> contacts;
            m_World->GetExternalImpactImpulse(*m_Bodies[index], m_QueryOwner, &contacts);
            for (const PhysicsContactImpulse& contact : contacts)
            {
                RagdollBodyImpulse impact;
                impact.body = index;
                impact.position = contact.position;
                impact.impulse = contact.impulse;
                impacts.push_back(impact);
            }
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
                // 관절 링크는 충격 모드를 지원하지 않으므로 한 고정 스텝의 힘으로 적분한다.
                const PxVec3 force = impact.impulse / static_cast<float>(PhysicsWorld::FixedDeltaTime);
                PxRigidBodyExt::addForceAtPos(*m_Bodies[impact.body], force, impact.position, PxForceMode::eFORCE);
            }
        }
    }

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

    bool Ragdoll::GetCharacterImpactCapsule(float radiusScale, PxTransform& pose, float& radius,
        float& halfLength, PxVec3& targetOffset, float heightScale) const
    {
        const int root = GetImpactRootBody();
        if (!IsSimulating() || !m_CollisionEnabled || root < 0 || !std::isfinite(radiusScale) || radiusScale <= 0 ||
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

    void Ragdoll::SetCharacterImpactEnabled(bool enabled, uint32_t ownerGroup)
    {
        enabled = enabled && IsActive() && m_CollisionEnabled;
        if (!enabled)
        {
            RemoveCharacterCapsule();
        }
        if (m_CharacterImpactEnabled == enabled && m_CharacterImpactGroup == ownerGroup)
        {
            return;
        }
        InvalidateContactState();
        m_CharacterImpactEnabled = enabled;
        m_CharacterImpactGroup = ownerGroup;
        RefreshCharacterImpactFilters();
    }

    void Ragdoll::RefreshCharacterImpactFilters()
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
                shape->setSimulationFilterData(data);
            }
            if (body->getScene())
            {
                body->getScene()->resetFiltering(*body);
            }
        }
        if (m_CharacterCapsuleShape)
        {
            PxFilterData capsuleData;
            capsuleData.word2 = m_CharacterImpactGroup;
            capsuleData.word3 = PhysicsWorld::CharacterCapsuleFilterFlag;
            m_CharacterCapsuleShape->setSimulationFilterData(capsuleData);
            if (m_CharacterCapsuleBody->getScene())
            {
                m_CharacterCapsuleBody->getScene()->resetFiltering(*m_CharacterCapsuleBody);
            }
        }
    }

    bool Ragdoll::ConfigureCharacterCapsule(bool enabled, float mass, float radiusScale, float skin, std::string& error,
        float heightScale)
    {
        if (!enabled)
        {
            RemoveCharacterCapsule();
            error.clear();
            return true;
        }
        const int root = GetImpactRootBody();
        if (!m_World || !IsSimulating() || !m_CollisionEnabled || root < 0 || !std::isfinite(mass) || mass < .01f || mass > 10000 ||
            !std::isfinite(radiusScale) || radiusScale <= 0 || !std::isfinite(heightScale) || heightScale <= 0 ||
            !std::isfinite(skin) || skin < 0 || skin > .1f)
        {
            error = u8"몸통 캡슐에는 동적 바디와 유효한 충돌 질량·형상 설정이 필요합니다.";
            return false;
        }
        PxRigidBody& body = *m_Bodies[root];
        if (!body.getScene() || body.getRigidBodyFlags().isSet(PxRigidBodyFlag::eKINEMATIC))
        {
            error = u8"씬에 등록된 동적 몸통에만 독립 충돌 캡슐을 사용할 수 있습니다.";
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
            InvalidateContactState();
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
        error.clear();
        return true;
    }

    void Ragdoll::SyncCharacterCapsule()
    {
        const int root = GetImpactRootBody();
        if (!m_CharacterCapsuleBody || !IsSimulating() || root < 0)
        {
            return;
        }
        const PxRigidBody& body = *m_Bodies[root];
        const PxTransform pose = body.getGlobalPose() * m_CapsuleRootPose;
        const PxVec3 linear = GetBodyPointVelocity(static_cast<size_t>(root), pose.p);
        const PxVec3 angular = body.getAngularVelocity();
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
        InvalidateContactState();
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
