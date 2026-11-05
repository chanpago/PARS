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
            pose = m_Bodies[root]->getGlobalPose() * m_CharacterCapsuleShape->getLocalPose();
            radius = geometry.radius;
            halfLength = geometry.halfHeight;
            targetOffset = m_SupportJoint ? m_SupportJoint->getDrivePosition().p - m_Bodies[root]->getGlobalPose().p : PxVec3(0);
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
        enabled = enabled && IsSimulating() && m_CollisionEnabled;
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
                if (shape == m_CharacterCapsuleShape)
                {
                    PxFilterData capsuleData;
                    capsuleData.word2 = m_CharacterImpactGroup;
                    capsuleData.word3 = PhysicsWorld::CharacterCapsuleFilterFlag;
                    shape->setSimulationFilterData(capsuleData);
                    continue;
                }
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
        if (!IsSimulating() || !m_CollisionEnabled || root < 0 || !std::isfinite(mass) || mass < .01f || mass > 10000 ||
            !std::isfinite(radiusScale) || radiusScale <= 0 || !std::isfinite(heightScale) || heightScale <= 0 ||
            !std::isfinite(skin) || skin < 0 || skin > .1f)
        {
            error = u8"몸통 캡슐에는 동적 바디와 유효한 질량·형상 설정이 필요합니다.";
            return false;
        }
        PxRigidBody& body = *m_Bodies[root];
        if (body.getRigidBodyFlags().isSet(PxRigidBodyFlag::eKINEMATIC))
        {
            error = u8"키네마틱 바디에는 몸통 충돌 질량을 설정할 수 없습니다.";
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
            PxShape* original = nullptr;
            PxMaterial* material = nullptr;
            body.getShapes(&original, 1);
            if (!original || original->getMaterials(&material, 1) == 0 ||
                !GetCharacterImpactCapsule(radiusScale, pose, radius, halfLength, targetOffset, heightScale))
            {
                error = u8"몸통 캡슐 형상과 재질을 준비하지 못했습니다.";
                return false;
            }
            m_CharacterCapsuleShape = PxRigidActorExt::createExclusiveShape(body, PxCapsuleGeometry(radius, halfLength), *material);
            if (!m_CharacterCapsuleShape)
            {
                error = u8"몸통 캡슐 충돌체를 만들지 못했습니다.";
                return false;
            }
            m_OriginalCapsuleMass = body.getMass();
            m_OriginalCapsuleInertia = body.getMassSpaceInertiaTensor();
            m_CapsuleRadiusScale = radiusScale;
            m_CapsuleHeightScale = heightScale;
            m_CapsuleSkin = skin;
            m_CharacterCapsuleShape->setLocalPose(body.getGlobalPose().getInverse() * pose);
            m_CharacterCapsuleShape->setContactOffset(PxMax(.001f, skin));
            m_CharacterCapsuleShape->setRestOffset(0);
            PxFilterData filter;
            filter.word2 = m_CharacterImpactGroup;
            filter.word3 = PhysicsWorld::CharacterCapsuleFilterFlag;
            m_CharacterCapsuleShape->setSimulationFilterData(filter);
            InvalidateContactState();
            RefreshCharacterImpactFilters();
        }
        if (body.getMass() != mass)
        {
            // 기존 질량중심과 질량 분포를 유지하고, 질량 배율에 맞춰 관성을 함께 바꾼다.
            body.setMass(mass);
            body.setMassSpaceInertiaTensor(m_OriginalCapsuleInertia * (mass / m_OriginalCapsuleMass));
            InvalidateContactState();
            WakeBodies();
        }
        error.clear();
        return true;
    }

    void Ragdoll::RemoveCharacterCapsule()
    {
        if (!m_CharacterCapsuleShape)
        {
            return;
        }
        const int root = GetImpactRootBody();
        InvalidateContactState();
        PxRigidBody& body = *m_Bodies[root];
        body.detachShape(*m_CharacterCapsuleShape);
        m_CharacterCapsuleShape = nullptr;
        body.setMass(m_OriginalCapsuleMass);
        body.setMassSpaceInertiaTensor(m_OriginalCapsuleInertia);
        m_OriginalCapsuleMass = 0;
        m_OriginalCapsuleInertia = PxVec3(0);
        m_CapsuleRadiusScale = 0;
        m_CapsuleHeightScale = 0;
        m_CapsuleSkin = 0;
        RefreshCharacterImpactFilters();
        WakeBodies();
    }

    float Ragdoll::GetCharacterCapsuleMass() const
    {
        const int root = GetImpactRootBody();
        return root >= 0 && static_cast<size_t>(root) < m_Bodies.size() ? m_Bodies[root]->getMass() : 0;
    }

    void Ragdoll::GetCharacterCapsuleContacts(std::vector<PhysicsCapsuleContact>& contacts) const
    {
        contacts.clear();
        if (m_World && m_CharacterCapsuleShape)
        {
            m_World->GetCharacterCapsuleContacts(*m_Bodies[GetImpactRootBody()], contacts);
        }
    }
}
