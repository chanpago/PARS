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
                impact.position = contact.position;
                impact.impulse = contact.impulse;
                impact.fromProjectile = contact.fromProjectile;
                m_ExternalImpacts.push_back(impact);
            }
        }
        return m_ExternalImpacts;
    }

    void Ragdoll::CaptureExternalImpacts(std::vector<RagdollBodyImpulse>& impacts) const
    {
        impacts = GetExternalImpacts();
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
        constexpr float ProjectileHoldTime = .08f;
        constexpr float ProjectileRecoveryTime = .24f;
        constexpr float ProjectileResponseTime = ProjectileHoldTime + ProjectileRecoveryTime;

        float ProjectileYield(float remaining)
        {
            const float t = PxClamp(remaining / ProjectileRecoveryTime, 0.0f, 1.0f);
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

    float Ragdoll::GetProjectileJointDriveScale(size_t jointIndex) const
    {
        return jointIndex < m_JointYieldTimes.size() ? 1.0f - .96f * ProjectileYield(m_JointYieldTimes[jointIndex]) : 1.0f;
    }

    float Ragdoll::GetProjectilePelvisDriveScale() const
    {
        return 1.0f - .85f * ProjectileYield(m_PelvisYieldTime);
    }

    void Ragdoll::UpdateProjectileReaction(const std::vector<RagdollBodyImpulse>& impacts, float deltaTime)
    {
        if (!m_World || !m_DrivesEnabled || !std::isfinite(deltaTime) || deltaTime <= 0)
        {
            return;
        }
        const uint64_t clock = m_World->GetClockRevision();
        const uint64_t step = m_World->GetStepCount();
        const bool hadReaction = !m_JointYieldTimes.empty() || m_PelvisYieldTime > 0;
        if (m_ProjectileReactionClock != clock)
        {
            m_JointYieldTimes.clear();
            m_PelvisYieldTime = 0;
        }
        else if (m_ProjectileReactionStep == step)
        {
            return;
        }
        m_ProjectileReactionClock = clock;
        m_ProjectileReactionStep = step;
        m_PelvisYieldTime = PxMax(0.0f, m_PelvisYieldTime - deltaTime);
        for (float& remaining : m_JointYieldTimes)
        {
            remaining = PxMax(0.0f, remaining - deltaTime);
        }
        RecordProjectileImpacts(impacts);
        if (std::none_of(m_JointYieldTimes.begin(), m_JointYieldTimes.end(), [](float time) -> bool
            {
                return time > 0;
            }))
        {
            m_JointYieldTimes.clear();
        }
        if (!m_Articulation && (hadReaction || !m_JointYieldTimes.empty() || m_PelvisYieldTime > 0))
        {
            ConfigureD6JointDrives();
            ConfigureRootSupportDrives();
        }
    }

    void Ragdoll::RecordProjectileImpacts(const std::vector<RagdollBodyImpulse>& impacts)
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
        // 지렛팔 30cm를 기준으로 전신을 돌릴 만큼 큰 충격일 때만 골반의 회전 보조도 잠시 줄인다.
        const float rotation = angularImpulse.magnitude() / (.3f * totalMass);
        m_PelvisYieldTime = PxMax(m_PelvisYieldTime, ProjectileResponseTime * PxClamp((rotation - .1f) / .65f, 0.0f, 1.0f));
        for (size_t body = 0; body < impulses.size(); ++body)
        {
            const float mass = PxMax(m_Bodies[body]->getMass(), .06f * totalMass);
            const float speedChange = impulses[body].magnitude() / mass;
            const float amount = PxClamp((speedChange - .4f) / 3.5f, 0.0f, 1.0f);
            if (amount > 0)
            {
                YieldImpactJoints(body, body == static_cast<size_t>(root), amount);
            }
        }
    }

    void Ragdoll::YieldImpactJoints(size_t body, bool isRoot, float amount)
    {
        m_JointYieldTimes.resize(m_Asset.constraints.size(), 0);
        const std::string& hitBone = m_Asset.bodies[body].bone;
        if (isRoot || IsSpine(hitBone))
        {
            for (size_t joint = 0; joint < m_Asset.constraints.size(); ++joint)
            {
                const PhysicsConstraint& child = m_Asset.constraints[joint];
                if (child.parentBone == hitBone && IsSpine(child.childBone))
                {
                    m_JointYieldTimes[joint] = PxMax(m_JointYieldTimes[joint], ProjectileResponseTime * amount * .84f);
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
            m_JointYieldTimes[index] = PxMax(m_JointYieldTimes[index], ProjectileResponseTime * amount);
            bone = &joint->parentBone;
            amount *= .80f;
        }
    }

    bool Ragdoll::IsTopplingFromProjectile() const
    {
        if (m_PelvisYieldTime <= 0)
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
        if (!m_World || !IsActive() || !m_CollisionEnabled || root < 0 || !std::isfinite(mass) || mass < .01f || mass > 10000 ||
            !std::isfinite(radiusScale) || radiusScale <= 0 || !std::isfinite(heightScale) || heightScale <= 0 ||
            !std::isfinite(skin) || skin < 0 || skin > .1f)
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
