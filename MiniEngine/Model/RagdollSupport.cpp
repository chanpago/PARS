#include "Ragdoll.h"
#include "PhysicsWorld.h"
#include <algorithm>
#include <cmath>

using namespace PARS;
using namespace physx;

std::vector<float> Ragdoll::ComputeSupportGeneralizedForce()
{
    m_SupportLoads = {};
#if defined(_M_X64)
    const unsigned count = m_Articulation ? 6 + m_Articulation->getDofs() : 0;
    std::vector<float> generalized(count, 0);
    if (!m_FootDrivenSupport || !m_Articulation || !m_ArticulationCache || !m_Articulation->getScene())
    {
        return generalized;
    }
    std::array<bool, 2> valid{};
    unsigned supports = 0;
    for (size_t side = 0; side < m_SupportContacts.size(); ++side)
    {
        const RagdollSupportContact& contact = m_SupportContacts[side];
        valid[side] = contact.supporting && contact.body >= 0 &&
            static_cast<size_t>(contact.body) < m_Bodies.size() && contact.position.isFinite() &&
            contact.normal.isFinite() && contact.normal.y >= .70710678f;
        if (valid[side])
        {
            ++supports;
        }
    }
    if (supports == 0)
    {
        return generalized;
    }
    Math::Vector3 center;
    float mass = 0;
    if (!GetCenterOfMass(center, mass))
    {
        return generalized;
    }
    if (supports == 2)
    {
        // 양발 사이로 투영한 COM에 따라 하중을 분담한다. 지면반력 측정값과는 별개의 목표값이다.
        const PxVec3 span = m_SupportContacts[1].position - m_SupportContacts[0].position;
        const PxVec3 relative = PxVec3(float(center.GetX()), float(center.GetY()), float(center.GetZ())) -
            m_SupportContacts[0].position;
        const float lengthSquared = span.x * span.x + span.z * span.z;
        const float right = lengthSquared > 1e-8f ?
            PxClamp((relative.x * span.x + relative.z * span.z) / lengthSquared, 0.0f, 1.0f) : .5f;
        m_SupportLoads.fractions = {1.0f - right, right};
    }
    else
    {
        m_SupportLoads.fractions = {valid[0] ? 1.0f : 0.0f, valid[1] ? 1.0f : 0.0f};
    }
    PxArticulationCache& cache = *m_ArticulationCache;
    PxU32 rows = 0, columns = 0;
    m_Articulation->computeDenseJacobian(cache, rows, columns);
    if (columns != count)
    {
        m_SupportLoads = {};
        return generalized;
    }
    const float weight = (std::max)(0.0f, -mass * m_Articulation->getScene()->getGravity().y);
    for (size_t side = 0; side < m_SupportContacts.size(); ++side)
    {
        if (!valid[side])
        {
            continue;
        }
        const RagdollSupportContact& contact = m_SupportContacts[side];
        const PxRigidBody& body = *m_Bodies[contact.body];
        const PxArticulationLink* link = body.is<PxArticulationLink>();
        if (!link || 6 * link->getLinkIndex() + 5 >= rows)
        {
            m_SupportLoads = {};
            return std::vector<float>(count, 0);
        }
        const PxVec3 force = contact.normal * (weight * m_SupportLoads.fractions[side] / contact.normal.y);
        const PxVec3 centerOfMass = (body.getGlobalPose() * body.getCMassLocalPose()).p;
        const PxVec3 torque = (contact.position - centerOfMass).cross(force);
        m_SupportLoads.forces[side] = force;
        const PxU32 firstRow = 6 * link->getLinkIndex();
        for (unsigned column = 0; column < count; ++column)
        {
            for (unsigned axis = 0; axis < 3; ++axis)
            {
                generalized[column] += cache.denseJacobian[(firstRow + axis) * count + column] * force[axis] +
                    cache.denseJacobian[(firstRow + 3 + axis) * count + column] * torque[axis];
            }
        }
    }
    // 이 힘을 바디에 가하지 않는다. 예상 지면반력 J^T F를 역동역학에서 빼 관절 토크만 계산한다.
    return generalized;
#else
    return {};
#endif
}
