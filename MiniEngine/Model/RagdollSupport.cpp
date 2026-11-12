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
    std::array<std::array<RagdollSupportPart, 2>, 2> parts;
    std::array<PxVec3, 2> positions{PxVec3(0), PxVec3(0)};
    unsigned supports = 0;
    for (size_t side = 0; side < m_SupportContacts.size(); ++side)
    {
        const RagdollSupportContact& contact = m_SupportContacts[side];
        if (!contact.supporting || contact.body < 0 || static_cast<size_t>(contact.body) >= m_Bodies.size())
        {
            continue;
        }
        parts[side] = contact.parts;
        if (parts[side][0].body < 0 && parts[side][1].body < 0)
        {
            // 부위 정보가 없는 기존 호출은 한 개의 발 바디가 지지하도록 유지한다.
            parts[side][0].body = contact.body;
            parts[side][0].position = contact.position;
            parts[side][0].normal = contact.normal;
            parts[side][0].fraction = 1;
        }
        float totalFraction = 0;
        for (RagdollSupportPart& part : parts[side])
        {
            if (part.body < 0 || static_cast<size_t>(part.body) >= m_Bodies.size() ||
                !part.position.isFinite() || !part.normal.isFinite() || part.normal.normalize() < 1e-6f ||
                part.normal.y < .70710678f || !std::isfinite(part.fraction) || part.fraction <= 0)
            {
                part.fraction = 0;
                continue;
            }
            totalFraction += part.fraction;
            positions[side] += part.position * part.fraction;
        }
        if (totalFraction <= 0)
        {
            continue;
        }
        valid[side] = true;
        ++supports;
        positions[side] /= totalFraction;
        for (RagdollSupportPart& part : parts[side])
        {
            part.fraction /= totalFraction;
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
        // 좌우 분담은 COM으로 정하고, 한 발 안에서는 실제 접촉 하중 비율을 사용한다.
        const PxVec3 span = positions[1] - positions[0];
        const PxVec3 relative = PxVec3(float(center.GetX()), float(center.GetY()), float(center.GetZ())) - positions[0];
        const float lengthSquared = span.x * span.x + span.z * span.z;
        const float right = lengthSquared > 1e-8f ? PxClamp((relative.x * span.x + relative.z * span.z) / lengthSquared, 0.0f, 1.0f) : .5f;
        m_SupportLoads.fractions = {1.0f - right, right};
    }
    else
    {
        m_SupportLoads.fractions = {valid[0] ? 1.0f : 0.0f, valid[1] ? 1.0f : 0.0f};
    }
    PxArticulationCache& cache = *m_ArticulationCache;
    PxU32 rows = 0;
    PxU32 columns = 0;
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
        if (weight > 0 && std::isfinite(contact.ankleDriveBoost))
        {
            m_SupportLoads.ankleDriveScales[side] +=
                PxClamp(contact.ankleDriveBoost, 0.0f, 1.0f) * m_SupportLoads.fractions[side];
        }
        for (size_t index = 0; index < parts[side].size(); ++index)
        {
            const RagdollSupportPart& part = parts[side][index];
            if (part.fraction <= 0)
            {
                continue;
            }
            const PxRigidBody& body = *m_Bodies[part.body];
            const PxArticulationLink* link = body.is<PxArticulationLink>();
            if (!link || 6 * link->getLinkIndex() + 5 >= rows)
            {
                m_SupportLoads = {};
                return std::vector<float>(count, 0);
            }
            const PxVec3 force = part.normal * (weight * m_SupportLoads.fractions[side] * part.fraction / part.normal.y);
            const PxVec3 centerOfMass = (body.getGlobalPose() * body.getCMassLocalPose()).p;
            const PxVec3 torque = (part.position - centerOfMass).cross(force);
            m_SupportLoads.forces[side] += force;
            m_SupportLoads.partForces[side][index] = force;
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
    }
    // 접촉 반력을 일반화 힘으로 돌려준다. 외력을 추가하지 않고 역동역학 보상에서 차감한다.
    return generalized;
#else
    return {};
#endif
}
