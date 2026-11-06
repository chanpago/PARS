#pragma once

#include <foundation/PxVec3.h>
#include <cstdint>

namespace PARS
{
    // 실제 접촉 샘플과 별도로 마지막 지지면을 짧게 유지한다. 충격량이나 착지는 만들지 않는다.
    class FootSupportContact
    {
    public:
        void Update(uint64_t completedStep, bool hasSample, bool grounded,
            const physx::PxVec3& position, const physx::PxVec3& normal)
        {
            if (!hasSample)
            {
                Reset();
                return;
            }
            if (grounded)
            {
                physx::PxVec3 unitNormal = normal;
                if (!position.isFinite() || !unitNormal.isFinite() ||
                    unitNormal.normalize() < 1e-6f || unitNormal.y < .70710678f)
                {
                    Reset();
                    return;
                }
                m_LastContactStep = completedStep;
                m_Position = position;
                m_Normal = unitNormal;
                m_Valid = true;
                return;
            }
            // 렌더 횟수나 조회 횟수 대신 완료 물리 스텝의 차이로 만료시킨다.
            if (!m_Valid || completedStep < m_LastContactStep ||
                completedStep - m_LastContactStep > GraceSteps)
            {
                Reset();
            }
        }

        void Reset()
        {
            m_LastContactStep = 0;
            m_Position = physx::PxVec3(0);
            m_Normal = physx::PxVec3(0);
            m_Valid = false;
        }

    public:
        bool IsValid() const
        {
            return m_Valid;
        }

        const physx::PxVec3& GetPosition() const
        {
            return m_Position;
        }

        const physx::PxVec3& GetNormal() const
        {
            return m_Normal;
        }

        uint64_t GetLastContactStep() const
        {
            return m_LastContactStep;
        }

    public:
        static constexpr uint64_t GraceSteps = 3;

    private:
        uint64_t m_LastContactStep = 0;
        physx::PxVec3 m_Position{0};
        physx::PxVec3 m_Normal{0};
        bool m_Valid = false;
    };
}
