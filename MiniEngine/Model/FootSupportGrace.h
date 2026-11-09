#pragma once

#include "FootSupportHull.h"
#include <cstdint>
#include <algorithm>
#include <cmath>

namespace PARS
{
    // 같은 물리 샘플을 여러 번 읽어도 B 이탈 시간을 중복 누적하지 않는다.
    class FootSupportGrace
    {
    public:
        void Reset()
        {
            m_OutsideTime = 0;
            m_Revision = 0;
            m_Step = 0;
            m_LastSupportWeight = 1;
            m_HasSample = false;
            m_Outside = false;
        }

        void Update(bool enabled, uint64_t revision, uint64_t step, double deltaTime,
            const FootSupportBalance& balance)
        {
            if (!enabled || (m_HasSample && revision != m_Revision))
            {
                Reset();
            }
            if (!enabled)
            {
                return;
            }
            const bool repeated = m_HasSample && step == m_Step;
            const bool consecutive = m_HasSample && step > m_Step && step - m_Step == 1;
            m_Revision = revision;
            m_Step = step;
            m_HasSample = true;
            if (balance.zone != FootSupportZone::Outside || !std::isfinite(deltaTime) || deltaTime <= 0)
            {
                m_OutsideTime = 0;
                m_Outside = false;
                if ((balance.zone == FootSupportZone::Inside || balance.zone == FootSupportZone::Buffer) &&
                    std::isfinite(balance.weight) && balance.weight > 0 && balance.weight <= 1)
                {
                    m_LastSupportWeight = balance.weight;
                }
                return;
            }
            // 최초 이탈 관측 시점은 0초다. 연속된 완료 물리 스텝 사이의 시간만 센다.
            // 같은 틱의 발바닥 설정 변경도 반영하되, 이미 센 물리 시간은 다시 더하지 않는다.
            if (!m_Outside || (!repeated && !consecutive))
            {
                m_OutsideTime = 0;
            }
            else if (consecutive)
            {
                m_OutsideTime = (std::min)(m_OutsideTime + deltaTime, Duration);
            }
            m_Outside = true;
        }

    public:
        double GetOutsideTime() const
        {
            return m_OutsideTime;
        }

        bool HasExpired() const
        {
            return m_Outside && m_OutsideTime + 1e-9 >= Duration;
        }

        bool IsActive() const
        {
            return m_Outside && !HasExpired();
        }

        float GetSupportWeight(const FootSupportBalance& balance) const
        {
            if (balance.zone == FootSupportZone::Outside)
            {
                // 유예 중에는 이탈 직전의 강도를 유지하며, 지지 강도를 새로 높이지 않는다.
                return IsActive() ? m_LastSupportWeight : 0;
            }
            return balance.weight;
        }

    public:
        static constexpr double Duration = 0.2;

    private:
        double m_OutsideTime = 0;
        uint64_t m_Revision = 0;
        uint64_t m_Step = 0;
        float m_LastSupportWeight = 1;
        bool m_HasSample = false;
        bool m_Outside = false;
    };
}
