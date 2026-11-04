#pragma once

#include <array>
#include <cassert>
#include <chrono>
#include <cstddef>
#include <cstdint>

namespace FramePerformance
{
    enum class Category : size_t
    {
        PhysXStep,
        ParsControl,
        ParsPose,
        ParsOther,
        RenderPreparation,
        ParsTotal,
        Count
    };

    constexpr size_t CategoryCount = static_cast<size_t>(Category::Count);
    constexpr size_t HistoryLength = 120;

    struct Sample
    {
    public:
        std::array<double, CategoryCount> milliseconds{};
        std::array<uint32_t, CategoryCount> calls{};
    };

    struct Statistics
    {
    public:
        double last = 0.0;
        double average = 0.0;
        double maximum = 0.0;
        uint32_t calls = 0;
    };

    class Recorder
    {
    public:
        void BeginFrame()
        {
            m_Current = Sample{};
        }
        void AddSample(Category category, double milliseconds)
        {
            const size_t index = static_cast<size_t>(category);
            m_Current.milliseconds[index] += milliseconds;
            ++m_Current.calls[index];
        }
        void EndFrame()
        {
            const size_t total = static_cast<size_t>(Category::ParsTotal);
            m_Current.milliseconds[total] =
                m_Current.milliseconds[static_cast<size_t>(Category::ParsControl)] +
                m_Current.milliseconds[static_cast<size_t>(Category::ParsPose)] +
                m_Current.milliseconds[static_cast<size_t>(Category::ParsOther)];
            m_History[m_Next] = m_Current;
            m_Next = (m_Next + 1) % HistoryLength;
            if (m_Count < HistoryLength)
            {
                ++m_Count;
            }
        }

    public:
        Statistics GetStatistics(Category category) const
        {
            Statistics result;
            if (m_Count == 0)
            {
                return result;
            }
            const size_t index = static_cast<size_t>(category);
            const Sample& last = m_History[(m_Next + HistoryLength - 1) % HistoryLength];
            result.last = last.milliseconds[index];
            result.calls = last.calls[index];
            for (size_t frame = 0; frame < m_Count; ++frame)
            {
                const double value = m_History[frame].milliseconds[index];
                result.average += value;
                if (value > result.maximum)
                {
                    result.maximum = value;
                }
            }
            // 물리가 실행되지 않은 프레임의 0도 평균에 포함한다.
            result.average /= static_cast<double>(m_Count);
            return result;
        }
        size_t GetFrameCount() const
        {
            return m_Count;
        }

    private:
        Sample m_Current;
        std::array<Sample, HistoryLength> m_History{};
        size_t m_Next = 0;
        size_t m_Count = 0;
    };

    class Scope;

    struct State
    {
    public:
        Recorder recorder;
        Scope* currentScope = nullptr;
        bool enabled = false;
    };

    inline State& GetState()
    {
        // 주 스레드의 경과 시간을 측정하며 PhysX 워커의 CPU 사용 시간을 합산하지 않는다.
        // 다른 스레드에서 호출한 범위는 해당 스레드에서 프레임을 시작하지 않으면 기록하지 않는다.
        static thread_local State state;
        return state;
    }

    inline void BeginFrame()
    {
        State& state = GetState();
        assert(state.currentScope == nullptr);
        state.recorder.BeginFrame();
        state.enabled = true;
    }

    inline void EndFrame()
    {
        State& state = GetState();
        assert(state.currentScope == nullptr);
        state.enabled = false;
        state.recorder.EndFrame();
    }

    class Scope
    {
    public:
        explicit Scope(Category category)
            : m_Category(category), m_Enabled(GetState().enabled)
        {
            if (m_Enabled)
            {
                m_Parent = GetState().currentScope;
                GetState().currentScope = this;
                m_Start = std::chrono::steady_clock::now();
            }
        }
        ~Scope()
        {
            if (!m_Enabled)
            {
                return;
            }
            const std::chrono::steady_clock::duration elapsed = std::chrono::steady_clock::now() - m_Start;
            const double milliseconds = std::chrono::duration<double, std::milli>(elapsed).count();
            // 중첩 범위의 시간을 부모에서 빼서 PARS와 PhysX를 중복 집계하지 않는다.
            GetState().recorder.AddSample(m_Category, milliseconds - m_ChildMilliseconds);
            GetState().currentScope = m_Parent;
            if (m_Parent)
            {
                m_Parent->m_ChildMilliseconds += milliseconds;
            }
        }
        Scope(const Scope&) = delete;

    public:
        Scope& operator=(const Scope&) = delete;

    private:
        Category m_Category;
        Scope* m_Parent = nullptr;
        std::chrono::steady_clock::time_point m_Start;
        double m_ChildMilliseconds = 0.0;
        bool m_Enabled;
    };
}
