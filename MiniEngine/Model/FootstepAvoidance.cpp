#include "FootstepAvoidance.h"
#include "FootstepCollisionBounds.h"
#include "FramePerformance.h"
#include <algorithm>
#include <cmath>
#include <limits>

using namespace PARS;
using namespace physx;

namespace
{
    constexpr uint32_t MaximumCandidateChecks = 6;
    constexpr uint32_t MaximumIntervalsPerTick = 384;

    bool Accept(const FootstepCollisionSnapshot& result)
    {
        return result.status == FootstepCollisionStatus::Clear && !result.reachLimited && !result.jointLimited;
    }

    float Angle(const PxQuat& first, const PxQuat& second)
    {
        return 2 * std::acos(PxClamp(PxAbs(first.dot(second)), 0.0f, 1.0f));
    }

    struct Candidate
    {
    public:
        size_t side = 0;
        FootstepPath path;
        float score = 0;
        int direction = 0;
    };

    float Score(const FootstepCollisionLeg& original, const FootstepPath& path, float remaining, float offset)
    {
        FootstepCollisionLeg candidate = original;
        candidate.path = path;
        float score = 0;
        PxVec3 previousPosition = SampleFootstepPosition(candidate.command, candidate.path, 0);
        std::array<PxTransform, 3> shapes;
        LegIKResult previous;
        if (!PredictFootstepLeg(candidate, 0, shapes, previous))
        {
            return std::numeric_limits<float>::infinity();
        }
        PxVec3 previousDelta(0);
        for (uint32_t index = 1; index <= 6; ++index)
        {
            const float time = remaining * static_cast<float>(index) / 6;
            LegIKResult solved;
            if (!PredictFootstepLeg(candidate, time, shapes, solved) || solved.reachLimited)
            {
                return std::numeric_limits<float>::infinity();
            }
            const PxVec3 position = SampleFootstepPosition(candidate.command, candidate.path, time);
            const PxVec3 delta(Angle(previous.hip.q, solved.hip.q),
                Angle(previous.knee.q, solved.knee.q), Angle(previous.ankle.q, solved.ankle.q));
            // 관절 회전량과 회전 증분의 급변, 경로 길이로 추종 난이도를 근사한다. 토크 최적화는 아니다.
            score += delta.magnitudeSquared() + .5f * (delta - previousDelta).magnitudeSquared();
            score += 2 * (position - previousPosition).magnitude();
            previous = solved;
            previousDelta = delta;
            previousPosition = position;
        }
        return score + 4 * offset;
    }
}

void FootstepAvoidance::Reset()
{
    m_Snapshot = {};
}

void FootstepAvoidance::Update(FootstepCollisionInput input, const PxVec3& characterForward)
{
    if (input.revision != m_Snapshot.revision)
    {
        Reset();
    }
    m_Snapshot.revision = input.revision;
    m_Snapshot.candidatesChecked = 0;
    m_Snapshot.totalIntervals = 0;
    m_Snapshot.totalQueries = 0;
    m_Snapshot.searchBudgetExhausted = false;
    for (size_t side = 0; side < 2; ++side)
    {
        const FootstepCommand& command = input.legs[side].command;
        const FootstepPath& path = m_Snapshot.paths[side];
        if (!command.IsInProgress() || command.startedStep != m_Snapshot.startedSteps[side] ||
            (command.motion.hasSupportDeadline && path.enabled && path.startTime + path.duration > command.motion.duration + 1e-6f))
        {
            m_Snapshot.paths[side] = {};
            m_Snapshot.directions[side] = 0;
        }
        m_Snapshot.startedSteps[side] = command.startedStep;
        input.legs[side].path = m_Snapshot.paths[side];
    }
    m_Snapshot.collision = PredictFootstepCollisions(input);
    m_Snapshot.totalIntervals = m_Snapshot.collision.intervalCount;
    m_Snapshot.totalQueries = m_Snapshot.collision.queryCount;
    if (m_Snapshot.collision.status == FootstepCollisionStatus::Inactive)
    {
        m_Snapshot.status = FootstepAvoidanceStatus::Inactive;
        return;
    }
    const bool hasDetour = m_Snapshot.paths[0].enabled || m_Snapshot.paths[1].enabled;
    if (m_Snapshot.collision.status == FootstepCollisionStatus::Clear &&
        (!hasDetour || Accept(m_Snapshot.collision)))
    {
        // 이미 통과한 경로는 유지하여 틱마다 앞·뒤 우회가 뒤집히는 것을 막는다.
        m_Snapshot.status = hasDetour ?
            FootstepAvoidanceStatus::Detouring : FootstepAvoidanceStatus::Direct;
        return;
    }
    m_Snapshot.status = FootstepAvoidanceStatus::Failed;
    if (m_Snapshot.collision.status == FootstepCollisionStatus::Unavailable || m_Snapshot.collision.fixedPoseBlocked)
    {
        return;
    }
    PxVec3 forward(characterForward.x, 0, characterForward.z);
    if (!forward.isFinite() || forward.normalize() <= 1e-6f)
    {
        return;
    }
    FramePerformance::Scope rankingTiming(FramePerformance::Category::DetourRanking);
    std::array<Candidate, 12> candidates;
    uint32_t count = 0;
    for (size_t side = 0; side < 2; ++side)
    {
        const FootstepCollisionLeg& leg = input.legs[side];
        if (!leg.command.IsInProgress())
        {
            continue;
        }
        const float flight = leg.command.motion.elapsedTime;
        const float remaining = leg.command.motion.duration - flight;
        if (remaining <= input.fixedDeltaTime)
        {
            continue;
        }
        const float fraction = remaining / leg.command.motion.duration;
        const PxVec3 offset = SampleFootstepPath(leg.path, flight);
        const PxVec3 velocity = SampleFootstepPathVelocity(leg.path, flight);
        for (uint32_t variant = 0; variant < 6; ++variant)
        {
            const int direction = variant % 2 == 0 ? 1 : -1;
            const float distance = variant < 2 ? .035f : .07f;
            const float lift = variant >= 4 ? .035f : 0;
            const PxVec3 detour = (forward * (distance * direction) + PxVec3(0, lift, 0)) * (fraction * fraction);
            Candidate candidate;
            candidate.side = side;
            candidate.direction = direction;
            candidate.path = MakeFootstepPath(offset, velocity, detour, flight, remaining);
            if (!candidate.path.enabled || !IsFootstepPathWithinLimits(candidate.path, 9.0f, 600.0f))
            {
                continue;
            }
            candidate.score = Score(leg, candidate.path, GetFootstepRemainingTime(leg.command), detour.magnitude());
            if (!std::isfinite(candidate.score))
            {
                continue;
            }
            if (m_Snapshot.directions[side] != 0 && m_Snapshot.directions[side] != direction)
            {
                candidate.score += .25f;
            }
            candidates[count++] = candidate;
        }
    }
    // 후보 수가 작으므로 힙 할당 없는 안정 정렬을 사용한다.
    for (uint32_t index = 1; index < count; ++index)
    {
        const Candidate candidate = candidates[index];
        uint32_t position = index;
        while (position > 0 && candidate.score < candidates[position - 1].score)
        {
            candidates[position] = candidates[position - 1];
            --position;
        }
        candidates[position] = candidate;
    }
    for (uint32_t index = 0; index < count && m_Snapshot.candidatesChecked < MaximumCandidateChecks; ++index)
    {
        if (m_Snapshot.totalIntervals >= MaximumIntervalsPerTick)
        {
            m_Snapshot.searchBudgetExhausted = true;
            break;
        }
        FootstepCollisionInput candidateInput = input;
        candidateInput.rejectInvalidPoses = true;
        const Candidate& candidate = candidates[index];
        candidateInput.legs[candidate.side].path = candidate.path;
        candidateInput.maximumIntervals = (std::min)(input.maximumIntervals,
            MaximumIntervalsPerTick - m_Snapshot.totalIntervals);
        const FootstepCollisionSnapshot tested =
            PredictFootstepCollisions(candidateInput, FootstepCollisionPass::DetourCandidate);
        ++m_Snapshot.candidatesChecked;
        m_Snapshot.totalIntervals += tested.intervalCount;
        m_Snapshot.totalQueries += tested.queryCount;
        if (Accept(tested))
        {
            m_Snapshot.paths[candidate.side] = candidate.path;
            m_Snapshot.directions[candidate.side] = candidate.direction;
            m_Snapshot.collision = tested;
            m_Snapshot.status = FootstepAvoidanceStatus::Detouring;
            return;
        }
    }
    // 실패 시 기존 연속 경로를 유지하고 실패를 공개한다. 착지점 재선정은 별도 정책으로 남긴다.
}
