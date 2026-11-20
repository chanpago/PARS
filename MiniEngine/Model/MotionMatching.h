#pragma once

#include "FootstepMode.h"
#include <DirectXMath.h>
#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>

namespace PARS
{
    enum class MotionFrameUsage : std::uint8_t
    {
        Idle,
        Walking,
        Count
    };

    // 전체 후보 배열의 [firstFrameIndex, firstFrameIndex + frameCount) 구간이다.
    struct MotionFrameRange
    {
    public:
        std::size_t firstFrameIndex = 0;
        std::size_t frameCount = 0;
    };

    // 한 보행 주기 안의 이동발(왼발 0, 오른발 1)과 그 발이 착지하는 방식이다.
    struct WalkingSwing
    {
    public:
        std::uint32_t movingFoot = UINT32_MAX;
        FootstepMode mode = FootstepMode::Normal;
    };

    struct MotionClipSelection
    {
    public:
        std::uint32_t clipIndex = UINT32_MAX;
        MotionFrameUsage usage = MotionFrameUsage::Idle;
    };

    // 클립 후보와 현재 캐릭터가 공유하는 특징이다. 캐릭터가 바라보는 수평 방향을 로컬 전방으로 쓴다.
    struct MotionFeatures
    {
    public:
        static constexpr float PredictionTimeSeconds = 0.2f;

    public:
        // 골반 기준 발 위치(m). 두 배열의 인덱스는 왼발 0, 오른발 1이다.
        std::array<DirectX::XMFLOAT3, 2> footPositions{};
        // 골반 기준 발 위치의 변화율(m/s).
        std::array<DirectX::XMFLOAT3, 2> footVelocities{};
        // 예측 시간 동안의 골반 변위(m)를 로컬 X(좌우), Y(상하), Z(전후) 순서로 저장한다.
        // 클립 후보에는 실제 변위를, 검색 입력에는 원하는 변위를 담는다.
        DirectX::XMFLOAT3 futureDisplacement{};
    };

    // AnimInstance에 등록한 클립의 특정 재생 시점을 검색 후보로 저장한다.
    struct MotionFrame
    {
    public:
        MotionFeatures features{};
        std::uint32_t clipIndex = UINT32_MAX;
        float clipTimeSeconds = 0.0f;
        WalkingSwing swing;
        MotionFrameUsage usage = MotionFrameUsage::Idle;
    };

    struct MotionMatchResult
    {
    public:
        std::uint32_t clipIndex = UINT32_MAX;
        float clipTimeSeconds = 0.0f;
        float score = std::numeric_limits<float>::infinity();
    };
}
