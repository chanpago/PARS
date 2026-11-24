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
        BalanceUpperBody,
        ShoveFall,
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

    // 월드 XZ 평면에서 캐릭터 전방을 0도로 삼는다. 진행 방향을 구할 수 없으면 NaN을 반환한다.
    float GetWalkingTravelAngleDegrees(const DirectX::XMFLOAT2& forwardXZ, const DirectX::XMFLOAT2& velocityXZ);

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
        // 충격 반응은 골반 기준 양손과 머리의 위치로 현재 상체와 가까운 자세를 찾는다.
        std::array<DirectX::XMFLOAT3, 3> upperBodyPositions{};
        // 예측 시간 동안의 골반 변위(m)를 로컬 X(좌우), Y(상하), Z(전후) 순서로 저장한다.
        // 클립 후보에는 실제 변위를, 검색 입력에는 원하는 변위를 담는다.
        DirectX::XMFLOAT3 futureDisplacement{};
        // 검색 입력은 전신 COM 속도, 보행 클립은 루프 한 주기의 평균 진행 속도로 계산한다.
        float walkingAngleDegrees = std::numeric_limits<float>::quiet_NaN();
    };

    // AnimInstance에 등록한 클립의 특정 재생 시점을 검색 후보로 저장한다.
    struct MotionFrame
    {
    public:
        MotionFeatures features{};
        std::uint32_t clipIndex = UINT32_MAX;
        float clipTimeSeconds = 0.0f;
        WalkingSwing swing;
        float remainingSwingTime = 0.0f;
        MotionFrameUsage usage = MotionFrameUsage::Idle;
    };

    struct MotionMatchResult
    {
    public:
        std::uint32_t clipIndex = UINT32_MAX;
        float clipTimeSeconds = 0.0f;
        float score = std::numeric_limits<float>::infinity();
        float requestedAngleDegrees = std::numeric_limits<float>::quiet_NaN();
        float clipAngleDegrees = std::numeric_limits<float>::quiet_NaN();
        float remainingSwingTime = 0.0f;
    };
}
