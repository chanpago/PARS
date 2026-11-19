#pragma once

#include <DirectXMath.h>
#include <array>
#include <cstdint>

namespace PARS
{
    // 발 플래너가 교차딛기를 선택했을 때만 CrossStep 후보를 검색한다.
    enum class MotionFrameUsage : std::uint8_t
    {
        General,
        CrossStep
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
        MotionFrameUsage usage = MotionFrameUsage::General;
    };
}
