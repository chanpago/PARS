#include "SpringArmComponent.h"
#include "Actor.h"
#include "World.h"
#include <algorithm>
#include <cmath>
#if defined(_M_X64)
#include "../PhysicsWorld.h"
#endif

using namespace Math;

namespace PARS
{
    bool SpringArmComponent::SetSettings(const Settings& settings)
    {
        const float values[] = {settings.targetArmLength, settings.probeRadius, settings.collisionPadding,
            settings.cameraLagSpeed, settings.maxCameraLagDistance, settings.armReturnSpeed};
        for (float value : values)
        {
            if (!std::isfinite(value) || value < 0 || value > 1000)
            {
                return false;
            }
        }
        if (settings.probeRadius <= 0)
        {
            return false;
        }
        m_Settings = settings;
        return true;
    }

    bool SpringArmComponent::SetControlRotation(float yaw, float pitch)
    {
        if (!std::isfinite(yaw) || !std::isfinite(pitch))
        {
            return false;
        }
        yaw = std::remainder(yaw, DirectX::XM_2PI);
        constexpr float maxPitch = DirectX::XM_PIDIV2 - .01f;
        pitch = std::max(-maxPitch, std::min(maxPitch, pitch));
        m_ViewDirection = Vector3(std::sin(yaw) * std::cos(pitch), -std::sin(pitch), std::cos(yaw) * std::cos(pitch));
        return true;
    }

    void SpringArmComponent::ResetLagState()
    {
        m_Initialized = false;
        m_CollisionFixed = false;
    }

    void SpringArmComponent::UpdateArm(float deltaTime)
    {
        const auto pivot = GetComponentTransform().GetTranslation();
        if (!std::isfinite(deltaTime) || deltaTime < 0 || !std::isfinite(float(pivot.GetX())) ||
            !std::isfinite(float(pivot.GetY())) || !std::isfinite(float(pivot.GetZ())))
        {
            return;
        }
        // 큰 순간이동은 이전 위치에서 카메라를 끌고 오지 않는다.
        const bool reset = !m_Initialized || float(Length(pivot - m_PreviousPivot)) > std::max(1.0f, 2 * m_Settings.targetArmLength);
        const auto blend = [deltaTime](float speed) {
            return speed > 0 ? -std::expm1(-speed * deltaTime) : 1.0f;
        };
        m_LaggedPivot = reset ? pivot : Lerp(m_LaggedPivot, pivot, blend(m_Settings.cameraLagSpeed));
        const auto lag = m_LaggedPivot - pivot;
        const float lagDistance = Length(lag);
        if (lagDistance > m_Settings.maxCameraLagDistance)
        {
            m_LaggedPivot = pivot + lag * (m_Settings.maxCameraLagDistance / lagDistance);
        }
        const auto desiredOffset = m_LaggedPivot - m_ViewDirection * m_Settings.targetArmLength - pivot;
        const float desiredLength = Length(desiredOffset);
        if (!std::isfinite(desiredLength))
        {
            ResetLagState();
            return;
        }
        float allowed = desiredLength;
#if defined(_M_X64)
        const auto* world = GetOwner().GetWorld();
        const auto* physics = world ? world->GetPhysicsWorld() : nullptr;
        if (m_Settings.collisionTest && physics && desiredLength > 1e-6f)
        {
            // 지연된 목표가 벽 반대편에 있어도 현재 피벗부터 검사해 벽을 뚫지 않는다.
            allowed = physics->SweepSphereDistance(
                physx::PxVec3(float(pivot.GetX()), float(pivot.GetY()), float(pivot.GetZ())),
                physx::PxVec3(float(desiredOffset.GetX()), float(desiredOffset.GetY()), float(desiredOffset.GetZ())),
                m_Settings.probeRadius, &GetOwner());
            if (allowed < desiredLength)
            {
                allowed = std::max(0.0f, allowed - m_Settings.collisionPadding);
            }
        }
#endif
        m_CollisionFixed = allowed < desiredLength - 1e-5f;
        // 벽 쪽으로는 즉시 줄이고, 다시 늘어날 때만 보간한다. 충돌 처리 뒤 위치를 보간하지 않는다.
        m_CurrentArmLength = reset || allowed < m_CurrentArmLength ? allowed
                                                                   : m_CurrentArmLength + (allowed - m_CurrentArmLength) * blend(m_Settings.armReturnSpeed);
        m_CameraLocation = desiredLength > 1e-6f ? pivot + desiredOffset * (m_CurrentArmLength / desiredLength) : pivot;
        m_PreviousPivot = pivot;
        m_Initialized = true;
    }
}
