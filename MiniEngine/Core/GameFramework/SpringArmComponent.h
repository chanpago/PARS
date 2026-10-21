#pragma once

#include "SceneComponent.h"

namespace PARS
{
    // 부착 위치에서 카메라를 뒤로 띄우고, 충돌 시 암을 줄인다. 카메라/GPU 리소스는 소유하지 않는다.
    class SpringArmComponent : public SceneComponent
    {
    public:
        using SceneComponent::SceneComponent;
        struct Settings
        {
            float targetArmLength = 4.5f;
            float probeRadius = .2f, collisionPadding = .02f;
            float cameraLagSpeed = 12, maxCameraLagDistance = .5f;
            float armReturnSpeed = 10;
            bool collisionTest = true;
        };

        bool SetSettings(const Settings& settings);
        const Settings& GetSettings() const
        {
            return m_Settings;
        }
        // 월드 기준 시점(라디안). 캐릭터가 이동 방향으로 회전해도 조작 중인 시점은 유지한다.
        bool SetControlRotation(float yaw, float pitch);
        void ResetLagState();
        // 기본 Tick 또는 호출자가 물리 갱신 후 한 번 실행한다. 수동 호출 시 Tick은 꺼둔다.
        void UpdateArm(float deltaTime);
        Math::Vector3 GetCameraLocation() const
        {
            return m_CameraLocation;
        }
        Math::Vector3 GetViewDirection() const
        {
            return m_ViewDirection;
        }
        float GetCurrentArmLength() const
        {
            return m_CurrentArmLength;
        }
        bool IsCollisionFixApplied() const
        {
            return m_CollisionFixed;
        }

    protected:
        void TickComponent(float deltaTime) override
        {
            UpdateArm(deltaTime);
        }
        void EndPlay() override
        {
            ResetLagState();
        }

    private:
        Settings m_Settings;
        Math::Vector3 m_CameraLocation{Math::kZero}, m_LaggedPivot{Math::kZero}, m_PreviousPivot{Math::kZero};
        Math::Vector3 m_ViewDirection{Math::kZUnitVector};
        float m_CurrentArmLength = 0;
        bool m_Initialized = false, m_CollisionFixed = false;
    };
}
