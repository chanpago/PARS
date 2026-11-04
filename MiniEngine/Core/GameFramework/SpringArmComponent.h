#pragma once

#include "SceneComponent.h"

namespace PARS
{
    // 부착 위치에서 카메라를 뒤로 띄우고, 충돌 시 암을 줄인다. 카메라/GPU 리소스는 소유하지 않는다.
    class SpringArmComponent : public SceneComponent
    {
    public:
        struct Settings
        {
        public:
            float targetArmLength = 4.5f;
            float probeRadius = .2f;
            float collisionPadding = .02f;
            float cameraLagSpeed = 12;
            float maxCameraLagDistance = .5f;
            float armReturnSpeed = 10;
            bool collisionTest = true;
        };

    public:
        using SceneComponent::SceneComponent;

    public:
        void ResetLagState();
        // 기본 Tick 또는 호출자가 물리 갱신 후 한 번 실행한다. 수동 호출 시 Tick은 꺼둔다.
        void UpdateArm(float deltaTime);

    protected:
        void TickComponent(float deltaTime) override
        {
            UpdateArm(deltaTime);
        }
        void EndPlay() override
        {
            ResetLagState();
        }

    public:
        bool SetSettings(const Settings& settings);
        const Settings& GetSettings() const
        {
            return m_Settings;
        }
        // 월드 기준 시점(라디안). 캐릭터가 이동 방향으로 회전해도 조작 중인 시점은 유지한다.
        bool SetControlRotation(float yaw, float pitch);
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

    private:
        Math::Vector3 m_CameraLocation{Math::kZero};
        Math::Vector3 m_LaggedPivot{Math::kZero};
        Math::Vector3 m_PreviousPivot{Math::kZero};
        Math::Vector3 m_ViewDirection{Math::kZUnitVector};
        Settings m_Settings;
        float m_CurrentArmLength = 0;
        bool m_Initialized = false;
        bool m_CollisionFixed = false;
    };
}
