#pragma once

#include "PhysicsAsset.h"
#include "FootSupportHull.h"
#include "FootstepPlanner.h"
#include <array>
#include <cstdint>

class Model;
class PhysicsWorld;

namespace PARS
{
    class SkeletalMeshComponent;
    class Ragdoll;

    struct FootSoleSettings
    {
        float halfWidth = .05f, halfLength = .1f;
        DirectX::XMFLOAT3 offset{0, 0, 0}, rotation{0, 0, 0};
    };

    struct FootLegSetup
    {
        bool available = false;
        int hipNode = -1, kneeNode = -1, ankleNode = -1;
        // 기준 자세의 실제 구간 길이(m). 현재 자세의 수직 투영 길이가 아니다.
        float thighLength = 0, calfLength = 0;
        physx::PxVec3 referenceBendInHip{0};
    };

    struct FootContactSample
    {
        bool hasSample = false, grounded = false;
        // 완료 스텝에 속하는 전이이므로 같은 스텝을 다시 읽어도 중복 집계하지 않는다.
        bool leftGround = false, landed = false;
        physx::PxVec3 position{0}, normal{0};
        uint32_t pointCount = 0;
        uint64_t takeoffCount = 0, landingCount = 0;
        uint64_t lastTakeoffStep = 0, lastLandingStep = 0;
    };

    struct FootSoleSample
    {
        bool available = false;
        int boneNode = -1;
        FootLegSetup leg;
        FootContactSample contact;
        // 발바닥 목표를 PA 바디 및 발목 본의 목표로 되돌릴 때 사용하는 변환이다.
        Math::Matrix4 soleInBody{Math::kIdentity}, soleInBone{Math::kIdentity}, soleWorld{Math::kIdentity};
        std::array<physx::PxVec3, 4> corners{}, projections{};
        std::array<bool, 4> projected{};
        std::array<float, 4> distances{};
    };

    struct FootSupportSnapshot
    {
        std::array<FootSoleSample, 2> feet;
        std::vector<physx::PxVec3> hull;
        FootSupportBalance balance;
        FootSupportCandidates candidates;
        FootstepSnapshot footstep;
        physx::PxVec3 centerOfMass{0}, projectedCenterOfMass{0};
        physx::PxVec3 centerOfMassVelocity{0};
        float totalMass = 0;
        size_t projectedPointCount = 0;
        bool hasCenterOfMass = false, hasComProjection = false, comInsideHull = false;
        bool hasCenterOfMassVelocity = false;
        uint64_t sampleNumber = 0, completedPhysicsStep = 0;
        uint64_t contactRevision = 0;
        std::string message;
    };

    // Samples actual bodies before simulate(), independently of rendering/debug visibility.
    // Measures geometry only; SkeletalMeshComponent applies the optional support controller.
    class FootSupport
    {
    public:
        static constexpr float RayLength = .2f;
        void Update(SkeletalMeshComponent& mesh, const PhysicsWorld& physics);
        void Reset();
        // 명시적으로 자동 지지를 재설정할 때 기존 해제 래치와 발 판단을 함께 초기화한다.
        void ResetFootstep();
        void ResetSoles()
        {
            m_Initialized = false;
        }
        const FootSupportSnapshot& GetSnapshot() const
        {
            return m_Snapshot;
        }
        FootSoleSettings& GetSoleSettings(size_t side)
        {
            return m_Settings.at(side);
        }
        float GetBufferScale() const
        {
            return m_BufferScale;
        }
        bool SetBufferScale(float scale);
        FootstepSettings& GetFootstepSettings()
        {
            return m_FootstepSettings;
        }

    private:
        bool Prepare(SkeletalMeshComponent& mesh);
        void UpdateContacts(const Ragdoll& ragdoll, const PhysicsWorld& physics);
        void UpdateFootstep(SkeletalMeshComponent& mesh);
        struct Sole
        {
            int bodyIndex = -1;
            PhysicsBody source;
            Math::Matrix4 soleInBody{Math::kIdentity};
            FootLegSetup leg;
        };
        std::array<Sole, 2> m_Feet;
        std::array<FootSoleSettings, 2> m_Settings;
        FootSupportSnapshot m_Snapshot;
        FootstepPlanner m_FootstepPlanner;
        FootstepSettings m_FootstepSettings;
        std::array<Math::Matrix4, 2> m_IdleFootBones{Math::Matrix4(Math::kIdentity), Math::Matrix4(Math::kIdentity)};
        bool m_HasIdleReference = false;
        uint64_t m_IdleReferenceRevision = 0;
        std::array<FootContactSample, 2> m_Contacts;
        const PhysicsWorld* m_ContactWorld = nullptr;
        uint64_t m_ClockRevision = 0, m_RagdollRevision = 0, m_ContactStep = 0, m_ContactRevision = 0;
        const Model* m_Model = nullptr;
        bool m_Initialized = false;
        float m_BufferScale = 2.0f;
    };
}
