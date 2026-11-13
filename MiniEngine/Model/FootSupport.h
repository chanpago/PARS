#pragma once

#include "PhysicsAsset.h"
#include "FootSupportContact.h"
#include "FootSupportHull.h"
#include "FootSupportGrace.h"
#include "FootstepPlanner.h"
#include "PhysicsWorld.h"
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
    public:
        float halfWidth = .05f;
        float halfLength = .1f;
        DirectX::XMFLOAT3 offset{0, 0, 0};
        DirectX::XMFLOAT3 rotation{0, 0, 0};
    };

    struct FootLegSetup
    {
    public:
        int hipNode = -1;
        int kneeNode = -1;
        int ankleNode = -1;
        // 기준 자세의 실제 구간 길이(m). 현재 자세의 수직 투영 길이가 아니다.
        float thighLength = 0;
        float calfLength = 0;
        physx::PxVec3 referenceBendInHip{0};
        bool available = false;
    };

    struct FootPartContact
    {
    public:
        FootSupportContact support;
        PhysicsGroundContact ground;
        bool landed = false;
    };

    // 발딛기는 한쪽 발 단위로 판단하고, 토크 보상에는 각 부위의 접촉을 남긴다.
    struct FootContactSample
    {
    public:
        void Update(uint64_t completedStep, bool available,
            const PhysicsGroundContact& foot, const PhysicsGroundContact& toe)
        {
            const bool previousHasSample = hasSample;
            const bool previousGrounded = grounded;
            hasSample = available;
            const std::array<PhysicsGroundContact, 2> samples{foot, toe};
            for (size_t part = 0; part < parts.size(); ++part)
            {
                const bool previouslyGrounded = parts[part].ground.pointCount > 0;
                parts[part].ground = hasSample ? samples[part] : PhysicsGroundContact();
                parts[part].landed = previousHasSample && hasSample && !previouslyGrounded &&
                    parts[part].ground.pointCount > 0;
                parts[part].support.Update(completedStep, hasSample, parts[part].ground.pointCount > 0,
                    parts[part].ground.position, parts[part].ground.normal);
            }
            pointCount = hasSample ? foot.pointCount + toe.pointCount : 0;
            grounded = pointCount > 0;
            position = physx::PxVec3(0);
            normal = physx::PxVec3(0);
            normalImpulse = 0;
            normalImpulseVector = physx::PxVec3(0);
            if (grounded)
            {
                normalImpulse = foot.normalImpulse + toe.normalImpulse;
                normalImpulseVector = foot.normalImpulseVector + toe.normalImpulseVector;
                const float verticalImpulse = normalImpulseVector.y;
                // 두 부위의 수직 하중으로 지지 위치를 계산한다. 무하중일 때는 접촉점 수를 사용한다.
                const float footWeight = verticalImpulse > 1e-8f ? foot.normalImpulseVector.y :
                    static_cast<float>(foot.pointCount);
                const float toeWeight = verticalImpulse > 1e-8f ? toe.normalImpulseVector.y :
                    static_cast<float>(toe.pointCount);
                const float totalWeight = footWeight + toeWeight;
                supportFractions = {footWeight / totalWeight, toeWeight / totalWeight};
                position = foot.position * supportFractions[0] + toe.position * supportFractions[1];
                normal = verticalImpulse > 1e-8f ? normalImpulseVector :
                    foot.normal * footWeight + toe.normal * toeWeight;
                normal.normalize();
            }

            support.Update(completedStep, hasSample, grounded, position, normal);
            if (!support.IsValid())
            {
                supportFractions = {};
            }
            // 발과 발가락 사이의 접촉 이동은 이륙이나 재착지로 세지 않는다.
            leftGround = previousHasSample && hasSample && previousGrounded && !grounded;
            landed = previousHasSample && hasSample && !previousGrounded && grounded;
            if (leftGround)
            {
                ++takeoffCount;
                lastTakeoffStep = completedStep;
            }
            if (landed)
            {
                ++landingCount;
                lastLandingStep = completedStep;
            }
        }

    public:
        bool IsPartSupporting(size_t part) const
        {
            return support.IsValid() && parts.at(part).support.IsValid() && supportFractions.at(part) > 0;
        }

        bool IsToeOnlySupport() const
        {
            return IsPartSupporting(1) && !IsPartSupporting(0);
        }

    public:
        uint64_t takeoffCount = 0;    // 누적 이륙횟수
        uint64_t landingCount = 0;    // 누적 착지횟수
        uint64_t lastTakeoffStep = 0; // 마지막 이륙이 발생한 물리 스텝 번호
        uint64_t lastLandingStep = 0; // 마지막 착지가 발생한 물리 스텝 번호
        FootSupportContact support; // 보정과 표시에만 쓰는 3틱 접촉 유예
        std::array<FootPartContact, 2> parts;
        // 접촉 위치와 접촉면 법선
        physx::PxVec3 position{0};
        physx::PxVec3 normal{0};
        physx::PxVec3 normalImpulseVector{0};
        // 접촉 유예 동안은 마지막 분담 비율만 유지한다. 측정 충격량은 재사용하지 않는다.
        std::array<float, 2> supportFractions{};
        uint32_t pointCount = 0; // 접촉점 개수
        float normalImpulse = 0; // 법선 방향 충격량
        bool hasSample = false;  // 접촉 판정에 사용할 샘플이 있는지
        bool grounded = false;   // 실제로 바닥에 닿아 있는지
        // 이번 완료 물리 스텝에서 이륙, 착지했는지
        bool leftGround = false;
        bool landed = false;
    };

    struct FootToeSample
    {
    public:
        Math::Matrix4 soleInBone{Math::kIdentity};
        Math::Matrix4 soleWorld{Math::kIdentity};
        Math::Matrix4 supportSoleWorld{Math::kIdentity};
        std::array<physx::PxVec3, 8> corners{};
        std::array<physx::PxVec3, 8> projections{};
        std::array<float, 8> distances{};
        int bodyIndex = -1;
        int boneNode = -1;
        bool available = false;
        std::array<bool, 8> projected{};
    };

    // 발목 IK 기준은 Foot에 두고 Toe의 물리 자세와 투영을 함께 보관한다.
    struct FootSoleSample
    {
    public:
        // 발바닥 목표를 PA 바디 및 발목 본의 목표로 되돌릴 때 사용하는 변환이다.
        Math::Matrix4 soleInBody{Math::kIdentity};
        Math::Matrix4 soleInBone{Math::kIdentity};
        Math::Matrix4 soleWorld{Math::kIdentity};
        Math::Matrix4 supportSoleWorld{Math::kIdentity};
        FootToeSample toe;
        FootContactSample contact;
        int bodyIndex = -1;
        int boneNode = -1;
        FootLegSetup leg;
        std::array<physx::PxVec3, 4> corners{};
        std::array<physx::PxVec3, 4> projections{};
        std::array<float, 4> distances{};
        bool available = false;
        std::array<bool, 4> projected{};
    };

    /*
    현재 측정, 판단 결과를 모아 놓은 FootSupportSnapshot
    */
    struct FootSupportSnapshot
    {
    public:
        FootstepRoles GetDisplayRoles() const
        {
            const FootstepRoles roles = footstep.GetRoles();
            if (!roles.IsValid())
            {
                return {};
            }
            const FootSoleSample& support = feet[roles.supportFoot];
            if (!support.available || !support.contact.hasSample || !support.contact.support.IsValid())
            {
                // 짧은 접촉 상실은 유예하고, 만료되면 확정한 제어 역할을 유지한 채 색만 숨긴다.
                return {};
            }
            return roles;
        }

    public:
        std::array<FootSoleSample, 2> feet; // 발 두개를 모아놓은 array
        std::vector<physx::PxVec3> hull;    // 현재 발에서 계산한 지지 영역 A
        FootSupportBalance balance;         // 현재 COM이 지지영역 A와 완충 영역 B를 기준으로 어디에 있는지 저장
        FootstepSnapshot footstep;          // 발딛기 플래너의 판단과 진행 상태. 발딛기가 필요한지, 어느 발을 움직일지, 어디에 착지할지, 현재 어느단계인지 저장
        size_t projectedPointCount = 0;     // Foot 사각형과 Toe 윤곽의 유효 레이 개수다. 실제 접촉점 개수와 구분한다.
        uint64_t sampleNumber = 0;          // [디버그]FootSupport::Update 를 호출할 때 마다 증가하는 디버그용 번호
        uint64_t completedPhysicsStep = 0;  // 결과가 속한 완료 물리 스텝번호. 회복제어와 발딛기 제어가 같은 물리결과를 중복 처리하지않도록 사용하는 값
        uint64_t contactRevision = 0;       // 접촉 이력이 초기화 됐는지 구분하는 번호. 물리월드 변경, 물리 시간 초기화, 래그돌의 접촉 무효화 등으로 기존 접촉 이력을 버려야할때 증가. 회적제어와 IK등은 이 값의 변경을 감지해 이전 상태를 초기화
        std::string message;
        FootSupportCandidates candidates;       // 지지발과 움직일 발의 후보
        physx::PxVec3 centerOfMass{0};          // 전신 질량중심의 월드 위치
        physx::PxVec3 projectedCenterOfMass{0}; // 지면 높이에 투영한 COM
        physx::PxVec3 centerOfMassVelocity{0};  // 전신 COM의 속도. 미래 COM 예측, 회복 종료 시 수평 속도 검사, 서있기 제어의 감쇠 등에 사용
        float totalMass = 0;                    // 전신 질량. ApplyRecoveryTargets()에서 외부 충격량을 이 값으로 나눠 충격의 크기를 속도 변화 단위로 환산
        bool hasCenterOfMass = false;           // COM 위치 계산의 성공 여부. centerOfMass를 유효하게 얻었는지 나타냄. COM위치를 사용할 때 이 값으로 유효성을 확인. false면 centerOfMass에 기본 값(0,0,0)이 남아있을 수 있음
        bool hasComProjection = false;          // COM투영값을 계산했는지. 유효한 COM과 바닥을 맞힌 레이 점이 있어서 projectedCenterOfMass를 계산할 수 있었는지
        bool comInsideHull = false;             // 현재 COM이 지지 영역 안에 있는지
        bool hasCenterOfMassVelocity = false;   // COM 속도 계산의 성공 여부
    };

    // Samples actual bodies before simulate(), independently of rendering/debug visibility.
    // Measures geometry only; SkeletalMeshComponent applies the optional support controller.

    // 양발의 위치와 접촉상태, 전신 질량중심, 지지 영역, 발딛기 판단이 들어잇음
    class FootSupport
    {
    private:
        struct Sole
        {
        public:
            Math::Matrix4 soleInBody{Math::kIdentity};
            Math::Matrix4 toeSoleInBody{Math::kIdentity};
            Math::Matrix4 toeSupportSoleWorld{Math::kIdentity};
            PhysicsBody source;
            PhysicsBody toeSource;
            int bodyIndex = -1;
            int toeBodyIndex = -1;
            FootLegSetup leg;
            bool hasToeSupportSole = false;
        };

    public:
        void Update(SkeletalMeshComponent& mesh, const PhysicsWorld& physics);
        void Reset();
        // 명시적으로 자동 지지를 재설정할 때 기존 해제 래치와 발 판단을 함께 초기화한다.
        void ResetFootstep();
        void UpdateBufferGrace(bool enabled);
        // PA 적용 시에만 저장한 발 바디의 연결·형상 변경 여부를 확인한다.
        void OnPhysicsAssetChanged(const PhysicsAsset& asset);
        // 발바닥 설정 초기화 후 다음 갱신에서 기준 정보를 다시 준비한다.
        void ResetSoles()
        {
            m_Initialized = false;
        }

    private:
        bool Prepare(SkeletalMeshComponent& mesh);
        void UpdateContacts(const Ragdoll& ragdoll, const PhysicsWorld& physics);
        void UpdateFootstep(SkeletalMeshComponent& mesh);

    public:
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
        const FootSupportGrace& GetBufferGrace() const
        {
            return m_BufferGrace;
        }
        FootstepSettings& GetFootstepSettings()
        {
            return m_FootstepSettings;
        }

    public:
        static constexpr float RayLength = .2f;

    private:
        std::array<Sole, 2> m_Feet;
        FootSupportSnapshot m_Snapshot;
        FootstepPlanner m_FootstepPlanner;                                                                            // FootSupport가 수집한 발, 균형 정보로 발딛기를 판단하고, 움직일 발, 착지 목표와 진행 단계 결정
        FootSupportGrace m_BufferGrace;
        std::array<FootContactSample, 2> m_Contacts;                                                                  // 접촉에 대한 정보를 포함하는 배열
        const PhysicsWorld* m_ContactWorld = nullptr;
        uint64_t m_ClockRevision = 0;   // 마지막으로 확인한 물리시간
        uint64_t m_RagdollRevision = 0; // 마지막으로 확인한 래그돌 접촉
        uint64_t m_ContactStep = 0;     // 마지막으로 확인한 물리스텝
        uint64_t m_ContactRevision = 0; // 자체의 접촉 이력버전
        std::array<FootSoleSettings, 2> m_Settings;
        FootstepSettings m_FootstepSettings; // 발 이동에 대한 세팅값
        float m_BufferScale = 2.0f;          // 지지영역 A를 확장해 완충 영역 B를 만드는 배율. 2배면 반지름이 두배
        bool m_Initialized = false;          // 발 지지 계산에 필요한 기본 정보를 준비했는지. prepare()에서 필요한 정보를 준비했는지 표현
    };
}
