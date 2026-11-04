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

    // 접촉에 대한 정보
    struct FootContactSample
    {
        bool hasSample = false;         // 접촉 판정에 사용할 샘플이 있는지
        bool grounded = false;          // 실제로 바닥에 닿아 있는지

        // 이번 완료 물리 스텝에서 이륙, 착지했는지
        bool leftGround = false;        
        bool landed = false;            

        // 접촉 위치와 접촉면 법선
        physx::PxVec3 position{0};
        physx::PxVec3 normal{ 0 };

        uint32_t pointCount = 0;        // 접촉점 개수
        float normalImpulse = 0;        // 법선 방향 충격량
        uint64_t takeoffCount = 0;      // 누적 이륙횟수
        uint64_t landingCount = 0;      // 누적 착지횟수
        uint64_t lastTakeoffStep = 0;   // 마지막 이륙이 발생한 물리 스텝 번호
        uint64_t lastLandingStep = 0;   // 마지막 착지가 발생한 물리 스텝 번호
    };

    /*
    발 하나에 대한 정보
    */
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


    /*
    현재 측정, 판단 결과를 모아 놓은 FootSupportSnapshot
    */
    struct FootSupportSnapshot
    {
        
        std::array<FootSoleSample, 2> feet;                                             // 발 두개를 모아놓은 array
        std::vector<physx::PxVec3> hull;                                                // 현재 발에서 계산한 지지 영역 A
        FootSupportBalance balance;                                                     // 현재 COM이 지지영역 A와 완충 영역 B를 기준으로 어디에 있는지 저장
        FootSupportCandidates candidates;                                               // 지지발과 움직일 발의 후보 
        FootstepSnapshot footstep;                                                      // 발딛기 플래너의 판단과 진행 상태. 발딛기가 필요한지, 어느 발을 움직일지, 어디에 착지할지, 현재 어느단계인지 저장
        physx::PxVec3 centerOfMass{0};                                                  // 전신 질량중심의 월드 위치
        physx::PxVec3 projectedCenterOfMass{0};                                         // 지면 높이에 투영한 COM
        physx::PxVec3 centerOfMassVelocity{0};                                          // 전신 COM의 속도. 미래 COM 예측, 회복 종료 시 수평 속도 검사, 서있기 제어의 감쇠 등에 사용
        float totalMass = 0;                                                            // 전신 질량. ApplyRecoveryTargets()에서 외부 충격량을 이 값으로 나눠 충격의 크기를 속도 변화 단위로 환산
        size_t projectedPointCount = 0;                                                 // 바닥을 맞힌 레이의 개수. 양박의 모서리에서 쏜 레이 중 실제로 무언가를 맞힌 개수 지금 범위는 0~8. hull생성과 평균 지면 높이 계산에 사용
        bool hasCenterOfMass = false;                                                   // COM 위치 계산의 성공 여부. centerOfMass를 유효하게 얻었는지 나타냄. COM위치를 사용할 때 이 값으로 유효성을 확인. false면 centerOfMass에 기본 값(0,0,0)이 남아있을 수 있음
        bool hasComProjection = false;                                                  // COM투영값을 계산했는지. 유효한 COM과 바닥을 맞힌 레이 점이 있어서 projectedCenterOfMass를 계산할 수 있었는지
        bool comInsideHull = false;                                                     // 현재 COM이 지지 영역 안에 있는지

        bool hasCenterOfMassVelocity = false;                                           // COM 속도 계산의 성공 여부
        uint64_t sampleNumber = 0;                                                      // [디버그]FootSupport::Update 를 호출할 때 마다 증가하는 디버그용 번호
        uint64_t completedPhysicsStep = 0;                                              // 결과가 속한 완료 물리 스텝번호. 회복제어와 발딛기 제어가 같은 물리결과를 중복 처리하지않도록 사용하는 값
        uint64_t contactRevision = 0;                                                   // 접촉 이력이 초기화 됐는지 구분하는 번호. 물리월드 변경, 물리 시간 초기화, 래그돌의 접촉 무효화 등으로 기존 접촉 이력을 버려야할때 증가. 회적제어와 IK등은 이 값의 변경을 감지해 이전 상태를 초기화
        std::string message;

        FootstepRoles GetDisplayRoles() const
        {
            const FootstepRoles roles = footstep.GetRoles();
            if (!roles.IsValid())
            {
                return {};
            }
            const FootSoleSample& support = feet[roles.supportFoot];
            physx::PxVec3 normal = support.contact.normal;
            if (!support.available || !support.contact.hasSample || !support.contact.grounded ||
                !normal.isFinite() || normal.normalize() < 1e-6f || normal.y < .70710678f)
            {
                // 진행 중에도 지지 접촉이 무효면 색상 표시만 숨긴다. 확정한 제어 역할은 유지한다.
                return {};
            }
            return roles;
        }
    };

    // Samples actual bodies before simulate(), independently of rendering/debug visibility.
    // Measures geometry only; SkeletalMeshComponent applies the optional support controller.

    // 양발의 위치와 접촉상태, 전신 질량중심, 지지 영역, 발딛기 판단이 들어잇음
    class FootSupport
    {
    public:
        static constexpr float RayLength = .2f;
        void Update(SkeletalMeshComponent& mesh, const PhysicsWorld& physics);
        void Reset();
        // 명시적으로 자동 지지를 재설정할 때 기존 해제 래치와 발 판단을 함께 초기화한다.
        void ResetFootstep();
        // PA 적용 시에만 저장한 발 바디의 연결·형상 변경 여부를 확인한다.
        void OnPhysicsAssetChanged(const PhysicsAsset& asset);
        // 발바닥 설정 초기화 후 다음 갱신에서 기준 정보를 다시 준비한다.
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
        FootstepPlanner m_FootstepPlanner;                                                                              // FootSupport가 수집한 발, 균형 정보로 발딛기를 판단하고, 움직일 발, 착지 목표와 진행 단계 결정
        FootstepSettings m_FootstepSettings;                                                                            // 발 이동에 대한 세팅값
        std::array<Math::Matrix4, 2> m_IdleFootBones{Math::Matrix4(Math::kIdentity), Math::Matrix4(Math::kIdentity)};   // idle 애니메이션에서 가져온 양발 본의 기준 위치,회전을 저장하는 배열. 발딛기 목표를 계산할 때 기준 발 간격과 배치를 알아내는데 사용
        
        bool m_HasIdleReference = false;                                                                                // idle기준 자세를 저장했는지. 기준의 존재 여부
        uint64_t m_IdleReferenceRevision = 0;                                                                           // 접촉 정보가 바뀌어 그 기준을 새로 준비해야하는지 판단
        std::array<FootContactSample, 2> m_Contacts;                                                                    // 접촉에 대한 정보를 포함하는 배열
        const PhysicsWorld* m_ContactWorld = nullptr;
        
        uint64_t m_ClockRevision = 0;                                                                                   // 마지막으로 확인한 물리시간
        uint64_t m_RagdollRevision = 0;                                                                                 // 마지막으로 확인한 래그돌 접촉
        uint64_t m_ContactStep = 0;                                                                                     // 마지막으로 확인한 물리스텝
        uint64_t m_ContactRevision = 0;                                                                                 // 자체의 접촉 이력버전
        bool m_Initialized = false;                                                                                     // 발 지지 계산에 필요한 기본 정보를 준비했는지. prepare()에서 필요한 정보를 준비했는지 표현
        float m_BufferScale = 2.0f;                                                                                     // 지지영역 A를 확장해 완충 영역 B를 만드는 배율. 2배면 반지름이 두배
    };
}
