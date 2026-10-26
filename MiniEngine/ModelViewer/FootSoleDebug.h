#pragma once

#include "PhysicsAsset.h"
#include "PhysicsWorld.h"
#include <array>

class Model;
namespace PARS { class SkeletalMeshComponent; }

// 실제 발 바디의 지면 투영, 지지 영역과 전체 COM의 수직 투영을 표시한다.
// 판정 결과는 표시용이며 캐릭터 구동이나 충돌 형상을 변경하지 않는다.
class FootSoleDebug
{
public:
    void DrawUI(PARS::SkeletalMeshComponent& mesh);
    void BuildLines(PARS::SkeletalMeshComponent& mesh, const PhysicsWorld& physics,
        std::vector<physx::PxDebugLine>& lines);

private:
    struct Sole
    {
        int bodyIndex = -1;
        PARS::PhysicsBody source;
        Math::Matrix4 soleInBody{Math::kIdentity};
        float halfWidth = .05f, halfLength = .1f;
        DirectX::XMFLOAT3 offset{0, 0, 0}, rotation{0, 0, 0};
        std::array<bool, 4> projected{};
        std::array<float, 4> distances{};
    };
    bool Prepare(PARS::SkeletalMeshComponent& mesh);
    static constexpr float RayLength = .2f;
    std::array<Sole, 2> m_Feet;
    std::vector<physx::PxVec3> m_SupportHull;
    size_t m_ProjectedPointCount = 0;
    physx::PxVec3 m_CenterOfMass{0}, m_ProjectedCenterOfMass{0};
    float m_TotalMass = 0;
    bool m_HasCenterOfMass = false, m_HasComProjection = false, m_ComInsideHull = false;
    const Model* m_Model = nullptr;
    bool m_Initialized = false, m_Visible = true, m_ShowRays = true, m_ShowHull = true;
    bool m_ShowCenterOfMass = true;
    std::string m_Message;
};
