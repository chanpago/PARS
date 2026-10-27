#pragma once

#include "PhysicsAsset.h"
#include "FootSupportHull.h"
#include <array>
#include <cstdint>

class Model;
class PhysicsWorld;

namespace PARS
{
    class SkeletalMeshComponent;

    struct FootSoleSettings
    {
        float halfWidth = .05f, halfLength = .1f;
        DirectX::XMFLOAT3 offset{0, 0, 0}, rotation{0, 0, 0};
    };

    struct FootSoleSample
    {
        bool available = false;
        std::array<physx::PxVec3, 4> corners{}, projections{};
        std::array<bool, 4> projected{};
        std::array<float, 4> distances{};
    };

    struct FootSupportSnapshot
    {
        std::array<FootSoleSample, 2> feet;
        std::vector<physx::PxVec3> hull;
        FootSupportBalance balance;
        physx::PxVec3 centerOfMass{0}, projectedCenterOfMass{0};
        float totalMass = 0;
        size_t projectedPointCount = 0;
        bool hasCenterOfMass = false, hasComProjection = false, comInsideHull = false;
        uint64_t sampleNumber = 0, completedPhysicsStep = 0;
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

    private:
        bool Prepare(SkeletalMeshComponent& mesh);
        struct Sole
        {
            int bodyIndex = -1;
            PhysicsBody source;
            Math::Matrix4 soleInBody{Math::kIdentity};
        };
        std::array<Sole, 2> m_Feet;
        std::array<FootSoleSettings, 2> m_Settings;
        FootSupportSnapshot m_Snapshot;
        const Model* m_Model = nullptr;
        bool m_Initialized = false;
        float m_BufferScale = 2.0f;
    };
}
