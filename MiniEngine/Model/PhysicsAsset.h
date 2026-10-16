#pragma once

#include "VectorMath.h"
#include <string>
#include <vector>

namespace PARS
{
    enum class PhysicsShapeType { Capsule, Box, Sphere };

    // One primitive body per bone for the initial editor. Distances are metres,
    // mass is kg. Offset/rotation are relative to the bone's RIGID world frame:
    // imported/animated bone scale never scales a PhysX collision primitive.
    // Capsule cylinder extends along local X, matching PxCapsuleGeometry.
    struct PhysicsBody
    {
        std::string bone;
        PhysicsShapeType shape = PhysicsShapeType::Capsule;
        DirectX::XMFLOAT3 offset{0, 0, 0};
        DirectX::XMFLOAT4 rotation{0, 0, 0, 1};
        DirectX::XMFLOAT3 halfExtents{.05f, .1f, .05f};
        float radius = .05f;
        float halfLength = .1f; // Cylinder half length; caps excluded.
        float mass = 1;

        Math::Matrix4 GetLocalTransform() const;
    };

    Math::Matrix4 PhysicsBoneFrame(const Math::Matrix4& boneWorld);

    struct PhysicsAsset
    {
        std::vector<PhysicsBody> bodies;
        int FindBody(const std::string& bone) const;
        bool Validate(std::string& error) const;
        bool Save(const std::string& path, std::string& error) const;
        // Transactional: a rejected file leaves this asset untouched.
        bool Load(const std::string& path, std::string& error);
    };
}
