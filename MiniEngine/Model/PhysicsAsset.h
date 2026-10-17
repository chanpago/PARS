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

    enum class PhysicsJointMotion { Locked, Limited, Free };

    struct PhysicsJointFrame
    {
        DirectX::XMFLOAT3 offset{0, 0, 0};
        DirectX::XMFLOAT4 rotation{0, 0, 0, 1};
        Math::Matrix4 GetTransform() const;
        static PhysicsJointFrame FromTransform(const Math::Matrix4& rigidTransform);
    };

    // D6 joint data: translation is locked; X is twist, Y/Z are swing.
    // Frames are relative to the BODY (bone rigid frame * collider offset).
    // Angles are radians, positions are metres. No axis/handedness conversion.
    struct PhysicsConstraint
    {
        std::string parentBone, childBone;
        PhysicsJointFrame parentFrame, childFrame;
        PhysicsJointMotion twist = PhysicsJointMotion::Limited;
        PhysicsJointMotion swingY = PhysicsJointMotion::Limited;
        PhysicsJointMotion swingZ = PhysicsJointMotion::Limited;
        float twistMin = -DirectX::XM_PIDIV4, twistMax = DirectX::XM_PIDIV4;
        float swingYLimit = DirectX::XM_PI / 6, swingZLimit = DirectX::XM_PI / 6;
        bool enableCollision = false;

        void SetWorldFrame(const Math::Matrix4& parentBodyWorld,
            const Math::Matrix4& childBodyWorld, const Math::Matrix4& jointWorld);
    };

    struct PhysicsAsset
    {
        std::vector<PhysicsBody> bodies;
        std::vector<PhysicsConstraint> constraints;
        int FindBody(const std::string& bone) const;
        int FindConstraint(const std::string& boneA, const std::string& boneB) const;
        // Removing a body also removes every attached joint.
        void RemoveBody(size_t index);
        bool Validate(std::string& error) const;
        bool ValidateSkeleton(const std::vector<std::string>& jointNames, std::string& error) const;
        bool Save(const std::string& path, std::string& error) const;
        // Transactional: a rejected file leaves this asset untouched.
        bool Load(const std::string& path, std::string& error);
    };
}
