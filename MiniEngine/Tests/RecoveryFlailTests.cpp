#include "../Model/RecoveryFlail.h"
#include "../Model/PhysicsPose.h"
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits>

namespace
{
    unsigned checks = 0;

    void Check(bool condition, const char* message)
    {
        ++checks;
        if (!condition)
        {
            std::fprintf(stderr, "FAIL: %s\n", message);
            std::exit(1);
        }
    }

    PARS::PhysicsSkeleton Skeleton()
    {
        PARS::PhysicsSkeleton skeleton;
        skeleton.names = {"Armature", "mixamorig1:Hips", "LeftUpLeg", "LeftLeg", "RightUpLeg",
            "mixamorig1:Spine", "mixamorig1:Spine1", "mixamorig1:Spine2", "LeftShoulder", "LeftArm", "Neck", "Head"};
        skeleton.parents = {-1, 0, 1, 2, 1, 1, 5, 6, 7, 8, 7, 10};
        skeleton.meshNodes.assign(skeleton.names.size(), -1);
        return skeleton;
    }

    bool SameRotation(Math::Quaternion first, Math::Quaternion second)
    {
        const float dot = std::abs(DirectX::XMVectorGetX(DirectX::XMQuaternionDot(first, second)));
        return std::abs(dot - 1.0f) < 1e-5f;
    }

    std::vector<GraphNode> Pose(size_t count, float angle)
    {
        std::vector<GraphNode> pose(count);
        for (size_t node = 0; node < count; ++node)
        {
            GraphNode& bone = pose[node];
            bone.xform = Math::Matrix4(Math::kIdentity);
            bone.xform.SetW(Math::Vector4(0, .1f * static_cast<float>(node), .02f, 1));
            bone.rotation = Math::Quaternion(Math::Vector3(Math::kYUnitVector), angle);
            bone.scale = DirectX::XMFLOAT3(1, 1, 1);
            bone.matrixIdx = static_cast<uint32_t>(node);
            bone.hasSibling = false;
            bone.hasChildren = false;
            bone.skeletonRoot = false;
            bone.staleMatrix = true;
        }
        return pose;
    }

    void TestDirectionAndLifetime()
    {
        const PARS::PhysicsSkeleton skeleton = Skeleton();
        PARS::RecoveryFlail flail;
        Check(!flail.Configure(skeleton.names, skeleton.parents, 3, 3), "Duplicate flail clips are rejected");
        Check(flail.Configure(skeleton.names, skeleton.parents, 3, 4), "Configure the Spine subtree");
        flail.RecordImpact(Math::Vector3(0, 5, 10), Math::Vector3(0, 0, 1));
        flail.Update(true, .075f, 4.7f, 8.3f);
        Check(flail.GetFrontWeight() == 1 && std::abs(flail.GetWeight() - .5f) < 1e-5f,
            "A forward received impulse selects Front with a gradual layer entrance");
        const float phase = flail.GetPhase();
        flail.Update(true, .075f, 4.7f, 8.3f);
        Check(flail.GetFrontWeight() == 1 && flail.GetWeight() == 1 && flail.GetPhase() > phase,
            "Direction survives contact loss and playback continues");
        flail.RecordImpact(Math::Vector3(0, 0, -10), Math::Vector3(0, 0, 1));
        flail.Update(true, 1.0f / 60.0f, 4.7f, 8.3f);
        Check(flail.GetFrontWeight() > 0 && flail.GetFrontWeight() < 1 && flail.GetPhase() > phase,
            "A new opposite impact blends direction without restarting playback");
        flail.Update(false, 1.0f / 60.0f, 4.7f, 8.3f);
        Check(flail.GetWeight() == 0 && flail.GetPhase() == 0 && !flail.HasImpactDirection() && flail.IsConfigured(),
            "Leaving Recovery clears playback and direction while preserving the configuration");
        flail.RecordImpact(Math::Vector3(0, 0, -10), Math::Vector3(0, 0, 1));
        Check(flail.GetFrontWeight() == 0, "A backward received impulse selects Back");
        flail.Reset();
        flail.RecordImpact(Math::Vector3(10, 0, 0), Math::Vector3(0, 0, 1));
        Check(flail.GetFrontWeight() == .5f, "A lateral impact blends Front and Back equally");
        flail.Reset();
        flail.RecordImpact(Math::Vector3(10, 0, 0), Math::Vector3(1, 0, 0));
        Check(flail.GetFrontWeight() == 1, "The blend uses character facing rather than the fixed world Z axis");
        flail.Reset();
        flail.RecordImpact(Math::Vector3(1, 0, 1), Math::Vector3(0, 0, 1));
        Check(std::abs(flail.GetFrontWeight() - (.5f + .5f / std::sqrt(2.0f))) < 1e-5f,
            "An oblique impact continuously interpolates the two clips");
        flail.Reset();
        flail.RecordImpact(Math::Vector3(0, 10, 0), Math::Vector3(0, 0, 1));
        Check(!flail.HasImpactDirection(), "A vertical or zero impulse cannot invent a horizontal direction");
        flail.RecordImpact(Math::Vector3(std::numeric_limits<float>::quiet_NaN(), 0, 1), Math::Vector3(0, 0, 1));
        Check(!flail.HasImpactDirection(), "Non-finite impulse input is ignored");
        flail.Update(true, std::numeric_limits<float>::quiet_NaN(), 4.7f, 8.3f);
        Check(flail.GetWeight() == 0 && flail.GetPhase() == 0, "Invalid time does not corrupt playback");
        flail.Clear();
        Check(!flail.IsConfigured(), "Replacing the mesh or animation instance can discard the old mask and clips");
    }

    void TestUpperBodyMask()
    {
        const PARS::PhysicsSkeleton skeleton = Skeleton();
        const std::vector<GraphNode> base = Pose(skeleton.names.size(), 0);
        const std::vector<GraphNode> front = Pose(skeleton.names.size(), .8f);
        const std::vector<GraphNode> back = Pose(skeleton.names.size(), -.4f);
        PARS::RecoveryFlail flail;
        Check(flail.Configure(skeleton.names, skeleton.parents, 3, 4), "Prepare the flail mask");
        flail.RecordImpact(Math::Vector3(0, 0, 1), Math::Vector3(0, 0, 1));
        flail.Update(true, .15f, 4.7f, 8.3f);
        std::vector<GraphNode> pose = base;
        Check(flail.Apply(front, back, pose), "Apply upper-body local rotation targets");
        for (size_t node = 0; node < pose.size(); ++node)
        {
            Check(std::memcmp(&pose[node].xform, &base[node].xform, sizeof(Math::Matrix4)) == 0 &&
                std::memcmp(&pose[node].scale, &base[node].scale, sizeof(DirectX::XMFLOAT3)) == 0,
                "Flail never replaces translations, scales, or matrix indices");
        }
        for (size_t node = 0; node < 5; ++node)
        {
            Check(std::memcmp(&pose[node], &base[node], sizeof(GraphNode)) == 0,
                "Armature, pelvis, and both leg branches remain completely unchanged");
        }
        Check(SameRotation(pose[5].rotation, Math::Quaternion(Math::Vector3(Math::kYUnitVector), .16f)),
            "The first Spine receives only a small fraction of the flail rotation");
        Check(SameRotation(pose[9].rotation, front[9].rotation), "The arm receives the full flail rotation");
        Check(SameRotation(pose[11].rotation, Math::Quaternion(Math::Vector3(Math::kYUnitVector), .2f)),
            "The head receives a reduced rotation");
        std::vector<Math::Matrix4> baseWorld;
        std::vector<Math::Matrix4> flailWorld;
        std::string error;
        Check(PARS::EvaluatePhysicsPose(skeleton, base, Math::Matrix4(Math::kIdentity), baseWorld, error) &&
            PARS::EvaluatePhysicsPose(skeleton, pose, Math::Matrix4(Math::kIdentity), flailWorld, error),
            "Convert the layered local pose through the actual physics hierarchy");
        for (size_t node = 0; node < 5; ++node)
        {
            Check(std::memcmp(&baseWorld[node], &flailWorld[node], sizeof(Math::Matrix4)) == 0,
                "World-space pelvis and leg targets are preserved before the existing leg IK");
        }
        flail.Reset();
        flail.RecordImpact(Math::Vector3(1, 0, 0), Math::Vector3(0, 0, 1));
        flail.Update(true, .15f, 4.7f, 8.3f);
        pose = base;
        Check(flail.Apply(front, back, pose) &&
            SameRotation(pose[9].rotation, Math::Quaternion(Math::Vector3(Math::kYUnitVector), .2f)),
            "A side impact produces the quaternion midpoint of the two arm clips");
        flail.Update(false, .016f, 4.7f, 8.3f);
        pose = base;
        Check(flail.Apply(front, back, pose) && std::memcmp(pose.data(), base.data(), pose.size() * sizeof(GraphNode)) == 0,
            "Disabled flail cannot alter normal Idle, falling, standing-up, or kinematic poses");
    }
}

void RunRecoveryFlailTests()
{
    TestDirectionAndLifetime();
    TestUpperBodyMask();
    std::printf("PASS: recovery upper-body flail (%u checks)\n", checks);
}
