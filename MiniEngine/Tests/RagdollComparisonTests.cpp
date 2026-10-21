#include "../Model/RagdollComparison.h"
#include "../Core/PhysicsWorld.h"
#include "../Core/GameFramework/World.h"
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iterator>
#include <limits>

using namespace Math;
using namespace PARS;

namespace
{
    void CheckComparison(bool value, const char* message)
    {
        if (!value)
        {
            std::fprintf(stderr, "FAIL: comparison: %s\n", message);
            std::exit(1);
        }
    }

    bool Near(float a, float b, float tolerance = 1e-4f)
    {
        return std::abs(a - b) < tolerance;
    }

    PhysicsAsset TestAsset()
    {
        PhysicsAsset asset;
        PhysicsBody parent, child;
        parent.bone = "Parent";
        child.bone = "Child";
        asset.bodies = {parent, child};
        PhysicsConstraint joint;
        joint.parentBone = parent.bone;
        joint.childBone = child.bone;
        joint.twistMin = -.2f;
        joint.twistMax = .2f;
        joint.swingY = PhysicsJointMotion::Locked;
        joint.swingZ = PhysicsJointMotion::Locked;
        asset.constraints.push_back(joint);
        return asset;
    }
}

void RunRagdollComparisonTests()
{
    std::string error;
    auto asset = TestAsset();
    RagdollComparison recorder;
    const Matrix4 placementA(Quaternion(Vector3(kYUnitVector), .5f), Vector3(-3, 1, 5));
    const Matrix4 placementB(Quaternion(Vector3(kYUnitVector), -.7f), Vector3(3, 1, -2));
    CheckComparison(recorder.Initialize(asset, placementA, placementB, .1f, error), "Initialize rotated placements");
    CheckComparison(!recorder.Sample({placementA, placementA}, {placementB, placementB}, 0, error), "Require a baseline");
    CheckComparison(recorder.Sample({placementA, placementA}, {placementB, placementB}, 0, error, true), "Capture baseline");
    const Matrix4 moved(Matrix3(kIdentity), Vector3(1, 0, 0));
    CheckComparison(recorder.Sample({placementA * moved, placementA * moved},
                        {placementB * moved, placementB * moved}, .1f, error),
        "Capture identical local motion");
    CheckComparison(recorder.GetBodies()[0].position.maximum < 1e-5f &&
            recorder.GetBodies()[0].movement.maximum < 1e-5f && recorder.GetBodies()[0].rotation.maximum < 1e-5f,
        "Initial translation and rotation do not create false errors");
    const Matrix4 ref(Matrix3(kIdentity), Vector3(2, 0, 0)), actual(Matrix3(kIdentity), Vector3(1, 1, 0));
    CheckComparison(recorder.Sample({placementA * ref, placementA * ref},
                        {placementB * actual, placementB * actual}, .2f, error),
        "Capture equal distance but different direction");
    CheckComparison(Near(recorder.GetFrames().back().bodies[0].movementError, std::sqrt(2.0f)),
        "Displacement vectors, not path length magnitudes, are compared");
    const Matrix4 raw(Quaternion(Vector3(kXUnitVector), .5f), Vector3(kZero));
    const Matrix4 bounded(Quaternion(Vector3(kXUnitVector), .2f), Vector3(kZero));
    CheckComparison(recorder.Initialize(asset, Matrix4(kIdentity), Matrix4(kIdentity), .1f, error), "Reset statistics");
    CheckComparison(recorder.Sample({Matrix4(kIdentity), raw}, {Matrix4(kIdentity), bounded}, 0, error, true), "Clamped baseline");
    for (int i = 0; i < 3; ++i)
    {
        CheckComparison(recorder.Sample({Matrix4(kIdentity), raw}, {Matrix4(kIdentity), bounded}, .1f * i, error), "Limit sample");
    }
    const auto& result = recorder.GetJoints()[0];
    CheckComparison(Near(result.clamp.Mean(), .3f) && result.tracking.maximum < 1e-5f && Near(result.total.maximum, .3f),
        "Raw target clamping remains visible even when physical tracking is perfect");
    CheckComparison(result.violations[0].frames == 3 && result.violations[0].longestFrames == 3 &&
            Near(result.violations[0].firstTime, .1f) && Near(result.violations[0].lastTime, .3f),
        "Violation exposure and timestamps use fixed physics samples");
    auto joint = asset.constraints[0];
    auto locked = AnalyzeComparisonJoint(joint, Quaternion(Vector3(kYUnitVector), .3f));
    CheckComparison(Near(locked.excess[1], .3f), "Locked swing violation");
    joint.swingY = joint.swingZ = PhysicsJointMotion::Limited;
    joint.swingYLimit = joint.swingZLimit = .5f;
    const float t = std::tan(.4f * .25f), r2 = 2 * t * t;
    const Quaternion diagonal(XMVectorSet(0, 2 * t / (1 + r2), 2 * t / (1 + r2), (1 - r2) / (1 + r2)));
    const auto cone = AnalyzeComparisonJoint(joint, diagonal);
    CheckComparison(cone.excess[1] == 0 && cone.excess[2] == 0 && cone.excess[3] > .06f,
        "Coupled swing cone detects diagonal violations within both individual limits");
    const auto native = AnalyzeComparisonJoint(joint, diagonal, true);
    CheckComparison(native.excess == std::array<float, 4>{} &&
        ComparisonRotationError(diagonal, ClampArticulationDriveTarget(joint, diagonal)) < 1e-5f,
        "Articulation diagnostics use independent rotation-vector bounds instead of the D6 cone");
    const auto nativeClamped = ClampArticulationDriveTarget(joint, Quaternion(Vector3(kYUnitVector), .8f));
    CheckComparison(Near(AnalyzeComparisonJoint(joint, nativeClamped, true).angles[1], .5f),
        "Articulation target and diagnostic limit coordinates agree");
    const auto negative = AnalyzeComparisonJoint(joint, -diagonal);
    CheckComparison(Near(cone.excess[3], negative.excess[3]) && ComparisonRotationError(diagonal, -diagonal) < 1e-6f,
        "Quaternion sign does not change diagnostics");
    joint.twist = joint.swingY = joint.swingZ = PhysicsJointMotion::Free;
    const auto free = AnalyzeComparisonJoint(joint, diagonal);
    CheckComparison(free.excess == std::array<float, 4>{}, "Free axes never produce limit exposure");
    CheckComparison(AnalyzeComparisonJoint(joint, Quaternion(XMVectorSet(0, 1, 0, 0))).singular,
        "Ambiguous twist at 180-degree swing is flagged");
    const size_t framesBefore = recorder.GetFrames().size();
    CheckComparison(!recorder.Sample({raw}, {bounded}, 0, error) && recorder.GetFrames().size() == framesBefore,
        "Invalid samples do not partially update results");
    CheckComparison(recorder.ExportHtml(".tools/comparison-unit.html", "</script><script>bad()</script>", error), "Export self-contained HTML");
    std::ifstream report(".tools/comparison-unit.html", std::ios::binary);
    const std::string html((std::istreambuf_iterator<char>(report)), {});
    CheckComparison(html.find("\\u003c/script\\u003e") != std::string::npos &&
            html.find("</script><script>bad()") == std::string::npos,
        "Report metadata cannot terminate the data script");

    PhysicsWorld owner;
    CheckComparison(owner.Initialize(false), "Owner physics initialized");
    const uint32_t errors = owner.GetErrorCount();
    {
        World isolated;
        CheckComparison(isolated.InitializePhysics(false, &owner), "Separate scene shares SDK services");
        auto* physics = isolated.GetPhysicsWorld();
        auto* body = physics->CreateDynamicBox(physx::PxTransform(physx::PxVec3(0, 3, 0)), physx::PxVec3(.2f), 1, "Isolated");
        CheckComparison(body && owner.GetDynamicBodyCount() == 0, "Objects stay out of owner scene");
        physics->Step();
        CheckComparison(physics->GetStepCount() == 1 && owner.GetStepCount() == 0, "Isolated fixed clock");
    }
    owner.Step();
    CheckComparison(owner.GetErrorCount() == errors && owner.GetStepCount() == 1, "Owner survives isolated scene destruction");
    std::puts("PASS: comparison measurements, joint limits, report escaping and isolated scene lifecycle");
}
