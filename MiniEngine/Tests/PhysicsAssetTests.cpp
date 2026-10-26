#include "../Model/PhysicsAsset.h"
#include <Windows.h>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <limits>
#pragma warning(push)
#pragma warning(disable: 4100)
#include "../Model/json.hpp"
#pragma warning(pop)

using namespace Math;
using namespace PARS;

namespace
{
    void Check(bool condition, const char* message)
    {
        if (!condition) { std::fprintf(stderr, "FAIL: %s\n", message); std::exit(1); }
    }
    bool Near(Vector3 a, Vector3 b) { return float(Length(a - b)) < 1e-5f; }
    bool SameFrame(const Matrix4& a, const Matrix4& b)
    {
        return Near(Vector3(a.GetW()), Vector3(b.GetW())) && Near(Vector3(a.GetX()), Vector3(b.GetX())) &&
            Near(Vector3(a.GetY()), Vector3(b.GetY())) && Near(Vector3(a.GetZ()), Vector3(b.GetZ()));
    }

    void TestBodyEditFrames()
    {
        PhysicsAsset asset;
        std::vector<Matrix4> bones;
        for (int i = 0; i < 4; ++i)
        {
            PhysicsBody body;
            body.bone = "edit-bone-" + std::to_string(i);
            body.offset = DirectX::XMFLOAT3(.1f, -.2f, .05f);
            DirectX::XMStoreFloat4(&body.rotation, Quaternion(Vector3(kZUnitVector), .4f));
            asset.bodies.push_back(body);
            bones.push_back(Matrix4(OrthogonalTransform(Quaternion(Vector3(kYUnitVector), .2f * i), Vector3(float(i), 2, -1))));
        }
        for (int i = 1; i < 4; ++i)
        {
            PhysicsConstraint joint;
            joint.parentBone = asset.bodies[i - 1].bone;
            joint.childBone = asset.bodies[i].bone;
            joint.SetWorldFrame(bones[i - 1] * asset.bodies[i - 1].GetLocalTransform(),
                bones[i] * asset.bodies[i].GetLocalTransform(), bones[i]);
            // Asymmetric limits intentionally use different parent and child orientations.
            joint.parentFrame = PhysicsJointFrame::FromTransform(joint.parentFrame.GetTransform() *
                Matrix4(Matrix3(Quaternion(Vector3(kZUnitVector), .3f)), Vector3(kZero)));
            asset.constraints.push_back(joint);
        }
        const auto original = asset;
        const auto checkFrames = [&]() {
            for (size_t i = 0; i < asset.constraints.size(); ++i)
            {
                Check(SameFrame(bones[i] * asset.bodies[i].GetLocalTransform() * asset.constraints[i].parentFrame.GetTransform(),
                          bones[i] * original.bodies[i].GetLocalTransform() * original.constraints[i].parentFrame.GetTransform()) &&
                        SameFrame(bones[i + 1] * asset.bodies[i + 1].GetLocalTransform() * asset.constraints[i].childFrame.GetTransform(),
                          bones[i + 1] * original.bodies[i + 1].GetLocalTransform() * original.constraints[i].childFrame.GetTransform()),
                    "Body edits preserve incoming/outgoing joint anchors, axes and asymmetric frame bias");
            }
        };
        auto edited = asset.bodies[1];
        edited.offset = DirectX::XMFLOAT3(-.3f, .15f, .2f);
        DirectX::XMStoreFloat4(&edited.rotation, Quaternion(Vector3(kXUnitVector), .8f) * Quaternion(Vector3(kYUnitVector), -.6f));
        edited.shape = PhysicsShapeType::Box;
        edited.halfExtents = DirectX::XMFLOAT3(.2f, .1f, .05f);
        Check(asset.UpdateBody(1, edited), "Edit collider transform and primitive together");
        Check(asset.bodies[1].shape == PhysicsShapeType::Box && Near(Vector3(asset.bodies[1].offset), Vector3(edited.offset)),
            "Body edit applies the requested shape and offset");
        checkFrames();
        edited.rotation = DirectX::XMFLOAT4(0, 0, 0, 1);
        Check(asset.UpdateBody(1, edited), "Reset collider rotation while preserving joint frames");
        checkFrames();
        Check(asset.UpdateBody(1, original.bodies[1]), "Restore original collider transform");
        checkFrames();
        for (size_t i = 0; i < asset.constraints.size(); ++i)
        {
            Check(SameFrame(asset.constraints[i].parentFrame.GetTransform(), original.constraints[i].parentFrame.GetTransform()) &&
                    SameFrame(asset.constraints[i].childFrame.GetTransform(), original.constraints[i].childFrame.GetTransform()),
                "Restoring a body also restores its joint attachment frames");
        }
        const auto framesBeforeResize = asset.constraints;
        edited = asset.bodies[1];
        edited.radius *= 2;
        edited.mass *= 3;
        Check(asset.UpdateBody(1, edited), "Edit collider dimensions and mass without changing its transform");
        for (size_t i = 0; i < asset.constraints.size(); ++i)
        {
            Check(std::memcmp(&asset.constraints[i].parentFrame, &framesBeforeResize[i].parentFrame, sizeof(PhysicsJointFrame)) == 0 &&
                    std::memcmp(&asset.constraints[i].childFrame, &framesBeforeResize[i].childFrame, sizeof(PhysicsJointFrame)) == 0,
                "Size and mass edits leave joint frames byte-for-byte unchanged");
        }
        Check(!asset.UpdateBody(asset.bodies.size(), edited), "Reject an invalid body edit index");
        edited.bone = "different-bone";
        Check(!asset.UpdateBody(1, edited), "Body editing cannot silently change the attached bone");
        std::puts("PASS: collider editing preserves all connected joint frames and asymmetric limits");
    }
}

void RunPhysicsAssetTests()
{
    TestBodyEditFrames();
    const Matrix4 scaled = Matrix4(OrthogonalTransform(Quaternion(Vector3(kZUnitVector), .7f), Vector3(1, 2, 3))) *
        Matrix4::MakeScale(Vector3(.01f, .02f, .03f));
    const Matrix4 frame = PhysicsBoneFrame(scaled);
    Check(Near(Vector3(frame.GetW()), Vector3(1, 2, 3)), "Physics frame preserves bone position");
    Check(std::abs(float(Length(Vector3(frame.GetX()))) - 1) < 1e-5f &&
        std::abs(float(Length(Vector3(frame.GetY()))) - 1) < 1e-5f &&
        std::abs(float(Dot(Vector3(frame.GetX()), Vector3(frame.GetY())))) < 1e-5f,
        "Physics frame removes import/nonuniform scale");
    PhysicsAsset original;
    for (int i = 0; i < 3; ++i)
    {
        PhysicsBody body;
        body.bone = "mixamorig:Bone" + std::to_string(i);
        body.shape = static_cast<PhysicsShapeType>(i);
        body.offset = DirectX::XMFLOAT3(.1f, .2f, .3f);
        DirectX::XMStoreFloat4(&body.rotation, Quaternion(Vector3(kZUnitVector), .4f));
        body.radius = .12f; body.halfLength = .23f; body.mass = 4.5f;
        original.bodies.push_back(body);
    }
    Check(Near(Vector3((frame * original.bodies[0].GetLocalTransform()).GetW()), Vector3(frame * Vector3(.1f, .2f, .3f))),
        "Body offset is measured in metres, not imported centimetres");
    char directory[MAX_PATH], path[MAX_PATH];
    Check(GetTempPathA(MAX_PATH, directory) && GetTempFileNameA(directory, "PAt", 0, path), "Create temporary PA test file");
    std::string error;
    const Matrix4 parentBody = frame * original.bodies[0].GetLocalTransform();
    const Matrix4 childBone = Matrix4(OrthogonalTransform(Quaternion(Vector3(kYUnitVector), -.8f), Vector3(2, 4, -1))) *
        Matrix4::MakeScale(Vector3(.01f, .01f, .01f));
    const Matrix4 childBody = PhysicsBoneFrame(childBone) * original.bodies[1].GetLocalTransform();
    const Matrix4 jointWorld(OrthogonalTransform(Quaternion(Vector3(kXUnitVector), .3f) *
        Quaternion(Vector3(kZUnitVector), -.5f), Vector3(2, 4, -1)));
    PhysicsConstraint joint;
    joint.parentBone = original.bodies[0].bone; joint.childBone = original.bodies[1].bone;
    joint.SetWorldFrame(parentBody, childBody, jointWorld);
    joint.twistMin = -.65f; joint.twistMax = .8f; joint.swingYLimit = .4f; joint.swingZLimit = .9f;
    joint.swingY = PhysicsJointMotion::Locked; joint.swingZ = PhysicsJointMotion::Free;
    joint.enableCollision = true;
    joint.drives[0] = {true, 135.0f, 12.0f, 28.0f};
    original.constraints.push_back(joint);
    Check(SameFrame(parentBody * joint.parentFrame.GetTransform(), jointWorld) &&
        SameFrame(childBody * joint.childFrame.GetTransform(), jointWorld),
        "Both body-local joint frames reconstruct the same world frame with import scale, offsets and different orientations");
    const Matrix4 global(OrthogonalTransform(Quaternion(Vector3(kYUnitVector), 1.1f), Vector3(-8, 3, 7)));
    Check(SameFrame((global * parentBody) * joint.parentFrame.GetTransform(), global * jointWorld) &&
        SameFrame((global * childBody) * joint.childFrame.GetTransform(), global * jointWorld),
        "Joint attachment is invariant under actor world movement and rotation");
    Check(original.ValidateSkeleton({joint.parentBone, joint.childBone, original.bodies[2].bone}, error), "Compatible skeleton accepts PA");
    Check(!original.ValidateSkeleton({joint.parentBone, joint.childBone}, error), "Missing skeletal joint rejects PA assignment");
    Check(original.Save(path, error), "Save physics asset");
    PhysicsAsset loaded;
    Check(loaded.Load(path, error) && loaded.bodies.size() == 3, "Load all primitive types");
    for (int i = 0; i < 3; ++i)
    {
        const auto& a = original.bodies[i]; const auto& b = loaded.bodies[i];
        Check(a.bone == b.bone && a.shape == b.shape && a.mass == b.mass && a.radius == b.radius && a.halfLength == b.halfLength &&
            Near(Vector3(a.offset), Vector3(b.offset)) && Near(Vector3(a.halfExtents), Vector3(b.halfExtents)) &&
            Near(Quaternion(DirectX::XMLoadFloat4(&a.rotation)) * Vector3(kXUnitVector), Quaternion(DirectX::XMLoadFloat4(&b.rotation)) * Vector3(kXUnitVector)),
            "Round trip preserves attachment, transform, geometry and mass");
    }
    Check(loaded.constraints.size() == 1, "Round trip preserves joint count");
    const auto& savedJoint = loaded.constraints[0];
    Check(savedJoint.drives[0].enabled && savedJoint.drives[0].stiffness == 135 && savedJoint.drives[0].damping == 12 &&
        savedJoint.drives[0].maxTorque == 28 && !savedJoint.drives[0].useAcceleration && !savedJoint.drives[1].enabled,
        "Legacy force settings and disabled axes survive PA v4 round trip");
    Check(savedJoint.parentBone == joint.parentBone && savedJoint.childBone == joint.childBone &&
        SameFrame(savedJoint.parentFrame.GetTransform(), joint.parentFrame.GetTransform()) &&
        SameFrame(savedJoint.childFrame.GetTransform(), joint.childFrame.GetTransform()) &&
        savedJoint.twist == joint.twist && savedJoint.swingY == joint.swingY && savedJoint.swingZ == joint.swingZ &&
        savedJoint.twistMin == joint.twistMin && savedJoint.twistMax == joint.twistMax &&
        savedJoint.swingYLimit == joint.swingYLimit && savedJoint.swingZLimit == joint.swingZLimit && savedJoint.enableCollision,
        "Joint round trip preserves frames, radians, all motion modes and collision setting");
    Check(loaded.FindConstraint(joint.childBone, joint.parentBone) == 0, "Joint pair lookup ignores order");
    PhysicsAsset invalid = loaded;
    invalid.constraints[0].drives[0].maxTorque = -1;
    Check(!invalid.Validate(error), "Reject negative drive torque");
    invalid = loaded; invalid.constraints[0].drives[0].stiffness = std::numeric_limits<float>::quiet_NaN();
    Check(!invalid.Validate(error), "Reject non-finite drive gains");
    invalid = loaded; invalid.constraints[0].useSlerpDrive = true;
    Check(!invalid.Validate(error), "Reject SLERP with a locked angular axis");
    auto slerp = loaded;
    slerp.constraints[0].swingY = PhysicsJointMotion::Limited;
    slerp.constraints[0].useSlerpDrive = true;
    PhysicsAsset slerpLoaded;
    Check(slerp.Save(path, error) && slerpLoaded.Load(path, error) && slerpLoaded.constraints[0].useSlerpDrive,
        "SLERP drive mode survives PA v4 round trip");
    auto acceleration = original;
    ConfigureDefaultDrives(acceleration);
    acceleration.constraints[0].drives[0].naturalFrequency = 24.0f;
    acceleration.constraints[0].drives[0].dampingRatio = .75f;
    PhysicsAsset accelerationLoaded;
    Check(acceleration.Save(path, error) && accelerationLoaded.Load(path, error), "Round trip acceleration drives");
    const auto& accelerationDrive = accelerationLoaded.constraints[0].drives[0];
    Check(accelerationDrive.useAcceleration && accelerationDrive.naturalFrequency == 24.0f &&
        accelerationDrive.dampingRatio == .75f && accelerationDrive.GetStiffness() == 576.0f &&
        accelerationDrive.GetDamping() == 36.0f && !accelerationLoaded.constraints[0].drives[1].enabled,
        "Acceleration drive preserves frequency, ratio and locked axes without interpreting torque gains");
    auto heavier = original;
    for (auto& body : heavier.bodies)
    {
        body.mass *= 3.0f;
    }
    ConfigureDefaultDrives(heavier);
    ConfigureDefaultDrives(acceleration);
    Check(heavier.constraints[0].drives[0].GetStiffness() == acceleration.constraints[0].drives[0].GetStiffness() &&
        heavier.constraints[0].drives[0].GetDamping() == acceleration.constraints[0].drives[0].GetDamping() &&
        heavier.constraints[0].drives[0].maxTorque > acceleration.constraints[0].drives[0].maxTorque,
        "Mass changes torque capacity but not the requested acceleration response");
    for (float frequency : {-1.0f, std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::max()})
    {
        auto badResponse = acceleration;
        badResponse.constraints[0].drives[0].naturalFrequency = frequency;
        Check(!badResponse.Validate(error), "Reject negative, non-finite or overflowing acceleration frequency");
    }
    auto badResponse = acceleration;
    badResponse.constraints[0].drives[0].dampingRatio = -1.0f;
    Check(!badResponse.Validate(error), "Reject negative damping ratio");
    badResponse.constraints[0].drives[0].dampingRatio = std::numeric_limits<float>::infinity();
    Check(!badResponse.Validate(error), "Reject non-finite damping ratio");
    Check(original.Save(path, error), "Restore fixture for legacy format tests");
    invalid = loaded;
    invalid.constraints[0].childBone = "missing";
    Check(!invalid.Validate(error), "Reject dangling joint body reference");
    invalid = loaded; invalid.constraints[0].childBone = joint.parentBone;
    Check(!invalid.Validate(error), "Reject joint connecting a body to itself");
    invalid = loaded; invalid.constraints.push_back(joint);
    std::swap(invalid.constraints[1].parentBone, invalid.constraints[1].childBone);
    Check(!invalid.Validate(error), "Reject duplicate reversed joint pair");
    invalid = loaded; invalid.constraints[0].parentFrame.rotation = DirectX::XMFLOAT4(0, 0, 0, 0);
    Check(!invalid.Validate(error), "Reject invalid joint frame quaternion");
    invalid = loaded; invalid.constraints[0].childFrame.offset.y = std::numeric_limits<float>::infinity();
    Check(!invalid.Validate(error), "Reject non-finite joint offset");
    invalid = loaded; invalid.constraints[0].twist = static_cast<PhysicsJointMotion>(99);
    Check(!invalid.Validate(error), "Reject invalid joint motion enum");
    invalid = loaded; invalid.constraints[0].twistMin = invalid.constraints[0].twistMax;
    Check(!invalid.Save(path, error) && loaded.Load(path, error) && loaded.constraints[0].twistMin == joint.twistMin,
        "Invalid twist range cannot overwrite existing joint file");
    for (float angle : {0.0f, DirectX::XM_PI, std::numeric_limits<float>::quiet_NaN()})
    {
        invalid = loaded; invalid.constraints[0].swingYLimit = angle;
        Check(!invalid.Validate(error), "Reject zero, pi and non-finite swing limits");
    }
    invalid = loaded;
    PhysicsConstraint second = joint; second.parentBone = joint.childBone; second.childBone = invalid.bodies[2].bone;
    invalid.constraints.push_back(second);
    invalid.RemoveBody(1);
    Check(invalid.bodies.size() == 2 && invalid.constraints.empty() && invalid.Validate(error),
        "Deleting the middle body removes both attached joints without dangling references");
    nlohmann::json saved;
    { std::ifstream input(path); input >> saved; }
    const auto writeJson = [&](const nlohmann::json& value) { std::ofstream output(path); output << value.dump(); };
    auto corruptJoint = saved;
    corruptJoint["constraints"][0]["childBone"] = "unknown";
    writeJson(corruptJoint);
    Check(!loaded.Load(path, error) && loaded.bodies.size() == 3 && loaded.constraints.size() == 1 &&
        loaded.constraints[0].childBone == joint.childBone, "Rejected joint load preserves bodies and joints together");
    corruptJoint = saved; corruptJoint["angleUnits"] = "degrees";
    writeJson(corruptJoint);
    Check(!loaded.Load(path, error) && loaded.constraints.size() == 1, "Reject ambiguous angular units");
    corruptJoint = saved;
    corruptJoint["constraints"][0]["drives"][0]["type"] = "unknown";
    writeJson(corruptJoint);
    Check(!loaded.Load(path, error) && !loaded.constraints[0].drives[0].useAcceleration,
        "Reject unknown drive type without changing the live asset");
    corruptJoint = saved;
    corruptJoint["constraints"][0]["drives"][0]["type"] = "acceleration";
    writeJson(corruptJoint);
    Check(!loaded.Load(path, error), "Acceleration files must supply frequency and ratio instead of legacy gains");
    auto legacyForce = saved;
    legacyForce["version"] = 3;
    legacyForce["driveUnits"] = "Nm-radians-seconds";
    for (auto& entry : legacyForce["constraints"])
    {
        for (auto& drive : entry["drives"])
        {
            drive.erase("type");
        }
    }
    writeJson(legacyForce);
    Check(loaded.Load(path, error) && !loaded.constraints[0].drives[0].useAcceleration &&
        loaded.constraints[0].drives[0].GetStiffness() == 135.0f && loaded.constraints[0].drives[0].GetDamping() == 12.0f,
        "Legacy v3 torque gains remain force drives rather than becoming acceleration gains");
    auto legacy = saved; legacy["version"] = 2; legacy.erase("driveUnits");
    for (auto& entry : legacy["constraints"]) entry.erase("drives");
    writeJson(legacy);
    Check(loaded.Load(path, error) && !loaded.constraints[0].drives[0].enabled, "Legacy v2 PA loads with angular drives disabled");
    legacy["version"] = 1; legacy.erase("constraints"); legacy.erase("angleUnits");
    writeJson(legacy);
    Check(loaded.Load(path, error) && loaded.bodies.size() == 3 && loaded.constraints.empty(),
        "Legacy version 1 body-only file loads and clears previous joints");
    writeJson(saved);
    Check(loaded.Load(path, error), "Restore version 4 asset");
    original.bodies[0].radius = -1;
    Check(!original.Save(path, error), "Reject negative radius before overwriting saved asset");
    Check(loaded.Load(path, error) && loaded.bodies[0].radius == .12f, "Failed save preserves previous file");
    original = loaded;
    original.bodies[0].offset.x = std::numeric_limits<float>::quiet_NaN();
    Check(!original.Validate(error), "Reject non-finite collider transforms");
    original = loaded; original.bodies[1].bone = original.bodies[0].bone;
    Check(!original.Validate(error), "Reject duplicate bone bodies");
    original = loaded; original.bodies[0].rotation.w = 0;
    Check(!original.Validate(error), "Reject invalid rotation");
    { std::ofstream corrupt(path); corrupt << "{\"version\":99,\"units\":\"metres\",\"bodies\":[]}"; }
    Check(!loaded.Load(path, error) && loaded.bodies.size() == 3, "Unsupported asset version preserves editor state");
    { std::ofstream corrupt(path); corrupt << "{broken json"; }
    Check(!loaded.Load(path, error) && loaded.bodies.size() == 3, "Malformed asset preserves editor state");
    DeleteFileA(path);
    std::puts("PASS: PhysicsAsset transforms, limits, v1/v2/v3 compatibility, v4 acceleration response and transactional validation");
}
