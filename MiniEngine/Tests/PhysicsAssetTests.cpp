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
#include <functional>
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
        const PARS::PhysicsAsset original = asset;
        const std::function<void()> checkFrames = [&]() -> void
        {
            for (size_t i = 0; i < asset.constraints.size(); ++i)
            {
                Check(SameFrame(bones[i] * asset.bodies[i].GetLocalTransform() * asset.constraints[i].parentFrame.GetTransform(),
                          bones[i] * original.bodies[i].GetLocalTransform() * original.constraints[i].parentFrame.GetTransform()) &&
                        SameFrame(bones[i + 1] * asset.bodies[i + 1].GetLocalTransform() * asset.constraints[i].childFrame.GetTransform(),
                            bones[i + 1] * original.bodies[i + 1].GetLocalTransform() * original.constraints[i].childFrame.GetTransform()),
                    "Body edits preserve incoming/outgoing joint anchors, axes and asymmetric frame bias");
            }
        };
        PARS::PhysicsBody edited = asset.bodies[1];
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
        const std::vector<PARS::PhysicsConstraint> framesBeforeResize = asset.constraints;
        edited = asset.bodies[1];
        edited.radius *= 2;
        edited.mass *= 3;
        edited.material.staticFriction = .1f;
        edited.material.dynamicFriction = .05f;
        edited.material.frictionCombine = PhysicsFrictionCombine::Minimum;
        Check(asset.UpdateBody(1, edited), "Edit collider dimensions and mass without changing its transform");
        Check(asset.bodies[1].material.staticFriction == .1f &&
            asset.bodies[1].material.frictionCombine == PhysicsFrictionCombine::Minimum, "Body edits retain material settings");
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
        if (i == 1)
        {
            body.material.staticFriction = .1f;
            body.material.dynamicFriction = .05f;
            body.material.restitution = 0;
            body.material.frictionCombine = PhysicsFrictionCombine::Minimum;
        }
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
    joint.twistMin = -.65f;
    joint.twistMax = .8f;
    joint.swingYLimit = .4f;
    joint.swingZMin = -.35f;
    joint.swingZMax = .9f;
    joint.swingY = PhysicsJointMotion::Locked; joint.swingZ = PhysicsJointMotion::Free;
    joint.enableCollision = true;
    joint.drives[0] = PhysicsAngularDrive{};
    joint.drives[0].enabled = true;
    joint.drives[0].stiffness = 135.0f;
    joint.drives[0].damping = 12.0f;
    joint.drives[0].maxTorque = 28.0f;
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
        const PARS::PhysicsBody& a = original.bodies[i];
        const PARS::PhysicsBody& b = loaded.bodies[i];
        Check(a.bone == b.bone && a.shape == b.shape && a.mass == b.mass && a.radius == b.radius && a.halfLength == b.halfLength &&
            Near(Vector3(a.offset), Vector3(b.offset)) && Near(Vector3(a.halfExtents), Vector3(b.halfExtents)) &&
            Near(Quaternion(DirectX::XMLoadFloat4(&a.rotation)) * Vector3(kXUnitVector), Quaternion(DirectX::XMLoadFloat4(&b.rotation)) * Vector3(kXUnitVector)),
            "Round trip preserves attachment, transform, geometry and mass");
        Check(a.material.staticFriction == b.material.staticFriction && a.material.dynamicFriction == b.material.dynamicFriction &&
            a.material.restitution == b.material.restitution && a.material.frictionCombine == b.material.frictionCombine,
            "Per-body friction, restitution and combine mode survive a PA round trip");
    }
    Check(loaded.constraints.size() == 1, "Round trip preserves joint count");
    const PARS::PhysicsConstraint& savedJoint = loaded.constraints[0];
    Check(savedJoint.drives[0].enabled && savedJoint.drives[0].stiffness == 135 && savedJoint.drives[0].damping == 12 &&
        savedJoint.drives[0].maxTorque == 28 && !savedJoint.drives[0].useAcceleration && !savedJoint.drives[1].enabled,
        "Legacy force settings and disabled axes survive PA v4 round trip");
    Check(savedJoint.parentBone == joint.parentBone && savedJoint.childBone == joint.childBone &&
        SameFrame(savedJoint.parentFrame.GetTransform(), joint.parentFrame.GetTransform()) &&
        SameFrame(savedJoint.childFrame.GetTransform(), joint.childFrame.GetTransform()) &&
        savedJoint.twist == joint.twist && savedJoint.swingY == joint.swingY && savedJoint.swingZ == joint.swingZ &&
        savedJoint.twistMin == joint.twistMin && savedJoint.twistMax == joint.twistMax &&
        savedJoint.swingYLimit == joint.swingYLimit && savedJoint.swingZMin == joint.swingZMin &&
        savedJoint.swingZMax == joint.swingZMax && savedJoint.enableCollision,
        "Joint round trip preserves frames, radians, all motion modes and collision setting");
    Check(loaded.FindConstraint(joint.childBone, joint.parentBone) == 0, "Joint pair lookup ignores order");
    PhysicsAsset invalid = loaded;
    for (float value : {-1.0f, std::numeric_limits<float>::infinity(), std::numeric_limits<float>::quiet_NaN()})
    {
        invalid = loaded; invalid.bodies[1].material.staticFriction = value;
        Check(!invalid.Validate(error), "Reject invalid static friction");
        invalid = loaded; invalid.bodies[1].material.dynamicFriction = value;
        Check(!invalid.Validate(error), "Reject invalid dynamic friction");
        invalid = loaded; invalid.bodies[1].material.restitution = value;
        Check(!invalid.Validate(error), "Reject invalid restitution");
    }
    invalid = loaded; invalid.bodies[1].material.restitution = 1.1f;
    Check(!invalid.Save(path, error), "Restitution above one cannot overwrite a saved PA");
    invalid = loaded; invalid.bodies[1].material.frictionCombine = static_cast<PhysicsFrictionCombine>(99);
    Check(!invalid.Validate(error), "Reject unknown friction combine enum");
    invalid = loaded;
    invalid.constraints[0].drives[0].maxTorque = -1;
    Check(!invalid.Validate(error), "Reject negative drive torque");
    invalid = loaded; invalid.constraints[0].drives[0].stiffness = std::numeric_limits<float>::quiet_NaN();
    Check(!invalid.Validate(error), "Reject non-finite drive gains");
    invalid = loaded; invalid.constraints[0].useSlerpDrive = true;
    Check(!invalid.Validate(error), "Reject SLERP with a locked angular axis");
    PARS::PhysicsAsset slerp = loaded;
    slerp.constraints[0].swingY = PhysicsJointMotion::Limited;
    slerp.constraints[0].useSlerpDrive = true;
    PhysicsAsset slerpLoaded;
    Check(slerp.Save(path, error) && slerpLoaded.Load(path, error) && slerpLoaded.constraints[0].useSlerpDrive,
        "SLERP drive mode survives PA v4 round trip");
    PARS::PhysicsAsset acceleration = original;
    ConfigureDefaultDrives(acceleration);
    acceleration.constraints[0].drives[0].naturalFrequency = 24.0f;
    acceleration.constraints[0].drives[0].dampingRatio = .75f;
    PhysicsAsset accelerationLoaded;
    Check(acceleration.Save(path, error) && accelerationLoaded.Load(path, error), "Round trip acceleration drives");
    const PARS::PhysicsAngularDrive& accelerationDrive = accelerationLoaded.constraints[0].drives[0];
    Check(accelerationDrive.useAcceleration && accelerationDrive.naturalFrequency == 24.0f &&
        accelerationDrive.dampingRatio == .75f && accelerationDrive.GetStiffness() == 576.0f &&
        accelerationDrive.GetDamping() == 36.0f && !accelerationLoaded.constraints[0].drives[1].enabled,
        "Acceleration drive preserves frequency, ratio and locked axes without interpreting torque gains");
    PARS::PhysicsAsset heavier = original;
    for (PARS::PhysicsBody& body : heavier.bodies)
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
        PARS::PhysicsAsset badResponse = acceleration;
        badResponse.constraints[0].drives[0].naturalFrequency = frequency;
        Check(!badResponse.Validate(error), "Reject negative, non-finite or overflowing acceleration frequency");
    }
    PARS::PhysicsAsset badResponse = acceleration;
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
    for (float angle : {-DirectX::XM_PI, DirectX::XM_PI, std::numeric_limits<float>::infinity(),
             std::numeric_limits<float>::quiet_NaN()})
    {
        invalid = loaded;
        invalid.constraints[0].swingZMin = angle;
        Check(!invalid.Validate(error), "Reject invalid Z minimum");
        invalid = loaded;
        invalid.constraints[0].swingZMax = angle;
        Check(!invalid.Validate(error), "Reject invalid Z maximum");
    }
    invalid = loaded;
    invalid.constraints[0].swingZMin = invalid.constraints[0].swingZMax;
    Check(!invalid.Validate(error), "Reject equal Z endpoints");
    invalid.constraints[0].swingZMin += .1f;
    Check(!invalid.Validate(error), "Reject reversed Z endpoints");
    invalid = loaded;
    invalid.constraints[0].swingZMin = .1f;
    invalid.constraints[0].swingZMax = .3f;
    Check(invalid.Validate(error), "Z endpoints may both lie on the same side of zero");
    invalid.constraints[0].swingY = PhysicsJointMotion::Free;
    invalid.constraints[0].swingZ = PhysicsJointMotion::Limited;
    Check(!invalid.Validate(error), "D6 rejects an asymmetric Z limit combined with free Y");
    invalid.constraints[0].drives[0].useAcceleration = true;
    Check(invalid.Validate(error), "Articulation supports asymmetric Z with free Y");
    invalid = loaded;
    PhysicsConstraint second = joint; second.parentBone = joint.childBone; second.childBone = invalid.bodies[2].bone;
    invalid.constraints.push_back(second);
    invalid.RemoveBody(1);
    Check(invalid.bodies.size() == 2 && invalid.constraints.empty() && invalid.Validate(error),
        "Deleting the middle body removes both attached joints without dangling references");
    nlohmann::json saved;
    { std::ifstream input(path); input >> saved; }
    const std::function<void(const nlohmann::json&)> writeJson = [&](const nlohmann::json& value) -> void
    {
        std::ofstream output(path);
        output << value.dump();
    };
    Check(saved.at("version").get<int>() == 6 && !saved["constraints"][0].contains("swingZLimit"),
        "PA version 6 stores separate Z endpoints without a stale symmetric limit");
    nlohmann::json legacyLimits = saved;
    legacyLimits["version"] = 5;
    for (nlohmann::json& entry : legacyLimits["constraints"])
    {
        entry["swingZLimit"] = entry.at("swingZMax");
        entry.erase("swingZMin");
        entry.erase("swingZMax");
    }
    writeJson(legacyLimits);
    Check(loaded.Load(path, error) && loaded.constraints[0].swingZMin == -.9f && loaded.constraints[0].swingZMax == .9f,
        "Legacy v5 Z half-angle loads with exactly the same symmetric bounds");
    nlohmann::json missingZ = saved;
    missingZ["constraints"][0].erase("swingZMin");
    writeJson(missingZ);
    Check(!loaded.Load(path, error) && loaded.constraints[0].swingZMin == -.9f,
        "A missing v6 Z endpoint rejects the load without changing the existing PA");
    missingZ = saved;
    missingZ["constraints"][0]["swingZMin"] = 1.0f;
    writeJson(missingZ);
    Check(!loaded.Load(path, error) && loaded.constraints[0].swingZMax == .9f,
        "Reversed v6 Z bounds cannot replace the existing PA");
    writeJson(saved);
    Check(loaded.Load(path, error) && loaded.constraints[0].swingZMin == -.35f,
        "Restore the asymmetric Z interval after legacy and invalid loads");
    nlohmann::json corruptMaterial = saved;
    corruptMaterial["bodies"][1]["material"]["frictionCombine"] = "unknown";
    writeJson(corruptMaterial);
    Check(!loaded.Load(path, error) && loaded.bodies[1].material.frictionCombine == PhysicsFrictionCombine::Minimum,
        "Invalid material load preserves the existing PA");
    corruptMaterial = saved;
    corruptMaterial["bodies"][1]["material"]["restitution"] = 2;
    writeJson(corruptMaterial);
    Check(!loaded.Load(path, error) && loaded.bodies[1].material.restitution == 0, "Invalid material values reject the whole load");
    nlohmann::json defaultMaterial = saved;
    defaultMaterial["bodies"][1].erase("material");
    writeJson(defaultMaterial);
    PhysicsAsset defaultsLoaded;
    Check(defaultsLoaded.Load(path, error) && defaultsLoaded.bodies[1].material.staticFriction == .6f &&
        defaultsLoaded.bodies[1].material.dynamicFriction == .5f && defaultsLoaded.bodies[1].material.restitution == .1f &&
        defaultsLoaded.bodies[1].material.frictionCombine == PhysicsFrictionCombine::Average,
        "Omitted material uses the original shared material values");
    nlohmann::json legacyMaterial = legacyLimits;
    legacyMaterial["version"] = 4;
    for (nlohmann::json& body : legacyMaterial["bodies"])
    {
        body.erase("material");
    }
    writeJson(legacyMaterial);
    Check(defaultsLoaded.Load(path, error) && defaultsLoaded.bodies[1].material.staticFriction == .6f &&
        defaultsLoaded.bodies[1].material.dynamicFriction == .5f && defaultsLoaded.bodies[1].material.restitution == .1f &&
        defaultsLoaded.bodies[1].material.frictionCombine == PhysicsFrictionCombine::Average &&
        defaultsLoaded.constraints[0].drives[0].stiffness == 135, "Existing v4 PAs retain material and drive behavior");
    nlohmann::json corruptJoint = saved;
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
    nlohmann::json legacyForce = legacyLimits;
    legacyForce["version"] = 3;
    legacyForce["driveUnits"] = "Nm-radians-seconds";
    for (nlohmann::json& entry : legacyForce["constraints"])
    {
        for (nlohmann::json& drive : entry["drives"])
        {
            drive.erase("type");
        }
    }
    writeJson(legacyForce);
    Check(loaded.Load(path, error) && !loaded.constraints[0].drives[0].useAcceleration &&
        loaded.constraints[0].drives[0].GetStiffness() == 135.0f && loaded.constraints[0].drives[0].GetDamping() == 12.0f,
        "Legacy v3 torque gains remain force drives rather than becoming acceleration gains");
    nlohmann::json legacy = legacyLimits;
    legacy["version"] = 2;
    legacy.erase("driveUnits");
    for (nlohmann::json& entry : legacy["constraints"])
    {
        entry.erase("drives");
    }
    writeJson(legacy);
    Check(loaded.Load(path, error) && !loaded.constraints[0].drives[0].enabled, "Legacy v2 PA loads with angular drives disabled");
    legacy["version"] = 1; legacy.erase("constraints"); legacy.erase("angleUnits");
    writeJson(legacy);
    Check(loaded.Load(path, error) && loaded.bodies.size() == 3 && loaded.constraints.empty(),
        "Legacy version 1 body-only file loads and clears previous joints");
    writeJson(saved);
    Check(loaded.Load(path, error), "Restore version 6 asset");
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
