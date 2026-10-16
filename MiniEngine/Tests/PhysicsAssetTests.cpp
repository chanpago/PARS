#include "../Model/PhysicsAsset.h"
#include <Windows.h>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <limits>

using namespace Math;
using namespace PARS;

namespace
{
    void Check(bool condition, const char* message)
    {
        if (!condition) { std::fprintf(stderr, "FAIL: %s\n", message); std::exit(1); }
    }
    bool Near(Vector3 a, Vector3 b) { return float(Length(a - b)) < 1e-5f; }
}

void RunPhysicsAssetTests()
{
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
    std::puts("PASS: PhysicsAsset units, rigid bone frames, primitive round trip and transactional validation");
}
