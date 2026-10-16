#include "PhysicsAsset.h"
#pragma warning(push)
#pragma warning(disable: 4100) // Vendored JSON header has an unused diagnostic parameter.
#include "json.hpp"
#pragma warning(pop)
#include <Windows.h>
#include <fstream>
#include <cmath>
#include <set>

using namespace Math;
using nlohmann::json;

namespace
{
    std::wstring WidePath(const std::string& path)
    {
        const int count = MultiByteToWideChar(CP_UTF8, MB_ERR_INVALID_CHARS, path.c_str(), -1, nullptr, 0);
        if (!count) throw std::runtime_error("Invalid UTF-8 file path.");
        std::wstring result(count, L'\0');
        MultiByteToWideChar(CP_UTF8, MB_ERR_INVALID_CHARS, path.c_str(), -1, &result[0], count);
        result.resize(count - 1);
        return result;
    }
}

namespace PARS
{
    Matrix4 PhysicsBody::GetLocalTransform() const
    {
        return Matrix4(OrthogonalTransform(Quaternion(DirectX::XMLoadFloat4(&rotation)), Vector3(offset)));
    }

    Matrix4 PhysicsBoneFrame(const Matrix4& boneWorld)
    {
        // Gram-Schmidt also removes shear caused by parent nonuniform scale.
        Vector3 x(boneWorld.GetX()), y(boneWorld.GetY());
        x = LengthSquare(x) > 1e-12f ? Normalize(x) : Vector3(kXUnitVector);
        y -= x * Dot(x, y);
        if (LengthSquare(y) < 1e-12f)
            y = Cross(std::abs(float(x.GetY())) < .9f ? Vector3(kYUnitVector) : Vector3(kZUnitVector), x);
        y = Normalize(y);
        return Matrix4(x, y, Normalize(Cross(x, y)), Vector3(boneWorld.GetW()));
    }

    int PhysicsAsset::FindBody(const std::string& bone) const
    {
        for (size_t i = 0; i < bodies.size(); ++i)
            if (bodies[i].bone == bone) return static_cast<int>(i);
        return -1;
    }

    bool PhysicsAsset::Validate(std::string& error) const
    {
        error.clear();
        std::set<std::string> names;
        if (bodies.size() > 4096) { error = "Too many physics bodies."; return false; }
        for (const auto& body : bodies)
        {
            const float values[] = {body.offset.x, body.offset.y, body.offset.z,
                body.rotation.x, body.rotation.y, body.rotation.z, body.rotation.w,
                body.halfExtents.x, body.halfExtents.y, body.halfExtents.z,
                body.radius, body.halfLength, body.mass};
            for (float value : values)
                if (!std::isfinite(value)) { error = "Body contains a non-finite value."; return false; }
            const float norm = DirectX::XMVectorGetX(DirectX::XMVector4LengthSq(DirectX::XMLoadFloat4(&body.rotation)));
            if (body.bone.empty() || !names.insert(body.bone).second)
                error = "Bodies need unique, nonempty bone names.";
            else if (body.shape != PhysicsShapeType::Capsule && body.shape != PhysicsShapeType::Box && body.shape != PhysicsShapeType::Sphere)
                error = "Unknown collision shape.";
            else if (body.radius <= 0 || body.halfLength < 0 || body.mass <= 0 ||
                body.halfExtents.x <= 0 || body.halfExtents.y <= 0 || body.halfExtents.z <= 0)
                error = "Body dimensions and mass must be positive.";
            else if (std::abs(norm - 1.0f) > .001f) error = "Body rotation must be a unit quaternion.";
            if (!error.empty()) return false;
        }
        return true;
    }

    bool PhysicsAsset::Save(const std::string& path, std::string& error) const
    {
        if (!Validate(error)) return false;
        std::wstring temporary;
        try
        {
            const auto destination = WidePath(path);
            temporary = destination + L".tmp";
            json root;
            root["version"] = 1;
            root["units"] = "metres";
            root["bodies"] = json::array();
            for (const auto& body : bodies)
            {
                json entry;
                entry["bone"] = body.bone;
                entry["shape"] = body.shape == PhysicsShapeType::Capsule ? "capsule" : body.shape == PhysicsShapeType::Box ? "box" : "sphere";
                entry["offset"] = {body.offset.x, body.offset.y, body.offset.z};
                entry["rotation"] = {body.rotation.x, body.rotation.y, body.rotation.z, body.rotation.w};
                entry["halfExtents"] = {body.halfExtents.x, body.halfExtents.y, body.halfExtents.z};
                entry["radius"] = body.radius;
                entry["halfLength"] = body.halfLength;
                entry["mass"] = body.mass;
                root["bodies"].push_back(entry);
            }
            std::ofstream output(temporary, std::ios::binary | std::ios::trunc);
            output << root.dump(2) << '\n';
            output.close();
            if (!output || !MoveFileExW(temporary.c_str(), destination.c_str(), MOVEFILE_REPLACE_EXISTING | MOVEFILE_WRITE_THROUGH))
                throw std::runtime_error("Could not write PhysicsAsset file (check path and permissions).");
            return true;
        }
        catch (const std::exception& exception) { error = exception.what(); if (!temporary.empty()) DeleteFileW(temporary.c_str()); return false; }
    }

    bool PhysicsAsset::Load(const std::string& path, std::string& error)
    {
        try
        {
            std::ifstream input(WidePath(path), std::ios::binary);
            if (!input) throw std::runtime_error("Could not open PhysicsAsset file.");
            json root; input >> root;
            if (root.at("version").get<int>() != 1 || root.at("units").get<std::string>() != "metres")
                throw std::runtime_error("Unsupported PhysicsAsset version or units.");
            const auto& entries = root.at("bodies");
            if (!entries.is_array() || entries.size() > 4096) throw std::runtime_error("Invalid body list.");
            PhysicsAsset candidate;
            for (const auto& entry : entries)
            {
                PhysicsBody body;
                body.bone = entry.at("bone").get<std::string>();
                const auto shape = entry.at("shape").get<std::string>();
                if (shape != "capsule" && shape != "box" && shape != "sphere") throw std::runtime_error("Unknown collision shape.");
                body.shape = shape == "capsule" ? PhysicsShapeType::Capsule : shape == "box" ? PhysicsShapeType::Box : PhysicsShapeType::Sphere;
                const auto read = [&](const char* key, float* values, size_t count) {
                    const auto& array = entry.at(key);
                    if (!array.is_array() || array.size() != count) throw std::runtime_error("Invalid body transform/dimensions.");
                    for (size_t i = 0; i < count; ++i) values[i] = array.at(i).get<float>();
                };
                read("offset", &body.offset.x, 3);
                read("rotation", &body.rotation.x, 4);
                read("halfExtents", &body.halfExtents.x, 3);
                body.radius = entry.at("radius").get<float>();
                body.halfLength = entry.at("halfLength").get<float>();
                body.mass = entry.at("mass").get<float>();
                candidate.bodies.push_back(body);
            }
            if (!candidate.Validate(error)) return false;
            bodies.swap(candidate.bodies);
            return true;
        }
        catch (const std::exception& exception) { error = exception.what(); return false; }
    }
}
