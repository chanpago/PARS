#include "PhysicsAsset.h"
#pragma warning(push)
#pragma warning(disable : 4100) // 저장소에 포함된 JSON 헤더에는 사용하지 않는 진단용 매개변수가 있다.
#include "json.hpp"
#pragma warning(pop)
#include <Windows.h>
#include <fstream>
#include <cmath>
#include <set>
#include <algorithm>

using namespace Math;
using nlohmann::json;

namespace
{
    std::wstring WidePath(const std::string& path)
    {
        const int count = MultiByteToWideChar(CP_UTF8, MB_ERR_INVALID_CHARS, path.c_str(), -1, nullptr, 0);
        if (!count)
        {
            throw std::runtime_error("Invalid UTF-8 file path.");
        }
        std::wstring result(count, L'\0');
        MultiByteToWideChar(CP_UTF8, MB_ERR_INVALID_CHARS, path.c_str(), -1, &result[0], count);
        result.resize(count - 1);
        return result;
    }
    bool ValidFrame(const PARS::PhysicsJointFrame& frame)
    {
        const float values[] = {frame.offset.x, frame.offset.y, frame.offset.z,
            frame.rotation.x, frame.rotation.y, frame.rotation.z, frame.rotation.w};
        for (float value : values)
        {
            if (!std::isfinite(value))
            {
                return false;
            }
        }
        return std::abs(DirectX::XMVectorGetX(DirectX::XMVector4LengthSq(DirectX::XMLoadFloat4(&frame.rotation))) - 1) <= .001f;
    }

    const char* MotionName(PARS::PhysicsJointMotion motion)
    {
        return motion == PARS::PhysicsJointMotion::Locked ? "locked" : motion == PARS::PhysicsJointMotion::Limited ? "limited"
                                                                                                                   : "free";
    }

    PARS::PhysicsJointMotion ReadMotion(const json& value)
    {
        const auto name = value.get<std::string>();
        if (name == "locked")
        {
            return PARS::PhysicsJointMotion::Locked;
        }
        if (name == "limited")
        {
            return PARS::PhysicsJointMotion::Limited;
        }
        if (name == "free")
        {
            return PARS::PhysicsJointMotion::Free;
        }
        throw std::runtime_error("Unknown joint motion.");
    }

    json WriteFrame(const PARS::PhysicsJointFrame& frame)
    {
        json result;
        result["offset"] = {frame.offset.x, frame.offset.y, frame.offset.z};
        result["rotation"] = {frame.rotation.x, frame.rotation.y, frame.rotation.z, frame.rotation.w};
        return result;
    }

    PARS::PhysicsJointFrame ReadFrame(const json& value)
    {
        const auto& p = value.at("offset");
        const auto& q = value.at("rotation");
        if (!p.is_array() || p.size() != 3 || !q.is_array() || q.size() != 4)
        {
            throw std::runtime_error("Invalid joint frame.");
        }
        PARS::PhysicsJointFrame result;
        result.offset = DirectX::XMFLOAT3(p.at(0).get<float>(), p.at(1).get<float>(), p.at(2).get<float>());
        result.rotation = DirectX::XMFLOAT4(q.at(0).get<float>(), q.at(1).get<float>(), q.at(2).get<float>(), q.at(3).get<float>());
        return result;
    }
}

namespace PARS
{
    bool PhysicsAsset::UsesArticulation() const
    {
        for (const auto& joint : constraints)
        {
            for (const auto& drive : joint.drives)
            {
                if (drive.useAcceleration)
                {
                    return true;
                }
            }
        }
        return false;
    }

    float PhysicsAngularDrive::GetStiffness() const
    {
        return useAcceleration ? naturalFrequency * naturalFrequency : stiffness;
    }

    float PhysicsAngularDrive::GetDamping() const
    {
        return useAcceleration ? 2.0f * dampingRatio * naturalFrequency : damping;
    }

    Matrix4 PhysicsBody::GetLocalTransform() const
    {
        return Matrix4(OrthogonalTransform(Quaternion(DirectX::XMLoadFloat4(&rotation)), Vector3(offset)));
    }

    Matrix4 PhysicsJointFrame::GetTransform() const
    {
        return Matrix4(OrthogonalTransform(Quaternion(DirectX::XMLoadFloat4(&rotation)), Vector3(offset)));
    }

    PhysicsJointFrame PhysicsJointFrame::FromTransform(const Matrix4& rigidTransform)
    {
        PhysicsJointFrame frame;
        DirectX::XMStoreFloat3(&frame.offset, rigidTransform.GetW());
        DirectX::XMStoreFloat4(&frame.rotation, Normalize(Quaternion(rigidTransform)));
        return frame;
    }

    void PhysicsConstraint::SetWorldFrame(const Matrix4& parentBodyWorld, const Matrix4& childBodyWorld, const Matrix4& jointWorld)
    {
        parentFrame = PhysicsJointFrame::FromTransform(Invert(parentBodyWorld) * jointWorld);
        childFrame = PhysicsJointFrame::FromTransform(Invert(childBodyWorld) * jointWorld);
    }

    Matrix4 PhysicsBoneFrame(const Matrix4& boneWorld)
    {
        // Gram-Schmidt 직교화로 부모의 비균일 스케일에서 생기는 전단도 제거한다.
        Vector3 x(boneWorld.GetX()), y(boneWorld.GetY());
        x = LengthSquare(x) > 1e-12f ? Normalize(x) : Vector3(kXUnitVector);
        y -= x * Dot(x, y);
        if (LengthSquare(y) < 1e-12f)
        {
            y = Cross(std::abs(float(x.GetY())) < .9f ? Vector3(kYUnitVector) : Vector3(kZUnitVector), x);
        }
        y = Normalize(y);
        return Matrix4(x, y, Normalize(Cross(x, y)), Vector3(boneWorld.GetW()));
    }

    int PhysicsAsset::FindBody(const std::string& bone) const
    {
        for (size_t i = 0; i < bodies.size(); ++i)
        {
            if (bodies[i].bone == bone)
            {
                return static_cast<int>(i);
            }
        }
        return -1;
    }

    int PhysicsAsset::FindConstraint(const std::string& boneA, const std::string& boneB) const
    {
        for (size_t i = 0; i < constraints.size(); ++i)
        {
            const auto& joint = constraints[i];
            if ((joint.parentBone == boneA && joint.childBone == boneB) ||
                (joint.parentBone == boneB && joint.childBone == boneA))
            {
                return static_cast<int>(i);
            }
        }
        return -1;
    }

    void PhysicsAsset::RemoveBody(size_t index)
    {
        if (index >= bodies.size())
        {
            return;
        }
        const auto bone = bodies[index].bone;
        constraints.erase(std::remove_if(constraints.begin(), constraints.end(), [&](const PhysicsConstraint& joint) {
            return joint.parentBone == bone || joint.childBone == bone;
        }),
            constraints.end());
        bodies.erase(bodies.begin() + index);
    }

    bool PhysicsAsset::ValidateSkeleton(const std::vector<std::string>& jointNames, std::string& error) const
    {
        if (!Validate(error))
        {
            return false;
        }
        const std::set<std::string> names(jointNames.begin(), jointNames.end());
        for (const auto& body : bodies)
        {
            if (!names.count(body.bone))
            {
                error = "Skeleton has no joint named: " + body.bone;
                return false;
            }
        }
        return true;
    }

    bool PhysicsAsset::Validate(std::string& error) const
    {
        error.clear();
        std::set<std::string> names;
        if (bodies.size() > 4096)
        {
            error = "Too many physics bodies.";
            return false;
        }
        for (const auto& body : bodies)
        {
            const float values[] = {body.offset.x, body.offset.y, body.offset.z,
                body.rotation.x, body.rotation.y, body.rotation.z, body.rotation.w,
                body.halfExtents.x, body.halfExtents.y, body.halfExtents.z,
                body.radius, body.halfLength, body.mass};
            for (float value : values)
            {
                if (!std::isfinite(value))
                {
                    error = "Body contains a non-finite value.";
                    return false;
                }
            }
            const float norm = DirectX::XMVectorGetX(DirectX::XMVector4LengthSq(DirectX::XMLoadFloat4(&body.rotation)));
            if (body.bone.empty() || !names.insert(body.bone).second)
            {
                error = "Bodies need unique, nonempty bone names.";
            }
            else if (body.shape != PhysicsShapeType::Capsule && body.shape != PhysicsShapeType::Box && body.shape != PhysicsShapeType::Sphere)
            {
                error = "Unknown collision shape.";
            }
            else if (body.radius <= 0 || body.halfLength < 0 || body.mass <= 0 ||
                body.halfExtents.x <= 0 || body.halfExtents.y <= 0 || body.halfExtents.z <= 0)
            {
                error = "Body dimensions and mass must be positive.";
            }
            else if (std::abs(norm - 1.0f) > .001f)
            {
                error = "Body rotation must be a unit quaternion.";
            }
            if (!error.empty())
            {
                return false;
            }
        }
        if (constraints.size() > 4096)
        {
            error = "Too many physics joints.";
            return false;
        }
        std::set<std::pair<std::string, std::string>> pairs;
        for (const auto& joint : constraints)
        {
            if (!names.count(joint.parentBone) || !names.count(joint.childBone) || joint.parentBone == joint.childBone)
            {
                error = "A joint needs two different, existing bodies.";
            }
            else if (!pairs.insert(std::minmax(joint.parentBone, joint.childBone)).second)
            {
                error = "Only one joint is allowed between a pair of bodies.";
            }
            else if (!ValidFrame(joint.parentFrame) || !ValidFrame(joint.childFrame))
            {
                error = "Joint frames need finite positions and unit quaternions.";
            }
            for (auto motion : {joint.twist, joint.swingY, joint.swingZ})
            {
                if (motion != PhysicsJointMotion::Locked && motion != PhysicsJointMotion::Limited && motion != PhysicsJointMotion::Free)
                {
                    error = "Unknown joint motion.";
                }
            }
            for (const auto& drive : joint.drives)
            {
                if (drive.useAcceleration &&
                    (!std::isfinite(drive.naturalFrequency) || !std::isfinite(drive.dampingRatio) ||
                        drive.naturalFrequency < 0 || drive.dampingRatio < 0))
                {
                    error = "Drive frequency and damping ratio must be finite and nonnegative.";
                }
                if (!std::isfinite(drive.GetStiffness()) || !std::isfinite(drive.GetDamping()) || !std::isfinite(drive.maxTorque) ||
                    drive.GetStiffness() < 0 || drive.GetDamping() < 0 || drive.maxTorque < 0)
                {
                    error = "Drive gains and torque limits must be finite and nonnegative.";
                }
            }
            if (joint.useSlerpDrive && (joint.twist == PhysicsJointMotion::Locked || joint.swingY == PhysicsJointMotion::Locked || joint.swingZ == PhysicsJointMotion::Locked))
            {
                error = "SLERP drive requires all three angular axes unlocked.";
            }
            if (!std::isfinite(joint.twistMin) || !std::isfinite(joint.twistMax) ||
                !std::isfinite(joint.swingYLimit) || !std::isfinite(joint.swingZLimit) ||
                joint.twistMin <= -DirectX::XM_PI || joint.twistMax >= DirectX::XM_PI || joint.twistMin >= joint.twistMax ||
                joint.swingYLimit <= 0 || joint.swingYLimit >= DirectX::XM_PI ||
                joint.swingZLimit <= 0 || joint.swingZLimit >= DirectX::XM_PI)
            {
                error = "Invalid joint angular limits (radians, strictly between -pi and pi).";
            }
            if (!error.empty())
            {
                return false;
            }
        }
        return true;
    }

    bool PhysicsAsset::Save(const std::string& path, std::string& error) const
    {
        if (!Validate(error))
        {
            return false;
        }
        std::wstring temporary;
        try
        {
            const auto destination = WidePath(path);
            temporary = destination + L".tmp";
            json root;
            root["version"] = 4;
            root["driveUnits"] = "mode-dependent-radians-seconds-Nm";
            root["units"] = "metres";
            root["angleUnits"] = "radians";
            root["bodies"] = json::array();
            for (const auto& body : bodies)
            {
                json entry;
                entry["bone"] = body.bone;
                entry["shape"] = body.shape == PhysicsShapeType::Capsule ? "capsule" : body.shape == PhysicsShapeType::Box ? "box"
                                                                                                                           : "sphere";
                entry["offset"] = {body.offset.x, body.offset.y, body.offset.z};
                entry["rotation"] = {body.rotation.x, body.rotation.y, body.rotation.z, body.rotation.w};
                entry["halfExtents"] = {body.halfExtents.x, body.halfExtents.y, body.halfExtents.z};
                entry["radius"] = body.radius;
                entry["halfLength"] = body.halfLength;
                entry["mass"] = body.mass;
                root["bodies"].push_back(entry);
            }
            root["constraints"] = json::array();
            for (const auto& joint : constraints)
            {
                json entry;
                entry["parentBone"] = joint.parentBone;
                entry["childBone"] = joint.childBone;
                entry["parentFrame"] = WriteFrame(joint.parentFrame);
                entry["childFrame"] = WriteFrame(joint.childFrame);
                entry["twist"] = MotionName(joint.twist);
                entry["swingY"] = MotionName(joint.swingY);
                entry["swingZ"] = MotionName(joint.swingZ);
                entry["twistMin"] = joint.twistMin;
                entry["twistMax"] = joint.twistMax;
                entry["swingYLimit"] = joint.swingYLimit;
                entry["swingZLimit"] = joint.swingZLimit;
                entry["enableCollision"] = joint.enableCollision;
                entry["driveMode"] = joint.useSlerpDrive ? "slerp" : "swingTwist";
                entry["drives"] = json::array();
                for (const auto& drive : joint.drives)
                {
                    json settings = {{"enabled", drive.enabled}, {"maxTorque", drive.maxTorque},
                        {"type", drive.useAcceleration ? "acceleration" : "force"}};
                    if (drive.useAcceleration)
                    {
                        settings["naturalFrequency"] = drive.naturalFrequency;
                        settings["dampingRatio"] = drive.dampingRatio;
                    }
                    else
                    {
                        settings["stiffness"] = drive.stiffness;
                        settings["damping"] = drive.damping;
                    }
                    entry["drives"].push_back(settings);
                }
                root["constraints"].push_back(entry);
            }
            std::ofstream output(temporary, std::ios::binary | std::ios::trunc);
            output << root.dump(2) << '\n';
            output.close();
            if (!output || !MoveFileExW(temporary.c_str(), destination.c_str(), MOVEFILE_REPLACE_EXISTING | MOVEFILE_WRITE_THROUGH))
            {
                throw std::runtime_error("Could not write PhysicsAsset file (check path and permissions).");
            }
            return true;
        }
        catch (const std::exception& exception)
        {
            error = exception.what();
            if (!temporary.empty())
            {
                DeleteFileW(temporary.c_str());
            }
            return false;
        }
    }

    bool PhysicsAsset::Load(const std::string& path, std::string& error)
    {
        try
        {
            std::ifstream input(WidePath(path), std::ios::binary);
            if (!input)
            {
                throw std::runtime_error("Could not open PhysicsAsset file.");
            }
            json root;
            input >> root;
            const int version = root.at("version").get<int>();
            if ((version < 1 || version > 4) || root.at("units").get<std::string>() != "metres")
            {
                throw std::runtime_error("Unsupported PhysicsAsset version or units.");
            }
            if (version == 3 && root.at("driveUnits").get<std::string>() != "Nm-radians-seconds")
            {
                throw std::runtime_error("Unsupported angular drive units.");
            }
            if (version == 4 && root.at("driveUnits").get<std::string>() != "mode-dependent-radians-seconds-Nm")
            {
                throw std::runtime_error("Unsupported angular drive units.");
            }
            const auto& entries = root.at("bodies");
            if (!entries.is_array() || entries.size() > 4096)
            {
                throw std::runtime_error("Invalid body list.");
            }
            PhysicsAsset candidate;
            for (const auto& entry : entries)
            {
                PhysicsBody body;
                body.bone = entry.at("bone").get<std::string>();
                const auto shape = entry.at("shape").get<std::string>();
                if (shape != "capsule" && shape != "box" && shape != "sphere")
                {
                    throw std::runtime_error("Unknown collision shape.");
                }
                body.shape = shape == "capsule" ? PhysicsShapeType::Capsule : shape == "box" ? PhysicsShapeType::Box
                                                                                             : PhysicsShapeType::Sphere;
                const auto read = [&](const char* key, float* values, size_t count) {
                    const auto& array = entry.at(key);
                    if (!array.is_array() || array.size() != count)
                    {
                        throw std::runtime_error("Invalid body transform/dimensions.");
                    }
                    for (size_t i = 0; i < count; ++i)
                    {
                        values[i] = array.at(i).get<float>();
                    }
                };
                read("offset", &body.offset.x, 3);
                read("rotation", &body.rotation.x, 4);
                read("halfExtents", &body.halfExtents.x, 3);
                body.radius = entry.at("radius").get<float>();
                body.halfLength = entry.at("halfLength").get<float>();
                body.mass = entry.at("mass").get<float>();
                candidate.bodies.push_back(body);
            }
            if (version >= 2)
            {
                if (root.at("angleUnits").get<std::string>() != "radians")
                {
                    throw std::runtime_error("Joint angles must use radians.");
                }
                const auto& joints = root.at("constraints");
                if (!joints.is_array() || joints.size() > 4096)
                {
                    throw std::runtime_error("Invalid joint list.");
                }
                for (const auto& entry : joints)
                {
                    PhysicsConstraint joint;
                    joint.parentBone = entry.at("parentBone").get<std::string>();
                    joint.childBone = entry.at("childBone").get<std::string>();
                    joint.parentFrame = ReadFrame(entry.at("parentFrame"));
                    joint.childFrame = ReadFrame(entry.at("childFrame"));
                    joint.twist = ReadMotion(entry.at("twist"));
                    joint.swingY = ReadMotion(entry.at("swingY"));
                    joint.swingZ = ReadMotion(entry.at("swingZ"));
                    joint.twistMin = entry.at("twistMin").get<float>();
                    joint.twistMax = entry.at("twistMax").get<float>();
                    joint.swingYLimit = entry.at("swingYLimit").get<float>();
                    joint.swingZLimit = entry.at("swingZLimit").get<float>();
                    joint.enableCollision = entry.at("enableCollision").get<bool>();
                    if (version >= 3)
                    {
                        const auto mode = entry.at("driveMode").get<std::string>();
                        if (mode != "slerp" && mode != "swingTwist")
                        {
                            throw std::runtime_error("Unknown angular drive mode.");
                        }
                        joint.useSlerpDrive = mode == "slerp";
                        const auto& drives = entry.at("drives");
                        if (!drives.is_array() || drives.size() != 3)
                        {
                            throw std::runtime_error("Expected three angular drives.");
                        }
                        for (size_t axis = 0; axis < 3; ++axis)
                        {
                            auto& drive = joint.drives[axis];
                            const auto& value = drives.at(axis);
                            drive.enabled = value.at("enabled").get<bool>();
                            drive.maxTorque = value.at("maxTorque").get<float>();
                            if (version == 4)
                            {
                                const auto type = value.at("type").get<std::string>();
                                if (type != "acceleration" && type != "force")
                                {
                                    throw std::runtime_error("Unknown angular drive type.");
                                }
                                drive.useAcceleration = type == "acceleration";
                            }
                            if (drive.useAcceleration)
                            {
                                drive.naturalFrequency = value.at("naturalFrequency").get<float>();
                                drive.dampingRatio = value.at("dampingRatio").get<float>();
                            }
                            else
                            {
                                drive.stiffness = value.at("stiffness").get<float>();
                                drive.damping = value.at("damping").get<float>();
                            }
                        }
                    }
                    candidate.constraints.push_back(joint);
                }
            }
            if (!candidate.Validate(error))
            {
                return false;
            }
            bodies.swap(candidate.bodies);
            constraints.swap(candidate.constraints);
            return true;
        }
        catch (const std::exception& exception)
        {
            error = exception.what();
            return false;
        }
    }

    void ConfigureDefaultDrives(PhysicsAsset& asset)
    {
        for (auto& joint : asset.constraints)
        {
            // 하위 부위 질량은 최대 토크의 초기 추정에만 사용한다.
            // 반응 속도와 감쇠비는 질량과 분리하고, 각 바디의 관성 반영은 가속도 드라이브에 맡긴다.
            std::set<std::string> descendants{joint.childBone};
            for (size_t pass = 0; pass < asset.bodies.size(); ++pass)
            {
                for (const auto& link : asset.constraints)
                {
                    if (descendants.count(link.parentBone))
                    {
                        descendants.insert(link.childBone);
                    }
                }
            }
            float mass = 0;
            for (const auto& body : asset.bodies)
            {
                if (descendants.count(body.bone))
                {
                    mass += body.mass;
                }
            }
            const float maxTorque = (std::max)(32.0f, (std::min)(1400.0f, mass * 32.0f));
            const PhysicsJointMotion motions[] = {joint.twist, joint.swingY, joint.swingZ};
            joint.useSlerpDrive = joint.twist != PhysicsJointMotion::Locked && joint.swingY != PhysicsJointMotion::Locked && joint.swingZ != PhysicsJointMotion::Locked;
            for (int axis = 0; axis < 3; ++axis)
            {
                auto& drive = joint.drives[axis];
                drive = PhysicsAngularDrive{};
                drive.enabled = motions[axis] != PhysicsJointMotion::Locked;
                drive.useAcceleration = true;
                drive.maxTorque = maxTorque;
            }
        }
    }

    bool HasEnabledAngularDrive(const PhysicsConstraint& joint)
    {
        const PhysicsJointMotion motions[] = {joint.twist, joint.swingY, joint.swingZ};
        for (int axis = 0; axis < (joint.useSlerpDrive ? 1 : 3); ++axis)
        {
            const auto& drive = joint.drives[axis];
            if (motions[axis] != PhysicsJointMotion::Locked && drive.enabled && drive.maxTorque > 0 && (drive.GetStiffness() > 0 || drive.GetDamping() > 0))
            {
                return true;
            }
        }
        return false;
    }
}
