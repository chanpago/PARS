#include "PhysicsPose.h"
#include <algorithm>
#include <cmath>

using namespace Math;
namespace PARS
{
    namespace
    {
        Matrix4 Local(const GraphNode& node)
        {
            Matrix4 result = node.xform;
            if (node.staleMatrix)
            {
                result.Set3x3(Matrix3(node.rotation) * Matrix3::MakeScale(node.scale));
            }
            return result;
        }
        bool SceneTransforms(const PhysicsSkeleton& skeleton, const std::vector<GraphNode>& pose,
            const Matrix4& component, std::vector<Matrix4>& scene)
        {
            const size_t count = pose.size();
            if (!count || skeleton.parents.size() != count || skeleton.names.size() != count || skeleton.meshNodes.size() != count)
            {
                return false;
            }
            scene.resize(count);
            for (size_t i = 0; i < count; ++i)
            {
                const int parent = skeleton.parents[i], mesh = skeleton.meshNodes[i];
                if (parent < -1 || parent >= static_cast<int>(i) || mesh < -1 || mesh >= static_cast<int>(count))
                {
                    return false;
                }
                scene[i] = pose[i].skeletonRoot ? Local(pose[i]) : (parent < 0 ? component : scene[parent]) * Local(pose[i]);
            }
            return true;
        }

        // X는 트위스트 축이다. 축 관절에서는 길이 방향이고, 힌지에서는 굽힘 회전축이다.
        // Y는 조인트 좌표계의 나머지 방향을 결정한다.
        // 임포터의 본 축 대신 해부학적 기준점을 이용해 오른손 좌표계를 구성한다.
        Matrix3 AnatomicalAxes(Vector3 x, Vector3 y)
        {
            x = Normalize(x);
            y = Normalize(y - x * Dot(x, y));
            return Matrix3(x, y, Cross(x, y));
        }

        Matrix3 HingeAxes(Vector3 segment, Vector3 hingeHint)
        {
            // 부위의 길이 방향은 유지하고 힌지 축을 그 방향에 수직으로 투영한다.
            // 가져온 팔다리가 약간 벌어진 경우도 같은 방식으로 처리한다.
            // D6의 트위스트 X축을 힌지로 사용하고, Y축은 말단 쪽 부위를 따라 각도 표시의 기준이 된다.
            const Matrix3 axes = AnatomicalAxes(segment, hingeHint);
            return Matrix3(axes.GetY(), axes.GetX(), -axes.GetZ());
        }

        struct AnatomicalCone
        {
            // 중심은 도 단위의 순수 스윙 회전 벡터이며, 오일러 각이 아니다.
            float centerY, centerZ, halfY, halfZ;
        };

        void SetAnatomicalFrames(PhysicsConstraint& joint, const Matrix4& parentBody,
            const Matrix4& childBody, Vector3 pivot, const Matrix3& parentNeutral,
            const Matrix3& childNeutral, AnatomicalCone cone, float twistMin, float twistMax)
        {
            const Vector3 bias(0, XMConvertToRadians(cone.centerY), XMConvertToRadians(cone.centerZ));
            const float angle = Length(bias);
            const Quaternion rotation = angle > 1e-6f ? Quaternion(bias / angle, angle) : Quaternion(kIdentity);
            // 대칭인 PhysX 원뿔도 해부학적으로 비대칭인 가동 범위를 표현할 수 있다.
            // 부모 좌표계만 허용 가동 범위(ROM)의 중앙 쪽으로 회전시킨다.
            // 자식 좌표계까지 같은 편향에 맞추면 효과가 상쇄되므로 그대로 둔다.
            joint.parentFrame = PhysicsJointFrame::FromTransform(
                Invert(parentBody) * Matrix4(parentNeutral * Matrix3(rotation), pivot));
            joint.childFrame = PhysicsJointFrame::FromTransform(Invert(childBody) * Matrix4(childNeutral, pivot));
            joint.swingYLimit = XMConvertToRadians(cone.halfY);
            joint.swingZLimit = XMConvertToRadians(cone.halfZ);
            joint.twistMin = XMConvertToRadians(twistMin);
            joint.twistMax = XMConvertToRadians(twistMax);
        }
    }

    bool EvaluatePhysicsPose(const PhysicsSkeleton& skeleton, const std::vector<GraphNode>& pose,
        const Matrix4& component, std::vector<Matrix4>& world, std::string& error)
    {
        std::vector<Matrix4> scene;
        if (!SceneTransforms(skeleton, pose, component, scene))
        {
            error = "Invalid physics skeleton hierarchy.";
            return false;
        }
        world = scene;
        for (size_t i = 0; i < scene.size(); ++i)
        {
            if (skeleton.meshNodes[i] >= 0)
            {
                world[i] = scene[skeleton.meshNodes[i]] * scene[i];
            }
        }
        error.clear();
        return true;
    }

    Quaternion EvaluateJointDriveTarget(const PhysicsConstraint& joint, const Matrix4& parentBodyTarget,
        const Matrix4& childBodyTarget, bool clampToLimits)
    {
        Quaternion q = Normalize(Quaternion(Invert(parentBodyTarget * joint.parentFrame.GetTransform()) *
            (childBodyTarget * joint.childFrame.GetTransform())));
        if (!clampToLimits)
        {
            return q;
        }
        XMFLOAT4 v;
        XMStoreFloat4(&v, q);
        if (v.w < 0)
        {
            q = -q;
            XMStoreFloat4(&v, q);
        }
        const float norm = std::sqrt(v.x * v.x + v.w * v.w);
        const Quaternion twist = norm > 1e-7f ? Quaternion(XMVectorSet(v.x / norm, 0, 0, v.w / norm)) : Quaternion(kIdentity);
        XMFLOAT4 s;
        XMStoreFloat4(&s, Normalize(q * ~twist));
        if (s.w < 0)
        {
            s.y = -s.y;
            s.z = -s.z;
            s.w = -s.w;
        }
        float x = norm > 1e-7f ? 2 * std::atan2(v.x, v.w) : 0;
        float y = 4 * std::atan2(s.y, 1 + s.w), z = 4 * std::atan2(s.z, 1 + s.w);
        if (joint.twist == PhysicsJointMotion::Locked)
        {
            x = 0;
        }
        else if (joint.twist == PhysicsJointMotion::Limited)
        {
            x = std::max(joint.twistMin, std::min(joint.twistMax, x));
        }
        if (joint.swingY == PhysicsJointMotion::Locked)
        {
            y = 0;
        }
        else if (joint.swingY == PhysicsJointMotion::Limited)
        {
            y = std::max(-joint.swingYLimit, std::min(joint.swingYLimit, y));
        }
        if (joint.swingZ == PhysicsJointMotion::Locked)
        {
            z = 0;
        }
        else if (joint.swingZ == PhysicsJointMotion::Limited)
        {
            z = std::max(-joint.swingZLimit, std::min(joint.swingZLimit, z));
        }
        if (joint.swingY == PhysicsJointMotion::Limited && joint.swingZ == PhysicsJointMotion::Limited)
        {
            // 스윙 각도 좌표에서 PhysX ConeLimitHelperTanLess의 타원과 일치하도록 처리한다.
            // 각 축을 따로 제한하는 것만으로는 대각선 방향의 범위 초과를 잡을 수 없다.
            const float r = std::sqrt(y * y / (joint.swingYLimit * joint.swingYLimit) + z * z / (joint.swingZLimit * joint.swingZLimit));
            if (r > 1)
            {
                y /= r;
                z /= r;
            }
        }
        const float ty = std::tan(y * .25f), tz = std::tan(z * .25f), r2 = ty * ty + tz * tz;
        const Quaternion swing(XMVectorSet(0, 2 * ty / (1 + r2), 2 * tz / (1 + r2), (1 - r2) / (1 + r2)));
        return Normalize(swing * Quaternion(Vector3(kXUnitVector), x));
    }

    bool OverridePhysicsPose(const PhysicsSkeleton& skeleton, const std::vector<GraphNode>& source,
        const Matrix4& component, const std::vector<int>& nodes, const std::vector<Matrix4>& targets,
        std::vector<GraphNode>& result)
    {
        std::vector<Matrix4> scene;
        if (nodes.size() != targets.size() || !SceneTransforms(skeleton, source, component, scene))
        {
            return false;
        }
        const auto sourceScene = scene;
        std::vector<int> bodyAtNode(source.size(), -1);
        for (size_t i = 0; i < nodes.size(); ++i)
        {
            if (nodes[i] < 0 || nodes[i] >= static_cast<int>(source.size()) || skeleton.meshNodes[nodes[i]] < 0)
            {
                return false;
            }
            bodyAtNode[nodes[i]] = static_cast<int>(i);
        }
        result = source;
        for (size_t i = 0; i < result.size(); ++i)
        {
            const int parent = skeleton.parents[i], mesh = skeleton.meshNodes[i];
            const Matrix4 parentWorld = parent < 0 ? component : scene[parent];
            if (bodyAtNode[i] >= 0)
            {
                // Model.cpp에서 스켈레톤 루트는 의도적으로 씬 부모를 건너뛴다.
                const Matrix4 desired = Invert(sourceScene[mesh]) * targets[bodyAtNode[i]];
                result[i].xform = result[i].skeletonRoot ? desired : Invert(parentWorld) * desired;
                result[i].staleMatrix = false; // 센티미터 단위로 가져온 경우까지 포함해 원래 스케일과 전단을 정확히 보존한다.
            }
            scene[i] = result[i].skeletonRoot ? Local(result[i]) : parentWorld * Local(result[i]);
        }
        return true;
    }

    bool BuildMixamoPhysicsAsset(const PhysicsSkeleton& skeleton, const std::vector<GraphNode>& reference,
        PhysicsAsset& result, std::string& error)
    {
        std::vector<Matrix4> world;
        if (!EvaluatePhysicsPose(skeleton, reference, Matrix4(kIdentity), world, error))
        {
            return false;
        }
        const auto find = [&](const std::string& suffix) {
            for (size_t i = 0; i < skeleton.names.size(); ++i)
            {
                const auto& name = skeleton.names[i];
                const auto colon = name.find_last_of(':');
                if (name.substr(colon == std::string::npos ? 0 : colon + 1) == suffix && skeleton.meshNodes[i] >= 0)
                {
                    return static_cast<int>(i);
                }
            }
            return -1;
        };
        // de Leva(1996) 표 4의 남성 표본을 사용하며 기준 체중은 73kg이다. 팔다리 질량 비율은
        // 각각 한쪽 기준이다. DOI: https://doi.org/10.1016/0021-9290(95)00178-6
        // 머리/목과 몸통/어깨의 그룹 구분은 Karatsidis 외(2017)의 표 1을 따른다.
        // 표 1 출처: https://doi.org/10.3390/s17010075
        // 젊은 성인 남성의 참고값이며, 한국인이나 세계 인구 전체의 평균은 아니다.
        constexpr float totalMassKg = 73.0f;
        // Roos 외(2020) 표 3: 남성 모델의 머리 4.20kg, 목 1.66kg.
        // https://doi.org/10.1371/journal.pone.0219954
        // 이 연구에서는 그룹 내부 비율만 가져오고, de Leva의 전체 비율 6.94%는 유지한다.
        // 서로 다른 연구를 연결한 근사값이다. 작은 충돌 캡슐의 부피만으로
        // 목 전체 조직의 부피를 신뢰성 있게 추정할 수는 없다.
        constexpr float neckShare = 1.66f / (4.20f + 1.66f);
        enum MassGroup
        {
            Pelvis,
            MiddleTrunk,
            UpperTrunk,
            Neck,
            Head,
            LeftUpperArm,
            LeftForearm,
            LeftHand,
            RightUpperArm,
            RightForearm,
            RightHand,
            LeftThigh,
            LeftShank,
            LeftFoot,
            RightThigh,
            RightShank,
            RightFoot,
            MassGroupCount
        };
        constexpr float massFractions[MassGroupCount] = {
            .1117f, .1633f, .1596f, .0694f * neckShare, .0694f * (1.0f - neckShare),
            .0271f, .0162f, .0061f, .0271f, .0162f, .0061f,
            .1416f, .0433f, .0137f, .1416f, .0433f, .0137f};
        struct Part
        {
            const char* bone;
            const char* end;
            float radius;
            MassGroup massGroup;
            PhysicsShapeType shape;
        };
        // 손가락, 발가락, 말단 표시 노드를 제외한 Mixamo의 모든 관절 본을 포함한다.
        // 척추/목/쇄골 바디를 유지해 머리와 위팔을 Spine2에 바로 붙이는 대신
        // 조인트가 실제 본 계층을 따르도록 한다.
        // 손가락/발가락은 별도 바디를 만들지 않으며, 해당 질량은 손/발에 포함한다.
        const Part parts[] = {
            {"Hips", "Spine", .13f, Pelvis, PhysicsShapeType::Box},
            {"Spine", "Spine1", .115f, MiddleTrunk, PhysicsShapeType::Box},
            {"Spine1", "Spine2", .125f, MiddleTrunk, PhysicsShapeType::Box},
            {"Spine2", "Neck", .13f, UpperTrunk, PhysicsShapeType::Box},
            {"Neck", "Head", .04f, Neck, PhysicsShapeType::Capsule},
            {"Head", "HeadTop_End", .09f, Head, PhysicsShapeType::Capsule},
            {"LeftShoulder", "LeftArm", .035f, UpperTrunk, PhysicsShapeType::Capsule},
            {"LeftArm", "LeftForeArm", .05f, LeftUpperArm, PhysicsShapeType::Capsule},
            {"LeftForeArm", "LeftHand", .04f, LeftForearm, PhysicsShapeType::Capsule},
            {"LeftHand", "LeftHandMiddle1", .035f, LeftHand, PhysicsShapeType::Capsule},
            {"RightShoulder", "RightArm", .035f, UpperTrunk, PhysicsShapeType::Capsule},
            {"RightArm", "RightForeArm", .05f, RightUpperArm, PhysicsShapeType::Capsule},
            {"RightForeArm", "RightHand", .04f, RightForearm, PhysicsShapeType::Capsule},
            {"RightHand", "RightHandMiddle1", .035f, RightHand, PhysicsShapeType::Capsule},
            {"LeftUpLeg", "LeftLeg", .075f, LeftThigh, PhysicsShapeType::Capsule},
            {"LeftLeg", "LeftFoot", .055f, LeftShank, PhysicsShapeType::Capsule},
            {"LeftFoot", "LeftToeBase", .045f, LeftFoot, PhysicsShapeType::Capsule},
            {"RightUpLeg", "RightLeg", .075f, RightThigh, PhysicsShapeType::Capsule},
            {"RightLeg", "RightFoot", .055f, RightShank, PhysicsShapeType::Capsule},
            {"RightFoot", "RightToeBase", .045f, RightFoot, PhysicsShapeType::Capsule}};
        PhysicsAsset candidate;
        std::vector<int> nodes;
        std::vector<double> bodyVolumes;
        double groupVolumes[MassGroupCount]{};
        for (const auto& part : parts)
        {
            const int node = find(part.bone), end = find(part.end);
            if (node < 0 || end < 0)
            {
                error = "Mixamo PA preset cannot find bone: " + std::string(node < 0 ? part.bone : part.end);
                return false;
            }
            const Vector3 endpoint(Invert(PhysicsBoneFrame(world[node])) * Vector3(world[end].GetW()));
            const float length = Length(endpoint);
            if (!std::isfinite(length) || length < .005f)
            {
                error = "Invalid Mixamo bone length.";
                return false;
            }
            const Vector3 axis = endpoint / length;
            const float dot = Clamp(float(axis.GetX()), -1.0f, 1.0f);
            Quaternion rotation(kIdentity);
            if (dot < -.9999f)
            {
                rotation = Quaternion(Vector3(kYUnitVector), XM_PI);
            }
            else if (dot < .9999f)
            {
                rotation = Quaternion(Normalize(Cross(Vector3(kXUnitVector), axis)), std::acos(dot));
            }
            PhysicsBody body;
            body.bone = skeleton.names[node];
            body.shape = part.shape;
            body.radius = part.radius;
            body.halfLength = Max(.0f, length * .5f - body.radius);
            // 몸통 박스는 각자 담당하는 본 구간만 덮는다. 반길이에 고정 최솟값 7cm를 적용하면
            // 추가된 짧은 척추 바디끼리 겹치게 된다.
            body.halfExtents = XMFLOAT3(length * .45f, part.radius, part.radius * .7f);
            XMStoreFloat3(&body.offset, endpoint * .5f);
            XMStoreFloat4(&body.rotation, rotation);
            const double radius = body.radius;
            const double volume = body.shape == PhysicsShapeType::Box
                ? 8.0 * body.halfExtents.x * body.halfExtents.y * body.halfExtents.z
                : XM_PI * radius * radius * (2.0 * body.halfLength + 4.0 * radius / 3.0);
            bodyVolumes.push_back(volume);
            groupVolumes[part.massGroup] += volume;
            candidate.bodies.push_back(body);
            nodes.push_back(node);
        }
        // de Leva 모델은 16개 부위이고 이 리그는 20개 바디다. 머리/목은 위에서 나눈다.
        // 몸통의 각 그룹 질량은 보존하고, 그룹 내부에서만 콜라이더 부피에 비례해 나눈다.
        // 이때 밀도는 균일하다고 가정한다. 추가적인 몸통 분할과 Mixamo의 해부학적 경계는
        // 근사값이며, 측정된 뼈별 질량 비율은 아니다.
        // 어깨는 상부 몸통에 속하므로 전체 체중에 별도 질량을 더하지 않는다.
        // PhysX의 질량 중심/관성은 연구의 텐서가 아니라 기본 형상에서 계산한다.
        for (size_t i = 0; i < candidate.bodies.size(); ++i)
        {
            const MassGroup group = parts[i].massGroup;
            candidate.bodies[i].mass = static_cast<float>(
                totalMassKg * massFractions[group] * bodyVolumes[i] / groupVolumes[group]);
        }

        const auto position = [&](int node) {
            return Vector3(world[node].GetW());
        };
        const Vector3 left = Normalize(position(find("LeftUpLeg")) - position(find("RightUpLeg")));
        const Vector3 upHint = position(find("Neck")) - position(find("Hips"));
        if (float(LengthSquare(Cross(left, upHint))) < 1e-8f)
        {
            error = "Mixamo PA preset needs an upright reference skeleton.";
            return false;
        }
        const Vector3 up = Normalize(upHint - left * Dot(left, upHint));
        const Vector3 forward = Normalize(Cross(left, up));
        std::vector<Vector3> longitudinal;
        for (size_t i = 0; i < nodes.size(); ++i)
        {
            longitudinal.push_back(Normalize(position(find(parts[i].end)) - position(nodes[i])));
        }
        Vector3 radial[2];
        for (int side = 0; side < 2; ++side)
        {
            const std::string prefix = side == 0 ? "Left" : "Right";
            const int index = find(prefix + "HandIndex1"), pinky = find(prefix + "HandPinky1");
            if (index < 0 || pinky < 0 || float(LengthSquare(position(index) - position(pinky))) < 1e-8f)
            {
                error = "Mixamo PA preset needs hand landmarks to calibrate wrist orientation.";
                return false;
            }
            radial[side] = Normalize(position(index) - position(pinky));
            // 프리셋은 가져온 T 포즈를 기준으로 보정하며, 샘플링한 애니메이션 프레임은 사용하지 않는다.
            // 굽힘 축을 정할 수 없는 퇴화된 경우에는 NaN을 만드는 대신 거부한다.
            for (const char* suffix : {"Arm", "ForeArm", "Hand"})
            {
                const int body = candidate.FindBody(skeleton.names[find(prefix + suffix)]);
                if (std::abs(float(Dot(longitudinal[body], forward))) > .95f ||
                    std::abs(float(Dot(longitudinal[body], radial[side]))) > .95f)
                {
                    error = "Mixamo PA preset needs an arms-out reference pose.";
                    return false;
                }
            }
        }

        // 관절 가동 범위(ROM)의 출처이며, 각도 단위는 도다.
        // CDC/Soucie 2011, 20~44세 남성의 수동 가동 범위 평균: 팔꿈치 -0.8~144.6,
        // 무릎 -1~137.7, 고관절 -17.4~130.4,
        // 발목 족저굴곡(발끝 내리기) 54.6 / 배측굴곡(발끝 올리기) 12.7.
        // https://archive.cdc.gov/www_cdc_gov/ncbddd/jointrom/index.html
        // Moromizato 2016 표 1, 남성의 수동 가동 범위 평균(20~29세 남성 42명):
        // 손목 신전/굴곡 79/87, 몸통 신전/굴곡 28.2/37.3,
        // 몸통 측굴 24.8, 회전 52.2, 고관절 외전/내전 32.1/13.7,
        // 고관절 외회전/내회전 46.7/37.1, 어깨 수평 신전/굴곡 54.3/130.7,
        // 어깨 외전 179.8, 외회전/내회전 91.5/58.4.
        // https://doi.org/10.1186/s40101-016-0112-8
        // Reynolds 2009 표 5, 남녀 혼합 표본에서 측정한 목 전체의 능동 가동 범위 합계:
        // 굴곡+신전 125, 좌우 측굴 79.8, 회전 131.
        // https://doi.org/10.1007/s00586-009-0894-z
        // 편집 가능한 참고 범위이며, 모든 사람에게 적용되는 절대적인 한계는 아니다.
        // 고정 원뿔만으로는 자세에 따라 달라지는 인체의 결합 운동 제한을 재현할 수 없다.
        // 팔꿈치/무릎은 순수 힌지를 사용한다. 아래팔 회내/회외에는 별도의 축 회전 조인트가
        // 필요하며, 이는 팔꿈치 굽힘 제한에 포함되지 않는다.
        //
        // 아래의 두 축 원뿔 계수는 이 SDK의 ConeLimitHelperTanLess 각도 공간에서
        // 각 평면 방향의 네 끝점에 맞춘 값이다.
        // (swingY/halfY)^2 + (swingZ/halfZ)^2 = 1이며, 트위스트를 제거한 뒤
        // swingY/Z = 4*atan2(q.y/z, 1+q.w)로 계산한다. 편향된 부모 좌표계는 순수 스윙이므로
        // 중립 자세의 트위스트는 0이다. 두 축 모두 단순히 가동 범위의 절반을 사용하면
        // 굴곡 편향이 클 때 측면 가동 범위가 잘못 줄어든다.
        // centerZ와 트위스트의 부호를 반전해 좌우의 해부학적 대칭을 유지한다.
        for (size_t i = 1; i < nodes.size(); ++i)
        {
            int parent = skeleton.parents[nodes[i]], parentBody = -1;
            while (parent >= 0)
            {
                parentBody = candidate.FindBody(skeleton.names[parent]);
                if (parentBody >= 0)
                {
                    break;
                }
                parent = skeleton.parents[parent];
            }
            if (parentBody < 0)
            {
                error = "Mixamo PA preset has disconnected limbs.";
                return false;
            }
            PhysicsConstraint joint;
            joint.parentBone = candidate.bodies[parentBody].bone;
            joint.childBone = candidate.bodies[i].bone;
            const Matrix4 parentWorld = PhysicsBoneFrame(world[parent]) * candidate.bodies[parentBody].GetLocalTransform();
            const Matrix4 childWorld = PhysicsBoneFrame(world[nodes[i]]) * candidate.bodies[i].GetLocalTransform();
            const std::string name(parts[i].bone);
            const bool isLeft = name.find("Left") == 0;
            const float side = isLeft ? 1.0f : -1.0f;
            const Vector3 x = longitudinal[i], parentX = longitudinal[parentBody];
            Matrix3 parentAxes = AnatomicalAxes(up, left), childAxes = parentAxes;
            AnatomicalCone cone{};
            float twistMin = -1, twistMax = 1; // 잠긴 축에서는 무시된다.
            if (name.find("Spine") == 0)
            {
                // 리그의 조인트 3개가 몸통 전체의 가동 범위를 동일하게 나눠 가진다.
                // 리그에 맞춘 근사 배분이며, 척추뼈별 가동 범위 측정값은 아니다.
                cone = {1.516667f, 0, 10.916667f, 8.347644f};
                twistMax = 52.2f / 3;
                twistMin = -twistMax;
            }
            else if (name == "Neck" || name == "Head")
            {
                // 보수적인 능동 가동 범위로, 측정된 총 125도 안에서 굴곡 55도 / 신전 70도를 선택한다.
                // 이 범위를 리그의 Neck/Head 조인트에 60:40으로 나눈다.
                // 방향별 범위와 리그 내 배분은 모델링상의 선택이다.
                // 이 두 조인트는 해부학적으로 실제 척추뼈 두 개를 나타내는 것이 아니다.
                const bool neck = name == "Neck";
                cone = neck ? AnatomicalCone{-4.5f, 0, 37.5f, 24.121613f}
                            : AnatomicalCone{-3, 0, 25, 16.078565f};
                twistMax = 65.5f * (neck ? .6f : .4f);
                twistMin = -twistMax;
            }
            else if (name.find("Shoulder") != std::string::npos)
            {
                // Shoulder 본이 어깨이음을 대신한다. 이 리그에는 별도의 견갑골 바디가 없다.
                // 여기에 수평 -10~20도와 거상 -10~30도를 배분하고,
                // 아래의 위팔뼈 범위에서 그만큼 뺀다. 근사 방식이다.
                parentAxes = childAxes = AnatomicalAxes(left * side, up * -side);
                // 대기/걷기/달리기 보고서의 결합 스윙을 수용하는 튜닝 범위다.
                // 각 축 최대값뿐 아니라 타원 경계에도 여유를 두며, 해부학적 ROM 측정값은 아니다.
                cone = {4.973932f, -9.956774f * side, 30, 35};
                // 실제 추종 궤적에도 여유를 두어 PD 비교 중 경계 반력이 개입하지 않게 한다.
                twistMin = -8;
                twistMax = 8;
            }
            else if (name == "LeftArm" || name == "RightArm")
            {
                parentAxes = AnatomicalAxes(left * side, up * -side);
                childAxes = AnatomicalAxes(x, Cross(x, forward));
                // 위팔뼈는 수평 -44.3~110.7도, T 포즈에서 내리기 90도 / 올리기 59.8도를 사용한다.
                // 어깨이음의 범위와 합쳐 측정된 전체 어깨 가동 범위를 근사하며,
                // 동일한 가동 범위를 중복으로 계산하지 않는다.
                // 걷기의 결합 스윙 목표와 실제 추종 궤적을 함께 수용하도록 반각을 보정한다.
                // 중심과 해부학적 축은 유지하며, 이 추가 여유는 가져온 애니메이션의 PD 비교용이다.
                cone = {32.595316f, 13.336539f * side, 90, 90};
                twistMin = isLeft ? -91.5f : -58.4f;
                twistMax = isLeft ? 58.4f : 91.5f;
            }
            else if (name.find("ForeArm") != std::string::npos)
            {
                // 굽힘에는 트위스트 X축을 사용한다. 좌우 X축을 대칭으로 잡아
                // 양쪽 모두 양의 트위스트에서 아래팔이 앞으로 굽혀지게 한다. Y축은 각 아래팔을 따라
                // 초록색 각도 표시가 실제 팔꿈치 굽힘을 나타내도록 한다.
                parentAxes = HingeAxes(parentX, Cross(parentX, forward));
                childAxes = HingeAxes(x, Cross(x, forward));
                // 걷기 보고서의 아래팔 목표는 Y 최대 23.4도, Z 최대 16.2도의
                // 보조 회전을 포함한다. 두 축을 잠그면 굽힘이 정상이어도 목표가 약 20도 잘린다.
                // 길이 방향 회전은 굽힘과 결합되어 두 스윙 성분에 나뉘므로 둘 다 Limited로 둔다.
                // 대각선 타원 한도에도 여유를 둔 리그 튜닝값이며, 해부학적 ROM 측정값은 아니다.
                cone = {0, 0, 30, 22};
                twistMin = -.8f;
                twistMax = 144.6f;
                joint.swingY = joint.swingZ = PhysicsJointMotion::Limited;
            }
            else if (name.find("Hand") != std::string::npos)
            {
                const Vector3 hinge = radial[isLeft ? 0 : 1] * -side;
                parentAxes = AnatomicalAxes(parentX, hinge);
                childAxes = AnatomicalAxes(x, hinge);
                // 신전 -79도~굴곡 87도다. 요측/척측 편위 20/30도는 보수적인 리그 제한값이며,
                // 위에서 인용한 손목 연구에서 측정된 값은 아니다.
                // 달리기의 스윙 Z 최대 26.4도와 대기의 트위스트 최대 41.6도를 포함한다.
                // 리그의 손 본에 들어 있는 축 회전을 수용하는 근사이며,
                // 생리학적인 손목 회내 동작이나 별도의 아래팔 축 회전 조인트를 의미하지 않는다.
                cone = {3.959488f, 5.002095f * side, 83.083931f, 30};
                twistMin = -45;
                twistMax = 45;
            }
            else if (name.find("UpLeg") != std::string::npos)
            {
                parentAxes = AnatomicalAxes(-up, -left);
                childAxes = AnatomicalAxes(x, -left);
                // 걷기의 스윙 Y 최대 77.5도와 동시 발생하는 Z 회전을 수용한다.
                // 측면 반각과 중심은 유지하고 굽힘 방향에 애니메이션용 여유만 추가한다.
                cone = {55.314192f, -9.524431f * side, 83, 37.496596f};
                twistMin = isLeft ? -46.7f : -37.1f;
                twistMax = isLeft ? 37.1f : 46.7f;
            }
            else if (name.find("Leg") != std::string::npos)
            {
                // D6의 트위스트 X축으로 굽힌다. X축은 무릎을 가로지르고 Y축은 다리를 따른다.
                // 양쪽 모두 양의 X 회전에서 정강이가 뒤로 굽혀진다.
                // 이 조인트 축들은 캡슐의 X축과 별개다.
                parentAxes = HingeAxes(parentX, left);
                childAxes = HingeAxes(x, left);
                // 걷기 목표의 보조 회전은 Y 최대 20.5도, Z 최대 9.8도다.
                // 단일 축 최대값만 맞추면 결합된 타원 한도에서 다시 잘리므로 여유를 둔다.
                // 이는 이 리그의 애니메이션을 위한 근사이며, 실제 무릎의 독립적인 측굴 ROM이 아니다.
                cone = {0, 0, 26, 16};
                twistMin = -1;
                twistMax = 137.7f;
                joint.swingY = joint.swingZ = PhysicsJointMotion::Limited;
            }
            else // 발은 발목에서 발가락으로 향하는 캡슐 X축 대신 가상의 아래쪽 축을 사용한다.
            {
                parentAxes = AnatomicalAxes(parentX, -left);
                childAxes = AnatomicalAxes(-up, -left);
                // 시상면 범위는 족저굴곡 -54.6도~배측굴곡 12.7도다. 내번/외번 30/15도는
                // 발목/거골하관절을 합친 대체 바디의 리그 제한값이며 CDC 측정값은 아니다.
                // 가져온 달리기 목표는 이 좌표계에서 Y 74.2도, Z 27.2도,
                // 트위스트 21.3도까지 필요하다. 결합 타원에도 여유를 둔 튜닝 범위로,
                // 위의 참고 ROM보다 넓다. 기준 프레임이나 목표 애니메이션은 바꾸지 않는다.
                cone = {-20.754667f, 7.333241f * side, 82, 38};
                twistMin = -25;
                twistMax = 25;
            }
            SetAnatomicalFrames(joint, parentWorld, childWorld, position(nodes[i]),
                parentAxes, childAxes, cone, twistMin, twistMax);
            candidate.constraints.push_back(joint);
        }
        if (!candidate.Validate(error))
        {
            return false;
        }
        ConfigureDefaultDrives(candidate);
        for (auto& joint : candidate.constraints)
        {
            const auto group = parts[candidate.FindBody(joint.childBone)].massGroup;
            if (group == LeftShank || group == RightShank)
            {
                // 무릎의 기존 토크 여유는 유지하되, 반응 속도와 감쇠비는 다른 관절과 같은 기본값을 쓴다.
                for (auto& drive : joint.drives)
                {
                    drive.maxTorque *= 2;
                }
            }
        }
        result = std::move(candidate);
        return true;
    }
}
