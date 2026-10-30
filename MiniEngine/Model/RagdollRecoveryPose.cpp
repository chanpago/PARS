#include "RagdollRecovery.h"
#include "Animation.h"
#include "PhysicsAsset.h"
#include <cmath>

using namespace Math;

bool PARS::BlendRecoveryPose(const std::vector<GraphNode>& source, std::vector<GraphNode>& target, float alpha)
{
    if (source.size() != target.size() || !std::isfinite(alpha) || alpha < 0 || alpha > 1)
    {
        return false;
    }
    if (alpha == 1)
    {
        return true;
    }
    auto result = target;
    const auto local = [](const GraphNode& node)
    {
        Matrix4 value = node.xform;
        if (node.staleMatrix)
        {
            value.Set3x3(Matrix3(node.rotation) * Matrix3::MakeScale(node.scale));
        }
        return value;
    };
    for (size_t i = 0; i < result.size(); ++i)
    {
        const Matrix4 from = local(source[i]), to = local(target[i]);
        if (DirectX::XMMatrixIsNaN(from) || DirectX::XMMatrixIsInfinite(from) ||
            DirectX::XMMatrixIsNaN(to) || DirectX::XMMatrixIsInfinite(to))
        {
            return false;
        }
        auto& node = result[i];
        if (alpha == 0)
        {
            node.xform = from;
        }
        else
        {
            const Matrix4 fromFrame = PhysicsBoneFrame(from), toFrame = PhysicsBoneFrame(to);
            const Matrix4 fromStretch = Invert(fromFrame) * from, toStretch = Invert(toFrame) * to;
            const Matrix3 stretch(Vector3(fromStretch.GetX()) * (1 - alpha) + Vector3(toStretch.GetX()) * alpha,
                Vector3(fromStretch.GetY()) * (1 - alpha) + Vector3(toStretch.GetY()) * alpha,
                Vector3(fromStretch.GetZ()) * (1 - alpha) + Vector3(toStretch.GetZ()) * alpha);
            const Quaternion rotation = Slerp(Quaternion(fromFrame), Quaternion(toFrame), alpha);
            const Vector3 position = Vector3(from.GetW()) * (1 - alpha) + Vector3(to.GetW()) * alpha;
            // 본 길이의 기준인 로컬 변환을 보간한다. 회전은 구면 보간하고 스케일·전단은 따로 보존한다.
            node.xform = Matrix4(Matrix3(rotation) * stretch, position);
        }
        node.staleMatrix = false;
    }
    target = std::move(result);
    return true;
}
