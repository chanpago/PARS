#include "AnimInstance.h"
#include "Components/SkeletalMeshComponent.h"
#include "Model.h"
#include "PhysicsPose.h"
#include <algorithm>
#include <cmath>

namespace PARS
{
    namespace
    {
        constexpr double SamplesPerSecond = 60.0;

        struct MotionBones
        {
        public:
            Math::Vector3 bindForwardInPelvis;
            size_t hips;
            std::array<size_t, 2> feet;
        };

        struct MotionSample
        {
        public:
            Math::Vector3 pelvis;
            Math::Matrix3 toHeadingSpace;
            std::array<Math::Vector3, 2> feet;
            float time;
        };

        size_t FindJoint(const PhysicsSkeleton& skeleton, const std::string& joint)
        {
            for (size_t i = 0; i < skeleton.names.size(); ++i)
            {
                const std::string& name = skeleton.names[i];
                const size_t separator = name.rfind(':');
                const size_t start = separator == std::string::npos ? 0 : separator + 1;
                if (skeleton.meshNodes[i] >= 0 && name.compare(start, std::string::npos, joint) == 0)
                {
                    return i;
                }
            }
            return skeleton.names.size();
        }

        bool SampleClip(const AnimInstance& animation, uint32_t clipIndex, const PhysicsSkeleton& skeleton,
            const MotionBones& bones, std::vector<MotionSample>& samples, std::string& error)
        {
            const float duration = animation.GetAnimationDuration(clipIndex);
            const double intervals = std::max(1.0, std::round(static_cast<double>(duration) * SamplesPerSecond));
            if (intervals >= static_cast<double>(UINT32_MAX))
            {
                error = u8"모션 후보를 생성할 수 있는 클립 길이를 초과했습니다.";
                return false;
            }
            samples.resize(static_cast<size_t>(intervals) + 1);
            std::vector<GraphNode> pose;
            std::vector<Math::Matrix4> modelPose;
            for (size_t i = 0; i < samples.size(); ++i)
            {
                MotionSample& sample = samples[i];
                sample.time = static_cast<float>(duration * static_cast<double>(i) / intervals);
                if (i > 0 && sample.time <= samples[i - 1].time)
                {
                    error = u8"클립의 인접 샘플을 서로 다른 재생 시각으로 표현할 수 없습니다.";
                    return false;
                }
                if (!animation.SampleAnimationPose(clipIndex, sample.time, pose))
                {
                    error = u8"애니메이션 트랙에서 본 자세를 읽을 수 없습니다.";
                    return false;
                }
                if (!EvaluatePhysicsPose(skeleton, pose, Math::Matrix4(Math::kIdentity), modelPose, error))
                {
                    return false;
                }

                const Math::Matrix4& hips = modelPose[bones.hips];
                sample.pelvis = Math::Vector3(hips.GetW());
                const Math::Vector3 facing = PhysicsBoneFrame(hips).Get3x3() * bones.bindForwardInPelvis;
                const Math::Vector3 horizontal(facing.GetX(), 0.0f, facing.GetZ());
                const float length = Math::Length(horizontal);
                if (!std::isfinite(length) || length < 1e-5f)
                {
                    error = u8"골반의 수평 전방을 계산할 수 없습니다.";
                    return false;
                }
                const Math::Vector3 forward = horizontal / length;
                const Math::Vector3 up(Math::kYUnitVector);
                sample.toHeadingSpace = Math::Transpose(Math::Matrix3(Math::Cross(up, forward), up, forward));
                for (size_t foot = 0; foot < bones.feet.size(); ++foot)
                {
                    const Math::Vector3 position(modelPose[bones.feet[foot]].GetW());
                    sample.feet[foot] = sample.toHeadingSpace * (position - sample.pelvis);
                    if (DirectX::XMVector3IsNaN(sample.feet[foot]) || DirectX::XMVector3IsInfinite(sample.feet[foot]))
                    {
                        error = u8"골반 또는 발 위치가 유한한 좌표가 아닙니다.";
                        return false;
                    }
                }
            }
            return true;
        }

        void AppendClipFrames(const MotionClipSelection& clip, const std::vector<MotionSample>& samples,
            std::vector<MotionFrame>& frames)
        {
            const size_t last = samples.size() - 1;
            const double lookahead = MotionFeatures::PredictionTimeSeconds * static_cast<double>(last) / samples.back().time;
            for (size_t i = 0; i <= last; ++i)
            {
                const MotionSample& sample = samples[i];
                // 내부 프레임은 중앙 차분을, 양 끝은 클립 안에서의 단측 차분을 사용한다.
                const MotionSample& previous = samples[i == 0 ? 0 : i - 1];
                const MotionSample& next = samples[std::min(i + 1, last)];
                MotionFrame frame;
                frame.clipIndex = clip.clipIndex;
                frame.clipTimeSeconds = sample.time;
                frame.usage = clip.usage;
                for (size_t foot = 0; foot < sample.feet.size(); ++foot)
                {
                    const Math::Vector3 velocity = (next.feet[foot] - previous.feet[foot]) / (next.time - previous.time);
                    DirectX::XMStoreFloat3(&frame.features.footPositions[foot], sample.feet[foot]);
                    DirectX::XMStoreFloat3(&frame.features.footVelocities[foot], velocity);
                }
                // 예측 구간이 클립 끝을 넘으면 마지막 골반 위치에서 멈춘다.
                const double future = std::min(i + lookahead, static_cast<double>(last));
                const size_t first = static_cast<size_t>(future);
                const size_t second = std::min(first + 1, last);
                const Math::Vector3 futurePelvis = Math::Lerp(samples[first].pelvis, samples[second].pelvis,
                    static_cast<float>(future - first));
                DirectX::XMStoreFloat3(&frame.features.futureDisplacement, sample.toHeadingSpace * (futurePelvis - sample.pelvis));
                frames.push_back(frame);
            }
        }
    }

    bool AnimInstance::BuildMotionFrames(std::initializer_list<MotionClipSelection> clips, std::string& error)
    {
        if (!m_Skeleton || m_Skeleton != m_Owner.GetSkeletalMesh() || clips.size() == 0)
        {
            error = u8"모션 후보를 생성할 메시 컴포넌트의 기준 스켈레톤과 등록된 클립이 필요합니다.";
            return false;
        }
        const std::shared_ptr<MotionDatabase> database = std::make_shared<MotionDatabase>();
        std::vector<bool> included(m_Clips.size(), false);
        for (const MotionClipSelection& clip : clips)
        {
            const size_t usageIndex = static_cast<size_t>(clip.usage);
            if (usageIndex >= database->groups.size())
            {
                error = u8"알 수 없는 모션 후보 용도입니다.";
                return false;
            }
            if (clip.clipIndex >= included.size() || included[clip.clipIndex])
            {
                error = u8"등록되지 않았거나 중복된 모션 후보 클립입니다.";
                return false;
            }
            included[clip.clipIndex] = true;
            database->groups[usageIndex].clipIndices.push_back(clip.clipIndex);
        }

        PhysicsSkeleton skeleton;
        if (!m_Owner.BuildPhysicsSkeleton(skeleton, error))
        {
            return false;
        }
        MotionBones bones;
        bones.hips = FindJoint(skeleton, "Hips");
        bones.feet = {FindJoint(skeleton, "LeftFoot"), FindJoint(skeleton, "RightFoot")};
        if (bones.hips == skeleton.names.size() || bones.feet[0] == skeleton.names.size() || bones.feet[1] == skeleton.names.size())
        {
            error = u8"모션 후보 생성에 필요한 Hips, LeftFoot, RightFoot 본을 찾을 수 없습니다.";
            return false;
        }
        const std::vector<GraphNode> reference(m_Skeleton->m_SceneGraph.get(), m_Skeleton->m_SceneGraph.get() + m_Skeleton->m_NumNodes);
        std::vector<Math::Matrix4> bindPose;
        if (!EvaluatePhysicsPose(skeleton, reference, Math::Matrix4(Math::kIdentity), bindPose, error))
        {
            return false;
        }
        // 기준 포즈의 모델 전방(+Z)을 골반 본의 좌표로 옮겨 클립마다 동일한 전방 기준을 쓴다.
        bones.bindForwardInPelvis = Math::Transpose(PhysicsBoneFrame(bindPose[bones.hips]).Get3x3()) * Math::Vector3(Math::kZUnitVector);

        std::vector<MotionSample> samples;
        for (size_t usageIndex = 0; usageIndex < database->groups.size(); ++usageIndex)
        {
            MotionFrameGroup& group = database->groups[usageIndex];
            group.range.firstFrameIndex = database->frames.size();
            for (uint32_t clipIndex : group.clipIndices)
            {
                if (!SampleClip(*this, clipIndex, skeleton, bones, samples, error))
                {
                    error = GetAnimationName(clipIndex) + " - " + error;
                    return false;
                }
                AppendClipFrames({clipIndex, static_cast<MotionFrameUsage>(usageIndex)}, samples, database->frames);
            }
            group.range.frameCount = database->frames.size() - group.range.firstFrameIndex;
        }
        m_MotionDatabase = database;
        error.clear();
        return true;
    }
}
