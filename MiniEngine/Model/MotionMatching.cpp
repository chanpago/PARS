#include "AnimInstance.h"
#include "Components/SkeletalMeshComponent.h"
#include "Model.h"
#include "PhysicsPose.h"
#include <algorithm>
#include <cmath>

namespace PARS
{
    float GetWalkingTravelAngleDegrees(const DirectX::XMFLOAT2& forwardXZ, const DirectX::XMFLOAT2& velocityXZ)
    {
        // 0.2초 동안 1cm도 움직이지 않을 때는 작은 흔들림에 진행각을 부여하지 않는다.
        constexpr float minimumSpeed = 0.05f;
        const float speed = std::hypot(velocityXZ.x, velocityXZ.y);
        const float lateral = forwardXZ.y * velocityXZ.x - forwardXZ.x * velocityXZ.y;
        const float forward = forwardXZ.x * velocityXZ.x + forwardXZ.y * velocityXZ.y;
        if (!std::isfinite(speed) || speed < minimumSpeed || !std::isfinite(lateral) || !std::isfinite(forward) ||
            (lateral == 0.0f && forward == 0.0f))
        {
            return std::numeric_limits<float>::quiet_NaN();
        }
        return DirectX::XMConvertToDegrees(std::atan2(lateral, forward));
    }

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
            std::array<Math::Vector3, 2> modelFeet;
            float time;
        };

        bool MakeHeadingSpace(const Math::Matrix4& pelvis, Math::Vector3 bindForward, Math::Matrix3& toHeadingSpace)
        {
            const Math::Vector3 facing = PhysicsBoneFrame(pelvis).Get3x3() * bindForward;
            const Math::Vector3 horizontal(facing.GetX(), 0.0f, facing.GetZ());
            const float length = Math::Length(horizontal);
            if (!std::isfinite(length) || length < 1e-5f)
            {
                return false;
            }
            const Math::Vector3 forward = horizontal / length;
            const Math::Vector3 up(Math::kYUnitVector);
            toHeadingSpace = Math::Transpose(Math::Matrix3(Math::Cross(up, forward), up, forward));
            return true;
        }

        float SquaredDistance(const DirectX::XMFLOAT3& first, const DirectX::XMFLOAT3& second)
        {
            return Math::LengthSquare(Math::Vector3(first) - Math::Vector3(second));
        }

        int WalkingAngleError(float requestedDegrees, float clipDegrees)
        {
            if (!std::isfinite(requestedDegrees))
            {
                return 0;
            }
            if (!std::isfinite(clipDegrees))
            {
                return 181;
            }
            // ±180도 경계를 잇고, 각도 오차를 1도 단위로 비교해 같은 진행 방향의 클립끼리는 자세로 고른다.
            const float difference = std::remainder(requestedDegrees - clipDegrees, 360.0f);
            return static_cast<int>(std::lround(std::abs(difference)));
        }

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
                if (!MakeHeadingSpace(hips, bones.bindForwardInPelvis, sample.toHeadingSpace))
                {
                    error = u8"골반의 수평 전방을 계산할 수 없습니다.";
                    return false;
                }
                for (size_t foot = 0; foot < bones.feet.size(); ++foot)
                {
                    const Math::Vector3 position(modelPose[bones.feet[foot]].GetW());
                    sample.modelFeet[foot] = position;
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

        void MarkWalkingSwings(const std::vector<MotionSample>& samples, MotionFrame* frames)
        {
            // 마지막 키는 다음 주기의 첫 자세다. 모델 공간 발목 이동량으로 양발이 지지를 교대하는 시점을 찾는다.
            const size_t period = samples.size() - 1;
            const Math::Vector3 travel = samples.back().pelvis - samples.front().pelvis;
            // 30 Hz 원본의 양옆 키까지 비교해 루프 봉합부의 한 보간 프레임을 지지발 교대로 오인하지 않는다.
            const size_t radius = (std::min)(size_t(2), period / 2);
            for (size_t i = 0; i < period; ++i)
            {
                const MotionSample& previous = samples[(i + period - radius) % period];
                const MotionSample& next = samples[(i + radius) % period];
                const Math::Vector3 wrap = i < radius || i + radius >= period ? travel : Math::Vector3(Math::kZero);
                const float left = Math::LengthSquare(next.modelFeet[0] - previous.modelFeet[0] + wrap);
                const float right = Math::LengthSquare(next.modelFeet[1] - previous.modelFeet[1] + wrap);
                frames[i].swing.movingFoot = left > right ? 0 : 1;
            }

            bool hasTransfer = false;
            for (size_t i = 0; i < period; ++i)
            {
                const uint32_t movingFoot = frames[i].swing.movingFoot;
                if (movingFoot == frames[(i + 1) % period].swing.movingFoot)
                {
                    continue;
                }
                hasTransfer = true;
                const Math::Vector3 separation = samples[i].modelFeet[0] - samples[i].modelFeet[1];
                FootstepMode mode = FootstepMode::Normal;
                // 믹사모 기준 자세에서 +X는 왼쪽, +Z는 전방이다. 착지 때 발의 좌우 순서와 앞뒤를 읽는다.
                if (separation.GetX() < 0.0f)
                {
                    const float forward = movingFoot == 0 ? float(separation.GetZ()) : -float(separation.GetZ());
                    mode = forward >= 0.0f ? FootstepMode::FrontCross : FootstepMode::BackCross;
                }
                size_t swingFrame = i;
                do
                {
                    frames[swingFrame].swing.mode = mode;
                    swingFrame = swingFrame == 0 ? period - 1 : swingFrame - 1;
                }
                while (frames[swingFrame].swing.movingFoot == movingFoot);
            }
            if (!hasTransfer)
            {
                for (size_t i = 0; i < period; ++i)
                {
                    frames[i].swing = {};
                }
            }
            frames[period].swing = frames[0].swing;
        }

        void AppendClipFrames(const MotionClipSelection& clip, const std::vector<MotionSample>& samples,
            std::vector<MotionFrame>& frames)
        {
            const size_t last = samples.size() - 1;
            const size_t start = frames.size();
            const bool walking = clip.usage == MotionFrameUsage::Walking;
            const Math::Vector3 travel = samples.back().pelvis - samples.front().pelvis;
            // 루프의 처음과 끝은 같은 보행 위상이다. 모델 전방(+Z) 기준 순이동으로 골반의 좌우 흔들림을 제외한다.
            const Math::Vector3 meanVelocity = travel / samples.back().time;
            const float walkingAngle = walking ? GetWalkingTravelAngleDegrees({0.0f, 1.0f},
                {meanVelocity.GetX(), meanVelocity.GetZ()}) : std::numeric_limits<float>::quiet_NaN();
            const double lookahead = MotionFeatures::PredictionTimeSeconds * static_cast<double>(last) / samples.back().time;
            for (size_t i = 0; i <= last; ++i)
            {
                const MotionSample& sample = samples[i];
                const MotionSample& previous = samples[i == 0 ? (walking ? last - 1 : 0) : i - 1];
                const MotionSample& next = samples[i == last ? (walking ? 1 : last) : i + 1];
                const float interval = walking ? 2.0f * samples.back().time / static_cast<float>(last) : next.time - previous.time;
                MotionFrame frame;
                frame.clipIndex = clip.clipIndex;
                frame.clipTimeSeconds = sample.time;
                frame.usage = clip.usage;
                frame.features.walkingAngleDegrees = walkingAngle;
                for (size_t foot = 0; foot < sample.feet.size(); ++foot)
                {
                    const Math::Vector3 velocity = (next.feet[foot] - previous.feet[foot]) / interval;
                    DirectX::XMStoreFloat3(&frame.features.footPositions[foot], sample.feet[foot]);
                    DirectX::XMStoreFloat3(&frame.features.footVelocities[foot], velocity);
                }
                // 루프 끝에서도 원본의 진행 방향과 속도를 유지해 골반의 미래 변위를 계산한다.
                const double cycles = walking ? std::floor((i + lookahead) / last) : 0.0;
                const double future = std::min(i + lookahead - cycles * last, static_cast<double>(last));
                const size_t first = static_cast<size_t>(future);
                const size_t second = std::min(first + 1, last);
                const Math::Vector3 futurePelvis = Math::Lerp(samples[first].pelvis, samples[second].pelvis,
                    static_cast<float>(future - first)) + travel * static_cast<float>(cycles);
                DirectX::XMStoreFloat3(&frame.features.futureDisplacement, sample.toHeadingSpace * (futurePelvis - sample.pelvis));
                frames.push_back(frame);
            }
            if (walking)
            {
                MarkWalkingSwings(samples, frames.data() + start);
            }
        }
    }

    bool AnimInstance::BuildMotionFrames(const std::vector<MotionClipSelection>& clips, std::string& error)
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
            database->groups[usageIndex].clips.push_back(clip);
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
        database->bindForwardInPelvis = bones.bindForwardInPelvis;

        std::vector<MotionSample> samples;
        for (size_t usageIndex = 0; usageIndex < database->groups.size(); ++usageIndex)
        {
            MotionFrameGroup& group = database->groups[usageIndex];
            group.range.firstFrameIndex = database->frames.size();
            for (const MotionClipSelection& clip : group.clips)
            {
                if (!SampleClip(*this, clip.clipIndex, skeleton, bones, samples, error))
                {
                    error = GetAnimationName(clip.clipIndex) + " - " + error;
                    return false;
                }
                AppendClipFrames(clip, samples, database->frames);
            }
            group.range.frameCount = database->frames.size() - group.range.firstFrameIndex;
        }
        m_MotionDatabase = database;
        error.clear();
        return true;
    }

    bool AnimInstance::GetMotionHeadingSpace(const Math::Matrix4& pelvis, Math::Matrix3& toHeadingSpace) const
    {
        return m_MotionDatabase && MakeHeadingSpace(pelvis, m_MotionDatabase->bindForwardInPelvis, toHeadingSpace);
    }

    float AnimInstance::GetWalkingPlaybackTime(const MotionMatchResult& match, float elapsed) const
    {
        const float duration = GetAnimationDuration(match.clipIndex);
        if (duration <= 0.0f)
        {
            return 0.0f;
        }
        const float time = match.clipTimeSeconds + elapsed;
        return time <= duration ? time : std::fmod(time, duration);
    }

    MotionMatchResult AnimInstance::FindMotionMatch(MotionFrameUsage usage, const MotionFeatures& currentFeatures,
        const WalkingSwing& swing) const
    {
        const MotionFrameRange range = GetMotionFrameRange(usage);
        const std::vector<MotionFrame>& frames = GetMotionFrames();
        MotionMatchResult best;
        int bestAngleError = std::numeric_limits<int>::max();
        const float requestedAngle = usage == MotionFrameUsage::Walking ? currentFeatures.walkingAngleDegrees :
            std::numeric_limits<float>::quiet_NaN();
        constexpr float velocityWeight = MotionFeatures::PredictionTimeSeconds * MotionFeatures::PredictionTimeSeconds;
        for (size_t offset = 0; offset < range.frameCount; ++offset)
        {
            const MotionFrame& frame = frames[range.firstFrameIndex + offset];
            if (swing.movingFoot != UINT32_MAX && (frame.swing.movingFoot != swing.movingFoot || frame.swing.mode != swing.mode))
            {
                continue;
            }
            const int angleError = WalkingAngleError(requestedAngle, frame.features.walkingAngleDegrees);
            if (angleError > bestAngleError)
            {
                continue;
            }
            float footError = 0.0f;
            for (size_t side = 0; side < currentFeatures.footPositions.size(); ++side)
            {
                footError += SquaredDistance(currentFeatures.footPositions[side], frame.features.footPositions[side]);
                footError += velocityWeight * SquaredDistance(currentFeatures.footVelocities[side], frame.features.footVelocities[side]);
            }
            // 양발 오차는 평균을 내고, 속도 차이는 예측 시간 동안의 거리로 환산한다.
            // 골반 변위에 두 배의 거리 가중치를 주어 회복할 방향을 반영한다.
            const float score = 0.5f * footError + 4.0f * SquaredDistance(currentFeatures.futureDisplacement, frame.features.futureDisplacement);
            if (std::isfinite(score) && (angleError < bestAngleError || score < best.score))
            {
                bestAngleError = angleError;
                best.clipIndex = frame.clipIndex;
                best.clipTimeSeconds = frame.clipTimeSeconds;
                best.score = score;
                best.requestedAngleDegrees = requestedAngle;
                best.clipAngleDegrees = frame.features.walkingAngleDegrees;
            }
        }
        return best;
    }
}
