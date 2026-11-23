#include "FootSoleDebug.h"
#include "Components/SkeletalMeshComponent.h"
#include "PhysicsWorld.h"
#include "imgui.h"
#include <algorithm>
#include <cmath>
#include <cstdio>

using namespace Math;
using namespace physx;

namespace
{
    constexpr PxU32 FootColors[] = {0xff44ccff, 0xffffbb44};
    constexpr PxU32 HitColor = 0xff55ff77, MissColor = 0xffcc5566;
    constexpr PxU32 HullColor = 0xffee66ff, BufferColor = 0xffffffff;
    constexpr PxU32 ComColor = 0xffffdd44, OutsideColor = 0xffff0000, UnknownColor = 0xff000000;
    constexpr PxU32 PredictionColor = 0xff00ffff, TargetColor = 0xffff8800, RawTargetColor = 0xffaaaaaa;
    constexpr PxU32 MotionTargetColor = 0xffff66cc;
    constexpr PxU32 LandingFootColor = 0xff3388ff;
    constexpr PxU32 LandingRawComColor = 0xffbb88ff;
    constexpr PxU32 LandingClampedComColor = 0xff88ff44;
    constexpr PxU32 LandingComForbiddenColor = 0xffdd4444;
    constexpr PxU32 LandingComAllowedColor = 0xff55cc77;
    constexpr PxU32 LandingComHysteresisColor = 0xffffaa33;
    constexpr PxU32 LandingComPreviewColor = 0xff889099;
    constexpr PxU32 IKStepColor = 0xffffff33, IKSupportColor = 0xff44dd88, KneeHintColor = 0xffcc88ff;
    constexpr PxU32 RecoveryModeColors[] = {0xff000000, 0xffff9933, 0xffbb66ff};
    constexpr PxU32 RecoveryCaptureColor = 0xff33dd77;
    constexpr PxU32 RecoveryTargetColor = 0xffeeffff;
    constexpr size_t NormalModeIndex = static_cast<size_t>(PARS::FootstepMode::Normal);

    const PARS::FootstepRecoverySnapshot* VisibleRecovery(const PARS::SkeletalMeshComponent& mesh)
    {
        const PARS::FootstepSnapshot& step = mesh.GetFootSupport().GetSnapshot().footstep;
        if (step.requestedFoot < 0 || step.requestedFoot > 1)
        {
            for (const PARS::FootstepTarget& candidate : step.candidates)
            {
                if (candidate.recovery.status == PARS::FootstepRecoveryStatus::Preview ||
                    candidate.recovery.normalReach.radius > 0)
                {
                    return &candidate.recovery;
                }
            }
            return nullptr;
        }
        const PARS::FootstepRecoverySnapshot& recovery = step.candidates[step.requestedFoot].recovery;
        return recovery.status != PARS::FootstepRecoveryStatus::Unavailable || recovery.normalReach.radius > 0 ?
            &recovery : nullptr;
    }

    bool ShowCrossReachRegions(const PARS::FootstepRecoverySnapshot& recovery)
    {
        if (recovery.status == PARS::FootstepRecoveryStatus::Preview)
        {
            return recovery.overlapAreas[NormalModeIndex] <= 0.0f;
        }
        return recovery.status != PARS::FootstepRecoveryStatus::Unavailable &&
            (!recovery.hasTarget || recovery.mode != PARS::FootstepMode::Normal ||
                (!recovery.modelFeasible && !recovery.targetInCaptureMargin));
    }

    void AppendRegion(std::vector<PxDebugTriangle>& triangles, const PARS::FootstepPolygon& polygon,
        PxU32 color, float lift)
    {
        for (size_t index = 1; index + 1 < polygon.size(); ++index)
        {
            const PxVec3 offset(0, lift, 0);
            triangles.emplace_back(polygon[0] + offset, polygon[index] + offset, polygon[index + 1] + offset, color);
        }
    }

    void AppendRegion(std::vector<PxDebugLine>& lines, const PARS::FootstepPolygon& polygon,
        PxU32 color, float lift)
    {
        for (size_t index = 0; index < polygon.size(); ++index)
        {
            const PxVec3 offset(0, lift, 0);
            lines.emplace_back(polygon[index] + offset, polygon[(index + 1) % polygon.size()] + offset, color);
        }
    }

    template <typename Primitive>
    void AppendVisibleRecoveryRegions(std::vector<Primitive>& primitives, const PARS::FootstepRecoverySnapshot& recovery,
        float captureLift, float reachLift)
    {
        // 검정은 고관절 도달 원에서 지지발 쪽 최소 간격을 제외한 범위다.
        AppendRegion(primitives, recovery.normalReach.BuildPolygon(), RecoveryModeColors[NormalModeIndex], reachLift);
        for (const PARS::FootstepPolygon& piece : recovery.capture)
        {
            AppendRegion(primitives, piece, RecoveryCaptureColor, captureLift);
        }
        if (ShowCrossReachRegions(recovery))
        {
            for (size_t mode = static_cast<size_t>(PARS::FootstepMode::FrontCross); mode < recovery.reachable.size(); ++mode)
            {
                const float lift = reachLift + static_cast<float>(mode) * .002f;
                AppendRegion(primitives, recovery.reachable[mode], RecoveryModeColors[mode], lift);
            }
        }
    }

    const char* FootLabel(int foot)
    {
        return foot == 0 ? u8"왼발" : foot == 1 ? u8"오른발" : u8"없음";
    }

    std::string FootstepDecisionText(const PARS::FootstepSnapshot& step, const PARS::FootstepRecoverySnapshot* decision)
    {
        std::string text;
        for (size_t side = 0; side < step.feet.size(); ++side)
        {
            const PARS::FootstepCommand& command = step.feet[side];
            if (!command.IsInProgress())
            {
                continue;
            }
            text += text.empty() ? u8"[딛기 명령] " : " / ";
            text += FootLabel(static_cast<int>(side));
            text += ' ';
            text += PARS::FootstepModeLabel(command.target.recovery.mode);
            const PARS::FootstepRecoveryStatus status = step.candidates[side].recovery.status;
            if (!command.justStarted && (status == PARS::FootstepRecoveryStatus::PoseRejected ||
                status == PARS::FootstepRecoveryStatus::NoReach))
            {
                text += u8" (기존 목표 유지)";
            }
        }
        if (!decision)
        {
            return text;
        }
        if (text.empty())
        {
            text = u8"[딛기 명령] 없음";
        }
        text += u8"\n[착지 탐색] ";
        if (decision->hasTarget)
        {
            text += PARS::FootstepModeLabel(decision->mode);
            text += u8" · ";
        }
        char result[160];
        std::snprintf(result, sizeof(result), u8"%s · 최대 %u스텝 · %.3fms",
            PARS::FootstepRecoveryStatusLabel(decision->status), decision->steps, decision->elapsedMilliseconds);
        text += result;
        return text;
    }

    const char* LandingDirectionLabel(PARS::FootstepLandingDirection direction)
    {
        switch (direction)
        {
        case PARS::FootstepLandingDirection::Front:
            return u8"앞 경계";
        case PARS::FootstepLandingDirection::Back:
            return u8"뒤 경계";
        default:
            return u8"선택 없음";
        }
    }

    uint32_t AppendLegContacts(PARS::SkeletalMeshComponent* mesh, const char* label, std::string& text)
    {
        if (!mesh || !mesh->GetRagdoll().IsSimulating())
        {
            return 0;
        }
        const PARS::Ragdoll& ragdoll = mesh->GetRagdoll();
        const PARS::FootSupportSnapshot& sample = mesh->GetFootSupport().GetSnapshot();
        const bool leftMoving = sample.footstep.feet[0].IsInProgress();
        const bool rightMoving = sample.footstep.feet[1].IsInProgress();
        if (!leftMoving && !rightMoving)
        {
            return 0;
        }

        const std::vector<int>& nodes = ragdoll.GetBoneNodes();
        std::array<std::array<size_t, 3>, 2> bodies{};
        for (size_t side = 0; side < bodies.size(); ++side)
        {
            const PARS::FootLegSetup& leg = sample.feet[side].leg;
            const std::array<int, 3> legNodes{leg.hipNode, leg.kneeNode, leg.ankleNode};
            for (size_t part = 0; part < legNodes.size(); ++part)
            {
                const std::vector<int>::const_iterator found = std::find(nodes.begin(), nodes.end(), legNodes[part]);
                bodies[side][part] = static_cast<size_t>(found - nodes.begin());
            }
        }

        const size_t movingSide = leftMoving ? 0 : 1;
        const size_t otherSide = 1 - movingSide;
        const char* const sideLabels[] = {u8"왼쪽", u8"오른쪽"};
        const char* const partLabels[] = {u8"허벅지", u8"종아리", u8"발"};
        PhysicsGroundContact ground;
        const bool otherGrounded = ragdoll.GetBodyGroundContact(bodies[otherSide][2], ground);
        // 양쪽에 이동 명령이 있거나 반대 발이 공중에 있으면 지지 중이라고 표시하지 않는다.
        const char* otherRole = sample.footstep.feet[otherSide].IsInProgress() ? u8"이동하는" :
            otherGrounded ? u8"지지하는" : u8"반대쪽";
        uint32_t contacts = 0;
        for (size_t movingPart = 0; movingPart < bodies[movingSide].size(); ++movingPart)
        {
            for (size_t otherPart = 0; otherPart < bodies[otherSide].size(); ++otherPart)
            {
                if (!ragdoll.AreBodiesInContact(bodies[movingSide][movingPart], bodies[otherSide][otherPart]))
                {
                    continue;
                }
                // 이동/지지 역할이 바뀌어도 같은 왼쪽·오른쪽 부위는 같은 접촉으로 식별한다.
                const size_t leftPart = movingSide == 0 ? movingPart : otherPart;
                const size_t rightPart = movingSide == 0 ? otherPart : movingPart;
                contacts |= uint32_t(1) << (leftPart * 3 + rightPart);
                if (!text.empty())
                {
                    text += '\n';
                }
                text += '[';
                text += label;
                text += u8"] 이동하는 다리의 (";
                text += sideLabels[movingSide];
                text += u8", ";
                text += partLabels[movingPart];
                text += u8")가 ";
                text += otherRole;
                text += u8" 다리의 (";
                text += sideLabels[otherSide];
                text += u8", ";
                text += partLabels[otherPart];
                text += u8")에 닿음";
            }
        }
        return contacts;
    }

    Vector3 ToMath(const PxVec3& point)
    {
        return Vector3(point.x, point.y, point.z);
    }

    void DashedLine(std::vector<PxDebugLine>& lines, const PxVec3& a, const PxVec3& b)
    {
        const PxVec3 delta = b - a;
        const float length = delta.magnitude();
        if (length < 1e-6f)
        {
            return;
        }
        const PxVec3 direction = delta / length, offset(0, .004f, 0);
        for (float distance = 0; distance < length; distance += .06f)
        {
            lines.emplace_back(a + offset + direction * distance,
                a + offset + direction * PxMin(distance + .035f, length), BufferColor);
        }
    }

    PxVec3 ToPx(Vector3 point)
    {
        return PxVec3(float(point.GetX()), float(point.GetY()), float(point.GetZ()));
    }

    // 표시만 살짝 올려 바닥과 선의 깊이가 겹치는 것을 피한다.
    void Line(std::vector<PxDebugLine>& lines, Vector3 a, Vector3 b, PxU32 color)
    {
        const Vector3 offset(0, .002f, 0);
        lines.emplace_back(ToPx(a + offset), ToPx(b + offset), color);
    }

    void Point(std::vector<PxDebugLine>& lines, Vector3 point, PxU32 color, float radius = .008f)
    {
        Line(lines, point - Vector3(radius, 0, 0), point + Vector3(radius, 0, 0), color);
        Line(lines, point - Vector3(0, 0, radius), point + Vector3(0, 0, radius), color);
    }

    void LandingFoot(std::vector<PxDebugLine>& lines, const PARS::FootSoleSample& foot, const PxTransform& target)
    {
        const float halfWidth = (foot.corners[1] - foot.corners[0]).magnitude() * .5f;
        const float halfLength = (foot.corners[2] - foot.corners[1]).magnitude() * .5f;
        if (!foot.available || !target.isSane() || !std::isfinite(halfWidth) || !std::isfinite(halfLength) ||
            halfWidth <= .001f || halfLength <= .001f)
        {
            return;
        }
        // 실제 발바닥 크기를 유지하고 지지 평면의 법선 방향으로 표시만 4mm 올린다.
        const PxVec3 offset = target.q.rotate(PxVec3(0, .004f, 0));
        for (int stroke = -1; stroke <= 1; ++stroke)
        {
            // 1픽셀 선을 겹쳐 바닥의 다른 디버그 선 사이에서도 착지 윤곽이 보이게 한다.
            const float w = halfWidth + stroke * .001f;
            const float l = halfLength + stroke * .001f;
            const std::array<PxVec3, 4> corners
            {
                target.transform(PxVec3(-w, 0, -l)) + offset,
                target.transform(PxVec3(w, 0, -l)) + offset,
                target.transform(PxVec3(w, 0, l)) + offset,
                target.transform(PxVec3(-w, 0, l)) + offset
            };
            for (size_t corner = 0; corner < corners.size(); ++corner)
            {
                lines.emplace_back(corners[corner], corners[(corner + 1) % corners.size()], LandingFootColor);
            }
        }
        // 발바닥 로컬 +Z가 발끝 방향이다. 화살표는 이동 방향이 아니라 착지할 발의 방향을 뜻한다.
        const PxVec3 tip = target.transform(PxVec3(0, 0, halfLength * .7f)) + offset;
        lines.emplace_back(target.transform(PxVec3(0, 0, -halfLength * .5f)) + offset, tip, LandingFootColor);
        lines.emplace_back(target.transform(PxVec3(-halfWidth * .5f, 0, halfLength * .3f)) + offset, tip, LandingFootColor);
        lines.emplace_back(target.transform(PxVec3(halfWidth * .5f, 0, halfLength * .3f)) + offset, tip, LandingFootColor);
    }

    void ProjectionMarker(std::vector<PxDebugLine>& lines, Vector3 point, PxU32 color, bool bold)
    {
        constexpr float radius = .025f;
        // GPU 라인은 1픽셀이므로 촘촘히 겹쳐 원과 십자에 12mm 폭을 준다.
        const int strokes = bold ? 13 : 1;
        const int segments = bold ? 48 : 24;
        for (int stroke = 0; stroke < strokes; ++stroke)
        {
            const float offset = bold ? (stroke - 6) * .001f : 0;
            Line(lines, point + Vector3(-radius, 0, offset), point + Vector3(radius, 0, offset), color);
            Line(lines, point + Vector3(offset, 0, -radius), point + Vector3(offset, 0, radius), color);
            const float ringRadius = radius + offset;
            for (int segment = 0; segment < segments; ++segment)
            {
                const float a = XM_2PI * segment / segments, b = XM_2PI * (segment + 1) / segments;
                Line(lines, point + Vector3(std::cos(a) * ringRadius, 0, std::sin(a) * ringRadius),
                    point + Vector3(std::cos(b) * ringRadius, 0, std::sin(b) * ringRadius), color);
            }
        }
    }

    void LandingComSector(std::vector<PxDebugLine>& lines, const PxVec3& origin, const PxVec3& right,
        const PxVec3& forward, float centerAngle, float halfAngle, float radius, PxU32 color, bool hatched)
    {
        if (halfAngle <= 0)
        {
            return;
        }
        PxVec3 previous(0);
        for (int ray = 0; ray <= 16; ++ray)
        {
            const float angle = centerAngle + halfAngle * (static_cast<float>(ray) / 8.0f - 1.0f);
            const PxVec3 direction = right * std::cos(angle) + forward * std::sin(angle);
            const PxVec3 end = origin + direction * radius;
            if (hatched || ray == 0 || ray == 8 || ray == 16)
            {
                Line(lines, ToMath(origin), ToMath(end), color);
            }
            if (ray > 0)
            {
                Line(lines, ToMath(previous), ToMath(end), color);
            }
            previous = end;
        }
    }

    bool CaptureLandingSole(const PARS::SkeletalMeshComponent& mesh, const PARS::FootSoleSample& foot,
        std::array<PxVec3, 4>& corners, bool& grounded)
    {
        const PARS::Ragdoll& ragdoll = mesh.GetRagdoll();
        const std::vector<int>& nodes = ragdoll.GetBoneNodes();
        const std::vector<int>::const_iterator found = std::find(nodes.begin(), nodes.end(), foot.boneNode);
        const size_t body = static_cast<size_t>(found - nodes.begin());
        if (!foot.available || found == nodes.end() || body >= ragdoll.GetBodyCount())
        {
            return false;
        }
        // 제어용 스냅샷을 고치지 않고 현재 완료된 물리 자세에서 사각형을 다시 배치한다.
        const Matrix4 inverseSample = Invert(foot.soleWorld);
        const Matrix4 current = ragdoll.GetBoneWorld(body) * foot.soleInBone;
        for (size_t corner = 0; corner < corners.size(); ++corner)
        {
            const Vector3 local(inverseSample * ToMath(foot.corners[corner]));
            corners[corner] = ToPx(Vector3(current * local));
            if (!corners[corner].isFinite())
            {
                return false;
            }
        }
        PhysicsGroundContact contact;
        grounded = ragdoll.GetBodyGroundContact(body, contact);
        return true;
    }

    void LandingComRegion(std::vector<PxDebugLine>& lines, const PARS::SkeletalMeshComponent& mesh,
        const PxVec3& currentCom)
    {
        const PARS::FootSupportSnapshot& sample = mesh.GetFootSupport().GetSnapshot();
        const PARS::FootstepSettings& settings = mesh.GetFootSupport().GetFootstepSettings();
        const PARS::FootstepSnapshot& step = sample.footstep;
        const bool hasPredictionRegion = step.hasPrediction && step.requestedFoot >= 0 &&
            step.requestedFoot < static_cast<int>(step.feet.size());
        if (!currentCom.isFinite() || !std::isfinite(settings.landingComClampHysteresisDegrees) ||
            settings.landingComClampHysteresisDegrees < 0 ||
            settings.landingComClampHysteresisDegrees >= 90)
        {
            return;
        }
        // 청록색 예측과 같은 물리 스텝의 원점·사각형을 사용해 표시 기준을 일치시킨다.
        PxVec3 origin = hasPredictionRegion ? step.startCom.origin : currentCom;
        if (!hasPredictionRegion)
        {
            if (sample.hasComProjection)
            {
                origin.y = sample.projectedCenterOfMass.y;
            }
            else if (step.hasGroundPlane)
            {
                origin.y = step.groundPosition.y;
            }
            else
            {
                return;
            }
        }
        const Matrix4 component(mesh.GetComponentTransform());
        PxVec3 forward = ToPx(Vector3(component.GetZ()));
        forward.y = 0;
        if (!origin.isFinite() || !forward.isFinite() || forward.normalize() <= 1e-6f)
        {
            return;
        }
        origin.y += .004f;
        const PxVec3 right(forward.z, 0, -forward.x);
        const PxU32 forbiddenColor = settings.landingComClampEnabled ? LandingComForbiddenColor : LandingComPreviewColor;
        const PxU32 allowedColor = settings.landingComClampEnabled ? LandingComAllowedColor : LandingComPreviewColor;
        const PxU32 hysteresisColor = settings.landingComClampEnabled ? LandingComHysteresisColor : LandingComPreviewColor;
        int movingFoot = hasPredictionRegion ? step.requestedFoot : -1;
        for (size_t side = 0; movingFoot < 0 && side < step.feet.size(); ++side)
        {
            if (sample.footstep.feet[side].IsInProgress())
            {
                movingFoot = static_cast<int>(side);
                break;
            }
        }
        std::array<PARS::FootstepLandingRegion, 2> regions;
        std::array<bool, 2> visible{};
        float radius = .6f;
        for (size_t side = 0; side < sample.feet.size(); ++side)
        {
            if (movingFoot >= 0 && side != static_cast<size_t>(1 - movingFoot))
            {
                continue;
            }
            if (hasPredictionRegion)
            {
                regions[side] = step.startCom.region;
                regions[side].origin = origin;
                for (PxVec3& corner : regions[side].corners)
                {
                    corner.y = origin.y;
                }
            }
            else
            {
                std::array<PxVec3, 4> corners{};
                bool grounded = false;
                if (!CaptureLandingSole(mesh, sample.feet[side], corners, grounded) || (mesh.IsRagdoll() && !grounded))
                {
                    continue;
                }
                // 시작 예측의 기준 발이 아직 없을 때만 현재 양발 구간을 미리 보여준다.
                regions[side] = PARS::BuildFootstepLandingRegion(origin, corners, forward);
            }
            visible[side] = regions[side].valid || regions[side].containsCom;
            if (visible[side])
            {
                for (const PxVec3& corner : regions[side].corners)
                {
                    radius = (std::max)(radius, (corner - origin).magnitude() + .1f);
                }
            }
        }
        PxVec3 previous = origin + right * radius;
        for (int segment = 1; segment <= 64; ++segment)
        {
            const float angle = PxTwoPi * static_cast<float>(segment) / 64;
            const PxVec3 end = origin + (right * std::cos(angle) + forward * std::sin(angle)) * radius;
            Line(lines, ToMath(previous), ToMath(end), allowedColor);
            previous = end;
        }
        for (size_t side = 0; side < regions.size(); ++side)
        {
            const PARS::FootstepLandingRegion& region = regions[side];
            if (!visible[side])
            {
                continue;
            }
            for (size_t corner = 0; corner < region.corners.size(); ++corner)
            {
                Line(lines, ToMath(region.corners[corner]),
                    ToMath(region.corners[(corner + 1) % region.corners.size()]), region.containsCom ? ComColor : forbiddenColor);
                Line(lines, ToMath(origin), ToMath(region.corners[corner]), RawTargetColor);
            }
            if (!region.valid)
            {
                continue;
            }
            const float centerAngle = std::atan2(region.centerRay.dot(forward), region.centerRay.dot(right));
            const float halfHysteresisAngle = (std::min)(region.halfAngleRadians,
                settings.landingComClampHysteresisDegrees * PxPi / 180.0f);
            LandingComSector(lines, origin, right, forward, centerAngle, region.halfAngleRadians,
                radius, forbiddenColor, true);
            LandingComSector(lines, origin, right, forward, centerAngle, halfHysteresisAngle,
                radius, hysteresisColor, false);
            Line(lines, ToMath(origin), ToMath(origin + region.frontRay * radius), allowedColor);
            Line(lines, ToMath(origin), ToMath(origin + region.backRay * radius), allowedColor);
        }
        ProjectionMarker(lines, ToMath(origin), ComColor, false);
        const PxVec3 tip = origin + forward * .55f;
        Line(lines, ToMath(origin), ToMath(tip), LandingFootColor);
        Line(lines, ToMath(tip - forward * .06f + right * .04f), ToMath(tip), LandingFootColor);
        Line(lines, ToMath(tip - forward * .06f - right * .04f), ToMath(tip), LandingFootColor);
    }
}

uint32_t FootSoleDebug::GetLegContacts(PARS::SkeletalMeshComponent* dummy, std::string& text) const
{
    text.clear();
    return AppendLegContacts(dummy, u8"더미", text);
}

std::string FootSoleDebug::GetFollowedAnimationText(const PARS::SkeletalMeshComponent& mesh)
{
    const PARS::AnimInstance& animation = mesh.GetAnimInstance();
    const uint32_t clip = animation.GetCurrentAnimationIndex();
    if (clip >= animation.GetAnimationCount())
    {
        return u8"없음";
    }

    char timing[128];
    std::snprintf(timing, sizeof(timing), " (%.2f / %.2f s)", animation.GetAnimationTime(), animation.GetAnimationDuration(clip));
    std::string followed = animation.GetAnimationName(clip) + timing;
    const uint32_t flail = mesh.GetImpactFlailClip();
    if (flail < animation.GetAnimationCount())
    {
        std::snprintf(timing, sizeof(timing), " (%.2f / %.2f s)", mesh.GetImpactFlailTime(), animation.GetAnimationDuration(flail));
        followed += std::string(u8" · 상체 ") + animation.GetAnimationName(flail) + timing;
    }
    return followed;
}

std::string FootSoleDebug::GetLegPredictionText(const PARS::SkeletalMeshComponent& mesh) const
{
    const PARS::FootstepCollisionSnapshot& prediction = mesh.GetFootstepCollisionSnapshot();
    const PARS::FootSupportSnapshot& support = mesh.GetFootSupport().GetSnapshot();
    std::string decisionText = FootstepDecisionText(support.footstep, VisibleRecovery(mesh));
    if (!decisionText.empty())
    {
        decisionText += '\n';
    }
    decisionText += u8"[추종 애니메이션] ";
    decisionText += GetFollowedAnimationText(mesh);
    decisionText += '\n';
    if (!mesh.GetFootSupport().GetFootstepSettings().pathAvoidanceEnabled)
    {
        return decisionText + u8"[더미] 이동 경로 충돌 예측 꺼짐";
    }
    if (!support.footstep.IsInProgress())
    {
        return decisionText + u8"[더미] 목표 경로 예측: 이동 명령 없음";
    }
    if (prediction.status == PARS::FootstepCollisionStatus::Unavailable ||
        prediction.status == PARS::FootstepCollisionStatus::Inactive ||
        prediction.revision != support.footstep.revision || prediction.completedStep != support.footstep.completedStep)
    {
        return decisionText + u8"[더미] 목표 경로 예측: 검사 불가 (유효한 다리 자세·충돌 형상 필요)";
    }

    const PARS::FootstepAvoidanceSnapshot& avoidance = mesh.GetFootstepAvoidanceSnapshot();
    const char* state = prediction.status == PARS::FootstepCollisionStatus::Collision ? u8"겹침 예상" :
        prediction.status == PARS::FootstepCollisionStatus::Clear ? u8"계획 경로 구간 통과" : u8"미확인";
    const char* route = avoidance.status == PARS::FootstepAvoidanceStatus::Detouring ? u8"우회 적용" :
        avoidance.status == PARS::FootstepAvoidanceStatus::Failed ? u8"회피 실패" : u8"기존 경로";
    char summary[512];
    std::snprintf(summary, sizeof(summary),
        u8"[더미] %s · %s · 후보 %u개 · 전체 %u구간 / %u쌍 · %.3fms · 추종 오차 %.1fcm",
        route, state, avoidance.candidatesChecked, avoidance.totalIntervals, avoidance.totalQueries,
        prediction.elapsedMilliseconds, prediction.maximumTrackingError * 100);
    std::string text = decisionText + summary;
    if (prediction.assumesTakeoff)
    {
        text += u8" (들기 완료 후 이륙 가정)";
    }
    if (prediction.reachLimited)
    {
        text += u8" (다리 도달 범위 제한 있음)";
    }
    if (prediction.jointLimited)
    {
        text += u8" (관절 각도 제한 있음)";
    }
    if (prediction.budgetExhausted || avoidance.searchBudgetExhausted)
    {
        text += u8" (검사 예산 도달)";
    }
    if (prediction.status == PARS::FootstepCollisionStatus::Uncertain)
    {
        text += u8"\n[미확인] 충돌 없음이 확인되지 않아 우회 후보로 채택하지 않음";
    }

    const size_t movingSide = prediction.moving[0] ? 0 : 1;
    const size_t otherSide = 1 - movingSide;
    const char* const sideLabels[] = {u8"왼쪽", u8"오른쪽"};
    const char* const partLabels[] = {u8"허벅지", u8"종아리", u8"발"};
    const char* otherRole = prediction.moving[otherSide] ? u8"이동하는" :
        support.feet[otherSide].contact.grounded ? u8"지지하는" : u8"반대쪽";
    uint32_t displayed = 0;
    for (uint32_t index = 0; index < prediction.sampleCount; ++index)
    {
        const PARS::FootstepCollisionSample& sample = prediction.samples[index];
        for (uint32_t pair = 0; pair < 9; ++pair)
        {
            const uint32_t bit = uint32_t(1) << pair;
            if (!(sample.overlapPairs & bit) || (displayed & bit))
            {
                continue;
            }
            displayed |= bit;
            const uint32_t movingPart = movingSide == 0 ? pair / 3 : pair % 3;
            const uint32_t otherPart = movingSide == 0 ? pair % 3 : pair / 3;
            char message[512];
            std::snprintf(message, sizeof(message),
                u8"\n[예측 %u/%u · %.0fms 뒤] 이동하는 다리의 (%s, %s)가 %s 다리의 (%s, %s)와 겹침 예상",
                index + 1, prediction.sampleCount, sample.timeAhead * 1000,
                sideLabels[movingSide], partLabels[movingPart], otherRole, sideLabels[otherSide], partLabels[otherPart]);
            text += message;
        }
    }
    const uint32_t remaining = prediction.overlapPairs & ~displayed;
    if (remaining != 0)
    {
        char message[256];
        std::snprintf(message, sizeof(message), u8"\n[구간 검사 · %.1fms 뒤] 표시 자세 사이에서 다리 겹침 예상",
            prediction.firstRiskTime * 1000);
        text += message;
    }
    return text;
}

void FootSoleDebug::DrawContactOverlay(const std::string& text, bool touching, float topInset) const
{
    // 툴바나 설정 창과 독립적인 화면 상단 표시이며 마우스 입력을 가로채지 않는다.
    const ImGuiViewport* viewport = ImGui::GetMainViewport();
    const float wrapWidth = (std::max)(1.0f, (std::min)(1000.0f, viewport->Size.x - 48.0f));
    const ImVec2 textSize = ImGui::CalcTextSize(text.c_str(), nullptr, false, wrapWidth);
    const ImVec2 position(viewport->Pos.x + (viewport->Size.x - textSize.x) * .5f,
        viewport->Pos.y + topInset + 14.0f);
    ImDrawList* draw = ImGui::GetForegroundDrawList();
    draw->AddRectFilled(ImVec2(position.x - 12.0f, position.y - 6.0f),
        ImVec2(position.x + textSize.x + 12.0f, position.y + textSize.y + 6.0f),
        IM_COL32(12, 16, 22, 205), 5.0f);
    const ImU32 color = touching ? IM_COL32(255, 192, 96, 255) : IM_COL32(200, 208, 220, 255);
    draw->AddText(ImGui::GetFont(), ImGui::GetFontSize(), position, color,
        text.c_str(), nullptr, wrapWidth);
}

void FootSoleDebug::BuildLines(const PARS::SkeletalMeshComponent& mesh, std::vector<PxDebugLine>& lines) const
{
    if (!m_Visible)
    {
        return;
    }
    const PARS::FootSupportSnapshot& sample = mesh.GetFootSupport().GetSnapshot();
    const PARS::FootstepRecoverySnapshot* recovery = VisibleRecovery(mesh);
    if (m_ShowRecoveryRegions && recovery)
    {
        AppendVisibleRecoveryRegions(lines, *recovery, .009f, .012f);
        if (recovery->hasTarget)
        {
            Point(lines, ToMath(recovery->target + PxVec3(0, .025f, 0)), RecoveryTargetColor, .04f);
        }
        if (recovery->predictionTime > 0)
        {
            Point(lines, ToMath(recovery->icp + PxVec3(0, .03f, 0)), RecoveryCaptureColor, .03f);
        }
    }
    Vector3 currentCenter(kZero);
    float currentMass = 0;
    const bool hasCurrentCom = mesh.GetRagdoll().GetCenterOfMass(currentCenter, currentMass);
    const PxVec3 currentCom = ToPx(currentCenter);
    for (size_t side = 0; side < sample.feet.size(); ++side)
    {
        const PARS::FootSoleSample& foot = sample.feet[side];
        if (!foot.available)
        {
            continue;
        }
        for (size_t i = 0; i < foot.corners.size(); ++i)
        {
            const size_t next = (i + 1) % foot.corners.size();
            const Vector3 corner = ToMath(foot.corners[i]), projected = ToMath(foot.projections[i]);
            if (m_ShowRays)
            {
                Line(lines, corner, projected, foot.projected[i] ? HitColor : MissColor);
            }
            Point(lines, corner, FootColors[side]);
            Line(lines, corner, ToMath(foot.corners[next]), FootColors[side]);
            if (foot.projected[i])
            {
                Point(lines, projected, HitColor);
            }
            if (foot.projected[i] && foot.projected[next])
            {
                Line(lines, projected, ToMath(foot.projections[next]), HitColor);
            }
        }
    }
    if (m_ShowHull)
    {
        for (size_t i = 0; i < sample.hull.size(); ++i)
        {
            const Vector3 point = ToMath(sample.hull[i]);
            Line(lines, point, ToMath(sample.hull[(i + 1) % sample.hull.size()]), HullColor);
            Point(lines, point, HullColor);
        }
    }
    if (m_ShowBuffer)
    {
        const std::vector<physx::PxVec3>& buffer = sample.balance.bufferHull;
        for (size_t i = 0; i < buffer.size(); ++i)
        {
            DashedLine(lines, buffer[i], buffer[(i + 1) % buffer.size()]);
        }
        if (!buffer.empty())
        {
            Point(lines, ToMath(sample.balance.center), BufferColor);
        }
    }
    if (m_ShowCenterOfMass && hasCurrentCom)
    {
        const Vector3 center = currentCenter;
        Point(lines, center, ComColor, .025f);
        Line(lines, center - Vector3(0, .025f, 0), center + Vector3(0, .025f, 0), ComColor);
        if (sample.hasComProjection)
        {
            const Vector3 projected(currentCom.x, sample.projectedCenterOfMass.y, currentCom.z);
            const bool inside = PARS::IsInsideFootSupportHull(sample.hull, currentCom);
            const PxU32 color = sample.hull.empty() ? UnknownColor : inside ? HitColor : OutsideColor;
            Line(lines, center, projected, ComColor);
            ProjectionMarker(lines, projected, color, sample.hull.empty() || !inside);
        }
    }
    if (m_ShowLandingComClamp && hasCurrentCom)
    {
        LandingComRegion(lines, mesh, currentCom);
        const PARS::FootstepSnapshot& step = sample.footstep;
        for (size_t side = 0; side < step.candidates.size(); ++side)
        {
            if (step.hasPrediction && step.requestedFoot >= 0 && step.requestedFoot != static_cast<int>(side))
            {
                continue;
            }
            const PARS::FootstepTarget& target = step.candidates[side];
            const PARS::FootstepComPrediction& landing = target.landingCom;
            if (!landing.valid)
            {
                continue;
            }
            ProjectionMarker(lines, ToMath(landing.raw), LandingRawComColor, false);
            Point(lines, ToMath(landing.clamped), LandingClampedComColor, .03f);
            Line(lines, ToMath(landing.origin), ToMath(landing.raw), LandingRawComColor);
            Line(lines, ToMath(landing.raw), ToMath(landing.clamped), LandingClampedComColor);
            if (m_ShowFootstep && target.valid)
            {
                Line(lines, ToMath(landing.clamped), ToMath(target.raw), RawTargetColor);
            }
        }
    }
    if (m_ShowLandingFoot)
    {
        const PARS::FootstepSnapshot& step = sample.footstep;
        const PARS::FootstepIKSnapshot& ik = mesh.GetFootstepIKSnapshot();
        for (size_t side = 0; side < step.feet.size(); ++side)
        {
            const PARS::FootstepCommand& command = step.feet[side];
            if (command.IsInProgress() && ik.status == PARS::FootstepIKStatus::Applied &&
                ik.revision == sample.contactRevision && ik.legs[side].valid)
            {
                // 발마다 마지막 착지점을 보존하며 이동 궤적의 높이는 포함하지 않는다.
                const PxTransform target(command.target.position, ik.legs[side].requestedSole.q);
                LandingFoot(lines, sample.feet[side], target);
            }
        }
    }
    if (m_ShowFootstep)
    {
        const PARS::FootstepCollisionSnapshot& prediction = mesh.GetFootstepCollisionSnapshot();
        if (mesh.GetFootSupport().GetFootstepSettings().pathAvoidanceEnabled && prediction.sampleCount > 0 &&
            prediction.revision == sample.footstep.revision && prediction.completedStep == sample.footstep.completedStep)
        {
            for (size_t side = 0; side < prediction.moving.size(); ++side)
            {
                if (!prediction.moving[side])
                {
                    continue;
                }
                for (uint32_t index = 0; index < prediction.sampleCount; ++index)
                {
                    const PARS::FootstepCollisionSample& future = prediction.samples[index];
                    const PxU32 color = future.overlapPairs ? OutsideColor : PredictionColor;
                    const Vector3 position = ToMath(future.solePositions[side]);
                    Point(lines, position, color, .018f);
                    if (index > 0)
                    {
                        Line(lines, ToMath(prediction.samples[index - 1].solePositions[side]), position, color);
                    }
                }
            }
        }
        const std::vector<physx::PxVec3>& referenceHull = mesh.GetRecoveryReferenceHull();
        for (size_t i = 0; i < referenceHull.size(); ++i)
        {
            Line(lines, ToMath(referenceHull[i]), ToMath(referenceHull[(i + 1) % referenceHull.size()]), 0xff88bbff);
        }
        const PARS::FootstepSnapshot& step = sample.footstep;
        if (step.hasPrediction)
        {
            Line(lines, ToMath(step.startCom.origin), ToMath(step.predictedCom), PredictionColor);
            ProjectionMarker(lines, ToMath(step.predictedCom), PredictionColor, false);
        }
        for (size_t side = 0; side < sample.feet.size(); ++side)
        {
            const PARS::FootContactSample& contact = sample.feet[side].contact;
            if (contact.grounded)
            {
                Point(lines, ToMath(contact.position), HitColor, .015f);
                Line(lines, ToMath(contact.position), ToMath(contact.position + contact.normal * .12f), HitColor);
            }
            const PARS::FootstepTarget& target = step.candidates[side];
            if (!target.valid)
            {
                continue;
            }
            Point(lines, ToMath(target.raw), RawTargetColor, .012f);
            Point(lines, ToMath(target.position), TargetColor, .025f);
            Line(lines, ToMath(target.position), ToMath(target.position + target.normal * .08f), TargetColor);
        }
        for (size_t side = 0; side < step.feet.size(); ++side)
        {
            const PARS::FootstepCommand& command = step.feet[side];
            const Vector3 foot(sample.feet[side].soleWorld.GetW());
            if (command.hasTarget)
            {
                // 연결선은 목표의 대응 관계이며 실제 이동 궤적을 뜻하지 않는다.
                Line(lines, foot, ToMath(command.target.position), TargetColor);
                ProjectionMarker(lines, ToMath(command.target.position), TargetColor, true);
                Point(lines, foot, TargetColor, .03f);
            }
            if (command.motion.valid)
            {
                const Vector3 goal = ToMath(command.motion.position);
                Point(lines, goal, MotionTargetColor, .035f);
                Line(lines, goal - Vector3(0, .025f, 0), goal + Vector3(0, .025f, 0), MotionTargetColor);
                Line(lines, ToMath(command.target.position), goal, MotionTargetColor);
                Line(lines, foot, goal, MotionTargetColor);
            }
        }
        const PARS::FootstepIKSnapshot& ik = mesh.GetFootstepIKSnapshot();
        if (ik.status == PARS::FootstepIKStatus::Applied)
        {
            for (size_t side = 0; side < ik.legs.size(); ++side)
            {
                const PARS::LegIKResult& leg = ik.legs[side];
                const PxU32 color = ik.motionTargets[side] ? IKStepColor : IKSupportColor;
                Line(lines, ToMath(leg.hip.p), ToMath(leg.knee.p), color);
                Line(lines, ToMath(leg.knee.p), ToMath(leg.ankle.p), color);
                Point(lines, ToMath(leg.knee.p), color, .025f);
                Point(lines, ToMath(leg.solvedSole.p), color, .02f);
                Line(lines, ToMath(leg.requestedSole.p), ToMath(leg.solvedSole.p), color);
                Line(lines, ToMath(leg.solvedSole.p), ToMath(leg.solvedSole.transform(PxVec3(0, 0, .12f))), color);
                Line(lines, ToMath(leg.hip.p), ToMath(leg.hip.p + leg.bendDirection * .18f), KneeHintColor);
            }
        }
    }
}

void FootSoleDebug::BuildRegionTriangles(const PARS::SkeletalMeshComponent& mesh,
    std::vector<PxDebugTriangle>& triangles) const
{
    const PARS::FootstepRecoverySnapshot* recovery = VisibleRecovery(mesh);
    if (!m_Visible || !m_ShowRecoveryRegions || !recovery)
    {
        return;
    }
    AppendVisibleRecoveryRegions(triangles, *recovery, .006f, .01f);
}

void FootSoleDebug::DrawUI(PARS::SkeletalMeshComponent& mesh, const std::function<void()>& drawBufferSettings)
{
    if (!ImGui::CollapsingHeader(u8"발바닥 / 지면 투영", ImGuiTreeNodeFlags_DefaultOpen))
    {
        return;
    }
    PARS::FootSupport& support = mesh.GetFootSupport();
    const PARS::FootSupportSnapshot& sample = support.GetSnapshot();
    const PARS::FootSupportBalance& balance = sample.balance;
    const char* zone = u8"판정 불가";
    switch (balance.zone)
    {
    case PARS::FootSupportZone::Inside:
        zone = u8"A 안 / 경계";
        break;
    case PARS::FootSupportZone::Buffer:
        zone = u8"A와 B 사이";
        break;
    case PARS::FootSupportZone::Outside:
        zone = u8"B 경계 / 밖";
        break;
    default:
        break;
    }
    ImGui::Text(u8"COM 위치: %s / 계산 계수 w = %.3f", zone, balance.weight);
    ImGui::ProgressBar(balance.weight, ImVec2(-1, 0));
    if (mesh.UsesAutomaticRootSupport())
    {
        const PARS::RootSupportRelease released = mesh.GetRootSupportRelease();
        const char* state = u8"지지 해제";
        switch (released)
        {
        case PARS::RootSupportRelease::None:
            state = u8"지지 중";
            break;
        case PARS::RootSupportRelease::NoSupport:
            state = u8"발 지지 없음으로 해제";
            break;
        case PARS::RootSupportRelease::OutsideBuffer:
            state = u8"실제 COM의 B 연속 이탈로 해제";
            break;
        case PARS::RootSupportRelease::ProjectileImpact:
            state = u8"피격 후 몸통 기울기 증가로 해제";
            break;
        default:
            break;
        }
        ImGui::Text(u8"적용 w = %.3f / %s", mesh.GetRagdoll().GetRootSupportWeight(), state);
        ImGui::Text(u8"실제 COM의 B 연속 이탈 %.2f / %.2f초", sample.footstep.outsideBufferTime,
            PARS::FootstepPlanner::OutsideBufferDuration);
        if (released != PARS::RootSupportRelease::None)
        {
            ImGui::TextUnformatted(u8"더미 초기화 전까지 해제 상태를 유지합니다.");
        }
    }
    else
    {
        ImGui::TextWrapped(u8"수동 지지 모드: 계산 계수는 골반 지지에 적용하지 않습니다.");
    }
    if (balance.hasDirection)
    {
        ImGui::Text(u8"중심 거리: COM %.1f / A %.1f / B %.1f cm",
            balance.comDistance * 100, balance.innerDistance * 100, balance.outerDistance * 100);
    }
    ImGui::Checkbox(u8"레이 표시", &m_ShowRays);
    ImGui::Checkbox(u8"지지 영역 A 표시 (Convex Hull)", &m_ShowHull);
    ImGui::Checkbox(u8"완충 영역 B 표시 (흰색 점선)", &m_ShowBuffer);
    ImGui::Checkbox(u8"질량중심 / 수직 투영 표시", &m_ShowCenterOfMass);
    ImGui::Checkbox(u8"시작·착지 COM 제한 구간 표시", &m_ShowLandingComClamp);
    if (m_ShowLandingComClamp)
    {
        ImGui::TextWrapped(u8"빨강 빗금: 지지발 사각형을 향하는 금지 부채꼴 · 초록 경계: 허용 · 주황: 경계 선택 유지 · 파랑 화살표: 캐릭터 앞 방향");
        ImGui::TextWrapped(u8"청록색 시작 예측과 금지 구간은 같은 물리 스텝의 COM·지지발을 사용합니다. 이동할 발이 선택되면 반대 지지발의 구간을 표시하고, 선택 전에는 양발을 미리 보여줍니다.");
        if (!support.GetFootstepSettings().landingComClampEnabled)
        {
            ImGui::TextWrapped(u8"COM 제한 기능 OFF: 회색으로 설정 구간을 미리 보여줍니다.");
        }
    }
    ImGui::TextWrapped(u8"아래 방향 20cm 이내 히트만 투영 · 초록: 성공 / 빨강: 범위 내 바닥 없음");
    ImGui::Text(u8"보라: A · 유효 투영점 %zu개 / 헐 꼭짓점 %zu개", sample.projectedPointCount, sample.hull.size());
    float scale = support.GetBufferScale();
    if (ImGui::SliderFloat(u8"B 거리 배율", &scale, 1.1f, 4.0f, "%.2f"))
    {
        support.SetBufferScale(scale);
    }
    ImGui::TextWrapped(u8"B는 A의 면적 중심 기준 확장입니다. 거리 2배 = 면적 4배.");
    if (drawBufferSettings)
    {
        drawBufferSettings();
    }
    ImGui::Text(u8"물리 스냅샷: %llu / 완료 스텝: %llu",
        static_cast<unsigned long long>(sample.sampleNumber), static_cast<unsigned long long>(sample.completedPhysicsStep));
    ImGui::TextWrapped(u8"표시를 꺼도 고정 틱에서 계산합니다. 설정 변경은 다음 물리 스텝에 반영됩니다.");
    if (sample.hull.empty())
    {
        ImGui::TextUnformatted(u8"지지 영역 없음: 유효한 점이 부족하거나 일직선입니다.");
    }
    if (sample.hasCenterOfMass)
    {
        ImGui::Text(u8"전체 질량 %.2f kg / COM (%.3f, %.3f, %.3f) m", sample.totalMass,
            sample.centerOfMass.x, sample.centerOfMass.y, sample.centerOfMass.z);
        ImGui::TextWrapped(u8"노랑: COM과 수직선 / 투영 원: 초록=A 안, 굵은 빨강=A 밖, 굵은 검정=판정 불가");
        if (!sample.hasComProjection)
        {
            ImGui::TextUnformatted(u8"COM 지면 투영점 없음");
        }
    }
    if (sample.hasCenterOfMassVelocity)
    {
        const physx::PxVec3& velocity = sample.centerOfMassVelocity;
        ImGui::Text(u8"실제 COM 속도 (%.3f, %.3f, %.3f) m/s", velocity.x, velocity.y, velocity.z);
    }
    if (ImGui::TreeNodeEx(u8"발딛기 판단 / 착지 목표", ImGuiTreeNodeFlags_DefaultOpen))
    {
        const PARS::FootstepSnapshot& step = sample.footstep;
        PARS::FootstepSettings& settings = support.GetFootstepSettings();
        const PARS::RecoverySnapshot& recovery = mesh.GetRecoverySnapshot();
        const PARS::RecoverySettings& recoverySettings = mesh.GetRecoverySettings();
        ImGui::Text(u8"액티브 내부 목표 제어: %s%s", recovery.state == PARS::RecoveryState::Recovery ? "Recovery" : "Animation",
            recovery.blending ? u8" (복귀 블렌드)" : "");
        ImGui::TextWrapped(u8"전환 판단: %s", PARS::RecoveryReasonLabel(recovery.reason));
        ImGui::Text(u8"외부 충격 / 질량 %.3f m/s (진입 %.2f) · COM 수평 속도 %.3f m/s (복귀 %.2f)",
            recovery.impactVelocity, recoverySettings.impactVelocity, recovery.horizontalSpeed, recoverySettings.maximumReturnSpeed);
        ImGui::Text(u8"기준 발 영역: %s · 현재 COM %s / 예측 COM %s · 양발 접촉 %s",
            recovery.hasReference ? "OK" : "--", recovery.insideReference ? "IN" : "OUT",
            recovery.predictionInsideReference ? "IN" : "OUT", recovery.bothGrounded ? "ON" : "OFF");
        ImGui::Text(u8"안정 유지 %.2f / %.2fs · 복귀 %.0f%% / %.2fs · 회복 %llu / 복귀 %llu",
            recovery.stableTime, recoverySettings.stableDuration, recovery.blendAlpha * 100, recoverySettings.blendDuration,
            static_cast<unsigned long long>(recovery.recoveryCount), static_cast<unsigned long long>(recovery.returnCount));
        ImGui::TextWrapped(u8"Recovery에서는 접촉이 잠시 끊겨도 양발 목표를 유지합니다. 연한 파랑 영역은 현재 양발 중심에 맞춘 애니메이션 기준 발 영역입니다.");
        const PARS::StandingSupportSnapshot& standing = mesh.GetStandingSupportSnapshot();
        PARS::StandingSupportSettings& standingSettings = mesh.GetStandingSupportSettings();
        PARS::HorizontalBalanceSettings& horizontalSettings = mesh.GetHorizontalBalanceSettings();
        float armStrengthPercent = mesh.GetArmDriveStrength() * 100.0f;
        if (ImGui::SliderFloat(u8"기본 팔 애니메이션 추종 강도", &armStrengthPercent,
            0.0f, 100.0f, "%.0f%%", ImGuiSliderFlags_AlwaysClamp))
        {
            mesh.SetArmDriveStrength(armStrengthPercent * .01f);
        }
        if (ImGui::IsItemHovered())
        {
            ImGui::SetTooltip(u8"어깨·위팔·아래팔·손의 PD 강성 배율입니다. 감쇠비는 유지하며 0%%에서는 팔 구동을 끕니다. 활성 물리 상태에서 적용됩니다.");
        }
        bool fallingAnimationEnabled = mesh.IsFallingAnimationEnabled();
        ImGui::Text(u8"균형 상태 %s", PARS::CharacterBalanceStateLabel(mesh.GetBalanceState()));
        if (ImGui::Checkbox(u8"지상 낙상 애니메이션 시퀀스", &fallingAnimationEnabled))
        {
            mesh.SetFallingAnimationEnabled(fallingAnimationEnabled);
        }
        if (ImGui::IsItemHovered())
        {
            ImGui::SetTooltip(u8"낙상 방향과 현재 전신 자세에 맞는 ShoveFall을 한 번 재생합니다.");
        }
        ImGui::Checkbox(u8"골반 XZ 직접 PD 보조", &horizontalSettings.enabled);
        if (horizontalSettings.enabled)
        {
            ImGui::SliderFloat(u8"수평 복원 고유각주파수 (rad/s)", &horizontalSettings.frequency,
                .5f, 8.0f, "%.2f", ImGuiSliderFlags_AlwaysClamp);
            ImGui::SliderFloat(u8"수평 감쇠비", &horizontalSettings.dampingRatio,
                .2f, 2.0f, "%.2f", ImGuiSliderFlags_AlwaysClamp);
            ImGui::SliderFloat(u8"최대 수평 보조 가속도 (m/s2)", &horizontalSettings.maximumAcceleration,
                0.0f, 5.0f, "%.2f", ImGuiSliderFlags_AlwaysClamp);
        }
        const PARS::HorizontalBalanceSnapshot& horizontal = mesh.GetHorizontalBalanceSnapshot();
        ImGui::Text(u8"수평 보조: %s / 실측 지지 %.0f%% / 제한 %s",
            horizontal.active ? "ON" : "OFF", horizontal.supportWeight * 100, horizontal.limited ? "ON" : "OFF");
        ImGui::Text(u8"골반 명령 힘 X/Z %.1f / %.1f N · COM 오차 X/Z %.1f / %.1f cm",
            horizontal.force.x, horizontal.force.z, horizontal.positionError.x * 100, horizontal.positionError.z * 100);
        ImGui::SliderFloat(u8"지지발 발목 최대 추가 강성·감쇠", &standingSettings.ankleDriveBoost,
            0.0f, 1.0f, "+%.2f", ImGuiSliderFlags_AlwaysClamp);
        if (ImGui::IsItemHovered())
        {
            ImGui::SetTooltip(u8"체중 전체를 받는 발의 추가 배율입니다. 0.5이면 최대 1.5배, 0이면 보정을 끕니다.");
        }
        ImGui::Checkbox(u8"지지발이 들린 방향으로 과도한 기울기 목표 제한", &standingSettings.limitFootTilt);
        if (standingSettings.limitFootTilt)
        {
            ImGui::SliderFloat(u8"기울기 제한 시작 (도)", &standingSettings.footTiltLimitStartDegrees,
                0.0f, standingSettings.footTiltLimitDegrees - .1f, "%.1f", ImGuiSliderFlags_AlwaysClamp);
            ImGui::SliderFloat(u8"기울기 제한 완료 (도)", &standingSettings.footTiltLimitDegrees,
                standingSettings.footTiltLimitStartDegrees + .1f, 30.0f, "%.1f", ImGuiSliderFlags_AlwaysClamp);
        }
        if (mesh.GetRagdoll().UsesFootDrivenSupport() && standing.valid)
        {
            const PARS::RagdollSupportLoads& loads = mesh.GetRagdoll().GetSupportLoads();
            ImGui::Text(u8"다리로 골반 높이 지지: 실제 %.3fm / 목표 %.3fm / 기준 %.3fm",
                standing.actualHeight, standing.targetHeight, standing.nominalHeight);
            ImGui::Text(u8"낮추기 요청: 발딛기 %.1fcm / 접촉 상실 %.1fcm / 다리 도달 %.1fcm",
                standing.stepLowering * 100, standing.contactLowering * 100, standing.reachLowering * 100);
            ImGui::Text(u8"적용 높이 차이 %.1fcm / 수평 자세 보정 %.1fcm / 도달 불가 %s",
                standing.lowering * 100, standing.horizontalCorrection.magnitude() * 100, standing.reachLimited ? "ON" : "OFF");
            ImGui::Text(u8"예상 지지 하중: 왼발 %.0fN / 오른발 %.0fN", loads.forces[0].y, loads.forces[1].y);
            ImGui::Text(u8"발목 강성·감쇠 배율: 왼발 %.2f / 오른발 %.2f · 기울기 목표 제한 %s / %s",
                loads.ankleDriveScales[0], loads.ankleDriveScales[1],
                standing.footTiltLimited[0] ? "ON" : "OFF", standing.footTiltLimited[1] ? "ON" : "OFF");
            ImGui::Text(u8"측정 접촉 반력: 왼발 %.0fN / 오른발 %.0fN",
                sample.feet[0].contact.normalImpulse / float(PhysicsWorld::FixedDeltaTime),
                sample.feet[1].contact.normalImpulse / float(PhysicsWorld::FixedDeltaTime));
            ImGui::TextWrapped(u8"골반을 직접 들어 올리는 힘 없이 지지 다리의 자세와 관절 토크로 높이를 조절합니다. 지지발 IK 목표에는 골반 오차 보정이 포함되므로 실제 발바닥 고정점과 다를 수 있습니다.");
        }
        ImGui::Checkbox(u8"발딛기 판단·목표 3D 표시", &m_ShowFootstep);
        ImGui::Checkbox(u8"회복·일반·교차 영역 색칠", &m_ShowRecoveryRegions);
        ImGui::Checkbox(u8"이동 경로 충돌 예측 및 우회", &settings.pathAvoidanceEnabled);
        ImGui::Checkbox(u8"회복 영역으로 일반 / 교차 선택", &settings.recovery.enabled);
        if (settings.recovery.enabled)
        {
            ImGui::TextWrapped(u8"검정은 고관절과 다리 길이로 닿는 범위에서 지지발 옆 최소 간격을 제외한 영역입니다. 실제 착지점에는 최대 보폭과 자세 검사를 적용합니다.");
            int maximumSteps = static_cast<int>(settings.recovery.maximumSteps);
            if (ImGui::SliderInt(u8"회복 예측 최대 발딛기 수", &maximumSteps, 1, 3))
            {
                settings.recovery.maximumSteps = static_cast<uint32_t>(maximumSteps);
            }
            if (settings.recovery.maximumSteps == 1)
            {
                ImGui::SliderFloat(u8"1스텝 회복 영역 배율", &settings.recovery.oneStepCaptureScale,
                    1.0f, 2.0f, "%.2fx", ImGuiSliderFlags_AlwaysClamp);
            }
            ImGui::SliderFloat(u8"하중 이동 예측 시간", &settings.recovery.transferTime, .01f, .3f, "%.3fs");
            ImGui::SliderFloat(u8"앞 교차 경계 각도", &settings.recovery.frontAngleDegrees, 0.0f, 60.0f, "%.1f도");
            ImGui::SliderFloat(u8"뒤 교차 경계 각도", &settings.recovery.backAngleDegrees, 0.0f, 60.0f, "%.1f도");
            const PARS::FootstepRecoverySnapshot* decision = VisibleRecovery(mesh);
            const std::string decisionText = FootstepDecisionText(step, decision);
            if (!decisionText.empty())
            {
                ImGui::TextUnformatted(decisionText.c_str());
            }
            if (decision)
            {
                if (m_ShowRecoveryRegions)
                {
                    ImGui::TextUnformatted(ShowCrossReachRegions(*decision) ?
                        u8"영역 색상 · 검정 일반 도달 / 연초록 회복 / 연주황 앞 교차 / 연보라 뒤 교차" :
                        u8"영역 색상 · 검정 일반 도달 / 연초록 회복");
                }
                ImGui::Text(u8"최대 보폭 적용 후 겹침 · 일반 %.1f / 앞 %.1f / 뒤 %.1f cm²",
                    decision->overlapAreas[0] * 10000, decision->overlapAreas[1] * 10000, decision->overlapAreas[2] * 10000);
                ImGui::Text(u8"착지 자세 %u회 / 영역 %u조각 / 착지 예측 %.3fs / 경로 %s",
                    decision->poseChecks, decision->capturePieceCount, decision->predictionTime,
                    !settings.pathAvoidanceEnabled ? u8"예측 꺼짐" : decision->pathRejected ? u8"미확보" : u8"아래 충돌 예측 참조");
                if (decision->targetInCaptureMargin)
                {
                    ImGui::TextUnformatted(u8"확장 영역의 착지 후보 · 한 걸음 회복 미확정");
                }
                else if (decision->hasTarget)
                {
                    ImGui::Text(u8"선택 목표의 필요 스텝: %u / 착지 자세 통과 %s / 검사 예산 %s",
                        decision->requiredSteps, decision->poseRejected ? u8"일부 거절 후 선택" : "ON",
                        decision->budgetExhausted ? u8"도달" : "OK");
                }
                if (decision->usesFutureReachApproximation)
                {
                    ImGui::TextWrapped(u8"후속 스텝은 현재 도달 범위가 지지발을 따라 이동한다고 근사합니다. 실제 착지와 하중 인계 후 다시 판단합니다.");
                }
            }
        }
        ImGui::Checkbox(u8"발별 착지 윤곽 표시", &m_ShowLandingFoot);
        ImGui::TextWrapped(u8"밝은 파랑 윤곽: 최종 착지 발바닥 위치·크기 / 화살표: 발끝 방향. 새 목표 계산이 잠시 멈춰도 마지막 착지 목표를 유지합니다.");
        ImGui::TextWrapped(u8"사유: %s", PARS::FootstepReasonLabel(step.reason));
        ImGui::Text(u8"현재 명령 대상: %s", FootLabel(step.requestedFoot));
        ImGui::Text(u8"다음 발딛기 하중 조건: 체중 %.0f%% 이상을 %.3f초 연속 확인",
            settings.supportLoadRatio * 100, settings.supportConfirmationTime);
        if (step.loadTransfer.required && step.requestedFoot >= 0 && step.requestedFoot < 2)
        {
            const size_t receivingFoot = static_cast<size_t>(1 - step.requestedFoot);
            ImGui::Text(u8"하중 확인 발: %s / 실측 %.0f%% / 연속 %.3f초 / 시작 허용 %s",
                FootLabel(static_cast<int>(receivingFoot)), step.loadTransfer.loadRatios[receivingFoot] * 100,
                step.loadTransfer.confirmationTimes[receivingFoot], step.loadTransfer.ready ? "ON" : "OFF");
        }
        if (step.hasImpact)
        {
            ImGui::Text(u8"마지막 루트 충격점 (%.3f, %.3f, %.3f) m", step.lastImpactPosition.x,
                step.lastImpactPosition.y, step.lastImpactPosition.z);
        }
        ImGui::Text(u8"시작 %llu / 착지 %llu / 중단 %llu",
            static_cast<unsigned long long>(step.startCount), static_cast<unsigned long long>(step.landingCount),
            static_cast<unsigned long long>(step.abortCount));
        ImGui::Text(u8"예측: 시작 %.2fs / 착지 %.2fs · 발 간격 %.2f · 최대 보폭 %.1fm",
            settings.startPredictionTime, settings.landingPredictionTime, settings.stanceScale, settings.maximumStepLength);
        float minimumStanceWidthPercent = settings.minimumStanceWidthRatio * 100.0f;
        if (ImGui::SliderFloat(u8"최소 좌우 간격 (Idle 대비)", &minimumStanceWidthPercent, 0.0f, 100.0f, "%.0f%%", ImGuiSliderFlags_AlwaysClamp))
        {
            settings.minimumStanceWidthRatio = minimumStanceWidthPercent * .01f;
        }
        if (ImGui::IsItemHovered())
        {
            ImGui::SetTooltip(u8"착지 목표가 이 간격보다 좁을 때만 바깥으로 보정합니다. 0%%이면 간격 보정을 끕니다.");
        }
        ImGui::Checkbox(u8"일반 딛기의 시작·착지 COM 방향 제한", &settings.landingComClampEnabled);
        ImGui::SliderFloat(u8"부채꼴 경계 선택 유지 반각 (도)", &settings.landingComClampHysteresisDegrees,
            0.0f, 30.0f, "%.1f", ImGuiSliderFlags_AlwaysClamp);
        ImGui::TextWrapped(u8"기존 일반 딛기 후보에 적용하는 COM 보정입니다. 회복 영역 판단과 교차 후보는 보정 전 실제 COM·속도를 사용합니다. 회복 영역 선택을 끄면 기존 청록색 보정점으로 시작을 판단합니다.");
        ImGui::DragFloat(u8"지지 다리 시한이 없을 때의 이동 시간 (초)", &settings.stepDuration, .01f,
            PARS::MinimumFootstepDuration, PARS::MaximumFootstepDuration, "%.2f");
        ImGui::DragFloat(u8"최소 들기 높이 (m)", &settings.minimumLiftHeight, .005f, .01f, .3f, "%.3f");
        ImGui::DragFloat(u8"착지 수평 목표 고정 진행률", &settings.landingLockProgress, .01f, .5f, 1.0f, "%.2f");
        ImGui::DragFloat(u8"착지 추가 하강 속도 (m/s)", &settings.landingLoweringSpeed, .005f, 0.0f, .3f, "%.3f");
        ImGui::DragFloat(u8"착지 최대 추가 하강 (m)", &settings.maximumLandingLowering, .005f, 0.0f, .1f, "%.3f");
        ImGui::Checkbox(u8"발 교차 억제 (경계 0.1m)", &settings.preventCrossing);
        if (step.hasPrediction)
        {
            ImGui::Text(u8"시작 판정용 COM 보정 예측 (%.3f, %.3f, %.3f) m / A %s", step.predictedCom.x,
                step.predictedCom.y, step.predictedCom.z, step.needsStep ? u8"밖" : u8"안");
            ImGui::Text(u8"  시작 예측 보정 %s / 원래 (%.3f, %.3f, %.3f) m", step.startCom.limited ? "ON" : "OFF",
                step.startCom.raw.x, step.startCom.raw.y, step.startCom.raw.z);
        }
        for (size_t side = 0; side < step.candidates.size(); ++side)
        {
            const PARS::FootstepTarget& target = step.candidates[side];
            if (target.valid)
            {
                const PARS::FootstepComPrediction& landing = target.landingCom;
                ImGui::Text(u8"%s 착지 COM: 제한 %s / 보정 %s / %s", FootLabel(static_cast<int>(side)),
                    landing.enabled ? "ON" : "OFF", landing.limited ? "ON" : "OFF",
                    LandingDirectionLabel(landing.direction));
                ImGui::Text(u8"  원래 (%.3f, %.3f, %.3f) / 보정 (%.3f, %.3f, %.3f) m",
                    landing.raw.x, landing.raw.y, landing.raw.z, landing.clamped.x, landing.clamped.y, landing.clamped.z);
                if (landing.region.valid)
                {
                    ImGui::Text(u8"  기준 지지발: %s / 사각형으로 계산한 금지 폭 %.1f도",
                        FootLabel(landing.region.supportFoot), landing.region.halfAngleRadians * 360.0f / PxPi);
                }
                else if (landing.region.containsCom)
                {
                    ImGui::TextUnformatted(u8"  COM이 지지발 사각형 안 또는 경계: 방향 금지 구간 없음");
                }
                ImGui::Text(u8"%s 목표 (%.3f, %.3f, %.3f) m", FootLabel(static_cast<int>(side)),
                    target.position.x, target.position.y, target.position.z);
                ImGui::Text(u8"  발 이동 %.2fm / 반대 발 거리 %.2fm · 보폭 제한 %s / 간격 보정 %s / 교차 제한 %s",
                    target.travelDistance, target.separationDistance, target.lengthLimited ? "ON" : "OFF",
                    target.stanceWidthLimited ? "ON" : "OFF",
                    target.crossingLimited ? "ON" : "OFF");
            }
            else
            {
                ImGui::Text(u8"%s 후보: %s", FootLabel(static_cast<int>(side)), PARS::FootstepReasonLabel(target.reason));
            }
        }
        for (size_t side = 0; side < step.feet.size(); ++side)
        {
            const PARS::FootstepCommand& command = step.feet[side];
            const PARS::FootContactSample& contact = sample.feet[side].contact;
            ImGui::Text(u8"%s: %s / 실제 접지 %s / 이동 명령 %s", FootLabel(static_cast<int>(side)),
                PARS::FootstepPhaseLabel(command.phase), contact.hasSample && contact.grounded ? "ON" : "OFF",
                command.IsInProgress() ? "ON" : "OFF");
            ImGui::TextWrapped(u8"  %s", PARS::FootstepReasonLabel(command.reason));
            if (!command.IsInProgress() && command.phase != PARS::FootstepPhase::Landed)
            {
                continue;
            }
            ImGui::Text(u8"시작 시 이동 거리 %.2fm / 목표 출력 %s", command.initialDistance, command.hasTarget ? "ON" : "OFF");
            const PARS::FootstepMotion& motion = command.motion;
            if (motion.hasSupportDeadline)
            {
                ImGui::Text(u8"지지 다리 IK 한계까지 계획한 착지 시간 %.3f초 / 남은 시간 %.3f초",
                    motion.duration, PARS::GetFootstepRemainingTime(command));
                if (command.phase == PARS::FootstepPhase::Landed)
                {
                    ImGui::Text(u8"실제 착지 시각과 시한의 차이 %+.3f초", motion.landingTimeError);
                }
            }
            ImGui::Text(u8"실제 이륙 확인 %s / 이륙 대기 %.3f초", command.hasTakenOff ? "ON" : "OFF", motion.takeoffElapsedTime);
            ImGui::Text(u8"계획 최고 높이 %.1fcm / 시작 시 종아리 상한 %.1fcm",
                motion.initialHeight * 100, motion.heightLimit * 100);
            ImGui::Text(u8"이동 시간 %.2f / %.2fs · 시간 진행 %.1f%% / 실제 수평 진행 %.1f%% / 남은 거리 %.1fcm",
                motion.elapsedTime, motion.duration, motion.progress * 100,
                motion.measuredProgress * 100, motion.remainingDistance * 100);
            ImGui::PushID(static_cast<int>(side));
            ImGui::ProgressBar(motion.progress, ImVec2(-1, 0));
            ImGui::PopID();
            ImGui::Text(u8"착지 수평 목표 고정: %s / 추가 하강 %.2f / %.2fcm",
                motion.landingTargetLocked ? "ON" : "OFF", motion.landingLowering * 100,
                motion.maximumLandingLowering * 100);
            ImGui::Text(u8"저장한 목표의 COM 보정 %s / %s", command.target.landingCom.limited ? "ON" : "OFF",
                LandingDirectionLabel(command.target.landingCom.direction));
            if (motion.valid)
            {
                ImGui::Text(u8"추가 높이 %.1fcm / 발 이동 목표 (%.3f, %.3f, %.3f) m", motion.height * 100,
                    motion.position.x, motion.position.y, motion.position.z);
            }
            else
            {
                ImGui::TextUnformatted(u8"새 발 이동 목표 없음 / Recovery는 저장한 목표 유지");
            }
            if (motion.hasLandingError)
            {
                ImGui::Text(u8"재접촉 시 수평 목표 오차 %.1fcm (목표 도달 여부와 접촉 종료는 별개)", motion.landingError * 100);
            }
        }
        ImGui::Text(u8"최고 높이 = min(max(시작 거리 x %.2f, %.2fm), 기준 종아리 길이)",
            settings.liftHeightScale, settings.minimumLiftHeight);
        ImGui::TextWrapped(u8"COM 제한은 다음 물리 스텝의 시작·착지 예측에 반영됩니다. 간격 설정은 잠기지 않은 착지 목표에, 이동 시간·들기·하강 설정은 새 명령부터 적용됩니다. 수평 목표를 고정한 뒤에도 실제 접촉을 확인해 종료합니다.");
        ImGui::TextWrapped(u8"노랑: 현재 COM · 청록: 시작 판정용 보정 COM 예측 · 연보라: 착지용 원래 COM 예측 · 연두: 착지용 보정 COM 예측 · 빨간 빗금: 지지발 방향의 금지 부채꼴");
        ImGui::TextWrapped(u8"회색: 발 오프셋을 더한 원시 후보 · 주황 십자: 착지 후보 · 주황 원: 저장한 발의 목표 · 초록 선: 실제 접촉 법선");
        ImGui::TextWrapped(u8"분홍 십자: 높이를 더한 발 이동 목표. 노랑/연두: 이동 명령 유무에 따른 제한 적용 전 IK 자세. 보라 선: 무릎 굽힘 방향. 연결선은 실제 이동 궤적이 아닙니다.");
        const PARS::FootstepIKSnapshot& ik = mesh.GetFootstepIKSnapshot();
        ImGui::Text(u8"IK → 관절 PD: %s", PARS::FootstepIKStatusLabel(ik.status));
        if (ik.status == PARS::FootstepIKStatus::Applied)
        {
            for (size_t side = 0; side < ik.legs.size(); ++side)
            {
                const PARS::LegIKResult& leg = ik.legs[side];
                const float actualError = (ToPx(Vector3(sample.feet[side].soleWorld.GetW())) - leg.requestedSole.p).magnitude();
                ImGui::Text(u8"%s (%s): IK 도달 오차 %.2fcm / 실제 발 목표 오차 %.2fcm%s", FootLabel(int(side)),
                    ik.motionTargets[side] ? u8"목표 이동" : u8"자세 유지", leg.reachError * 100, actualError * 100,
                    leg.reachLimited ? u8" / 다리 길이 제한" : "");
                ImGui::Text(u8"  무릎 Hint: %s / 발목 목표 (%.3f, %.3f, %.3f) m",
                    PARS::KneeHintSourceLabel(leg.hintSource), leg.requestedAnkle.x, leg.requestedAnkle.y, leg.requestedAnkle.z);
            }
            ImGui::Text(u8"기존 관절 제한의 최대 목표 수정각 (전신): %.2f도", XMConvertToDegrees(mesh.GetRagdoll().GetMaxTargetClampAngle()));
            ImGui::TextWrapped(u8"IK 도달 오차는 다리 길이로 제한한 결과입니다. 이후 PA 관절 제한과 실제 물리 추종 오차는 별도이며, IK 표시만으로 착지나 회복 성공을 뜻하지 않습니다.");
        }
        ImGui::TreePop();
    }
    if (!sample.message.empty())
    {
        ImGui::TextWrapped("%s", sample.message.c_str());
    }
    if (ImGui::Button(u8"발바닥 기본 영역으로 초기화"))
    {
        support.ResetSoles();
    }
    const char* const labels[] = {u8"왼발", u8"오른발"};
    for (size_t side = 0; side < sample.feet.size(); ++side)
    {
        PARS::FootSoleSettings& settings = support.GetSoleSettings(side);
        const PARS::FootSoleSample& foot = sample.feet[side];
        ImGui::PushID(static_cast<int>(side));
        if (foot.available && ImGui::TreeNodeEx(labels[side], ImGuiTreeNodeFlags_DefaultOpen))
        {
            const PARS::FootContactSample& contact = foot.contact;
            ImGui::Text(u8"실제 접촉: %s / 지면 접촉점 %u개", !contact.hasSample ? u8"물리 스텝 대기" :
                contact.grounded ? u8"지지 중" : u8"접촉 없음", contact.pointCount);
            ImGui::Text(u8"접촉 이력: 이륙 %llu회 / 착지 %llu회",
                static_cast<unsigned long long>(contact.takeoffCount), static_cast<unsigned long long>(contact.landingCount));
            if (contact.grounded)
            {
                ImGui::Text(u8"접촉점 (%.3f, %.3f, %.3f) m", contact.position.x, contact.position.y, contact.position.z);
                ImGui::Text(u8"지면 법선 (%.3f, %.3f, %.3f)", contact.normal.x, contact.normal.y, contact.normal.z);
            }
            if (foot.leg.available)
            {
                ImGui::Text(u8"다리 본: 고관절 %d / 무릎 %d / 발목 %d",
                    foot.leg.hipNode, foot.leg.kneeNode, foot.leg.ankleNode);
                ImGui::Text(u8"기준 길이: 허벅지 %.1f / 종아리 %.1f cm",
                    foot.leg.thighLength * 100, foot.leg.calfLength * 100);
            }
            else
            {
                ImGui::TextUnformatted(u8"다리 본 연결 또는 기준 길이를 찾을 수 없습니다.");
            }
            float dimensions[] = {settings.halfWidth * 200, settings.halfLength * 200};
            if (ImGui::DragFloat2(u8"폭 / 길이", dimensions, .1f, 1, 100, "%.1f cm", ImGuiSliderFlags_AlwaysClamp))
            {
                settings.halfWidth = dimensions[0] * .005f;
                settings.halfLength = dimensions[1] * .005f;
            }
            ImGui::DragFloat3(u8"로컬 위치", &settings.offset.x, .001f, -.5f, .5f, "%.3f m", ImGuiSliderFlags_AlwaysClamp);
            ImGui::DragFloat3(u8"로컬 회전", &settings.rotation.x, .5f, -180, 180, "%.1f deg", ImGuiSliderFlags_AlwaysClamp);
            unsigned count = 0;
            for (bool hit : foot.projected)
            {
                count += hit ? 1 : 0;
            }
            ImGui::Text(u8"투영점 %u / 4", count);
            for (int i = 0; i < 4; ++i)
            {
                if (foot.projected[i])
                {
                    ImGui::Text(u8"점 %d: %.1f cm", i + 1, foot.distances[i] * 100);
                }
                else
                {
                    ImGui::Text(u8"점 %d: 투영 없음", i + 1);
                }
            }
            ImGui::TreePop();
        }
        ImGui::PopID();
    }
}
