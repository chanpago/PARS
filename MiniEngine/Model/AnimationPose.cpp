// Copyright (c) Microsoft. All rights reserved.
// Licensed under the MIT License. Sampling helpers adapted from Animation.cpp.

#include "Animation.h"
#include <cmath>
#include <cstring>

using Math::Quaternion;
using Math::Vector4;
using DirectX::XMFLOAT4;
using DirectX::FXMVECTOR;

static inline float ToFloat(const int8_t x) { return Math::Max(x / 127.0f, -1.0f); }
static inline float ToFloat(const uint8_t x) { return x / 255.0f; }
static inline float ToFloat(const int16_t x) { return Math::Max(x / 32767.0f, -1.0f); }
static inline float ToFloat(const uint16_t x) { return x / 65535.0f; }

static inline void Lerp3(float* Dest, const float* Key1, const float* Key2, float T)
{
    Dest[0] = Math::Lerp(Key1[0], Key2[0], T);
    Dest[1] = Math::Lerp(Key1[1], Key2[1], T);
    Dest[2] = Math::Lerp(Key1[2], Key2[2], T);
}

template <typename T>
static inline Quaternion ToQuat(const T* rot)
{
    return (Quaternion)Vector4(ToFloat(rot[0]), ToFloat(rot[1]), ToFloat(rot[2]), ToFloat(rot[3]));
}

static inline Quaternion ToQuat(const float* rot)
{
    return Quaternion(XMLoadFloat4((const XMFLOAT4*)rot));
}

static inline void Slerp(float* Dest, const void* Key1, const void* Key2, float T, uint32_t Format)
{
    switch (Format)
    {
    case AnimationCurve::kSNorm8:
    {
        const int8_t* key1 = (const int8_t*)Key1;
        const int8_t* key2 = (const int8_t*)Key2;
        XMStoreFloat4((XMFLOAT4*)Dest, (FXMVECTOR)Math::Slerp(ToQuat(key1), ToQuat(key2), T));
        break;
    }
    case AnimationCurve::kUNorm8:
    {
        const uint8_t* key1 = (const uint8_t*)Key1;
        const uint8_t* key2 = (const uint8_t*)Key2;
        XMStoreFloat4((XMFLOAT4*)Dest, (FXMVECTOR)Math::Slerp(ToQuat(key1), ToQuat(key2), T));
        break;
    }
    case AnimationCurve::kSNorm16:
    {
        const int16_t* key1 = (const int16_t*)Key1;
        const int16_t* key2 = (const int16_t*)Key2;
        XMStoreFloat4((XMFLOAT4*)Dest, (FXMVECTOR)Math::Slerp(ToQuat(key1), ToQuat(key2), T));
        break;
    }
    case AnimationCurve::kUNorm16:
    {
        const uint16_t* key1 = (const uint16_t*)Key1;
        const uint16_t* key2 = (const uint16_t*)Key2;
        XMStoreFloat4((XMFLOAT4*)Dest, (FXMVECTOR)Math::Slerp(ToQuat(key1), ToQuat(key2), T));
        break;
    }
    case AnimationCurve::kFloat:
    {
        const float* key1 = (const float*)Key1;
        const float* key2 = (const float*)Key2;
        XMStoreFloat4((XMFLOAT4*)Dest, (FXMVECTOR)Math::Slerp(ToQuat(key1), ToQuat(key2), T));
        break;
    }
    default:
        // The caller validates the key format.
        break;
    }
}

bool EvaluateAnimationPose(const AnimationSet& animation, const AnimationCurve* curves,
    const uint8_t* keyFrames, GraphNode* pose, uint32_t nodeCount, float time)
{
    if (!curves || !keyFrames || !pose || !std::isfinite(time))
        return false;
    for (uint32_t i = 0; i < animation.numCurves; ++i)
    {
        const AnimationCurve& curve = curves[animation.firstCurve + i];
        if (curve.targetNode >= nodeCount || curve.numSegments < 1 ||
            curve.keyFrameFormat > AnimationCurve::kFloat || curve.targetPath == AnimationCurve::kWeights)
            return false;
        const float progress = Math::Clamp((time - curve.startTime) * curve.rangeScale, 0.0f, curve.numSegments);
        const uint32_t segment = static_cast<uint32_t>(Math::Min(progress, curve.numSegments - 1.0f));
        const float fraction = progress - float(segment);
        const size_t stride = curve.keyFrameStride * 4;
        const uint8_t* first = keyFrames + curve.keyFrameOffset + stride * segment;
        const uint8_t* second = first + stride;
        GraphNode& node = pose[curve.targetNode];
        if (curve.targetPath == AnimationCurve::kRotation)
        {
            Slerp(reinterpret_cast<float*>(&node.rotation), first, second, fraction, curve.keyFrameFormat);
            node.staleMatrix = true;
        }
        else
        {
            if (curve.keyFrameFormat != AnimationCurve::kFloat)
                return false;
            float* destination = curve.targetPath == AnimationCurve::kTranslation ?
                reinterpret_cast<float*>(&node.xform) + 12 : reinterpret_cast<float*>(&node.scale);
            Lerp3(destination, reinterpret_cast<const float*>(first), reinterpret_cast<const float*>(second), fraction);
            if (curve.targetPath == AnimationCurve::kScale)
                node.staleMatrix = true;
        }
    }
    return true;
}

void BlendAnimationPoses(const GraphNode* source, GraphNode* target, uint32_t count, float alpha)
{
    if (!source || !target || !std::isfinite(alpha) || alpha >= 1.0f)
        return;
    if (alpha <= 0.0f)
    {
        for (uint32_t i = 0; i < count; ++i)
        {
            target[i].xform = source[i].xform;
            target[i].rotation = source[i].rotation;
            target[i].scale = source[i].scale;
            target[i].staleMatrix = source[i].staleMatrix;
        }
        return;
    }
    for (uint32_t i = 0; i < count; ++i)
    {
        const auto& from = source[i];
        auto& to = target[i];
        // Preserve matrix-only static nodes; rebuild TRS only for animated nodes.
        if (from.staleMatrix || to.staleMatrix ||
            std::memcmp(&from.rotation, &to.rotation, sizeof(to.rotation)) != 0 ||
            std::memcmp(&from.scale, &to.scale, sizeof(to.scale)) != 0)
        {
            to.rotation = Math::Slerp(from.rotation, to.rotation, alpha);
            Lerp3(reinterpret_cast<float*>(&to.scale), reinterpret_cast<const float*>(&from.scale),
                reinterpret_cast<const float*>(&to.scale), alpha);
            to.staleMatrix = true;
        }
        Lerp3(reinterpret_cast<float*>(&to.xform) + 12, reinterpret_cast<const float*>(&from.xform) + 12,
            reinterpret_cast<const float*>(&to.xform) + 12, alpha);
    }
}
