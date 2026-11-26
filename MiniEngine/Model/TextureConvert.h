//
// Copyright (c) Microsoft. All rights reserved.
// This code is licensed under the MIT License (MIT).
// THIS CODE IS PROVIDED *AS IS* WITHOUT WARRANTY OF
// ANY KIND, EITHER EXPRESS OR IMPLIED, INCLUDING ANY
// IMPLIED WARRANTIES OF FITNESS FOR A PARTICULAR
// PURPOSE, MERCHANTABILITY, OR NON-INFRINGEMENT.
//
// Developed by Minigraph
//
// Author:  James Stanard
//

#pragma once

#include <cstdint>
#include <string>

enum TexConversionFlags
{
    kSRGB = 1,          // Texture contains sRGB colors
    kPreserveAlpha = 2, // Keep four channels
    kNormalMap = 4,     // Texture contains normals
    kBumpToNormal = 8,  // Generate a normal map from a bump map
    kDefaultBC = 16,    // Apply standard block compression (BC1-5)
    kQualityBC = 32,    // Apply quality block compression (BC6H/7)
    kFlipVertical = 64,
};

inline uint8_t TextureOptions(bool sRGB, bool hasAlpha=false, bool invertY=false)
{
    return (sRGB ? kSRGB : 0) | (hasAlpha ? kPreserveAlpha : 0) | (invertY ? kFlipVertical : 0);
}

// 재질 용도에 맞는 DDS를 준비하고 실제로 읽을 경로를 반환한다.
std::wstring CompileTextureOnDemand(const std::wstring& originalFile, uint32_t flags);

// 원본을 밉맵이 있는 DDS로 변환한다. 출력 경로를 생략하면 원본 확장자를 .dds로 바꾼다.
bool ConvertToDDS(const std::wstring& filePath, uint32_t flags, const std::wstring& outputFile = L"");
