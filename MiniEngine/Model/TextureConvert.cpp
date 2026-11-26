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
// Author(s):  James Stanard
//             Chuck Walbourn (ATG)
//
// This code depends on DirectXTex
//

#include "TextureConvert.h"
#include "../Core/Utility.h"
#include "DirectXTex.h"

using namespace DirectX;

#define GetFlag(f) ((Flags & f) != 0)

std::wstring CompileTextureOnDemand(const std::wstring& originalFile, uint32_t flags)
{
    const std::wstring extension = Utility::ToLower(Utility::GetFileExtension(originalFile));
    if (extension == L"dds")
    {
        return originalFile;
    }

    const std::wstring legacyFile = Utility::RemoveExtension(originalFile) + L".dds";
    // 색공간이나 뒤집기 옵션이 다른 재질끼리 캐시를 공유하지 않도록 원본 이름과 옵션을 함께 쓴다.
    const std::wstring ddsFile = extension == L"hdr" || extension == L"exr" ? legacyFile :
        originalFile + L".v2." + std::to_wstring(flags) + L".dds";
    struct _stat64 srcStat;
    struct _stat64 ddsStat;
    const bool hasCache = _wstat64(ddsFile.c_str(), &ddsStat) == 0;
    if (_wstat64(originalFile.c_str(), &srcStat) != 0)
    {
        return hasCache ? ddsFile : legacyFile;
    }
    if (hasCache && ddsStat.st_mtime >= srcStat.st_mtime)
    {
        return ddsFile;
    }

    const std::wstring pending = ddsFile + L".tmp." + std::to_wstring(GetCurrentProcessId()) +
        L"." + std::to_wstring(GetCurrentThreadId());
    if (ConvertToDDS(originalFile, flags, pending) &&
        MoveFileExW(pending.c_str(), ddsFile.c_str(), MOVEFILE_REPLACE_EXISTING))
    {
        return ddsFile;
    }
    DeleteFileW(pending.c_str());
    Utility::Printf("Could not rebuild texture %ws.\n", originalFile.c_str());
    return GetFileAttributesW(ddsFile.c_str()) != INVALID_FILE_ATTRIBUTES ? ddsFile : legacyFile;
}

bool ConvertToDDS(const std::wstring& filePath, uint32_t Flags, const std::wstring& outputFile)
{
    bool bInterpretAsSRGB =	GetFlag(kSRGB);
    bool bPreserveAlpha =	GetFlag(kPreserveAlpha);
    bool bContainsNormals = GetFlag(kNormalMap);
    bool bBumpMap =			GetFlag(kBumpToNormal);
    bool bBlockCompress =	GetFlag(kDefaultBC);
    bool bUseBestBC =		GetFlag(kQualityBC);
    bool bFlipImage =       GetFlag(kFlipVertical);

    // Can't be both
    ASSERT(!bInterpretAsSRGB || !bContainsNormals);
    ASSERT(!bPreserveAlpha || !bContainsNormals);

    Utility::Printf( "Converting file \"%ws\" to DDS.\n", filePath.c_str() );

    // Get extension as utf8 (ascii)
    std::wstring ext = Utility::ToLower(Utility::GetFileExtension(filePath));

    // Load texture image
    TexMetadata info;
    std::unique_ptr<ScratchImage> image(new ScratchImage);

    bool isDDS = false;
    bool isHDR = false;
    if ( ext == L"dds" )
    {
        // TODO:  It might be desired to compress or recompress existing DDS files
        //Utility::Printf("Ignoring existing DDS \"%ws\".\n", filePath.c_str());
        //return false;

        isDDS = true;
        HRESULT hr = LoadFromDDSFile( filePath.c_str(), DDS_FLAGS_NONE, &info, *image );
        if (FAILED(hr))
        {
            Utility::Printf( "Could not load texture \"%ws\" (DDS: %08X).\n", filePath.c_str(), hr );
            return false;
        }
    }
    else if ( ext == L"tga" )
    {
        HRESULT hr = LoadFromTGAFile( filePath.c_str(), &info, *image );
        if (FAILED(hr))
        {
            Utility::Printf( "Could not load texture \"%ws\" (TGA: %08X).\n", filePath.c_str(), hr );
            return false;
        }
    }
    else if ( ext == L"hdr" )
    {
        isHDR = true;
        HRESULT hr = LoadFromHDRFile( filePath.c_str(), &info, *image );
        if (FAILED(hr))
        {
            Utility::Printf( "Could not load texture \"%ws\" (HDR: %08X).\n", filePath.c_str(), hr );
            return false;
        }
    }
    else if ( ext == L"exr" )
    {
#ifdef USE_OPENEXR
        isHDR = true;
        HRESULT hr = LoadFromEXRFile(filePath.c_str(), &info, *image);
        if (FAILED(hr))
        {
            Utility::Printf("Could not load texture \"%ws\" (EXR: %08X).\n", filePath.c_str(), hr);
            return false;
        }
#else
        Utility::Printf("OpenEXR not supported for this build of the content exporter\n");
        return false;
#endif
    }
    else
    {
        HRESULT hr = LoadFromWICFile(filePath.c_str(), WIC_FLAGS_IGNORE_SRGB, &info, *image);
        if (FAILED(hr))
        {
            Utility::Printf( "Could not load texture \"%ws\" (WIC: %08X).\n", filePath.c_str(), hr );
            return false;
        }
    }

    if ( info.width > 16384 || info.height > 16384 )
    {
        Utility::Printf("Texture size (%Iu,%Iu) too large for feature level 11.0 or later (16384) \"%ws\".\n",
            info.width, info.height, filePath.c_str());
        return false;
    }

    if (bFlipImage)
    {
        std::unique_ptr<ScratchImage> timage(new ScratchImage);

        HRESULT hr = FlipRotate( image->GetImages()[0], TEX_FR_FLIP_VERTICAL, *timage );

        if (FAILED(hr))
        {
            Utility::Printf( "Could not flip image \"%ws\" (%08X).\n", filePath.c_str(), hr );
        }
        else
        {
            image.swap(timage);
        }
       
    }

    DXGI_FORMAT tformat;
    DXGI_FORMAT cformat;

    if (isHDR)
    {
        tformat = DXGI_FORMAT_R9G9B9E5_SHAREDEXP;
        cformat = bBlockCompress ? DXGI_FORMAT_BC6H_UF16 : DXGI_FORMAT_R9G9B9E5_SHAREDEXP;
    }
    else if (bBlockCompress)
    {
        tformat = bInterpretAsSRGB ? DXGI_FORMAT_R8G8B8A8_UNORM_SRGB : DXGI_FORMAT_R8G8B8A8_UNORM;
        if (bUseBestBC)
            cformat = bInterpretAsSRGB ? DXGI_FORMAT_BC7_UNORM_SRGB : DXGI_FORMAT_BC7_UNORM;
        else if (bPreserveAlpha)
            cformat = bInterpretAsSRGB ? DXGI_FORMAT_BC3_UNORM_SRGB : DXGI_FORMAT_BC3_UNORM;
        else
            cformat = bInterpretAsSRGB ? DXGI_FORMAT_BC1_UNORM_SRGB : DXGI_FORMAT_BC1_UNORM;
    }
    else
    {
        cformat = tformat = bInterpretAsSRGB ? DXGI_FORMAT_R8G8B8A8_UNORM_SRGB : DXGI_FORMAT_R8G8B8A8_UNORM;
    }

    // 원본 색 텍스처의 바이트는 이미 sRGB로 인코딩되어 있다. 픽셀 배치만 변환하고
    // 밉맵을 만들기 직전에 재질의 색공간을 지정해야 색과 선형 데이터가 각각 보존된다.
    const bool authoredLdr = !isDDS && !isHDR;
    const DXGI_FORMAT pixelFormat = authoredLdr ? MakeLinear(tformat) : tformat;
    if (authoredLdr && IsSRGB(image->GetMetadata().format) &&
        !image->OverrideFormat(MakeLinear(image->GetMetadata().format)))
    {
        return false;
    }
    if (bBumpMap)
    {
        std::unique_ptr<ScratchImage> timage(new ScratchImage);

        HRESULT hr = ComputeNormalMap(image->GetImages(), image->GetImageCount(), image->GetMetadata(),
            CNMAP_CHANNEL_LUMINANCE, 10.0f, tformat, *timage);

        if (FAILED(hr))
        {
            Utility::Printf( "Could not compute normal map for \"%ws\" (%08X).\n", filePath.c_str(), hr );
        }
        else
        {
            image.swap(timage);
            info.format = tformat;
        }
    }
    else if (image->GetMetadata().format != pixelFormat)
    {
        std::unique_ptr<ScratchImage> timage(new ScratchImage);

        HRESULT hr = Convert( image->GetImages(), image->GetImageCount(), image->GetMetadata(),
            pixelFormat, TEX_FILTER_DEFAULT, 0.5f, *timage );

        if (FAILED(hr))
        {
            Utility::Printf( "Could not convert \"%ws\" (%08X).\n", filePath.c_str(), hr );
            return false;
        }
        image.swap(timage);
    }
    if (authoredLdr && image->GetMetadata().format != tformat && !image->OverrideFormat(tformat))
    {
        return false;
    }

    // Handle mipmaps
    if (info.mipLevels == 1)
    {
        std::unique_ptr<ScratchImage> timage(new ScratchImage);

        // 선형 데이터 맵에서는 알파가 다른 채널의 평균에 영향을 주면 안 된다.
        const TEX_FILTER_FLAGS filter = authoredLdr && !bInterpretAsSRGB ? TEX_FILTER_SEPARATE_ALPHA : TEX_FILTER_DEFAULT;
        HRESULT hr = GenerateMipMaps(image->GetImages(), image->GetImageCount(), image->GetMetadata(), filter, 0, *timage);
        if (FAILED(hr))
        {
            Utility::Printf("Could not generate mipmaps for \"%ws\" (%08X).\n", filePath.c_str(), hr);
            return false;
        }
        image.swap(timage);
    }

    // Handle compression
    if (bBlockCompress)
    {
        if (info.width % 4 || info.height % 4)
        {
            Utility::Printf( "Texture size (%Iux%Iu) not a multiple of 4 \"%ws\", so skipping compress\n", info.width, info.height, filePath.c_str() );
        }
        else
        {
            std::unique_ptr<ScratchImage> timage(new ScratchImage);

            HRESULT hr = Compress( image->GetImages(), image->GetImageCount(), image->GetMetadata(), cformat, TEX_COMPRESS_DEFAULT, 0.5f, *timage );
            if (FAILED(hr))
            {
                Utility::Printf( "Failing compressing \"%ws\" (WIC: %08X).\n", filePath.c_str(), hr );
            }
            else
            {
                image.swap(timage);
            }
        }
    }

    const std::wstring wDest = outputFile.empty() ? Utility::RemoveExtension(filePath) + L".dds" : outputFile;
    
    // Save DDS
    HRESULT hr = SaveToDDSFile( image->GetImages(), image->GetImageCount(), image->GetMetadata(), DDS_FLAGS_NONE, wDest.c_str() );
    if( FAILED( hr ) )
    {
        Utility::Printf( "Could not write texture to file \"%ws\" (%08X).\n", wDest.c_str(), hr );
        return false;
    }

    return true;
}
