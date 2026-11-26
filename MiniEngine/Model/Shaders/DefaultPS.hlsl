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
// Author(s):	James Stanard
//              Justin Saunders (ATG)

#include "Common.hlsli"

Texture2D<float4> baseColorTexture          : register(t0);
Texture2D<float3> metallicRoughnessTexture  : register(t1);
Texture2D<float1> occlusionTexture          : register(t2);
Texture2D<float3> emissiveTexture           : register(t3);
Texture2D<float3> normalTexture             : register(t4);

SamplerState baseColorSampler               : register(s0);
SamplerState metallicRoughnessSampler       : register(s1);
SamplerState occlusionSampler               : register(s2);
SamplerState emissiveSampler                : register(s3);
SamplerState normalSampler                  : register(s4);

TextureCube<float3> radianceIBLTexture      : register(t10);
TextureCube<float3> irradianceIBLTexture    : register(t11);
Texture2D<float> texSSAO			        : register(t12);
Texture2D<float> texSunShadow			    : register(t13);

cbuffer MaterialConstants : register(b0)
{
    float4 baseColorFactor;
    float3 emissiveFactor;
    float normalTextureScale;
    float2 metallicRoughnessFactor;
    uint flags;
}

cbuffer GlobalConstants : register(b1)
{
    float4x4 ViewProj;
    float4x4 SunShadowMatrix;
    float3 ViewerPos;
    float3 SunDirection;
    float3 SunIntensity;
    float _pad;
    float IBLRange;
    float IBLBias;
    uint EnableDirectionalLight;
    float IBLIntensity;
}

struct VSOutput
{
    float4 position : SV_POSITION;
    float3 normal : NORMAL;
#ifndef NO_TANGENT_FRAME
    float4 tangent : TANGENT;
#endif
    float2 uv0 : TEXCOORD0;
#ifndef NO_SECOND_UV
    float2 uv1 : TEXCOORD1;
#endif
    float3 worldPos : TEXCOORD2;
    float3 sunShadowCoord : TEXCOORD3;
    float4 debugColor : COLOR0;
};

// Numeric constants
static const float PI = 3.14159265;
static const float3 kDielectricSpecular = float3(0.04, 0.04, 0.04);

// Flag helpers
static const uint BASECOLOR = 0;
static const uint METALLICROUGHNESS = 1;
static const uint OCCLUSION = 2;
static const uint EMISSIVE = 3;
static const uint NORMAL = 4;
#ifdef NO_SECOND_UV
#define UVSET( offset ) vsOutput.uv0
#else
#define UVSET( offset ) lerp(vsOutput.uv0, vsOutput.uv1, (flags >> offset) & 1)
#endif

struct SurfaceProperties
{
    float3 N;
    float3 V;
    float3 c_diff;
    float3 c_spec;
    float specularF90;
    float roughness;
    float alpha; // roughness squared
    float alphaSqr; // alpha squared
    float NdotV;
};

struct LightProperties
{
    float3 L;
    float NdotL;
    float LdotH;
    float NdotH;
};

//
// Shader Math
//

float Pow5(float x)
{
    float xSq = x * x;
    return xSq * xSq * x;
}

// Shlick's approximation of Fresnel
float3 Fresnel_Shlick(float3 F0, float3 F90, float cosine)
{
    return lerp(F0, F90, Pow5(1.0 - cosine));
}

float Fresnel_Shlick(float F0, float F90, float cosine)
{
    return lerp(F0, F90, Pow5(1.0 - cosine));
}

// Burley's diffuse BRDF
float3 Diffuse_Burley(SurfaceProperties Surface, LightProperties Light)
{
    float fd90 = 0.5 + 2.0 * Surface.roughness * Light.LdotH * Light.LdotH;
    return (Surface.c_diff / PI) * Fresnel_Shlick(1, fd90, Light.NdotL) * Fresnel_Shlick(1, fd90, Surface.NdotV);
}

// GGX specular D (normal distribution)
float Specular_D_GGX(SurfaceProperties Surface, LightProperties Light)
{
    float lower = lerp(1, Surface.alphaSqr, Light.NdotH * Light.NdotH);
    return Surface.alphaSqr / (PI * lower * lower);
}

// Schlick-Smith specular geometric visibility function
float G_Schlick_Smith(SurfaceProperties Surface, LightProperties Light)
{
    return 0.25 / max(1e-6, lerp(Surface.NdotV, 1, Surface.alpha * 0.5) * lerp(Light.NdotL, 1, Surface.alpha * 0.5));
}


// A microfacet based BRDF.
// alpha:    This is roughness squared as in the Disney PBR model by Burley et al.
// c_spec:   The F0 reflectance value - 0.04 for non-metals, or RGB for metals.  This is the specular albedo.
// NdotV, NdotL, LdotH, NdotH:  vector dot products
//  N - surface normal
//  V - normalized view vector
//  L - normalized direction to light
//  H - normalized half vector (L+V)/2 -- halfway between L and V
float3 Specular_BRDF(SurfaceProperties Surface, LightProperties Light)
{
    // Normal Distribution term
    float ND = Specular_D_GGX(Surface, Light);

    // Geometric Visibility term
    float GV = G_Schlick_Smith(Surface, Light);

    // Fresnel term
    float3 F = Fresnel_Shlick(Surface.c_spec, Surface.specularF90, Light.LdotH);

    return ND * GV * F;
}

float3 ShadeDirectionalLight(SurfaceProperties Surface, float3 L, float3 c_light)
{
    LightProperties Light;
    Light.L = L;
    Light.NdotL = saturate(dot(Surface.N, L));
    if (Light.NdotL <= 0.0 || Surface.NdotV <= 0.0)
    {
        return 0.0;
    }

    // Half vector
    float3 H = normalize(L + Surface.V);

    // Pre-compute dot products
    Light.LdotH = saturate(dot(L, H));
    Light.NdotH = saturate(dot(Surface.N, H));

    // Diffuse & specular factors
    float3 diffuse = Diffuse_Burley(Surface, Light);
    float3 specular = Specular_BRDF(Surface, Light);

    // Directional light
    return Light.NdotL * c_light * (diffuse + specular);
}

float3 Diffuse_IBL(SurfaceProperties Surface)
{
    // 반구에서 들어오는 확산광은 법선 방향으로 미리 적분되어 있다.
    return Surface.c_diff * irradianceIBLTexture.Sample(defaultSampler, Surface.N);
}

float3 Specular_IBL(SurfaceProperties Surface)
{
    // Karis의 환경 BRDF 근사로 거친 표면의 미세면 가림까지 반사 강도에 반영한다.
    float smoothness = 1.0 - Surface.roughness;
    float grazing = smoothness * min(smoothness * smoothness, exp2(-9.28 * Surface.NdotV));
    grazing += 0.0425 - 0.0275 * Surface.roughness;
    float scale = 1.04 - 0.572 * Surface.roughness - 1.04 * grazing;
    float bias = 1.04 * grazing + 0.022 * Surface.roughness - 0.04;
    float3 reflectance = Surface.c_spec * saturate(scale) + Surface.specularF90 * saturate(bias);
    float lod = Surface.roughness * IBLRange + IBLBias;
    return reflectance * radianceIBLTexture.SampleLevel(cubeMapSampler, reflect(-Surface.V, Surface.N), lod);
}

float SunVisibility(float3 shadowCoord)
{
    if (any(shadowCoord < 0.0) || any(shadowCoord > 1.0))
    {
        return 1.0;
    }
    return texSunShadow.SampleCmpLevelZero(shadowSampler, shadowCoord.xy, shadowCoord.z);
}

float3 ComputeNormal(VSOutput vsOutput)
{
    float3 normal = normalize(vsOutput.normal);

#ifdef NO_TANGENT_FRAME
    return normal;
#else
    // Construct tangent frame
    float3 tangent = normalize(vsOutput.tangent.xyz);
    float3 bitangent = normalize(cross(normal, tangent)) * vsOutput.tangent.w;
    float3x3 tangentFrame = float3x3(tangent, bitangent, normal);

    // Read normal map and convert to SNORM (TODO:  convert all normal maps to R8G8B8A8_SNORM?)
    normal = normalTexture.Sample(normalSampler, UVSET(NORMAL)) * 2.0 - 1.0;

    // glTF spec says to normalize N before and after scaling, but that's excessive
    normal = normalize(normal * float3(normalTextureScale, normalTextureScale, 1));

    // Multiply by transpose (reverse order)
    return normalize(mul(normal, tangentFrame));
#endif
}

[RootSignature(Renderer_RootSig)]
float4 main(VSOutput vsOutput, bool isFrontFace : SV_IsFrontFace) : SV_Target0
{
    // Load and modulate textures
    float4 baseColor = baseColorFactor * baseColorTexture.Sample(baseColorSampler, UVSET(BASECOLOR));
    float2 metallicRoughness = metallicRoughnessFactor * 
        metallicRoughnessTexture.Sample(metallicRoughnessSampler, UVSET(METALLICROUGHNESS)).bg;
    float occlusion = occlusionTexture.Sample(occlusionSampler, UVSET(OCCLUSION));
    float3 emissive = emissiveFactor * emissiveTexture.Sample(emissiveSampler, UVSET(EMISSIVE));
    float3 normal = ComputeNormal(vsOutput);

    SurfaceProperties Surface;
    // 양면 재질의 뒷면은 법선을 뒤집어 카메라 쪽 표면의 조명과 환경 반사를 계산한다.
    Surface.N = isFrontFace ? normal : -normal;
    Surface.V = normalize(ViewerPos - vsOutput.worldPos);
    Surface.NdotV = saturate(dot(Surface.N, Surface.V));
    // 비금속의 반사 강도는 정면 반사와 비스듬히 볼 때의 반사에 함께 적용한다.
    float dielectricWeight = 1.0 - float((flags >> 8) & 255) / 255.0;
    float3 dielectricF0 = kDielectricSpecular * dielectricWeight;
    Surface.c_diff = baseColor.rgb * (1 - dielectricF0) * (1 - metallicRoughness.x);
    Surface.c_spec = lerp(dielectricF0, baseColor.rgb, metallicRoughness.x);
    Surface.specularF90 = lerp(dielectricWeight, 1.0, metallicRoughness.x);
    Surface.roughness = metallicRoughness.y;
    float directRoughness = max(Surface.roughness, 0.045);
    Surface.alpha = directRoughness * directRoughness;
    Surface.alphaSqr = Surface.alpha * Surface.alpha;

    // Begin accumulating light starting with emissive
    float3 colorAccum = emissive;

    if (EnableDirectionalLight != 0)
    {
        colorAccum += SunVisibility(vsOutput.sunShadowCoord) * ShadeDirectionalLight(Surface, SunDirection, SunIntensity);
    }
    float ambientOcclusion = occlusion * texSSAO[uint2(vsOutput.position.xy)];
    colorAccum += IBLIntensity * ambientOcclusion * (Diffuse_IBL(Surface) + Specular_IBL(Surface));

    // TODO: Shade each light using Forward+ tiles

    // 조명이나 텍스처에 가려지지 않는 후보 색을 사용하고, 발목 경계만 부드럽게 잇는다.
    float debugWeight = smoothstep(0.25, 0.65, vsOutput.debugColor.a);
    float3 debugColor = vsOutput.debugColor.rgb / max(vsOutput.debugColor.a, 1e-5);
    return float4(lerp(colorAccum, debugColor, debugWeight), baseColor.a);
}
