#include "../../Core/Shaders/ToneMappingUtility.hlsli"

Texture2D<float3> preview : register(t0);
SamplerState linearClamp : register(s0);

float4 main(float4 position : SV_Position, float2 uv : TEXCOORD0) : SV_Target0
{
    float3 linearColor = max(preview.SampleLevel(linearClamp, uv, 0), 0);
    // 게임의 SDR 톤 곡선을 적용한 뒤 UI 텍스처에 표시할 sRGB 값으로 변환한다.
    return float4(ApplySRGBCurve(TM_Stanard(linearColor)), 1);
}
