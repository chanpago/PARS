Texture2D<float3> preview : register(t0);
SamplerState linearClamp : register(s0);

float4 main(float4 position : SV_Position, float2 uv : TEXCOORD0) : SV_Target0
{
    float3 color = max(preview.SampleLevel(linearClamp, uv, 0), 0);
    color = color / (1 + color);
    return float4(pow(color, 1.0 / 2.2), 1);
}
