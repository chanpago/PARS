cbuffer Camera : register(b0)
{
    float4x4 ViewProjection;
    float4 Eye;
};

struct Output { float4 color : SV_Target0; float4 id : SV_Target1; };

Output main(float4 position : SV_Position, float3 worldPosition : POSITION, float3 normal : NORMAL,
    float4 color : COLOR, nointerpolation uint id : TEXCOORD)
{
    float3 n = normalize(normal);
    float3 view = normalize(Eye.xyz - worldPosition);
    float rim = pow(1 - saturate(dot(n, view)), 3);
    float light = 0.6 + 0.4 * saturate(dot(n, normalize(float3(-0.4, 0.8, 0.6))));
    Output output;
    output.color = float4(lerp(color.rgb * light, color.rgb, rim), saturate(color.a + rim * 0.45));
    output.id = float4(float3(id & 255, (id >> 8) & 255, (id >> 16) & 255) / 255.0, 1);
    return output;
}
