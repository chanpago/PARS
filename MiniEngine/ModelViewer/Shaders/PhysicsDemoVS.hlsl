cbuffer Camera : register(b0)
{
    float4x4 ViewProjection;
};

struct VertexOutput
{
    float4 position : SV_POSITION;
    float3 worldPosition : TEXCOORD0;
    float3 normal : NORMAL;
    float3 color : COLOR;
};

VertexOutput main(float3 position : POSITION, float3 normal : NORMAL, float3 color : COLOR)
{
    VertexOutput output;
    output.position = mul(ViewProjection, float4(position, 1));
    output.worldPosition = position;
    output.normal = normal;
    output.color = color;
    return output;
}
