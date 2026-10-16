cbuffer Camera : register(b0)
{
    float4x4 ViewProjection;
    float4 Eye;
};

struct Output
{
    float4 position : SV_Position;
    float3 worldPosition : POSITION;
    float3 normal : NORMAL;
    float4 color : COLOR;
    nointerpolation uint id : TEXCOORD;
};

Output main(float3 position : POSITION, float3 normal : NORMAL, float4 color : COLOR, uint id : TEXCOORD)
{
    Output output;
    output.position = mul(ViewProjection, float4(position, 1));
    output.worldPosition = position;
    output.normal = normal;
    output.color = color;
    output.id = id;
    return output;
}
