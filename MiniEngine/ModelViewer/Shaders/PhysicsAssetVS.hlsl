cbuffer Camera : register(b0)
{
    float4x4 ViewProjection;
    float4 Eye;
};

cbuffer Body : register(b1)
{
    float4x4 World;
    float4 Dimensions; // XYZ: half extents or radius; W: capsule cylinder half length.
    float4 BodyColor;
    uint BodyId;
};

struct Output
{
    float4 position : SV_Position;
    float3 worldPosition : POSITION;
    float3 normal : NORMAL;
    float4 color : COLOR;
    nointerpolation uint id : TEXCOORD;
};

Output main(float3 position : POSITION, float3 normal : NORMAL, float capOffset : TEXCOORD)
{
    float3 local = position * Dimensions.xyz;
    local.x += capOffset * Dimensions.w;
    float3 worldPosition = mul(World, float4(local, 1)).xyz;
    Output output;
    output.position = mul(ViewProjection, float4(worldPosition, 1));
    output.worldPosition = worldPosition;
    output.normal = normalize(mul((float3x3)World, normal));
    output.color = BodyColor;
    output.id = BodyId;
    return output;
}
