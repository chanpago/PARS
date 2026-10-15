float4 main(float4 position : SV_POSITION, float3 worldPosition : TEXCOORD0,
    float3 normal : NORMAL, float3 color : COLOR) : SV_TARGET
{
    float light = 0.3 + 0.7 * saturate(dot(normalize(normal), normalize(float3(0.4, 1, 0.6))));
    // One metre tiles on the ground make the physics scale visible.
    if (color.b > color.r && normal.y > 0.9)
    {
        float tile = frac((floor(worldPosition.x) + floor(worldPosition.z)) * 0.5) * 2;
        color *= lerp(0.7, 1.0, tile);
    }
    return float4(color * light, 1);
}
