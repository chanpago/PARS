struct Output
{
    float4 color : SV_Target0;
    float4 id : SV_Target1;
};

Output main(float4 position : SV_Position, float3 color : COLOR, nointerpolation uint id : TEXCOORD)
{
    Output output;
    output.color = float4(color, 1);
    output.id = float4(float3(id & 255, (id >> 8) & 255, (id >> 16) & 255) / 255.0, 1);
    return output;
}
