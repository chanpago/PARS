struct Output
{
    float4 position : SV_Position;
    float3 color : COLOR;
    nointerpolation uint id : TEXCOORD;
};

[RootSignature("RootFlags(ALLOW_INPUT_ASSEMBLER_INPUT_LAYOUT)")]
Output main(float2 position : POSITION, float3 color : COLOR, uint id : TEXCOORD)
{
    Output output;
    output.position = float4(position, 0, 1);
    output.color = color;
    output.id = id;
    return output;
}
