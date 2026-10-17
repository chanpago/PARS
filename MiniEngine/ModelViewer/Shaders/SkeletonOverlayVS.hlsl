struct Output
{
    float4 position : SV_Position;
    float4 color : COLOR;
    nointerpolation uint id : TEXCOORD;
};

[RootSignature("RootFlags(ALLOW_INPUT_ASSEMBLER_INPUT_LAYOUT)")]
Output main(float4 position : POSITION, float4 color : COLOR, uint id : TEXCOORD)
{
    Output output;
    output.position = position;
    output.color = color;
    output.id = id;
    return output;
}
