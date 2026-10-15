void main(uint vertex : SV_VertexID, out float4 position : SV_Position, out float2 uv : TEXCOORD0)
{
    uv = float2(uint2(vertex, vertex << 1) & 2);
    position = float4(lerp(float2(-1, 1), float2(1, -1), uv), 0, 1);
}
