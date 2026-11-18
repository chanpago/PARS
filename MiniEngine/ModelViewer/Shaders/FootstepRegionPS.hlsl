float4 main(float4 position : SV_POSITION, float3 worldPosition : TEXCOORD0,
    float3 normal : NORMAL, float3 color : COLOR) : SV_TARGET
{
    // 검정은 바닥과 구분되도록 진하게, 나머지 색은 겹친 영역이 보이도록 옅게 칠한다.
    const float opacity = all(color == 0.0) ? 0.85 : 0.16;
    return float4(color, opacity);
}
