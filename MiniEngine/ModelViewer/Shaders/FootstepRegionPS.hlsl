float4 main(float4 position : SV_POSITION, float3 worldPosition : TEXCOORD0,
    float3 normal : NORMAL, float3 color : COLOR) : SV_TARGET
{
    // 영역이 겹치는 곳에서도 바닥과 발을 볼 수 있도록 반투명하게 표시한다.
    return float4(color, 0.16);
}
