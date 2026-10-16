cbuffer Pick : register(b0) { uint2 Pixel; }
Texture2D<float4> IdTexture : register(t0);
RWByteAddressBuffer Result : register(u0);

[RootSignature("RootConstants(num32BitConstants=2, b0), DescriptorTable(SRV(t0)), UAV(u0)")]
[numthreads(1, 1, 1)]
void main()
{
    const uint3 color = (uint3)round(saturate(IdTexture.Load(int3(Pixel, 0)).rgb) * 255.0);
    Result.Store(0, color.x | (color.y << 8) | (color.z << 16));
}
