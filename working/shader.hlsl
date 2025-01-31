struct VertexIn {
    [[vk::location(0)]]
    float3 position : POSITION;
    [[vk::location(1)]]
    float2 texcoord : TEXCOORD;
    [[vk::location(2)]]
    float3 color : COLOR;
};

struct VertexOut {
    float4 position : SV_Position;
    float2 texcoord : TEXCOORD;
    float3 color : COLOR;
};

[[vk::binding(0)]]
cbuffer Uniforms {
    float4x4 mvp;
};

VertexOut vsmain(VertexIn vin) {
    VertexOut vout;
    vout.position = mul(mvp, float4(vin.position, 1.0));
    vout.texcoord = vin.texcoord;
    vout.color = vin.color;
    return vout;
}

struct PixelOut {
    float4 color : SV_Target;
};

[[vk::binding(1)]]
Texture2D texture;

[[vk::binding(1)]]
SamplerState textureSampler;

PixelOut psmain(VertexOut vout) {
    float4 color = texture.Sample(textureSampler, vout.texcoord);
    
    PixelOut pout;
    pout.color = float4(color.xyz, 1.0);
    if (color.a == 0.0) {
        uint2 s = vout.texcoord * 2.0;
        float3 noTexColor = float3(s.x - s.y, 0.0, s.y - s.x);
        pout.color = float4(noTexColor, 1.0);
    }
    return pout;
}