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
    
    uint instanceID : SV_InstanceID;
};

[[vk::binding(0)]]
cbuffer Uniforms {
    float4x4 mvp[];
};

VertexOut vsmain(VertexIn vin, uint instanceID : SV_InstanceID) {
    VertexOut vout;
    vout.position = mul(mvp[instanceID], float4(vin.position, 1.0));
    vout.texcoord = vin.texcoord;
    vout.color = vin.color;
    vout.instanceID = instanceID;
    return vout;
}

struct PixelOut {
    float4 color : SV_Target;
};

[[vk::binding(1)]]
Texture2D textures[];

[[vk::binding(1)]]
SamplerState textureSamplers[];

PixelOut psmain(VertexOut vout) {
    PixelOut pout;
    pout.color = textures[vout.instanceID].Sample(textureSamplers[vout.instanceID], vout.texcoord);
    return pout;
}