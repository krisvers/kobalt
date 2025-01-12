struct VertexIn {
    [[vk::location(0)]]
    float3 position : POSITION;
    [[vk::location(1)]]
    float2 texcoord : TEXCOORD;
};

struct VertexOut {
    float4 position : SV_Position;
    float2 texcoord : TEXCOORD;
};

VertexOut vsmain(VertexIn vin) {
    VertexOut vout;
    vout.position = float4(vin.position, 1.0);
    vout.texcoord = vin.texcoord;
    return vout;
}

struct PixelOut {
    float4 color : SV_Target;
};

PixelOut psmain(VertexOut vout) {
    PixelOut pout;
    pout.color = float4(1.0, 0.0, 0.0, 1.0);
    return pout;
}