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

VertexOut vsmain(VertexIn vin) {
    VertexOut vout;
    vout.position = float4(vin.position, 1.0);
    vout.texcoord = vin.texcoord;
    vout.color = vin.color;
    return vout;
}

struct PixelOut {
    float4 color : SV_Target;
};

PixelOut psmain(VertexOut vout) {
    PixelOut pout;
    pout.color = float4(vout.color, 1.0);
    return pout;
}