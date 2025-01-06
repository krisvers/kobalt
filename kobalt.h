#ifndef KOBALT_H
#define KOBALT_H

#define KOBALT_HANDLE(handle_) typedef struct ::kobalt::internal::Object_t* handle_

#define KOBALT_ENUM_BITMASK(T_)                                      \
inline T_ operator|(T_ a, T_ b) {                                    \
    return static_cast<T_>(static_cast<T_>(a) | static_cast<T_>(b)); \
}

#include <cstdint>

namespace kobalt {

namespace internal { struct Object_t { struct Device* device; }; }

KOBALT_HANDLE(Device);
KOBALT_HANDLE(VertexInputState);
KOBALT_HANDLE(GraphicsPipeline);
KOBALT_HANDLE(Buffer);
KOBALT_HANDLE(Texture);
KOBALT_HANDLE(CommandList);

enum class VertexAttributeType {
    UInt8,
    SInt8,
    UNorm8,
    SNorm8,
    
    UInt16,
    SInt16,
    UNorm16,
    SNorm16,
    Half16,
    
    UInt32,
    SInt32,
    UNorm32,
    SNorm32,
    Float32,
    
    UInt64,
    SInt64,
    Double64,
};

enum class TextureFormat {
    R8_UInt,
    R8_SInt,
    R8_UNorm,
    R8_SNorm,
    R8_sRGB,
    
    RG8_UInt,
    RG8_SInt,
    RG8_UNorm,
    RG8_SNorm,
    RG8_sRGB,
    
    RGB8_UInt,
    RGB8_SInt,
    RGB8_UNorm,
    RGB8_SNorm,
    RGB8_sRGB,
    
    BGR8_UInt,
    BGR8_SInt,
    BGR8_UNorm,
    BGR8_SNorm,
    BGR8_sRGB,
    
    RGBA8_UInt,
    RGBA8_SInt,
    RGBA8_UNorm,
    RGBA8_SNorm,
    RGBA8_sRGB,
    
    BGRA8_UInt,
    BGRA8_SInt,
    BGRA8_UNorm,
    BGRA8_SNorm,
    BGRA8_sRGB,
    
    R16_UInt,
    R16_SInt,
    R16_UNorm,
    R16_SNorm,
    R16_Half,
    
    RG16_UInt,
    RG16_SInt,
    RG16_UNorm,
    RG16_SNorm,
    RG16_Half,
    
    RGB16_UInt,
    RGB16_SInt,
    RGB16_UNorm,
    RGB16_SNorm,
    RGB16_Half,
    
    RGBA16_UInt,
    RGBA16_SInt,
    RGBA16_UNorm,
    RGBA16_SNorm,
    RGBA16_Half,
    
    R32_UInt,
    R32_SInt,
    R32_Float,
    
    RG32_UInt,
    RG32_SInt,
    RG32_Float,
    
    RGB32_UInt,
    RGB32_SInt,
    RGB32_Float,
    
    RGBA32_UInt,
    RGBA32_SInt,
    RGBA32_Float,
    
    R64_UInt,
    R64_SInt,
    R64_Float,
    
    RG64_UInt,
    RG64_SInt,
    RG64_Float,
    
    RGB64_UInt,
    RGB64_SInt,
    RGB64_Float,
    
    RGBA64_UInt,
    RGBA64_SInt,
    RGBA64_Float,
    
    D16_UNorm,
    D32_Float,
    
    S8_UInt,
    
    D16_UNorm_S8_UInt,
    D24_UNorm_S8_UInt,
    D32_Float_S8_UInt,
};

enum class Topology {
    PointList,
    LineList,
    LineStrip,
    TriangleList,
    TriangleStrip,
    TriangleFan,
    PatchList,
};

enum class MemoryLocation {
    DeviceLocal,
    HostLocal,
};

enum class BufferUsage {
    VertexBuffer = 0x1,
    IndexBuffer = 0x2,
    StorageBuffer = 0x4,
    UniformBuffer = 0x8,
    TransferSrc = 0x10,
    TransferDst = 0x20,
};

KOBALT_ENUM_BITMASK(BufferUsage);

enum class TextureUsage {
    SampledTexture = 0x1,
    StorageTexture = 0x2,
    RenderTarget = 0x4,
    DepthTarget = 0x8,
    StencilTarget = 0x10,
};

KOBALT_ENUM_BITMASK(TextureUsage);

enum class CommandListSpecialization {
    Graphics = 0x1,
    Transfer = 0x2,
    Compute = 0x4,
    
    All = Graphics | Transfer | Compute,
};

KOBALT_ENUM_BITMASK(CommandListSpecialization);

struct DeviceSupport {
    bool dynamicRenderState = false;
};

struct VertexAttribute {
    uint32_t location;
    uint32_t binding;
    VertexAttributeType type;
    uint32_t count;
};

struct VertexBinding {
    uint32_t binding;
    uint32_t size;
    bool advancePerInstance;
};

bool createDefaultDevice(Device& device, DeviceSupport support);
bool createDevice(Device& device, uint32_t id, DeviceSupport support);

bool createShaderSPIRV(Shader& shader, Device device, void const* data, uint32_t size);
bool createVertexInputState(VertexInputState& vertexInputState, Topology topology, VertexAttribute const* vertexAttributes, uint32_t
vertexAttributeCount, VertexBinding const* vertexBindings, uint32_t vertexBindingCount);

bool createBuffer(Buffer& buffer, Device device, uint64_t size, MemoryLocation location, BufferUsage usage);
bool uploadBufferData(Buffer buffer, uint64_t offset, void const* data, uint64_t size);
bool downloadBufferData(Buffer buffer, uint64_t offset, void* data, uint64_t size);
bool mapBuffer(void*& mappedPointer, Buffer buffer, uint64_t offset, uint64_t size);

bool createTexture2D(Texture& texture, Device device, uint32_t width, uint32_t height, TextureFormat format, MemoryLocation location, TextureUsage usage);
bool createTexture3D(Texture& texture, Device device, uint32_t width, uint32_t height, uint32_t depth, TextureFormat format, MemoryLocation location, TextureUsage
usage);
bool createTexture2DArray(Texture& texture, Device device, uint32_t width, uint32_t height, uint32_t layers, TextureFormat format, MemoryLocation location,
TextureUsage usage);
bool createTexture3DArray(Texture& texture, Device device, uint32_t width, uint32_t height, uint32_t depth, TextureFormat format, uint32_t layers, MemoryLocation
location, TextureUsage usage);

bool createCommandList(CommandList& commandList, Device device, CommandListSpecialization specialization);
bool createSecondaryCommandList(CommandList& commandList, Device dev);
bool executeCommandList(CommandList commandList);

namespace cmd {

bool beginRecording(CommandList commandList);
bool bindGraphicsPipeline(CommandList commandList, GraphicsPipeline pipeline);
bool bindVertexBuffer(CommandList commandList, uint32_t index, Buffer buffer, uint64_t offset);
bool bindIndexBuffer(CommandList commandList, Buffer buffer, uint64_t offset);
bool recordCommands(CommandList commandList, CommandList secondaryList);
bool endRecording(CommandList commandList);

} /* namespace cmd */

} /* namespace kobalt */

#endif /* #ifndef KOBALT_H */
