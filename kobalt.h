#ifndef KOBALT_H
#define KOBALT_H

#include <cstdint>

#ifdef KOBALT_GLFW

#if defined(_glfw3_h_) && !defined(VK_VERSION_1_0)
#define KOBALT_INTERNAL_GLFW_VULKAN_NOT_DEFINED
#else
#define VK_VERSION_1_0
#include <GLFW/glfw3.h>
#endif /* #ifdef _glfw3_h_ */

#endif /* #ifdef KOBALT_GLFW */

#define KOBALT_HANDLE(handle_) typedef struct ::kobalt::Object_t* handle_

#define KOBALT_ENUM_BITMASK(T_) \
inline T_ operator|(T_ a, T_ b) { \
    return static_cast<T_>(static_cast<uint32_t>(a) | static_cast<uint32_t>(b)); \
} \
inline T_ operator&(T_ a, T_ b) { \
    return static_cast<T_>(static_cast<uint32_t>(a) & static_cast<uint32_t>(b)); \
} \
inline T_ operator|=(T_& a, T_ b) { \
    a = a | b; \
    return a; \
}

namespace kobalt {

namespace internal {

enum class ObjectType {
    Unknown = 0,

    Device,

    RenderAttachmentState,
    RenderSubpass,
    RenderPass,
    Framebuffer,

    Shader,
    VertexInputState,
    RasterizationState,
    Pipeline,

    Buffer,
    BufferView,
    Texture,
    TextureView,

    CommandList,

    QueueSync,
    HostSync,
    StageSync,

    WSI_Swapchain,
};

} /* namespace internal */
    
struct Object_t {
    struct Object_t* device;
    internal::ObjectType type;
};

KOBALT_HANDLE(Device);
KOBALT_HANDLE(Shader);
KOBALT_HANDLE(VertexInputState);
KOBALT_HANDLE(RasterizationState);
KOBALT_HANDLE(RenderAttachmentState);
KOBALT_HANDLE(DepthStencilState);
KOBALT_HANDLE(BlendState);
KOBALT_HANDLE(RenderSubpass);
KOBALT_HANDLE(RenderPass);
KOBALT_HANDLE(Framebuffer);
KOBALT_HANDLE(Pipeline);
KOBALT_HANDLE(Buffer);
KOBALT_HANDLE(BufferView);
KOBALT_HANDLE(Texture);
KOBALT_HANDLE(TextureView);
KOBALT_HANDLE(CommandList);
KOBALT_HANDLE(QueueSync);
KOBALT_HANDLE(HostSync);
KOBALT_HANDLE(StageSync);

enum class DebugSeverity {
    Warning,
    Error,
};

enum class DeviceVendor {
    Unknown,

    NVIDIA = 0x10de,
    AMD = 0x1002,
    Intel = 0x8086,
    ARM = 0x13b5,
    Qualcomm = 0x5143,
    ImgTec = 0x1010,
    Apple = 0x106b,
    Mesa = 0x10005,
};

enum class DeviceType {
    Discrete,
    Integrated,
    CPU,
    Virtual,
};

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
    Float32,
    
    UInt64,
    SInt64,
    Double64,
};

enum class CompareOp {
    Never,
    Less,
    LEqual,
    Equal,
    GEqual,
    Greater,
    Always,
};

enum class StencilOp {
    Zero,
    Keep,
    Replace,
    IncClamp,
    DecClamp,
    Invert,
    IncWrap,
    DecWrap,
};

enum class LogicOp {
    NoOp,
    Clear,
    Set,
    And,
    RevAnd,
    InvAnd,
    Copy,
    InvCopy,
    Xor,
    Or,
    RevOr,
    InvOr,
    Nor,
    XNor,
    Not,
    Nand,

    /* aliases */
    Equivalent = XNor,
};

enum class BlendOp {
    Add,
    Subtract,
    RevSubtract,
    Min,
    Max,
};

enum class BlendFactor {
    Zero,
    One,
    SrcColor,
    InvSrcColor,
    DstColor,
    InvDstColor,
    SrcAlpha,
    InvSrcAlpha,
    DstAlpha,
    InvDstAlpha,
    ConstColor,
    InvConstColor,
    ConstAlpha,
    InvConstAlpha,
    SrcAlphaSat,
    Src1Color,
    InvSrc1Color,
    Src1Alpha,
    InvSrc1Alpha,
};

enum class TextureFormat {
    Undefined = 0,

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

enum class TextureLayout {
    Undefined,
    RenderTarget,
    DepthStencilTarget,
    DepthStencilRead,
    ShaderRead,
    TransferSrc,
    TransferDst,
    PresentSrc,
};

enum class TextureAspect {
    Color = 0x1,
    Depth = 0x2,
    Stencil = 0x4,
};

KOBALT_ENUM_BITMASK(TextureAspect);

enum class ResourceAccess {
    None = 0x0,

    IndirectRead = 0x1,
    IndexRead = 0x2,
    VertexAttributeRead = 0x4,
    UniformRead = 0x8,
    InputAttachmentRead = 0x10,
    ShaderRead = 0x20,
    ShaderWrite = 0x40,
    RenderTargetRead = 0x80,
    RenderTargetWrite = 0x100,
    DepthStencilTargetRead = 0x200,
    DepthStencilTargetWrite = 0x400,
    TransferRead = 0x800,
    TransferWrite = 0x1000,
    HostRead = 0x2000,
    HostWrite = 0x4000,
    MemoryRead = 0x8000,
    MemoryWrite = 0x10000,
};

KOBALT_ENUM_BITMASK(ResourceAccess);

enum class ComponentSwizzle {
    Identity,
    Zero,
    One,
    R,
    G,
    B,
    A,

    X = R,
    Y = G,
    Z = B,
    W = A,
};

enum class TextureDimensions {
    Texture1D,
    Texture2D,
    Texture3D,
    Array1D,
    Array2D,
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

enum class QueueType {
    Graphics = 0x1,
    Compute = 0x2,
    Transfer = 0x4,
    General = 0x8,
};

enum class QueueTransfer {
    Identity,

    Graphics,
    Compute,
    Transfer,
    General,
};

enum class FillMode {
    Fill,
    Wireframe,
    Points,
};

enum class CullMode {
    None = 0x0,
    Back = 0x1,
    Front = 0x2,
};

KOBALT_ENUM_BITMASK(CullMode);

enum class FrontFace {
    Clockwise,
    CounterClockwise,
};

enum class RenderAttachmentLoadOp {
    DontCare,
    Clear,
    Load,
};

enum class RenderAttachmentStoreOp {
    DontCare,
    Store,
};

enum class PipelineStage {
    Top = 0x1,
    DrawIndirect = 0x2,
    VertexInput = 0x4,
    VertexShader = 0x8,
    TessellationControlShader = 0x10,
    TessellationEvaluationShader = 0x20,
    GeometryShader = 0x40,
    PreFragmentTests = 0x80,
    FragmentShader = 0x100,
    PostFragmentTests = 0x200,
    RenderTarget = 0x400,
    ComputeShader = 0x800,
    Transfer = 0x1000,
    Bottom = 0x2000,

    /* aliases */
    PixelShader = FragmentShader,

    PreFragmentDepthTest = PreFragmentTests,
    PreFragmentScissorTest = PreFragmentTests,
    DepthStencilTargetLoad = PreFragmentTests,

    PostFragmentDepthTest = PostFragmentTests,
    PostFragmentScissorTest = PostFragmentTests,
    DepthStencilTargetStore = PostFragmentTests,

    BlendingOperations = RenderTarget,
    LogicOperations = RenderTarget,
    RenderTargetLoad = RenderTarget,
    RenderTargetStore = RenderTarget,

    Copy = Transfer,
    Clear = Transfer,
};

KOBALT_ENUM_BITMASK(PipelineStage);

struct DeviceSupport {
    bool dynamicRenderPass;
    bool swapchain;
};

inline bool operator==(DeviceSupport const& a, DeviceSupport const& b) {
    return (a.dynamicRenderPass == b.dynamicRenderPass && a.swapchain == b.swapchain);
}

inline bool operator<(DeviceSupport const& a, DeviceSupport const& b) {
    return (a.dynamicRenderPass < b.dynamicRenderPass && a.swapchain < b.swapchain);
}

inline bool operator<=(DeviceSupport const& a, DeviceSupport const& b) {
    return a == b || a < b;
}

inline bool operator>(DeviceSupport const& a, DeviceSupport const& b) {
    return !(a <= b);
}

inline bool operator>=(DeviceSupport const& a, DeviceSupport const& b) {
    return a > b || a == b;
}

struct DeviceAdapterInfo {
    DeviceVendor vendor;
    DeviceType type;
    DeviceSupport support;

    uint32_t maxTextureSize1D;
    uint32_t maxTextureSize2D;
    uint32_t maxTextureSize3D;

    uint32_t maxRenderTargetSize[3];
    uint32_t maxRenderTargets;

    uint32_t maxComputeWorkGroupCount[3];
    uint32_t maxComputeWorkGroupSize[3];

    uint64_t localMemorySize;
    uint64_t otherMemorySize;

    char name[64];
};

struct VertexAttribute {
    uint32_t location;
    uint32_t binding;
    VertexAttributeType type;
    uint32_t count;
    uint32_t offset;
};

struct VertexBinding {
    uint32_t binding;
    uint32_t size;
    bool advancePerInstance;
};

struct RenderAttachment {
    TextureFormat format;
    uint32_t sampleCount;
    RenderAttachmentLoadOp loadOp;
    RenderAttachmentStoreOp storeOp;
    RenderAttachmentLoadOp stencilLoadOp;
    RenderAttachmentStoreOp stencilStoreOp;
    TextureLayout initialLayout;
    TextureLayout finalLayout;
};

struct RenderAttachmentReference {
    uint32_t index;
    TextureLayout layout;
};

struct GraphicsPipelineAttachment {
    TextureFormat format;
    uint32_t sampleCount;
};

struct FramebufferAttachment {
    uint32_t width;
    uint32_t height;
    uint32_t layerCount;
    TextureUsage usage;
};

struct ClearColor {
    float rgbaFloat[4];
};

struct ClearDepthStencil {
    float depth;
    uint32_t stencil;
};

struct ClearValue {
    ClearColor color;
    ClearDepthStencil depthStencil;
};

struct DynamicRenderAttachment {
    TextureView view;
    TextureLayout layout;
    RenderAttachmentLoadOp loadOp;
    RenderAttachmentStoreOp storeOp;
    ClearValue clearValue;
};

struct DynamicRenderPassInfo {
    uint32_t viewMask;
    TextureFormat const* renderTargetFormats;
    uint32_t renderTargetCount;
    TextureFormat const* depthTargetFormat;
    TextureFormat const* stencilTargetFormat;
};

struct GraphicsPipelineRenderPassInfo {
    DynamicRenderPassInfo const* dynamicRenderPass;
    RenderPass renderPass;
};

struct Rectangle {
    int32_t x;
    int32_t y;
    uint32_t width;
    uint32_t height;
};

struct TextureSubresource {
    TextureAspect aspect;
    uint32_t mipBase;
    uint32_t mipCount;
    uint32_t layerBase;
    uint32_t layerCount;
};

struct ComponentMapping {
    ComponentSwizzle r;
    ComponentSwizzle g;
    ComponentSwizzle b;
    ComponentSwizzle a;
};

typedef void (*DebugCallback)(const char* message, DebugSeverity severity, Object_t* srcObject);

/* you are able to call this before initialization */
void setDebugCallback(DebugCallback callback);

bool init(bool debug = false);
void deinit();

void setDebugName(Object_t* object, const char* name);
void destroy(Object_t* object);

bool enumerateDeviceAdapters(DeviceAdapterInfo& adapterInfo, uint32_t id);

bool createDefaultDevice(Device& device, DeviceSupport support);
bool createDevice(Device& device, uint32_t id, DeviceSupport support);

void waitForDevice(Device device);

namespace wsi {

KOBALT_HANDLE(Swapchain);

enum class PresentMode {
    Immediate = 0x1,
    VSync = 0x2,
    Mailbox = 0x4,

    Any = Immediate | VSync | Mailbox,
};

KOBALT_ENUM_BITMASK(PresentMode);

enum class SwapchainStatus {
    ResizeRequired,
    Timeout,
    Ready,
    NotReady,
    Suboptimal,
    PresentationSuccess,
    PresentationFailure,
};

TextureFormat getSwapchainFormat(Swapchain swapchain);
uint32_t getSwapchainTextureCount(Swapchain swapchain);
PresentMode getSwapchainPresentMode(Swapchain swapchain);

Texture getSwapchainBackbuffer(Swapchain swapchain);
bool prepareNextSwapchainTexture(SwapchainStatus* status, Swapchain swapchain, uint64_t timeout, QueueSync queueSync, HostSync hostSync);

bool present(SwapchainStatus* status, Swapchain swapchain, QueueSync const* waitSyncs, uint32_t waitSyncCount);

#ifdef KOBALT_GLFW

namespace glfw {

bool createSwapchain(Swapchain& swapchain, Device device, GLFWwindow* window, uint32_t width, uint32_t height, uint32_t layerCount, TextureUsage usage, bool sRGBFormat, PresentMode presentMode);

} /* namespace glfw */

#endif /* #ifdef KOBALT_GLFW */

} /* namespace wsi */

/* render pass */
bool createRenderAttachmentState(RenderAttachmentState& renderAttachmentState, Device device, RenderAttachment const* renderAttachments, uint32_t renderAttachmentCount);
bool createRenderSubpass(RenderSubpass& subpass, Device device, RenderAttachmentReference const* inputAttachments, uint32_t inputAttachmentCount, RenderAttachmentReference const* renderTargets, uint32_t renderTargetCount, RenderAttachmentReference const* depthStencilTarget);
bool createRenderPass(RenderPass& renderPass, Device device, RenderSubpass const* subpasses, uint32_t subpassCount, RenderAttachmentState renderAttachmentState);

/* pipeline */
bool createShaderSPIRV(Shader& shader, Device device, uint32_t const* data, size_t size);
bool createVertexInputState(VertexInputState& vertexInputState, Device device, Topology topology, VertexAttribute const* vertexAttributes, uint32_t vertexAttributeCount, VertexBinding const* vertexBindings, uint32_t vertexBindingCount);
bool createRasterizationState(RasterizationState& rasterizationState, Device device, FillMode fillMode, CullMode cullMode, FrontFace frontFace, float depthBias, float depthBiasClamp, float slopeScaledDepthBias);

bool createGraphicsPipeline(Pipeline& pipeline, Device device, VertexInputState vertexInputState, RasterizationState rasterizationState, DepthStencilState depthStencilState, BlendState blendState, Shader vertexShader, Shader fragmentShader, Shader geometryShader, GraphicsPipelineAttachment const* inputAttachments, uint32_t inputAttachmentCount, GraphicsPipelineAttachment const* renderTargets, uint32_t renderTargetCount, GraphicsPipelineAttachment const* depthStencilTarget, uint32_t subpass, bool dynamicRenderPass, uint32_t viewMask);
bool storePipeline(Pipeline pipeline, void* data, uint64_t* size);
bool loadPipeline(Pipeline& pipeline, void* data, uint64_t size);

/* resources */
bool createBuffer(Buffer& buffer, Device device, uint64_t size, MemoryLocation location, BufferUsage usage);
bool uploadBufferData(Buffer buffer, uint64_t offset, void const* data, uint64_t size);
bool downloadBufferData(Buffer buffer, uint64_t offset, void* data, uint64_t size);
bool mapBuffer(void*& mappedPointer, Buffer buffer, uint64_t offset, uint64_t size);
void unmapBuffer(Buffer buffer, void* pointer);

bool createTexture(Texture& texture, Device device, TextureDimensions dimensions, uint32_t width, uint32_t height, uint32_t depth, uint32_t layerCount, uint32_t mipCount, uint32_t samples, TextureFormat format, MemoryLocation location, TextureUsage usage);
bool createTextureView(TextureView& view, Device device, Texture texture, TextureFormat format, TextureDimensions dimensions, ComponentMapping const* mapping, TextureSubresource const* subresource);

/* command list */
bool createCommandList(CommandList& commandList, Device device, QueueType queueType, bool isSecondary);
bool executeCommandList(CommandList commandList, QueueSync const* waitQueueSyncs, PipelineStage const* waitDstStages, uint32_t waitQueueSyncCount, QueueSync const* signalQueueSyncs, uint32_t signalQueueSyncCount, HostSync hostSync);
bool resetCommandList(CommandList commandList);

bool beginRecordingCommandList(CommandList commandList);
bool recordSecondaryCommands(CommandList commandList, CommandList secondaryList);
bool endRecordingCommandList(CommandList commandList);

bool waitForQueue(Device device, QueueType queueType);

/* synchronization */
bool createQueueSync(QueueSync& sync, Device device);

bool createHostSync(HostSync& sync, Device device, bool signaled);
bool waitForHostSync(HostSync sync, uint64_t timeout);
bool resetHostSync(HostSync sync);
bool getHostSyncSignal(HostSync sync);

bool createStageSync(StageSync& sync, Device device);
bool setStageSyncSignal(StageSync sync, bool signal);
bool getStageSyncSignal(StageSync sync);

namespace cmd {

/* synchronization */
bool setupStageSync(CommandList commandList, StageSync sync, PipelineStage stage);
bool resetStageSync(CommandList commandList, StageSync sync, PipelineStage stage);
bool waitStageSync(CommandList commandList, StageSync sync, PipelineStage srcStage, PipelineStage dstStage);

bool executionBarrier(CommandList commandList, PipelineStage srcStage, PipelineStage dstStage);
bool globalResourceBarrier(CommandList commandList, ResourceAccess srcAccess, ResourceAccess dstAccess);
bool textureBarrier(CommandList commandList, ResourceAccess srcAccess, ResourceAccess dstAccess, QueueTransfer srcQueue, QueueTransfer dstQueue, TextureLayout oldLayout, TextureLayout newLayout, Texture texture, TextureSubresource const* subresource);
bool bufferBarrier(CommandList commandList, ResourceAccess srcAccess, ResourceAccess dstAccess, QueueTransfer srcQueue, QueueTransfer dstQueue, Buffer buffer, uint64_t offset, uint64_t size);

/* binding */
bool bindGraphicsPipeline(CommandList commandList, Pipeline pipeline);
bool bindVertexBuffer(CommandList commandList, uint32_t index, Buffer buffer, uint64_t offset);
bool bindIndexBuffer(CommandList commandList, Buffer buffer, uint64_t offset);

/* render passes */
bool beginRenderPass(CommandList commandList, RenderPass renderPass, Framebuffer framebuffer, Rectangle renderArea, ClearColor const* clearColors, uint32_t clearColorCount, ClearDepthStencil const* clearDepthStencil, bool secondaryCommandList);
bool beginDynamicRenderPass(CommandList commandList, Rectangle renderArea, uint32_t layerCount, uint32_t viewMask, DynamicRenderAttachment const* renderTargets, uint32_t renderTargetCount, DynamicRenderAttachment const* depthTarget, DynamicRenderAttachment const* stencilTarget);
bool nextSubpass(CommandList commandList, bool secondaryCommandList);
bool endRenderPass(CommandList commandList);

/* drawing */
bool draw(CommandList commandList, uint32_t vertex, uint32_t count);
bool drawIndexed(CommandList commandList, uint32_t index, uint32_t count);

} /* namespace cmd */

} /* namespace kobalt */

#if !defined(KOBALT_IMPL) && defined(KOBALT_INTERNAL_GLFW_VULKAN_NOT_DEFINED)
#undef KOBALT_INTERNAL_GLFW_VULKAN_NOT_DEFINED
#endif /* #ifndef KOBALT_IMPL */

#ifdef KOBALT_IMPL

#define PRISM_VK_IMPL
#include "prism_vk.hpp"

#ifdef _WIN32
#define VK_USE_PLATFORM_WIN32_KHR
#elif __APPLE__
#include <TargetConditionals.h>
#define VK_USE_PLATFORM_METAL_EXT

#ifdef TARGET_OS_MAC
#define VK_USE_PLATFORM_MACOS_MVK
#elif defined(TARGET_OS_IOS)
#define VK_USE_PLATFORM_IOS_MVK
#endif /* #ifdef TARGET_OS_MAC */

#else
#define VK_USE_PLATFORM_XCB_KHR
#define VK_USE_PLATFORM_XLIB_KHR
#endif /* #ifdef _WIN32 */
#include <vulkan/vulkan.h>

#ifdef KOBALT_DEFAULT_DEBUG_MESSENGER
#include <cstdio>
#endif /* #ifdef KOBALT_DEFAULT_DEBUG_MESSENGER */

#ifdef KOBALT_INTERNAL_GLFW_VULKAN_NOT_DEFINED
extern "C" {

GLFWAPI VkResult glfwCreateWindowSurface(VkInstance instance, GLFWwindow* window, const VkAllocationCallbacks* allocator, VkSurfaceKHR* surface);

}
#endif /* #ifdef KOBALT_INTERNAL_GLFW_VULKAN_NOT_DEFINED */

namespace kobalt {

KOBALT_ENUM_BITMASK(kobalt::QueueType);

namespace internal {

inline VkObjectType objectTypeToVkObjectType(ObjectType type) {
    switch (type) {
        case ObjectType::Device:        return VK_OBJECT_TYPE_DEVICE;
        case ObjectType::RenderPass:    return VK_OBJECT_TYPE_RENDER_PASS;
        case ObjectType::Framebuffer:   return VK_OBJECT_TYPE_FRAMEBUFFER;
        case ObjectType::Shader:        return VK_OBJECT_TYPE_SHADER_MODULE;
        case ObjectType::Pipeline:      return VK_OBJECT_TYPE_PIPELINE;
        case ObjectType::Buffer:        return VK_OBJECT_TYPE_BUFFER;
        case ObjectType::BufferView:    return VK_OBJECT_TYPE_BUFFER_VIEW;
        case ObjectType::Texture:       return VK_OBJECT_TYPE_IMAGE;
        case ObjectType::TextureView:   return VK_OBJECT_TYPE_IMAGE_VIEW;
        case ObjectType::CommandList:   return VK_OBJECT_TYPE_COMMAND_BUFFER;
        case ObjectType::QueueSync:     return VK_OBJECT_TYPE_SEMAPHORE;
        case ObjectType::HostSync:      return VK_OBJECT_TYPE_FENCE;
        case ObjectType::StageSync:     return VK_OBJECT_TYPE_EVENT;
        case ObjectType::WSI_Swapchain: return VK_OBJECT_TYPE_SWAPCHAIN_KHR;
        default: break;
    }

    return VK_OBJECT_TYPE_UNKNOWN;
}

inline VkPrimitiveTopology topologyToVkPrimitiveTopology(Topology topology) {
    switch (topology) {
        case Topology::PointList:       return VK_PRIMITIVE_TOPOLOGY_POINT_LIST;
        case Topology::LineList:        return VK_PRIMITIVE_TOPOLOGY_LINE_LIST;
        case Topology::LineStrip:       return VK_PRIMITIVE_TOPOLOGY_LINE_STRIP;
        case Topology::TriangleList:    return VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST;
        case Topology::TriangleStrip:   return VK_PRIMITIVE_TOPOLOGY_TRIANGLE_STRIP;
        case Topology::TriangleFan:     return VK_PRIMITIVE_TOPOLOGY_TRIANGLE_FAN;
        case Topology::PatchList:       return VK_PRIMITIVE_TOPOLOGY_PATCH_LIST;
        default: break;
    }

    return VK_PRIMITIVE_TOPOLOGY_MAX_ENUM;
}

inline VkPolygonMode fillModeToVkPolygonMode(FillMode fillMode) {
    switch (fillMode) {
        case FillMode::Fill:        return VK_POLYGON_MODE_FILL;
        case FillMode::Wireframe:   return VK_POLYGON_MODE_LINE;
        case FillMode::Points:      return VK_POLYGON_MODE_POINT;
        default: return VK_POLYGON_MODE_MAX_ENUM;
    }
}

inline VkCullModeFlags cullModeToVkCullModeFlags(CullMode cullMode) {
    VkCullModeFlags flags = 0;
    if ((cullMode & CullMode::Back) != CullMode::None) {
        flags |= VK_CULL_MODE_BACK_BIT;
    }

    if ((cullMode & CullMode::Front) != CullMode::None) {
        flags |= VK_CULL_MODE_FRONT_BIT;
    }

    return flags;
}

inline VkFrontFace frontFaceToVkFrontFace(FrontFace frontFace) {
    switch (frontFace) {
        case FrontFace::Clockwise:          return VK_FRONT_FACE_CLOCKWISE;
        case FrontFace::CounterClockwise:   return VK_FRONT_FACE_COUNTER_CLOCKWISE;
        default: break;
    }

    return VK_FRONT_FACE_MAX_ENUM;
}

inline uint32_t vertexAttributeTypeSize(VertexAttributeType type) {
    switch (type) {
        case VertexAttributeType::UInt8:
        case VertexAttributeType::SInt8:
        case VertexAttributeType::UNorm8:
        case VertexAttributeType::SNorm8:
            return 1;

        case VertexAttributeType::UInt16:
        case VertexAttributeType::SInt16:
        case VertexAttributeType::UNorm16:
        case VertexAttributeType::SNorm16:
        case VertexAttributeType::Half16:
            return 2;

        case VertexAttributeType::UInt32:
        case VertexAttributeType::SInt32:
        case VertexAttributeType::Float32:
            return 4;

        case VertexAttributeType::UInt64:
        case VertexAttributeType::SInt64:
        case VertexAttributeType::Double64:
            return 8;

        default: break;
    }

    return 0;
}

inline VkFormat vertexAttributeTypeToVkFormat(VertexAttributeType type, uint32_t count) {
    switch (type) {
        case VertexAttributeType::UInt8:
            switch (count) {
                case 1: return VK_FORMAT_R8_UINT;
                case 2: return VK_FORMAT_R8G8_UINT;
                case 3: return VK_FORMAT_R8G8B8_UINT;
                case 4: return VK_FORMAT_R8G8B8A8_UINT;
                default: break;
            }
            break;
        case VertexAttributeType::SInt8:
            switch (count) {
                case 1: return VK_FORMAT_R8_SINT;
                case 2: return VK_FORMAT_R8G8_SINT;
                case 3: return VK_FORMAT_R8G8B8_SINT;
                case 4: return VK_FORMAT_R8G8B8A8_SINT;
                default: break;
            }
            break;
        case VertexAttributeType::UNorm8:
            switch (count) {
                case 1: return VK_FORMAT_R8_UNORM;
                case 2: return VK_FORMAT_R8G8_UNORM;
                case 3: return VK_FORMAT_R8G8B8_UNORM;
                case 4: return VK_FORMAT_R8G8B8A8_UNORM;
                default: break;
            }
            break;
        case VertexAttributeType::SNorm8:
            switch (count) {
                case 1: return VK_FORMAT_R8_SNORM;
                case 2: return VK_FORMAT_R8G8_SNORM;
                case 3: return VK_FORMAT_R8G8B8_SNORM;
                case 4: return VK_FORMAT_R8G8B8A8_SNORM;
                default: break;
            }
            break;
        case VertexAttributeType::UInt16:
            switch (count) {
                case 1: return VK_FORMAT_R16_UINT;
                case 2: return VK_FORMAT_R16G16_UINT;
                case 3: return VK_FORMAT_R16G16B16_UINT;
                case 4: return VK_FORMAT_R16G16B16A16_UINT;
                default: break;
            }
            break;
        case VertexAttributeType::SInt16:
            switch (count) {
                case 1: return VK_FORMAT_R16_SINT;
                case 2: return VK_FORMAT_R16G16_SINT;
                case 3: return VK_FORMAT_R16G16B16_SINT;
                case 4: return VK_FORMAT_R16G16B16A16_SINT;
                default: break;
            }
            break;
        case VertexAttributeType::UNorm16:
            switch (count) {
                case 1: return VK_FORMAT_R16_UNORM;
                case 2: return VK_FORMAT_R16G16_UNORM;
                case 3: return VK_FORMAT_R16G16B16_UNORM;
                case 4: return VK_FORMAT_R16G16B16A16_UNORM;
                default: break;
            }
            break;
        case VertexAttributeType::SNorm16:
            switch (count) {
                case 1: return VK_FORMAT_R16_SNORM;
                case 2: return VK_FORMAT_R16G16_SNORM;
                case 3: return VK_FORMAT_R16G16B16_SNORM;
                case 4: return VK_FORMAT_R16G16B16A16_SNORM;
                default: break;
            }
            break;
        case VertexAttributeType::Half16:
            switch (count) {
                case 1: return VK_FORMAT_R16_SFLOAT;
                case 2: return VK_FORMAT_R16G16_SFLOAT;
                case 3: return VK_FORMAT_R16G16B16_SFLOAT;
                case 4: return VK_FORMAT_R16G16B16A16_SFLOAT;
                default: break;
            }
            break;
        case VertexAttributeType::UInt32:
            switch (count) {
                case 1: return VK_FORMAT_R32_UINT;
                case 2: return VK_FORMAT_R32G32_UINT;
                case 3: return VK_FORMAT_R32G32B32_UINT;
                case 4: return VK_FORMAT_R32G32B32A32_UINT;
                default: break;
            }
            break;
        case VertexAttributeType::SInt32:
            switch (count) {
                case 1: return VK_FORMAT_R32_SINT;
                case 2: return VK_FORMAT_R32G32_SINT;
                case 3: return VK_FORMAT_R32G32B32_SINT;
                case 4: return VK_FORMAT_R32G32B32A32_SINT;
                default: break;
            }
            break;
        case VertexAttributeType::Float32:
            switch (count) {
                case 1: return VK_FORMAT_R32_SFLOAT;
                case 2: return VK_FORMAT_R32G32_SFLOAT;
                case 3: return VK_FORMAT_R32G32B32_SFLOAT;
                case 4: return VK_FORMAT_R32G32B32A32_SFLOAT;
                default: break;
            }
            break;
        case VertexAttributeType::UInt64:
            switch (count) {
                case 1: return VK_FORMAT_R64_UINT;
                case 2: return VK_FORMAT_R64G64_UINT;
                case 3: return VK_FORMAT_R64G64B64_UINT;
                case 4: return VK_FORMAT_R64G64B64A64_UINT;
                default: break;
            }
            break;
        case VertexAttributeType::SInt64:
            switch (count) {
                case 1: return VK_FORMAT_R64_SINT;
                case 2: return VK_FORMAT_R64G64_SINT;
                case 3: return VK_FORMAT_R64G64B64_SINT;
                case 4: return VK_FORMAT_R64G64B64A64_SINT;
                default: break;
            }
            break;
        case VertexAttributeType::Double64:
            switch (count) {
                case 1: return VK_FORMAT_R64_SFLOAT;
                case 2: return VK_FORMAT_R64G64_SFLOAT;
                case 3: return VK_FORMAT_R64G64B64_SFLOAT;
                case 4: return VK_FORMAT_R64G64B64A64_SFLOAT;
                default: break;
            }
            break;
        default:
            break;
    }

    return VK_FORMAT_MAX_ENUM;
}

inline VkImageLayout textureLayoutToVkImageLayout(TextureLayout layout) {
    switch (layout) {
        case TextureLayout::Undefined:          return VK_IMAGE_LAYOUT_UNDEFINED;
        case TextureLayout::RenderTarget:       return VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
        case TextureLayout::DepthStencilTarget: return VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL;
        case TextureLayout::DepthStencilRead:   return VK_IMAGE_LAYOUT_DEPTH_STENCIL_READ_ONLY_OPTIMAL;
        case TextureLayout::ShaderRead:         return VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
        case TextureLayout::TransferSrc:        return VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL;
        case TextureLayout::TransferDst:        return VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
        case TextureLayout::PresentSrc:         return VK_IMAGE_LAYOUT_PRESENT_SRC_KHR;
        default: break;
    }

    return VK_IMAGE_LAYOUT_MAX_ENUM;
}

inline VkImageAspectFlags textureAspectToVkImageAspectFlags(TextureAspect aspect) {
    VkImageAspectFlags flags = 0;
    if ((aspect & TextureAspect::Color) == TextureAspect::Color) {
        flags |= VK_IMAGE_ASPECT_COLOR_BIT;
    }

    if ((aspect & TextureAspect::Depth) == TextureAspect::Depth) {
        flags |= VK_IMAGE_ASPECT_DEPTH_BIT;
    }

    if ((aspect & TextureAspect::Stencil) == TextureAspect::Stencil) {
        flags |= VK_IMAGE_ASPECT_STENCIL_BIT;
    }

    if (flags == 0) {
        return VK_IMAGE_ASPECT_FLAG_BITS_MAX_ENUM;
    }

    return flags;
}

inline VkFormat textureFormatToVkFormat(TextureFormat format) {
    switch (format) {
        case TextureFormat::Undefined:          return VK_FORMAT_UNDEFINED;
        case TextureFormat::R8_UInt:            return VK_FORMAT_R8_UINT;
        case TextureFormat::R8_SInt:            return VK_FORMAT_R8_SINT;
        case TextureFormat::R8_UNorm:           return VK_FORMAT_R8_UNORM;
        case TextureFormat::R8_SNorm:           return VK_FORMAT_R8_SNORM;
        case TextureFormat::R8_sRGB:            return VK_FORMAT_R8_SRGB;
        case TextureFormat::RG8_UInt:           return VK_FORMAT_R8G8_UINT;
        case TextureFormat::RG8_SInt:           return VK_FORMAT_R8G8_SINT;
        case TextureFormat::RG8_UNorm:          return VK_FORMAT_R8G8_UNORM;
        case TextureFormat::RG8_SNorm:          return VK_FORMAT_R8G8_SNORM;
        case TextureFormat::RG8_sRGB:           return VK_FORMAT_R8G8_SRGB;
        case TextureFormat::RGB8_UInt:          return VK_FORMAT_R8G8B8_UINT;
        case TextureFormat::RGB8_SInt:          return VK_FORMAT_R8G8B8_SINT;
        case TextureFormat::RGB8_UNorm:         return VK_FORMAT_R8G8B8_UNORM;
        case TextureFormat::RGB8_SNorm:         return VK_FORMAT_R8G8B8_SNORM;
        case TextureFormat::RGB8_sRGB:          return VK_FORMAT_R8G8B8_SRGB;
        case TextureFormat::BGR8_UInt:          return VK_FORMAT_B8G8R8_UINT;
        case TextureFormat::BGR8_SInt:          return VK_FORMAT_B8G8R8_SINT;
        case TextureFormat::BGR8_UNorm:         return VK_FORMAT_B8G8R8_UNORM;
        case TextureFormat::BGR8_SNorm:         return VK_FORMAT_B8G8R8_SNORM;
        case TextureFormat::BGR8_sRGB:          return VK_FORMAT_B8G8R8_SRGB;
        case TextureFormat::RGBA8_UInt:         return VK_FORMAT_R8G8B8A8_UINT;
        case TextureFormat::RGBA8_SInt:         return VK_FORMAT_R8G8B8A8_SINT;
        case TextureFormat::RGBA8_UNorm:        return VK_FORMAT_R8G8B8A8_UNORM;
        case TextureFormat::RGBA8_SNorm:        return VK_FORMAT_R8G8B8A8_SNORM;
        case TextureFormat::RGBA8_sRGB:         return VK_FORMAT_R8G8B8A8_SRGB;
        case TextureFormat::BGRA8_UInt:         return VK_FORMAT_B8G8R8A8_UINT;
        case TextureFormat::BGRA8_SInt:         return VK_FORMAT_B8G8R8A8_SINT;
        case TextureFormat::BGRA8_UNorm:        return VK_FORMAT_B8G8R8A8_UNORM;
        case TextureFormat::BGRA8_SNorm:        return VK_FORMAT_B8G8R8A8_SNORM;
        case TextureFormat::BGRA8_sRGB:         return VK_FORMAT_B8G8R8A8_SRGB;
        case TextureFormat::R16_UInt:           return VK_FORMAT_R16_UINT;
        case TextureFormat::R16_SInt:           return VK_FORMAT_R16_SINT;
        case TextureFormat::R16_UNorm:          return VK_FORMAT_R16_UNORM;
        case TextureFormat::R16_SNorm:          return VK_FORMAT_R16_SNORM;
        case TextureFormat::R16_Half:           return VK_FORMAT_R16_SFLOAT;
        case TextureFormat::RG16_UInt:          return VK_FORMAT_R16G16_UINT;
        case TextureFormat::RG16_SInt:          return VK_FORMAT_R16G16_SINT;
        case TextureFormat::RG16_UNorm:         return VK_FORMAT_R16G16_UNORM;
        case TextureFormat::RG16_SNorm:         return VK_FORMAT_R16G16_SNORM;
        case TextureFormat::RG16_Half:          return VK_FORMAT_R16G16_SFLOAT;
        case TextureFormat::RGB16_UInt:         return VK_FORMAT_R16G16B16_UINT;
        case TextureFormat::RGB16_SInt:         return VK_FORMAT_R16G16B16_SINT;
        case TextureFormat::RGB16_UNorm:        return VK_FORMAT_R16G16B16_UNORM;
        case TextureFormat::RGB16_SNorm:        return VK_FORMAT_R16G16B16_SNORM;
        case TextureFormat::RGB16_Half:         return VK_FORMAT_R16G16B16_SFLOAT;
        case TextureFormat::RGBA16_UInt:        return VK_FORMAT_R16G16B16A16_UINT;
        case TextureFormat::RGBA16_SInt:        return VK_FORMAT_R16G16B16A16_SINT;
        case TextureFormat::RGBA16_UNorm:       return VK_FORMAT_R16G16B16A16_UNORM;
        case TextureFormat::RGBA16_SNorm:       return VK_FORMAT_R16G16B16A16_SNORM;
        case TextureFormat::RGBA16_Half:        return VK_FORMAT_R16G16B16A16_SFLOAT;
        case TextureFormat::R32_UInt:           return VK_FORMAT_R32_UINT;
        case TextureFormat::R32_SInt:           return VK_FORMAT_R32_SINT;
        case TextureFormat::R32_Float:          return VK_FORMAT_R32_SFLOAT;
        case TextureFormat::RG32_UInt:          return VK_FORMAT_R32G32_UINT;
        case TextureFormat::RG32_SInt:          return VK_FORMAT_R32G32_SINT;
        case TextureFormat::RG32_Float:         return VK_FORMAT_R32G32_SFLOAT;
        case TextureFormat::RGB32_UInt:         return VK_FORMAT_R32G32B32_UINT;
        case TextureFormat::RGB32_SInt:         return VK_FORMAT_R32G32B32_SINT;
        case TextureFormat::RGB32_Float:        return VK_FORMAT_R32G32B32_SFLOAT;
        case TextureFormat::RGBA32_UInt:        return VK_FORMAT_R32G32B32A32_UINT;
        case TextureFormat::RGBA32_SInt:        return VK_FORMAT_R32G32B32A32_SINT;
        case TextureFormat::RGBA32_Float:       return VK_FORMAT_R32G32B32A32_SFLOAT;
        case TextureFormat::R64_UInt:           return VK_FORMAT_R64_UINT;
        case TextureFormat::R64_SInt:           return VK_FORMAT_R64_SINT;
        case TextureFormat::R64_Float:          return VK_FORMAT_R64_SFLOAT;
        case TextureFormat::RG64_UInt:          return VK_FORMAT_R64G64_UINT;
        case TextureFormat::RG64_SInt:          return VK_FORMAT_R64G64_SINT;
        case TextureFormat::RG64_Float:         return VK_FORMAT_R64G64_SFLOAT;
        case TextureFormat::RGB64_UInt:         return VK_FORMAT_R64G64B64_UINT;
        case TextureFormat::RGB64_SInt:         return VK_FORMAT_R64G64B64_SINT;
        case TextureFormat::RGB64_Float:        return VK_FORMAT_R64G64B64_SFLOAT;
        case TextureFormat::RGBA64_UInt:        return VK_FORMAT_R64G64B64A64_UINT;
        case TextureFormat::RGBA64_SInt:        return VK_FORMAT_R64G64B64A64_SINT;
        case TextureFormat::RGBA64_Float:       return VK_FORMAT_R64G64B64A64_SFLOAT;
        case TextureFormat::D16_UNorm:          return VK_FORMAT_D16_UNORM;
        case TextureFormat::D32_Float:          return VK_FORMAT_D32_SFLOAT;
        case TextureFormat::S8_UInt:            return VK_FORMAT_S8_UINT;
        case TextureFormat::D16_UNorm_S8_UInt:  return VK_FORMAT_D16_UNORM_S8_UINT;
        case TextureFormat::D24_UNorm_S8_UInt:  return VK_FORMAT_D24_UNORM_S8_UINT;
        case TextureFormat::D32_Float_S8_UInt:  return VK_FORMAT_D32_SFLOAT_S8_UINT;
        default: break;
    }

    return VK_FORMAT_MAX_ENUM;
}

inline TextureFormat textureFormatFromVkFormat(VkFormat format) {
    switch (format) {
        case VK_FORMAT_UNDEFINED:           return TextureFormat::Undefined;
        case VK_FORMAT_R8_UINT:             return TextureFormat::R8_UInt;
        case VK_FORMAT_R8_SINT:             return TextureFormat::R8_SInt;
        case VK_FORMAT_R8_UNORM:            return TextureFormat::R8_UNorm;
        case VK_FORMAT_R8_SNORM:            return TextureFormat::R8_SNorm;
        case VK_FORMAT_R8_SRGB:             return TextureFormat::R8_sRGB;
        case VK_FORMAT_R8G8_UINT:           return TextureFormat::RG8_UInt;
        case VK_FORMAT_R8G8_SINT:           return TextureFormat::RG8_SInt;
        case VK_FORMAT_R8G8_UNORM:          return TextureFormat::RG8_UNorm;
        case VK_FORMAT_R8G8_SNORM:          return TextureFormat::RG8_SNorm;
        case VK_FORMAT_R8G8_SRGB:           return TextureFormat::RG8_sRGB;
        case VK_FORMAT_R8G8B8_UINT:         return TextureFormat::RGB8_UInt;
        case VK_FORMAT_R8G8B8_SINT:         return TextureFormat::RGB8_SInt;
        case VK_FORMAT_R8G8B8_UNORM:        return TextureFormat::RGB8_UNorm;
        case VK_FORMAT_R8G8B8_SNORM:        return TextureFormat::RGB8_SNorm;
        case VK_FORMAT_R8G8B8_SRGB:         return TextureFormat::RGB8_sRGB;
        case VK_FORMAT_B8G8R8_UINT:         return TextureFormat::BGR8_UInt;
        case VK_FORMAT_B8G8R8_SINT:         return TextureFormat::BGR8_SInt;
        case VK_FORMAT_B8G8R8_UNORM:        return TextureFormat::BGR8_UNorm;
        case VK_FORMAT_B8G8R8_SNORM:        return TextureFormat::BGR8_SNorm;
        case VK_FORMAT_B8G8R8_SRGB:         return TextureFormat::BGR8_sRGB;
        case VK_FORMAT_R8G8B8A8_UINT:       return TextureFormat::RGBA8_UInt;
        case VK_FORMAT_R8G8B8A8_SINT:       return TextureFormat::RGBA8_SInt;
        case VK_FORMAT_R8G8B8A8_UNORM:      return TextureFormat::RGBA8_UNorm;
        case VK_FORMAT_R8G8B8A8_SNORM:      return TextureFormat::RGBA8_SNorm;
        case VK_FORMAT_R8G8B8A8_SRGB:       return TextureFormat::RGBA8_sRGB;
        case VK_FORMAT_B8G8R8A8_UINT:       return TextureFormat::BGRA8_UInt;
        case VK_FORMAT_B8G8R8A8_SINT:       return TextureFormat::BGRA8_SInt;
        case VK_FORMAT_B8G8R8A8_UNORM:      return TextureFormat::BGRA8_UNorm;
        case VK_FORMAT_B8G8R8A8_SNORM:      return TextureFormat::BGRA8_SNorm;
        case VK_FORMAT_B8G8R8A8_SRGB:       return TextureFormat::BGRA8_sRGB;
        case VK_FORMAT_R16_UINT:            return TextureFormat::R16_UInt;
        case VK_FORMAT_R16_SINT:            return TextureFormat::R16_SInt;
        case VK_FORMAT_R16_UNORM:           return TextureFormat::R16_UNorm;
        case VK_FORMAT_R16_SNORM:           return TextureFormat::R16_SNorm;
        case VK_FORMAT_R16_SFLOAT:          return TextureFormat::R16_Half;
        case VK_FORMAT_R16G16_UINT:         return TextureFormat::RG16_UInt;
        case VK_FORMAT_R16G16_SINT:         return TextureFormat::RG16_SInt;
        case VK_FORMAT_R16G16_UNORM:        return TextureFormat::RG16_UNorm;
        case VK_FORMAT_R16G16_SNORM:        return TextureFormat::RG16_SNorm;
        case VK_FORMAT_R16G16_SFLOAT:       return TextureFormat::RG16_Half;
        case VK_FORMAT_R16G16B16_UINT:      return TextureFormat::RGB16_UInt;
        case VK_FORMAT_R16G16B16_SINT:      return TextureFormat::RGB16_SInt;
        case VK_FORMAT_R16G16B16_UNORM:     return TextureFormat::RGB16_UNorm;
        case VK_FORMAT_R16G16B16_SNORM:     return TextureFormat::RGB16_SNorm;
        case VK_FORMAT_R16G16B16_SFLOAT:    return TextureFormat::RGB16_Half;
        case VK_FORMAT_R16G16B16A16_UINT:   return TextureFormat::RGBA16_UInt;
        case VK_FORMAT_R16G16B16A16_SINT:   return TextureFormat::RGBA16_SInt;
        case VK_FORMAT_R16G16B16A16_UNORM:  return TextureFormat::RGBA16_UNorm;
        case VK_FORMAT_R16G16B16A16_SNORM:  return TextureFormat::RGBA16_SNorm;
        case VK_FORMAT_R16G16B16A16_SFLOAT: return TextureFormat::RGBA16_Half;
        case VK_FORMAT_R32_UINT:            return TextureFormat::R32_UInt;
        case VK_FORMAT_R32_SINT:            return TextureFormat::R32_SInt;
        case VK_FORMAT_R32_SFLOAT:          return TextureFormat::R32_Float;
        case VK_FORMAT_R32G32_UINT:         return TextureFormat::RG32_UInt;
        case VK_FORMAT_R32G32_SINT:         return TextureFormat::RG32_SInt;
        case VK_FORMAT_R32G32_SFLOAT:       return TextureFormat::RG32_Float;
        case VK_FORMAT_R32G32B32_UINT:      return TextureFormat::RGB32_UInt;
        case VK_FORMAT_R32G32B32_SINT:      return TextureFormat::RGB32_SInt;
        case VK_FORMAT_R32G32B32_SFLOAT:    return TextureFormat::RGB32_Float;
        case VK_FORMAT_R32G32B32A32_UINT:   return TextureFormat::RGBA32_UInt;
        case VK_FORMAT_R32G32B32A32_SINT:   return TextureFormat::RGBA32_SInt;
        case VK_FORMAT_R32G32B32A32_SFLOAT: return TextureFormat::RGBA32_Float;
        case VK_FORMAT_R64_UINT:            return TextureFormat::R64_UInt;
        case VK_FORMAT_R64_SINT:            return TextureFormat::R64_SInt;
        case VK_FORMAT_R64_SFLOAT:          return TextureFormat::R64_Float;
        case VK_FORMAT_R64G64_UINT:         return TextureFormat::RG64_UInt;
        case VK_FORMAT_R64G64_SINT:         return TextureFormat::RG64_SInt;
        case VK_FORMAT_R64G64_SFLOAT:       return TextureFormat::RG64_Float;
        case VK_FORMAT_R64G64B64_UINT:      return TextureFormat::RGB64_UInt;
        case VK_FORMAT_R64G64B64_SINT:      return TextureFormat::RGB64_SInt;
        case VK_FORMAT_R64G64B64_SFLOAT:    return TextureFormat::RGB64_Float;
        case VK_FORMAT_R64G64B64A64_UINT:   return TextureFormat::RGBA64_UInt;
        case VK_FORMAT_R64G64B64A64_SINT:   return TextureFormat::RGBA64_SInt;
        case VK_FORMAT_R64G64B64A64_SFLOAT: return TextureFormat::RGBA64_Float;
        case VK_FORMAT_D16_UNORM:           return TextureFormat::D16_UNorm;
        case VK_FORMAT_D32_SFLOAT:          return TextureFormat::D32_Float;
        case VK_FORMAT_S8_UINT:             return TextureFormat::S8_UInt;
        case VK_FORMAT_D16_UNORM_S8_UINT:   return TextureFormat::D16_UNorm_S8_UInt;
        case VK_FORMAT_D24_UNORM_S8_UINT:   return TextureFormat::D24_UNorm_S8_UInt;
        case VK_FORMAT_D32_SFLOAT_S8_UINT:  return TextureFormat::D32_Float_S8_UInt;
        default: break;
    }

    return static_cast<TextureFormat>(-1);
}

inline VkImageUsageFlags textureUsageToVkImageUsageFlags(TextureUsage usage) {
    VkImageUsageFlags flags = 0;
    if ((usage & TextureUsage::SampledTexture) == TextureUsage::SampledTexture) {
        flags |= VK_IMAGE_USAGE_SAMPLED_BIT;
    }

    if ((usage & TextureUsage::StorageTexture) == TextureUsage::StorageTexture) {
        flags |= VK_IMAGE_USAGE_STORAGE_BIT;
    }

    if ((usage & TextureUsage::RenderTarget) == TextureUsage::RenderTarget) {
        flags |= VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT;
    }

    if ((usage & TextureUsage::DepthTarget) == TextureUsage::DepthTarget) {
        flags |= VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT;
    }

    if ((usage & TextureUsage::StencilTarget) == TextureUsage::StencilTarget) {
        flags |= VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT;
    }

    return flags;
}

inline VkImageType textureDimensionsToVkImageType(TextureDimensions dimensions) {
    switch (dimensions) {
        case TextureDimensions::Texture1D:  return VK_IMAGE_TYPE_1D;
        case TextureDimensions::Texture2D:  return VK_IMAGE_TYPE_2D;
        case TextureDimensions::Texture3D:  return VK_IMAGE_TYPE_3D;
        case TextureDimensions::Array1D:  return VK_IMAGE_TYPE_1D;
        case TextureDimensions::Array2D:  return VK_IMAGE_TYPE_2D;
        default: break;
    }

    return VK_IMAGE_TYPE_MAX_ENUM;
}

inline VkImageViewType textureDimensionsToVkImageViewType(TextureDimensions dimensions) {
    switch (dimensions) {
        case TextureDimensions::Texture1D:  return VK_IMAGE_VIEW_TYPE_1D;
        case TextureDimensions::Texture2D:  return VK_IMAGE_VIEW_TYPE_2D;
        case TextureDimensions::Texture3D:  return VK_IMAGE_VIEW_TYPE_3D;
        case TextureDimensions::Array1D:    return VK_IMAGE_VIEW_TYPE_1D_ARRAY;
        case TextureDimensions::Array2D:    return VK_IMAGE_VIEW_TYPE_2D_ARRAY;
        default: break;
    }

    return VK_IMAGE_VIEW_TYPE_MAX_ENUM;
}

inline VkComponentSwizzle componentSwizzleToVkComponentSwizzle(ComponentSwizzle swizzle) {
    switch (swizzle) {
        case ComponentSwizzle::Identity:    return VK_COMPONENT_SWIZZLE_IDENTITY;
        case ComponentSwizzle::Zero:        return VK_COMPONENT_SWIZZLE_ZERO;
        case ComponentSwizzle::One:         return VK_COMPONENT_SWIZZLE_ONE;
        case ComponentSwizzle::R:           return VK_COMPONENT_SWIZZLE_R;
        case ComponentSwizzle::G:           return VK_COMPONENT_SWIZZLE_G;
        case ComponentSwizzle::B:           return VK_COMPONENT_SWIZZLE_B;
        case ComponentSwizzle::A:           return VK_COMPONENT_SWIZZLE_A;
        default: break;
    }

    return VK_COMPONENT_SWIZZLE_MAX_ENUM;
}

inline kobalt::wsi::PresentMode presentModeFromVkPresentModeKHR(VkPresentModeKHR mode) {
    switch (mode) {
        case VK_PRESENT_MODE_IMMEDIATE_KHR:     return kobalt::wsi::PresentMode::Immediate;
        case VK_PRESENT_MODE_MAILBOX_KHR:       return kobalt::wsi::PresentMode::Mailbox;

        case VK_PRESENT_MODE_FIFO_KHR:
        case VK_PRESENT_MODE_FIFO_RELAXED_KHR:  return kobalt::wsi::PresentMode::VSync;
        default: break;
    }

    return static_cast<kobalt::wsi::PresentMode>(-1);
}

inline VkAttachmentLoadOp renderAttachmentLoadOpToVkAttachmentLoadOp(RenderAttachmentLoadOp op) {
    switch (op) {
        case RenderAttachmentLoadOp::DontCare:  return VK_ATTACHMENT_LOAD_OP_DONT_CARE;
        case RenderAttachmentLoadOp::Clear:     return VK_ATTACHMENT_LOAD_OP_CLEAR;
        case RenderAttachmentLoadOp::Load:      return VK_ATTACHMENT_LOAD_OP_LOAD;
        default: break;
    }

    return VK_ATTACHMENT_LOAD_OP_MAX_ENUM;
}

inline VkAttachmentStoreOp renderAttachmentStoreOpToVkAttachmentStoreOp(RenderAttachmentStoreOp op) {
    switch (op) {
        case RenderAttachmentStoreOp::DontCare:  return VK_ATTACHMENT_STORE_OP_DONT_CARE;
        case RenderAttachmentStoreOp::Store:     return VK_ATTACHMENT_STORE_OP_STORE;
        default: break;
    }

    return VK_ATTACHMENT_STORE_OP_MAX_ENUM;
}

inline VkPipelineStageFlags pipelineStageToVkPipelineStageFlags(PipelineStage stage) {
    VkPipelineStageFlags flags = 0;
    if ((stage & PipelineStage::Top) == PipelineStage::Top) {
        flags |= VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT;
    }

    if ((stage & PipelineStage::DrawIndirect) == PipelineStage::DrawIndirect) {
        flags |= VK_PIPELINE_STAGE_DRAW_INDIRECT_BIT;
    }

    if ((stage & PipelineStage::VertexInput) == PipelineStage::VertexInput) {
        flags |= VK_PIPELINE_STAGE_VERTEX_INPUT_BIT;
    }

    if ((stage & PipelineStage::VertexShader) == PipelineStage::VertexShader) {
        flags |= VK_PIPELINE_STAGE_VERTEX_SHADER_BIT;
    }

    if ((stage & PipelineStage::TessellationControlShader) == PipelineStage::TessellationControlShader) {
        flags |= VK_PIPELINE_STAGE_TESSELLATION_CONTROL_SHADER_BIT;
    }

    if ((stage & PipelineStage::TessellationEvaluationShader) == PipelineStage::TessellationEvaluationShader) {
        flags |= VK_PIPELINE_STAGE_TESSELLATION_EVALUATION_SHADER_BIT;
    }

    if ((stage & PipelineStage::GeometryShader) == PipelineStage::GeometryShader) {
        flags |= VK_PIPELINE_STAGE_GEOMETRY_SHADER_BIT;
    }

    if ((stage & PipelineStage::PreFragmentTests) == PipelineStage::PreFragmentTests) {
        flags |= VK_PIPELINE_STAGE_EARLY_FRAGMENT_TESTS_BIT;
    }

    if ((stage & PipelineStage::FragmentShader) == PipelineStage::FragmentShader) {
        flags |= VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT;
    }

    if ((stage & PipelineStage::PostFragmentTests) == PipelineStage::PostFragmentTests) {
        flags |= VK_PIPELINE_STAGE_LATE_FRAGMENT_TESTS_BIT;
    }

    if ((stage & PipelineStage::RenderTarget) == PipelineStage::RenderTarget) {
        flags |= VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
    }

    if ((stage & PipelineStage::ComputeShader) == PipelineStage::ComputeShader) {
        flags |= VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT;
    }

    if ((stage & PipelineStage::Transfer) == PipelineStage::Transfer) {
        flags |= VK_PIPELINE_STAGE_TRANSFER_BIT;
    }

    if ((stage & PipelineStage::Bottom) == PipelineStage::Bottom) {
        flags |= VK_PIPELINE_STAGE_BOTTOM_OF_PIPE_BIT;
    }

    if (flags == 0) {
        return VK_PIPELINE_STAGE_FLAG_BITS_MAX_ENUM;
    }

    return flags;
}

inline VkAccessFlags resourceAccessToVkAccessFlags(ResourceAccess access) {
    VkAccessFlags flags = 0;
    if ((access & ResourceAccess::IndirectRead) == ResourceAccess::IndirectRead) {
        flags |= VK_ACCESS_INDIRECT_COMMAND_READ_BIT;
    }

    if ((access & ResourceAccess::IndexRead) == ResourceAccess::IndexRead) {
        flags |= VK_ACCESS_INDEX_READ_BIT;
    }

    if ((access & ResourceAccess::VertexAttributeRead) == ResourceAccess::VertexAttributeRead) {
        flags |= VK_ACCESS_VERTEX_ATTRIBUTE_READ_BIT;
    }

    if ((access & ResourceAccess::UniformRead) == ResourceAccess::UniformRead) {
        flags |= VK_ACCESS_UNIFORM_READ_BIT;
    }

    if ((access & ResourceAccess::InputAttachmentRead) == ResourceAccess::InputAttachmentRead) {
        flags |= VK_ACCESS_INPUT_ATTACHMENT_READ_BIT;
    }

    if ((access & ResourceAccess::ShaderRead) == ResourceAccess::ShaderRead) {
        flags |= VK_ACCESS_SHADER_READ_BIT;
    }

    if ((access & ResourceAccess::ShaderWrite) == ResourceAccess::ShaderWrite) {
        flags |= VK_ACCESS_SHADER_WRITE_BIT;
    }

    if ((access & ResourceAccess::RenderTargetRead) == ResourceAccess::RenderTargetRead) {
        flags |= VK_ACCESS_COLOR_ATTACHMENT_READ_BIT;
    }

    if ((access & ResourceAccess::RenderTargetWrite) == ResourceAccess::RenderTargetWrite) {
        flags |= VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;
    }

    if ((access & ResourceAccess::DepthStencilTargetRead) == ResourceAccess::DepthStencilTargetRead) {
        flags |= VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_READ_BIT;
    }

    if ((access & ResourceAccess::DepthStencilTargetWrite) == ResourceAccess::DepthStencilTargetWrite) {
        flags |= VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT;
    }

    if ((access & ResourceAccess::TransferRead) == ResourceAccess::TransferRead) {
        flags |= VK_ACCESS_TRANSFER_READ_BIT;
    }

    if ((access & ResourceAccess::TransferWrite) == ResourceAccess::TransferWrite) {
        flags |= VK_ACCESS_TRANSFER_WRITE_BIT;
    }

    if ((access & ResourceAccess::HostRead) == ResourceAccess::HostRead) {
        flags |= VK_ACCESS_HOST_READ_BIT;
    }

    if ((access & ResourceAccess::HostWrite) == ResourceAccess::HostWrite) {
        flags |= VK_ACCESS_HOST_WRITE_BIT;
    }

    if ((access & ResourceAccess::MemoryRead) == ResourceAccess::MemoryRead) {
        flags |= VK_ACCESS_MEMORY_READ_BIT;
    }

    if ((access & ResourceAccess::MemoryWrite) == ResourceAccess::MemoryWrite) {
        flags |= VK_ACCESS_MEMORY_WRITE_BIT;
    }

    return flags;
}

inline TextureAspect maximumTextureAspectFromTextureFormat(TextureFormat format) {
    switch (format) {
        case TextureFormat::R8_UInt:        case TextureFormat::R8_SInt:        case TextureFormat::R8_UNorm:       case TextureFormat::R8_SNorm:       case TextureFormat::R8_sRGB:
        case TextureFormat::RG8_UInt:       case TextureFormat::RG8_SInt:       case TextureFormat::RG8_UNorm:      case TextureFormat::RG8_SNorm:      case TextureFormat::RG8_sRGB:
        case TextureFormat::RGB8_UInt:      case TextureFormat::RGB8_SInt:      case TextureFormat::RGB8_UNorm:     case TextureFormat::RGB8_SNorm:     case TextureFormat::RGB8_sRGB:
        case TextureFormat::BGR8_UInt:      case TextureFormat::BGR8_SInt:      case TextureFormat::BGR8_UNorm:     case TextureFormat::BGR8_SNorm:     case TextureFormat::BGR8_sRGB:
        case TextureFormat::RGBA8_UInt:     case TextureFormat::RGBA8_SInt:     case TextureFormat::RGBA8_UNorm:    case TextureFormat::RGBA8_SNorm:    case TextureFormat::RGBA8_sRGB:
        case TextureFormat::BGRA8_UInt:     case TextureFormat::BGRA8_SInt:     case TextureFormat::BGRA8_UNorm:    case TextureFormat::BGRA8_SNorm:    case TextureFormat::BGRA8_sRGB:
        case TextureFormat::R16_UInt:       case TextureFormat::R16_SInt:       case TextureFormat::R16_UNorm:      case TextureFormat::R16_SNorm:      case TextureFormat::R16_Half:
        case TextureFormat::RG16_UInt:      case TextureFormat::RG16_SInt:      case TextureFormat::RG16_UNorm:     case TextureFormat::RG16_SNorm:     case TextureFormat::RG16_Half:
        case TextureFormat::RGB16_UInt:     case TextureFormat::RGB16_SInt:     case TextureFormat::RGB16_UNorm:    case TextureFormat::RGB16_SNorm:    case TextureFormat::RGB16_Half:
        case TextureFormat::RGBA16_UInt:    case TextureFormat::RGBA16_SInt:    case TextureFormat::RGBA16_UNorm:   case TextureFormat::RGBA16_SNorm:   case TextureFormat::RGBA16_Half:
        case TextureFormat::R32_UInt:       case TextureFormat::R32_SInt:       case TextureFormat::R32_Float:
        case TextureFormat::RG32_UInt:      case TextureFormat::RG32_SInt:      case TextureFormat::RG32_Float:
        case TextureFormat::RGB32_UInt:     case TextureFormat::RGB32_SInt:     case TextureFormat::RGB32_Float:
        case TextureFormat::RGBA32_UInt:    case TextureFormat::RGBA32_SInt:    case TextureFormat::RGBA32_Float:
        case TextureFormat::R64_UInt:       case TextureFormat::R64_SInt:       case TextureFormat::R64_Float:
        case TextureFormat::RG64_UInt:      case TextureFormat::RG64_SInt:      case TextureFormat::RG64_Float:
        case TextureFormat::RGB64_UInt:     case TextureFormat::RGB64_SInt:     case TextureFormat::RGB64_Float:
        case TextureFormat::RGBA64_UInt:    case TextureFormat::RGBA64_SInt:    case TextureFormat::RGBA64_Float:
            return TextureAspect::Color;
        case TextureFormat::D16_UNorm:
        case TextureFormat::D32_Float:
            return TextureAspect::Depth;
        case TextureFormat::S8_UInt:
            return TextureAspect::Stencil;
        case TextureFormat::D16_UNorm_S8_UInt:
        case TextureFormat::D24_UNorm_S8_UInt:
        case TextureFormat::D32_Float_S8_UInt:
            return TextureAspect::Depth | TextureAspect::Stencil;
        default: break;
    }

    return static_cast<TextureAspect>(0);
}

enum class SurfaceType {
    GLFW
};

struct Surface {
    VkSurfaceKHR vkSurface;
    VkSurfaceCapabilitiesKHR vkCapabilities;
    SurfaceType type;
    void* window;

    uint32_t refCount = 0;
};

struct InstanceSymbols {
    PFN_vkSetDebugUtilsObjectNameEXT vkSetDebugUtilsObjectNameEXT;
};

static VkInstance vkInstance = nullptr;
static std::vector<VkPhysicalDevice> vkPhysicalDevices;
static std::vector<Surface> surfaces;

static VkDebugUtilsMessengerEXT vkDebugMessengerEXT = nullptr;
static InstanceSymbols instanceSymbols = {};

struct EnabledFeaturesX {
    VkStructureType sType;
    EnabledFeaturesX* pNext;
};

struct ObjectBase_t {
    Object_t obj;
    char debugName[64];
    uint64_t vkHandle = 0;

    ObjectBase_t(Object_t* device, ObjectType type, void* handle) {
        obj.device = device;
        obj.type = type;
        memset(debugName, 0, 64);

        vkHandle = reinterpret_cast<uint64_t>(handle);
    }
};

struct ExtraDeviceFeatures {

};

struct DeviceQueues {
    VkQueue graphics;
    uint32_t graphicsFamily;
    QueueType graphicsTypes;

    VkQueue compute;
    uint32_t computeFamily;
    QueueType computeTypes;

    VkQueue transfer;
    uint32_t transferFamily;
    QueueType transferTypes;

    VkQueue general;
    uint32_t generalFamily;
    QueueType generalTypes;

    DeviceQueues(uint32_t gf, QueueType gt, uint32_t cf, QueueType ct, uint32_t tf, QueueType tt, uint32_t af, QueueType at) : graphicsFamily(gf), graphicsTypes(gt), computeFamily(cf), computeTypes(ct), transferFamily(tf), transferTypes(tt), generalFamily(af), generalTypes(at) {}
};

struct DeviceSymbols {
    PFN_vkCmdBeginRenderingKHR vkCmdBeginRenderingKHR;
    PFN_vkCmdEndRenderingKHR vkCmdEndRenderingKHR;
};

struct Device_t {
    ObjectBase_t base;

    VkDevice vkDevice;
    VkPhysicalDevice vkPhysicalDevice;
    
    DeviceSupport enabledSupport;
    ExtraDeviceFeatures extraFeatures;

    DeviceSymbols symbols;
    DeviceQueues queues;

    Device_t(VkDevice vkDev, VkPhysicalDevice vkPhys, DeviceSupport const& support, ExtraDeviceFeatures const& extra, DeviceQueues const& qs) : base(nullptr, ObjectType::Device, vkDev), vkDevice(vkDev), vkPhysicalDevice(vkPhys), enabledSupport(support), extraFeatures(extra), queues(qs) {
        vkGetDeviceQueue(vkDev, queues.graphicsFamily, 0, &queues.graphics);
        vkGetDeviceQueue(vkDev, queues.computeFamily, 0, &queues.compute);
        vkGetDeviceQueue(vkDev, queues.transferFamily, 0, &queues.transfer);
        vkGetDeviceQueue(vkDev, queues.generalFamily, 0, &queues.general);

        symbols.vkCmdBeginRenderingKHR = reinterpret_cast<PFN_vkCmdBeginRenderingKHR>(vkGetDeviceProcAddr(vkDev, "vkCmdBeginRenderingKHR"));
        symbols.vkCmdEndRenderingKHR = reinterpret_cast<PFN_vkCmdEndRenderingKHR>(vkGetDeviceProcAddr(vkDev, "vkCmdEndRenderingKHR"));
    }

    ~Device_t() {
        vkDeviceWaitIdle(vkDevice);
        vkDestroyDevice(vkDevice, nullptr);
    }
};

struct Texture_t {
    ObjectBase_t base;

    Device_t* device;
    VkImage vkImage = nullptr;

    TextureFormat format;
    TextureUsage usage;
    TextureDimensions dimensions;
    MemoryLocation location;
    
    uint32_t width;
    uint32_t height;
    uint32_t depth;
    uint32_t layerCount;
    uint32_t samples;
    uint32_t mipCount;

    kobalt::wsi::Swapchain swapchain = nullptr;

    Texture_t(Device_t* device, VkImage vkImage, TextureFormat format, TextureUsage usage, TextureDimensions dimensions, MemoryLocation location, uint32_t width, uint32_t height, uint32_t depth, uint32_t layerCount, uint32_t samples, uint32_t mipCount) : base(&device->base.obj, internal::ObjectType::Texture, vkImage), device(device), vkImage(vkImage), format(format), usage(usage), dimensions(dimensions), location(location), width(width), height(height), depth(depth), layerCount(layerCount), samples(samples), mipCount(mipCount) {}
    Texture_t(Device_t* device, kobalt::wsi::Swapchain swapchain, TextureFormat format, TextureUsage usage, TextureDimensions dimensions, MemoryLocation location, uint32_t width, uint32_t height, uint32_t depth, uint32_t layerCount, uint32_t samples, uint32_t mipCount) : base(&device->base.obj, internal::ObjectType::Unknown, nullptr), device(device), swapchain(swapchain), format(format), usage(usage), dimensions(dimensions), location(location), width(width), height(height), depth(depth), layerCount(layerCount), samples(samples), mipCount(mipCount) {}

    ~Texture_t() {
        if (swapchain == nullptr) {
            vkDeviceWaitIdle(device->vkDevice);
            vkDestroyImage(device->vkDevice, vkImage, nullptr);
        }
    }
};

struct TextureView_t {
    ObjectBase_t base;
    
    Device_t* device;
    Texture_t* texture;

    VkImage vkImage;
    VkImageView vkImageView;

    TextureFormat format;
    TextureDimensions dimensions;

    kobalt::wsi::Swapchain swapchain = nullptr;
    uint32_t swapchainInternalViewIndex = 0;

    TextureView_t(Device_t* device, Texture_t* texture, VkImageView vkImageView, TextureFormat format, TextureDimensions dimensions) : base(&device->base.obj, ObjectType::TextureView, vkImageView), device(device), texture(texture), vkImage(texture->vkImage), vkImageView(vkImageView), format(format), dimensions(dimensions) {}
    TextureView_t(Device_t* device, kobalt::wsi::Swapchain swapchain, Texture_t* texture, uint32_t swapchainInternalViewIndex, TextureFormat format, TextureDimensions dimensions) : base(&device->base.obj, ObjectType::TextureView, nullptr), device(device), texture(texture), swapchain(swapchain), swapchainInternalViewIndex(swapchainInternalViewIndex), vkImage(nullptr), vkImageView(nullptr), format(format), dimensions(dimensions) {}

    ~TextureView_t() {
        if (swapchain != nullptr) {
            vkDestroyImageView(device->vkDevice, vkImageView, nullptr);
        }
    }
};

struct Buffer_t {
    ObjectBase_t base;

    Device_t* device;
    VkBuffer vkBuffer;
    VkDeviceMemory vkMemory;
    uint64_t size;

    Buffer_t(Device_t* device, VkBuffer vkBuffer, VkDeviceMemory vkMemory) : base(&device->base.obj, internal::ObjectType::Buffer, vkBuffer), device(device), vkBuffer(vkBuffer), vkMemory(vkMemory) {}

    ~Buffer_t() {
        vkDeviceWaitIdle(device->vkDevice);
        vkDestroyBuffer(device->vkDevice, vkBuffer, nullptr);
        vkFreeMemory(device->vkDevice, vkMemory, nullptr);
    }
};

struct Shader_t {
    ObjectBase_t base;

    Device_t* device;
    VkShaderModule vkShaderModule;

    Shader_t(Device_t* device, VkShaderModule vkShaderModule) : base(&device->base.obj, internal::ObjectType::Shader, vkShaderModule), device(device), vkShaderModule(vkShaderModule) {}

    ~Shader_t() {
        vkDeviceWaitIdle(device->vkDevice);
        vkDestroyShaderModule(device->vkDevice, vkShaderModule, nullptr);
    }
};

struct VertexInputState_t {
    ObjectBase_t base;

    std::vector<VkVertexInputAttributeDescription> attributes;
    std::vector<VkVertexInputBindingDescription> bindings;

    VkPipelineVertexInputStateCreateInfo vertexInputCI = {};
    VkPipelineInputAssemblyStateCreateInfo inputAssemblyCI = {};

    VertexInputState_t(Object_t* device, Topology topo, VertexAttribute const* attrs, uint32_t attrCount, VertexBinding const* binds, uint32_t bindCount) : base(device, ObjectType::VertexInputState, nullptr) {
        attributes.resize(attrCount);
        bindings.resize(bindCount);

        for (uint32_t i = 0; i < attrCount; ++i) {
            attributes[i].location = attrs[i].location;
            attributes[i].binding = attrs[i].binding;
            attributes[i].format = internal::vertexAttributeTypeToVkFormat(attrs[i].type, attrs[i].count);
            attributes[i].offset = attrs[i].offset;
        }

        for (uint32_t i = 0; i < bindCount; ++i) {
            bindings[i].binding = binds[i].binding;
            bindings[i].stride = binds[i].size;
            bindings[i].inputRate = binds[i].advancePerInstance ? VK_VERTEX_INPUT_RATE_INSTANCE : VK_VERTEX_INPUT_RATE_VERTEX;
        }

        vertexInputCI.sType = VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO;
        vertexInputCI.vertexBindingDescriptionCount = bindCount;
        vertexInputCI.pVertexBindingDescriptions = bindings.data();
        vertexInputCI.vertexAttributeDescriptionCount = attrCount;
        vertexInputCI.pVertexAttributeDescriptions = attributes.data();

        inputAssemblyCI.sType = VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO;
        inputAssemblyCI.topology = internal::topologyToVkPrimitiveTopology(topo);
    }
};

struct RasterizationState_t {
    ObjectBase_t base;

    VkPipelineRasterizationStateCreateInfo createInfo = {};

    RasterizationState_t(Device device, FillMode fillMode, CullMode cullMode, FrontFace frontFace, float depthBias, float depthBiasClamp, float slopeScaledDepthBias) : base(device, ObjectType::RasterizationState, nullptr) {
        createInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_RASTERIZATION_STATE_RASTERIZATION_ORDER_AMD;
        createInfo.depthClampEnable = depthBiasClamp != 0.0f;
        createInfo.polygonMode = internal::fillModeToVkPolygonMode(fillMode);
        createInfo.cullMode = internal::cullModeToVkCullModeFlags(cullMode);
        createInfo.frontFace = internal::frontFaceToVkFrontFace(frontFace);
        createInfo.depthBiasEnable = depthBias != 0.0f;
        createInfo.depthBiasConstantFactor = 1.0f;
        createInfo.depthBiasClamp = depthBiasClamp;
        createInfo.depthBiasSlopeFactor = slopeScaledDepthBias;
        createInfo.lineWidth = 1.0f;
    }
};

struct QueueSync_t {
    ObjectBase_t base;

    Device_t* device;
    VkSemaphore vkSemaphore;

    QueueSync_t(Device_t* device, VkSemaphore vkSemaphore) : base(&device->base.obj, ObjectType::QueueSync, vkSemaphore), device(device), vkSemaphore(vkSemaphore) {}

    ~QueueSync_t() {
        vkDeviceWaitIdle(device->vkDevice);
        vkDestroySemaphore(device->vkDevice, vkSemaphore, nullptr);
    }
};

struct HostSync_t {
    ObjectBase_t base;

    Device_t* device;
    VkFence vkFence;

    HostSync_t(Device_t* device, VkFence vkFence) : base(&device->base.obj, ObjectType::HostSync, vkFence), device(device), vkFence(vkFence) {}

    ~HostSync_t() {
        vkDeviceWaitIdle(device->vkDevice);
        vkDestroyFence(device->vkDevice, vkFence, nullptr);
    }
};

struct StageSync_t {
    ObjectBase_t base;

    Device_t* device;
    VkEvent vkEvent;

    StageSync_t(Device_t* device, VkEvent vkEvent) : base(&device->base.obj, ObjectType::StageSync, vkEvent), device(device), vkEvent(vkEvent) {}

    ~StageSync_t() {
        vkDeviceWaitIdle(device->vkDevice);
        vkDestroyEvent(device->vkDevice, vkEvent, nullptr);
    }
};

struct RenderPass_t {
    /* TODO: */
};

struct TextureBarrier {
    ResourceAccess srcAccess;
    ResourceAccess dstAccess;
    TextureLayout oldLayout;
    TextureLayout newLayout;
    QueueTransfer srcQueue;
    QueueTransfer dstQueue;
    Texture_t* texture;
    TextureSubresource subresource;
};

struct BufferBarrier {
    ResourceAccess srcAccess;
    ResourceAccess dstAccess;
    QueueTransfer srcQueue;
    QueueTransfer dstQueue;
    Buffer_t* buffer;
    uint64_t offset;
    uint64_t size;
};

struct GlobalResourceBarrier {
    ResourceAccess srcAccess;
    ResourceAccess dstAccess;
};

struct ExecutionBarrier {
    PipelineStage srcStage;
    PipelineStage dstStage;
};

struct PipelineBarrier {
    StageSync_t* stageSync = nullptr;
    ExecutionBarrier exec;

    std::vector<TextureBarrier> textures;
    std::vector<BufferBarrier> buffers;
    std::vector<GlobalResourceBarrier> globals;

    void clear() {
        exec = {};

        textures.clear();
        buffers.clear();
        globals.clear();
    }
};

struct CommandListRenderPass {
    RenderPass_t* renderPass;

    bool dynamic;
    std::vector<DynamicRenderAttachment> renderTargets;
    DynamicRenderAttachment depthTarget;
    DynamicRenderAttachment stencilTarget;

    void clear() {
        renderPass = nullptr;

        dynamic = false;
        renderTargets.clear();
        depthTarget = {};
        stencilTarget = {};
    }
};

struct CommandList_t {
    ObjectBase_t base;

    Device_t* device;
    VkCommandBuffer vkCmdBuffer;
    VkCommandPool vkCmdPool;

    QueueType queueType;
    uint32_t queueFamily;

    bool wasLastCmdBarrier = false;
    PipelineBarrier barrier = {};

    CommandListRenderPass renderPass = {};

    CommandList_t(Device_t* device, VkCommandBuffer vkCmdBuffer, VkCommandPool vkCmdPool, QueueType queueType) : base(&device->base.obj, ObjectType::CommandList, vkCmdBuffer), device(device), vkCmdBuffer(vkCmdBuffer), vkCmdPool(vkCmdPool), queueType(queueType) {
        switch (queueType) {
            case QueueType::Graphics:
                queueFamily = device->queues.graphicsFamily;
                break;
            case QueueType::Compute:
                queueFamily = device->queues.computeFamily;
                break;
            case QueueType::Transfer:
                queueFamily = device->queues.transferFamily;
                break;
            case QueueType::General:
                queueFamily = device->queues.generalFamily;
                break;
        }
    }

    ~CommandList_t() {
        vkDeviceWaitIdle(device->vkDevice);
        vkFreeCommandBuffers(device->vkDevice, vkCmdPool, 1, &vkCmdBuffer);
        vkDestroyCommandPool(device->vkDevice, vkCmdPool, nullptr);
    }

    inline void selectQueueFamily(uint32_t& index, QueueTransfer transfer) {
        switch (transfer) {
            case QueueTransfer::Identity:
                index = queueFamily;
                break;
            case QueueTransfer::Graphics:
                index = device->queues.graphicsFamily;
                break;
            case QueueTransfer::Compute:
                index = device->queues.computeFamily;
                break;
            case QueueTransfer::Transfer:
                index = device->queues.transferFamily;
                break;
            case QueueTransfer::General:
                index = device->queues.generalFamily;
                break;
        }
    }

    void finalizeBarrier() {
        if (!wasLastCmdBarrier) {
            return;
        }

        std::vector<VkImageMemoryBarrier> images;
        images.reserve(barrier.textures.size());
        for (TextureBarrier const& b : barrier.textures) {
            VkImageMemoryBarrier barrier = {};
            barrier.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER;
            barrier.srcAccessMask = resourceAccessToVkAccessFlags(b.srcAccess);
            barrier.dstAccessMask = resourceAccessToVkAccessFlags(b.dstAccess);
            barrier.oldLayout = textureLayoutToVkImageLayout(b.oldLayout);
            barrier.newLayout = textureLayoutToVkImageLayout(b.newLayout);
            selectQueueFamily(barrier.srcQueueFamilyIndex, b.srcQueue);
            selectQueueFamily(barrier.dstQueueFamilyIndex, b.dstQueue);
            barrier.image = b.texture->vkImage;
            barrier.subresourceRange.aspectMask = textureAspectToVkImageAspectFlags(b.subresource.aspect);
            barrier.subresourceRange.baseMipLevel = b.subresource.mipBase;
            barrier.subresourceRange.levelCount = b.subresource.mipCount;
            barrier.subresourceRange.baseArrayLayer = b.subresource.layerBase;
            barrier.subresourceRange.layerCount = b.subresource.layerCount;

            images.push_back(barrier);
        }

        std::vector<VkBufferMemoryBarrier> buffers;
        buffers.reserve(barrier.buffers.size());
        for (BufferBarrier const& b : barrier.buffers) {
            VkBufferMemoryBarrier barrier = {};
            barrier.sType = VK_STRUCTURE_TYPE_BUFFER_MEMORY_BARRIER;
            barrier.srcAccessMask = resourceAccessToVkAccessFlags(b.srcAccess);
            barrier.dstAccessMask = resourceAccessToVkAccessFlags(b.dstAccess);
            selectQueueFamily(barrier.srcQueueFamilyIndex, b.srcQueue);
            selectQueueFamily(barrier.dstQueueFamilyIndex, b.dstQueue);
            barrier.buffer = b.buffer->vkBuffer;
            barrier.offset = static_cast<VkDeviceSize>(b.offset);
            barrier.size = static_cast<VkDeviceSize>(b.size);

            buffers.push_back(barrier);
        }

        std::vector<VkMemoryBarrier> globals;
        globals.reserve(barrier.globals.size());
        for (GlobalResourceBarrier const& b : barrier.globals) {
            VkMemoryBarrier barrier = {};
            barrier.sType = VK_STRUCTURE_TYPE_MEMORY_BARRIER;
            barrier.srcAccessMask = resourceAccessToVkAccessFlags(b.srcAccess);
            barrier.dstAccessMask = resourceAccessToVkAccessFlags(b.dstAccess);

            globals.push_back(barrier);
        }

        if (barrier.stageSync == nullptr) {
            vkCmdPipelineBarrier(vkCmdBuffer, pipelineStageToVkPipelineStageFlags(barrier.exec.srcStage), pipelineStageToVkPipelineStageFlags(barrier.exec.dstStage), 0, static_cast<uint32_t>(globals.size()), globals.empty() ? nullptr : globals.data(), static_cast<uint32_t>(buffers.size()), buffers.empty() ? nullptr : buffers.data(), static_cast<uint32_t>(images.size()), images.empty() ? nullptr : images.data());
        } else {
            vkCmdWaitEvents(vkCmdBuffer, 1, &barrier.stageSync->vkEvent, pipelineStageToVkPipelineStageFlags(barrier.exec.srcStage), pipelineStageToVkPipelineStageFlags(barrier.exec.dstStage), static_cast<uint32_t>(globals.size()), globals.empty() ? nullptr : globals.data(), static_cast<uint32_t>(buffers.size()), buffers.empty() ? nullptr : buffers.data(), static_cast<uint32_t>(images.size()), images.empty() ? nullptr : images.data());
        }

        barrier.clear();
        wasLastCmdBarrier = false;
    }
};

namespace wsi {

 struct SwapchainBackbuffer {
     VkImage vkImage;
     std::vector<VkImageView> vkImageViews;
 };

struct Swapchain_t {
    ObjectBase_t base;

    Device_t* device;
    VkSwapchainKHR vkSwapchain;
    Surface surface;

    TextureFormat format;
    kobalt::wsi::PresentMode presentMode;
    uint32_t count;

    Texture_t backbuffer;
    std::vector<TextureView_t*> backbufferViews;
    std::vector<SwapchainBackbuffer> internalBackbuffers;
    uint32_t currentIndex = 0;

    bool imageAcquired = false;
    
    Swapchain_t(Device_t* device, VkSwapchainKHR vkSwapchain, Surface& surface, TextureFormat format, TextureUsage usage, uint32_t width, uint32_t height, uint32_t layerCount, kobalt::wsi::PresentMode presentMode, uint32_t count) : base(&device->base.obj, internal::ObjectType::WSI_Swapchain, vkSwapchain), device(device), vkSwapchain(vkSwapchain), format(format), presentMode(presentMode), count(count), surface(surface), backbuffer(device, &base.obj, format, usage, kobalt::TextureDimensions::Texture2D, kobalt::MemoryLocation::DeviceLocal, width, height, 1, layerCount, 1, 1) {
        backbufferViews = {};
        internalBackbuffers.resize(count);

        ++surface.refCount;

        std::vector<VkImage> vkImages(count);
        vkGetSwapchainImagesKHR(device->vkDevice, vkSwapchain, &count, &vkImages[0]);

        for (uint32_t i = 0; i < count; ++i) {
            internalBackbuffers[i].vkImage = vkImages[i];
            internalBackbuffers[i].vkImageViews = {};
        }
    }

    ~Swapchain_t() {
        vkDeviceWaitIdle(device->vkDevice);
        vkDestroySwapchainKHR(device->vkDevice, vkSwapchain, nullptr);

        size_t destroyIndex = SIZE_MAX;
        for (size_t i = 0; i < surfaces.size(); ++i) {
            if (surfaces[i].vkSurface == surface.vkSurface) {
                if (--surfaces[i].refCount == 0) {
                    destroyIndex = i;
                }
                break;
            }
        }

        if (destroyIndex != SIZE_MAX) {
            vkDestroySurfaceKHR(vkInstance, surfaces[destroyIndex].vkSurface, nullptr);
            surfaces.erase(surfaces.begin() + destroyIndex);
        }
    }

    Texture_t* getBackbuffer() {
        return &backbuffer;
    }
    
    void destroyView(TextureView_t* view) {
        vkDeviceWaitIdle(device->vkDevice);
        uint32_t index = view->swapchainInternalViewIndex;
        for (SwapchainBackbuffer& b : internalBackbuffers) {
            vkDestroyImageView(device->vkDevice, b.vkImageViews[index], nullptr);
        }

        backbufferViews.erase(backbufferViews.begin() + index);
        for (size_t i = index; i < backbufferViews.size(); ++i) {
            backbufferViews[i]->swapchainInternalViewIndex = static_cast<uint32_t>(i);
        }
    }
};

} /* namespace wsi */

#ifdef KOBALT_DEFAULT_DEBUG_MESSENGER
static void defaultDebugMessenger(const char* message, DebugSeverity severity, Object_t* object) {
    const char* severityStrings[] = {
        "WARNING",
        "ERROR",
    };

    const char* objectTypeStrings[] = {
        "Device",
        "Shader",
        "VertexInputState",
        "RasterizationState",
        "GraphicsPipeline",
        "Buffer",
        "Texture",
        "CommandList",
        "wsi::Swapchain"
    };

    ObjectBase_t* objBase = reinterpret_cast<ObjectBase_t*>(object);
    const char* sevStr = (severity >= DebugSeverity::Warning && severity <= DebugSeverity::Error) ? severityStrings[static_cast<uint32_t>(severity)] : "UNKNOWN";
    const char* typeStr = (object != nullptr) ? objectTypeStrings[static_cast<uint32_t>(objBase->obj.type)] : "NONE";

    if (object != nullptr) {
        printf("[kobalt] %s (Source Object: %s 0x%016llx \"%s\"): %s\n", sevStr, typeStr, reinterpret_cast<unsigned long long>(objBase), objBase->debugName, message);
    } else {
        printf("[kobalt] %s: %s\n", sevStr, message);
    }
}

static DebugCallback debugCallback = defaultDebugMessenger;
#else
static DebugCallback debugCallback = nullptr;
#endif /* #ifdef KOBALT_DEFAULT_DEBUG_MESSENGER */

#define KOBALT_PRINT(sev_, obj_, msg_) if (internal::debugCallback != nullptr) { internal::debugCallback(msg_, sev_, obj_); }
#define KOBALT_PRINTF(sev_, obj_, fmt_, ...) if (internal::debugCallback != nullptr) { char buffer[256]; snprintf(buffer, 256, fmt_, __VA_ARGS__); internal::debugCallback(buffer, sev_, obj_); }
#define KOBALT_PRINTF_SIZED(sev_, obj_, bufsize_, fmt_, ...) if (internal::debugCallback != nullptr) { char buffer[bufsize_]; snprintf(buffer, bufsize_, fmt_, __VA_ARGS__); internal::debugCallback(buffer, sev_, obj_); }

static VkBool32 vkDebugMessengerCallback(VkDebugUtilsMessageSeverityFlagBitsEXT messageSeverity, VkDebugUtilsMessageTypeFlagsEXT messageTypes, const VkDebugUtilsMessengerCallbackDataEXT* pCallbackData, void* pUserData) {
    DebugSeverity severity = DebugSeverity::Error;
    if (messageSeverity == VK_DEBUG_UTILS_MESSAGE_SEVERITY_WARNING_BIT_EXT) {
        severity = DebugSeverity::Warning;
    }

    KOBALT_PRINTF_SIZED(severity, nullptr, 2048, "%s", pCallbackData->pMessage);
    return VK_FALSE;
}

static bool createSwapchainGeneric(kobalt::wsi::Swapchain& swapchain, Device_t* dev, Surface& surface, uint32_t width, uint32_t height, uint32_t layerCount, TextureUsage usage, bool sRGBFormat, kobalt::wsi::PresentMode presentMode) {
    uint32_t surfaceFormatCount;
    if (vkGetPhysicalDeviceSurfaceFormatsKHR(dev->vkPhysicalDevice, surface.vkSurface, &surfaceFormatCount, nullptr) != VK_SUCCESS) {
        KOBALT_PRINT(DebugSeverity::Error, &dev->base.obj, "internal Vulkan error");
        return false;
    }

    std::vector<VkSurfaceFormatKHR> surfaceFormats(surfaceFormatCount);
    if (vkGetPhysicalDeviceSurfaceFormatsKHR(dev->vkPhysicalDevice, surface.vkSurface, &surfaceFormatCount, &surfaceFormats[0]) != VK_SUCCESS) {
        KOBALT_PRINT(DebugSeverity::Error, &dev->base.obj, "internal Vulkan error");
        return false;
    }

    uint32_t preferredSurfaceFormat = 0;
    for (uint32_t i = 0; i < surfaceFormatCount; ++i) {
        if (sRGBFormat) {
            switch (surfaceFormats[i].format) {
                case VK_FORMAT_R8G8B8_SRGB:
                case VK_FORMAT_B8G8R8_SRGB:
                case VK_FORMAT_R8G8B8A8_SRGB:
                case VK_FORMAT_B8G8R8A8_SRGB:
                    if (surfaceFormats[i].colorSpace == VK_COLOR_SPACE_SRGB_NONLINEAR_KHR) {
                        preferredSurfaceFormat = i;
                        i = surfaceFormatCount;
                    }
                default:
                    break;
            }
            continue;
        }

        switch (surfaceFormats[i].format) {
            case VK_FORMAT_R8G8B8_UNORM:
            case VK_FORMAT_B8G8R8_UNORM:
            case VK_FORMAT_R8G8B8A8_UNORM:
            case VK_FORMAT_B8G8R8A8_UNORM:
                if (surfaceFormats[i].colorSpace == VK_COLOR_SPACE_SRGB_NONLINEAR_KHR) {
                    preferredSurfaceFormat = i;
                    i = surfaceFormatCount;
                }
            default:
                break;
        }
    }

    if (sRGBFormat) {
        switch (surfaceFormats[preferredSurfaceFormat].format) {
            case VK_FORMAT_R8G8B8_SRGB:
            case VK_FORMAT_B8G8R8_SRGB:
            case VK_FORMAT_R8G8B8A8_SRGB:
            case VK_FORMAT_B8G8R8A8_SRGB:
                break;
            default:
                KOBALT_PRINT(DebugSeverity::Error, &dev->base.obj, "failed to find suitable sRGB format");
                return false;
        }
    } else {
        switch (surfaceFormats[preferredSurfaceFormat].format) {
            case VK_FORMAT_R8G8B8_UNORM:
            case VK_FORMAT_B8G8R8_UNORM:
            case VK_FORMAT_R8G8B8A8_UNORM:
            case VK_FORMAT_B8G8R8A8_UNORM:
                break;
            default:
                KOBALT_PRINT(DebugSeverity::Error, &dev->base.obj, "failed to find suitable non-sRGB format");
                return false;
        }
    }

    uint32_t presentModeCount;
    if (vkGetPhysicalDeviceSurfacePresentModesKHR(dev->vkPhysicalDevice, surface.vkSurface, &presentModeCount, nullptr) != VK_SUCCESS) {
        KOBALT_PRINT(DebugSeverity::Error, &dev->base.obj, "internal Vulkan error");
        return false;
    }

    std::vector<VkPresentModeKHR> presentModes(presentModeCount);
    if (vkGetPhysicalDeviceSurfacePresentModesKHR(dev->vkPhysicalDevice, surface.vkSurface, &presentModeCount, &presentModes[0]) != VK_SUCCESS) {
        KOBALT_PRINT(DebugSeverity::Error, &dev->base.obj, "internal Vulkan error");
        return false;
    }

    uint32_t preferredPresentMode = 0;
    uint32_t preferredPresentModeScore = 0;

    const uint32_t mailboxMailboxScore = 4;

    const uint32_t fifoRelaxedVSyncScore = 3;
    const uint32_t fifoVSyncScore = 2;
    const uint32_t mailboxVSyncScore = 1;

    const uint32_t immediateImmediateScore = 2;
    const uint32_t fifoRelaxedImmediateScore = 1;

    for (uint32_t i = 0; i < presentModeCount; ++i) {
        if ((presentMode & kobalt::wsi::PresentMode::Mailbox) == kobalt::wsi::PresentMode::Mailbox && presentModes[i] == VK_PRESENT_MODE_MAILBOX_KHR && preferredPresentMode < mailboxMailboxScore) {
            preferredPresentMode = i;
            preferredPresentModeScore = mailboxMailboxScore;
        }

        if ((presentMode & kobalt::wsi::PresentMode::VSync) == kobalt::wsi::PresentMode::VSync) {
            if (presentModes[i] == VK_PRESENT_MODE_FIFO_RELAXED_KHR && preferredPresentMode < fifoRelaxedVSyncScore) {
                preferredPresentMode = i;
                preferredPresentModeScore = fifoRelaxedVSyncScore;
            } else if (presentModes[i] == VK_PRESENT_MODE_FIFO_KHR && preferredPresentModeScore < fifoVSyncScore) {
                preferredPresentMode = i;
                preferredPresentModeScore = fifoVSyncScore;
            }
        }

        if ((presentMode & kobalt::wsi::PresentMode::Immediate) == kobalt::wsi::PresentMode::Immediate) {
            if (presentModes[i] == VK_PRESENT_MODE_IMMEDIATE_KHR && preferredPresentModeScore < immediateImmediateScore) {
                preferredPresentMode = i;
                preferredPresentModeScore = immediateImmediateScore;
            } else if (presentModes[i] == VK_PRESENT_MODE_FIFO_RELAXED_KHR && preferredPresentModeScore < fifoRelaxedImmediateScore) {
                preferredPresentMode = i;
                preferredPresentModeScore = fifoRelaxedImmediateScore;
            }
        }
    }

    if (preferredPresentModeScore == 0) {
        KOBALT_PRINT(DebugSeverity::Error, &dev->base.obj, "failed to find suitable present mode");
        return false;
    }

    VkSwapchainCreateInfoKHR swapchainCI = {};
    swapchainCI.sType = VK_STRUCTURE_TYPE_SWAPCHAIN_CREATE_INFO_KHR;
    swapchainCI.surface = surface.vkSurface;
    swapchainCI.minImageCount = std::min(std::max(presentModes[preferredPresentMode] == VK_PRESENT_MODE_MAILBOX_KHR ? 3u : 2u, surface.vkCapabilities.minImageCount), surface.vkCapabilities.maxImageCount);
    swapchainCI.imageFormat = surfaceFormats[preferredSurfaceFormat].format;
    swapchainCI.imageColorSpace = surfaceFormats[preferredSurfaceFormat].colorSpace;
    swapchainCI.imageExtent.width = width;
    swapchainCI.imageExtent.height = height;
    swapchainCI.imageArrayLayers = layerCount;
    swapchainCI.imageUsage = textureUsageToVkImageUsageFlags(usage);
    swapchainCI.imageSharingMode = VK_SHARING_MODE_EXCLUSIVE;
    swapchainCI.preTransform = VK_SURFACE_TRANSFORM_IDENTITY_BIT_KHR;
    swapchainCI.compositeAlpha = VK_COMPOSITE_ALPHA_OPAQUE_BIT_KHR;
    swapchainCI.presentMode = presentModes[preferredPresentMode];
    swapchainCI.clipped = VK_FALSE;
    swapchainCI.oldSwapchain = nullptr;

    VkSwapchainKHR vkSwapchain;
    if (vkCreateSwapchainKHR(dev->vkDevice, &swapchainCI, nullptr, &vkSwapchain)) {
        KOBALT_PRINT(DebugSeverity::Error, &dev->base.obj, "internal Vulkan error; failed to create swapchain");
        return false;
    }

    swapchain = &(new wsi::Swapchain_t(dev, vkSwapchain, surface, textureFormatFromVkFormat(swapchainCI.imageFormat), usage, width, height, layerCount, presentModeFromVkPresentModeKHR(swapchainCI.presentMode), swapchainCI.minImageCount))->base.obj;
    return true;
}

void setDebugUtilsObjectName(VkDevice device, VkObjectType type, uint64_t handle, const char* name) {
    if (internal::instanceSymbols.vkSetDebugUtilsObjectNameEXT != nullptr) {
        VkDebugUtilsObjectNameInfoEXT objectNI = {};
        objectNI.sType = VK_STRUCTURE_TYPE_DEBUG_UTILS_OBJECT_NAME_INFO_EXT;
        objectNI.objectType = type;
        objectNI.objectHandle = handle;
        objectNI.pObjectName = name;

        if (objectNI.objectType != VK_OBJECT_TYPE_UNKNOWN && objectNI.objectHandle != 0) {
            internal::instanceSymbols.vkSetDebugUtilsObjectNameEXT(device, &objectNI);
        }
    }
}

} /* namespace internal */

void setDebugCallback(DebugCallback callback) {
    internal::debugCallback = callback;
}

bool init(bool debug) {
    if (internal::vkInstance != nullptr) {
        KOBALT_PRINT(DebugSeverity::Warning, nullptr, "kobalt is already initialized");
        return true;
    }

    VkInstanceCreateFlags flags = 0;

    std::vector<prism::vk::Extension> extensions = {
        VK_KHR_SURFACE_EXTENSION_NAME
    };

#ifdef _WIN32
    extensions.push_back("VK_KHR_win32_surface");
#elif __APPLE__
    flags |= VK_INSTANCE_CREATE_ENUMERATE_PORTABILITY_BIT_KHR;
    extensions.push_back(VK_KHR_PORTABILITY_ENUMERATION_EXTENSION_NAME);
    extensions.push_back("VK_EXT_metal_surface");
#ifdef TARGET_OS_MAC
    extensions.push_back(prism::vk::Extension("VK_MVK_macos_surface", false));
#elif defined(TARGET_OS_IOS)
    extensions.push_back(prism::vk::Extension("VK_MVK_ios_surface", false));
#endif /* #ifdef TARGET_OS_MAC */
#else
    extensions.push_back(prism::vk::Extension("VK_KHR_xlib_surface", false));
    extensions.push_back(prism::vk::Extension("VK_KHR_xcb_surface", false));
#endif /* #ifdef _WIN32 */

    std::vector<prism::vk::Layer> layers;
    if (debug) {
        extensions.push_back(prism::vk::Extension(VK_EXT_DEBUG_UTILS_EXTENSION_NAME, false));
        layers.push_back(prism::vk::Layer("VK_LAYER_KHRONOS_validation", false));
    }

    if (!prism::vk::createInstance(flags, extensions, layers, VK_MAKE_API_VERSION(0, 1, 0, 0), nullptr, internal::vkInstance)) {
        KOBALT_PRINT(DebugSeverity::Error, nullptr, "failed to create Vulkan instance");
        return false;
    }

    for (prism::vk::Extension const& e : extensions) {
        if (e.name == VK_EXT_DEBUG_UTILS_EXTENSION_NAME) {
            PFN_vkCreateDebugUtilsMessengerEXT createDebugUtilsMessengerEXT = reinterpret_cast<PFN_vkCreateDebugUtilsMessengerEXT>(vkGetInstanceProcAddr(internal::vkInstance, "vkCreateDebugUtilsMessengerEXT"));
            if (createDebugUtilsMessengerEXT != nullptr) {
                VkDebugUtilsMessengerCreateInfoEXT createInfo = {};
                createInfo.sType = VK_STRUCTURE_TYPE_DEBUG_UTILS_MESSENGER_CREATE_INFO_EXT;
                createInfo.messageSeverity = VK_DEBUG_UTILS_MESSAGE_SEVERITY_ERROR_BIT_EXT | VK_DEBUG_UTILS_MESSAGE_SEVERITY_WARNING_BIT_EXT;
                createInfo.messageType = VK_DEBUG_UTILS_MESSAGE_TYPE_GENERAL_BIT_EXT | VK_DEBUG_UTILS_MESSAGE_TYPE_PERFORMANCE_BIT_EXT | VK_DEBUG_UTILS_MESSAGE_TYPE_VALIDATION_BIT_EXT;
                createInfo.pfnUserCallback = internal::vkDebugMessengerCallback;

                createDebugUtilsMessengerEXT(internal::vkInstance, &createInfo, nullptr, &internal::vkDebugMessengerEXT);
            }
        }
    }

    internal::instanceSymbols.vkSetDebugUtilsObjectNameEXT = reinterpret_cast<PFN_vkSetDebugUtilsObjectNameEXT>(vkGetInstanceProcAddr(internal::vkInstance, "vkSetDebugUtilsObjectNameEXT"));

    if (!prism::vk::enumeratePhysicalDevices(internal::vkInstance, internal::vkPhysicalDevices)) {
        KOBALT_PRINT(DebugSeverity::Error, nullptr, "internal Vulkan error");

        deinit();
        return false;
    }

    return true;
}

void deinit() {
    if (internal::vkInstance == nullptr) {
        KOBALT_PRINT(DebugSeverity::Warning, nullptr, "kobalt cannot be deinitialized without being initialized");
        return;
    }

    internal::vkPhysicalDevices.clear();

    if (internal::vkDebugMessengerEXT != nullptr) {
        PFN_vkDestroyDebugUtilsMessengerEXT destroyDebugUtilsMessengerEXT = reinterpret_cast<PFN_vkDestroyDebugUtilsMessengerEXT>(vkGetInstanceProcAddr(internal::vkInstance, "vkDestroyDebugUtilsMessengerEXT"));
        if (destroyDebugUtilsMessengerEXT != nullptr) {
            destroyDebugUtilsMessengerEXT(internal::vkInstance, internal::vkDebugMessengerEXT, nullptr);
        }
    }

    vkDestroyInstance(internal::vkInstance, nullptr);
    internal::vkInstance = nullptr;
}

void setDebugName(Object_t* object, const char* name) {
    if (object == nullptr || name == nullptr) {
        KOBALT_PRINT(DebugSeverity::Error, nullptr, "object or name is null; cannot set object debug name");
        return;
    }

    internal::ObjectBase_t* baseObj = reinterpret_cast<internal::ObjectBase_t*>(object);

    uint32_t len = static_cast<uint32_t>(strlen(name));
    memcpy(baseObj->debugName, name, std::min(len, 63u));
    baseObj->debugName[std::min(len, 63u)] = '\0';

    VkDevice vkDevice = nullptr;
    if (object->device == nullptr || object->type == internal::ObjectType::Device) {
        internal::Device_t* dev = reinterpret_cast<internal::Device_t*>(object);
        vkDevice = dev->vkDevice;
    } else {
        internal::Device_t* dev = reinterpret_cast<internal::Device_t*>(object->device);
        vkDevice = dev->vkDevice;
    }

    internal::setDebugUtilsObjectName(vkDevice, internal::objectTypeToVkObjectType(object->type), baseObj->vkHandle, baseObj->debugName);
}

#define KOBALT_INTERNAL_DESTROY_OBJ_EXP(obj_, enum_, type_) case internal::ObjectType::enum_: delete reinterpret_cast<internal::type_##_t*>(obj_); break
#define KOBALT_INTERNAL_DESTROY_OBJ(obj_, type_) case internal::ObjectType::type_: delete reinterpret_cast<internal::type_##_t*>(obj_); break

void destroy(Object_t* object) {
    if (object == nullptr) {
        KOBALT_PRINT(DebugSeverity::Error, nullptr, "cannot destroy null object");
        return;
    }

    switch (object->type) {
        KOBALT_INTERNAL_DESTROY_OBJ(object, Device);
        KOBALT_INTERNAL_DESTROY_OBJ(object, Shader);
        KOBALT_INTERNAL_DESTROY_OBJ(object, VertexInputState);
        KOBALT_INTERNAL_DESTROY_OBJ(object, RasterizationState);
        KOBALT_INTERNAL_DESTROY_OBJ(object, CommandList);
        KOBALT_INTERNAL_DESTROY_OBJ(object, QueueSync);
        KOBALT_INTERNAL_DESTROY_OBJ(object, HostSync);
        KOBALT_INTERNAL_DESTROY_OBJ(object, StageSync);
        KOBALT_INTERNAL_DESTROY_OBJ_EXP(object, WSI_Swapchain, wsi::Swapchain);

        case internal::ObjectType::TextureView: {
            internal::TextureView_t* view = reinterpret_cast<internal::TextureView_t*>(object);
            if (view->swapchain != nullptr) {
                internal::wsi::Swapchain_t* swp = reinterpret_cast<internal::wsi::Swapchain_t*>(view->swapchain);
                swp->destroyView(view);
            }

            delete view;
            break;
        }

        default:
            KOBALT_PRINT(DebugSeverity::Error, nullptr, "unknown object type to destroy");
            return;
    }
}

bool enumerateDeviceAdapters(DeviceAdapterInfo& adapterInfo, uint32_t id) {
    if (id >= internal::vkPhysicalDevices.size()) {
        return false;
    }

    VkPhysicalDevice physical = internal::vkPhysicalDevices[id];

    VkPhysicalDeviceProperties props;
    vkGetPhysicalDeviceProperties(physical, &props);

    switch (props.vendorID) {
        case 0x10de:
            adapterInfo.vendor = DeviceVendor::NVIDIA;
            break;
        case 0x1002:
            adapterInfo.vendor = DeviceVendor::AMD;
            break;
        case 0x8086:
            adapterInfo.vendor = DeviceVendor::Intel;
            break;
        case 0x13b5:
            adapterInfo.vendor = DeviceVendor::ARM;
            break;
        case 0x5143:
            adapterInfo.vendor = DeviceVendor::Qualcomm;
            break;
        case 0x1010:
            adapterInfo.vendor = DeviceVendor::ImgTec;
            break;
        case 0x106b:
            adapterInfo.vendor = DeviceVendor::Apple;
            break;
        case 0x10005:
            adapterInfo.vendor = DeviceVendor::Mesa;
            break;
        default:
            adapterInfo.vendor = DeviceVendor::Unknown;
            break;
    }

    switch (props.deviceType) {
        case VK_PHYSICAL_DEVICE_TYPE_DISCRETE_GPU:
            adapterInfo.type = DeviceType::Discrete;
            break;
        case VK_PHYSICAL_DEVICE_TYPE_INTEGRATED_GPU:
            adapterInfo.type = DeviceType::Integrated;
            break;
        case VK_PHYSICAL_DEVICE_TYPE_VIRTUAL_GPU:
            adapterInfo.type = DeviceType::Virtual;
            break;
        case VK_PHYSICAL_DEVICE_TYPE_CPU:
        default:
            adapterInfo.type = DeviceType::CPU;
            break;
    }

    std::vector<VkExtensionProperties> availableExtensions;
    if (!prism::vk::enumerateDeviceExtensionProperties(physical, nullptr, availableExtensions)) {
        KOBALT_PRINT(DebugSeverity::Error, nullptr, "internal Vulkan error");
        return false;
    }

    for (VkExtensionProperties const& e : availableExtensions) {
        if (strcmp(e.extensionName, VK_KHR_DYNAMIC_RENDERING_EXTENSION_NAME) == 0) {
            adapterInfo.support.dynamicRenderPass = true;
        } else if (strcmp(e.extensionName, VK_KHR_SWAPCHAIN_EXTENSION_NAME) == 0) {
            adapterInfo.support.swapchain = true;
        }
    }

    adapterInfo.maxTextureSize1D = props.limits.maxImageDimension1D;
    adapterInfo.maxTextureSize2D = props.limits.maxImageDimension2D;
    adapterInfo.maxTextureSize3D = props.limits.maxImageDimension3D;

    adapterInfo.maxRenderTargetSize[0] = props.limits.maxFramebufferWidth;
    adapterInfo.maxRenderTargetSize[1] = props.limits.maxFramebufferHeight;
    adapterInfo.maxRenderTargetSize[2] = props.limits.maxFramebufferLayers;
    adapterInfo.maxRenderTargets = props.limits.maxColorAttachments;

    adapterInfo.maxComputeWorkGroupCount[0] = props.limits.maxComputeWorkGroupCount[0];
    adapterInfo.maxComputeWorkGroupCount[1] = props.limits.maxComputeWorkGroupCount[1];
    adapterInfo.maxComputeWorkGroupCount[2] = props.limits.maxComputeWorkGroupCount[2];

    adapterInfo.maxComputeWorkGroupSize[0] = props.limits.maxComputeWorkGroupSize[0];
    adapterInfo.maxComputeWorkGroupSize[1] = props.limits.maxComputeWorkGroupSize[1];
    adapterInfo.maxComputeWorkGroupSize[2] = props.limits.maxComputeWorkGroupSize[2];

    VkPhysicalDeviceMemoryProperties mprops;
    vkGetPhysicalDeviceMemoryProperties(physical, &mprops);

    adapterInfo.localMemorySize = 0;
    adapterInfo.otherMemorySize = 0;
    for (uint32_t i = 0; i < mprops.memoryHeapCount; ++i) {
        if (mprops.memoryHeaps[i].flags & VK_MEMORY_HEAP_DEVICE_LOCAL_BIT) {
            adapterInfo.localMemorySize += mprops.memoryHeaps[i].size;
        } else {
            adapterInfo.otherMemorySize += mprops.memoryHeaps[i].size;
        }
    }

    uint32_t len = std::min(static_cast<uint32_t>(strlen(props.deviceName)), 63u);
    memcpy(adapterInfo.name, props.deviceName, len);
    adapterInfo.name[len] = '\0';
    return true;
}

bool createDevice(Device& device, uint32_t id, DeviceSupport support) {
    if (internal::vkInstance == nullptr) {
        KOBALT_PRINT(DebugSeverity::Error, nullptr, "kobalt is not initialized yet");
        return false;
    }

    if (id >= internal::vkPhysicalDevices.size()) {
        KOBALT_PRINTF(DebugSeverity::Error, nullptr, "provided device adapter ID (%u) is greater than available devices (0 to %zu)", id, internal::vkPhysicalDevices.size())
            return false;
    }

    VkPhysicalDevice vkPhysical = internal::vkPhysicalDevices[id];

    std::vector<VkExtensionProperties> availableExtensions;
    if (!prism::vk::enumerateDeviceExtensionProperties(vkPhysical, nullptr, availableExtensions)) {
        KOBALT_PRINT(DebugSeverity::Error, nullptr, "internal Vulkan error");
        return false;
    }

    internal::ExtraDeviceFeatures extraSupport = {};
    DeviceSupport availableSupport = {};
    for (VkExtensionProperties const& e : availableExtensions) {
        if (strcmp(e.extensionName, VK_KHR_DYNAMIC_RENDERING_EXTENSION_NAME) == 0) {
            availableSupport.dynamicRenderPass = true;
        } else if (strcmp(e.extensionName, VK_KHR_SWAPCHAIN_EXTENSION_NAME) == 0) {
            availableSupport.swapchain = true;
        }
    }

    if (availableSupport < support) {
        KOBALT_PRINT(DebugSeverity::Error, nullptr, "device does not support requested features");
        return false;
    }

    std::vector<prism::vk::Extension> extensions;
    if (support.dynamicRenderPass) {
        extensions.push_back(VK_KHR_DYNAMIC_RENDERING_EXTENSION_NAME);
    }

    if (support.swapchain) {
        extensions.push_back(VK_KHR_SWAPCHAIN_EXTENSION_NAME);
    }
    
#ifdef __APPLE__
    extensions.push_back("VK_KHR_portability_subset");
#endif /* #ifdef __APPLE__ */

    std::vector<VkQueueFamilyProperties> queueFamilyProperties;
    prism::vk::getPhysicalDeviceQueueFamilyProperties(vkPhysical, queueFamilyProperties);

    uint32_t bestGeneral = 0;
    uint32_t generalScore = 0;
    QueueType generalTypes = static_cast<QueueType>(0);

    uint32_t bestGraphics = 0;
    uint32_t graphicsExtra = UINT32_MAX;
    QueueType graphicsTypes = static_cast<QueueType>(0);

    uint32_t bestCompute = 0;
    uint32_t computeExtra = UINT32_MAX;
    QueueType computeTypes = static_cast<QueueType>(0);

    uint32_t bestTransfer = 0;
    uint32_t transferExtra = UINT32_MAX;
    QueueType transferTypes = static_cast<QueueType>(0);

    for (uint32_t i = 0; i < queueFamilyProperties.size(); ++i) {
        VkQueueFamilyProperties const& p = queueFamilyProperties[i];
        uint32_t extra = 0;
        QueueType types = static_cast<QueueType>(0);
        for (uint32_t i = 0; i < 32; ++i) {
            if (p.queueFlags & (1 << i)) {
                ++extra;
            }
        }

        if (p.queueFlags & VK_QUEUE_GRAPHICS_BIT) {
            types |= QueueType::Graphics;
        }

        if (p.queueFlags & VK_QUEUE_COMPUTE_BIT) {
            types |= QueueType::Compute;
        }

        if (p.queueFlags & VK_QUEUE_TRANSFER_BIT) {
            types |= QueueType::Transfer;
        }

        if (extra > generalScore) {
            bestGeneral = i;
            generalScore = extra;
            generalTypes = types;
        }

        if (p.queueFlags & VK_QUEUE_GRAPHICS_BIT && extra < graphicsExtra) {
            bestGraphics = i;
            graphicsExtra = extra;
            graphicsTypes = types;
        }

        if (p.queueFlags & VK_QUEUE_COMPUTE_BIT && extra < computeExtra) {
            bestCompute = i;
            computeExtra = extra;
            computeTypes = types;
        }

        if (p.queueFlags & VK_QUEUE_TRANSFER_BIT && extra < transferExtra) {
            bestTransfer = i;
            transferExtra = extra;
            transferTypes = types;
        }
    }

    float priority = 1.0f;
    VkDeviceQueueCreateInfo queueCI = {};
    queueCI.sType = VK_STRUCTURE_TYPE_DEVICE_QUEUE_CREATE_INFO;
    queueCI.queueFamilyIndex = bestGraphics;
    queueCI.queueCount = 1;
    queueCI.pQueuePriorities = &priority;

    std::vector<VkDeviceQueueCreateInfo> queueCIs = { queueCI };
    if (bestGraphics != bestGeneral) {
        queueCI.queueFamilyIndex = bestGeneral;
        queueCIs.push_back(queueCI);
    }

    if (bestCompute != bestGraphics && bestCompute != bestGeneral) {
        queueCI.queueFamilyIndex = bestCompute;
        queueCIs.push_back(queueCI);
    }

    if (bestTransfer != bestCompute && bestTransfer != bestGraphics && bestTransfer != bestGeneral) {
        queueCI.queueFamilyIndex = bestTransfer;
        queueCIs.push_back(queueCI);
    }
    
    std::vector<internal::EnabledFeaturesX*> enabledFeatures;

    VkPhysicalDeviceDynamicRenderingFeaturesKHR drFeatures = {};
    drFeatures.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_DYNAMIC_RENDERING_FEATURES_KHR;
    drFeatures.dynamicRendering = support.dynamicRenderPass;
    if (support.dynamicRenderPass) {
        enabledFeatures.push_back(reinterpret_cast<internal::EnabledFeaturesX*>(&drFeatures));
    }
    
    void* pNext = nullptr;
    if (!enabledFeatures.empty()) {
        for (uint32_t i = 0; i < enabledFeatures.size() - 1; ++i) {
            enabledFeatures[i]->pNext = enabledFeatures[i + 1];
        }
    
        pNext = enabledFeatures[0];
    }

    VkDevice vkDevice;
    if (!prism::vk::createDevice(vkPhysical, queueCIs, extensions, pNext, nullptr, nullptr, vkDevice)) {
        KOBALT_PRINT(DebugSeverity::Error, nullptr, "internal Vulkan error: device creation");
        return false;
    }

    internal::Device_t* dev = new internal::Device_t(vkDevice, vkPhysical, support, extraSupport, internal::DeviceQueues(bestGraphics, graphicsTypes, bestCompute, computeTypes, bestTransfer, transferTypes, bestGeneral, generalTypes));
    device = &dev->base.obj;
    return true;
}

namespace wsi {

TextureFormat getSwapchainFormat(Swapchain swapchain) {
    if (swapchain == nullptr) {
        KOBALT_PRINT(DebugSeverity::Error, nullptr, "swapchain is null");
        return static_cast<TextureFormat>(-1);
    }

    internal::wsi::Swapchain_t* swp = reinterpret_cast<internal::wsi::Swapchain_t*>(swapchain);
    return swp->format;
}

PresentMode getSwapchainPresentMode(Swapchain swapchain) {
    if (swapchain == nullptr) {
        KOBALT_PRINT(DebugSeverity::Error, nullptr, "swapchain is null");
        return static_cast<PresentMode>(-1);
    }

    internal::wsi::Swapchain_t* swp = reinterpret_cast<internal::wsi::Swapchain_t*>(swapchain);
    return swp->presentMode;
}

uint32_t getSwapchainTextureCount(Swapchain swapchain) {
    if (swapchain == nullptr) {
        KOBALT_PRINT(DebugSeverity::Error, nullptr, "swapchain is null");
        return 0;
    }

    internal::wsi::Swapchain_t* swp = reinterpret_cast<internal::wsi::Swapchain_t*>(swapchain);
    return swp->count;
}

Texture getSwapchainBackbuffer(Swapchain swapchain) {
    if (swapchain == nullptr) {
        KOBALT_PRINT(DebugSeverity::Error, nullptr, "swapchain is null");
        return nullptr;
    }

    internal::wsi::Swapchain_t* swp = reinterpret_cast<internal::wsi::Swapchain_t*>(swapchain);
    return &swp->getBackbuffer()->base.obj;
}

bool prepareNextSwapchainTexture(SwapchainStatus* status, Swapchain swapchain, uint64_t timeout, QueueSync queueSync, HostSync hostSync) {
    if (swapchain == nullptr) {
        KOBALT_PRINT(DebugSeverity::Error, nullptr, "swapchain is null");
        return false;
    }

    internal::wsi::Swapchain_t* swp = reinterpret_cast<internal::wsi::Swapchain_t*>(swapchain);
    internal::QueueSync_t* qs = reinterpret_cast<internal::QueueSync_t*>(queueSync);
    internal::HostSync_t* hs = reinterpret_cast<internal::HostSync_t*>(hostSync);

    uint32_t index;
    VkResult vkResult = vkAcquireNextImageKHR(swp->device->vkDevice, swp->vkSwapchain, timeout, qs != nullptr ? qs->vkSemaphore : nullptr, hs != nullptr ? hs->vkFence : nullptr, &index);
    switch (vkResult) {
        case VK_SUCCESS:
            if (status != nullptr) {
                *status = SwapchainStatus::Ready;
            }
            break;
        case VK_NOT_READY:
            if (status != nullptr) {
                *status = SwapchainStatus::NotReady;
            }
            break;
        case VK_TIMEOUT:
            if (status != nullptr) {
                *status = SwapchainStatus::Timeout;
            }
            break;
        case VK_SUBOPTIMAL_KHR:
            if (status != nullptr) {
                *status = SwapchainStatus::Suboptimal;
            }
            break;
        case VK_ERROR_OUT_OF_DATE_KHR:
            if (status != nullptr) {
                *status = SwapchainStatus::ResizeRequired;
            }
            return false;
        default:
            return false;
    }

    swp->backbuffer.vkImage = swp->internalBackbuffers[index].vkImage;
    swp->currentIndex = index;
    return true;
}

bool present(SwapchainStatus* status, Swapchain swapchain, QueueSync const* waitSyncs, uint32_t waitSyncCount) {
    if (swapchain == nullptr) {
        KOBALT_PRINT(DebugSeverity::Error, nullptr, "swapchain is null");
        return false;
    }

    internal::wsi::Swapchain_t* swp = reinterpret_cast<internal::wsi::Swapchain_t*>(swapchain);

    std::vector<VkSemaphore> waits(waitSyncCount);
    VkSemaphore const* pWaits = nullptr;
    if (waitSyncs != nullptr) {
        for (uint32_t i = 0; i < waitSyncCount; ++i) {
            internal::QueueSync_t* qs = reinterpret_cast<internal::QueueSync_t*>(waitSyncs[i]);
            if (qs == nullptr) {
                KOBALT_PRINTF(DebugSeverity::Error, swapchain, "waitSyncs[%u] is null", i);
                return false;
            }

            waits[i] = qs->vkSemaphore;
        }

        pWaits = waits.data();
    }

    VkPresentInfoKHR presentInfo = {};
    presentInfo.sType = VK_STRUCTURE_TYPE_PRESENT_INFO_KHR;
    presentInfo.waitSemaphoreCount = pWaits != nullptr ? waitSyncCount : 0;
    presentInfo.pWaitSemaphores = pWaits;
    presentInfo.swapchainCount = 1;
    presentInfo.pSwapchains = &swp->vkSwapchain;
    presentInfo.pImageIndices = &swp->currentIndex;

    VkResult vkResult = vkQueuePresentKHR(swp->device->queues.general, &presentInfo);
    switch (vkResult) {
        case VK_SUCCESS:
            if (status != nullptr) {
                *status = SwapchainStatus::PresentationSuccess;
            }
            break;
        case VK_SUBOPTIMAL_KHR:
            if (status != nullptr) {
                *status = SwapchainStatus::Suboptimal;
            }
            break;
        case VK_ERROR_OUT_OF_DATE_KHR:
            if (status != nullptr) {
                *status = SwapchainStatus::ResizeRequired;
            }
            return false;
        default:
            if (status != nullptr) {
                *status = SwapchainStatus::PresentationFailure;
            }
            return false;
    }

    return true;
}

#ifdef KOBALT_GLFW

namespace glfw {

bool createSwapchain(Swapchain& swapchain, Device device, GLFWwindow* window, uint32_t width, uint32_t height, uint32_t layerCount, TextureUsage usage, bool sRGBFormat, PresentMode presentMode) {
    if (device == nullptr) {
        KOBALT_PRINT(DebugSeverity::Error, nullptr, "device is null");
        return false;
    }

    if (window == nullptr) {
        KOBALT_PRINT(DebugSeverity::Error, nullptr, "window is null");
        return false;
    }

    if (width == 0) {
        KOBALT_PRINT(DebugSeverity::Error, nullptr, "width must not be 0");
        return false;
    }

    if (height == 0) {
        KOBALT_PRINT(DebugSeverity::Error, nullptr, "height must not be 0");
        return false;
    }

    if (layerCount == 0) {
        KOBALT_PRINT(DebugSeverity::Error, nullptr, "layerCount must not be 0");
        return false;
    }

    if (internal::textureUsageToVkImageUsageFlags(usage) == 0) {
        KOBALT_PRINTF(DebugSeverity::Error, nullptr, "usage has invalid value: %u", static_cast<uint32_t>(usage));
        return false;
    }

    if ((presentMode & PresentMode::Any) != presentMode) {
        KOBALT_PRINTF(DebugSeverity::Error, nullptr, "unknown combination of PresentMode options: %u", static_cast<uint32_t>(presentMode));
        return false;
    }

    internal::Device_t* dev = reinterpret_cast<internal::Device_t*>(device);

    internal::Surface* surface = nullptr;
    for (internal::Surface& s : internal::surfaces) {
        if (s.type == internal::SurfaceType::GLFW && s.window == window) {
            surface = &s;
            break;
        }
    }

    if (surface == nullptr) {
        VkSurfaceKHR vkSurface;
        if (glfwCreateWindowSurface(internal::vkInstance, window, nullptr, &vkSurface) != VK_SUCCESS) {
            KOBALT_PRINT(DebugSeverity::Error, nullptr, "internal Vulkan error; failed to create GLFW Vulkan surface");
            return false;
        }

        internal::Surface s = {};
        s.vkSurface = vkSurface;
        s.type = internal::SurfaceType::GLFW;
        s.window = reinterpret_cast<void*>(window);

        vkGetPhysicalDeviceSurfaceCapabilitiesKHR(dev->vkPhysicalDevice, vkSurface, &s.vkCapabilities);

        internal::surfaces.push_back(s);
        surface = &internal::surfaces[internal::surfaces.size() - 1];
    }

    return internal::createSwapchainGeneric(swapchain, dev, *surface, width, height, layerCount, usage, sRGBFormat, presentMode);
}

} /* namespace glfw */

#endif /* #ifdef KOBALT_GLFW */

} /* namespace wsi */

bool createShaderSPIRV(Shader& shader, Device device, uint32_t const* data, size_t size) {
    if (device == nullptr) {
        KOBALT_PRINT(DebugSeverity::Error, nullptr, "device is null");
        return false;
    }

    if (data == nullptr) {
        KOBALT_PRINT(DebugSeverity::Error, device, "data is null");
        return false;
    }

    if (size == 0) {
        KOBALT_PRINT(DebugSeverity::Error, device, "size cannot be 0");
        return false;
    }

    if (size % 4 != 0) {
        KOBALT_PRINT(DebugSeverity::Error, device, "size must be a multiple of 4 for SPIR-V");
        return false;
    }

    VkShaderModuleCreateInfo createInfo = {};
    createInfo.sType = VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO;
    createInfo.codeSize = size;
    createInfo.pCode = data;

    internal::Device_t* dev = reinterpret_cast<internal::Device_t*>(device);

    VkShaderModule vkShaderModule;
    if (vkCreateShaderModule(dev->vkDevice, &createInfo, nullptr, &vkShaderModule) != VK_SUCCESS) {
        KOBALT_PRINT(DebugSeverity::Error, device, "internal Vulkan error: failed to create shader module");
        return false;
    }

    internal::Shader_t* sh = new internal::Shader_t(dev, vkShaderModule);
    shader = &sh->base.obj;
    return true;
}

bool createVertexInputState(VertexInputState& vertexInputState, Device device, Topology topology, VertexAttribute const* vertexAttributes, uint32_t vertexAttributeCount, VertexBinding const* vertexBindings, uint32_t vertexBindingCount) {
    if (device == nullptr) {
        KOBALT_PRINT(DebugSeverity::Error, nullptr, "device is null");
        return false;
    }

    if (internal::topologyToVkPrimitiveTopology(topology) == VK_PRIMITIVE_TOPOLOGY_MAX_ENUM) {
        KOBALT_PRINTF(DebugSeverity::Error, device, "topology has invalid value: %u", static_cast<uint32_t>(topology));
        return false;
    }

    for (uint32_t i = 0; i < vertexAttributeCount; ++i) {
        if (vertexAttributes[i].count > 4) {
            KOBALT_PRINTF(DebugSeverity::Error, device, "vertexAttributes[%u].count has invalid value: %u; count must be between >= 1 and <= 4", i, vertexAttributes[i].count);
            return false;
        }

        if (internal::vertexAttributeTypeToVkFormat(vertexAttributes[i].type, vertexAttributes[i].count) == VK_FORMAT_MAX_ENUM) {
            KOBALT_PRINTF(DebugSeverity::Error, device, "vertexAttributes[%u].type has invalid value: %u", i, static_cast<uint32_t>(vertexAttributes[i].type));
            return false;
        }

        bool foundBinding = false;
        for (uint32_t j = 0; j < vertexBindingCount; ++j) {
            if (vertexBindings[j].binding == vertexAttributes[i].binding) {
                foundBinding = true;
                uint32_t size = internal::vertexAttributeTypeSize(vertexAttributes[i].type) * vertexAttributes[i].count;
                if (size + vertexAttributes[i].offset > vertexBindings[j].size) {
                    KOBALT_PRINTF(DebugSeverity::Error, device, "vertexAttributes[%u].offset has invalid value: %u; attributes must stay within size bounds of their binding (binding %u has a size bound of %u and this attribute has a size of %u)", i, vertexAttributes[i].offset, j, vertexBindings[j].size, size);
                    return false;
                }
                break;
            }
        }

        if (!foundBinding) {
            KOBALT_PRINTF(DebugSeverity::Error, device, "vertexAttributes[%u].binding has invalid value: %u; this binding does not exist in vertexBindings", i, vertexAttributes[i].binding);
            return false;
        }
    }

    internal::VertexInputState_t* viState = new internal::VertexInputState_t(device, topology, vertexAttributes, vertexAttributeCount, vertexBindings, vertexBindingCount);
    vertexInputState = &viState->base.obj;
    return true;
}

bool createRasterizationState(RasterizationState& rasterizationState, Device device, FillMode fillMode, CullMode cullMode, FrontFace frontFace, float depthBias, float depthBiasClamp, float slopeScaledDepthBias) {
    if (device == nullptr) {
        KOBALT_PRINT(DebugSeverity::Error, nullptr, "device is null");
        return false;
    }

    if (internal::fillModeToVkPolygonMode(fillMode) == VK_POLYGON_MODE_MAX_ENUM) {
        KOBALT_PRINTF(DebugSeverity::Error, nullptr, "fillMode has invalid value: %u", static_cast<uint32_t>(fillMode));
        return false;
    }

    if (internal::cullModeToVkCullModeFlags(cullMode) == 0 && cullMode != CullMode::None) {
        KOBALT_PRINTF(DebugSeverity::Error, nullptr, "cullMode has invalid value: %u", static_cast<uint32_t>(cullMode));
        return false;
    }

    if (internal::frontFaceToVkFrontFace(frontFace) == VK_FRONT_FACE_MAX_ENUM) {
        KOBALT_PRINTF(DebugSeverity::Error, nullptr, "frontFace has invalid value: %u", static_cast<uint32_t>(frontFace));
        return false;
    }

    internal::RasterizationState_t* rState = new internal::RasterizationState_t(device, fillMode, cullMode, frontFace, depthBias, depthBiasClamp, slopeScaledDepthBias);
    rasterizationState = &rState->base.obj;
    return true;
}

bool createGraphicsPipeline(Pipeline& pipeline, Device device, VertexInputState vertexInputState, RasterizationState rasterizationState, Shader vertexShader, Shader fragmentShader, Shader geometryShader, GraphicsPipelineAttachment const* inputAttachments, uint32_t inputAttachmentCount, GraphicsPipelineAttachment const* renderTargets, uint32_t renderTargetCount, GraphicsPipelineAttachment const* depthStencilTarget, uint32_t subpass, bool dynamicRenderPass, uint32_t viewMask) {
    if (device == nullptr) {
        KOBALT_PRINT(DebugSeverity::Error, nullptr, "device is null");
        return false;
    }
    
    if (vertexShader == nullptr) {
        KOBALT_PRINT(DebugSeverity::Error, device, "vertexShader is null");
        return false;
    }
    
    if (fragmentShader == nullptr) {
        KOBALT_PRINT(DebugSeverity::Error, device, "fragmentShader is null");
        return false;
    }

    VkPipelineVertexInputStateCreateInfo defaultVertexInputCI = {};
    defaultVertexInputCI.sType = VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO;
    defaultVertexInputCI.vertexBindingDescriptionCount = 0;
    defaultVertexInputCI.pVertexBindingDescriptions = nullptr;
    defaultVertexInputCI.vertexAttributeDescriptionCount = 0;
    defaultVertexInputCI.pVertexAttributeDescriptions = nullptr;

    VkPipelineInputAssemblyStateCreateInfo defaultInputAssemblyCI = {};
    defaultInputAssemblyCI.sType = VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO;
    defaultInputAssemblyCI.topology = VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST;

    VkPipelineRasterizationStateCreateInfo defaultRasterizationCI = {};
    defaultRasterizationCI.sType = VK_STRUCTURE_TYPE_PIPELINE_RASTERIZATION_STATE_CREATE_INFO;
    defaultRasterizationCI.polygonMode = VK_POLYGON_MODE_FILL;
    defaultRasterizationCI.cullMode = VK_CULL_MODE_NONE;
    defaultRasterizationCI.frontFace = VK_FRONT_FACE_CLOCKWISE;
    defaultRasterizationCI.lineWidth = 1.0f;
}

bool createTextureView(TextureView& view, Device device, Texture texture, TextureFormat format, TextureDimensions dimensions, ComponentMapping const* mapping, TextureSubresource const* subresource) {
    if (device == nullptr) {
        KOBALT_PRINT(DebugSeverity::Error, nullptr, "device is null");
        return false;
    }

    if (texture == nullptr) {
        KOBALT_PRINT(DebugSeverity::Error, device, "texture is null");
        return false;
    }

    if (internal::textureFormatToVkFormat(format) == VK_FORMAT_MAX_ENUM) {
        KOBALT_PRINTF(DebugSeverity::Error, texture, "format has invalid value: %u", static_cast<uint32_t>(format));
        return false;
    }

    if (internal::textureDimensionsToVkImageType(dimensions) == VK_IMAGE_TYPE_MAX_ENUM) {
        KOBALT_PRINTF(DebugSeverity::Error, texture, "dimensions has invalid value: %u", static_cast<uint32_t>(dimensions));
        return false;
    }

    ComponentMapping map;
    if (mapping != nullptr) {
        if (internal::componentSwizzleToVkComponentSwizzle(mapping->r) == VK_COMPONENT_SWIZZLE_MAX_ENUM) {
            KOBALT_PRINTF(DebugSeverity::Error, texture, "mapping->r has invalid value: %u", static_cast<uint32_t>(mapping->r));
            return false;
        }

        if (internal::componentSwizzleToVkComponentSwizzle(mapping->g) == VK_COMPONENT_SWIZZLE_MAX_ENUM) {
            KOBALT_PRINTF(DebugSeverity::Error, texture, "mapping->g has invalid value: %u", static_cast<uint32_t>(mapping->g));
            return false;
        }

        if (internal::componentSwizzleToVkComponentSwizzle(mapping->b) == VK_COMPONENT_SWIZZLE_MAX_ENUM) {
            KOBALT_PRINTF(DebugSeverity::Error, texture, "mapping->b has invalid value: %u", static_cast<uint32_t>(mapping->b));
            return false;
        }

        if (internal::componentSwizzleToVkComponentSwizzle(mapping->a) == VK_COMPONENT_SWIZZLE_MAX_ENUM) {
            KOBALT_PRINTF(DebugSeverity::Error, texture, "mapping->a has invalid value: %u", static_cast<uint32_t>(mapping->a));
            return false;
        }

        map = *mapping;
    } else {
        map.r = ComponentSwizzle::Identity;
        map.g = ComponentSwizzle::Identity;
        map.b = ComponentSwizzle::Identity;
        map.a = ComponentSwizzle::Identity;
    }

    internal::Device_t* dev = reinterpret_cast<internal::Device_t*>(device);
    internal::Texture_t* tex = reinterpret_cast<internal::Texture_t*>(texture);

    /* TODO: better compatibility check */

    TextureAspect maxTextureAspect = internal::maximumTextureAspectFromTextureFormat(tex->format);
    TextureAspect maxViewAspect = internal::maximumTextureAspectFromTextureFormat(format);
    if ((maxTextureAspect & maxViewAspect) != maxTextureAspect) {
        KOBALT_PRINTF(DebugSeverity::Error, texture, "format is incompatible with texture format based on supported aspect. maximum texture format aspect: %u, provided view format maximum aspect: %u", static_cast<uint32_t>(maxTextureAspect), static_cast<uint32_t>(maxViewAspect));
        return false;
    }

    TextureSubresource subrsrc;
    if (subresource != nullptr) {
        if ((subresource->aspect & (TextureAspect::Color | TextureAspect::Depth | TextureAspect::Stencil)) != subresource->aspect) {
            KOBALT_PRINTF(DebugSeverity::Error, texture, "subresource->aspect has invalid value: %u", static_cast<uint32_t>(subresource->aspect));
            return false;
        } else if ((subresource->aspect & TextureAspect::Color) == TextureAspect::Color && (maxTextureAspect & TextureAspect::Color) != TextureAspect::Color) {
            KOBALT_PRINT(DebugSeverity::Error, texture, "texture format has no color aspect, but subresource->aspect references color");
            return false;
        } else if ((subresource->aspect & TextureAspect::Depth) == TextureAspect::Depth && (maxTextureAspect & TextureAspect::Depth) != TextureAspect::Depth) {
            KOBALT_PRINT(DebugSeverity::Error, texture, "texture format has no depth aspect, but subresource->aspect references depth");
            return false;
        } else if ((subresource->aspect & TextureAspect::Stencil) == TextureAspect::Stencil && (maxTextureAspect & TextureAspect::Stencil) != TextureAspect::Stencil) {
            KOBALT_PRINT(DebugSeverity::Error, texture, "texture format has no stencil aspect, but subresource->aspect references stencil");
            return false;
        } else if (subresource->layerBase + subresource->layerCount > tex->layerCount) {
            KOBALT_PRINTF(DebugSeverity::Error, texture, "texture has %u layers, but subresource references base mip %u with count %u", tex->layerCount, subresource->layerBase, subresource->layerCount);
            return false;
        } else if (subresource->mipBase + subresource->mipCount > tex->mipCount) {
            KOBALT_PRINTF(DebugSeverity::Error, texture, "texture has %u mips, but subresource references base mip %u with count %u", tex->mipCount, subresource->mipBase, subresource->mipCount);
            return false;
        }

        subrsrc = *subresource;
    } else {
        subrsrc.aspect = maxViewAspect;
        subrsrc.layerBase = 0;
        subrsrc.layerCount = tex->layerCount;
        subrsrc.mipBase = 0;
        subrsrc.mipCount = tex->mipCount;
    }

    VkImageViewCreateInfo createInfo = {};
    createInfo.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
    createInfo.image = tex->vkImage;
    createInfo.viewType = internal::textureDimensionsToVkImageViewType(dimensions);
    createInfo.format = internal::textureFormatToVkFormat(format);
    createInfo.components.r = internal::componentSwizzleToVkComponentSwizzle(map.r);
    createInfo.components.g = internal::componentSwizzleToVkComponentSwizzle(map.g);
    createInfo.components.b = internal::componentSwizzleToVkComponentSwizzle(map.b);
    createInfo.components.a = internal::componentSwizzleToVkComponentSwizzle(map.a);
    createInfo.subresourceRange.aspectMask = internal::textureAspectToVkImageAspectFlags(subrsrc.aspect);
    createInfo.subresourceRange.baseMipLevel = subrsrc.mipBase;
    createInfo.subresourceRange.levelCount = subrsrc.mipCount;
    createInfo.subresourceRange.baseArrayLayer = subrsrc.layerBase;
    createInfo.subresourceRange.layerCount = subrsrc.layerCount;

    if (tex->swapchain != nullptr) {
        internal::wsi::Swapchain_t* swp = reinterpret_cast<internal::wsi::Swapchain_t*>(tex->swapchain);
        size_t internalViewIndex = swp->internalBackbuffers[0].vkImageViews.size();
        for (size_t i = 0; i < swp->internalBackbuffers.size(); ++i) {
            createInfo.image = swp->internalBackbuffers[i].vkImage;

            VkImageView vkImageView;
            if (vkCreateImageView(dev->vkDevice, &createInfo, nullptr, &vkImageView) != VK_SUCCESS) {
                KOBALT_PRINTF(DebugSeverity::Error, device, "internal Vulkan error; failed to create image view for swapchain internal image %zu", i);
                for (size_t j = 0; j < i; ++i) {
                    vkDestroyImageView(dev->vkDevice, swp->internalBackbuffers[j].vkImageViews[internalViewIndex], nullptr);
                }

                return false;
            }

            swp->internalBackbuffers[i].vkImageViews.push_back(vkImageView);
        }

        internal::TextureView_t* v = new internal::TextureView_t(dev, tex->swapchain, tex, static_cast<uint32_t>(internalViewIndex), format, dimensions);
        view = &v->base.obj;
        swp->backbufferViews.push_back(v);
        return true;
    }

    VkImageView vkImageView;
    if (vkCreateImageView(dev->vkDevice, &createInfo, nullptr, &vkImageView) != VK_SUCCESS) {
        KOBALT_PRINT(DebugSeverity::Error, device, "internal Vulkan error; failed to create image view");
        return false;
    }

    view = &(new internal::TextureView_t(dev, tex, vkImageView, format, dimensions))->base.obj;
    return true;
}

bool createCommandList(CommandList& commandList, Device device, QueueType queueType, bool isSecondary) {
    if (device == nullptr) {
        KOBALT_PRINT(DebugSeverity::Error, nullptr, "device is null");
        return false;
    }

    internal::Device_t* dev = reinterpret_cast<internal::Device_t*>(device);

    uint32_t queueFamily;
    switch (queueType) {
        case QueueType::Graphics:
            queueFamily = dev->queues.graphicsFamily;
            break;
        case QueueType::Compute:
            queueFamily = dev->queues.computeFamily;
            break;
        case QueueType::Transfer:
            queueFamily = dev->queues.transferFamily;
            break;
        case QueueType::General:
            queueFamily = dev->queues.generalFamily;
            break;
        default:
            KOBALT_PRINTF(DebugSeverity::Error, device, "invalid queueType: %u", static_cast<uint32_t>(queueType));
            return false;
    }

    VkCommandPoolCreateInfo cmdPoolCI = {};
    cmdPoolCI.sType = VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO;
    cmdPoolCI.flags = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT;
    cmdPoolCI.queueFamilyIndex = queueFamily;

    VkCommandPool vkCmdPool;
    if (vkCreateCommandPool(dev->vkDevice, &cmdPoolCI, nullptr, &vkCmdPool) != VK_SUCCESS) {
        KOBALT_PRINT(DebugSeverity::Error, device, "internal Vulkan error; failed to create command pool");
        return false;
    }

    VkCommandBufferAllocateInfo cmdBufferAI = {};
    cmdBufferAI.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
    cmdBufferAI.commandPool = vkCmdPool;
    cmdBufferAI.level = isSecondary ? VK_COMMAND_BUFFER_LEVEL_SECONDARY : VK_COMMAND_BUFFER_LEVEL_PRIMARY;
    cmdBufferAI.commandBufferCount = 1;

    VkCommandBuffer vkCmdBuffer;
    if (vkAllocateCommandBuffers(dev->vkDevice, &cmdBufferAI, &vkCmdBuffer) != VK_SUCCESS) {
        vkDestroyCommandPool(dev->vkDevice, vkCmdPool, nullptr);
        KOBALT_PRINT(DebugSeverity::Error, device, "internal Vulkan error; failed to allocate command buffer");
        return false;
    }

    commandList = &(new internal::CommandList_t(dev, vkCmdBuffer, vkCmdPool, queueType))->base.obj;
    return true;
}

bool executeCommandList(CommandList commandList, QueueSync const* waitQueueSyncs, PipelineStage const* waitDstStages, uint32_t waitQueueSyncCount, QueueSync const* signalQueueSyncs, uint32_t signalQueueSyncCount, HostSync hostSync) {
    if (commandList == nullptr) {
        KOBALT_PRINT(DebugSeverity::Error, nullptr, "commandList is null");
        return false;
    }

    std::vector<VkSemaphore> waits(waitQueueSyncCount);
    std::vector<VkPipelineStageFlags> waitStages(waitQueueSyncCount);
    VkSemaphore const* pWaits = nullptr;
    VkPipelineStageFlags const* pStages = nullptr;
    if (waitQueueSyncs != nullptr) {
        if (waitDstStages == nullptr) {
            KOBALT_PRINT(DebugSeverity::Error, commandList, "if waitQueueSyncs is not null and waitQueueSyncCount is not 0, waitDstStages must not be null");
            return false;
        }

        for (uint32_t i = 0; i < waitQueueSyncCount; ++i) {
            if (waitQueueSyncs[i] == nullptr) {
                KOBALT_PRINTF(DebugSeverity::Error, commandList, "waitQueueSyncs[%u] is null", i);
                return false;
            }

            waits[i] = reinterpret_cast<internal::QueueSync_t*>(waitQueueSyncs[i])->vkSemaphore;
            waitStages[i] = internal::pipelineStageToVkPipelineStageFlags(waitDstStages[i]);
        }

        pWaits = waits.data();
        pStages = waitStages.data();
    }

    std::vector<VkSemaphore> signals(signalQueueSyncCount);
    VkSemaphore const* pSignals = nullptr;
    if (signalQueueSyncs != nullptr) {
        for (uint32_t i = 0; i < signalQueueSyncCount; ++i) {
            if (signalQueueSyncs[i] == nullptr) {
                KOBALT_PRINTF(DebugSeverity::Error, commandList, "signalQueueSyncs[%u] is null", i);
                return false;
            }

            signals[i] = reinterpret_cast<internal::QueueSync_t*>(signalQueueSyncs[i])->vkSemaphore;
        }

        pSignals = signals.data();
    }

    internal::CommandList_t* cmdList = reinterpret_cast<internal::CommandList_t*>(commandList);
    internal::Device_t* dev = reinterpret_cast<internal::Device_t*>(commandList->device);

    VkSubmitInfo submitInfo = {};
    submitInfo.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
    submitInfo.waitSemaphoreCount = pWaits != nullptr ? waitQueueSyncCount : 0;
    submitInfo.pWaitSemaphores = pWaits;
    submitInfo.pWaitDstStageMask = pStages;
    submitInfo.commandBufferCount = 1;
    submitInfo.pCommandBuffers = &cmdList->vkCmdBuffer;
    submitInfo.signalSemaphoreCount = pSignals != nullptr ? signalQueueSyncCount : 0;
    submitInfo.pSignalSemaphores = pSignals;

    VkQueue vkQueue;
    switch (cmdList->queueType) {
        case QueueType::Graphics:
            vkQueue = dev->queues.graphics;
            break;
        case QueueType::Compute:
            vkQueue = dev->queues.compute;
            break;
        case QueueType::Transfer:
            vkQueue = dev->queues.transfer;
            break;
        case QueueType::General:
            vkQueue = dev->queues.general;
            break;
        default:
            KOBALT_PRINTF(DebugSeverity::Error, commandList, "internal error; invalid command list queue type: %u", static_cast<uint32_t>(cmdList->queueType))
            return false;
    }

    VkFence vkFence = nullptr;
    if (hostSync != nullptr) {
        internal::HostSync_t* hs = reinterpret_cast<internal::HostSync_t*>(hostSync);
        vkFence = hs->vkFence;
    }

    return vkQueueSubmit(vkQueue, 1, &submitInfo, vkFence) == VK_SUCCESS;
}

bool resetCommandList(CommandList commandList) {
    if (commandList == nullptr) {
        KOBALT_PRINT(DebugSeverity::Error, nullptr, "commandList is null");
        return false;
    }

    internal::CommandList_t* cmdList = reinterpret_cast<internal::CommandList_t*>(commandList);
    return vkResetCommandBuffer(cmdList->vkCmdBuffer, 0) == VK_SUCCESS;
}

bool beginRecordingCommandList(CommandList commandList) {
    if (commandList == nullptr) {
        KOBALT_PRINT(DebugSeverity::Error, nullptr, "commandList is null");
        return false;
    }

    internal::CommandList_t* cmdList = reinterpret_cast<internal::CommandList_t*>(commandList);

    VkCommandBufferBeginInfo beginInfo = {};
    beginInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;

    return vkBeginCommandBuffer(cmdList->vkCmdBuffer, &beginInfo) == VK_SUCCESS;
}

/* TODO: secondary command list recording */
bool beginRecordingSecondaryCommandList(CommandList commandList, RenderPass renderPass, uint32_t subpass, Framebuffer framebuffer, bool encapsulated) {
    if (commandList == nullptr) {
        KOBALT_PRINT(DebugSeverity::Error, nullptr, "commandList is null");
        return false;
    }

    internal::CommandList_t* cmdList = reinterpret_cast<internal::CommandList_t*>(commandList);

    VkCommandBufferInheritanceInfo inheritance = {};
    inheritance.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_INHERITANCE_INFO;
    inheritance.renderPass = nullptr;

    VkCommandBufferBeginInfo beginInfo = {};
    beginInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
    beginInfo.flags = encapsulated ? VK_COMMAND_BUFFER_USAGE_RENDER_PASS_CONTINUE_BIT : 0;
    // beginInfo.pInheritanceInfo = nullptr;
    
    /* ... */

    return false;
}

bool recordSecondaryCommands(CommandList commandList, CommandList secondaryList) {
    if (commandList == nullptr) {
        KOBALT_PRINT(DebugSeverity::Error, nullptr, "commandList is null");
        return false;
    }

    if (secondaryList == nullptr) {
        KOBALT_PRINT(DebugSeverity::Error, commandList, "secondaryList is null");
        return false;
    }

    internal::CommandList_t* cmdList = reinterpret_cast<internal::CommandList_t*>(commandList);
    internal::CommandList_t* sCmdList = reinterpret_cast<internal::CommandList_t*>(secondaryList);

    vkCmdExecuteCommands(cmdList->vkCmdBuffer, 1, &sCmdList->vkCmdBuffer);
    return true;
}

bool endRecordingCommandList(CommandList commandList) {
    if (commandList == nullptr) {
        KOBALT_PRINT(DebugSeverity::Error, nullptr, "commandList is null");
        return false;
    }

    internal::CommandList_t* cmdList = reinterpret_cast<internal::CommandList_t*>(commandList);
    cmdList->finalizeBarrier();

    return vkEndCommandBuffer(cmdList->vkCmdBuffer) == VK_SUCCESS;
}

bool waitForQueue(Device device, QueueType queueType) {
    if (device == nullptr) {
        KOBALT_PRINT(DebugSeverity::Error, nullptr, "device is null");
        return false;
    }

    internal::Device_t* dev = reinterpret_cast<internal::Device_t*>(device);

    VkQueue vkQueue;
    switch (queueType) {
        case QueueType::Graphics:
            vkQueue = dev->queues.graphics;
            break;
        case QueueType::Compute:
            vkQueue = dev->queues.compute;
            break;
        case QueueType::Transfer:
            vkQueue = dev->queues.transfer;
            break;
        case QueueType::General:
            vkQueue = dev->queues.general;
            break;
        default:
            KOBALT_PRINTF(DebugSeverity::Error, device, "queueType has invalid value: %u", static_cast<uint32_t>(queueType));
            return false;
    }

    return vkQueueWaitIdle(vkQueue) == VK_SUCCESS;
}

bool createQueueSync(QueueSync& sync, Device device) {
    if (device == nullptr) {
        KOBALT_PRINT(DebugSeverity::Error, nullptr, "device is null");
        return false;
    }

    internal::Device_t* dev = reinterpret_cast<internal::Device_t*>(device);

    VkSemaphore vkSemaphore;
    if (!prism::vk::createSemaphore(dev->vkDevice, nullptr, vkSemaphore)) {
        KOBALT_PRINT(DebugSeverity::Error, device, "internal Vulkan error; failed to create semaphore");
        return false;
    }

    sync = &(new internal::QueueSync_t(dev, vkSemaphore))->base.obj;
    return true;
}

bool createHostSync(HostSync& sync, Device device, bool signaled) {
    if (device == nullptr) {
        KOBALT_PRINT(DebugSeverity::Error, nullptr, "device is null");
        return false;
    }

    internal::Device_t* dev = reinterpret_cast<internal::Device_t*>(device);

    VkFence vkFence;
    if (!prism::vk::createFence(dev->vkDevice, signaled, nullptr, vkFence)) {
        KOBALT_PRINT(DebugSeverity::Error, device, "internal Vulkan error; failed to create fence");
        return false;
    }

    sync = &(new internal::HostSync_t(dev, vkFence))->base.obj;
    return true;
}

bool waitForHostSync(HostSync sync, uint64_t timeout) {
    if (sync == nullptr) {
        KOBALT_PRINT(DebugSeverity::Error, nullptr, "sync is null");
        return false;
    }

    internal::Device_t* dev = reinterpret_cast<internal::Device_t*>(sync->device);
    internal::HostSync_t* hs = reinterpret_cast<internal::HostSync_t*>(sync);
    return vkWaitForFences(dev->vkDevice, 1, &hs->vkFence, VK_TRUE, timeout) == VK_SUCCESS;
}

bool resetHostSync(HostSync sync) {
    if (sync == nullptr) {
        KOBALT_PRINT(DebugSeverity::Error, nullptr, "sync is null");
        return false;
    }

    internal::Device_t* dev = reinterpret_cast<internal::Device_t*>(sync->device);
    internal::HostSync_t* hs = reinterpret_cast<internal::HostSync_t*>(sync);
    return vkResetFences(dev->vkDevice, 1, &hs->vkFence) == VK_SUCCESS;
}

bool getHostSyncSignal(HostSync sync) {
    if (sync == nullptr) {
        KOBALT_PRINT(DebugSeverity::Error, nullptr, "sync is null");
        return false;
    }

    internal::Device_t* dev = reinterpret_cast<internal::Device_t*>(sync->device);
    internal::HostSync_t* hs = reinterpret_cast<internal::HostSync_t*>(sync);
    return vkGetFenceStatus(dev->vkDevice, hs->vkFence) == VK_SUCCESS;
}

bool createStageSync(StageSync& sync, Device device) {
    if (device == nullptr) {
        KOBALT_PRINT(DebugSeverity::Error, nullptr, "device is null");
        return false;
    }

    internal::Device_t* dev = reinterpret_cast<internal::Device_t*>(device);

    VkEventCreateInfo createInfo = {};
    createInfo.sType = VK_STRUCTURE_TYPE_EVENT_CREATE_INFO;

    VkEvent vkEvent;
    if (vkCreateEvent(dev->vkDevice, &createInfo, nullptr, &vkEvent) != VK_SUCCESS) {
        KOBALT_PRINT(DebugSeverity::Error, device, "internal Vulkan error; failed to create event");
        return false;
    }

    sync = &(new internal::StageSync_t(dev, vkEvent))->base.obj;
    return true;
}

bool setStageSyncSignal(StageSync sync, bool signal) {
    if (sync == nullptr) {
        KOBALT_PRINT(DebugSeverity::Error, nullptr, "sync is null");
        return false;
    }

    internal::Device_t* dev = reinterpret_cast<internal::Device_t*>(sync->device);
    internal::StageSync_t* ss = reinterpret_cast<internal::StageSync_t*>(sync);
    if (signal) {
        if (vkSetEvent(dev->vkDevice, ss->vkEvent) == VK_SUCCESS) {
            return true;
        }

        KOBALT_PRINT(DebugSeverity::Error, sync->device, "internal Vulkan error; failed to set event");
        return false;
    }

    return vkResetEvent(dev->vkDevice, ss->vkEvent) == VK_SUCCESS;
}

bool getStageSyncSignal(StageSync sync) {
    if (sync == nullptr) {
        KOBALT_PRINT(DebugSeverity::Error, nullptr, "sync is null");
        return false;
    }

    internal::Device_t* dev = reinterpret_cast<internal::Device_t*>(sync->device);
    internal::StageSync_t* ss = reinterpret_cast<internal::StageSync_t*>(sync);
    return vkGetEventStatus(dev->vkDevice, ss->vkEvent) == VK_EVENT_SET;
}

namespace cmd {

bool setStageSync(CommandList commandList, StageSync sync, PipelineStage stage, bool signaled) {
    if (commandList == nullptr) {
        KOBALT_PRINT(DebugSeverity::Error, nullptr, "commandList is null");
        return false;
    }

    internal::CommandList_t* cmdList = reinterpret_cast<internal::CommandList_t*>(commandList);
    cmdList->finalizeBarrier();

    if (sync == nullptr) {
        KOBALT_PRINT(DebugSeverity::Error, commandList, "sync is null");
        return false;
    }

    if (internal::pipelineStageToVkPipelineStageFlags(stage) == VK_PIPELINE_STAGE_FLAG_BITS_MAX_ENUM) {
        KOBALT_PRINTF(DebugSeverity::Error, nullptr, "stage has invalid value: %u", static_cast<uint32_t>(stage));
        return false;
    }

    internal::StageSync_t* ss = reinterpret_cast<internal::StageSync_t*>(sync);
    if (signaled) {
        vkCmdSetEvent(cmdList->vkCmdBuffer, ss->vkEvent, internal::pipelineStageToVkPipelineStageFlags(stage));
    } else {
        vkCmdResetEvent(cmdList->vkCmdBuffer, ss->vkEvent, internal::pipelineStageToVkPipelineStageFlags(stage));
    }

    return true;
}

bool waitStageSync(CommandList commandList, StageSync sync, PipelineStage srcStage, PipelineStage dstStage) {
    if (commandList == nullptr) {
        KOBALT_PRINT(DebugSeverity::Error, nullptr, "commandList is null");
        return false;
    }

    internal::CommandList_t* cmdList = reinterpret_cast<internal::CommandList_t*>(commandList);
    cmdList->finalizeBarrier();

    if (sync == nullptr) {
        KOBALT_PRINT(DebugSeverity::Error, commandList, "sync is null");
        return false;
    }

    if (internal::pipelineStageToVkPipelineStageFlags(srcStage) == VK_PIPELINE_STAGE_FLAG_BITS_MAX_ENUM) {
        KOBALT_PRINTF(DebugSeverity::Error, nullptr, "srcStage has invalid value: %u", static_cast<uint32_t>(srcStage));
        return false;
    }

    if (internal::pipelineStageToVkPipelineStageFlags(dstStage) == VK_PIPELINE_STAGE_FLAG_BITS_MAX_ENUM) {
        KOBALT_PRINTF(DebugSeverity::Error, nullptr, "dstStage has invalid value: %u", static_cast<uint32_t>(dstStage));
        return false;
    }

    internal::StageSync_t* ss = reinterpret_cast<internal::StageSync_t*>(sync);

    cmdList->barrier.stageSync = ss;
    cmdList->barrier.exec.srcStage = srcStage;
    cmdList->barrier.exec.dstStage = dstStage;
    return true;
}

bool executionBarrier(CommandList commandList, PipelineStage srcStage, PipelineStage dstStage) {
    if (commandList == nullptr) {
        KOBALT_PRINT(DebugSeverity::Error, nullptr, "commandList is null");
        return false;
    }

    internal::CommandList_t* cmdList = reinterpret_cast<internal::CommandList_t*>(commandList);
    cmdList->finalizeBarrier();

    if (internal::pipelineStageToVkPipelineStageFlags(srcStage) == VK_PIPELINE_STAGE_FLAG_BITS_MAX_ENUM) {
        KOBALT_PRINTF(DebugSeverity::Error, nullptr, "srcStage has invalid value: %u", static_cast<uint32_t>(srcStage));
        return false;
    }

    if (internal::pipelineStageToVkPipelineStageFlags(dstStage) == VK_PIPELINE_STAGE_FLAG_BITS_MAX_ENUM) {
        KOBALT_PRINTF(DebugSeverity::Error, nullptr, "dstStage has invalid value: %u", static_cast<uint32_t>(dstStage));
        return false;
    }

    cmdList->barrier.exec.srcStage = srcStage;
    cmdList->barrier.exec.dstStage = dstStage;
    cmdList->wasLastCmdBarrier = true;
    return true;
}

bool globalResourceBarrier(CommandList commandList, ResourceAccess srcAccess, ResourceAccess dstAccess) {
    if (commandList == nullptr) {
        KOBALT_PRINT(DebugSeverity::Error, nullptr, "commandList is null");
        return false;
    }

    if (internal::resourceAccessToVkAccessFlags(srcAccess) == VK_ACCESS_FLAG_BITS_MAX_ENUM) {
        KOBALT_PRINTF(DebugSeverity::Error, nullptr, "srcAccess has invalid value: %u", static_cast<uint32_t>(srcAccess));
        return false;
    }

    if (internal::resourceAccessToVkAccessFlags(dstAccess) == VK_ACCESS_FLAG_BITS_MAX_ENUM) {
        KOBALT_PRINTF(DebugSeverity::Error, nullptr, "dstAccess has invalid value: %u", static_cast<uint32_t>(dstAccess));
        return false;
    }

    internal::CommandList_t* cmdList = reinterpret_cast<internal::CommandList_t*>(commandList);
    if (!cmdList->wasLastCmdBarrier) {
        KOBALT_PRINT(DebugSeverity::Error, nullptr, "resource barriers require a prior execution barrier or stage synchronization wait");
        return false;
    }

    internal::GlobalResourceBarrier barrier = {};
    barrier.srcAccess = srcAccess;
    barrier.dstAccess = dstAccess;

    cmdList->barrier.globals.push_back(barrier);
    return true;
}

bool textureBarrier(CommandList commandList, ResourceAccess srcAccess, ResourceAccess dstAccess, QueueTransfer srcQueue, QueueTransfer dstQueue, TextureLayout oldLayout, TextureLayout newLayout, Texture texture, TextureSubresource const* subresource) {
    if (commandList == nullptr) {
        KOBALT_PRINT(DebugSeverity::Error, nullptr, "commandList is null");
        return false;
    }

    if (internal::resourceAccessToVkAccessFlags(srcAccess) == VK_ACCESS_FLAG_BITS_MAX_ENUM) {
        KOBALT_PRINTF(DebugSeverity::Error, nullptr, "srcAccess has invalid value: %u", static_cast<uint32_t>(srcAccess));
        return false;
    }

    if (internal::resourceAccessToVkAccessFlags(dstAccess) == VK_ACCESS_FLAG_BITS_MAX_ENUM) {
        KOBALT_PRINTF(DebugSeverity::Error, nullptr, "dstAccess has invalid value: %u", static_cast<uint32_t>(dstAccess));
        return false;
    }

    switch (srcQueue) {
        case QueueTransfer::Identity:
        case QueueTransfer::Graphics:   case QueueTransfer::Compute:
        case QueueTransfer::Transfer:   case QueueTransfer::General:
            break;
        default:
            KOBALT_PRINTF(DebugSeverity::Error, nullptr, "srcQueue has invalid value: %u", static_cast<uint32_t>(srcQueue));
            return false;
    }

    switch (dstQueue) {
        case QueueTransfer::Identity:
        case QueueTransfer::Graphics:   case QueueTransfer::Compute:
        case QueueTransfer::Transfer:   case QueueTransfer::General:
            break;
        default:
            KOBALT_PRINTF(DebugSeverity::Error, nullptr, "dstQueue has invalid value: %u", static_cast<uint32_t>(dstQueue));
            return false;
    }

    if (internal::textureLayoutToVkImageLayout(oldLayout) == VK_IMAGE_LAYOUT_MAX_ENUM) {
        KOBALT_PRINTF(DebugSeverity::Error, nullptr, "oldLayout has invalid value: %u", static_cast<uint32_t>(oldLayout));
        return false;
    }

    if (internal::textureLayoutToVkImageLayout(newLayout) == VK_IMAGE_LAYOUT_MAX_ENUM) {
        KOBALT_PRINTF(DebugSeverity::Error, nullptr, "newLayout has invalid value: %u", static_cast<uint32_t>(newLayout));
        return false;
    }

    if (texture == nullptr) {
        KOBALT_PRINT(DebugSeverity::Error, nullptr, "texture is null");
        return false;
    }

    internal::CommandList_t* cmdList = reinterpret_cast<internal::CommandList_t*>(commandList);
    if (!cmdList->wasLastCmdBarrier) {
        KOBALT_PRINT(DebugSeverity::Error, nullptr, "resource barriers require a prior execution barrier or stage synchronization wait");
        return false;
    }

    internal::Texture_t* tex = reinterpret_cast<internal::Texture_t*>(texture);
    internal::TextureBarrier barrier = {};
    barrier.srcAccess = srcAccess;
    barrier.dstAccess = dstAccess;
    barrier.srcQueue = srcQueue;
    barrier.dstQueue = dstQueue;
    barrier.oldLayout = oldLayout;
    barrier.newLayout = newLayout;
    barrier.texture = tex;

    TextureAspect maxAspect = internal::maximumTextureAspectFromTextureFormat(tex->format);
    if (subresource != nullptr) {
        if ((subresource->aspect & (TextureAspect::Color | TextureAspect::Depth | TextureAspect::Stencil)) != subresource->aspect) {
            KOBALT_PRINTF(DebugSeverity::Error, texture, "subresource->aspect has invalid value: %u", static_cast<uint32_t>(subresource->aspect));
            return false;
        } else if ((subresource->aspect & TextureAspect::Color) == TextureAspect::Color && (maxAspect & TextureAspect::Color) != TextureAspect::Color) {
            KOBALT_PRINT(DebugSeverity::Error, texture, "texture format has no color aspect, but subresource->aspect references color");
            return false;
        } else if ((subresource->aspect & TextureAspect::Depth) == TextureAspect::Depth && (maxAspect & TextureAspect::Depth) != TextureAspect::Depth) {
            KOBALT_PRINT(DebugSeverity::Error, texture, "texture format has no depth aspect, but subresource->aspect references depth");
            return false;
        } else if ((subresource->aspect & TextureAspect::Stencil) == TextureAspect::Stencil && (maxAspect & TextureAspect::Stencil) != TextureAspect::Stencil) {
            KOBALT_PRINT(DebugSeverity::Error, texture, "texture format has no stencil aspect, but subresource->aspect references stencil");
            return false;
        } else if (subresource->layerBase + subresource->layerCount > tex->layerCount) {
            KOBALT_PRINTF(DebugSeverity::Error, texture, "texture has %u layers, but subresource references base mip %u with count %u", tex->layerCount, subresource->layerBase, subresource->layerCount);
            return false;
        } else if (subresource->mipBase + subresource->mipCount > tex->mipCount) {
            KOBALT_PRINTF(DebugSeverity::Error, texture, "texture has %u mips, but subresource references base mip %u with count %u", tex->mipCount, subresource->mipBase, subresource->mipCount);
            return false;
        }

        barrier.subresource = *subresource;
    } else {
        barrier.subresource.aspect = maxAspect;
        barrier.subresource.layerBase = 0;
        barrier.subresource.layerCount = tex->layerCount;
        barrier.subresource.mipBase = 0;
        barrier.subresource.mipCount = tex->mipCount;
    }

    cmdList->barrier.textures.push_back(barrier);
    return true;
}

bool bufferBarrier(CommandList commandList, ResourceAccess srcAccess, ResourceAccess dstAccess, QueueTransfer srcQueue, QueueTransfer dstQueue, Buffer buffer, uint64_t offset, uint64_t size);

bool beginDynamicRenderPass(CommandList commandList, Rectangle renderArea, uint32_t layerCount, uint32_t viewMask, DynamicRenderAttachment const* renderTargets, uint32_t renderTargetCount, DynamicRenderAttachment const* depthTarget, DynamicRenderAttachment const* stencilTarget) {
    if (commandList == nullptr) {
        KOBALT_PRINT(DebugSeverity::Error, nullptr, "command list is null");
        return false;
    }

    internal::CommandList_t* cmdList = reinterpret_cast<internal::CommandList_t*>(commandList);
    cmdList->finalizeBarrier();

    if (cmdList->renderPass.renderPass != nullptr || cmdList->renderPass.dynamic) {
        KOBALT_PRINT(DebugSeverity::Error, commandList, "render pass already started");
        return false;
    }

    internal::Device_t* dev = cmdList->device;
    if (!dev->enabledSupport.dynamicRenderPass) {
        KOBALT_PRINT(DebugSeverity::Warning, commandList->device, "device support for dynamic render passes was not enabled on creation");
        return false;
    }

    if (dev->symbols.vkCmdBeginRenderingKHR == nullptr) {
        KOBALT_PRINT(DebugSeverity::Error, commandList->device, "device does not support dynamic render passes");
        return false;
    }

    std::vector<VkRenderingAttachmentInfoKHR> colorAIs(renderTargetCount);
    for (uint32_t i = 0; i < renderTargetCount; ++i) {
        internal::TextureView_t* view = reinterpret_cast<internal::TextureView_t*>(renderTargets[i].view);
        if ((internal::maximumTextureAspectFromTextureFormat(view->format) & TextureAspect::Color) != TextureAspect::Color) {
            KOBALT_PRINTF(DebugSeverity::Error, &view->base.obj, "renderTargets[%u].view does not support color aspect", i);
            return false;
        }

        if (internal::textureLayoutToVkImageLayout(renderTargets[i].layout) == VK_IMAGE_LAYOUT_MAX_ENUM) {
            KOBALT_PRINTF(DebugSeverity::Error, commandList, "renderTargets[%u].layout has invalid value: %u", i, static_cast<uint32_t>(renderTargets[i].layout));
            return false;
        }

        colorAIs[i] = {};
        colorAIs[i].sType = VK_STRUCTURE_TYPE_RENDERING_ATTACHMENT_INFO_KHR;
        if (view->swapchain != nullptr) {
            internal::wsi::Swapchain_t* swp = reinterpret_cast<internal::wsi::Swapchain_t*>(view->swapchain);
            colorAIs[i].imageView = swp->internalBackbuffers[swp->currentIndex].vkImageViews[view->swapchainInternalViewIndex];
        } else {
            colorAIs[i].imageView = view->vkImageView;
        }

        colorAIs[i].imageLayout = internal::textureLayoutToVkImageLayout(renderTargets[i].layout);
        colorAIs[i].loadOp = internal::renderAttachmentLoadOpToVkAttachmentLoadOp(renderTargets[i].loadOp);
        colorAIs[i].storeOp = internal::renderAttachmentStoreOpToVkAttachmentStoreOp(renderTargets[i].storeOp);
        colorAIs[i].clearValue.color.float32[0] = renderTargets[i].clearValue.color.rgbaFloat[0];
        colorAIs[i].clearValue.color.float32[1] = renderTargets[i].clearValue.color.rgbaFloat[1];
        colorAIs[i].clearValue.color.float32[2] = renderTargets[i].clearValue.color.rgbaFloat[2];
        colorAIs[i].clearValue.color.float32[3] = renderTargets[i].clearValue.color.rgbaFloat[3];
    }

    VkRenderingInfoKHR renderingInfo = {};
    renderingInfo.sType = VK_STRUCTURE_TYPE_RENDERING_INFO_KHR;
    renderingInfo.renderArea.offset.x = renderArea.x;
    renderingInfo.renderArea.offset.y = renderArea.y;
    renderingInfo.renderArea.extent.width = renderArea.width;
    renderingInfo.renderArea.extent.height = renderArea.height;
    renderingInfo.layerCount = layerCount;
    renderingInfo.viewMask = viewMask;
    renderingInfo.colorAttachmentCount = renderTargets != nullptr ? renderTargetCount : 0;
    renderingInfo.pColorAttachments = renderTargets != nullptr ? colorAIs.data() : nullptr;
    renderingInfo.pDepthAttachment = nullptr;
    renderingInfo.pStencilAttachment = nullptr;

    VkRenderingAttachmentInfoKHR depthAI = {};
    if (depthTarget != nullptr) {
        internal::TextureView_t* view = reinterpret_cast<internal::TextureView_t*>(depthTarget->view);
        if ((internal::maximumTextureAspectFromTextureFormat(view->format) & TextureAspect::Depth) != TextureAspect::Depth) {
            KOBALT_PRINT(DebugSeverity::Error, &view->base.obj, "depthTarget->view does not support depth aspect");
            return false;
        }

        depthAI.sType = VK_STRUCTURE_TYPE_RENDERING_ATTACHMENT_INFO_KHR;
        depthAI.imageView = view->vkImageView;
        depthAI.imageLayout = internal::textureLayoutToVkImageLayout(depthTarget->layout);
        depthAI.loadOp = internal::renderAttachmentLoadOpToVkAttachmentLoadOp(depthTarget->loadOp);
        depthAI.storeOp = internal::renderAttachmentStoreOpToVkAttachmentStoreOp(depthTarget->storeOp);
        depthAI.clearValue.depthStencil.depth = depthTarget->clearValue.depthStencil.depth;

        renderingInfo.pDepthAttachment = &depthAI;
    }

    VkRenderingAttachmentInfoKHR stencilAI = {};
    if (stencilTarget != nullptr) {
        internal::TextureView_t* view = reinterpret_cast<internal::TextureView_t*>(stencilTarget->view);
        if ((internal::maximumTextureAspectFromTextureFormat(view->format) & TextureAspect::Stencil) != TextureAspect::Stencil) {
            KOBALT_PRINT(DebugSeverity::Error, &view->base.obj, "stencilTarget->view does not support stencil aspect");
            return false;
        }

        stencilAI.sType = VK_STRUCTURE_TYPE_RENDERING_ATTACHMENT_INFO_KHR;
        stencilAI.imageView = view->vkImageView;
        stencilAI.imageLayout = internal::textureLayoutToVkImageLayout(stencilTarget->layout);
        stencilAI.loadOp = internal::renderAttachmentLoadOpToVkAttachmentLoadOp(stencilTarget->loadOp);
        stencilAI.storeOp = internal::renderAttachmentStoreOpToVkAttachmentStoreOp(stencilTarget->storeOp);
        stencilAI.clearValue.depthStencil.stencil = stencilTarget->clearValue.depthStencil.stencil;

        renderingInfo.pStencilAttachment = &stencilAI;
    }

    dev->symbols.vkCmdBeginRenderingKHR(cmdList->vkCmdBuffer, &renderingInfo);
    cmdList->renderPass.renderPass = nullptr;
    cmdList->renderPass.dynamic = true;
    cmdList->renderPass.renderTargets.assign(renderTargets, renderTargets + renderTargetCount);
    if (depthTarget == nullptr) {
        cmdList->renderPass.depthTarget.view = nullptr;
    } else {
        cmdList->renderPass.depthTarget = *depthTarget;
    }

    if (stencilTarget == nullptr) {
        cmdList->renderPass.stencilTarget.view = nullptr;
    } else {
        cmdList->renderPass.stencilTarget = *stencilTarget;
    }

    return true;
}


bool endRenderPass(CommandList commandList) {
    if (commandList == nullptr) {
        KOBALT_PRINT(DebugSeverity::Error, nullptr, "command list is null");
        return false;
    }

    internal::CommandList_t* cmdList = reinterpret_cast<internal::CommandList_t*>(commandList);
    internal::Device_t* dev = cmdList->device;

    if (cmdList->renderPass.renderPass != nullptr) {
        /* TODO: */
        return false;
    } else if (cmdList->renderPass.dynamic) {
        dev->symbols.vkCmdEndRenderingKHR(cmdList->vkCmdBuffer);
        cmdList->renderPass.clear();
        return true;
    }

    KOBALT_PRINT(DebugSeverity::Error, commandList, "no render pass started");
    return false;
}

} /* namespace cmd */

} /* namespace kobalt */

#endif /* #ifdef KOBALT_IMPL */

#endif /* #ifndef KOBALT_H */
