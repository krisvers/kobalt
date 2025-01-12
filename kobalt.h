#ifndef KOBALT_H
#define KOBALT_H

#include <cstdint>

#ifdef KOBALT_GLFW
#include <GLFW/glfw3.h>
#endif /* #ifdef KOBALT_GLFW */

#define KOBALT_HANDLE(handle_) typedef struct ::kobalt::Object_t* handle_

#define KOBALT_ENUM_BITMASK(T_)                                      \
inline T_ operator|(T_ a, T_ b) {                                    \
    return static_cast<T_>(static_cast<uint32_t>(a) | static_cast<uint32_t>(b)); \
}

namespace kobalt {

namespace internal {

enum class ObjectType {
    Device,
    Shader,
    VertexInputState,
    RasterizationState,
    GraphicsPipeline,
    Buffer,
    Texture,
    CommandList,
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
KOBALT_HANDLE(GraphicsPipeline);
KOBALT_HANDLE(Buffer);
KOBALT_HANDLE(Texture);
KOBALT_HANDLE(CommandList);

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
    Graphics,
    Compute,
    Transfer,
    All,
};

enum class FillMode {
    Fill,
    Wireframe,
    Points,
};

enum class CullMode {
    Back,
    Front,
    None,
};

enum class FrontFace {
    Clockwise,
    CounterClockwise,
};

struct DeviceSupport {
    bool dynamicRenderState;
    bool swapchain;
};

inline bool operator==(DeviceSupport const& a, DeviceSupport const& b) {
    return (a.dynamicRenderState == b.dynamicRenderState && a.swapchain == b.swapchain);
}

inline bool operator<(DeviceSupport const& a, DeviceSupport const& b) {
    return (a.dynamicRenderState < b.dynamicRenderState && a.swapchain < b.swapchain);
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

namespace wsi {

KOBALT_HANDLE(Swapchain);

#ifdef KOBALT_GLFW

namespace glfw {

bool createSwapchain(Swapchain& swapchain, Device device, GLFWwindow* window, uint32_t width, uint32_t height, TextureUsage usage, bool sRGBFormat, bool vsync);

} /* namespace glfw */

TextureFormat getSwapchainFormat(Swapchain swapchain);
uint32_t getSwapchainImageCount(Swapchain swapchain);

#endif /* #ifdef KOBALT_GLFW */

} /* namespace wsi */

bool createShaderSPIRV(Shader& shader, Device device, void const* data, uint32_t size);
bool createVertexInputState(VertexInputState& vertexInputState, Device device, Topology topology, VertexAttribute const* vertexAttributes, uint32_t vertexAttributeCount, VertexBinding const* vertexBindings, uint32_t vertexBindingCount);
bool createRasterizationState(RasterizationState& rasterizationState, Device device, FillMode fillMode, CullMode cullMode, FrontFace frontFace);
bool createGraphicsPipeline(GraphicsPipeline& graphicsPipeline, Device device, VertexInputState vertexInputState, RasterizationState rasterizationState, Shader vertexShader, Shader fragmentShader, Shader geometryShader);

bool createBuffer(Buffer& buffer, Device device, uint64_t size, MemoryLocation location, BufferUsage usage);
bool uploadBufferData(Buffer buffer, uint64_t offset, void const* data, uint64_t size);
bool downloadBufferData(Buffer buffer, uint64_t offset, void* data, uint64_t size);
bool mapBuffer(void*& mappedPointer, Buffer buffer, uint64_t offset, uint64_t size);
void unmapBuffer(Buffer buffer, void* pointer);

bool createTexture2D(Texture& texture, Device device, uint32_t width, uint32_t height, TextureFormat format, MemoryLocation location, TextureUsage usage);
bool createTexture3D(Texture& texture, Device device, uint32_t width, uint32_t height, uint32_t depth, TextureFormat format, MemoryLocation location, TextureUsage usage);
bool createTexture2DArray(Texture& texture, Device device, uint32_t width, uint32_t height, uint32_t layers, TextureFormat format, MemoryLocation location, TextureUsage usage);
bool createTexture3DArray(Texture& texture, Device device, uint32_t width, uint32_t height, uint32_t depth, TextureFormat format, uint32_t layers, MemoryLocation location, TextureUsage usage);

bool createCommandList(CommandList& commandList, Device device, CommandListSpecialization specialization);
bool createSecondaryCommandList(CommandList& commandList, Device dev);
bool executeCommandList(CommandList commandList);
bool waitForCommandList(CommandList commandList);
void resetCommandList(CommandList commandList);

namespace cmd {

bool beginRecording(CommandList commandList);
bool recordSecondaryCommands(CommandList commandList, CommandList secondaryList);
bool endRecording(CommandList commandList);

bool bindGraphicsPipeline(CommandList commandList, GraphicsPipeline pipeline);
bool bindVertexBuffer(CommandList commandList, uint32_t index, Buffer buffer, uint64_t offset);
bool bindIndexBuffer(CommandList commandList, Buffer buffer, uint64_t offset);

bool draw(CommandList commandList, uint32_t vertex, uint32_t count);
bool drawIndexed(CommandList commandList, uint32_t index, uint32_t count);

} /* namespace cmd */

} /* namespace kobalt */

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

namespace kobalt {

namespace internal {

inline VkPrimitiveTopology topologyToVkPrimitiveTopology(Topology topology) {
    switch (topology) {
        case Topology::PointList: return VK_PRIMITIVE_TOPOLOGY_POINT_LIST;
        case Topology::LineList: return VK_PRIMITIVE_TOPOLOGY_LINE_LIST;
        case Topology::LineStrip: return VK_PRIMITIVE_TOPOLOGY_LINE_STRIP;
        case Topology::TriangleList: return VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST;
        case Topology::TriangleStrip: return VK_PRIMITIVE_TOPOLOGY_TRIANGLE_STRIP;
        case Topology::TriangleFan: return VK_PRIMITIVE_TOPOLOGY_TRIANGLE_FAN;
        case Topology::PatchList: return VK_PRIMITIVE_TOPOLOGY_PATCH_LIST;
        default: return VK_PRIMITIVE_TOPOLOGY_MAX_ENUM;
    }
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

        default: return 0;
    }
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

struct ObjectBase_t {
    Object_t obj;
    char debugName[64];

    ObjectBase_t(ObjectType type) {
        obj.device = nullptr;
        obj.type = type;
        memset(debugName, 0, 64);
    }

    ObjectBase_t(Object_t* device, ObjectType type) {
        obj.device = device;
        obj.type = type;
        memset(debugName, 0, 64);
    }
};

struct ExtraDeviceFeatures {

};

struct Device_t {
    ObjectBase_t base;

    VkDevice vkDevice;
    VkPhysicalDevice vkPhysicalDevice;
    
    DeviceSupport enabledSupport;
    ExtraDeviceFeatures extraFeatures;

    Device_t(VkDevice vkDev, VkPhysicalDevice vkPhys, DeviceSupport const& support, ExtraDeviceFeatures const& extra) : base(ObjectType::Device), vkDevice(vkDev), vkPhysicalDevice(vkPhys), enabledSupport(support), extraFeatures(extra) {}

    ~Device_t() {
        vkDestroyDevice(vkDevice, nullptr);
    }
};

struct Shader_t {
    ObjectBase_t base;

    Device_t* device;
    VkShaderModule vkShaderModule;

    Shader_t(Device_t* device, VkShaderModule vkShaderModule) : base(reinterpret_cast<Object_t*>(device), internal::ObjectType::Shader), device(device), vkShaderModule(vkShaderModule) {}

    ~Shader_t() {
        vkDestroyShaderModule(device->vkDevice, vkShaderModule, nullptr);
    }
};

struct VertexInputState_t {
    ObjectBase_t base;

    std::vector<VkVertexInputAttributeDescription> attributes;
    std::vector<VkVertexInputBindingDescription> bindings;

    VkPipelineVertexInputStateCreateInfo vertexInputCI = {};
    VkPipelineInputAssemblyStateCreateInfo inputAssemblyCI = {};

    VertexInputState_t(Object_t* device, Topology topo, VertexAttribute const* attrs, uint32_t attrCount, VertexBinding const* binds, uint32_t bindCount) : base(device, ObjectType::VertexInputState) {
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
        "Swapchain"
    };

    ObjectBase_t* objBase = reinterpret_cast<ObjectBase_t*>(object);
    const char* sevStr = (severity >= DebugSeverity::Warning && severity <= DebugSeverity::Error) ? severityStrings[static_cast<uint32_t>(severity)] : "UNKNOWN";
    const char* typeStr = (object != nullptr) ? objectTypeStrings[static_cast<uint32_t>(objBase->obj.type)] : "NONE";

    if (object != nullptr) {
        printf("[kobalt] %s (Source Object: %s %p \"%s\"): %s\n", sevStr, typeStr, reinterpret_cast<void*>(objBase), objBase->debugName, message);
    } else {
        printf("[kobalt] %s: %s\n", sevStr, message);
    }
}

static DebugCallback debugCallback = defaultDebugMessenger;
#else
static DebugCallback debugCallback = nullptr;
#endif /* #ifdef KOBALT_DEFAULT_DEBUG_MESSENGER */

#define KOBALT_PRINT(sev_, obj_, msg_) if (internal::debugCallback != nullptr) { internal::debugCallback(msg_, sev_, obj_); }
#define KOBALT_PRINTF(sev_, obj_, fmt_, ...) if (internal::debugCallback != nullptr) { char buffer[512]; snprintf(buffer, 512, fmt_, __VA_ARGS__); internal::debugCallback(buffer, sev_, obj_); }

static VkInstance vkInstance = nullptr;
static std::vector<VkPhysicalDevice> vkPhysicalDevices;

static VkDebugUtilsMessengerEXT vkDebugMessengerEXT = nullptr;

static VkBool32 vkDebugMessengerCallback(VkDebugUtilsMessageSeverityFlagBitsEXT messageSeverity, VkDebugUtilsMessageTypeFlagsEXT messageTypes, const VkDebugUtilsMessengerCallbackDataEXT* pCallbackData, void* pUserData) {
    DebugSeverity severity = DebugSeverity::Error;
    if (messageSeverity == VK_DEBUG_UTILS_MESSAGE_SEVERITY_WARNING_BIT_EXT) {
        severity = DebugSeverity::Warning;
    }

    KOBALT_PRINTF(severity, nullptr, "%s", pCallbackData->pMessage);
    return VK_FALSE;
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
#elif APPLE
    flags |= VK_INSTANCE_CREATE_ENUMERATE_PORTABILITY_BIT_KHR;
    extensions.push_back("VK_KHR_portability_enumeration");
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
}

void destroy(Object_t* object) {
    if (object == nullptr) {
        KOBALT_PRINT(DebugSeverity::Error, nullptr, "cannot destroy null object");
        return;
    }

    switch (object->type) {
        case internal::ObjectType::Device:
            delete reinterpret_cast<internal::Device_t*>(object);
            return;
        case internal::ObjectType::Shader:
            delete reinterpret_cast<internal::Shader_t*>(object);
            return;
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
            adapterInfo.support.dynamicRenderState = true;
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
            availableSupport.dynamicRenderState = true;
        } else if (strcmp(e.extensionName, VK_KHR_SWAPCHAIN_EXTENSION_NAME) == 0) {
            availableSupport.swapchain = true;
        }
    }

    if (availableSupport < support) {
        KOBALT_PRINT(DebugSeverity::Error, nullptr, "device does not support requested features");
        return false;
    }

    std::vector<prism::vk::Extension> extensions;
    if (support.dynamicRenderState) {
        extensions.push_back(VK_KHR_DYNAMIC_RENDERING_EXTENSION_NAME);
    }

    if (support.swapchain) {
        extensions.push_back(VK_KHR_SWAPCHAIN_EXTENSION_NAME);
    }

    std::vector<VkQueueFamilyProperties> queueFamilyProperties;
    prism::vk::getPhysicalDeviceQueueFamilyProperties(vkPhysical, queueFamilyProperties);

    uint32_t bestGeneral = 0;
    uint32_t generalScore = 0;
    uint32_t bestGraphics = 0;
    uint32_t graphicsExtra = UINT32_MAX;
    uint32_t bestCompute = 0;
    uint32_t computeExtra = UINT32_MAX;
    uint32_t bestTransfer = 0;
    uint32_t transferExtra = UINT32_MAX;

    for (uint32_t i = 0; i < queueFamilyProperties.size(); ++i) {
        VkQueueFamilyProperties const& p = queueFamilyProperties[i];
        uint32_t extra = 0;
        for (uint32_t i = 0; i < 32; ++i) {
            if (p.queueFlags & (1 << i)) {
                ++extra;
            }
        }

        if (extra > generalScore) {
            bestGeneral = i;
            generalScore = extra;
        }

        if (p.queueFlags & VK_QUEUE_GRAPHICS_BIT && extra < graphicsExtra) {
            bestGraphics = i;
            graphicsExtra = extra;
        }

        if (p.queueFlags & VK_QUEUE_COMPUTE_BIT && extra < computeExtra) {
            bestCompute = i;
            computeExtra = extra;
        }

        if (p.queueFlags & VK_QUEUE_TRANSFER_BIT && extra < transferExtra) {
            bestTransfer = i;
            transferExtra = extra;
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

    VkPhysicalDeviceDynamicRenderingFeaturesKHR drFeatures = {};
    drFeatures.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_DYNAMIC_RENDERING_FEATURES_KHR;
    drFeatures.dynamicRendering = support.dynamicRenderState;

    VkDevice vkDevice;
    if (!prism::vk::createDevice(vkPhysical, queueCIs, extensions, &drFeatures, nullptr, nullptr, vkDevice)) {
        KOBALT_PRINT(DebugSeverity::Error, nullptr, "internal Vulkan error: device creation");
        return false;
    }

    internal::Device_t* dev = new internal::Device_t(vkDevice, vkPhysical, support, extraSupport);
    device = &dev->base.obj;
    return true;
}

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
                if (size + vertexAttributes[i].offset > vertexBindings[i].size) {
                    KOBALT_PRINTF(DebugSeverity::Error, device, "vertexAttributes[%u].offset has invalid value: %u; attributes must stay within size bounds of their binding (this attribute has a size of %u)", i, vertexAttributes[i].offset, size);
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

} /* namespace kobalt */

#endif /* #ifdef KOBALT_IMPL */

#endif /* #ifndef KOBALT_H */
