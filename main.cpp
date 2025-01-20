#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>

#define KOBALT_IMPL
#define KOBALT_GLFW
#define KOBALT_DEFAULT_DEBUG_MESSENGER
#include "kobalt.h"

#include <cassert>
#include <iostream>

void formatMemorySize(uint64_t& size, char& suffix, bool decimal) {
    char suffixes[] = { 'B', 'K', 'M', 'G', 'T' };
    uint32_t index = 0;
    uint32_t div = decimal ? 1000 : 1024;

    while (size >= div && index < 4) {
        size /= div;
        ++index;
    }

    suffix = suffixes[index];
}

bool loadShader(kobalt::Shader& shader, kobalt::Device device, const char* path) {
    std::fstream file(path, std::ios::binary | std::ios::in);
    if (!file.good()) {
        return false;
    }

    file.seekg(0, std::ios::end);
    size_t size = file.tellg();
    file.seekg(0, std::ios::beg);

    std::vector<char> data(size);
    file.read(&data[0], size);
    file.close();

    return kobalt::createShaderSPIRV(shader, device, reinterpret_cast<uint32_t const*>(data.data()), size);
}

int main() {
    assert(glfwInit());

    glfwWindowHint(GLFW_CLIENT_API, GLFW_NO_API);
    GLFWwindow* window = glfwCreateWindow(1200, 800, "kobalt", nullptr, nullptr);
    assert(window != nullptr);

    assert(kobalt::init(true));

    kobalt::DeviceAdapterInfo adapterInfo;
    for (uint32_t i = 0; kobalt::enumerateDeviceAdapters(adapterInfo, i); ++i) {
        std::cout << adapterInfo.name << std::endl;
        std::cout << "    vendor: " << std::hex << static_cast<uint32_t>(adapterInfo.vendor) << std::dec << std::endl;
        std::cout << "    type: " << static_cast<uint32_t>(adapterInfo.type) << std::endl;
        std::cout << "      dynamic render state: " << adapterInfo.support.dynamicRenderState << std::endl;
        std::cout << "      swapchain: " << adapterInfo.support.swapchain << std::endl;
        std::cout << "    texture 1D: " << adapterInfo.maxTextureSize1D << std::endl;
        std::cout << "    texture 2D: " << adapterInfo.maxTextureSize2D << std::endl;
        std::cout << "    texture 3D: " << adapterInfo.maxTextureSize3D << std::endl;
        std::cout << "    render target: " << adapterInfo.maxRenderTargetSize[0] << "x" << adapterInfo.maxRenderTargetSize[1] << "x" << adapterInfo.maxRenderTargetSize[2] << std::endl;
        std::cout << "    render target count: " << adapterInfo.maxRenderTargets << std::endl;
        std::cout << "    compute work group count: " << adapterInfo.maxComputeWorkGroupCount[0] << "x" << adapterInfo.maxComputeWorkGroupCount[1] << "x" << adapterInfo.maxComputeWorkGroupCount[2] << std::endl;
        std::cout << "    compute work group size: " << adapterInfo.maxComputeWorkGroupSize[0] << "x" << adapterInfo.maxComputeWorkGroupSize[1] << "x" << adapterInfo.maxComputeWorkGroupSize[2] << std::endl;

        uint64_t memSize = adapterInfo.localMemorySize;
        char suffix;
        formatMemorySize(memSize, suffix, false);

        std::cout << "    local memory: " << memSize << suffix << std::endl;

        memSize = adapterInfo.otherMemorySize;
        formatMemorySize(memSize, suffix, false);
        std::cout << "    other memory: " << memSize << suffix << std::endl;
    }

    kobalt::Device device;
    assert(kobalt::createDevice(device, 0, { false, true }));
    kobalt::setDebugName(device, "Device");

    int w, h;
    glfwGetFramebufferSize(window, &w, &h);

    kobalt::wsi::Swapchain swapchain;
    assert(kobalt::wsi::glfw::createSwapchain(swapchain, device, window, w, h, 1, kobalt::TextureUsage::RenderTarget, false, kobalt::wsi::PresentMode::Any));
    kobalt::setDebugName(swapchain, "Swapchain");

    kobalt::Shader vertexShader;
    assert(loadShader(vertexShader, device, "shader.vertex.spv"));
    kobalt::setDebugName(vertexShader, "Vertex Shader");

    kobalt::Shader pixelShader;
    assert(loadShader(pixelShader, device, "shader.pixel.spv"));
    kobalt::setDebugName(pixelShader, "Pixel Shader");

    kobalt::VertexAttribute attributes[2] = {
        { 0, 0, kobalt::VertexAttributeType::Float32, 3, 0 },
        { 1, 0, kobalt::VertexAttributeType::Float32, 2, sizeof(float) * 3 },
    };

    kobalt::VertexBinding bindings[1] = {
        { 0, sizeof(float) * 5, false },
    };

    kobalt::VertexInputState vertexInputState;
    assert(kobalt::createVertexInputState(vertexInputState, device, kobalt::Topology::TriangleList, attributes, 2, bindings, 1));
    kobalt::setDebugName(vertexInputState, "Vertex Input State");

    kobalt::RasterizationState rasterizationState;
    assert(kobalt::createRasterizationState(rasterizationState, device, kobalt::FillMode::Fill, kobalt::CullMode::None, kobalt::FrontFace::CounterClockwise, 0.0f, 0.0f, 0.0f));
    kobalt::setDebugName(rasterizationState, "Rasterization State");

    kobalt::RenderAttachment renderTargetAttachment = {};
    renderTargetAttachment.format = kobalt::wsi::getSwapchainFormat(swapchain);
    renderTargetAttachment.sampleCount = 1;
    renderTargetAttachment.loadOp = kobalt::RenderAttachmentLoadOp::Clear;
    renderTargetAttachment.storeOp = kobalt::RenderAttachmentStoreOp::Store;
    renderTargetAttachment.stencilLoadOp = kobalt::RenderAttachmentLoadOp::DontCare;
    renderTargetAttachment.stencilStoreOp = kobalt::RenderAttachmentStoreOp::DontCare;
    renderTargetAttachment.initialLayout = kobalt::TextureLayout::RenderTarget;
    renderTargetAttachment.finalLayout = kobalt::TextureLayout::PresentSrc;

    kobalt::RenderAttachmentState renderAttachmentState;
    assert(kobalt::createRenderAttachmentState(renderAttachmentState, device, &renderTargetAttachment, 1));

    kobalt::RenderAttachmentReference renderTargetReference = {};
    renderTargetReference.index = 0;
    renderTargetReference.layout = kobalt::TextureLayout::RenderTarget;

    kobalt::RenderSubpass renderSubpass;
    assert(kobalt::createRenderSubpass(renderSubpass, device, nullptr, 0, &renderTargetReference, 1, nullptr));

    kobalt::RenderPass renderPass;
    assert(kobalt::createRenderPass(renderPass, device, &renderSubpass, 0, renderAttachmentState));

    kobalt::GraphicsPipelineAttachment gpRenderTargetAttachment = {};
    gpRenderTargetAttachment.format = renderTargetAttachment.format;
    gpRenderTargetAttachment.sampleCount = renderTargetAttachment.sampleCount;

    kobalt::Pipeline graphicsPipeline;
    assert(kobalt::createGraphicsPipeline(graphicsPipeline, device, vertexInputState, rasterizationState, vertexShader, pixelShader, nullptr, nullptr, 0, &gpRenderTargetAttachment, 1, nullptr, 0));

    while (!glfwWindowShouldClose(window)) {
        glfwPollEvents();
    }

    kobalt::destroy(rasterizationState);
    kobalt::destroy(vertexInputState);
    kobalt::destroy(vertexShader);
    kobalt::destroy(pixelShader);
    kobalt::destroy(swapchain);
    kobalt::destroy(device);
    kobalt::deinit();

    glfwDestroyWindow(window);
    glfwTerminate();

    return 0;
}
