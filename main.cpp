#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>

#define KOBALT_IMPL
#define KOBALT_GLFW
#define KOBALT_DEFAULT_DEBUG_MESSENGER
#include "kobalt.h"

#include <cassert>
#include <iostream>
#include <vector>
#include <fstream>

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
        std::cout << "      dynamic render state: " << adapterInfo.support.dynamicRenderPass << std::endl;
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
    assert(kobalt::createDevice(device, 0, { true, true, true, false }));
    kobalt::setDebugName(device, "Device");

    int w, h;
    glfwGetFramebufferSize(window, &w, &h);

    kobalt::wsi::Swapchain swapchain;
    assert(kobalt::wsi::glfw::createSwapchain(swapchain, device, window, w, h, 1, kobalt::TextureUsage::RenderTarget, false, kobalt::wsi::PresentMode::Any));
    kobalt::setDebugName(swapchain, "Swapchain");

    kobalt::Texture backbuffer = kobalt::wsi::getSwapchainBackbuffer(swapchain);

    kobalt::TextureView backbufferView;
    assert(kobalt::createTextureView(backbufferView, backbuffer, kobalt::wsi::getSwapchainFormat(swapchain), kobalt::TextureDimensions::Texture2D, nullptr, nullptr));

    kobalt::Texture depthTexture;
    assert(kobalt::createTexture(depthTexture, device, kobalt::TextureDimensions::Texture2D, w, h, 1, 1, 1, 1, kobalt::TextureFormat::D16_UNorm, kobalt::MemoryLocation::DeviceLocal, kobalt::TextureUsage::DepthTarget));

    kobalt::TextureView depthTextureView;
    assert(kobalt::createTextureView(depthTextureView, depthTexture, kobalt::TextureFormat::D16_UNorm, kobalt::TextureDimensions::Texture2D, nullptr, nullptr));

    kobalt::Shader vertexShader;
    assert(loadShader(vertexShader, device, "shader.vertex.spv"));
    kobalt::setDebugName(vertexShader, "Vertex Shader");

    kobalt::Shader pixelShader;
    assert(loadShader(pixelShader, device, "shader.pixel.spv"));
    kobalt::setDebugName(pixelShader, "Pixel Shader");

    float vertexData[] = {
        -1.0f, -1.0f, 0.001f,     0.0f, 1.0f,     1.0f, 0.0f, 1.0f,
         1.0f, -1.0f, 0.001f,     1.0f, 1.0f,     0.0f, 1.0f, 1.0f,
         0.0f,  1.0f, 0.001f,     0.5f, 0.0f,     1.0f, 1.0f, 0.0f,
    };

    kobalt::Buffer vertexBufferUpload;
    assert(kobalt::createBuffer(vertexBufferUpload, device, sizeof(vertexData), kobalt::MemoryLocation::HostLocal, kobalt::BufferUsage::TransferSrc));

    void* vertexUploadMapped;
    assert(kobalt::mapBuffer(vertexUploadMapped, vertexBufferUpload, 0, sizeof(vertexData)));
    memcpy(vertexUploadMapped, vertexData, sizeof(vertexData));
    kobalt::unmapBuffer(vertexBufferUpload, vertexUploadMapped);

    kobalt::HostSync vertexBufferUploadSync;
    assert(kobalt::createHostSync(vertexBufferUploadSync, device, false));

    kobalt::Buffer vertexBuffer;
    assert(kobalt::createBuffer(vertexBuffer, device, sizeof(vertexData), kobalt::MemoryLocation::DeviceLocal, kobalt::BufferUsage::VertexBuffer | kobalt::BufferUsage::IndexBuffer | kobalt::BufferUsage::TransferDst));
    assert(kobalt::copyBuffer(vertexBufferUpload, 0, vertexBuffer, 0, sizeof(vertexData), vertexBufferUploadSync, nullptr, 0));

    kobalt::VertexAttribute attributes[3] = {
        { 0, 0, kobalt::VertexAttributeType::Float32, 3, 0 },
        { 1, 0, kobalt::VertexAttributeType::Float32, 2, sizeof(float) * 3 },
        { 2, 0, kobalt::VertexAttributeType::Float32, 3, sizeof(float) * 5 },
    };

    kobalt::VertexBinding bindings[1] = {
        { 0, sizeof(float) * 8, false },
    };

    kobalt::VertexInputState vertexInputState;
    assert(kobalt::createVertexInputState(vertexInputState, device, kobalt::Topology::TriangleList, attributes, 3, bindings, 1));
    kobalt::setDebugName(vertexInputState, "Vertex Input State");

    kobalt::RasterizationState rasterizationState;
    assert(kobalt::createRasterizationState(rasterizationState, device, kobalt::FillMode::Fill, kobalt::CullMode::None, kobalt::FrontFace::CounterClockwise, 0.0f, 0.0f, 0.0f));
    kobalt::setDebugName(rasterizationState, "Rasterization State");

    kobalt::DepthStencilState depthStencilState;
    assert(kobalt::createDepthStencilState(depthStencilState, device, true, true, kobalt::CompareOp::Greater, false, nullptr, nullptr));

/* static render pass
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
    assert(kobalt::createGraphicsPipeline(graphicsPipeline, device, vertexInputState, rasterizationState, vertexShader, pixelShader, nullptr, nullptr, 0, &gpRenderTargetAttachment, 1, nullptr, 0, false, 0));
*/

    kobalt::GraphicsPipelineAttachment graphicsPipelineAttachments[2];
    graphicsPipelineAttachments[0].format = kobalt::wsi::getSwapchainFormat(swapchain);
    graphicsPipelineAttachments[0].sampleCount = 1;
    graphicsPipelineAttachments[1].format = kobalt::TextureFormat::D16_UNorm;
    graphicsPipelineAttachments[1].sampleCount = 1;

    kobalt::PipelineShader pipelineVS = {};
    pipelineVS.shader = vertexShader;
    pipelineVS.name = "vsmain";

    kobalt::PipelineShader pipelinePS = {};
    pipelinePS.shader = pixelShader;
    pipelinePS.name = "psmain";

    kobalt::Pipeline graphicsPipeline;
    assert(kobalt::createGraphicsPipeline(graphicsPipeline, device, vertexInputState, nullptr, rasterizationState, depthStencilState, nullptr, &pipelineVS, nullptr, nullptr, nullptr, &pipelinePS, nullptr, nullptr, 0, &graphicsPipelineAttachments[0], 1, &graphicsPipelineAttachments[1], 0, true, 0));

    kobalt::CommandList commandList;
    assert(kobalt::createCommandList(commandList, device, kobalt::QueueType::Graphics, false));
    kobalt::setDebugName(commandList, "Command List");

    kobalt::HostSync readyToRenderSync;
    assert(kobalt::createHostSync(readyToRenderSync, device, true));
    kobalt::setDebugName(readyToRenderSync, "Ready to Render Host Sync");

    kobalt::QueueSync backbufferAvailableSync;
    assert(kobalt::createQueueSync(backbufferAvailableSync, device));
    kobalt::setDebugName(backbufferAvailableSync, "Backbuffer Available Queue Sync");

    kobalt::QueueSync renderFinishedSync;
    assert(kobalt::createQueueSync(renderFinishedSync, device));
    kobalt::setDebugName(renderFinishedSync, "Render Finished Queue Sync");

    assert(kobalt::waitForHostSync(vertexBufferUploadSync, UINT64_MAX));
    kobalt::destroy(vertexBufferUpload);
    kobalt::destroy(vertexBufferUploadSync);

    while (!glfwWindowShouldClose(window)) {
        assert(kobalt::waitForHostSync(readyToRenderSync, UINT64_MAX));
        assert(kobalt::resetHostSync(readyToRenderSync));

        assert(kobalt::wsi::prepareNextSwapchainTexture(nullptr, swapchain, UINT64_MAX, backbufferAvailableSync, nullptr));

        assert(kobalt::resetCommandList(commandList));
        assert(kobalt::beginRecordingCommandList(commandList));

        assert(kobalt::cmd::executionBarrier(commandList, kobalt::PipelineStage::Top, kobalt::PipelineStage::RenderTarget));
        assert(kobalt::cmd::textureBarrier(commandList, kobalt::ResourceAccess::None, kobalt::ResourceAccess::RenderTargetWrite, kobalt::QueueTransfer::Identity, kobalt::QueueTransfer::Identity, kobalt::TextureLayout::Undefined, kobalt::TextureLayout::RenderTarget, backbuffer, nullptr));

        kobalt::DynamicRenderAttachment attachments[2];
        attachments[0].view = backbufferView;
        attachments[0].layout = kobalt::TextureLayout::RenderTarget;
        attachments[0].loadOp = kobalt::RenderAttachmentLoadOp::Clear;
        attachments[0].storeOp = kobalt::RenderAttachmentStoreOp::Store;
        attachments[0].clearValue.color.rgbaFloat[0] = 0.0f;
        attachments[0].clearValue.color.rgbaFloat[1] = 0.0f;
        attachments[0].clearValue.color.rgbaFloat[2] = 0.0f;
        attachments[0].clearValue.color.rgbaFloat[3] = 1.0f;
        attachments[1].view = depthTextureView;
        attachments[1].layout = kobalt::TextureLayout::DepthStencilTarget;
        attachments[1].loadOp = kobalt::RenderAttachmentLoadOp::Clear;
        attachments[1].storeOp = kobalt::RenderAttachmentStoreOp::DontCare;
        attachments[1].clearValue.depthStencil.depth = 0.0f;

        assert(kobalt::cmd::beginDynamicRenderPass(commandList, { 0, 0, static_cast<uint32_t>(w), static_cast<uint32_t>(h) }, 1, 0, &attachments[0], 1, &attachments[1], nullptr));
        assert(kobalt::cmd::bindGraphicsPipeline(commandList, graphicsPipeline));
        assert(kobalt::cmd::bindVertexBuffer(commandList, 0, vertexBuffer, 0));
        assert(kobalt::cmd::setViewport(commandList, 0.0f, 0.0f, static_cast<float>(w), static_cast<float>(h), 0.0f, 1.0f));
        assert(kobalt::cmd::setScissor(commandList, 0, 0, w, h));
        assert(kobalt::cmd::draw(commandList, 0, 3));
        assert(kobalt::cmd::endRenderPass(commandList));

        kobalt::PipelineStage const finalStage = kobalt::PipelineStage::Bottom;
        assert(kobalt::cmd::executionBarrier(commandList, kobalt::PipelineStage::RenderTarget, finalStage));
        assert(kobalt::cmd::textureBarrier(commandList, kobalt::ResourceAccess::RenderTargetWrite, kobalt::ResourceAccess::MemoryRead, kobalt::QueueTransfer::Identity, kobalt::QueueTransfer::Identity, kobalt::TextureLayout::RenderTarget, kobalt::TextureLayout::PresentSrc, backbuffer, nullptr));
        assert(kobalt::endRecordingCommandList(commandList));

        assert(kobalt::executeCommandList(commandList, &backbufferAvailableSync, &finalStage, 1, &renderFinishedSync, 1, readyToRenderSync));
        assert(kobalt::wsi::present(nullptr, swapchain, &renderFinishedSync, 1));

        glfwPollEvents();
    }

    kobalt::destroy(renderFinishedSync);
    kobalt::destroy(backbufferAvailableSync);
    kobalt::destroy(readyToRenderSync);
    kobalt::destroy(commandList);
    kobalt::destroy(graphicsPipeline);
    kobalt::destroy(depthStencilState);
    kobalt::destroy(rasterizationState);
    kobalt::destroy(vertexInputState);
    kobalt::destroy(vertexBuffer);
    kobalt::destroy(vertexShader);
    kobalt::destroy(pixelShader);
    kobalt::destroy(depthTextureView);
    kobalt::destroy(depthTexture);
    kobalt::destroy(backbufferView);
    kobalt::destroy(swapchain);
    kobalt::destroy(device);
    kobalt::deinit();

    glfwDestroyWindow(window);
    glfwTerminate();

    return 0;
}
