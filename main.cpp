#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>

#define KOBALT_IMPL
#define KOBALT_GLFW
#define KOBALT_DEFAULT_DEBUG_MESSENGER
#include "kobalt.h"

#define STBI_FAILURE_USERMSG
#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

#include "linmath.h"

#include <cassert>
#include <cstring>
#include <iostream>
#include <vector>
#include <fstream>
#include <filesystem>

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

struct Uniforms {
    mat4x4 mvp;
};

struct TextureResource {
    kobalt::Texture texture;
    kobalt::TextureView view;
};

void loadTextures(std::vector<TextureResource>& textures, kobalt::Device device) {
    std::filesystem::path const path("textures");
    for (std::filesystem::directory_entry const& entry : std::filesystem::directory_iterator(path)) {
        int textureWidth, textureHeight, textureComp;
        stbi_uc* textureData = stbi_load(entry.path().string().c_str(), &textureWidth, &textureHeight, &textureComp, 4);
        if (textureData == nullptr) {
            std::cout << "Failed to load " << entry.path().string() << ": " << stbi_failure_reason() << std::endl;
            continue;
        }

        kobalt::Texture texture;
        assert(kobalt::createTexture(texture, device, kobalt::TextureDimensions::Texture2D, textureWidth, textureHeight, 1, 1, 1, 1, kobalt::TextureFormat::RGBA8_UNorm, kobalt::MemoryLocation::DeviceLocal, kobalt::TextureUsage::SampledTexture | kobalt::TextureUsage::TransferDst));
        kobalt::setDebugName(texture, entry.path().string().c_str());

        assert(kobalt::uploadTextureData(texture, 0, 0, 0, textureWidth, textureHeight, 1, nullptr, textureData));
        stbi_image_free(textureData);

        kobalt::TextureView view;
        assert(kobalt::createTextureView(view, texture, kobalt::TextureFormat::RGBA8_UNorm, kobalt::TextureDimensions::Texture2D, nullptr, nullptr));
        kobalt::setDebugName(view, entry.path().string().c_str());

        textures.push_back({ texture, view });
    }
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
        std::cout << "      swapchain: " << adapterInfo.support.swapchain << std::endl;
        std::cout << "      flip viewport: " << adapterInfo.support.flipViewport << std::endl;
        std::cout << "      dynamic render state: " << adapterInfo.support.dynamicRenderPass << std::endl;
        std::cout << "      dynamic pipeline resources: " << adapterInfo.support.dynamicPipelineResources << std::endl;
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

    kobalt::DeviceSupport support = {};
    support.swapchain = true;
    support.flipViewport = true;
    support.dynamicRenderPass = true;
    support.dynamicPipelineResources = true;
    support.pipelineResourceIndexing = true;
    support.partiallyBoundPipelineResources = true;
    support.variablePipelineResourceArray = true;

    kobalt::Device device;
    assert(kobalt::createDevice(device, 0, support));
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

    std::vector<TextureResource> textures;
    loadTextures(textures, device);

    kobalt::Sampler sampler;
    assert(kobalt::createSampler(sampler, device, kobalt::SampleFilter::Nearest, kobalt::SampleFilter::Nearest, kobalt::SampleFilter::Nearest, kobalt::SampleMode::Clamp, kobalt::SampleMode::Clamp, kobalt::SampleMode::Clamp, 1.0f, 0.0f, 0.0f));

    kobalt::Shader vertexShader;
    assert(loadShader(vertexShader, device, "shader.vertex.spv"));
    kobalt::setDebugName(vertexShader, "Vertex Shader");

    kobalt::Shader pixelShader;
    assert(loadShader(pixelShader, device, "shader.pixel.spv"));
    kobalt::setDebugName(pixelShader, "Pixel Shader");

    float vertexData[] = {
        -3.25f,  1.0f, 0.5f,     0.0f, 0.0f,     1.0f, 0.0f, 1.0f,
        -3.25f, -1.0f, 0.5f,     0.0f, 1.0f,     1.0f, 0.0f, 1.0f,
         3.25f, -1.0f, 0.5f,     1.0f, 1.0f,     1.0f, 0.0f, 1.0f,
         3.25f,  1.0f, 0.5f,     1.0f, 0.0f,     1.0f, 0.0f, 1.0f,
         3.25f, -1.0f, 0.5f,     1.0f, 1.0f,     1.0f, 0.0f, 1.0f,
        -3.25f,  1.0f, 0.5f,     0.0f, 0.0f,     1.0f, 0.0f, 1.0f,
    };
    
    uint32_t indexData[] = {
        0, 1, 2,
        2, 3, 0,
    };

    kobalt::Buffer vertexBuffer;
    assert(kobalt::createBuffer(vertexBuffer, device, sizeof(vertexData) + sizeof(indexData), kobalt::MemoryLocation::DeviceLocal, kobalt::BufferUsage::VertexBuffer | kobalt::BufferUsage::IndexBuffer | kobalt::BufferUsage::TransferDst));
    assert(kobalt::uploadBufferData(vertexBuffer, 0, vertexData, sizeof(vertexData)));
    assert(kobalt::uploadBufferData(vertexBuffer, sizeof(vertexData), indexData, sizeof(indexData)));

    kobalt::Buffer uniformBuffer;
    assert(kobalt::createBuffer(uniformBuffer, device, sizeof(Uniforms) * textures.size(), kobalt::MemoryLocation::HostLocal, kobalt::BufferUsage::UniformBuffer));

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

    kobalt::BlendAttachmentState blendAttachment;
    assert(kobalt::createBlendAttachmentState(blendAttachment, device, true, kobalt::BlendFactor::SrcAlpha, kobalt::BlendFactor::InvSrcAlpha, kobalt::BlendOp::Add, kobalt::BlendFactor::One, kobalt::BlendFactor::Zero, kobalt::BlendOp::Add, kobalt::ColorComponentMask::All));
    
    kobalt::BlendState blendState;
    assert(kobalt::createBlendState(blendState, device, &blendAttachment, 1, false, kobalt::LogicOp::Clear, nullptr));
    kobalt::destroy(blendAttachment);

    kobalt::PipelineResourceBinding pipelineBindings[2];
    pipelineBindings[0].binding = 0;
    pipelineBindings[0].type = kobalt::PipelineResourceType::Buffer;
    pipelineBindings[0].access = kobalt::PipelineResourceAccess::Uniform;
    pipelineBindings[0].stages = kobalt::ShaderStage::Vertex;
    pipelineBindings[0].arrayLength = 8;
    pipelineBindings[1].binding = 1;
    pipelineBindings[1].type = kobalt::PipelineResourceType::TextureAndSampler;
    pipelineBindings[1].access = kobalt::PipelineResourceAccess::Sampled;
    pipelineBindings[1].stages = kobalt::ShaderStage::Pixel;
    pipelineBindings[1].arrayLength = 8;
    pipelineBindings[1].partiallyBoundExt = true;
    pipelineBindings[1].variableArrayExt = true;
    
    kobalt::PipelineResourceLayout pipelineLayout;
    assert(kobalt::createPipelineResourceLayout(pipelineLayout, device, pipelineBindings, 2, nullptr, 0, true));

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
    assert(kobalt::createGraphicsPipeline(graphicsPipeline, device, vertexInputState, nullptr, rasterizationState, depthStencilState, blendState, &pipelineVS, nullptr, nullptr, nullptr, &pipelinePS, &pipelineLayout, 1, nullptr, 0, &graphicsPipelineAttachments[0], 1, &graphicsPipelineAttachments[1], 0, true, 0));

    kobalt::destroy(blendState);
    kobalt::destroy(depthStencilState);
    kobalt::destroy(rasterizationState);
    kobalt::destroy(vertexInputState);

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

    void* mappedUniformBuffer;
    assert(kobalt::mapBuffer(mappedUniformBuffer, uniformBuffer, 0, sizeof(Uniforms)));
    
    float aspect = w / static_cast<float>(h);
    while (!glfwWindowShouldClose(window)) {
        mat4x4 m;
        mat4x4_identity(m);
        mat4x4_rotate_Z(m, m, glfwGetTime());
        mat4x4_scale_aniso(m, m, 0.05f, 0.05f, 0.05f);

        float n = -1.0f;
        float f = 1.0f;

        mat4x4 p;
        mat4x4_identity(p);
        mat4x4_scale_aniso(p, p, 1.0f, aspect, 1.0f);
        p[2][2] = -f / (f - n);
        p[3][2] = -f * n / (f - n);
        
        Uniforms uniforms = {};
        mat4x4_mul(uniforms.mvp, p, m);
        memcpy(mappedUniformBuffer, &uniforms, sizeof(Uniforms));

        mat4x4_translate(m, 0.5f, 0.0f, 0.0f);
        mat4x4_rotate_Z(m, m, -glfwGetTime());
        mat4x4_scale_aniso(m, m, 0.15f, 0.15f, 0.15f);

        mat4x4_mul(uniforms.mvp, p, m);
        memcpy(reinterpret_cast<Uniforms*>(mappedUniformBuffer) + 1, &uniforms, sizeof(Uniforms));

        assert(kobalt::waitForHostSync(readyToRenderSync, UINT64_MAX));
        assert(kobalt::resetHostSync(readyToRenderSync));

        assert(kobalt::wsi::prepareNextSwapchainTexture(nullptr, swapchain, UINT64_MAX, backbufferAvailableSync, nullptr));

        assert(kobalt::resetCommandList(commandList));
        assert(kobalt::beginRecordingCommandList(commandList));

        assert(kobalt::cmd::executionBarrier(commandList, kobalt::PipelineStage::Top, kobalt::PipelineStage::PixelShader));
        for (TextureResource const& r : textures) {
            assert(kobalt::cmd::textureBarrier(commandList, kobalt::ResourceAccess::None, kobalt::ResourceAccess::ShaderRead, kobalt::QueueTransfer::Identity, kobalt::QueueTransfer::Identity, kobalt::TextureLayout::Undefined, kobalt::TextureLayout::ShaderRead, r.texture, nullptr));
        }

        assert(kobalt::cmd::executionBarrier(commandList, kobalt::PipelineStage::Top, kobalt::PipelineStage::RenderTarget));
        assert(kobalt::cmd::textureBarrier(commandList, kobalt::ResourceAccess::None, kobalt::ResourceAccess::RenderTargetWrite, kobalt::QueueTransfer::Identity, kobalt::QueueTransfer::Identity, kobalt::TextureLayout::Undefined, kobalt::TextureLayout::RenderTarget, backbuffer, nullptr));

        kobalt::DynamicRenderAttachment attachments[2];
        attachments[0].view = backbufferView;
        attachments[0].layout = kobalt::TextureLayout::RenderTarget;
        attachments[0].loadOp = kobalt::RenderAttachmentLoadOp::Clear;
        attachments[0].storeOp = kobalt::RenderAttachmentStoreOp::Store;
        attachments[0].clearValue.color.rgbaFloat[0] = 0.12f;
        attachments[0].clearValue.color.rgbaFloat[1] = 0.10f;
        attachments[0].clearValue.color.rgbaFloat[2] = 0.12f;
        attachments[0].clearValue.color.rgbaFloat[3] = 1.00f;
        attachments[1].view = depthTextureView;
        attachments[1].layout = kobalt::TextureLayout::DepthStencilTarget;
        attachments[1].loadOp = kobalt::RenderAttachmentLoadOp::Clear;
        attachments[1].storeOp = kobalt::RenderAttachmentStoreOp::DontCare;
        attachments[1].clearValue.depthStencil.depth = 0.0f;

        std::vector<kobalt::PipelineResourceBuffer> resourceBuffers(textures.size());
        std::vector<kobalt::PipelineResourceTextureSampler> resourceTextures(textures.size());
        for (size_t i = 0; i < textures.size(); ++i) {
            resourceBuffers[i].binding = 0;
            resourceBuffers[i].elementBase = i;
            resourceBuffers[i].elementCount = 1;
            resourceBuffers[i].type = kobalt::PipelineResourceType::Buffer;
            resourceBuffers[i].access = kobalt::PipelineResourceAccess::Uniform;
            resourceBuffers[i].offset = sizeof(Uniforms) * i;
            resourceBuffers[i].range = sizeof(Uniforms);
            resourceBuffers[i].buffer = uniformBuffer;

            resourceTextures[i].binding = 1;
            resourceTextures[i].elementBase = i;
            resourceTextures[i].elementCount = 1;
            resourceTextures[i].type = kobalt::PipelineResourceType::TextureAndSampler;
            resourceTextures[i].access = kobalt::PipelineResourceAccess::Sampled;
            resourceTextures[i].sampler = sampler;
            resourceTextures[i].view = textures[i].view;
            resourceTextures[i].layout = kobalt::TextureLayout::ShaderRead;
        }

        assert(kobalt::cmd::beginDynamicRenderPass(commandList, { 0, 0, static_cast<uint32_t>(w), static_cast<uint32_t>(h) }, 1, 0, &attachments[0], 1, &attachments[1], nullptr));
        
        assert(kobalt::cmd::bindPipeline(commandList, graphicsPipeline));
        assert(kobalt::cmd::pushDynamicPipelineResources(commandList, graphicsPipeline, 0, resourceTextures.data(), static_cast<uint32_t>(resourceTextures.size()), resourceBuffers.data(), 1, nullptr, 0));
        
        assert(kobalt::cmd::bindVertexBuffer(commandList, 0, vertexBuffer, 0));
        assert(kobalt::cmd::bindIndexBuffer(commandList, vertexBuffer, sizeof(vertexData), kobalt::IndexType::Uint32));
        
        assert(kobalt::cmd::setViewport(commandList, 0.0f, 0.0f, static_cast<float>(w), static_cast<float>(h), 0.0f, 1.0f));
        assert(kobalt::cmd::setScissor(commandList, 0, 0, w, h));
        
        assert(kobalt::cmd::drawInstancedIndexed(commandList, 0, 0, 6, 0, textures.size()));
        
        assert(kobalt::cmd::endRenderPass(commandList));

        kobalt::PipelineStage const finalStage = kobalt::PipelineStage::Bottom;
        assert(kobalt::cmd::executionBarrier(commandList, kobalt::PipelineStage::RenderTarget, finalStage));
        assert(kobalt::cmd::textureBarrier(commandList, kobalt::ResourceAccess::RenderTargetWrite, kobalt::ResourceAccess::MemoryRead, kobalt::QueueTransfer::Identity, kobalt::QueueTransfer::Identity, kobalt::TextureLayout::RenderTarget, kobalt::TextureLayout::PresentSrc, backbuffer, nullptr));
        assert(kobalt::endRecordingCommandList(commandList));

        assert(kobalt::executeCommandList(commandList, &backbufferAvailableSync, &finalStage, 1, &renderFinishedSync, 1, readyToRenderSync));
        assert(kobalt::wsi::present(nullptr, swapchain, &renderFinishedSync, 1));

        glfwPollEvents();
    }

    kobalt::unmapBuffer(uniformBuffer, mappedUniformBuffer);

    kobalt::destroy(renderFinishedSync);
    kobalt::destroy(backbufferAvailableSync);
    kobalt::destroy(readyToRenderSync);
    kobalt::destroy(commandList);
    kobalt::destroy(graphicsPipeline);
    kobalt::destroy(pipelineLayout);
    kobalt::destroy(uniformBuffer);
    kobalt::destroy(vertexBuffer);
    kobalt::destroy(vertexShader);
    kobalt::destroy(pixelShader);
    kobalt::destroy(sampler);

    for (TextureResource const& r : textures) {
        kobalt::destroy(r.view);
        kobalt::destroy(r.texture);
    }

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
