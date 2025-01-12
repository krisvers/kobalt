#ifndef PRISM_VK_HPP
#define PRISM_VK_HPP

#include <vulkan/vulkan.h>
#include <vector>
#include <array>
#include <string>
#include <algorithm>
#include <cstring>

namespace prism {

namespace vk {

struct Section3D {
    Section3D(uint32_t w, uint32_t h) : width(w), height(h) {}
    Section3D(uint32_t w, uint32_t h, uint32_t d) : width(w), height(h), depth(d) {}
    Section3D(uint32_t x, uint32_t y, uint32_t z, uint32_t w, uint32_t h, uint32_t d) : x(x), y(y), z(z), width(w), height(h), depth(d) {}

    uint32_t x = 0;
    uint32_t y = 0;
    uint32_t z = 0;
    uint32_t width;
    uint32_t height;
    uint32_t depth = 1;
};

struct Extension {
    std::string name;
    bool required;
    
    Extension(const char* name, bool required) : name(name), required(required) {}
    Extension(const char* name) : Extension(name, true) {}
    
    Extension(std::string const& name, bool required) : name(name), required(required) {}
    Extension(std::string const& name) : Extension(name, true) {}
};

struct DescriptorSetLayoutBuilder {
    void addBinding(uint32_t binding, VkDescriptorType type, uint32_t count, VkShaderStageFlags stageFlags) {
        bindings.push_back({ binding, type, count, stageFlags });
    }

    std::vector<VkDescriptorSetLayoutBinding> bindings;
};

struct DescriptorSetWriteBuilder {
    void addImageWrite(VkDescriptorSet set, uint32_t binding, uint32_t arrayElement, uint32_t count, VkDescriptorType type, VkSampler sampler, VkImageView view, VkImageLayout layout) {
        VkWriteDescriptorSet write = {};
        write.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
        write.dstSet = set;
        write.dstBinding = binding;
        write.dstArrayElement = arrayElement;
        write.descriptorCount = count;
        write.descriptorType = type;

        VkDescriptorImageInfo imageInfo = {};
        imageInfo.sampler = sampler;
        imageInfo.imageView = view;
        imageInfo.imageLayout = layout;

        WriteSet writeSet = { static_cast<uint32_t>(imageInfos.size()), WriteType::Image };

        writeSetIndices.push_back(writeSet);
        imageInfos.push_back(imageInfo);
        writes.push_back(write);
    }

    void addBufferWrite(VkDescriptorSet set, uint32_t binding, uint32_t arrayElement, uint32_t count, VkDescriptorType type, VkBuffer buffer, VkDeviceSize offset, VkDeviceSize range) {
        VkWriteDescriptorSet write = {};
        write.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
        write.dstSet = set;
        write.dstBinding = binding;
        write.dstArrayElement = arrayElement;
        write.descriptorCount = count;
        write.descriptorType = type;

        VkDescriptorBufferInfo bufferInfo = {};
        bufferInfo.buffer = buffer;
        bufferInfo.offset = offset;
        bufferInfo.range = range;

        WriteSet writeSet = { static_cast<uint32_t>(bufferInfos.size()), WriteType::Buffer };

        writeSetIndices.push_back(writeSet);
        bufferInfos.push_back(bufferInfo);
        writes.push_back(write);
    }

    void addTexelBufferWrite(VkDescriptorSet set, uint32_t binding, uint32_t arrayElement, uint32_t count, VkDescriptorType type, VkBufferView view) {
        VkWriteDescriptorSet write = {};
        write.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
        write.dstSet = set;
        write.dstBinding = binding;
        write.dstArrayElement = arrayElement;
        write.descriptorCount = count;
        write.descriptorType = type;

        WriteSet writeSet = { static_cast<uint32_t>(texelBufferViews.size()), WriteType::TexelBuffer };

        writeSetIndices.push_back(writeSet);
        texelBufferViews.push_back(view);
        writes.push_back(write);
    }

    std::vector<VkWriteDescriptorSet> const& build() {
        for (size_t i = 0; i < writes.size(); ++i) {
            if (writeSetIndices[i].type == WriteType::Image) {
                writes[i].pImageInfo = &imageInfos[writeSetIndices[i].index];
            } else if (writeSetIndices[i].type == WriteType::Buffer) {
                writes[i].pBufferInfo = &bufferInfos[writeSetIndices[i].index];
            } else if (writeSetIndices[i].type == WriteType::TexelBuffer) {
                writes[i].pTexelBufferView = &texelBufferViews[writeSetIndices[i].index];
            }
        }

        return writes;
    }

    void clear() {
        imageInfos.clear();
        bufferInfos.clear();
        texelBufferViews.clear();

        writeSetIndices.clear();
        writes.clear();
    }

    enum class WriteType {
        Image,
        Buffer,
        TexelBuffer,
    };

    struct WriteSet {
        uint32_t index;
        WriteType type;
    };

    std::vector<VkDescriptorImageInfo> imageInfos;
    std::vector<VkDescriptorBufferInfo> bufferInfos;
    std::vector<VkBufferView> texelBufferViews;

    std::vector<WriteSet> writeSetIndices;
    std::vector<VkWriteDescriptorSet> writes;
};

struct GraphicsPipelineBuilder {
    void setNext(void* pNext) {
        graphicsPipeline.pNext = pNext;
    }
    
    void setFlags(VkPipelineCreateFlags flags) {
        graphicsPipeline.flags = flags;
    }

    void addStage(VkPipelineShaderStageCreateFlags flags, void* pNext, VkShaderStageFlagBits stage, VkShaderModule module, const char* pName, const VkSpecializationInfo* pSpecializationInfo) {
        VkPipelineShaderStageCreateInfo ci = {};
        ci.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
        ci.pNext = pNext;
        ci.flags = flags;
        ci.stage = stage;
        ci.module = module;
        ci.pName = pName;
        ci.pSpecializationInfo = pSpecializationInfo;

        stageCreateInfos.push_back(ci);
    }
    
    void clearStages() {
        stageCreateInfos.clear();
    }

    void addVertexInputBinding(uint32_t binding, uint32_t stride, VkVertexInputRate inputRate) {
        VkVertexInputBindingDescription desc = {};
        desc.binding = binding;
        desc.stride = stride;
        desc.inputRate = inputRate;

        vertexInputBindings.push_back(desc);
    }
    
    void clearVertexInputBindings() {
        vertexInputBindings.clear();
    }

    void addVertexInputAttribute(uint32_t location, uint32_t binding, VkFormat format, uint32_t offset) {
        VkVertexInputAttributeDescription desc = {};
        desc.location = location;
        desc.binding = binding;
        desc.format = format;
        desc.offset = offset;

        vertexInputAttributes.push_back(desc);
    }
    
    void clearVertexInputAttributes() {
        vertexInputAttributes.clear();
    }

    void setVertexInputStateNext(void* pNext) {
        vertexInputState.pNext = pNext;
    }

    void setInputAssemblyState(VkPrimitiveTopology topology, bool primitiveRestartEnable, void* pNext = nullptr) {
        inputAssemblyState.pNext = pNext;
        inputAssemblyState.topology = topology;
        inputAssemblyState.primitiveRestartEnable = primitiveRestartEnable ? VK_TRUE : VK_FALSE;
    }

    void setTesselationState(uint32_t patchControlPoints, void* pNext) {
        tesselationState.pNext = pNext;
        tesselationState.patchControlPoints = patchControlPoints;
    }

    void addViewport(VkViewport viewport) {
        viewports.push_back(viewport);
    }
    
    void setViewportCount(size_t count) {
        viewports.resize(count);
    }
    
    void clearViewports() {
        viewports.clear();
    }

    void addScissor(VkRect2D scissor) {
        scissors.push_back(scissor);
    }

    void setScissorCount(size_t count) {
        scissors.resize(count);
    }
    
    void clearScissors() {
        scissors.clear();
    }

    void setRasterizationState(VkPolygonMode polygonMode, VkCullModeFlags cullMode, VkFrontFace frontFace, void* pNext = nullptr, bool depthClampEnable = false, bool rasterizerDiscardEnable = false, bool depthBiasEnable = false, float depthBiasConstantFactor = 0.0f, float depthBiasClamp = 0.0f, float depthBiasSlopeFactor = 0.0f, float lineWidth = 1.0f) {
        rasterizationState.pNext = pNext;
        rasterizationState.depthClampEnable = depthClampEnable;
        rasterizationState.rasterizerDiscardEnable = rasterizerDiscardEnable;
        rasterizationState.polygonMode = polygonMode;
        rasterizationState.cullMode = cullMode;
        rasterizationState.frontFace = frontFace;
        rasterizationState.depthBiasEnable = depthBiasEnable;
        rasterizationState.depthBiasConstantFactor = depthBiasConstantFactor;
        rasterizationState.depthBiasClamp = depthBiasClamp;
        rasterizationState.depthBiasSlopeFactor = depthBiasSlopeFactor;
        rasterizationState.lineWidth = lineWidth;
    }

    void setMultisampleState(VkSampleCountFlagBits rasterizationSamples, bool sampleShadingEnable, float minSampleShading, bool sampleMaskEnable, VkSampleMask sampleMask, bool alphaToCoverageEnable, bool alphaToOneEnable, void* pNext = nullptr) {
        multisampleSampleMask = sampleMask;

        multisampleState.pNext = pNext;
        multisampleState.rasterizationSamples = rasterizationSamples;
        multisampleState.sampleShadingEnable = sampleShadingEnable;
        multisampleState.minSampleShading = minSampleShading;
        multisampleState.pSampleMask = sampleMaskEnable ? &multisampleSampleMask : nullptr;
        multisampleState.alphaToCoverageEnable = alphaToCoverageEnable;
        multisampleState.alphaToOneEnable = alphaToOneEnable;
    }

    void setDepthState(bool testEnable, bool writeEnable, VkCompareOp compareOp, bool boundsTestEnable, float minDepthBounds, float maxDepthBounds, void* pNext = nullptr) {
        depthStencilState.depthTestEnable = testEnable;
        depthStencilState.depthWriteEnable = writeEnable;
        depthStencilState.depthCompareOp = compareOp;
        depthStencilState.depthBoundsTestEnable = boundsTestEnable;
        depthStencilState.minDepthBounds = minDepthBounds;
        depthStencilState.maxDepthBounds = maxDepthBounds;
    }

    void setStencilStateTestEnable(bool testEnable) {
        depthStencilState.stencilTestEnable = testEnable;
    }

    void setDepthStencilStateFlagsAndNext(VkPipelineDepthStencilStateCreateFlags flags, void* pNext) {
        depthStencilState.pNext = pNext;
        depthStencilState.flags = flags;
    }

    void setDepthStencilStateFront(VkStencilOp failOp, VkStencilOp passOp, VkStencilOp depthFailOp, VkCompareOp compareOp, uint32_t compareMask, uint32_t writeMask, uint32_t reference) {
        depthStencilState.front.failOp = failOp;
        depthStencilState.front.passOp = passOp;
        depthStencilState.front.depthFailOp = depthFailOp;
        depthStencilState.front.compareOp = compareOp;
        depthStencilState.front.compareMask = compareMask;
        depthStencilState.front.writeMask = writeMask;
        depthStencilState.front.reference = reference;
    }

    void setDepthStencilStateBack(VkStencilOp failOp, VkStencilOp passOp, VkStencilOp depthFailOp, VkCompareOp compareOp, uint32_t compareMask, uint32_t writeMask, uint32_t reference) {
        depthStencilState.back.failOp = failOp;
        depthStencilState.back.passOp = passOp;
        depthStencilState.back.depthFailOp = depthFailOp;
        depthStencilState.back.compareOp = compareOp;
        depthStencilState.back.compareMask = compareMask;
        depthStencilState.back.writeMask = writeMask;
        depthStencilState.back.reference = reference;
    }

    void setColorBlendState(VkPipelineColorBlendStateCreateFlags flags, bool logicOpEnable, VkLogicOp logicOp, std::array<float, 4> const& blendConstants, void* pNext = nullptr) {
        colorBlendState.pNext = pNext;
        colorBlendState.flags = flags;
        colorBlendState.logicOpEnable = logicOpEnable;
        colorBlendState.logicOp = logicOp;
        std::memcpy(colorBlendState.blendConstants, blendConstants.data(), sizeof(float) * 4);
    }

    void addColorBlendAttachment(bool blendEnable, VkBlendFactor srcColorBlendFactor, VkBlendFactor dstColorBlendFactor, VkBlendOp colorBlendOp, VkBlendFactor srcAlphaBlendFactor, VkBlendFactor dstAlphaBlendFactor, VkBlendOp alphaBlendOp, VkColorComponentFlags colorWriteMask) {
        VkPipelineColorBlendAttachmentState attachment = {};
        attachment.blendEnable = blendEnable;
        attachment.srcColorBlendFactor = srcColorBlendFactor;
        attachment.dstColorBlendFactor = dstColorBlendFactor;
        attachment.colorBlendOp = colorBlendOp;
        attachment.srcAlphaBlendFactor = srcAlphaBlendFactor;
        attachment.dstAlphaBlendFactor = dstAlphaBlendFactor;
        attachment.alphaBlendOp = alphaBlendOp;
        attachment.colorWriteMask = colorWriteMask;

        colorBlendAttachments.push_back(attachment);
    }
    
    void clearColorBlendAttachments() {
        colorBlendAttachments.clear();
    }

    void setDynamicStates(std::initializer_list<VkDynamicState> const& states) {
        dynamicStates = states;
    }

    void addDynamicState(VkDynamicState dynamicState) {
        dynamicStates.push_back(dynamicState);
    }
    
    void clearDynamicStates() {
        dynamicStates.clear();
    }

    void setDynamicStateNext(void* pNext) {
        dynamicState.pNext = pNext;
    }

    void setLayout(VkPipelineLayout layout) {
        graphicsPipeline.layout = layout;
    }

    void setRenderPass(VkRenderPass renderPass, uint32_t subpass) {
        graphicsPipeline.renderPass = renderPass;
        graphicsPipeline.subpass = subpass;
    }

    void setBasePipeline(VkPipeline basePipeline, int32_t baseIndex) {
        graphicsPipeline.basePipelineHandle = basePipeline;
        graphicsPipeline.basePipelineIndex = baseIndex;
    }
    
    std::vector<VkPipelineShaderStageCreateInfo> stageCreateInfos;

    std::vector<VkVertexInputBindingDescription> vertexInputBindings;
    std::vector<VkVertexInputAttributeDescription> vertexInputAttributes;

    std::vector<VkViewport> viewports;
    std::vector<VkRect2D> scissors;

    VkSampleMask multisampleSampleMask = VK_SAMPLE_COUNT_1_BIT;

    std::vector<VkPipelineColorBlendAttachmentState> colorBlendAttachments;

    std::vector<VkDynamicState> dynamicStates;

    VkPipelineVertexInputStateCreateInfo vertexInputState = {};
    VkPipelineInputAssemblyStateCreateInfo inputAssemblyState = {};
    VkPipelineTessellationStateCreateInfo tesselationState = {};
    VkPipelineViewportStateCreateInfo viewportState = {};
    VkPipelineRasterizationStateCreateInfo rasterizationState = {};
    VkPipelineMultisampleStateCreateInfo multisampleState = {};
    VkPipelineDepthStencilStateCreateInfo depthStencilState = {};
    VkPipelineColorBlendStateCreateInfo colorBlendState = {};
    VkPipelineDynamicStateCreateInfo dynamicState = {};

    VkGraphicsPipelineCreateInfo graphicsPipeline = {};

    VkGraphicsPipelineCreateInfo const& build(bool vertexInputEnabled, bool inputAssemblyEnabled, bool tesselationEnabled, bool viewportEnabled, bool rasterizationEnabled, bool multisampleEnabled, bool depthStencilEnabled, bool colorBlendEnabled, bool dynamicStateEnabled) {
        vertexInputState.vertexBindingDescriptionCount = static_cast<uint32_t>(vertexInputBindings.size());
        vertexInputState.pVertexBindingDescriptions = vertexInputBindings.data();
        vertexInputState.vertexAttributeDescriptionCount = static_cast<uint32_t>(vertexInputAttributes.size());
        vertexInputState.pVertexAttributeDescriptions = vertexInputAttributes.data();

        viewportState.viewportCount = static_cast<uint32_t>(viewports.size());
        viewportState.pViewports = viewports.data();
        viewportState.scissorCount = static_cast<uint32_t>(scissors.size());
        viewportState.pScissors = scissors.data();
        
        colorBlendState.attachmentCount = static_cast<uint32_t>(colorBlendAttachments.size());
        colorBlendState.pAttachments = colorBlendAttachments.data();
        
        dynamicState.dynamicStateCount = static_cast<uint32_t>(dynamicStates.size());
        dynamicState.pDynamicStates = dynamicStates.data();

        graphicsPipeline.stageCount = static_cast<uint32_t>(stageCreateInfos.size());
        graphicsPipeline.pStages = stageCreateInfos.data();
        graphicsPipeline.pVertexInputState = vertexInputEnabled ? &vertexInputState : nullptr;
        graphicsPipeline.pInputAssemblyState = inputAssemblyEnabled ? &inputAssemblyState : nullptr;
        graphicsPipeline.pTessellationState = tesselationEnabled ? &tesselationState : nullptr;
        graphicsPipeline.pViewportState = viewportEnabled ? &viewportState : nullptr;
        graphicsPipeline.pRasterizationState = rasterizationEnabled ? &rasterizationState : nullptr;
        graphicsPipeline.pMultisampleState = multisampleEnabled ? &multisampleState : nullptr;
        graphicsPipeline.pDepthStencilState = depthStencilEnabled ? &depthStencilState : nullptr;
        graphicsPipeline.pColorBlendState = colorBlendEnabled ? &colorBlendState : nullptr;
        graphicsPipeline.pDynamicState = dynamicStateEnabled ? &dynamicState : nullptr;

        return graphicsPipeline;
    }

    GraphicsPipelineBuilder() {
        vertexInputState.sType = VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO;
        inputAssemblyState.sType = VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO;
        tesselationState.sType = VK_STRUCTURE_TYPE_PIPELINE_TESSELLATION_STATE_CREATE_INFO;
        viewportState.sType = VK_STRUCTURE_TYPE_PIPELINE_VIEWPORT_STATE_CREATE_INFO;
        rasterizationState.sType = VK_STRUCTURE_TYPE_PIPELINE_RASTERIZATION_STATE_CREATE_INFO;
        multisampleState.sType = VK_STRUCTURE_TYPE_PIPELINE_MULTISAMPLE_STATE_CREATE_INFO;
        depthStencilState.sType = VK_STRUCTURE_TYPE_PIPELINE_DEPTH_STENCIL_STATE_CREATE_INFO;
        colorBlendState.sType = VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO;
        dynamicState.sType = VK_STRUCTURE_TYPE_PIPELINE_DYNAMIC_STATE_CREATE_INFO;

        graphicsPipeline.sType = VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO;
    }
};

using Layer = Extension;

bool enumerateInstanceExtensionProperties(const char* layerName, std::vector<VkExtensionProperties>& extensionProperties);
bool enumerateInstanceLayerProperties(std::vector<VkLayerProperties>& layerProperties);

bool createInstance(VkInstanceCreateFlags flags, std::vector<Extension>& extensions, std::vector<Layer>& layers, uint32_t apiVersion, VkAllocationCallbacks const* pAllocator, VkInstance& instance);

bool enumeratePhysicalDevices(VkInstance instance, std::vector<VkPhysicalDevice>& physicalDevices);
bool enumerateDeviceExtensionProperties(VkPhysicalDevice physicalDevice, const char* layerName, std::vector<VkExtensionProperties>& extensions);

void getPhysicalDeviceQueueFamilyProperties(VkPhysicalDevice physicalDevice, std::vector<VkQueueFamilyProperties>& queueProperties);
bool getPhysicalDeviceSurfaceFormats(VkPhysicalDevice physicalDevice, VkSurfaceKHR surface, std::vector<VkSurfaceFormatKHR>& formats);
bool getPhysicalDeviceSurfacePresentModes(VkPhysicalDevice physicalDevice, VkSurfaceKHR surface, std::vector<VkPresentModeKHR>& presentModes);

bool createDevice(VkPhysicalDevice physicalDevice, std::vector<VkDeviceQueueCreateInfo> const& queueCreateInfos, std::vector<Extension>& extensions, void* pNext, VkPhysicalDeviceFeatures* pEnabledFeatures, VkAllocationCallbacks const* pAllocator, VkDevice& device);

bool createFence(VkDevice device, bool signaled, VkFence& fence);
bool createSemaphore(VkDevice device, VkAllocationCallbacks* pAllocator, VkSemaphore& semaphore);

/* will revert to VK_FORMAT_R8G8B8A8_SRGB or VK_FORMAT_B8G8R8A8_SRGB, VK_COLOR_SPACE_SRGB_NONLINEAR_KHR, or VK_PRESENT_MODE_FIFO_KHR if preferred values were not found and preferrenceFail is false */
bool createSwapchainForSurface(VkDevice device, VkPhysicalDevice physicalDevice, VkSurfaceKHR surface, VkAllocationCallbacks* pAllocator, std::vector<VkSurfaceFormatKHR>& preferredSurfaceFormats, std::vector<VkPresentModeKHR>& preferredPresentModes, uint32_t preferredImageCount, VkExtent2D fallbackPreferredExtent, bool preferenceFail, VkImageUsageFlags imageUsage, VkSurfaceTransformFlagBitsKHR preTransform, VkSwapchainKHR oldSwapchain, VkSwapchainKHR& swapchain);
bool getSwapchainImages(VkDevice device, VkSwapchainKHR swapchain, std::vector<VkImage>& images);

bool createShaderSPIRV(VkDevice device, const char* path, VkShaderModule& shader);

bool createComputePipeline(VkDevice device, VkShaderModule shaderModule, const char* pShaderEntry, VkPipelineLayout layout, VkAllocationCallbacks* pAllocator, VkPipeline& pipeline);

bool findMemoryTypeIndex(VkPhysicalDevice physicalDevice, uint32_t typeFilter, VkMemoryPropertyFlags memoryFlags, uint32_t& typeIndex);

bool createImageSimple(VkDevice device, VkImageType type, VkFormat format, VkExtent3D extent, VkImageUsageFlags usage, VkImageLayout initialLayout, VkAllocationCallbacks* pAllocator, VkImage& image);
bool allocateImageSimple(VkDevice device, VkPhysicalDevice physicalDevice, VkImage image, VkMemoryPropertyFlags memoryFlags, VkAllocationCallbacks* pAllocator, VkDeviceMemory& memory);
bool createImageViewSimple(VkDevice device, VkImage image, VkImageViewType viewType, VkFormat format, VkImageAspectFlags aspectMask, VkAllocationCallbacks* pAllocator, VkImageView& imageView);

bool createBufferSimple(VkDevice device, VkDeviceSize size, VkBufferUsageFlags usage, VkAllocationCallbacks* pAllocator, VkBuffer& buffer);
bool allocateBufferSimple(VkDevice device, VkPhysicalDevice physicalDevice, VkBuffer buffer, VkMemoryPropertyFlags memoryFlags, VkAllocationCallbacks* pAllocator, VkDeviceMemory& memory);
bool uploadPrivateBufferData(VkDevice device, VkPhysicalDevice physicalDevice, VkBuffer buffer, VkDeviceSize dstOffset, VkDeviceSize srcSize, void* pSrcData, VkQueue queue, VkCommandBuffer commandBuffer, VkAllocationCallbacks* pAllocator);
bool copyBuffer(VkQueue queue, VkCommandBuffer commandBuffer, VkDeviceSize srcOffset, VkDeviceSize srcSize, VkBuffer srcBuffer, VkDeviceSize dstOffset, VkBuffer dstBuffer);

bool createDescriptorSetLayout(VkDevice device, VkDescriptorSetLayoutCreateFlags flags, DescriptorSetLayoutBuilder builder, VkAllocationCallbacks* pAllocator, VkDescriptorSetLayout& setLayout);

void cmdImageBarrierSimple(VkCommandBuffer commandBuffer, VkPipelineStageFlags srcStage, VkPipelineStageFlags dstStage, VkDependencyFlags dependencyFlags, VkAccessFlags srcAccess, VkAccessFlags dstAccess, VkImageLayout oldLayout, VkImageLayout newLayout, VkImage image, VkImageAspectFlags aspect);
void cmdBlitImageSimple(VkCommandBuffer commandBuffer, VkImage srcImage, VkImageLayout srcLayout, VkImage dstImage, VkImageLayout dstLayout, VkFilter filter, Section3D section, VkImageAspectFlags srcAspect, VkImageAspectFlags dstAspect);

} /* namespace vk */

} /* namespace prism */

#ifdef PRISM_VK_IMPL

#include <fstream>

namespace prism {

namespace vk {

bool enumerateInstanceExtensionProperties(const char* layerName, std::vector<VkExtensionProperties>& extensionProperties) {
    uint32_t count;
    VkResult result = vkEnumerateInstanceExtensionProperties(layerName, &count, nullptr);
    if (result != VK_SUCCESS) {
        return false;
    }
    
    extensionProperties.resize(count);
    result = vkEnumerateInstanceExtensionProperties(layerName, &count, &extensionProperties[0]);
    return result == VK_SUCCESS;
}

bool enumerateInstanceLayerProperties(std::vector<VkLayerProperties>& layerProperties) {
    uint32_t count;
    VkResult result = vkEnumerateInstanceLayerProperties(&count, nullptr);
    if (result != VK_SUCCESS) {
        return false;
    }
    
    layerProperties.resize(count);
    result = vkEnumerateInstanceLayerProperties(&count, &layerProperties[0]);
    return result == VK_SUCCESS;
}

bool createInstance(VkInstanceCreateFlags flags, std::vector<Extension>& extensions, std::vector<Layer>& layers, uint32_t apiVersion, VkAllocationCallbacks const* pAllocator, VkInstance& instance) {
    std::vector<VkExtensionProperties> availableExtensions;
    if (!enumerateInstanceExtensionProperties(nullptr, availableExtensions)) {
        return false;
    }
    
    std::vector<VkLayerProperties> availableLayers;
    if (!enumerateInstanceLayerProperties(availableLayers)) {
        return false;
    }
    
    std::vector<const char*> enabledExtensions;
    enabledExtensions.reserve(extensions.size());
    for (size_t i = 0; i < extensions.size(); ++i) {
        Extension const& e = extensions[i];
        
        bool found = false;
        for (VkExtensionProperties const& p : availableExtensions) {
            if (e.name == p.extensionName) {
                found = true;
                enabledExtensions.push_back(p.extensionName);
                break;
            }
        }
        
        if (!found) {
            if (e.required) {
                return false;
            } else {
                extensions.erase(extensions.begin() + i);
                --i;
            }
        }
    }
    
    std::vector<const char*> enabledLayers;
    enabledLayers.reserve(layers.size());
    for (size_t i = 0; i < layers.size(); ++i) {
        Layer const& l = layers[i];
        bool found = false;
        for (VkLayerProperties const& p : availableLayers) {
            if (l.name == p.layerName) {
                found = true;
                enabledLayers.push_back(p.layerName);
                break;
            }
        }
        
        if (!found) {
            if (l.required) {
                return false;
            } else {
                layers.erase(layers.begin() + i);
                --i;
            }
        }
    }
    
    VkApplicationInfo appInfo = {};
    appInfo.sType = VK_STRUCTURE_TYPE_APPLICATION_INFO;
    appInfo.pApplicationName = "Prism";
    appInfo.applicationVersion = VK_MAKE_API_VERSION(0, 1, 0, 0);
    appInfo.pEngineName = "Prism";
    appInfo.engineVersion = VK_MAKE_API_VERSION(0, 1, 0, 0);
    appInfo.apiVersion = VK_API_VERSION_1_2;
    
    VkInstanceCreateInfo createInfo = {};
    createInfo.sType = VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO;
    createInfo.flags = flags;
    createInfo.pApplicationInfo = &appInfo;
    createInfo.enabledLayerCount = static_cast<uint32_t>(enabledLayers.size());
    createInfo.ppEnabledLayerNames = enabledLayers.data();
    createInfo.enabledExtensionCount = static_cast<uint32_t>(enabledExtensions.size());
    createInfo.ppEnabledExtensionNames = enabledExtensions.data();
    
    VkResult result = vkCreateInstance(&createInfo, pAllocator, &instance);
    return result == VK_SUCCESS;
}

bool enumeratePhysicalDevices(VkInstance instance, std::vector<VkPhysicalDevice>& physicalDevices) {
    uint32_t count;
    VkResult result = vkEnumeratePhysicalDevices(instance, &count, nullptr);
    if (result != VK_SUCCESS) {
        return false;
    }
    
    physicalDevices.resize(count);
    result = vkEnumeratePhysicalDevices(instance, &count, &physicalDevices[0]);
    return result == VK_SUCCESS;
}

bool enumerateDeviceExtensionProperties(VkPhysicalDevice physicalDevice, const char* layerName, std::vector<VkExtensionProperties>& extensions) {
    uint32_t count;
    VkResult result = vkEnumerateDeviceExtensionProperties(physicalDevice, layerName, &count, nullptr);
    if (result != VK_SUCCESS) {
        return false;
    }
    
    extensions.resize(count);
    result = vkEnumerateDeviceExtensionProperties(physicalDevice, layerName, &count, &extensions[0]);
    return result == VK_SUCCESS;
}

void getPhysicalDeviceQueueFamilyProperties(VkPhysicalDevice physicalDevice, std::vector<VkQueueFamilyProperties>& queueProperties) {
    uint32_t count;
    vkGetPhysicalDeviceQueueFamilyProperties(physicalDevice, &count, nullptr);
    
    queueProperties.resize(count);
    vkGetPhysicalDeviceQueueFamilyProperties(physicalDevice, &count, &queueProperties[0]);
}

bool createDevice(VkPhysicalDevice physicalDevice, std::vector<VkDeviceQueueCreateInfo> const& queueCreateInfos, std::vector<Extension>& extensions, void* pNext, VkPhysicalDeviceFeatures* pEnabledFeatures, VkAllocationCallbacks const* pAllocator, VkDevice& device) {
    std::vector<VkExtensionProperties> availableExtensions;
    if (!enumerateDeviceExtensionProperties(physicalDevice, nullptr, availableExtensions)) {
        return false;
    }
    
    std::vector<const char*> enabledExtensions;
    enabledExtensions.reserve(extensions.size());
    for (size_t i = 0; i < extensions.size(); ++i) {
        Extension const& e = extensions[i];
        
        bool found = false;
        for (VkExtensionProperties const& p : availableExtensions) {
            if (e.name == p.extensionName) {
                found = true;
                enabledExtensions.push_back(p.extensionName);
                break;
            }
        }
        
        if (!found) {
            if (e.required) {
                return false;
            } else {
                extensions.erase(extensions.begin() + i);
                --i;
            }
        }
    }
    
    VkDeviceCreateInfo createInfo = {};
    createInfo.sType = VK_STRUCTURE_TYPE_DEVICE_CREATE_INFO;
    createInfo.pNext = pNext;
    createInfo.queueCreateInfoCount = static_cast<uint32_t>(queueCreateInfos.size());
    createInfo.pQueueCreateInfos = queueCreateInfos.data();
    createInfo.enabledLayerCount = 0; /* deprecated */
    createInfo.ppEnabledLayerNames = nullptr; /* deprecated */
    createInfo.enabledExtensionCount = static_cast<uint32_t>(enabledExtensions.size());
    createInfo.ppEnabledExtensionNames = enabledExtensions.data();
    createInfo.pEnabledFeatures = pEnabledFeatures;
    
    VkResult result = vkCreateDevice(physicalDevice, &createInfo, pAllocator, &device);
    return result == VK_SUCCESS;
}

bool createFence(VkDevice device, bool signaled, VkAllocationCallbacks* pAllocator, VkFence& fence) {
    VkFenceCreateInfo createInfo = {};
    createInfo.sType = VK_STRUCTURE_TYPE_FENCE_CREATE_INFO;
    createInfo.flags = signaled ? VK_FENCE_CREATE_SIGNALED_BIT : 0;
    
    VkResult result = vkCreateFence(device, &createInfo, pAllocator, &fence);
    return result == VK_SUCCESS;
}

bool createSemaphore(VkDevice device, VkAllocationCallbacks* pAllocator, VkSemaphore& semaphore) {
    VkSemaphoreCreateInfo createInfo = {};
    createInfo.sType = VK_STRUCTURE_TYPE_SEMAPHORE_CREATE_INFO;
    
    VkResult result = vkCreateSemaphore(device, &createInfo, pAllocator, &semaphore);
    return result == VK_SUCCESS;
}

bool getPhysicalDeviceSurfaceFormats(VkPhysicalDevice physicalDevice, VkSurfaceKHR surface, std::vector<VkSurfaceFormatKHR>& formats) {
    uint32_t count;
    VkResult result = vkGetPhysicalDeviceSurfaceFormatsKHR(physicalDevice, surface, &count, nullptr);
    if (result != VK_SUCCESS) {
        return false;
    }

    formats.resize(count);
    result = vkGetPhysicalDeviceSurfaceFormatsKHR(physicalDevice, surface, &count, &formats[0]);
    return result == VK_SUCCESS;
}

bool getPhysicalDeviceSurfacePresentModes(VkPhysicalDevice physicalDevice, VkSurfaceKHR surface, std::vector<VkPresentModeKHR>& presentModes) {
    uint32_t count;
    VkResult result = vkGetPhysicalDeviceSurfacePresentModesKHR(physicalDevice, surface, &count, nullptr);
    if (result != VK_SUCCESS) {
        return false;
    }

    presentModes.resize(count);
    result = vkGetPhysicalDeviceSurfacePresentModesKHR(physicalDevice, surface, &count, &presentModes[0]);
    return result == VK_SUCCESS;
}

bool createSwapchainForSurface(VkDevice device, VkPhysicalDevice physicalDevice, VkSurfaceKHR surface, VkAllocationCallbacks* pAllocator, std::vector<VkSurfaceFormatKHR>& preferredSurfaceFormats, std::vector<VkPresentModeKHR>& preferredPresentModes, uint32_t preferredImageCount, VkExtent2D fallbackPreferredExtent, bool preferenceFail, VkImageUsageFlags imageUsage, VkSurfaceTransformFlagBitsKHR preTransform, VkSwapchainKHR oldSwapchain, VkSwapchainKHR& swapchain) {
    VkSurfaceCapabilitiesKHR capabilities;
    VkResult result = vkGetPhysicalDeviceSurfaceCapabilitiesKHR(physicalDevice, surface, &capabilities);
    if (result != VK_SUCCESS) {
        return false;
    }

    if (capabilities.maxImageCount == 0) {
        capabilities.maxImageCount = UINT32_MAX;
    }
    
    VkSwapchainCreateInfoKHR createInfo = {};
    createInfo.sType = VK_STRUCTURE_TYPE_SWAPCHAIN_CREATE_INFO_KHR;
    createInfo.surface = surface;
    
    if (preferredImageCount < capabilities.minImageCount || preferredImageCount > capabilities.maxImageCount) {
        if (preferenceFail) {
            return false;
        }
        
        createInfo.minImageCount = std::min(std::max(preferredImageCount, capabilities.minImageCount), capabilities.maxImageCount);
    } else {
        createInfo.minImageCount = preferredImageCount;
    }
    
    std::vector<VkSurfaceFormatKHR> availableFormats;
    if (!getPhysicalDeviceSurfaceFormats(physicalDevice, surface, availableFormats)) {
        return false;
    }
    
    preferredSurfaceFormats.erase(std::remove_if(preferredSurfaceFormats.begin(), preferredSurfaceFormats.end(), [availableFormats](VkSurfaceFormatKHR const& sf) {
        for (VkSurfaceFormatKHR const& a : availableFormats) {
            if (a.format == sf.format && a.colorSpace == sf.colorSpace) {
                return false;
            }
        }

        return true;
    }), preferredSurfaceFormats.end());

    if (preferredSurfaceFormats.empty()) {
        if (preferenceFail) {
            return false;
        }

        createInfo.imageFormat = availableFormats[0].format;
        createInfo.imageColorSpace = availableFormats[0].colorSpace;
    } else {
        createInfo.imageFormat = preferredSurfaceFormats[0].format;
        createInfo.imageColorSpace = preferredSurfaceFormats[0].colorSpace;
    }

    if (capabilities.currentExtent.width == UINT32_MAX || capabilities.currentExtent.height == UINT32_MAX) {
        if (fallbackPreferredExtent.width > capabilities.maxImageExtent.width || fallbackPreferredExtent.height > capabilities.maxImageExtent.height || fallbackPreferredExtent.width < capabilities.minImageExtent.width || fallbackPreferredExtent.height < capabilities.minImageExtent.height) {
            if (preferenceFail) {
                return false;
            }

            createInfo.imageExtent.width = std::min(std::max(fallbackPreferredExtent.width, capabilities.minImageExtent.width), capabilities.maxImageExtent.width);
            createInfo.imageExtent.height = std::min(std::max(fallbackPreferredExtent.height, capabilities.minImageExtent.height), capabilities.maxImageExtent.height);
        } else {
            createInfo.imageExtent = fallbackPreferredExtent;
        }
    } else {
        createInfo.imageExtent = capabilities.currentExtent;
    }

    createInfo.imageArrayLayers = 1;
    createInfo.imageUsage = imageUsage;
    createInfo.imageSharingMode = VK_SHARING_MODE_EXCLUSIVE;
    createInfo.preTransform = preTransform;
    createInfo.compositeAlpha = VK_COMPOSITE_ALPHA_OPAQUE_BIT_KHR;
    
    std::vector<VkPresentModeKHR> availablePresentModes;
    if (!getPhysicalDeviceSurfacePresentModes(physicalDevice, surface, availablePresentModes)) {
        return false;
    }

    preferredPresentModes.erase(std::remove_if(preferredPresentModes.begin(), preferredPresentModes.end(), [availablePresentModes](VkPresentModeKHR const& pm) {
        for (VkPresentModeKHR const& a : availablePresentModes) {
            if (a == pm) {
                return false;
            }
        }

        return true;
    }), preferredPresentModes.end());

    if (preferredPresentModes.empty()) {
        if (preferenceFail) {
            return false;
        } else {
            createInfo.presentMode = VK_PRESENT_MODE_FIFO_KHR;
        }
    } else {
        createInfo.presentMode = preferredPresentModes[0];
    }

    createInfo.clipped = VK_FALSE;
    createInfo.oldSwapchain = oldSwapchain;

    result = vkCreateSwapchainKHR(device, &createInfo, pAllocator, &swapchain);
    return result == VK_SUCCESS;
}

bool getSwapchainImages(VkDevice device, VkSwapchainKHR swapchain, std::vector<VkImage>& images) {
    uint32_t count;
    VkResult result = vkGetSwapchainImagesKHR(device, swapchain, &count, nullptr);
    if (result != VK_SUCCESS) {
        return false;
    }
    
    images.resize(count);
    result = vkGetSwapchainImagesKHR(device, swapchain, &count, &images[0]);
    return result == VK_SUCCESS;
}

bool createShaderSPIRV(VkDevice device, const char* path, VkAllocationCallbacks* pAllocator, VkShaderModule& shader) {
    std::ifstream file(path, std::ios::in | std::ios::binary);
    if (!file.good()) {
        return false;
    }
    
    file.seekg(0, file.end);
    size_t size = file.tellg();
    file.seekg(0, file.beg);
    
    uint8_t* data = new uint8_t[size];
    file.read(reinterpret_cast<char*>(data), size);
    file.close();
    
    VkShaderModuleCreateInfo createInfo = {};
    createInfo.sType = VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO;
    createInfo.codeSize = size;
    createInfo.pCode = const_cast<const uint32_t*>(reinterpret_cast<uint32_t*>(data));
    
    VkResult result = vkCreateShaderModule(device, &createInfo, pAllocator, &shader);
    return result == VK_SUCCESS;
}

bool createComputePipeline(VkDevice device, VkShaderModule shaderModule, const char* pShaderEntry, VkPipelineLayout layout, VkAllocationCallbacks* pAllocator, VkPipeline& pipeline) {
    VkComputePipelineCreateInfo createInfo = {};
    createInfo.sType = VK_STRUCTURE_TYPE_COMPUTE_PIPELINE_CREATE_INFO;
    createInfo.stage.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
    createInfo.stage.stage = VK_SHADER_STAGE_COMPUTE_BIT;
    createInfo.stage.module = shaderModule;
    createInfo.stage.pName = pShaderEntry;
    createInfo.layout = layout;

    VkResult result = vkCreateComputePipelines(device, nullptr, 1, &createInfo, pAllocator, &pipeline);
    return result == VK_SUCCESS;
}

bool findMemoryTypeIndex(VkPhysicalDevice physicalDevice, uint32_t typeFilter, VkMemoryPropertyFlags memoryFlags, uint32_t& typeIndex) {
    VkPhysicalDeviceMemoryProperties props;
    vkGetPhysicalDeviceMemoryProperties(physicalDevice, &props);
    
    for (uint32_t i = 0; i < props.memoryTypeCount; ++i) {
        if ((typeFilter & (1 << i)) && (props.memoryTypes[i].propertyFlags & memoryFlags) == memoryFlags) {
            typeIndex = i;
            return true;
        }
    }
    
    return false;
}

bool createImageSimple(VkDevice device, VkImageType type, VkFormat format, VkExtent3D extent, VkImageUsageFlags usage, VkImageLayout initialLayout, VkAllocationCallbacks* pAllocator, VkImage& image) {
    VkImageCreateInfo createInfo = {};
    createInfo.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO;
    createInfo.imageType = type;
    createInfo.format = format;
    createInfo.extent = extent;
    createInfo.mipLevels = 1;
    createInfo.arrayLayers = 1;
    createInfo.samples = VK_SAMPLE_COUNT_1_BIT;
    createInfo.tiling = VK_IMAGE_TILING_OPTIMAL;
    createInfo.usage = usage;
    createInfo.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
    createInfo.initialLayout = initialLayout;
    
    VkResult result = vkCreateImage(device, &createInfo, pAllocator, &image);
    return result == VK_SUCCESS;
}

bool allocateImageSimple(VkDevice device, VkPhysicalDevice physicalDevice, VkImage image, VkMemoryPropertyFlags memoryFlags, VkAllocationCallbacks* pAllocator, VkDeviceMemory& memory) {
    VkMemoryRequirements reqs;
    vkGetImageMemoryRequirements(device, image, &reqs);
    
    VkMemoryAllocateInfo allocInfo = {};
    allocInfo.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
    allocInfo.allocationSize = reqs.size;
    
    if (!findMemoryTypeIndex(physicalDevice, reqs.memoryTypeBits, memoryFlags, allocInfo.memoryTypeIndex)) {
        return false;
    }
    
    VkResult result = vkAllocateMemory(device, &allocInfo, pAllocator, &memory);
    return result == VK_SUCCESS;
}

bool createImageViewSimple(VkDevice device, VkImage image, VkImageViewType viewType, VkFormat format, VkImageAspectFlags aspectMask, VkAllocationCallbacks* pAllocator, VkImageView& imageView) {
    VkImageViewCreateInfo createInfo = {};
    createInfo.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
    createInfo.image = image;
    createInfo.viewType = viewType;
    createInfo.format = format;
    createInfo.components.r = VK_COMPONENT_SWIZZLE_IDENTITY;
    createInfo.components.g = VK_COMPONENT_SWIZZLE_IDENTITY;
    createInfo.components.b = VK_COMPONENT_SWIZZLE_IDENTITY;
    createInfo.components.a = VK_COMPONENT_SWIZZLE_IDENTITY;
    createInfo.subresourceRange.aspectMask = aspectMask;
    createInfo.subresourceRange.baseMipLevel = 0;
    createInfo.subresourceRange.levelCount = 1;
    createInfo.subresourceRange.baseArrayLayer = 0;
    createInfo.subresourceRange.layerCount = 1;
    
    VkResult result = vkCreateImageView(device, &createInfo, pAllocator, &imageView);
    return result == VK_SUCCESS;
}

bool createBufferSimple(VkDevice device, VkDeviceSize size, VkBufferUsageFlags usage, VkAllocationCallbacks* pAllocator, VkBuffer& buffer) {
    VkBufferCreateInfo createInfo = {};
    createInfo.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
    createInfo.size = size;
    createInfo.usage = usage;
    createInfo.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
    
    VkResult result = vkCreateBuffer(device, &createInfo, pAllocator, &buffer);
    return result == VK_SUCCESS;
}

bool allocateBufferSimple(VkDevice device, VkPhysicalDevice physicalDevice, VkBuffer buffer, VkMemoryPropertyFlags memoryFlags, VkAllocationCallbacks* pAllocator, VkDeviceMemory& memory) {
    VkMemoryRequirements reqs;
    vkGetBufferMemoryRequirements(device, buffer, &reqs);
    
    VkMemoryAllocateInfo allocInfo = {};
    allocInfo.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
    allocInfo.allocationSize = reqs.size;
    
    if (!findMemoryTypeIndex(physicalDevice, reqs.memoryTypeBits, memoryFlags, allocInfo.memoryTypeIndex)) {
        return false;
    }
    
    VkResult result = vkAllocateMemory(device, &allocInfo, pAllocator, &memory);
    return result == VK_SUCCESS;
}

bool uploadPrivateBufferData(VkDevice device, VkPhysicalDevice physicalDevice, VkBuffer buffer, VkDeviceSize dstOffset, VkDeviceSize srcSize, void* pSrcData, VkQueue queue, VkCommandBuffer commandBuffer, VkAllocationCallbacks* pAllocator) {
    VkBuffer srcBuffer;
    if (!createBufferSimple(device, srcSize, VK_BUFFER_USAGE_TRANSFER_SRC_BIT, pAllocator, srcBuffer)) {
        return false;
    }
    
    VkDeviceMemory srcMemory;
    if (!allocateBufferSimple(device, physicalDevice, srcBuffer, VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT, pAllocator, srcMemory)) {
        vkDestroyBuffer(device, srcBuffer, pAllocator);
        return false;
    }
    
    if (vkBindBufferMemory(device, srcBuffer, srcMemory, 0) != VK_SUCCESS) {
        vkFreeMemory(device, srcMemory, pAllocator);
        vkDestroyBuffer(device, srcBuffer, pAllocator);
        return false;
    }
    
    void* srcMapped;
    if (vkMapMemory(device, srcMemory, 0, srcSize, 0, &srcMapped) != VK_SUCCESS) {
        vkDestroyBuffer(device, srcBuffer, pAllocator);
        vkFreeMemory(device, srcMemory, pAllocator);
        return false;
    }
    
    memcpy(srcMapped, pSrcData, srcSize);
    vkUnmapMemory(device, srcMemory);
    
    bool result = copyBuffer(queue, commandBuffer, 0, srcSize, srcBuffer, dstOffset, buffer);
    vkQueueWaitIdle(queue);
    if (!result) {
        vkDestroyBuffer(device, srcBuffer, pAllocator);
        vkFreeMemory(device, srcMemory, pAllocator);
        return false;
    }
    
    vkDestroyBuffer(device, srcBuffer, pAllocator);
    vkFreeMemory(device, srcMemory, pAllocator);
    return true;
}

bool copyBuffer(VkQueue queue, VkCommandBuffer commandBuffer, VkDeviceSize srcOffset, VkDeviceSize srcSize, VkBuffer srcBuffer, VkDeviceSize dstOffset, VkBuffer dstBuffer) {
    VkCommandBufferBeginInfo beginInfo = {};
    beginInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;

    VkResult result = vkBeginCommandBuffer(commandBuffer, &beginInfo);
    if (result != VK_SUCCESS) {
        return false;
    }

    VkBufferCopy region = {};
    region.srcOffset = srcOffset;
    region.dstOffset = dstOffset;
    region.size = srcSize;

    vkCmdCopyBuffer(commandBuffer, srcBuffer, dstBuffer, 1, &region);

    result = vkEndCommandBuffer(commandBuffer);
    if (result != VK_SUCCESS) {
        return false;
    }

    VkSubmitInfo submitInfo = {};
    submitInfo.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
    submitInfo.commandBufferCount = 1;
    submitInfo.pCommandBuffers = &commandBuffer;

    result = vkQueueSubmit(queue, 1, &submitInfo, nullptr);
    return result == VK_SUCCESS;
}

bool createDescriptorSetLayout(VkDevice device, VkDescriptorSetLayoutCreateFlags flags, DescriptorSetLayoutBuilder builder, VkAllocationCallbacks* pAllocator, VkDescriptorSetLayout& setLayout) {
    VkDescriptorSetLayoutCreateInfo createInfo = {};
    createInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
    createInfo.flags = flags;
    createInfo.bindingCount = static_cast<uint32_t>(builder.bindings.size());
    createInfo.pBindings = builder.bindings.data();

    VkResult result = vkCreateDescriptorSetLayout(device, &createInfo, pAllocator, &setLayout);
    return result == VK_SUCCESS;
}

void cmdImageBarrierSimple(VkCommandBuffer commandBuffer, VkPipelineStageFlags srcStage, VkPipelineStageFlags dstStage, VkDependencyFlags dependencyFlags, VkAccessFlags srcAccess, VkAccessFlags dstAccess, VkImageLayout oldLayout, VkImageLayout newLayout, VkImage image, VkImageAspectFlags aspect) {
    VkImageMemoryBarrier barrier = {};
    barrier.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER;
    barrier.srcAccessMask = srcAccess;
    barrier.dstAccessMask = dstAccess;
    barrier.oldLayout = oldLayout;
    barrier.newLayout = newLayout;
    barrier.image = image;
    barrier.subresourceRange.aspectMask = aspect;
    barrier.subresourceRange.baseMipLevel = 0;
    barrier.subresourceRange.levelCount = 1;
    barrier.subresourceRange.baseArrayLayer = 0;
    barrier.subresourceRange.layerCount = 1;

    vkCmdPipelineBarrier(commandBuffer, srcStage, dstStage, dependencyFlags, 0, nullptr, 0, nullptr, 1, &barrier);
}

void cmdBlitImageSimple(VkCommandBuffer commandBuffer, VkImage srcImage, VkImageLayout srcLayout, VkImage dstImage, VkImageLayout dstLayout, VkFilter filter, Section3D section, VkImageAspectFlags srcAspect, VkImageAspectFlags dstAspect) {
    VkImageBlit region = {};
    region.srcSubresource.aspectMask = srcAspect;
    region.srcSubresource.baseArrayLayer = 0;
    region.srcSubresource.layerCount = 1;
    region.srcSubresource.mipLevel = 0;
    region.srcOffsets[0].x = section.x;
    region.srcOffsets[0].y = section.y;
    region.srcOffsets[0].z = section.z;
    region.srcOffsets[1].x = section.x + section.width;
    region.srcOffsets[1].y = section.y + section.height;
    region.srcOffsets[1].z = section.z + section.depth;
    region.dstSubresource.aspectMask = dstAspect;
    region.dstSubresource.baseArrayLayer = 0;
    region.dstSubresource.layerCount = 1;
    region.dstSubresource.mipLevel = 0;
    region.dstOffsets[0].x = section.x;
    region.dstOffsets[0].y = section.y;
    region.dstOffsets[0].z = section.z;
    region.dstOffsets[1].x = section.x + section.width;
    region.dstOffsets[1].y = section.y + section.height;
    region.dstOffsets[1].z = section.z + section.depth;

    vkCmdBlitImage(commandBuffer, srcImage, srcLayout, dstImage, dstLayout, 1, &region, filter);
}

} /* namespace vk */

} /* namespace prism */

#endif /* #ifdef PRISM_VK_IMPL */

#endif /* #ifndef PRISM_VK_HPP */
