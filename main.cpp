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

    while (!glfwWindowShouldClose(window)) {
        glfwPollEvents();
    }

    kobalt::destroy(device);
    kobalt::deinit();

    glfwDestroyWindow(window);
    glfwTerminate();

    return 0;
}
