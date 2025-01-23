cmake_minimum_required(VERSION 3.13)
project(kobalt)

find_package(Vulkan REQUIRED)
find_package(glfw3 CONFIG REQUIRED)

add_executable(kobalt main.cpp kobalt.h kobalt.cpp)

target_link_directories(kobalt PRIVATE ${Vulkan_LIBRARY})
target_link_libraries(kobalt PRIVATE ${Vulkan_LIBRARIES})
target_include_directories(kobalt PRIVATE ${Vulkan_INCLUDE_DIRS})

target_link_libraries(kobalt PRIVATE glfw)

set_property(TARGET kobalt PROPERTY CXX_STANDARD 11)
