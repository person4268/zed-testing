cmake_minimum_required(VERSION 3.12)


set(CMAKE_SYSTEM_PROCESSOR x64)
set(CMAKE_CL_64 TRUE) # Zed SDK needs this and our toolchain file doesn't set it
set(CMAKE_TOOLCHAIN_FILE "${CMAKE_CURRENT_LIST_DIR}/cmake/Windows.MSVC.toolchain.cmake"
    CACHE STRING "")

project(zed-test LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Add your source files here
set(SOURCES main.cpp)

# Find ZED and CUDA
find_package(ZED 3 REQUIRED)
find_package(CUDAToolkit ${ZED_CUDA_VERSION} REQUIRED)

include_directories(${ZED_INCLUDE_DIRS})
link_directories(${ZED_LIBRARY_DIR})

# Create executable
add_executable(zed-test ${SOURCES})

# Link against ZED SDK
target_link_libraries(zed-test CUDA::toolkit ${ZED_LIBRARIES})