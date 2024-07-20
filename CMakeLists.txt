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

set(BUILD_SHARED_LIBS OFF)

# Find ZED and CUDA
find_package(ZED 3 REQUIRED)
find_package(CUDAToolkit ${ZED_CUDA_VERSION} REQUIRED)

include_directories(${ZED_INCLUDE_DIRS})
link_directories(${ZED_LIBRARY_DIR})

include(FetchContent)

FetchContent_Declare(
  fmt
  GIT_REPOSITORY https://github.com/fmtlib/fmt
  GIT_TAG        0c9fce2ffefecfdce794e1859584e25877b7b592) # 11.0.2
FetchContent_MakeAvailable(fmt)


set(BUILD_EXAMPLES OFF)
set(BUILD_PYTHON_WRAPPER OFF) 
FetchContent_Declare(
  apriltag
  GIT_REPOSITORY https://github.com/AprilRobotics/apriltag
  GIT_TAG        3806edf38ac4400153677e510c9f9dcb81f472c8) # Current master as of 2024-07-20
FetchContent_MakeAvailable(apriltag)


find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})

# Create executable
add_executable(zed-test ${SOURCES})

target_link_libraries(zed-test CUDA::toolkit apriltag fmt::fmt ${ZED_LIBRARIES} ${OpenCV_LIBRARIES})