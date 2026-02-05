# ONNX to TensorRT 转换工具 CMakeLists.txt

cmake_minimum_required(VERSION 3.10)
project(onnx_to_trt LANGUAGES CXX CUDA)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找 CUDA
find_package(CUDA REQUIRED)
include_directories(${CUDA_INCLUDE_DIRS})

# 查找 TensorRT
# Jetson 上通常在 /usr/include/aarch64-linux-gnu 和 /usr/lib/aarch64-linux-gnu
find_path(TENSORRT_INCLUDE_DIR NvInfer.h
    HINTS /usr/include/aarch64-linux-gnu /usr/include /usr/local/include
)
find_library(TENSORRT_LIBRARY nvinfer
    HINTS /usr/lib/aarch64-linux-gnu /usr/lib /usr/local/lib
)
find_library(NVONNXPARSER_LIBRARY nvonnxparser
    HINTS /usr/lib/aarch64-linux-gnu /usr/lib /usr/local/lib
)

if(NOT TENSORRT_INCLUDE_DIR OR NOT TENSORRT_LIBRARY OR NOT NVONNXPARSER_LIBRARY)
    message(FATAL_ERROR "TensorRT not found!")
endif()

message(STATUS "TensorRT include: ${TENSORRT_INCLUDE_DIR}")
message(STATUS "TensorRT library: ${TENSORRT_LIBRARY}")
message(STATUS "nvonnxparser library: ${NVONNXPARSER_LIBRARY}")

include_directories(${TENSORRT_INCLUDE_DIR})

# 编译目标
add_executable(onnx_to_trt onnx_to_trt.cpp)

target_link_libraries(onnx_to_trt
    ${TENSORRT_LIBRARY}
    ${NVONNXPARSER_LIBRARY}
    ${CUDA_LIBRARIES}
)

# 安装
install(TARGETS onnx_to_trt DESTINATION bin)
