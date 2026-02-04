cmake_minimum_required(VERSION 3.10)
project(JetsonRLDeploy LANGUAGES CXX CUDA)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ============================================================
# CUDA & TensorRT 配置
# ============================================================
find_package(CUDA REQUIRED)

# TensorRT路径 (根据实际安装位置修改)
set(TENSORRT_ROOT "/usr/local/tensorrt" CACHE PATH "TensorRT root directory")
if(NOT EXISTS ${TENSORRT_ROOT})
    set(TENSORRT_ROOT "/usr")
endif()

find_path(TENSORRT_INCLUDE_DIR NvInfer.h
    HINTS ${TENSORRT_ROOT}/include /usr/include/aarch64-linux-gnu /usr/include/x86_64-linux-gnu)
find_library(TENSORRT_LIBRARY nvinfer
    HINTS ${TENSORRT_ROOT}/lib ${TENSORRT_ROOT}/lib64 /usr/lib/aarch64-linux-gnu /usr/lib/x86_64-linux-gnu)

if(NOT TENSORRT_INCLUDE_DIR OR NOT TENSORRT_LIBRARY)
    message(FATAL_ERROR "TensorRT not found. Set TENSORRT_ROOT or install TensorRT.")
endif()

message(STATUS "CUDA version: ${CUDA_VERSION}")
message(STATUS "TensorRT include: ${TENSORRT_INCLUDE_DIR}")
message(STATUS "TensorRT library: ${TENSORRT_LIBRARY}")

# ============================================================
# 主程序 (TensorRT)
# ============================================================
set(SOURCES
    src/main.cpp
    src/communication.cpp
    src/trt_inference.cpp
)

set(HEADERS
    include/communication.h
    include/trt_inference.h
    include/types.h
)

add_executable(${PROJECT_NAME} ${SOURCES} ${HEADERS})

target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CUDA_INCLUDE_DIRS}
    ${TENSORRT_INCLUDE_DIR}
)

target_link_libraries(${PROJECT_NAME} PRIVATE
    ${TENSORRT_LIBRARY}
    ${CUDA_LIBRARIES}
    ${CUDA_cudart_static_LIBRARY}
)

# ============================================================
# 标定工具
# ============================================================
add_executable(calibration_tool src/calibration_tool.cpp)
target_include_directories(calibration_tool PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)

# ============================================================
# 测试程序
# ============================================================
add_executable(test_udp test/test_udp.cpp)
target_include_directories(test_udp PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)

add_executable(test_motors test/test_motors.cpp)
target_include_directories(test_motors PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)

# TensorRT模型测试
add_executable(test_trt_engine test/test_trt_engine.cpp)
target_include_directories(test_trt_engine PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CUDA_INCLUDE_DIRS}
    ${TENSORRT_INCLUDE_DIR}
)
target_link_libraries(test_trt_engine PRIVATE
    ${TENSORRT_LIBRARY}
    ${CUDA_LIBRARIES}
    ${CUDA_cudart_static_LIBRARY}
)
