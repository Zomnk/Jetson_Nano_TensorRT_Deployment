/**
 * @file onnx_to_trt.cpp
 * @brief ONNX模型转TensorRT引擎工具 (C++版本)
 * @author Zomnk
 * @date 2026-02-05
 *
 * @note 使用方法:
 *       ./onnx_to_trt model.onnx model.engine [--fp16]
 *
 * @note 编译方法:
 *       g++ -o onnx_to_trt onnx_to_trt.cpp -lnvinfer -lnvonnxparser
 */

#include <NvInfer.h>
#include <NvInferVersion.h>  // 用于版本检测
#include <NvOnnxParser.h>
#include <fstream>
#include <iostream>
#include <memory>
#include <cstring>

using namespace nvinfer1;

/**
 * @brief TensorRT日志记录器
 */
class Logger : public ILogger {
public:
    void log(Severity severity, const char* msg) noexcept override {
        if (severity <= Severity::kWARNING)
            std::cout << "[TRT] " << msg << std::endl;
    }
} gLogger;

/**
 * @brief 将ONNX模型转换为TensorRT引擎
 */
bool convertOnnxToTrt(const std::string& onnx_path,
                      const std::string& engine_path,
                      bool use_fp16) {
    // ========== 1. 创建Builder ==========
    std::unique_ptr<IBuilder> builder(createInferBuilder(gLogger));
    if (!builder) {
        std::cerr << "创建Builder失败" << std::endl;
        return false;
    }

    // ========== 2. 创建Network (显式batch) ==========
    const auto explicitBatch = 1U << static_cast<uint32_t>(
        NetworkDefinitionCreationFlag::kEXPLICIT_BATCH);
    std::unique_ptr<INetworkDefinition> network(
        builder->createNetworkV2(explicitBatch));
    if (!network) {
        std::cerr << "创建Network失败" << std::endl;
        return false;
    }

    // ========== 3. 创建ONNX解析器 ==========
    std::unique_ptr<nvonnxparser::IParser> parser(
        nvonnxparser::createParser(*network, gLogger));
    if (!parser) {
        std::cerr << "创建ONNX解析器失败" << std::endl;
        return false;
    }

    // ========== 4. 解析ONNX文件 ==========
    std::cout << "解析ONNX文件: " << onnx_path << std::endl;
    if (!parser->parseFromFile(onnx_path.c_str(),
            static_cast<int>(ILogger::Severity::kWARNING))) {
        std::cerr << "ONNX解析失败:" << std::endl;
        for (int i = 0; i < parser->getNbErrors(); ++i) {
            std::cerr << "  " << parser->getError(i)->desc() << std::endl;
        }
        return false;
    }
    std::cout << "ONNX解析成功!" << std::endl;

    // 打印网络信息
    std::cout << "网络输入数量: " << network->getNbInputs() << std::endl;
    for (int i = 0; i < network->getNbInputs(); ++i) {
        auto input = network->getInput(i);
        std::cout << "  输入[" << i << "]: " << input->getName()
                  << ", 维度: ";
        auto dims = input->getDimensions();
        for (int j = 0; j < dims.nbDims; ++j) {
            std::cout << dims.d[j];
            if (j < dims.nbDims - 1) std::cout << "x";
        }
        std::cout << std::endl;
    }
    std::cout << "网络输出数量: " << network->getNbOutputs() << std::endl;

    // ========== 5. 配置Builder ==========
    std::unique_ptr<IBuilderConfig> config(builder->createBuilderConfig());
    if (!config) {
        std::cerr << "创建BuilderConfig失败" << std::endl;
        return false;
    }

    // 设置工作空间大小 (1GB)
    // TensorRT 8.4+ 使用 setMemoryPoolLimit，旧版本使用 setMaxWorkspaceSize
#if NV_TENSORRT_MAJOR >= 8 && NV_TENSORRT_MINOR >= 4
    config->setMemoryPoolLimit(MemoryPoolType::kWORKSPACE, 1ULL << 30);
#else
    config->setMaxWorkspaceSize(1ULL << 30);
#endif

    // FP16模式
    if (use_fp16) {
        if (builder->platformHasFastFp16()) {
            std::cout << "启用FP16精度" << std::endl;
            config->setFlag(BuilderFlag::kFP16);
        } else {
            std::cout << "警告: 平台不支持快速FP16，使用FP32" << std::endl;
        }
    } else {
        std::cout << "使用FP32精度" << std::endl;
    }

    // ========== 6. 构建引擎 ==========
    std::cout << "构建TensorRT引擎 (可能需要几分钟)..." << std::endl;
    std::unique_ptr<IHostMemory> serializedEngine(
        builder->buildSerializedNetwork(*network, *config));
    if (!serializedEngine) {
        std::cerr << "引擎构建失败" << std::endl;
        return false;
    }

    // ========== 7. 保存引擎 ==========
    std::cout << "保存引擎: " << engine_path << std::endl;
    std::ofstream file(engine_path, std::ios::binary);
    if (!file) {
        std::cerr << "无法创建文件: " << engine_path << std::endl;
        return false;
    }
    file.write(static_cast<const char*>(serializedEngine->data()),
               serializedEngine->size());
    file.close();

    std::cout << "引擎构建成功! 大小: "
              << serializedEngine->size() / 1024.0 << " KB" << std::endl;
    return true;
}

void printUsage(const char* prog) {
    std::cout << "用法: " << prog << " <onnx_file> <engine_file> [选项]" << std::endl;
    std::cout << "选项:" << std::endl;
    std::cout << "  --fp16    使用FP16精度 (默认)" << std::endl;
    std::cout << "  --fp32    使用FP32精度" << std::endl;
    std::cout << "示例:" << std::endl;
    std::cout << "  " << prog << " model.onnx model.engine" << std::endl;
    std::cout << "  " << prog << " model.onnx model.engine --fp32" << std::endl;
}

int main(int argc, char** argv) {
    if (argc < 3) {
        printUsage(argv[0]);
        return 1;
    }

    std::string onnx_path = argv[1];
    std::string engine_path = argv[2];
    bool use_fp16 = true;  // 默认使用FP16

    // 解析参数
    for (int i = 3; i < argc; ++i) {
        if (std::strcmp(argv[i], "--fp32") == 0) {
            use_fp16 = false;
        } else if (std::strcmp(argv[i], "--fp16") == 0) {
            use_fp16 = true;
        }
    }

    std::cout << "========================================" << std::endl;
    std::cout << "  ONNX to TensorRT 转换工具" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "输入: " << onnx_path << std::endl;
    std::cout << "输出: " << engine_path << std::endl;
    std::cout << "精度: " << (use_fp16 ? "FP16" : "FP32") << std::endl;
    std::cout << "========================================" << std::endl;

    if (convertOnnxToTrt(onnx_path, engine_path, use_fp16)) {
        std::cout << "转换完成!" << std::endl;
        return 0;
    } else {
        std::cerr << "转换失败!" << std::endl;
        return 1;
    }
}
