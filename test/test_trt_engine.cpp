/**
 * @file test_trt_engine.cpp
 * @brief TensorRT引擎测试程序
 * @author Zomnk
 * @date 2026-02-04
 *
 * @details 本程序用于测试TensorRT引擎的加载和推理功能。
 *          验证内容：
 *          1. CUDA设备是否可用
 *          2. TensorRT引擎能否正确加载
 *          3. 推理是否正常工作
 *          4. 推理性能（FPS）
 *
 * @note 使用方法:
 *       ./test_trt_engine <engine.engine>
 */

#include <NvInfer.h>
#include <NvInferVersion.h>
#include <cuda_runtime.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <random>
#include <iomanip>

/**
 * @brief TensorRT日志记录器
 */
class Logger : public nvinfer1::ILogger {
public:
    void log(Severity severity, const char* msg) noexcept override {
        if (severity <= Severity::kWARNING)
            std::cout << "[TRT] " << msg << std::endl;
    }
} gLogger;

/**
 * @brief 主函数
 *
 * @details 测试流程：
 *          1. 检查CUDA设备
 *          2. 加载TensorRT引擎
 *          3. 打印引擎绑定信息
 *          4. 创建随机测试输入
 *          5. 执行推理并计时
 *          6. 打印输出结果
 */
int main(int argc, char** argv) {
    // ========== 检查命令行参数 ==========
    if (argc < 2) {
        std::cout << "用法: " << argv[0] << " <engine.engine>" << std::endl;
        return 1;
    }

    std::string engine_path = argv[1];

    // ========== 打印测试信息 ==========
    std::cout << "========================================" << std::endl;
    std::cout << "  TensorRT引擎测试" << std::endl;
    std::cout << "  TensorRT版本: " << NV_TENSORRT_MAJOR << "."
              << NV_TENSORRT_MINOR << "." << NV_TENSORRT_PATCH << std::endl;
    std::cout << "  引擎: " << engine_path << std::endl;
    std::cout << "========================================" << std::endl;

    // ========== 检查CUDA设备 ==========
    int device_count = 0;
    cudaGetDeviceCount(&device_count);
    std::cout << "CUDA设备数: " << device_count << std::endl;

    if (device_count == 0) {
        std::cerr << "没有可用的CUDA设备!" << std::endl;
        return 1;
    }

    // 打印GPU信息
    cudaDeviceProp prop;
    cudaGetDeviceProperties(&prop, 0);
    std::cout << "GPU: " << prop.name << std::endl;
    std::cout << "计算能力: " << prop.major << "." << prop.minor << std::endl;

    // ========== 加载TensorRT引擎 ==========
    std::cout << "\n加载TensorRT引擎..." << std::endl;
    std::ifstream file(engine_path, std::ios::binary);
    if (!file.good()) {
        std::cerr << "无法打开引擎文件!" << std::endl;
        return 1;
    }

    // 读取引擎文件
    file.seekg(0, std::ios::end);
    size_t size = file.tellg();
    file.seekg(0, std::ios::beg);
    std::cout << "引擎大小: " << size / 1024 << " KB" << std::endl;

    std::vector<char> buffer(size);
    file.read(buffer.data(), size);
    file.close();

    // 创建运行时和引擎
    auto runtime = nvinfer1::createInferRuntime(gLogger);
    auto engine = runtime->deserializeCudaEngine(buffer.data(), size);
    if (!engine) {
        std::cerr << "反序列化引擎失败!" << std::endl;
        return 1;
    }

    auto context = engine->createExecutionContext();
    std::cout << "引擎加载成功!" << std::endl;

    // ========== 打印绑定信息 ==========
    // 根据TensorRT版本使用不同的API
#if NV_TENSORRT_MAJOR >= 8 && NV_TENSORRT_MINOR >= 5
    // TensorRT 8.5+ 新API
    int nb_io = engine->getNbIOTensors();
    std::cout << "\nIO张量数量: " << nb_io << std::endl;
    for (int i = 0; i < nb_io; i++) {
        const char* name = engine->getIOTensorName(i);
        auto mode = engine->getTensorIOMode(name);
        auto dims = engine->getTensorShape(name);
        std::cout << "  [" << i << "] " << name
                  << " (" << (mode == nvinfer1::TensorIOMode::kINPUT ? "输入" : "输出") << "): [";
        for (int j = 0; j < dims.nbDims; j++) {
            std::cout << dims.d[j];
            if (j < dims.nbDims - 1) std::cout << ", ";
        }
        std::cout << "]" << std::endl;
    }
#else
    // TensorRT 8.2-8.4 旧API
    int nb_bindings = engine->getNbBindings();
    std::cout << "\n绑定数量: " << nb_bindings << std::endl;
    for (int i = 0; i < nb_bindings; i++) {
        auto dims = engine->getBindingDimensions(i);
        bool is_input = engine->bindingIsInput(i);
        std::cout << "  [" << i << "] " << engine->getBindingName(i)
                  << " (" << (is_input ? "输入" : "输出") << "): [";
        for (int j = 0; j < dims.nbDims; j++) {
            std::cout << dims.d[j];
            if (j < dims.nbDims - 1) std::cout << ", ";
        }
        std::cout << "]" << std::endl;
    }
#endif

    // ========== 创建测试输入 ==========
    const int PROP_DIM = 39;           // proprioception 维度
    const int HISTORY_LEN = 10;        // history 长度
    const int HISTORY_DIM = HISTORY_LEN * PROP_DIM;  // history 总维度 = 10 * 39 = 390
    const int OUTPUT_DIM = 10;         // 动作维度

    std::cout << "\n创建测试输入..." << std::endl;
    std::cout << "  proprioception: [1, " << PROP_DIM << "]" << std::endl;
    std::cout << "  history: [1, " << HISTORY_LEN << ", " << PROP_DIM << "]" << std::endl;

    std::vector<float> input_prop(PROP_DIM);
    std::vector<float> input_history(HISTORY_DIM);

    // 使用随机数填充输入
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<float> dist(-1.0f, 1.0f);
    for (int i = 0; i < PROP_DIM; i++) {
        input_prop[i] = dist(gen);
    }
    for (int i = 0; i < HISTORY_DIM; i++) {
        input_history[i] = dist(gen);
    }

    // 打印输入的前5个值
    std::cout << "proprioception前5个值: ";
    for (int i = 0; i < 5; i++) {
        std::cout << std::fixed << std::setprecision(4) << input_prop[i] << " ";
    }
    std::cout << "..." << std::endl;

    // ========== 分配GPU内存 ==========
    void* d_prop;
    void* d_history;
    void* d_output;
    cudaMalloc(&d_prop, PROP_DIM * sizeof(float));
    cudaMalloc(&d_history, HISTORY_DIM * sizeof(float));
    cudaMalloc(&d_output, OUTPUT_DIM * sizeof(float));

    cudaStream_t stream;
    cudaStreamCreate(&stream);

    // 拷贝输入到GPU
    cudaMemcpyAsync(d_prop, input_prop.data(), PROP_DIM * sizeof(float),
                    cudaMemcpyHostToDevice, stream);
    cudaMemcpyAsync(d_history, input_history.data(), HISTORY_DIM * sizeof(float),
                    cudaMemcpyHostToDevice, stream);

    // ========== 执行推理 ==========
    std::cout << "\n执行推理..." << std::endl;

    // 创建CUDA事件用于计时
    cudaEvent_t start, stop;
    cudaEventCreate(&start);
    cudaEventCreate(&stop);

    // 预热（执行10次推理，不计时）
    for (int i = 0; i < 10; i++) {
#if NV_TENSORRT_MAJOR >= 8 && NV_TENSORRT_MINOR >= 5
        context->setTensorAddress("proprioception", d_prop);
        context->setTensorAddress("history", d_history);
        context->setTensorAddress("actions", d_output);
        context->enqueueV3(stream);
#else
        void* bindings[] = {d_prop, d_history, d_output};
        context->enqueueV2(bindings, stream, nullptr);
#endif
    }
    cudaStreamSynchronize(stream);

    // 正式计时（执行100次推理）
    const int iterations = 100;
    cudaEventRecord(start, stream);
    for (int i = 0; i < iterations; i++) {
#if NV_TENSORRT_MAJOR >= 8 && NV_TENSORRT_MINOR >= 5
        context->setTensorAddress("proprioception", d_prop);
        context->setTensorAddress("history", d_history);
        context->setTensorAddress("actions", d_output);
        context->enqueueV3(stream);
#else
        void* bindings[] = {d_prop, d_history, d_output};
        context->enqueueV2(bindings, stream, nullptr);
#endif
    }
    cudaEventRecord(stop, stream);
    cudaStreamSynchronize(stream);

    // 计算平均推理时间
    float elapsed_ms;
    cudaEventElapsedTime(&elapsed_ms, start, stop);
    float avg_ms = elapsed_ms / iterations;

    std::cout << "推理成功!" << std::endl;
    std::cout << "平均推理时间: " << std::fixed << std::setprecision(3)
              << avg_ms << " ms (" << 1000.0f / avg_ms << " FPS)" << std::endl;

    // ========== 获取输出 ==========
    std::vector<float> output(OUTPUT_DIM);
    cudaMemcpyAsync(output.data(), d_output, OUTPUT_DIM * sizeof(float),
                    cudaMemcpyDeviceToHost, stream);
    cudaStreamSynchronize(stream);

    // 打印输出
    std::cout << "\n输出 (" << OUTPUT_DIM << "维动作):" << std::endl;
    for (int i = 0; i < OUTPUT_DIM; i++) {
        std::cout << "  [" << i << "] " << std::fixed << std::setprecision(6)
                  << output[i] << std::endl;
    }

    // ========== 清理资源 ==========
    cudaFree(d_prop);
    cudaFree(d_history);
    cudaFree(d_output);
    cudaStreamDestroy(stream);
    cudaEventDestroy(start);
    cudaEventDestroy(stop);

    std::cout << "\n========================================" << std::endl;
    std::cout << "测试通过!" << std::endl;
    std::cout << "========================================" << std::endl;

    return 0;
}
