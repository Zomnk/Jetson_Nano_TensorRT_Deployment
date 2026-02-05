/**
 * @file test_trt_engine_csv.cpp
 * @brief 从CSV文件读取黄金数据进行TensorRT推理对比测试
 */

#include <NvInfer.h>
#include <NvInferVersion.h>
#include <cuda_runtime.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <iomanip>
#include <sstream>
#include <cmath>

class Logger : public nvinfer1::ILogger {
public:
    void log(Severity severity, const char* msg) noexcept override {
        if (severity <= Severity::kWARNING)
            std::cout << "[TRT] " << msg << std::endl;
    }
} gLogger;

// 从CSV行解析浮点数
std::vector<float> parseCSVLine(const std::string& line) {
    std::vector<float> values;
    std::stringstream ss(line);
    std::string token;
    while (std::getline(ss, token, ',')) {
        try {
            values.push_back(std::stof(token));
        } catch (...) {
            values.push_back(0.0f);
        }
    }
    return values;
}

int main(int argc, char** argv) {
    if (argc < 3) {
        std::cout << "用法: " << argv[0] << " <engine.engine> <csv_file>" << std::endl;
        return 1;
    }

    std::string engine_path = argv[1];
    std::string csv_path = argv[2];

    const int PROP_DIM = 39;
    const int HISTORY_LEN = 10;
    const int HISTORY_DIM = HISTORY_LEN * PROP_DIM;
    const int OUTPUT_DIM = 10;

    // ========== 加载TensorRT引擎 ==========
    std::cout << "加载TensorRT引擎..." << std::endl;
    std::ifstream engine_file(engine_path, std::ios::binary);
    if (!engine_file.good()) {
        std::cerr << "无法打开引擎文件!" << std::endl;
        return 1;
    }

    engine_file.seekg(0, std::ios::end);
    size_t size = engine_file.tellg();
    engine_file.seekg(0, std::ios::beg);

    std::vector<char> buffer(size);
    engine_file.read(buffer.data(), size);
    engine_file.close();

    auto runtime = nvinfer1::createInferRuntime(gLogger);
    auto engine = runtime->deserializeCudaEngine(buffer.data(), size);
    if (!engine) {
        std::cerr << "反序列化引擎失败!" << std::endl;
        return 1;
    }

    auto context = engine->createExecutionContext();
    std::cout << "引擎加载成功!" << std::endl;

    // ========== 分配GPU内存 ==========
    void* d_prop;
    void* d_history;
    void* d_output;
    cudaMalloc(&d_prop, PROP_DIM * sizeof(float));
    cudaMalloc(&d_history, HISTORY_DIM * sizeof(float));
    cudaMalloc(&d_output, OUTPUT_DIM * sizeof(float));

    cudaStream_t stream;
    cudaStreamCreate(&stream);

    // ========== 读取CSV文件 ==========
    std::cout << "\n读取CSV文件: " << csv_path << std::endl;
    std::ifstream csv_file(csv_path);
    if (!csv_file.good()) {
        std::cerr << "无法打开CSV文件!" << std::endl;
        return 1;
    }

    std::string header;
    std::getline(csv_file, header);  // 跳过表头

    std::vector<std::vector<float>> all_rows;
    std::string line;
    int row_count = 0;
    while (std::getline(csv_file, line) && row_count < 5) {
        auto values = parseCSVLine(line);
        all_rows.push_back(values);
        row_count++;
    }
    csv_file.close();

    std::cout << "读取了 " << row_count << " 行数据\n" << std::endl;

    // ========== 对每一行进行推理 ==========
    for (int row_idx = 0; row_idx < row_count; row_idx++) {
        auto& row = all_rows[row_idx];

        // 提取 t0 (前39维)
        std::vector<float> prop(row.begin(), row.begin() + PROP_DIM);

        // 提取 history (390维，从列39开始)
        // 注意：CSV中的结构是 t0(39) + priv(44) + scan(187) + history(390) + actions(10)
        // history 从列 39+44+187=270 开始
        int history_start = 39 + 44 + 187;
        std::vector<float> history(row.begin() + history_start,
                                   row.begin() + history_start + HISTORY_DIM);

        // 提取仿真中的真实 action (最后10维)
        std::vector<float> gt_action(row.end() - OUTPUT_DIM, row.end());

        // 拷贝到GPU
        cudaMemcpyAsync(d_prop, prop.data(), PROP_DIM * sizeof(float),
                        cudaMemcpyHostToDevice, stream);
        cudaMemcpyAsync(d_history, history.data(), HISTORY_DIM * sizeof(float),
                        cudaMemcpyHostToDevice, stream);

        // 执行推理
        void* bindings[] = {d_prop, d_history, d_output};
        context->enqueueV2(bindings, stream, nullptr);

        // 获取输出
        std::vector<float> infer_action(OUTPUT_DIM);
        cudaMemcpyAsync(infer_action.data(), d_output, OUTPUT_DIM * sizeof(float),
                        cudaMemcpyDeviceToHost, stream);
        cudaStreamSynchronize(stream);

        // 打印对比结果
        std::cout << "========== 第 " << (row_idx + 1) << " 步 ==========" << std::endl;

        std::cout << "推理输出 (TensorRT): ";
        for (int i = 0; i < OUTPUT_DIM; i++) {
            std::cout << std::fixed << std::setprecision(6) << infer_action[i] << " ";
        }
        std::cout << std::endl;

        std::cout << "仿真输出 (Golden):   ";
        for (int i = 0; i < OUTPUT_DIM; i++) {
            std::cout << std::fixed << std::setprecision(6) << gt_action[i] << " ";
        }
        std::cout << std::endl;

        // 计算误差
        float max_error = 0.0f;
        float mean_error = 0.0f;
        for (int i = 0; i < OUTPUT_DIM; i++) {
            float error = std::abs(infer_action[i] - gt_action[i]);
            max_error = std::max(max_error, error);
            mean_error += error;
        }
        mean_error /= OUTPUT_DIM;

        std::cout << "最大误差: " << std::fixed << std::setprecision(6) << max_error << std::endl;
        std::cout << "平均误差: " << std::fixed << std::setprecision(6) << mean_error << std::endl;
        std::cout << std::endl;
    }

    // ========== 清理资源 ==========
    cudaFree(d_prop);
    cudaFree(d_history);
    cudaFree(d_output);
    cudaStreamDestroy(stream);

    std::cout << "测试完成!" << std::endl;
    return 0;
}
