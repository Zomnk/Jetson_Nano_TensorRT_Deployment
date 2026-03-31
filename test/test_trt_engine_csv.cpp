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

    // Term-major 布局子维度
    const int ANG_VEL_DIM = 3;
    const int GRAVITY_DIM = 3;
    const int CMD_DIM = 3;
    const int JOINT_DIM = 10;

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
    while (std::getline(csv_file, line) && row_count < 200) {
        auto values = parseCSVLine(line);
        all_rows.push_back(values);
        row_count++;
    }
    csv_file.close();

    std::cout << "读取了 " << row_count << " 行数据\n" << std::endl;

    // ========== 对每一行进行推理 ==========
    float total_max_error = 0.0f;
    float total_mean_error = 0.0f;
    float total_max_rel_error = 0.0f;
    int error_count = 0;

    for (int row_idx = 0; row_idx < row_count; row_idx++) {
        auto& row = all_rows[row_idx];

        // CSV 格式：step(1) + obs_history(390, term-major) + act(10)
        // CSV 中的数据已经是 IsaacLab term-major 格式，无需转换！
        //
        // term-major 布局：
        //   [0-29]    base_ang_vel 历史      [10帧 × 3维]
        //   [30-59]   projected_gravity 历史  [10帧 × 3维]
        //   [60-89]   velocity_commands 历史  [10帧 × 3维]
        //   [90-189]  joint_pos 历史          [10帧 × 10维]
        //   [190-289] joint_vel 历史          [10帧 × 10维]
        //   [290-389] last_action 历史        [10帧 × 10维]

        // 直接读取历史缓冲区 (390维, term-major)
        std::vector<float> history(row.begin() + 1, row.begin() + 1 + HISTORY_DIM);

        // 从历史缓冲区各 term 的最后一帧提取当前 proprioception (39维)
        std::vector<float> prop(PROP_DIM);
        int last_frame_idx = HISTORY_LEN - 1;  // 第9帧（最新帧）
        int offset = 0;

        // base_ang_vel 最后一帧: [27-29]
        for (int d = 0; d < ANG_VEL_DIM; d++)
            prop[offset++] = history[last_frame_idx * ANG_VEL_DIM + d];
        // projected_gravity 最后一帧: [57-59]
        for (int d = 0; d < GRAVITY_DIM; d++)
            prop[offset++] = history[HISTORY_LEN * ANG_VEL_DIM + last_frame_idx * GRAVITY_DIM + d];
        // velocity_commands 最后一帧: [87-89]
        for (int d = 0; d < CMD_DIM; d++)
            prop[offset++] = history[HISTORY_LEN * (ANG_VEL_DIM + GRAVITY_DIM) + last_frame_idx * CMD_DIM + d];
        // joint_pos 最后一帧: [180-189]
        int joint_pos_offset = HISTORY_LEN * (ANG_VEL_DIM + GRAVITY_DIM + CMD_DIM);
        for (int d = 0; d < JOINT_DIM; d++)
            prop[offset++] = history[joint_pos_offset + last_frame_idx * JOINT_DIM + d];
        // joint_vel 最后一帧: [280-289]
        int joint_vel_offset = joint_pos_offset + HISTORY_LEN * JOINT_DIM;
        for (int d = 0; d < JOINT_DIM; d++)
            prop[offset++] = history[joint_vel_offset + last_frame_idx * JOINT_DIM + d];
        // last_action 最后一帧: [380-389]
        int action_offset = joint_vel_offset + HISTORY_LEN * JOINT_DIM;
        for (int d = 0; d < JOINT_DIM; d++)
            prop[offset++] = history[action_offset + last_frame_idx * JOINT_DIM + d];

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
        float max_rel_error = 0.0f;
        for (int i = 0; i < OUTPUT_DIM; i++) {
            float error = std::abs(infer_action[i] - gt_action[i]);
            max_error = std::max(max_error, error);
            mean_error += error;

            // 计算相对误差
            if (std::abs(gt_action[i]) > 1e-6) {
                float rel_error = error / std::abs(gt_action[i]);
                max_rel_error = std::max(max_rel_error, rel_error);
            }
        }
        mean_error /= OUTPUT_DIM;

        std::cout << "最大误差: " << std::fixed << std::setprecision(6) << max_error << std::endl;
        std::cout << "平均误差: " << std::fixed << std::setprecision(6) << mean_error << std::endl;
        std::cout << "最大相对误差: " << std::fixed << std::setprecision(6) << max_rel_error << std::endl;
        std::cout << std::endl;

        // 累计统计
        total_max_error += max_error;
        total_mean_error += mean_error;
        total_max_rel_error += max_rel_error;
        error_count++;
    }

    // ========== 打印总体统计 ==========
    std::cout << "\n========== 总体统计 ==========" << std::endl;
    std::cout << "测试样本数: " << error_count << std::endl;
    std::cout << "平均最大误差: " << std::fixed << std::setprecision(6) << (total_max_error / error_count) << std::endl;
    std::cout << "平均平均误差: " << std::fixed << std::setprecision(6) << (total_mean_error / error_count) << std::endl;
    std::cout << "平均最大相对误差: " << std::fixed << std::setprecision(6) << (total_max_rel_error / error_count) << std::endl;

    // ========== 清理资源 ==========
    cudaFree(d_prop);
    cudaFree(d_history);
    cudaFree(d_output);
    cudaStreamDestroy(stream);

    std::cout << "测试完成!" << std::endl;
    return 0;
}
