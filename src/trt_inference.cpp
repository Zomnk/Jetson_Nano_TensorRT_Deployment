/**
 * @file trt_inference.cpp
 * @brief TensorRT推理引擎类实现文件
 * @author Zomnk
 * @date 2026-02-04
 *
 * @details 实现了TRTInference类的所有成员函数。
 *          负责加载TensorRT引擎、构建观测向量、执行GPU推理。
 *          支持TensorRT 8.2及以上版本。
 */

#include "trt_inference.h"
#include <NvInferVersion.h>  // 用于版本检测宏
#include <fstream>
#include <iostream>
#include <cstring>
#include <cmath>
#include <algorithm>

/**
 * @brief TensorRT日志记录器
 *
 * @details TensorRT需要一个ILogger实现来输出日志信息。
 *          这里只输出WARNING及以上级别的日志。
 */
class Logger : public nvinfer1::ILogger {
public:
    void log(Severity severity, const char* msg) noexcept override {
        // 只输出警告和错误信息，忽略INFO和VERBOSE
        if (severity <= Severity::kWARNING)
            std::cout << "[TRT] " << msg << std::endl;
    }
} gLogger;  // 全局日志记录器实例

/**
 * @brief 构造函数
 *
 * @details 初始化所有成员变量为默认值：
 *          - CUDA资源指针设为nullptr
 *          - 状态缓存清零
 *          - 引擎加载标志设为false
 */
TRTInference::TRTInference()
    : stream_(nullptr)          // CUDA流
    , d_input_(nullptr)         // GPU输入缓冲区 - 当前观测
    , d_obs_buf_(nullptr)       // GPU输入缓冲区 - 历史观测缓存
    , d_output_(nullptr)        // GPU输出缓冲区
    , cmd_x_(0), cmd_y_(0), cmd_rate_(0)  // 滤波后的控制指令
    , engine_loaded_(false) {   // 引擎未加载
    // 初始化初始姿态为全零
    std::fill(init_pos_, init_pos_ + DOF_NUM, 0.0f);
    // 初始化上次动作为全零
    std::fill(last_action_, last_action_ + ACTION_DIM, 0.0f);
    // 初始化临时动作缓存为全零
    std::fill(action_temp_, action_temp_ + ACTION_DIM, 0.0f);
    // 初始化历史观测缓存为全零
    std::fill(obs_buf_, obs_buf_ + HISTORY_LENGTH * OBS_DIM, 0.0f);
}

/**
 * @brief 析构函数
 *
 * @details 释放所有CUDA资源：
 *          - 释放GPU内存
 *          - 销毁CUDA流
 *          TensorRT对象由unique_ptr自动管理
 */
TRTInference::~TRTInference() {
    // 释放GPU输入缓冲区 - 当前观测
    if (d_input_) cudaFree(d_input_);
    // 释放GPU输入缓冲区 - 历史观测缓存
    if (d_obs_buf_) cudaFree(d_obs_buf_);
    // 释放GPU输出缓冲区
    if (d_output_) cudaFree(d_output_);
    // 销毁CUDA流
    if (stream_) cudaStreamDestroy(stream_);
}

/**
 * @brief 加载TensorRT引擎文件
 *
 * @details 完整的加载流程：
 *          1. 以二进制模式读取.engine文件
 *          2. 创建TensorRT运行时
 *          3. 反序列化引擎
 *          4. 创建执行上下文
 *          5. 创建CUDA流
 *          6. 分配GPU输入输出缓冲区
 *
 * @param engine_path 引擎文件路径
 * @return 加载成功返回true，失败返回false
 */
bool TRTInference::loadEngine(const std::string& engine_path) {
    // ========== 步骤1: 读取引擎文件 ==========
    std::ifstream file(engine_path, std::ios::binary);
    if (!file.good()) {
        std::cerr << "[TRT] Cannot open engine: " << engine_path << std::endl;
        return false;
    }

    // 获取文件大小
    file.seekg(0, std::ios::end);
    size_t size = file.tellg();
    file.seekg(0, std::ios::beg);

    // 读取文件内容到内存
    std::vector<char> buffer(size);
    file.read(buffer.data(), size);
    file.close();

    // ========== 步骤2: 创建TensorRT运行时 ==========
    runtime_.reset(nvinfer1::createInferRuntime(gLogger));
    if (!runtime_) {
        std::cerr << "[TRT] Failed to create runtime" << std::endl;
        return false;
    }

    // ========== 步骤3: 反序列化引擎 ==========
    // 将序列化的引擎数据转换为可执行的引擎对象
    engine_.reset(runtime_->deserializeCudaEngine(buffer.data(), size));
    if (!engine_) {
        std::cerr << "[TRT] Failed to deserialize engine" << std::endl;
        return false;
    }

    // ========== 步骤4: 创建执行上下文 ==========
    // 执行上下文用于执行推理
    context_.reset(engine_->createExecutionContext());
    if (!context_) {
        std::cerr << "[TRT] Failed to create context" << std::endl;
        return false;
    }

    // ========== 步骤5: 创建CUDA流 ==========
    // CUDA流用于异步操作，提高效率
    cudaStreamCreate(&stream_);

    // ========== 步骤6: 分配GPU内存 ==========
    // 输入缓冲区1: 39个float (当前观测向量)
    cudaMalloc(&d_input_, OBS_DIM * sizeof(float));
    // 输入缓冲区2: HISTORY_LENGTH * 39个float (历史观测缓存)
    cudaMalloc(&d_obs_buf_, HISTORY_LENGTH * OBS_DIM * sizeof(float));
    // 输出缓冲区: 10个float (动作向量)
    cudaMalloc(&d_output_, ACTION_DIM * sizeof(float));

    // 初始化历史观测缓存为全零
    cudaMemset(d_obs_buf_, 0, HISTORY_LENGTH * OBS_DIM * sizeof(float));

    std::cout << "[TRT] Engine loaded: " << engine_path << std::endl;
    std::cout << "[TRT] History length: " << HISTORY_LENGTH << std::endl;
    engine_loaded_ = true;
    return true;
}

/**
 * @brief 设置初始站立姿态
 *
 * @details 将标定的初始姿态保存到成员变量。
 *          初始姿态用于计算关节位置偏差（观测向量的一部分）。
 *
 * @param init_pos 10个关节的初始位置数组
 */
void TRTInference::setInitPose(const float* init_pos) {
    std::copy(init_pos, init_pos + DOF_NUM, init_pos_);
}

/**
 * @brief 重置推理状态
 *
 * @details 清零所有状态缓存：
 *          - 上次动作
 *          - 滤波后的控制指令
 *          通常在开始新的控制周期时调用。
 */
void TRTInference::reset() {
    std::fill(last_action_, last_action_ + ACTION_DIM, 0.0f);
    std::fill(action_temp_, action_temp_ + ACTION_DIM, 0.0f);
    cmd_x_ = cmd_y_ = cmd_rate_ = 0.0f;
    // 重置历史观测缓存
    std::fill(obs_buf_, obs_buf_ + HISTORY_LENGTH * OBS_DIM, 0.0f);
    if (d_obs_buf_) {
        cudaMemset(d_obs_buf_, 0, HISTORY_LENGTH * OBS_DIM * sizeof(float));
    }
}

/**
 * @brief 应用死区
 *
 * @details 如果输入值的绝对值小于死区阈值，返回0。
 *          用于消除遥控器的微小抖动。
 *
 * @param value 输入值
 * @param deadzone 死区阈值
 * @return 处理后的值
 */
float TRTInference::applyDeadzone(float value, float deadzone) {
    return std::fabs(value) < deadzone ? 0.0f : value;
}

/**
 * @brief 构建观测向量
 *
 * @details 将机器人状态转换为39维观测向量，作为神经网络的输入。
 *          观测向量的组成（与训练时保持一致）：
 *
 *          索引范围    维度    内容                    缩放系数
 *          ─────────────────────────────────────────────────────
 *          [0-2]       3      角速度 (rad/s)          × 0.25
 *          [3-5]       3      欧拉角 (rad)            × 1.0
 *          [6-8]       3      控制指令 (滤波后)       × 2.0/0.25
 *          [9-18]      10     关节位置偏差 (rad)      × 1.0
 *          [19-28]     10     关节速度 (rad/s)        × 0.05
 *          [29-38]     10     上次动作                × 1.0
 *          ─────────────────────────────────────────────────────
 *          总计        39
 *
 * @param request 请求消息（包含机器人状态）
 * @param obs 输出的观测数组
 */
void TRTInference::buildObservation(const MsgRequest& request, float* obs) {
    int idx = 0;

    // ========== [0-2] 角速度 ==========
    // IMU测量的机身角速度，乘以缩放系数
    obs[idx++] = request.omega[0] * OMEGA_SCALE;
    obs[idx++] = request.omega[1] * OMEGA_SCALE;
    obs[idx++] = request.omega[2] * OMEGA_SCALE;

    // ========== [3-5] 欧拉角 ==========
    // IMU测量的机身姿态角
    obs[idx++] = request.eu_ang[0] * EU_ANG_SCALE;
    obs[idx++] = request.eu_ang[1] * EU_ANG_SCALE;
    obs[idx++] = request.eu_ang[2] * EU_ANG_SCALE;

    // ========== [6-8] 控制指令 ==========
    // 来自遥控器的速度指令，应用死区和一阶低通滤波
    // 滤波公式: y = y * (1 - α) + x * α，其中α = SMOOTH = 0.03
    cmd_x_ = cmd_x_ * (1 - SMOOTH) + applyDeadzone(request.command[0], DEAD_ZONE) * SMOOTH;
    cmd_y_ = cmd_y_ * (1 - SMOOTH) + applyDeadzone(request.command[1], DEAD_ZONE) * SMOOTH;
    cmd_rate_ = cmd_rate_ * (1 - SMOOTH) + applyDeadzone(request.command[2], DEAD_ZONE) * SMOOTH;
    obs[idx++] = cmd_x_ * LIN_VEL_SCALE;      // 前进速度指令
    obs[idx++] = cmd_y_ * LIN_VEL_SCALE;      // 侧移速度指令
    obs[idx++] = cmd_rate_ * ANG_VEL_SCALE;   // 转向速度指令

    // ========== [9-18] 关节位置偏差 ==========
    // 当前关节位置相对于初始姿态的偏差
    for (int i = 0; i < DOF_NUM; ++i)
        obs[idx++] = (request.q[i] - init_pos_[i]) * POS_SCALE;

    // ========== [19-28] 关节速度 ==========
    // 当前关节角速度
    for (int i = 0; i < DOF_NUM; ++i)
        obs[idx++] = request.dq[i] * VEL_SCALE;

    // ========== [29-38] 上次动作 ==========
    // 上一个控制周期输出的动作（使用限幅后的值，与LibTorch版本一致）
    for (int i = 0; i < ACTION_DIM; ++i)
        obs[idx++] = action_temp_[i];
}

/**
 * @brief 执行推理
 *
 * @details 完整的推理流程：
 *          1. 检查触发标志和引擎状态
 *          2. 更新初始姿态（从请求消息）
 *          3. 构建39维观测向量
 *          4. 将观测数据拷贝到GPU
 *          5. 执行TensorRT推理
 *          6. 将结果拷贝回CPU
 *          7. 应用动作滤波（80%新动作 + 20%旧动作）
 *          8. 应用限幅（±15.0）
 *
 * @param request ODroid发送的请求消息
 * @param action_out 输出的动作数组（10个关节目标位置）
 * @return 推理成功返回true，失败返回false
 */
bool TRTInference::infer(const MsgRequest& request, float* action_out) {
    // 检查引擎是否已加载
    if (!engine_loaded_) return false;

    // 检查触发标志，只有trigger=1.0时才执行推理
    if (request.trigger != 1.0f) return false;

    // ========== 更新初始位置 ==========
    // 从请求消息中获取最新的初始姿态
    for (int i = 0; i < DOF_NUM; ++i)
        init_pos_[i] = request.init_pos[i];

    // ========== 构建当前观测向量 ==========
    float obs[OBS_DIM];
    buildObservation(request, obs);

    // ========== 拷贝数据到GPU ==========
    // 拷贝当前观测到GPU
    cudaMemcpyAsync(d_input_, obs, OBS_DIM * sizeof(float), cudaMemcpyHostToDevice, stream_);
    // 拷贝历史观测缓存到GPU
    cudaMemcpyAsync(d_obs_buf_, obs_buf_, HISTORY_LENGTH * OBS_DIM * sizeof(float), cudaMemcpyHostToDevice, stream_);

    // ========== 执行TensorRT推理 ==========
    // 模型输入：当前观测 [1, 39] + 历史观测缓存 [1, HISTORY_LENGTH, 39]
    // 根据TensorRT版本选择不同的API
#if NV_TENSORRT_MAJOR >= 8 && NV_TENSORRT_MINOR >= 5
    // TensorRT 8.5+ 新API
    context_->setTensorAddress("input", d_input_);
    context_->setTensorAddress("obs_buf", d_obs_buf_);
    context_->setTensorAddress("output", d_output_);
    context_->enqueueV3(stream_);
#else
    // TensorRT 8.2-8.4 旧API
    // bindings顺序需要与ONNX模型的输入输出顺序一致
    void* bindings[] = {d_input_, d_obs_buf_, d_output_};
    context_->enqueueV2(bindings, stream_, nullptr);
#endif

    // ========== 拷贝结果回CPU ==========
    float output[ACTION_DIM];
    cudaMemcpyAsync(output, d_output_, ACTION_DIM * sizeof(float), cudaMemcpyDeviceToHost, stream_);

    // 等待所有CUDA操作完成
    cudaStreamSynchronize(stream_);

    // ========== 更新历史观测缓存（滑动窗口） ==========
    // 移除最旧的观测，添加最新的观测
    // obs_buf_布局: [obs_0, obs_1, ..., obs_{HISTORY_LENGTH-1}]
    // 更新后: [obs_1, obs_2, ..., obs_{HISTORY_LENGTH-1}, obs_new]
    for (int i = 0; i < (HISTORY_LENGTH - 1) * OBS_DIM; ++i) {
        obs_buf_[i] = obs_buf_[i + OBS_DIM];
    }
    // 将当前观测添加到缓存末尾
    for (int i = 0; i < OBS_DIM; ++i) {
        obs_buf_[(HISTORY_LENGTH - 1) * OBS_DIM + i] = obs[i];
    }

    // ========== 动作后处理 ==========
    for (int i = 0; i < ACTION_DIM; ++i) {
        // 动作滤波: 80%新动作 + 20%旧动作，使动作更平滑
        float blended = 0.8f * output[i] + 0.2f * last_action_[i];

        // 限幅: 将动作限制在[-15, 15]范围内
        float clamped = blended < -15.0f ? -15.0f : (blended > 15.0f ? 15.0f : blended);

        // 输出动作
        action_out[i] = clamped;

        // 保存原始输出用于下次滤波
        last_action_[i] = output[i];

        // 保存限幅后的值用于下次观测构建（与LibTorch版本一致）
        action_temp_[i] = clamped;
    }

    return true;
}
