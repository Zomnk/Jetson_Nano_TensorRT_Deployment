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
    // 初始化硬件配置为默认值
    std::fill(default_angles_, default_angles_ + DOF_NUM, 0.0f);
    std::fill(offset_, offset_ + DOF_NUM, 0.0f);
    std::fill(sign_array_, sign_array_ + DOF_NUM, 1);
    // 初始化上次动作为全零
    std::fill(last_action_, last_action_ + ACTION_DIM, 0.0f);
    // 初始化临时动作缓存为全零
    std::fill(action_temp_, action_temp_ + ACTION_DIM, 0.0f);
    // 初始化所有 term 独立历史缓冲区为全零（IsaacLab term-major）
    std::fill(hist_ang_vel_, hist_ang_vel_ + HISTORY_LENGTH * ANG_VEL_DIM, 0.0f);
    std::fill(hist_gravity_, hist_gravity_ + HISTORY_LENGTH * GRAVITY_DIM, 0.0f);
    std::fill(hist_cmd_, hist_cmd_ + HISTORY_LENGTH * CMD_DIM, 0.0f);
    std::fill(hist_joint_pos_, hist_joint_pos_ + HISTORY_LENGTH * JOINT_DIM, 0.0f);
    std::fill(hist_joint_vel_, hist_joint_vel_ + HISTORY_LENGTH * JOINT_DIM, 0.0f);
    std::fill(hist_action_, hist_action_ + HISTORY_LENGTH * JOINT_DIM, 0.0f);
    // 初始化拼接后的历史观测缓存为全零
    std::fill(obs_buf_, obs_buf_ + HISTORY_LENGTH * OBS_DIM, 0.0f);
    // 初始化最后一次观测为全零
    std::fill(last_obs_, last_obs_ + OBS_DIM, 0.0f);
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
 * @brief 设置硬件配置（用于 Sim-to-Real 映射）
 *
 * @details 保存硬件配置参数，用于在观测和控制链路中进行坐标变换。
 *          这些参数通常从 robot.yaml 加载。
 *
 * @param default_angles 算法层基准姿态（10个关节）
 * @param offset 硬件零点偏置（10个关节）
 * @param sign_array 关节方向映射（10个关节，1 或 -1）
 */
void TRTInference::setHardwareConfig(const float* default_angles,
                                     const float* offset,
                                     const int* sign_array) {
    std::copy(default_angles, default_angles + DOF_NUM, default_angles_);
    std::copy(offset, offset + DOF_NUM, offset_);
    std::copy(sign_array, sign_array + DOF_NUM, sign_array_);
}

/**
 * @brief 设置初始站立姿态（已废弃，保留用于向后兼容）
 *
 * @details 此方法已被 setHardwareConfig() 替代。
 *          为保持向后兼容，此方法将 init_pos 设置为 offset。
 *
 * @param init_pos 10个关节的初始位置数组
 */
void TRTInference::setInitPose(const float* init_pos) {
    std::copy(init_pos, init_pos + DOF_NUM, offset_);
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
    // 重置所有 term 独立历史缓冲区
    std::fill(hist_ang_vel_, hist_ang_vel_ + HISTORY_LENGTH * ANG_VEL_DIM, 0.0f);
    std::fill(hist_gravity_, hist_gravity_ + HISTORY_LENGTH * GRAVITY_DIM, 0.0f);
    std::fill(hist_cmd_, hist_cmd_ + HISTORY_LENGTH * CMD_DIM, 0.0f);
    std::fill(hist_joint_pos_, hist_joint_pos_ + HISTORY_LENGTH * JOINT_DIM, 0.0f);
    std::fill(hist_joint_vel_, hist_joint_vel_ + HISTORY_LENGTH * JOINT_DIM, 0.0f);
    std::fill(hist_action_, hist_action_ + HISTORY_LENGTH * JOINT_DIM, 0.0f);
    // 重置拼接后的 obs_buf_
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
 * @brief 从欧拉角计算投影重力向量
 *
 * @details 投影重力向量 = R^T * [0, 0, -g]
 *          其中 R 是从欧拉角构建的旋转矩阵
 *          结果是重力在机器人坐标系中的投影
 */
void TRTInference::computeProjectedGravity(const float eu_ang[3], float gravity_proj[3]) {
    float roll = eu_ang[0];
    float pitch = eu_ang[1];

    float cos_roll = std::cos(roll);
    float sin_roll = std::sin(roll);
    float cos_pitch = std::cos(pitch);
    float sin_pitch = std::sin(pitch);

    // 投影重力向量（在机器人坐标系中）
    gravity_proj[0] = sin_pitch;
    gravity_proj[1] = -sin_roll * cos_pitch;
    gravity_proj[2] = -cos_roll * cos_pitch;
}

/**
 * @brief 按 IsaacLab 的 term-major 方式更新历史缓存
 *
 * @details 对每个 term 分别维护滑动窗口，然后按 term-major 拼接：
 *          [0-29]    base_ang_vel 的 10 帧历史（每帧 3 维）
 *          [30-59]   projected_gravity 的 10 帧历史（每帧 3 维）
 *          [60-89]   velocity_commands 的 10 帧历史（每帧 3 维）
 *          [90-189]  joint_pos 的 10 帧历史（每帧 10 维）
 *          [190-289] joint_vel 的 10 帧历史（每帧 10 维）
 *          [290-389] last_action 的 10 帧历史（每帧 10 维）
 *
 * @param obs 当前帧观测（39维），按 frame-major 排列：
 *            [0-2]   base_ang_vel
 *            [3-5]   projected_gravity
 *            [6-8]   velocity_commands
 *            [9-18]  joint_pos
 *            [19-28] joint_vel
 *            [29-38] last_action
 */
void TRTInference::updateHistoryBuffer(const float* obs) {
    // ========== 滑动窗口：每个 term 独立移位 ==========
    // base_ang_vel: shift [3..29], append obs[0-2]
    for (int i = 0; i < (HISTORY_LENGTH - 1) * ANG_VEL_DIM; ++i)
        hist_ang_vel_[i] = hist_ang_vel_[i + ANG_VEL_DIM];
    for (int i = 0; i < ANG_VEL_DIM; ++i)
        hist_ang_vel_[(HISTORY_LENGTH - 1) * ANG_VEL_DIM + i] = obs[i];

    // projected_gravity: shift [3..29], append obs[3-5]
    for (int i = 0; i < (HISTORY_LENGTH - 1) * GRAVITY_DIM; ++i)
        hist_gravity_[i] = hist_gravity_[i + GRAVITY_DIM];
    for (int i = 0; i < GRAVITY_DIM; ++i)
        hist_gravity_[(HISTORY_LENGTH - 1) * GRAVITY_DIM + i] = obs[3 + i];

    // velocity_commands: shift [3..29], append obs[6-8]
    for (int i = 0; i < (HISTORY_LENGTH - 1) * CMD_DIM; ++i)
        hist_cmd_[i] = hist_cmd_[i + CMD_DIM];
    for (int i = 0; i < CMD_DIM; ++i)
        hist_cmd_[(HISTORY_LENGTH - 1) * CMD_DIM + i] = obs[6 + i];

    // joint_pos: shift [10..99], append obs[9-18]
    for (int i = 0; i < (HISTORY_LENGTH - 1) * JOINT_DIM; ++i)
        hist_joint_pos_[i] = hist_joint_pos_[i + JOINT_DIM];
    for (int i = 0; i < JOINT_DIM; ++i)
        hist_joint_pos_[(HISTORY_LENGTH - 1) * JOINT_DIM + i] = obs[9 + i];

    // joint_vel: shift [10..99], append obs[19-28]
    for (int i = 0; i < (HISTORY_LENGTH - 1) * JOINT_DIM; ++i)
        hist_joint_vel_[i] = hist_joint_vel_[i + JOINT_DIM];
    for (int i = 0; i < JOINT_DIM; ++i)
        hist_joint_vel_[(HISTORY_LENGTH - 1) * JOINT_DIM + i] = obs[19 + i];

    // last_action: shift [10..99], append obs[29-38]
    for (int i = 0; i < (HISTORY_LENGTH - 1) * JOINT_DIM; ++i)
        hist_action_[i] = hist_action_[i + JOINT_DIM];
    for (int i = 0; i < JOINT_DIM; ++i)
        hist_action_[(HISTORY_LENGTH - 1) * JOINT_DIM + i] = obs[29 + i];

    // ========== 按 term-major 拼接成 390 维 obs_buf_ ==========
    int offset = 0;

    // [0-29] base_ang_vel 历史
    std::memcpy(obs_buf_ + offset, hist_ang_vel_, HISTORY_LENGTH * ANG_VEL_DIM * sizeof(float));
    offset += HISTORY_LENGTH * ANG_VEL_DIM;

    // [30-59] projected_gravity 历史
    std::memcpy(obs_buf_ + offset, hist_gravity_, HISTORY_LENGTH * GRAVITY_DIM * sizeof(float));
    offset += HISTORY_LENGTH * GRAVITY_DIM;

    // [60-89] velocity_commands 历史
    std::memcpy(obs_buf_ + offset, hist_cmd_, HISTORY_LENGTH * CMD_DIM * sizeof(float));
    offset += HISTORY_LENGTH * CMD_DIM;

    // [90-189] joint_pos 历史
    std::memcpy(obs_buf_ + offset, hist_joint_pos_, HISTORY_LENGTH * JOINT_DIM * sizeof(float));
    offset += HISTORY_LENGTH * JOINT_DIM;

    // [190-289] joint_vel 历史
    std::memcpy(obs_buf_ + offset, hist_joint_vel_, HISTORY_LENGTH * JOINT_DIM * sizeof(float));
    offset += HISTORY_LENGTH * JOINT_DIM;

    // [290-389] last_action 历史
    std::memcpy(obs_buf_ + offset, hist_action_, HISTORY_LENGTH * JOINT_DIM * sizeof(float));
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
    // IMU测量的机身角速度，乘以缩放系数（现在为 1.0，适配 IsaacLab）
    obs[idx++] = request.omega[0] * OMEGA_SCALE;
    obs[idx++] = request.omega[1] * OMEGA_SCALE;
    obs[idx++] = request.omega[2] * OMEGA_SCALE;

    // ========== [3-5] 投影重力向量 ==========
    // 改为使用投影重力向量而不是欧拉角，适配 IsaacLab
    float gravity_proj[3];
    computeProjectedGravity(request.eu_ang, gravity_proj);
    obs[idx++] = gravity_proj[0];
    obs[idx++] = gravity_proj[1];
    obs[idx++] = gravity_proj[2];

    // ========== [6-8] 控制指令 ==========
    // 改为直接使用控制指令，不进行死区、滤波、缩放，适配 IsaacLab
    obs[idx++] = request.command[0];
    obs[idx++] = request.command[1];
    obs[idx++] = request.command[2];

    // ========== [9-18] 关节位置偏差 ==========
    // 应用 Real→Sim 映射，然后计算相对于 default_angles 的偏差
    // 改为应用方向掩码（sign_array），适配 IsaacLab
    // q_sim = (q_real - offset) * sign_array
    // obs = (q_sim - default_angles) * sign_array
    for (int i = 0; i < DOF_NUM; ++i) {
        float q_sim = (request.q[i] - offset_[i]) * sign_array_[i];
        obs[idx++] = (q_sim - default_angles_[i]) * sign_array_[i];
    }

    // ========== [19-28] 关节速度 ==========
    // 应用 Real→Sim 映射
    // 改为应用方向掩码（sign_array），适配 IsaacLab
    // dq_sim = dq_real * sign_array
    // obs = dq_sim * sign_array
    for (int i = 0; i < DOF_NUM; ++i) {
        float dq_sim = request.dq[i] * sign_array_[i];
        obs[idx++] = dq_sim * sign_array_[i];
    }

    // ========== [29-38] 上次动作 ==========
    // 上一个控制周期输出的动作（使用限幅后的值，与 LibTorch 版本一致）
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

    // 注意: init_pos_ 仅在启动时通过 setInitPose() 从 robot.yaml 加载
    // 不再从 request.init_pos 动态更新，确保使用标定值

    // ========== 构建当前观测向量 ==========
    float obs[OBS_DIM];
    buildObservation(request, obs);

    // ========== 拷贝数据到GPU ==========
    // 拷贝当前观测到GPU
    cudaMemcpyAsync(d_input_, obs, OBS_DIM * sizeof(float), cudaMemcpyHostToDevice, stream_);
    // 拷贝历史观测缓存到GPU
    cudaMemcpyAsync(d_obs_buf_, obs_buf_, HISTORY_LENGTH * OBS_DIM * sizeof(float), cudaMemcpyHostToDevice, stream_);

    // ========== 执行TensorRT推理 ==========
    // 模型输入：proprioception [1, 39] + history [1, HISTORY_LENGTH, 39]
    // 模型输出：actions [1, 10]
    // 根据TensorRT版本选择不同的API
#if NV_TENSORRT_MAJOR >= 8 && NV_TENSORRT_MINOR >= 5
    // TensorRT 8.5+ 新API
    context_->setTensorAddress("proprioception", d_input_);
    context_->setTensorAddress("history", d_obs_buf_);
    context_->setTensorAddress("actions", d_output_);
    context_->enqueueV3(stream_);
#else
    // TensorRT 8.2-8.4 旧API
    // bindings顺序需要与ONNX模型的输入输出顺序一致
    // [0] proprioception, [1] history, [2] actions
    void* bindings[] = {d_input_, d_obs_buf_, d_output_};
    context_->enqueueV2(bindings, stream_, nullptr);
#endif

    // ========== 拷贝结果回CPU ==========
    float output[ACTION_DIM];
    cudaMemcpyAsync(output, d_output_, ACTION_DIM * sizeof(float), cudaMemcpyDeviceToHost, stream_);

    // 等待所有CUDA操作完成
    cudaStreamSynchronize(stream_);

    // ========== 更新历史观测缓存（term-major 滑动窗口） ==========
    updateHistoryBuffer(obs);

    // ========== 动作后处理 ==========
    // 改为直接使用网络输出，不进行滤波和限幅，适配 IsaacLab
    for (int i = 0; i < ACTION_DIM; ++i) {
        // 直接使用网络输出，不进行 80%新+20%旧 的混合
        float processed = output[i];

        // 保存用于下次观测
        action_temp_[i] = processed;
        last_action_[i] = output[i];
    }

    // ========== 应用 Sim→Real 映射 ==========
    // 将算法空间的动作转换为电机空间的命令
    // 1. 先将动作缩放并加上 default_angles，得到 Sim 空间的目标位置
    // 2. 然后应用 sign 和 offset，转换为 Real 空间的电机命令
    // 公式：q_real = (action * 0.25 + default_angles) * sign_array + offset
    // 其中 offset = q_encoder_stand - default_angles
    for (int i = 0; i < ACTION_DIM; ++i) {
        float q_sim_target = action_temp_[i] * 0.25f + default_angles_[i];
        action_out[i] = q_sim_target * sign_array_[i] + offset_[i];
    }

    // 保存观测向量用于调试输出
    std::copy(obs, obs + OBS_DIM, last_obs_);

    return true;
}

/**
 * @brief 获取最后一次的观测向量
 */
void TRTInference::getLastObservation(float* obs) const {
    std::copy(last_obs_, last_obs_ + OBS_DIM, obs);
}
