/**
 * @file trt_inference.h
 * @brief TensorRT推理引擎类头文件
 * @author Zomnk
 * @date 2026-02-04
 *
 * @details 本文件定义了TRTInference类，用于加载TensorRT引擎并执行推理。
 *          支持TensorRT 8.x版本，自动适配不同版本的API。
 */

#ifndef TRT_INFERENCE_H
#define TRT_INFERENCE_H

#include "types.h"
#include <NvInfer.h>
#include <cuda_runtime.h>
#include <string>
#include <vector>
#include <memory>

/**
 * @brief TensorRT推理引擎类
 *
 * @details 封装了TensorRT引擎的加载、推理和资源管理。
 *          模型输入：390维 term-major 历史观测缓存 [1, 390]
 *          模型输出：10维动作向量 [1, 10]
 */
class TRTInference {
public:
    /**
     * @brief 构造函数，初始化成员变量
     */
    TRTInference();

    /**
     * @brief 析构函数，释放CUDA资源
     */
    ~TRTInference();

    /**
     * @brief 加载TensorRT引擎文件
     * @param engine_path 引擎文件路径（.engine格式）
     * @return 加载成功返回true，失败返回false
     *
     * @details 执行以下操作：
     *          1. 读取引擎文件到内存
     *          2. 创建TensorRT运行时
     *          3. 反序列化引擎
     *          4. 创建执行上下文
     *          5. 分配GPU内存
     */
    bool loadEngine(const std::string& engine_path);

    /**
     * @brief 设置硬件配置（用于 Sim-to-Real 映射）
     * @param default_angles 算法层基准姿态（10个关节）
     * @param offset 硬件零点偏置（10个关节）
     * @param sign_array 关节方向映射（10个关节，1 或 -1）
     *
     * @details 这些参数用于在观测和控制链路中进行坐标变换
     */
    void setHardwareConfig(const float* default_angles,
                          const float* offset,
                          const int* sign_array);

    /**
     * @brief 设置初始站立姿态（已废弃，保留用于向后兼容）
     * @param init_pos 10个关节的初始位置数组
     *
     * @details 此方法已被 setHardwareConfig() 替代。
     *          为保持向后兼容，此方法将 init_pos 设置为 offset。
     */
    void setInitPose(const float* init_pos);

    /**
     * @brief 重置推理状态
     *
     * @details 清零上次动作缓存和历史观测缓冲区，
     *          通常在开始新的控制周期或故障恢复时调用。
     */
    void reset();

    /**
     * @brief 执行推理
     * @param request ODroid发送的请求消息（包含机器人状态）
     * @param action_out 输出的动作数组（10个关节目标位置，Real空间）
     * @return 推理成功返回true，失败返回false
     *
     * @details 构建观测向量 → 更新历史缓存 → GPU推理 → Sim→Real映射
     */
    bool infer(const MsgRequest& request, float* action_out);

    /**
     * @brief 获取最后一次的观测向量
     * @param obs 输出的观测数组（39维）
     */
    void getLastObservation(float* obs) const;

private:
    /**
     * @brief 从欧拉角计算投影重力向量
     * @param eu_ang 欧拉角 [roll, pitch, yaw]
     * @param gravity_proj 输出的投影重力向量
     *
     * @details 投影重力向量是重力在机器人坐标系中的投影
     *          用于表示机器人的倾斜状态
     */
    void computeProjectedGravity(const float eu_ang[3], float gravity_proj[3]);

    /**
     * @brief 从四元数计算投影重力向量
     * @param quat 四元数 [w, x, y, z]
     * @param gravity_proj 输出的投影重力向量
     *
     * @details 使用四元数旋转变换将世界坐标系重力 [0, 0, -1]
     *          变换到机体坐标系，得到投影重力向量
     */
    void computeProjectedGravityFromQuat(const float quat[4], float gravity_proj[3]);

    /**
     * @brief 构建当前帧观测向量
     * @param request 请求消息
     * @param obs 输出的观测数组（39维）
     *
     * @details 当前帧观测组成：
     *          [0-2]   base_ang_vel
     *          [3-5]   projected_gravity
     *          [6-8]   velocity_commands
     *          [9-18]  joint_pos
     *          [19-28] joint_vel
     *          [29-38] last_action
     */
    void buildObservation(const MsgRequest& request, float* obs);

    /**
     * @brief 按 IsaacLab 的 term-major 方式更新历史缓存
     * @param obs 当前帧观测（39维）
     *
     * @details 历史缓存布局为 390 维：
     *          [0-29]    base_ang_vel 的 10 帧历史（每帧 3 维）
     *          [30-59]   projected_gravity 的 10 帧历史（每帧 3 维）
     *          [60-89]   velocity_commands 的 10 帧历史（每帧 3 维）
     *          [90-189]  joint_pos 的 10 帧历史（每帧 10 维）
     *          [190-289] joint_vel 的 10 帧历史（每帧 10 维）
     *          [290-389] last_action 的 10 帧历史（每帧 10 维）
     */
    void updateHistoryBuffer(const float* obs);

    /**
     * @brief 应用死区
     * @param value 输入值
     * @param deadzone 死区阈值
     * @return 如果|value| < deadzone返回0，否则返回value
     */
    float applyDeadzone(float value, float deadzone);

    /*
     * ============================================================
     * TensorRT相关成员
     * ============================================================
     */
    std::unique_ptr<nvinfer1::IRuntime> runtime_;           ///< TensorRT运行时
    std::unique_ptr<nvinfer1::ICudaEngine> engine_;         ///< TensorRT引擎
    std::unique_ptr<nvinfer1::IExecutionContext> context_;  ///< 执行上下文

    /*
     * ============================================================
     * CUDA相关成员
     * ============================================================
     */
    cudaStream_t stream_;   ///< CUDA流，用于异步操作
    void* d_obs_buf_;       ///< GPU输入缓冲区 - 历史观测缓存（390维, term-major）
    void* d_output_;        ///< GPU输出缓冲区（10个float）

    /*
     * ============================================================
     * 状态缓存
     * ============================================================
     */
    float default_angles_[DOF_NUM]; ///< 算法层基准姿态（训练时的默认姿态）
    float offset_[DOF_NUM];         ///< 硬件零点偏置（每台机器独立标定）
    int sign_array_[DOF_NUM];       ///< 关节方向映射（1 或 -1）
    float last_action_[ACTION_DIM]; ///< 上次动作（用于观测构建）
    float action_temp_[ACTION_DIM]; ///< 临时动作缓存（网络原始输出，用于观测构建）
    float last_obs_[OBS_DIM];       ///< 最后一次的观测向量（用于调试输出）

    /*
     * ============================================================
     * 历史观测缓存（IsaacLab term-major 布局）
     * ============================================================
     */
    static constexpr int HISTORY_LENGTH = 10;  ///< 历史缓存长度（帧数）
    static constexpr int ANG_VEL_DIM = 3;      ///< base_ang_vel 维度
    static constexpr int GRAVITY_DIM = 3;      ///< projected_gravity 维度
    static constexpr int CMD_DIM = 3;          ///< velocity_commands 维度
    static constexpr int JOINT_DIM = 10;       ///< joint_pos/joint_vel/action 维度

    ///< 各 term 的独立历史缓冲区（每个都是滑动窗口，最新帧在末尾）
    float hist_ang_vel_[HISTORY_LENGTH * ANG_VEL_DIM];   ///< [0-29]   base_ang_vel 历史
    float hist_gravity_[HISTORY_LENGTH * GRAVITY_DIM];   ///< [30-59]  projected_gravity 历史
    float hist_cmd_[HISTORY_LENGTH * CMD_DIM];           ///< [60-89]  velocity_commands 历史
    float hist_joint_pos_[HISTORY_LENGTH * JOINT_DIM];   ///< [90-189] joint_pos 历史
    float hist_joint_vel_[HISTORY_LENGTH * JOINT_DIM];   ///< [190-289] joint_vel 历史
    float hist_action_[HISTORY_LENGTH * JOINT_DIM];      ///< [290-389] last_action 历史

    float obs_buf_[HISTORY_LENGTH * OBS_DIM];  ///< 拼接后的 term-major 历史缓存（传给GPU）

    /*
     * ============================================================
     * 缩放参数（与 IsaacLab 训练保持一致）
     * ============================================================
     */
    static constexpr float OMEGA_SCALE = 1.0f;      ///< 角速度缩放系数

    bool engine_loaded_;    ///< 引擎加载状态标志
};

#endif // TRT_INFERENCE_H
