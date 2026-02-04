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
 *          主要功能：
 *          1. 加载序列化的TensorRT引擎文件（.engine）
 *          2. 构建观测向量（39维）
 *          3. 执行GPU推理
 *          4. 输出动作向量（10维）
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
     * @brief 设置初始站立姿态
     * @param init_pos 10个关节的初始位置数组
     *
     * @details 初始姿态用于计算关节位置偏差，
     *          通常从robot.yaml标定文件加载。
     */
    void setInitPose(const float* init_pos);

    /**
     * @brief 重置推理状态
     *
     * @details 清零上次动作缓存和控制指令滤波器，
     *          通常在开始新的控制周期时调用。
     */
    void reset();

    /**
     * @brief 执行推理
     * @param request ODroid发送的请求消息（包含机器人状态）
     * @param action_out 输出的动作数组（10个关节目标位置）
     * @return 推理成功返回true，失败返回false
     *
     * @details 执行以下操作：
     *          1. 检查触发标志（trigger必须为1.0）
     *          2. 构建39维观测向量
     *          3. 将数据拷贝到GPU
     *          4. 执行TensorRT推理
     *          5. 将结果拷贝回CPU
     *          6. 应用动作滤波和限幅
     */
    bool infer(const MsgRequest& request, float* action_out);

private:
    /**
     * @brief 构建观测向量
     * @param request 请求消息
     * @param obs 输出的观测数组（39维）
     *
     * @details 观测向量组成：
     *          [0-2]   角速度 * OMEGA_SCALE
     *          [3-5]   欧拉角 * EU_ANG_SCALE
     *          [6-8]   控制指令（带死区和平滑滤波）
     *          [9-18]  关节位置偏差 * POS_SCALE
     *          [19-28] 关节速度 * VEL_SCALE
     *          [29-38] 上次动作
     */
    void buildObservation(const MsgRequest& request, float* obs);

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
    void* d_input_;         ///< GPU输入缓冲区（39个float）
    void* d_output_;        ///< GPU输出缓冲区（10个float）

    /*
     * ============================================================
     * 状态缓存
     * ============================================================
     */
    float init_pos_[DOF_NUM];       ///< 初始站立姿态（标定值）
    float last_action_[ACTION_DIM]; ///< 上次动作（用于观测和滤波）
    float cmd_x_, cmd_y_, cmd_rate_;///< 滤波后的控制指令

    /*
     * ============================================================
     * 缩放参数（与训练时保持一致）
     * ============================================================
     */
    static constexpr float OMEGA_SCALE = 0.25f;     ///< 角速度缩放系数
    static constexpr float EU_ANG_SCALE = 1.0f;     ///< 欧拉角缩放系数
    static constexpr float POS_SCALE = 1.0f;        ///< 位置缩放系数
    static constexpr float VEL_SCALE = 0.05f;       ///< 速度缩放系数
    static constexpr float LIN_VEL_SCALE = 2.0f;    ///< 线速度指令缩放系数
    static constexpr float ANG_VEL_SCALE = 0.25f;   ///< 角速度指令缩放系数
    static constexpr float SMOOTH = 0.03f;          ///< 控制指令平滑系数
    static constexpr float DEAD_ZONE = 0.01f;       ///< 控制指令死区

    bool engine_loaded_;    ///< 引擎加载状态标志
};

#endif // TRT_INFERENCE_H
