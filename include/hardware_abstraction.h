/**
 * @file hardware_abstraction.h
 * @brief 硬件抽象层 - 处理 Sim-to-Real 坐标映射
 * @author Claude Code
 * @date 2026-03-25
 *
 * @details 本模块负责在算法空间（Sim）和电机空间（Real）之间进行坐标变换
 *          核心功能：
 *          1. 加载 robot.yaml 中的 default_angles / offset / sign_array
 *          2. 提供 Real→Sim 映射（观测链路）
 *          3. 提供 Sim→Real 映射（控制链路）
 *
 * @note 坐标变换关系：
 *       观测链路: q_sim = (q_real - offset) * sign_array
 *                dq_sim = dq_real * sign_array
 *       控制链路: q_real = q_sim * sign_array + offset
 */

#ifndef HARDWARE_ABSTRACTION_H
#define HARDWARE_ABSTRACTION_H

#include "types.h"
#include <array>
#include <string>

/**
 * @brief 硬件配置结构体
 */
struct HardwareConfig {
    std::array<float, DOF_NUM> default_angles;  ///< 算法层基准姿态
    std::array<float, DOF_NUM> offset;          ///< 硬件零点偏置
    std::array<int, DOF_NUM> sign_array;        ///< 关节方向映射 (1 or -1)
};

/**
 * @brief 硬件抽象层类
 */
class HardwareAbstraction {
public:
    HardwareAbstraction();
    ~HardwareAbstraction() = default;

    /**
     * @brief 从 YAML 文件加载硬件配置
     * @param filename YAML 配置文件路径
     * @return 成功返回 true，失败返回 false
     */
    bool loadConfig(const std::string& filename);

    /**
     * @brief Real → Sim 位置映射（观测链路）
     * @param q_real 电机空间的关节位置
     * @param q_sim 算法空间的关节位置（输出）
     */
    void realToSim_position(const float* q_real, float* q_sim) const;

    /**
     * @brief Real → Sim 速度映射（观测链路）
     * @param dq_real 电机空间的关节速度
     * @param dq_sim 算法空间的关节速度（输出）
     */
    void realToSim_velocity(const float* dq_real, float* dq_sim) const;

    /**
     * @brief Sim → Real 位置映射（控制链路）
     * @param q_sim 算法空间的关节位置
     * @param q_real 电机空间的关节位置（输出）
     */
    void simToReal_position(const float* q_sim, float* q_real) const;

    /**
     * @brief 获取硬件配置
     * @return 硬件配置结构体的常量引用
     */
    const HardwareConfig& getConfig() const { return config_; }

    /**
     * @brief 打印当前配置（用于调试）
     */
    void printConfig() const;

private:
    HardwareConfig config_;  ///< 硬件配置数据
};

#endif // HARDWARE_ABSTRACTION_H
