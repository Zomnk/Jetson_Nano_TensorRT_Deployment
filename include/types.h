/**
 * @file types.h
 * @brief 数据类型定义头文件
 * @author Zomnk
 * @date 2026-02-04
 *
 * @details 本文件定义了Jetson与ODroid之间UDP通信所使用的消息结构体，
 *          以及强化学习模型的输入输出维度常量。
 *          所有结构体的内存布局必须与ODroid端保持完全一致。
 */

#ifndef TYPES_H
#define TYPES_H

#include <cstdint>

/**
 * @brief ODroid发送给Jetson的请求消息结构体
 *
 * @details 包含机器人的完整状态信息，作为强化学习模型的输入。
 *          数据由ODroid从STM32采集后通过UDP发送给Jetson。
 *          总大小: 1 + 4 + 3 + 3 + 3 + 10 + 10 + 10 + 10 = 54个float = 216字节
 */
struct MsgRequest {
    float trigger;          ///< 触发标志: 1.0表示开始推理, 其他值表示待机
    float command[4];       ///< 控制指令: [vx, vy, yaw_rate, reserved] 来自遥控器
    float eu_ang[3];        ///< 欧拉角: [roll, pitch, yaw] 单位:弧度
    float omega[3];         ///< 角速度: [wx, wy, wz] 单位:rad/s
    float acc[3];           ///< 加速度: [ax, ay, az] 单位:g
    float q[10];            ///< 关节位置: 10个关节的当前角度 单位:弧度
    float dq[10];           ///< 关节速度: 10个关节的当前角速度 单位:rad/s
    float tau[10];          ///< 关节力矩: 10个关节的当前力矩 单位:Nm
    float init_pos[10];     ///< 初始位置: 标定的初始站立姿态 单位:弧度
};

/**
 * @brief Jetson发送给ODroid的响应消息结构体
 *
 * @details 包含强化学习模型输出的动作指令。
 *          ODroid收到后将指令转发给STM32执行。
 *          总大小: 10 + 10 + 10 = 30个float = 120字节
 */
struct MsgResponse {
    float q_exp[10];        ///< 期望关节位置: 10个关节的目标角度 单位:弧度
    float dq_exp[10];       ///< 期望关节速度: 10个关节的目标角速度 单位:rad/s (通常为0)
    float tau_exp[10];      ///< 期望关节力矩: 10个关节的前馈力矩 单位:Nm (通常为0)
};

/*
 * ============================================================
 * 强化学习模型维度常量
 * ============================================================
 */

/// 观测空间维度 (模型输入)
/// 组成: 角速度(3) + 欧拉角(3) + 控制指令(3) + 关节位置偏差(10) + 关节速度(10) + 上次动作(10) = 39
constexpr int OBS_DIM = 39;

/// 动作空间维度 (模型输出)
/// 组成: 10个关节的目标位置
constexpr int ACTION_DIM = 10;

/// 自由度数量 (关节数)
/// 左腿5个 + 右腿5个 = 10
constexpr int DOF_NUM = 10;

#endif // TYPES_H
