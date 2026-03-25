/**
 * @file hardware_abstraction.cpp
 * @brief 硬件抽象层实现
 * @author Claude Code
 * @date 2026-03-25
 */

#include "hardware_abstraction.h"
#include <fstream>
#include <sstream>
#include <iostream>
#include <iomanip>

HardwareAbstraction::HardwareAbstraction() {
    // 初始化为默认值
    config_.default_angles.fill(0.0f);
    config_.offset.fill(0.0f);
    config_.sign_array.fill(1);
}

bool HardwareAbstraction::loadConfig(const std::string& filename) {
    std::ifstream f(filename);
    if (!f.is_open()) {
        std::cerr << "错误: 无法打开配置文件 " << filename << std::endl;
        return false;
    }

    bool in_default = false, in_offset = false, in_sign = false;
    bool in_left = false, in_right = false;
    int count_default = 0, count_offset = 0, count_sign = 0;

    std::string line;
    while (std::getline(f, line)) {
        // 跳过注释和空行
        size_t first = line.find_first_not_of(" \t");
        if (first == std::string::npos || line[first] == '#') continue;

        // 检测段落
        if (line.find("default_angles:") != std::string::npos) {
            in_default = true;
            in_offset = false;
            in_sign = false;
            continue;
        }
        if (line.find("offset:") != std::string::npos) {
            in_default = false;
            in_offset = true;
            in_sign = false;
            continue;
        }
        if (line.find("sign_array:") != std::string::npos) {
            in_default = false;
            in_offset = false;
            in_sign = true;
            continue;
        }

        // 检测左右腿
        if (line.find("left_leg:") != std::string::npos) {
            in_left = true;
            in_right = false;
            continue;
        }
        if (line.find("right_leg:") != std::string::npos) {
            in_left = false;
            in_right = true;
            continue;
        }

        // 解析键值对
        size_t pos = line.find(':');
        if (pos == std::string::npos) continue;

        std::string key = line.substr(0, pos);
        std::string val_str = line.substr(pos + 1);

        // 移除注释
        size_t comment = val_str.find('#');
        if (comment != std::string::npos) {
            val_str = val_str.substr(0, comment);
        }

        // 移除前后空格
        size_t start = val_str.find_first_not_of(" \t");
        if (start == std::string::npos) continue;
        val_str = val_str.substr(start);

        size_t end = val_str.find_last_not_of(" \t\r\n");
        if (end != std::string::npos) {
            val_str = val_str.substr(0, end + 1);
        }

        try {
            float value = std::stof(val_str);
            int offset_idx = in_left ? 0 : 5;

            // 根据关节名称确定索引
            int joint_idx = -1;
            if (key.find("yaw") != std::string::npos) joint_idx = 0;
            else if (key.find("roll") != std::string::npos) joint_idx = 1;
            else if (key.find("pitch") != std::string::npos) joint_idx = 2;
            else if (key.find("knee") != std::string::npos) joint_idx = 3;
            else if (key.find("ankle") != std::string::npos) joint_idx = 4;

            if (joint_idx >= 0) {
                int global_idx = offset_idx + joint_idx;

                if (in_default) {
                    config_.default_angles[global_idx] = value;
                    count_default++;
                }
                else if (in_offset) {
                    config_.offset[global_idx] = value;
                    count_offset++;
                }
                else if (in_sign) {
                    config_.sign_array[global_idx] = (int)value;
                    count_sign++;
                }
            }
        } catch (const std::exception& e) {
            // 忽略解析错误
            continue;
        }
    }

    f.close();

    std::cout << "硬件配置加载完成: default_angles=" << count_default
              << ", offset=" << count_offset
              << ", sign_array=" << count_sign << std::endl;

    bool success = (count_default == 10 && count_offset == 10 && count_sign == 10);

    if (!success) {
        std::cerr << "警告: 配置文件不完整，某些参数可能使用默认值" << std::endl;
    }

    return success;
}

void HardwareAbstraction::realToSim_position(const float* q_real, float* q_sim) const {
    for (int i = 0; i < DOF_NUM; i++) {
        // 观测链路映射：Real → Sim
        // q_sim = (q_real - offset) * sign_array
        // 其中 offset = q_encoder_stand - default_angles
        // 这样 q_sim 就是相对于 default_angles 的偏差
        q_sim[i] = (q_real[i] - config_.offset[i]) * config_.sign_array[i];
    }
}

void HardwareAbstraction::realToSim_velocity(const float* dq_real, float* dq_sim) const {
    for (int i = 0; i < DOF_NUM; i++) {
        // dq_sim = dq_real * sign
        dq_sim[i] = dq_real[i] * config_.sign_array[i];
    }
}

void HardwareAbstraction::simToReal_position(const float* q_sim, float* q_real) const {
    for (int i = 0; i < DOF_NUM; i++) {
        // 控制链路映射：Sim → Real
        // q_real = q_sim * sign_array + offset
        // 其中 offset = q_encoder_stand - default_angles
        // 这样 q_real 就是电机应该收到的命令
        q_real[i] = q_sim[i] * config_.sign_array[i] + config_.offset[i];
    }
}

void HardwareAbstraction::printConfig() const {
    std::cout << "\n========== 硬件配置 ==========" << std::endl;

    std::cout << "\n[Default Angles (算法层基准)]" << std::endl;
    std::cout << "  左腿: [" << std::fixed << std::setprecision(3);
    for (int i = 0; i < 5; i++) {
        std::cout << config_.default_angles[i];
        if (i < 4) std::cout << ", ";
    }
    std::cout << "]" << std::endl;
    std::cout << "  右腿: [";
    for (int i = 5; i < 10; i++) {
        std::cout << config_.default_angles[i];
        if (i < 9) std::cout << ", ";
    }
    std::cout << "]" << std::endl;

    std::cout << "\n[Offset (硬件零点偏置)]" << std::endl;
    std::cout << "  左腿: [";
    for (int i = 0; i < 5; i++) {
        std::cout << config_.offset[i];
        if (i < 4) std::cout << ", ";
    }
    std::cout << "]" << std::endl;
    std::cout << "  右腿: [";
    for (int i = 5; i < 10; i++) {
        std::cout << config_.offset[i];
        if (i < 9) std::cout << ", ";
    }
    std::cout << "]" << std::endl;

    std::cout << "\n[Sign Array (关节方向)]" << std::endl;
    std::cout << "  左腿: [";
    for (int i = 0; i < 5; i++) {
        std::cout << std::setw(2) << config_.sign_array[i];
        if (i < 4) std::cout << ", ";
    }
    std::cout << "]" << std::endl;
    std::cout << "  右腿: [";
    for (int i = 5; i < 10; i++) {
        std::cout << std::setw(2) << config_.sign_array[i];
        if (i < 9) std::cout << ", ";
    }
    std::cout << "]" << std::endl;
    std::cout << "============================\n" << std::endl;
}
