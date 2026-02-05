/**
 * @file main.cpp
 * @brief 主程序入口文件
 * @author Zomnk
 * @date 2026-02-04
 *
 * @details 本文件是Jetson RL部署程序的主入口。
 *          主要功能：
 *          1. 解析命令行参数
 *          2. 加载标定配置文件
 *          3. 初始化UDP通信
 *          4. 加载TensorRT引擎
 *          5. 运行500Hz控制循环
 *
 * @note 使用方法:
 *       ./JetsonRLDeploy <engine_path> [--ip IP] [--port PORT] [--config FILE]
 */

#include "communication.h"
#include "trt_inference.h"
#include <iostream>
#include <fstream>
#include <cstring>
#include <csignal>
#include <cmath>
#include <unistd.h>

/*
 * ============================================================
 * 全局变量
 * ============================================================
 */

/// 运行标志，用于控制主循环
/// 当收到SIGINT或SIGTERM信号时设为false
volatile bool g_running = true;

/**
 * @brief 信号处理函数
 *
 * @details 处理SIGINT(Ctrl+C)和SIGTERM信号，
 *          设置g_running为false以优雅退出主循环。
 *
 * @param sig 信号编号
 */
void signalHandler(int sig) {
    std::cout << "\n收到信号 " << sig << ", 正在关闭..." << std::endl;
    g_running = false;
}

/**
 * @brief 打印使用说明
 *
 * @param prog 程序名称（argv[0]）
 */
void printUsage(const char* prog) {
    std::cout << "用法: " << prog << " <engine_path> [选项]" << std::endl;
    std::cout << "  engine_path: TensorRT引擎文件路径 (.engine)" << std::endl;
    std::cout << "  --ip <IP>:    ODroid IP地址 (默认: 192.168.5.159)" << std::endl;
    std::cout << "  --port <N>:   UDP端口 (默认: 10000)" << std::endl;
    std::cout << "  --config <F>: 标定文件路径 (默认: ../robot.yaml)" << std::endl;
}

/**
 * @brief 从YAML文件加载标定数据
 *
 * @details 解析robot.yaml文件，提取10个关节的初始位置。
 *          文件格式示例：
 *          ```yaml
 *          robot_config:
 *            init_pose:
 *              left_leg:
 *                yaw:   0.0
 *                roll:  0.1
 *                ...
 *          ```
 *
 * @param filename YAML文件路径
 * @param init_pos 输出的初始位置数组（10个float）
 * @return 成功读取10个值返回true，否则返回false
 */
bool loadCalibration(const std::string& filename, float init_pos[10]) {
    std::ifstream f(filename);
    if (!f.is_open()) return false;

    std::string line;
    int idx = 0;

    // 逐行解析YAML文件
    while (std::getline(f, line) && idx < 10) {
        // 查找冒号分隔符
        size_t pos = line.find(':');
        if (pos == std::string::npos) continue;

        // 提取冒号后的值
        std::string val = line.substr(pos + 1);

        // 移除注释部分
        size_t comment = val.find('#');
        if (comment != std::string::npos) val = val.substr(0, comment);

        // 跳过前导空白
        size_t start = val.find_first_not_of(" \t");
        if (start == std::string::npos) continue;

        // 尝试解析为浮点数
        try {
            init_pos[idx++] = std::stof(val.substr(start));
        } catch (...) {
            // 解析失败，跳过此行
        }
    }

    // 必须成功读取10个值
    return idx == 10;
}

/**
 * @brief 缓慢移动机器人到初始姿态
 *
 * @details 使用线性插值将机器人从当前位置平滑移动到标定的初始姿态。
 *          这是一个安全措施，避免机器人突然跳到目标位置。
 *
 *          插值公式: pos = current + alpha * (target - current)
 *          其中alpha从0线性增加到1
 *
 * @param udp UDP通信对象
 * @param init_pos 目标初始姿态（10个关节）
 * @param steps 插值步数（默认500步，约1秒）
 */
void moveToInitPose(UDPCommunication& udp, const float init_pos[10], int steps = 500) {
    std::cout << "正在移动到标定的初始姿态..." << std::endl;

    MsgRequest request;
    MsgResponse response;
    std::memset(&response, 0, sizeof(response));

    // 当前位置（从反馈获取）
    float current_pos[10] = {0};
    bool got_feedback = false;

    // ========== 步骤1: 获取当前位置 ==========
    // 尝试最多50次（约500ms）获取当前关节位置
    for (int i = 0; i < 50 && !got_feedback; i++) {
        udp.sendResponse(response);
        if (udp.receiveRequest(request)) {
            // 保存当前关节位置
            for (int j = 0; j < 10; j++) {
                current_pos[j] = request.q[j];
            }
            got_feedback = true;
        }
        usleep(10000);  // 10ms
    }

    if (!got_feedback) {
        std::cout << "未收到反馈，使用零位作为起点" << std::endl;
    }

    // ========== 步骤2: 线性插值移动 ==========
    for (int step = 0; step <= steps && g_running; step++) {
        // 计算插值系数 (0.0 -> 1.0)
        float alpha = (float)step / steps;

        // 计算插值位置
        for (int i = 0; i < 10; i++) {
            response.q_exp[i] = current_pos[i] + alpha * (init_pos[i] - current_pos[i]);
        }

        // 发送位置指令
        udp.sendResponse(response);
        udp.receiveRequest(request);

        usleep(2000);  // 2ms，500Hz
    }

    std::cout << "已到达初始姿态" << std::endl;
}

/**
 * @brief 主函数
 *
 * @details 程序执行流程：
 *          1. 解析命令行参数
 *          2. 注册信号处理函数
 *          3. 加载标定配置
 *          4. 初始化UDP通信
 *          5. 加载TensorRT引擎
 *          6. 移动到初始姿态
 *          7. 进入500Hz控制循环
 *          8. 收到退出信号后清理退出
 *
 * @param argc 命令行参数数量
 * @param argv 命令行参数数组
 * @return 正常退出返回0，错误返回1
 */
int main(int argc, char** argv) {
    // ========== 检查命令行参数 ==========
    if (argc < 2) {
        printUsage(argv[0]);
        return 1;
    }

    // ========== 解析命令行参数 ==========
    std::string engine_path = argv[1];              // TensorRT引擎路径
    std::string target_ip = "192.168.5.159";        // ODroid IP地址
    int port = 10000;                               // UDP端口
    std::string config_file = "../robot.yaml";      // 标定文件路径

    for (int i = 2; i < argc; i++) {
        std::string arg = argv[i];
        if (arg == "--ip" && i + 1 < argc) {
            target_ip = argv[++i];
        } else if (arg == "--port" && i + 1 < argc) {
            port = std::atoi(argv[++i]);
        } else if (arg == "--config" && i + 1 < argc) {
            config_file = argv[++i];
        }
    }

    // ========== 注册信号处理函数 ==========
    struct sigaction sa;
    std::memset(&sa, 0, sizeof(sa));
    sa.sa_handler = signalHandler;
    sa.sa_flags = 0;  // 不使用 SA_RESTART，允许阻塞调用被中断
    sigaction(SIGINT, &sa, nullptr);   // Ctrl+C
    sigaction(SIGTERM, &sa, nullptr);  // kill命令

    // ========== 打印启动信息 ==========
    std::cout << "========================================" << std::endl;
    std::cout << "  Jetson RL 部署程序 (TensorRT)" << std::endl;
    std::cout << "  引擎: " << engine_path << std::endl;
    std::cout << "  目标: " << target_ip << ":" << port << std::endl;
    std::cout << "========================================" << std::endl;

    // ========== 加载标定配置 ==========
    float calibrated_init_pos[10] = {0};
    if (loadCalibration(config_file, calibrated_init_pos)) {
        std::cout << "已加载标定文件: " << config_file << std::endl;
    } else {
        std::cout << "未找到标定文件，使用默认值" << std::endl;
    }

    // ========== 初始化UDP通信 ==========
    UDPCommunication udp(target_ip, port);
    if (!udp.init()) {
        std::cerr << "UDP初始化失败" << std::endl;
        return 1;
    }

    // ========== 加载TensorRT引擎 ==========
    TRTInference inference;
    if (!inference.loadEngine(engine_path)) {
        std::cerr << "TensorRT引擎加载失败" << std::endl;
        return 1;
    }

    // ========== 设置初始姿态并移动 ==========
    inference.setInitPose(calibrated_init_pos);
    moveToInitPose(udp, calibrated_init_pos);

    // ========== 开始控制循环 ==========
    std::cout << "========================================" << std::endl;
    std::cout << "开始控制循环 (500Hz)..." << std::endl;
    std::cout << "按 Ctrl+C 退出" << std::endl;
    std::cout << "========================================" << std::endl;

    MsgRequest request;
    MsgResponse response;
    std::memset(&response, 0, sizeof(response));

    // ========== 问题1: 用init_pos初始化response ==========
    // 避免第一个数据包发送零位置指令
    for (int i = 0; i < ACTION_DIM; ++i) {
        response.q_exp[i] = calibrated_init_pos[i];
    }

    int loop_count = 0;   // 循环计数
    int infer_count = 0;  // 推理计数

    // ========== 主控制循环 ==========
    while (g_running) {
        // 发送上一次的响应
        udp.sendResponse(response);

        // 接收新的请求
        if (udp.receiveRequest(request)) {
            float action[ACTION_DIM];

            // 执行推理
            bool infer_success = inference.infer(request, action);

            // ========== 检查输出中是否有NaN ==========
            bool has_nan = false;
            if (infer_success) {
                for (int i = 0; i < ACTION_DIM; ++i) {
                    if (std::isnan(action[i])) {
                        has_nan = true;
                        break;
                    }
                }
            }

            // ========== 推理失败、trigger!=1.0或输出有NaN时的处理 ==========
            if (infer_success && !has_nan) {
                // 推理成功且无NaN，使用推理结果
                for (int i = 0; i < ACTION_DIM; ++i) {
                    // 限制action范围在 [-1.57, 1.57]
                    float clamped = action[i];
                    if (clamped < -1.57f) clamped = -1.57f;
                    if (clamped > 1.57f) clamped = 1.57f;
                    response.q_exp[i] = clamped;
                }
                infer_count++;

                // 每0.5秒打印一次状态（250次 × 2ms = 500ms）
                if (infer_count % 250 == 0) {
                    // 获取观测向量
                    float obs[OBS_DIM];
                    inference.getLastObservation(obs);

                    std::cout << "[推理 #" << infer_count << "] 观测: ";
                    for (int i = 0; i < OBS_DIM; ++i) {
                        std::cout << obs[i] << " ";
                    }
                    std::cout << std::endl;

                    std::cout << "                动作: ";
                    for (int i = 0; i < ACTION_DIM; ++i) {
                        std::cout << response.q_exp[i] << " ";
                    }
                    std::cout << std::endl;
                }
            } else {
                // 推理失败、trigger!=1.0或输出有NaN，终止推理并回到初始姿态
                if (has_nan) {
                    std::cout << "\n[错误] 推理输出包含NaN，终止推理" << std::endl;
                } else if (!infer_success) {
                    std::cout << "\n[错误] 推理失败或trigger!=1.0，终止推理" << std::endl;
                }

                std::cout << "正在回到初始姿态..." << std::endl;
                moveToInitPose(udp, calibrated_init_pos);

                // 重新初始化response
                std::memset(&response, 0, sizeof(response));
                for (int i = 0; i < ACTION_DIM; ++i) {
                    response.q_exp[i] = calibrated_init_pos[i];
                }

                std::cout << "已回到初始姿态，重新开始控制循环" << std::endl;
                std::cout << "========================================" << std::endl;
            }
        }

        loop_count++;
        usleep(2000);  // 2ms，500Hz
    }

    // ========== 打印退出信息 ==========
    std::cout << "\n========================================" << std::endl;
    std::cout << "程序已退出" << std::endl;
    std::cout << "总循环次数: " << loop_count << std::endl;
    std::cout << "总推理次数: " << infer_count << std::endl;
    std::cout << "========================================" << std::endl;

    return 0;
}
