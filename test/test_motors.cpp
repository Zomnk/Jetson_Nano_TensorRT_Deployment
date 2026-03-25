/**
 * @file test_motors.cpp
 * @brief 电机控制测试程序（使用硬件抽象层）
 * @author Claude Code
 * @date 2026-03-25
 *
 * @details 本程序用于测试电机控制和验证关节方向映射。
 *          功能：
 *          1. 从 robot.yaml 读取硬件配置
 *          2. 在初始姿态基础上叠加正弦波
 *          3. 验证每个关节的运动方向是否正确
 *          4. 实时显示机器人状态
 *
 * @note 使用方法:
 *       ./test_motors [--ip IP] [--port PORT] [--config FILE] [--joint N]
 *       --joint N: 只测试指定关节（0-9），不指定则测试所有关节
 *
 * @warning 运行前请确保机器人处于安全状态！
 */

#define _USE_MATH_DEFINES
#include "hardware_abstraction.h"
#include <cmath>
#include <iostream>
#include <iomanip>
#include <cstring>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <csignal>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using namespace std;

struct MsgRequest {
    float trigger;
    float command[4];
    float eu_ang[3];
    float omega[3];
    float acc[3];
    float q[10];
    float dq[10];
    float tau[10];
    float init_pos[10];
};

struct MsgResponse {
    float q_exp[10];
    float dq_exp[10];
    float tau_exp[10];
};

const char* JOINT_NAMES[10] = {
    "左腿Yaw",
    "左腿Roll",
    "左腿Pitch",
    "左腿Knee",
    "左腿Ankle",
    "右腿Yaw",
    "右腿Roll",
    "右腿Pitch",
    "右腿Knee",
    "右腿Ankle"
};

volatile bool g_running = true;

void signal_handler(int sig) {
    cout << "\n收到信号 " << sig << ", 准备退出..." << endl;
    g_running = false;
}

int main(int argc, char** argv) {
    string target_ip = "192.168.5.159";
    int port = 10000;
    string config_file = "../robot.yaml";
    int test_joint = -1;  // -1 表示测试所有关节

    for (int i = 1; i < argc; i++) {
        string arg = argv[i];
        if (arg == "--ip" && i + 1 < argc) {
            target_ip = argv[++i];
        } else if (arg == "--port" && i + 1 < argc) {
            port = atoi(argv[++i]);
        } else if (arg == "--config" && i + 1 < argc) {
            config_file = argv[++i];
        } else if (arg == "--joint" && i + 1 < argc) {
            test_joint = atoi(argv[++i]);
            if (test_joint < 0 || test_joint > 9) {
                cerr << "错误: 关节编号必须在 0-9 之间" << endl;
                return 1;
            }
        }
    }

    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    cout << "========================================" << endl;
    cout << "  电机控制测试程序" << endl;
    cout << "  目标: " << target_ip << ":" << port << endl;
    if (test_joint >= 0) {
        cout << "  测试关节: " << JOINT_NAMES[test_joint] << " (#" << test_joint << ")" << endl;
    } else {
        cout << "  测试模式: 所有关节" << endl;
    }
    cout << "========================================" << endl;

    // 加载硬件配置
    HardwareAbstraction hw_abstraction;
    if (!hw_abstraction.loadConfig(config_file)) {
        cerr << "错误: 无法加载硬件配置" << endl;
        return 1;
    }
    hw_abstraction.printConfig();

    // 获取配置
    const auto& hw_config = hw_abstraction.getConfig();

    // 计算初始姿态在 Real 空间的位置
    float init_pose_sim[10];
    for (int i = 0; i < 10; i++) {
        init_pose_sim[i] = hw_config.default_angles[i];
    }

    float init_pose_real[10];
    hw_abstraction.simToReal_position(init_pose_sim, init_pose_real);

    cout << "\n初始姿态 (Real 空间):" << endl;
    cout << "  左腿: [" << fixed << setprecision(3);
    for (int i = 0; i < 5; i++) {
        cout << init_pose_real[i];
        if (i < 4) cout << ", ";
    }
    cout << "]" << endl;
    cout << "  右腿: [";
    for (int i = 5; i < 10; i++) {
        cout << init_pose_real[i];
        if (i < 9) cout << ", ";
    }
    cout << "]" << endl;

    // 创建UDP socket
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) {
        cerr << "错误: 无法创建socket" << endl;
        return 1;
    }

    // 设置接收超时
    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 100000;  // 100ms
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    // 绑定本地端口
    struct sockaddr_in local_addr;
    memset(&local_addr, 0, sizeof(local_addr));
    local_addr.sin_family = AF_INET;
    local_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    local_addr.sin_port = htons(port);

    if (bind(sock, (struct sockaddr*)&local_addr, sizeof(local_addr)) < 0) {
        cerr << "错误: 绑定端口失败" << endl;
        close(sock);
        return 1;
    }

    // 配置远程地址
    struct sockaddr_in remote_addr;
    memset(&remote_addr, 0, sizeof(remote_addr));
    remote_addr.sin_family = AF_INET;
    remote_addr.sin_addr.s_addr = inet_addr(target_ip.c_str());
    remote_addr.sin_port = htons(port);

    cout << "\n正在连接ODroid..." << endl;

    MsgRequest request;
    MsgResponse response;
    memset(&response, 0, sizeof(response));

    // 初始化response为初始姿态
    for (int i = 0; i < 10; i++) {
        response.q_exp[i] = init_pose_real[i];
    }

    // 等待连接
    bool connected = false;
    for (int i = 0; i < 50 && !connected; i++) {
        sendto(sock, (char*)&response, sizeof(response), 0,
               (struct sockaddr*)&remote_addr, sizeof(remote_addr));

        socklen_t addr_len = sizeof(remote_addr);
        if (recvfrom(sock, (char*)&request, sizeof(request), 0,
                     (struct sockaddr*)&remote_addr, &addr_len) > 0) {
            connected = true;
            cout << "已连接ODroid" << endl;
        }
        usleep(10000);
    }

    if (!connected) {
        cerr << "错误: 无法连接到ODroid" << endl;
        close(sock);
        return 1;
    }

    cout << "\n开始测试..." << endl;
    cout << "正弦波参数: 幅度=0.2 rad, 周期=2秒" << endl;
    cout << "按 Ctrl+C 退出" << endl;
    cout << endl;

    int loop_count = 0;
    const float amplitude = 0.2f;  // 正弦波幅度 (rad)
    const float frequency = 0.5f;  // 频率 (Hz)

    while (g_running) {
        // 计算正弦波值
        float t = loop_count * 0.002f;  // 时间 (秒)
        float sine_val = amplitude * sin(2.0f * M_PI * frequency * t);

        // 在 Sim 空间构建目标位置
        float target_sim[10];
        for (int i = 0; i < 10; i++) {
            if (test_joint < 0 || i == test_joint) {
                // 测试此关节：在 default_angles 基础上叠加正弦波
                target_sim[i] = hw_config.default_angles[i] + sine_val;
            } else {
                // 不测试此关节：保持 default_angles
                target_sim[i] = hw_config.default_angles[i];
            }
        }

        // 转换到 Real 空间
        float target_real[10];
        hw_abstraction.simToReal_position(target_sim, target_real);

        // 发送命令
        for (int i = 0; i < 10; i++) {
            response.q_exp[i] = target_real[i];
        }

        sendto(sock, (char*)&response, sizeof(response), 0,
               (struct sockaddr*)&remote_addr, sizeof(remote_addr));

        // 接收反馈
        socklen_t addr_len = sizeof(remote_addr);
        if (recvfrom(sock, (char*)&request, sizeof(request), 0,
                     (struct sockaddr*)&remote_addr, &addr_len) > 0) {

            // 每0.5秒打印一次状态
            if (loop_count % 250 == 0) {
                // 将 Real 空间的反馈转换到 Sim 空间
                float q_sim[10], dq_sim[10];
                hw_abstraction.realToSim_position(request.q, q_sim);
                hw_abstraction.realToSim_velocity(request.dq, dq_sim);

                cout << "[循环 #" << loop_count << "]" << endl;
                cout << "  正弦波值: " << fixed << setprecision(3) << sine_val << " rad" << endl;

                cout << "  关节位置 (Sim空间): [";
                for (int i = 0; i < 10; i++) {
                    cout << setw(6) << q_sim[i];
                    if (i < 9) cout << " ";
                }
                cout << "]" << endl;

                cout << "  关节速度 (Sim空间): [";
                for (int i = 0; i < 10; i++) {
                    cout << setw(6) << dq_sim[i];
                    if (i < 9) cout << " ";
                }
                cout << "]" << endl;

                cout << "  IMU欧拉角: roll=" << request.eu_ang[0]
                     << " pitch=" << request.eu_ang[1]
                     << " yaw=" << request.eu_ang[2] << endl;
                cout << endl;
            }
        }

        loop_count++;
        usleep(2000);  // 2ms, 500Hz
    }

    cout << "\n========================================" << endl;
    cout << "程序已退出" << endl;
    cout << "总循环次数: " << loop_count << endl;
    cout << "========================================" << endl;

    close(sock);
    return 0;
}
