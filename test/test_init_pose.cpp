/**
 * @file test_init_pose.cpp
 * @brief 初始姿态测试程序（使用硬件抽象层）
 * @author Claude Code
 * @date 2026-03-25
 *
 * @details 本程序用于测试机器人是否能正确回到初始站立姿态。
 *          功能：
 *          1. 从 robot.yaml 读取硬件配置
 *          2. 计算初始姿态在 Real 空间的目标位置
 *          3. 通过线性插值平滑移动到初始姿态
 *          4. 到达后保持初始位置不变
 *
 * @note 使用方法:
 *       ./test_init_pose [--ip IP] [--port PORT] [--config FILE] [--steps N]
 *
 * @warning 运行前请确保机器人处于安全状态！
 */

#include "hardware_abstraction.h"
#include <iostream>
#include <fstream>
#include <cstring>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <csignal>
#include <iomanip>

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

volatile bool g_running = true;

void signal_handler(int sig) {
    cout << "\n收到信号 " << sig << ", 准备退出..." << endl;
    g_running = false;
}

int main(int argc, char** argv) {
    string target_ip = "192.168.5.159";
    int port = 10000;
    string config_file = "../robot.yaml";
    int steps = 500;  // 线性插值步数

    for (int i = 1; i < argc; i++) {
        string arg = argv[i];
        if (arg == "--ip" && i + 1 < argc) {
            target_ip = argv[++i];
        } else if (arg == "--port" && i + 1 < argc) {
            port = atoi(argv[++i]);
        } else if (arg == "--config" && i + 1 < argc) {
            config_file = argv[++i];
        } else if (arg == "--steps" && i + 1 < argc) {
            steps = atoi(argv[++i]);
        }
    }

    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    cout << "========================================" << endl;
    cout << "  初始姿态测试程序" << endl;
    cout << "  目标: " << target_ip << ":" << port << endl;
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

    // 计算初始姿态在 Real 空间的目标位置
    // 初始姿态在 Sim 空间是 default_angles
    float target_sim[10];
    for (int i = 0; i < 10; i++) {
        target_sim[i] = hw_config.default_angles[i];
    }

    // 转换到 Real 空间
    float target_real[10];
    hw_abstraction.simToReal_position(target_sim, target_real);

    cout << "\n目标姿态 (Sim 空间 - default_angles):" << endl;
    cout << "  左腿: [" << fixed << setprecision(3);
    for (int i = 0; i < 5; i++) {
        cout << target_sim[i];
        if (i < 4) cout << ", ";
    }
    cout << "]" << endl;
    cout << "  右腿: [";
    for (int i = 5; i < 10; i++) {
        cout << target_sim[i];
        if (i < 9) cout << ", ";
    }
    cout << "]" << endl;

    cout << "\n目标姿态 (Real 空间 - 电机命令):" << endl;
    cout << "  左腿: [";
    for (int i = 0; i < 5; i++) {
        cout << target_real[i];
        if (i < 4) cout << ", ";
    }
    cout << "]" << endl;
    cout << "  右腿: [";
    for (int i = 5; i < 10; i++) {
        cout << target_real[i];
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

    // 获取当前位置
    float current_pos[10] = {0};
    bool got_feedback = false;

    for (int i = 0; i < 50 && !got_feedback; i++) {
        sendto(sock, (char*)&response, sizeof(response), 0,
               (struct sockaddr*)&remote_addr, sizeof(remote_addr));

        socklen_t addr_len = sizeof(remote_addr);
        if (recvfrom(sock, (char*)&request, sizeof(request), 0,
                     (struct sockaddr*)&remote_addr, &addr_len) > 0) {
            for (int j = 0; j < 10; j++) {
                current_pos[j] = request.q[j];
            }
            got_feedback = true;
            cout << "已连接ODroid" << endl;
        }
        usleep(10000);
    }

    if (!got_feedback) {
        cout << "警告: 未收到反馈，使用零位作为起点" << endl;
    }

    cout << "\n正在平滑移动到初始姿态..." << endl;

    // 线性插值移动
    for (int step = 0; step <= steps && g_running; step++) {
        float alpha = (float)step / steps;

        for (int i = 0; i < 10; i++) {
            response.q_exp[i] = current_pos[i] + alpha * (target_real[i] - current_pos[i]);
        }

        sendto(sock, (char*)&response, sizeof(response), 0,
               (struct sockaddr*)&remote_addr, sizeof(remote_addr));

        socklen_t addr_len = sizeof(remote_addr);
        recvfrom(sock, (char*)&request, sizeof(request), 0,
                 (struct sockaddr*)&remote_addr, &addr_len);

        if (step % 50 == 0) {
            cout << "进度: " << (int)(alpha * 100) << "%" << endl;
        }

        usleep(2000);  // 2ms
    }

    cout << "已到达初始姿态，保持位置不变..." << endl;
    cout << "按 Ctrl+C 退出" << endl;

    // 保持初始位置
    int loop_count = 0;
    while (g_running) {
        for (int i = 0; i < 10; i++) {
            response.q_exp[i] = target_real[i];
        }

        sendto(sock, (char*)&response, sizeof(response), 0,
               (struct sockaddr*)&remote_addr, sizeof(remote_addr));

        socklen_t addr_len = sizeof(remote_addr);
        recvfrom(sock, (char*)&request, sizeof(request), 0,
                 (struct sockaddr*)&remote_addr, &addr_len);

        loop_count++;
        if (loop_count % 250 == 0) {
            cout << "保持中... (循环 #" << loop_count << ")" << endl;
        }

        usleep(2000);
    }

    cout << "\n========================================" << endl;
    cout << "程序已退出" << endl;
    cout << "总循环次数: " << loop_count << endl;
    cout << "========================================" << endl;

    close(sock);
    return 0;
}
