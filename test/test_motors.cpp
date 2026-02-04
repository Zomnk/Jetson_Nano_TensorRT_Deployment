/**
 * @file test_motors.cpp
 * @brief 电机控制测试程序
 * @author Zomnk
 * @date 2026-02-04
 *
 * @details 本程序用于测试Jetson能否通过UDP驱动电机。
 *          发送正弦波位置指令，验证电机响应。
 *          不涉及模型推理，仅测试通信和电机控制。
 *
 * @note 使用方法:
 *       ./test_motors [--ip IP] [--port PORT] [--config FILE]
 *
 * @warning 运行前请确保机器人处于安全状态！
 *          正弦波会使所有关节同步运动。
 */

#define _USE_MATH_DEFINES
#include <cmath>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <cstring>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <sys/time.h>
#include <csignal>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using namespace std;

/*
 * ============================================================
 * 消息结构体定义
 * ============================================================
 */

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

/// 运行标志
volatile bool g_running = true;

/**
 * @brief 信号处理函数
 */
void signal_handler(int sig) {
    cout << "\n收到信号 " << sig << ", 退出..." << endl;
    g_running = false;
}

/**
 * @brief 从YAML文件加载初始姿态
 *
 * @param filename YAML文件路径
 * @param init_pos 输出的初始位置数组
 * @return 成功返回true
 */
bool load_init_pose(const string& filename, float init_pos[10]) {
    ifstream f(filename);
    if (!f.is_open()) return false;

    string line;
    int idx = 0;
    while (getline(f, line) && idx < 10) {
        size_t pos = line.find(':');
        if (pos == string::npos) continue;
        string val = line.substr(pos + 1);
        size_t comment = val.find('#');
        if (comment != string::npos) val = val.substr(0, comment);
        size_t start = val.find_first_not_of(" \t");
        if (start == string::npos) continue;
        try {
            init_pos[idx++] = stof(val.substr(start));
        } catch (...) {}
    }
    return idx == 10;
}

/**
 * @brief 主函数
 *
 * @details 测试流程：
 *          1. 加载标定文件（可选）
 *          2. 创建UDP连接
 *          3. 等待ODroid连接
 *          4. 发送正弦波位置指令
 *
 *          正弦波参数：
 *          - 幅值: 0.5 rad (约28.6度)
 *          - 周期: 10秒
 *          - 公式: pos = init_pos + 0.5 * sin(2π/10 * t)
 */
int main(int argc, char** argv) {
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    // ========== 解析命令行参数 ==========
    string ip = "192.168.5.159";
    int port = 10000;
    string yaml_file = "../robot.yaml";

    for (int i = 1; i < argc; i++) {
        string arg = argv[i];
        if (arg == "--config" && i + 1 < argc) yaml_file = argv[++i];
        else if (arg == "--ip" && i + 1 < argc) ip = argv[++i];
        else if (arg == "--port" && i + 1 < argc) port = atoi(argv[++i]);
    }

    // ========== 打印测试信息 ==========
    cout << "========================================" << endl;
    cout << "  电机控制测试 (正弦波)" << endl;
    cout << "  目标: " << ip << ":" << port << endl;
    cout << "========================================" << endl;

    // ========== 加载初始姿态 ==========
    float init_pos[10] = {0};
    if (load_init_pose(yaml_file, init_pos)) {
        cout << "已加载标定文件: " << yaml_file << endl;
    } else {
        cout << "使用默认初始位置 (全零)" << endl;
    }

    // ========== 创建UDP socket ==========
    int sock_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock_fd < 0) {
        cerr << "创建socket失败!" << endl;
        return 1;
    }

    // 设置接收超时
    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 100000;
    setsockopt(sock_fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    // ========== 绑定本地端口 ==========
    struct sockaddr_in local_addr, remote_addr;
    memset(&local_addr, 0, sizeof(local_addr));
    local_addr.sin_family = AF_INET;
    local_addr.sin_addr.s_addr = INADDR_ANY;
    local_addr.sin_port = htons(port);

    if (bind(sock_fd, (struct sockaddr*)&local_addr, sizeof(local_addr)) < 0) {
        cerr << "Bind失败!" << endl;
        close(sock_fd);
        return 1;
    }

    // 配置远程地址
    memset(&remote_addr, 0, sizeof(remote_addr));
    remote_addr.sin_family = AF_INET;
    remote_addr.sin_addr.s_addr = inet_addr(ip.c_str());
    remote_addr.sin_port = htons(port);

    cout << "等待ODroid连接..." << endl;
    cout << "正弦参数: 幅值=0.5rad, 周期=10s" << endl;

    // ========== 正弦波参数 ==========
    const float amplitude = 0.5f;               // 幅值 0.5 rad
    const float period = 10.0f;                 // 周期 10秒
    const float omega = 2.0f * M_PI / period;   // 角频率

    MsgRequest request;
    MsgResponse response;
    memset(&response, 0, sizeof(response));

    socklen_t addr_len = sizeof(remote_addr);
    bool connected = false;

    // 记录起始时间
    struct timeval start_tv;
    gettimeofday(&start_tv, NULL);
    uint64_t start_us = start_tv.tv_sec * 1000000ULL + start_tv.tv_usec;

    int count = 0;

    // ========== 主循环 ==========
    while (g_running) {
        char buf[512];

        // 接收ODroid数据
        int n = recvfrom(sock_fd, buf, sizeof(buf), 0,
                        (struct sockaddr*)&remote_addr, &addr_len);

        if (n > 0) {
            // 首次收到数据，标记为已连接
            if (!connected) {
                connected = true;
                cout << "已连接，开始发送正弦波..." << endl;
                // 重置起始时间
                gettimeofday(&start_tv, NULL);
                start_us = start_tv.tv_sec * 1000000ULL + start_tv.tv_usec;
            }
            memcpy(&request, buf, sizeof(request));
        }

        // 计算当前时间
        struct timeval now_tv;
        gettimeofday(&now_tv, NULL);
        uint64_t now_us = now_tv.tv_sec * 1000000ULL + now_tv.tv_usec;
        float t = (now_us - start_us) / 1000000.0f;  // 转换为秒

        // 计算正弦波值
        float sine_val = amplitude * sin(omega * t);

        // 设置所有关节的目标位置 = 初始位置 + 正弦波
        for (int i = 0; i < 10; i++) {
            response.q_exp[i] = init_pos[i] + sine_val;
            response.dq_exp[i] = 0.0f;
            response.tau_exp[i] = 0.0f;
        }

        // 发送响应
        if (connected) {
            memcpy(buf, &response, sizeof(response));
            sendto(sock_fd, buf, sizeof(response), 0,
                  (struct sockaddr*)&remote_addr, addr_len);
            count++;

            // 每0.5秒打印一次状态
            if (count % 250 == 0) {
                cout << "[t=" << fixed << setprecision(2) << t << "s] "
                     << "sine=" << setprecision(4) << sine_val << " rad" << endl;
            }
        }

        usleep(2000);  // 2ms, 500Hz
    }

    // ========== 打印统计信息 ==========
    cout << "\n========================================" << endl;
    cout << "测试结束 - 发送: " << count << " 包" << endl;
    cout << "========================================" << endl;

    close(sock_fd);
    return 0;
}
