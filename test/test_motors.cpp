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
    cout << "\n收到信号 " << sig << ", 准备退出..." << endl;
    g_running = false;
}

/**
 * @brief 从YAML文件加载初始姿态
 */
bool load_init_pose(const string& filename, float init_pos[10]) {
    ifstream f(filename);
    if (!f.is_open()) {
        cerr << "警告: 无法打开配置文件 " << filename << endl;
        cerr << "      将使用默认初始姿态" << endl;
        return false;
    }

    cout << "正在读取配置文件: " << filename << " ... ";

    // 关节名称到索引的映射
    // 左腿: yaw(0), roll(1), pitch(2), knee(3), ankle(4)
    // 右腿: yaw(5), roll(6), pitch(7), knee(8), ankle(9)
    bool in_left_leg = false;
    bool in_right_leg = false;
    int count = 0;

    string line;
    while (getline(f, line)) {
        // 跳过注释行
        size_t first_char = line.find_first_not_of(" \t");
        if (first_char != string::npos && line[first_char] == '#') continue;

        // 检测当前在哪个腿的配置块
        if (line.find("left_leg:") != string::npos) {
            in_left_leg = true;
            in_right_leg = false;
            continue;
        }
        if (line.find("right_leg:") != string::npos) {
            in_left_leg = false;
            in_right_leg = true;
            continue;
        }

        // 只在腿配置块内解析关节数据
        if (!in_left_leg && !in_right_leg) continue;

        int offset = in_left_leg ? 0 : 5;  // 左腿偏移0，右腿偏移5
        int joint_idx = -1;

        if (line.find("yaw:") != string::npos) joint_idx = 0;
        else if (line.find("roll:") != string::npos) joint_idx = 1;
        else if (line.find("pitch:") != string::npos) joint_idx = 2;
        else if (line.find("knee:") != string::npos) joint_idx = 3;
        else if (line.find("ankle:") != string::npos) joint_idx = 4;

        if (joint_idx >= 0) {
            size_t pos = line.find(':');
            if (pos != string::npos) {
                string val = line.substr(pos + 1);
                size_t comment = val.find('#');
                if (comment != string::npos) val = val.substr(0, comment);
                size_t start = val.find_first_not_of(" \t");
                if (start != string::npos) {
                    try {
                        init_pos[offset + joint_idx] = stof(val.substr(start));
                        count++;
                    } catch (...) {}
                }
            }
        }
    }

    if (count == 10) {
        cout << "✓ 成功!" << endl;
        return true;
    } else {
        cerr << "\n错误: 配置文件不完整，仅读取到 " << count << " 个关节" << endl;
        return false;
    }
}

/**
 * @brief 打印观测信息（39维）
 */
void print_observation(const MsgRequest& obs, int count) {
    cout << "\n========== 观测信息 #" << count << " ==========" << endl;

    // 角速度 (3维)
    cout << "[角速度 omega] (rad/s)" << endl;
    cout << "  ω_x = " << fixed << setprecision(4) << obs.omega[0] << endl;
    cout << "  ω_y = " << obs.omega[1] << endl;
    cout << "  ω_z = " << obs.omega[2] << endl;

    // 欧拉角姿态 (3维)
    cout << "[欧拉角 eu_ang] (rad)" << endl;
    cout << "  Roll  = " << obs.eu_ang[0] << endl;
    cout << "  Pitch = " << obs.eu_ang[1] << endl;
    cout << "  Yaw   = " << obs.eu_ang[2] << endl;

    // 控制指令 (4维)
    cout << "[控制指令 command]" << endl;
    cout << "  vx = " << obs.command[0] << ", vy = " << obs.command[1]
         << ", yaw_rate = " << obs.command[2] << ", reserved = " << obs.command[3] << endl;

    // 关节位置 (10维)
    cout << "[关节位置 q] (rad)" << endl;
    cout << "  左腿: [" << obs.q[0] << ", " << obs.q[1] << ", "
         << obs.q[2] << ", " << obs.q[3] << ", " << obs.q[4] << "]" << endl;
    cout << "  右腿: [" << obs.q[5] << ", " << obs.q[6] << ", "
         << obs.q[7] << ", " << obs.q[8] << ", " << obs.q[9] << "]" << endl;

    // 关节速度 (10维)
    cout << "[关节速度 dq] (rad/s)" << endl;
    cout << "  左腿: [" << obs.dq[0] << ", " << obs.dq[1] << ", "
         << obs.dq[2] << ", " << obs.dq[3] << ", " << obs.dq[4] << "]" << endl;
    cout << "  右腿: [" << obs.dq[5] << ", " << obs.dq[6] << ", "
         << obs.dq[7] << ", " << obs.dq[8] << ", " << obs.dq[9] << "]" << endl;

    // 上次动作 (10维)
    cout << "[上次动作 last_action] (rad)" << endl;
    cout << "  左腿: [" << obs.init_pos[0] << ", " << obs.init_pos[1] << ", "
         << obs.init_pos[2] << ", " << obs.init_pos[3] << ", " << obs.init_pos[4] << "]" << endl;
    cout << "  右腿: [" << obs.init_pos[5] << ", " << obs.init_pos[6] << ", "
         << obs.init_pos[7] << ", " << obs.init_pos[8] << ", " << obs.init_pos[9] << "]" << endl;

    // 加速度 (3维)
    cout << "[加速度 acc] (g)" << endl;
    cout << "  [" << obs.acc[0] << ", " << obs.acc[1] << ", " << obs.acc[2] << "]" << endl;

    // 力矩 (10维)
    cout << "[关节力矩 tau] (Nm)" << endl;
    cout << "  左腿: [" << obs.tau[0] << ", " << obs.tau[1] << ", "
         << obs.tau[2] << ", " << obs.tau[3] << ", " << obs.tau[4] << "]" << endl;
    cout << "  右腿: [" << obs.tau[5] << ", " << obs.tau[6] << ", "
         << obs.tau[7] << ", " << obs.tau[8] << ", " << obs.tau[9] << "]" << endl;

    cout << "======================================" << endl;
}

/**
 * @brief 主函数
 */
int main(int argc, char** argv) {
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    cout << "========================================" << endl;
    cout << "  Jetson完整数据流测试" << endl;
    cout << "  功能: 发送正弦action，接收观测反馈" << endl;
    cout << "========================================" << endl;

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

    // ========== 加载初始姿态 ==========
    float init_pos[10] = {0};
    bool yaml_loaded = load_init_pose(yaml_file, init_pos);

    if (yaml_loaded) {
        cout << "\n初始姿态配置 (从 " << yaml_file << "):" << endl;
        cout << "  左腿: [";
        for (int i = 0; i < 5; i++) {
            cout << fixed << setprecision(3) << init_pos[i];
            if (i < 4) cout << ", ";
        }
        cout << "]" << endl;
        cout << "  右腿: [";
        for (int i = 5; i < 10; i++) {
            cout << init_pos[i];
            if (i < 9) cout << ", ";
        }
        cout << "]" << endl;
    } else {
        cout << "\n使用默认初始姿态（未找到配置文件）" << endl;
        cout << "提示: 运行 ./calibration_tool 进行标定" << endl;
    }
    cout << "========================================" << endl;

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
        cerr << "Bind端口失败! 请检查端口" << port << "是否被占用" << endl;
        close(sock_fd);
        return 1;
    }

    // 配置远程地址
    memset(&remote_addr, 0, sizeof(remote_addr));
    remote_addr.sin_family = AF_INET;
    remote_addr.sin_addr.s_addr = inet_addr(ip.c_str());
    remote_addr.sin_port = htons(port);

    cout << "本地监听: 0.0.0.0:" << port << endl;
    cout << "目标ODroid IP: " << ip << endl;
    cout << "Request消息大小: " << sizeof(MsgRequest) << " bytes" << endl;
    cout << "Response消息大小: " << sizeof(MsgResponse) << " bytes" << endl;
    cout << "========================================" << endl;
    cout << "正弦参数: 幅值=1.57rad (约90°), 周期=10秒" << endl;
    cout << "说明: 正弦波 + 初始位置 = 电机目标位置" << endl;
    cout << "========================================" << endl;
    cout << "开始完整数据流测试，按Ctrl+C退出" << endl;
    cout << "等待ODroid连接..." << endl;
    cout << "========================================" << endl;

    // ========== 正弦波参数 ==========
    const float amplitude = 1.57f;              // 幅值 1.57 rad (约90度)
    const float period = 10.0f;                 // 周期 10秒
    const float omega = 2.0f * M_PI / period;   // 角频率

    MsgRequest request;
    MsgResponse response;
    memset(&response, 0, sizeof(response));

    socklen_t addr_len = sizeof(remote_addr);
    bool connected = false;
    bool initialization_done = false;

    // 记录起始时间
    struct timeval start_tv;
    gettimeofday(&start_tv, NULL);
    uint64_t start_us = start_tv.tv_sec * 1000000ULL + start_tv.tv_usec;
    uint64_t connection_time_us = 0;
    uint64_t last_print_time_us = start_us;

    int send_count = 0;
    int recv_count = 0;
    int loop_count = 0;

    // ========== 主循环 ==========
    while (g_running) {
        char buf[512];

        // 接收ODroid数据
        int n = recvfrom(sock_fd, buf, sizeof(buf), 0,
                        (struct sockaddr*)&remote_addr, &addr_len);

        if (n > 0) {
            memcpy(&request, buf, sizeof(request));
            recv_count++;

            // 首次收到数据，标记为已连接
            if (!connected) {
                connected = true;
                char client_ip[INET_ADDRSTRLEN];
                inet_ntop(AF_INET, &remote_addr.sin_addr, client_ip, INET_ADDRSTRLEN);
                cout << "\n收到ODroid连接: " << client_ip << ":"
                     << ntohs(remote_addr.sin_port) << endl;
                cout << "开始数据交互..." << endl;

                // 记录连接时间
                struct timeval conn_tv;
                gettimeofday(&conn_tv, NULL);
                connection_time_us = conn_tv.tv_sec * 1000000ULL + conn_tv.tv_usec;

                cout << "\n等待5秒让机器人回到初始姿态..." << endl;
            }
        }

        // 获取当前时间
        struct timeval now_tv;
        gettimeofday(&now_tv, NULL);
        uint64_t now_us = now_tv.tv_sec * 1000000ULL + now_tv.tv_usec;

        // 检查初始化延迟（5秒）
        if (connected && !initialization_done) {
            uint64_t elapsed_since_connect = now_us - connection_time_us;
            if (elapsed_since_connect >= 5000000) {
                initialization_done = true;
                cout << "✓ 初始化完成，开始正弦波测试..." << endl;
                start_us = now_us;
                last_print_time_us = now_us;
            } else {
                // 初始化期间，发送初始位置
                for (int i = 0; i < 10; i++) {
                    response.q_exp[i] = init_pos[i];
                    response.dq_exp[i] = 0.0f;
                    response.tau_exp[i] = 0.0f;
                }

                // 显示倒计时
                float remaining_s = (5000000 - elapsed_since_connect) / 1000000.0f;
                if ((int)(remaining_s * 10) % 5 == 0) {
                    cout << "\r初始化中... " << fixed << setprecision(1)
                         << remaining_s << "s      " << flush;
                }

                if (connected) {
                    memcpy(buf, &response, sizeof(response));
                    sendto(sock_fd, buf, sizeof(response), 0,
                          (struct sockaddr*)&remote_addr, addr_len);
                }

                usleep(2000);
                continue;
            }
        }

        // 计算正弦波
        float t = (now_us - start_us) / 1000000.0f;
        float sine_val = amplitude * sin(omega * t);

        // 每500ms打印一次详细观测信息
        if (initialization_done && now_us - last_print_time_us >= 500000) {
            if (recv_count > 0) {
                print_observation(request, recv_count);
            }
            cout << "[当前时间] t=" << fixed << setprecision(3) << t
                 << "s, 目标位置=" << sine_val << " rad ("
                 << (sine_val * 180.0 / M_PI) << " deg)" << endl;
            last_print_time_us = now_us;
        }

        // 设置所有关节的目标位置 = 初始位置 + 正弦波
        for (int i = 0; i < 10; i++) {
            response.q_exp[i] = init_pos[i] + sine_val;
            response.dq_exp[i] = 0.0f;
            response.tau_exp[i] = 0.0f;
        }

        // 发送响应
        if (connected) {
            memcpy(buf, &response, sizeof(response));
            int send_num = sendto(sock_fd, buf, sizeof(response), 0,
                  (struct sockaddr*)&remote_addr, addr_len);
            if (send_num > 0) {
                send_count++;
            }
        }

        loop_count++;

        // 每100个循环打印一个点
        if (loop_count % 100 == 0) {
            cout << "." << flush;
        }

        usleep(2000);  // 2ms, 500Hz
    }

    // ========== 打印统计信息 ==========
    cout << "\n\n========================================" << endl;
    cout << "测试结束" << endl;
    cout << "总循环: " << loop_count << endl;
    cout << "发送Action包: " << send_count << endl;
    cout << "接收Obs包: " << recv_count << endl;
    if (send_count > 0) {
        cout << "丢包率: " << fixed << setprecision(2)
             << (1.0 - (float)recv_count / send_count) * 100 << "%" << endl;
    }
    cout << "========================================" << endl;

    close(sock_fd);
    return 0;
}
