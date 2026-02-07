/**
 * @file test_udp.cpp
 * @brief UDP通信测试程序
 * @author Zomnk
 * @date 2026-02-04
 *
 * @details 本程序用于测试Jetson与ODroid之间的UDP通信是否正常。
 *          不涉及模型推理，仅验证网络连接和数据收发。
 *
 * @note 使用方法:
 *       ./test_udp [IP] [端口]
 *       ./test_udp 192.168.137.4 10000
 */

#include <iostream>
#include <iomanip>
#include <cstring>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <csignal>

using namespace std;

/*
 * ============================================================
 * 消息结构体定义（与ODroid保持一致）
 * ============================================================
 */

/// ODroid发送的请求消息
struct MsgRequest {
    float trigger;          ///< 触发标志
    float command[4];       ///< 控制指令
    float eu_ang[3];        ///< 欧拉角
    float omega[3];         ///< 角速度
    float acc[3];           ///< 加速度
    float q[10];            ///< 关节位置
    float dq[10];           ///< 关节速度
    float tau[10];          ///< 关节力矩
    float init_pos[10];     ///< 初始位置
};

/// Jetson发送的响应消息
struct MsgResponse {
    float q_exp[10];        ///< 期望关节位置
    float dq_exp[10];       ///< 期望关节速度
    float tau_exp[10];      ///< 期望关节力矩
};

/// 运行标志
volatile bool g_running = true;

/**
 * @brief 信号处理函数
 * @param sig 信号编号
 */
void signal_handler(int sig) {
    cout << "\n收到信号 " << sig << ", 退出..." << endl;
    g_running = false;
}

/**
 * @brief 主函数
 *
 * @details 测试流程：
 *          1. 创建UDP socket并绑定端口
 *          2. 等待ODroid发送数据
 *          3. 收到数据后立即回复固定响应
 *          4. 统计收发包数量
 *
 * @param argc 参数数量
 * @param argv 参数数组: [IP] [端口]
 * @return 0表示正常退出
 */
int main(int argc, char** argv) {
    // 注册信号处理
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    // 解析命令行参数
    string ip = (argc >= 2) ? argv[1] : "192.168.137.4";
    int port = (argc >= 3) ? atoi(argv[2]) : 10000;

    // 打印测试信息
    cout << "========================================" << endl;
    cout << "  UDP通信测试" << endl;
    cout << "  目标: " << ip << ":" << port << endl;
    cout << "========================================" << endl;

    // ========== 创建UDP socket ==========
    int sock_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock_fd < 0) {
        cerr << "创建socket失败!" << endl;
        return 1;
    }

    // ========== 设置接收超时 ==========
    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 100000;  // 100ms
    setsockopt(sock_fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    // ========== 绑定本地端口 ==========
    struct sockaddr_in local_addr, remote_addr;
    memset(&local_addr, 0, sizeof(local_addr));
    local_addr.sin_family = AF_INET;
    local_addr.sin_addr.s_addr = INADDR_ANY;
    local_addr.sin_port = htons(port);

    if (bind(sock_fd, (struct sockaddr*)&local_addr, sizeof(local_addr)) < 0) {
        cerr << "Bind失败! 端口 " << port << " 可能被占用" << endl;
        close(sock_fd);
        return 1;
    }

    // ========== 配置远程地址 ==========
    memset(&remote_addr, 0, sizeof(remote_addr));
    remote_addr.sin_family = AF_INET;
    remote_addr.sin_addr.s_addr = inet_addr(ip.c_str());
    remote_addr.sin_port = htons(port);

    cout << "等待ODroid数据..." << endl;

    // ========== 初始化响应消息 ==========
    MsgRequest request;
    MsgResponse response;
    memset(&response, 0, sizeof(response));

    // 设置固定的测试响应值
    for (int i = 0; i < 10; i++) {
        response.q_exp[i] = 0.1f * (i + 1);  // 0.1, 0.2, ..., 1.0
    }

    socklen_t addr_len = sizeof(remote_addr);
    int recv_count = 0, send_count = 0;

    // ========== 主循环 ==========
    while (g_running) {
        char buf[512];

        // 接收数据
        int n = recvfrom(sock_fd, buf, sizeof(buf), 0,
                        (struct sockaddr*)&remote_addr, &addr_len);

        if (n > 0) {
            // 解析请求
            memcpy(&request, buf, sizeof(request));
            recv_count++;

            // 发送响应
            memcpy(buf, &response, sizeof(response));
            if (sendto(sock_fd, buf, sizeof(response), 0,
                      (struct sockaddr*)&remote_addr, addr_len) > 0) {
                send_count++;
            }

            // 每0.5秒打印一次状态
            if (recv_count % 250 == 0) {
                cout << "\n[#" << recv_count << "] 收发正常" << endl;
                cout << "  trigger: " << request.trigger << endl;
                cout << "  q[0-4]: [" << request.q[0] << ", " << request.q[1]
                     << ", " << request.q[2] << ", " << request.q[3]
                     << ", " << request.q[4] << "]" << endl;
            }
        }
    }

    // ========== 打印统计信息 ==========
    cout << "\n========================================" << endl;
    cout << "测试结束 - 发送: " << send_count << ", 接收: " << recv_count << endl;
    cout << "========================================" << endl;

    close(sock_fd);
    return 0;
}
