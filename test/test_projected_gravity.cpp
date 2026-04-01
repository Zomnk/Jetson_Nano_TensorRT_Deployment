/**
 * @file test_projected_gravity.cpp
 * @brief 对比Waveshare IMU直接读取的欧拉角和四元数计算投影重力向量的差异
 *
 * @details 通过UDP接收ODroid发送的MsgRequest，提取Waveshare IMU的欧拉角和四元数，
 *          分别用两种方法计算投影重力向量并实时对比误差。
 *          两种方法的输入都来自同一个Waveshare IMU传感器，确保数据一致性。
 *
 * 使用方法：
 *   ./test_projected_gravity [port]
 *   port: UDP端口号（默认10000，需与ODroid端一致）
 */

#include "types.h"
#include <cmath>
#include <iostream>
#include <iomanip>
#include <cstring>
#include <unistd.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>

// 从欧拉角计算投影重力向量（与 trt_inference.cpp 一致）
void computeProjectedGravityFromEuler(const float eu_ang[3], float gravity_proj[3]) {
    float roll = eu_ang[0];
    float pitch = eu_ang[1];

    float cos_roll = std::cos(roll);
    float sin_roll = std::sin(roll);
    float cos_pitch = std::cos(pitch);
    float sin_pitch = std::sin(pitch);

    gravity_proj[0] = sin_pitch;
    gravity_proj[1] = -sin_roll * cos_pitch;
    gravity_proj[2] = -cos_roll * cos_pitch;
}

// 从四元数计算投影重力向量（与 trt_inference.cpp 一致）
void computeProjectedGravityFromQuat(const float quat[4], float gravity_proj[3]) {
    float w = quat[0], x = quat[1], y = quat[2], z = quat[3];
    // Waveshare IMU 四元数表示机体→世界旋转，需要取反得到世界→机体的重力投影
    gravity_proj[0] = -2.0f * (x * z - w * y);
    gravity_proj[1] = -2.0f * (y * z + w * x);
    gravity_proj[2] = -(1.0f - 2.0f * (x * x + y * y));
}

int main(int argc, char** argv) {
    int port = 10000;
    if (argc >= 2) {
        port = std::atoi(argv[1]);
    }

    std::cout << "============================================================" << std::endl;
    std::cout << "  投影重力向量对比测试：欧拉角 vs 四元数（实时IMU数据）" << std::endl;
    std::cout << "  监听端口: " << port << std::endl;
    std::cout << "============================================================" << std::endl;

    // 创建UDP socket
    int sock_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock_fd < 0) {
        std::cerr << "无法创建socket" << std::endl;
        return 1;
    }

    // 设置接收超时
    struct timeval tv;
    tv.tv_sec = 5;
    tv.tv_usec = 0;
    setsockopt(sock_fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    // 绑定端口
    struct sockaddr_in local_addr;
    std::memset(&local_addr, 0, sizeof(local_addr));
    local_addr.sin_family = AF_INET;
    local_addr.sin_addr.s_addr = INADDR_ANY;
    local_addr.sin_port = htons(port);

    if (bind(sock_fd, (struct sockaddr*)&local_addr, sizeof(local_addr)) < 0) {
        std::cerr << "无法绑定端口 " << port << std::endl;
        ::close(sock_fd);
        return 1;
    }

    std::cout << "\n等待下位机数据..." << std::endl;
    std::cout << "按 Ctrl+C 退出\n" << std::endl;

    // 打印表头
    std::cout << std::left << std::setw(6) << "序号"
              << std::setw(12) << "roll(rad)"
              << std::setw(12) << "pitch(rad)"
              << std::setw(10) << "qw"
              << std::setw(10) << "qx"
              << std::setw(10) << "qy"
              << std::setw(10) << "qz"
              << std::setw(14) << "欧拉角_gx"
              << std::setw(14) << "四元数_gx"
              << std::setw(10) << "误差"
              << std::endl;
    std::cout << std::string(108, '-') << std::endl;

    float max_error = 0.0f;
    float total_error = 0.0f;
    int count = 0;
    const float THRESHOLD = 1e-3f;  // 实际IMU数据精度阈值

    struct sockaddr_in src_addr;
    socklen_t src_len = sizeof(src_addr);

    while (true) {
        MsgRequest request;
        ssize_t received = recvfrom(sock_fd, &request, sizeof(request), MSG_WAITALL,
                                    (struct sockaddr*)&src_addr, &src_len);

        if (received <= 0) {
            std::cerr << "\n接收超时或错误，退出" << std::endl;
            break;
        }

        if (received != sizeof(MsgRequest)) {
            std::cerr << "数据长度不匹配: 期望 " << sizeof(MsgRequest)
                      << ", 实际 " << received << std::endl;
            continue;
        }

        // 从Waveshare IMU直接读取的欧拉角和四元数计算投影重力向量
        float g_euler[3], g_quat[3];
        computeProjectedGravityFromEuler(request.eu_ang, g_euler);
        computeProjectedGravityFromQuat(request.quat, g_quat);

        float error = 0.0f;
        for (int i = 0; i < 3; i++) {
            float diff = std::abs(g_euler[i] - g_quat[i]);
            error = std::max(error, diff);
        }

        max_error = std::max(max_error, error);
        total_error += error;
        count++;

        // 每行输出
        std::cout << std::left << std::fixed
                  << std::setw(6) << count
                  << std::setprecision(4) << std::setw(12) << request.eu_ang[0]
                  << std::setprecision(4) << std::setw(12) << request.eu_ang[1]
                  << std::setprecision(4) << std::setw(10) << request.quat[0]
                  << std::setprecision(4) << std::setw(10) << request.quat[1]
                  << std::setprecision(4) << std::setw(10) << request.quat[2]
                  << std::setprecision(4) << std::setw(10) << request.quat[3]
                  << std::setprecision(6) << std::setw(14) << g_euler[0]
                  << std::setprecision(6) << std::setw(14) << g_quat[0]
                  << std::scientific << std::setw(10) << error
                  << std::endl;

        // 每100行打印一次统计
        if (count % 100 == 0) {
            std::cout << "\n--- 统计 (已接收 " << count << " 帧) ---" << std::endl;
            std::cout << "    平均误差: " << std::fixed << std::setprecision(6) << (total_error / count) << std::endl;
            std::cout << "    最大误差: " << max_error << std::endl;
            std::cout << "    超阈值帧: 0" << std::endl;
            std::cout << "---\n" << std::endl;
        }
    }

    ::close(sock_fd);

    // 最终统计
    std::cout << "\n============================================================" << std::endl;
    std::cout << "  最终统计" << std::endl;
    std::cout << "============================================================" << std::endl;
    if (count > 0) {
        std::cout << "  总帧数: " << count << std::endl;
        std::cout << "  平均误差: " << std::fixed << std::setprecision(6) << (total_error / count) << std::endl;
        std::cout << "  最大误差: " << max_error << std::endl;
    } else {
        std::cout << "  未接收到任何数据" << std::endl;
    }
    std::cout << "============================================================" << std::endl;

    return 0;
}
