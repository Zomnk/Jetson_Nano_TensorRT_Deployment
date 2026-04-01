/**
 * @file calibration_tool.cpp
 * @brief 双足机器人 Sim-to-Real 标定工具
 * @author Zomnk
 * @date 2026-03-25
 *
 * @note 功能说明：
 *       1. 默认写入算法层基准姿态（default_angles）
 *       2. 通过UDP驱动机器人平滑移动到站立姿态
 *       3. 到达后读取编码器值，计算硬件零点偏置（offset）
 *       4. 保存完整的 robot.yaml 配置文件
 *
 * @note 使用方法：
 *       ./calibration_tool [--ip IP] [--port PORT]
 */

#include <iostream>
#include <iomanip>
#include <fstream>
#include <cstring>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <csignal>
#include <ctime>
#include <string>
#include <sstream>
#include <fcntl.h>
#include "types.h"

using namespace std;

/*
 * ============================================================
 * 关节名称映射
 * ============================================================
 */
const char* JOINT_NAMES[10] = {
    "左腿Yaw", "左腿Roll", "左腿Pitch", "左腿Knee", "左腿Ankle",
    "右腿Yaw", "右腿Roll", "右腿Pitch", "右腿Knee", "右腿Ankle"
};

const char* YAML_KEYS[10] = {
    "yaw", "roll", "pitch", "knee", "ankle",
    "yaw", "roll", "pitch", "knee", "ankle"
};

/*
 * ============================================================
 * 默认算法层基准姿态（训练时使用的站立姿态）
 * ============================================================
 */
const float DEFAULT_ANGLES[10] = {
    0.0f, 0.0f, 0.18f, 1.11f, 0.92f,   // 左腿
    0.0f, 0.0f, 0.18f, 1.11f, 0.92f    // 右腿
};

/*
 * ============================================================
 * 全局变量
 * ============================================================
 */
volatile bool g_running = true;

void signal_handler(int sig) {
    cout << "\n收到信号 " << sig << ", 退出标定..." << endl;
    g_running = false;
}

/**
 * @brief 保存 robot.yaml 配置文件
 */
bool save_yaml(const float default_angles[10],
               const float offset[10],
               const int sign_array[10],
               const string& filename) {
    ofstream f(filename);
    if (!f.is_open()) {
        cerr << "无法创建文件: " << filename << endl;
        return false;
    }

    time_t now = time(nullptr);
    char time_str[100];
    strftime(time_str, sizeof(time_str), "%Y-%m-%d %H:%M:%S", localtime(&now));

    f << "# 双足机器人 Sim-to-Real 配置文件" << endl;
    f << "# 生成时间: " << time_str << endl;
    f << "# 单位: 弧度 (rad) 和方向系数" << endl;
    f << endl;
    f << "robot_config:" << endl;

    f << "  # ========================================" << endl;
    f << "  # 算法层基准姿态 (default_angles)" << endl;
    f << "  # ========================================" << endl;
    f << "  default_angles:" << endl;
    f << "    left_leg:" << endl;
    for (int i = 0; i < 5; i++) {
        f << "      " << left << setw(6) << YAML_KEYS[i] << ": "
          << fixed << setprecision(6) << setw(10) << default_angles[i] << "  # rad" << endl;
    }
    f << "    right_leg:" << endl;
    for (int i = 5; i < 10; i++) {
        f << "      " << left << setw(6) << YAML_KEYS[i] << ": "
          << fixed << setprecision(6) << setw(10) << default_angles[i] << "  # rad" << endl;
    }
    f << endl;

    f << "  # ========================================" << endl;
    f << "  # 硬件零点偏置 (offset)" << endl;
    f << "  # ========================================" << endl;
    f << "  offset:" << endl;
    f << "    left_leg:" << endl;
    for (int i = 0; i < 5; i++) {
        f << "      " << left << setw(6) << YAML_KEYS[i] << ": "
          << fixed << setprecision(6) << setw(10) << offset[i] << "  # rad" << endl;
    }
    f << "    right_leg:" << endl;
    for (int i = 5; i < 10; i++) {
        f << "      " << left << setw(6) << YAML_KEYS[i] << ": "
          << fixed << setprecision(6) << setw(10) << offset[i] << "  # rad" << endl;
    }
    f << endl;

    f << "  # ========================================" << endl;
    f << "  # 关节方向映射 (sign_array)" << endl;
    f << "  # ========================================" << endl;
    f << "  sign_array:" << endl;
    f << "    left_leg:" << endl;
    for (int i = 0; i < 5; i++) {
        f << "      " << left << setw(6) << YAML_KEYS[i] << ": "
          << setw(2) << sign_array[i] << "  # 1 or -1" << endl;
    }
    f << "    right_leg:" << endl;
    for (int i = 5; i < 10; i++) {
        f << "      " << left << setw(6) << YAML_KEYS[i] << ": "
          << setw(2) << sign_array[i] << "  # 1 or -1" << endl;
    }
    f << endl;

    f << "# ========================================" << endl;
    f << "# 坐标变换关系：" << endl;
    f << "# - 观测链路 (Real → Sim):" << endl;
    f << "#   q_sim = (q_real - offset) * sign_array" << endl;
    f << "#   dq_sim = dq_real * sign_array" << endl;
    f << "# - 控制链路 (Sim → Real):" << endl;
    f << "#   q_real = q_sim * sign_array + offset" << endl;

    f.close();
    return true;
}

/**
 * @brief 从用户输入读取整数数组
 */
bool read_int_array(const string& prompt, int arr[10], int default_val = 1) {
    cout << prompt << endl;
    cout << "输入格式: 10个整数（1 或 -1），用空格分隔" << endl;
    cout << "或直接按 Enter 使用默认值 [" << default_val << " × 10]: ";

    string line;
    getline(cin, line);

    if (line.empty()) {
        for (int i = 0; i < 10; i++) arr[i] = default_val;
        return true;
    }

    istringstream iss(line);
    for (int i = 0; i < 10; i++) {
        if (!(iss >> arr[i])) {
            cerr << "输入格式错误，需要10个数值" << endl;
            return false;
        }
        if (arr[i] != 1 && arr[i] != -1) {
            cerr << "sign_array 的值必须是 1 或 -1" << endl;
            return false;
        }
    }
    return true;
}

int main(int argc, char** argv) {
    string target_ip = "192.168.5.159";
    int port = 10000;

    for (int i = 1; i < argc; i++) {
        string arg = argv[i];
        if (arg == "--ip" && i + 1 < argc) target_ip = argv[++i];
        else if (arg == "--port" && i + 1 < argc) port = atoi(argv[++i]);
    }

    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    cout << "========================================" << endl;
    cout << "  双足机器人 Sim-to-Real 标定工具" << endl;
    cout << "  目标: " << target_ip << ":" << port << endl;
    cout << "========================================" << endl;
    cout << endl;

    // ========== 默认 default_angles ==========
    float default_angles[10];
    memcpy(default_angles, DEFAULT_ANGLES, sizeof(DEFAULT_ANGLES));

    cout << "【默认算法层基准姿态 (default_angles)】" << endl;
    cout << "  左腿: [";
    for (int i = 0; i < 5; i++) {
        cout << fixed << setprecision(2) << default_angles[i];
        if (i < 4) cout << ", ";
    }
    cout << "]" << endl;
    cout << "  右腿: [";
    for (int i = 5; i < 10; i++) {
        cout << fixed << setprecision(2) << default_angles[i];
        if (i < 9) cout << ", ";
    }
    cout << "]" << endl;
    cout << endl;

    // ========== 创建UDP socket ==========
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) {
        cerr << "无法创建socket" << endl;
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
        cerr << "绑定端口失败" << endl;
        close(sock);
        return 1;
    }

    // 配置远程地址
    struct sockaddr_in remote_addr;
    memset(&remote_addr, 0, sizeof(remote_addr));
    remote_addr.sin_family = AF_INET;
    remote_addr.sin_addr.s_addr = inet_addr(target_ip.c_str());
    remote_addr.sin_port = htons(port);

    cout << "正在连接 ODroid..." << endl;

    MsgRequest request;
    MsgResponse response;
    memset(&response, 0, sizeof(response));

    // 获取当前位置
    float current_pos[10] = {0};
    bool got_feedback = false;

    for (int i = 0; i < 50 && !got_feedback && g_running; i++) {
        sendto(sock, (char*)&response, sizeof(response), 0,
               (struct sockaddr*)&remote_addr, sizeof(remote_addr));

        socklen_t addr_len = sizeof(remote_addr);
        if (recvfrom(sock, (char*)&request, sizeof(request), 0,
                     (struct sockaddr*)&remote_addr, &addr_len) > 0) {
            memcpy(current_pos, request.q, sizeof(current_pos));
            got_feedback = true;
            cout << "已连接 ODroid" << endl;
        }
        usleep(10000);
    }

    if (!got_feedback || !g_running) {
        cerr << "错误: 无法连接到 ODroid" << endl;
        close(sock);
        return 1;
    }

    cout << "\n当前位置 (Real 空间):" << endl;
    cout << "  左腿: [";
    for (int i = 0; i < 5; i++) {
        cout << fixed << setprecision(4) << current_pos[i];
        if (i < 4) cout << ", ";
    }
    cout << "]" << endl;
    cout << "  右腿: [";
    for (int i = 5; i < 10; i++) {
        cout << fixed << setprecision(4) << current_pos[i];
        if (i < 9) cout << ", ";
    }
    cout << "]" << endl;

    // ========== 计算目标姿态 (Real 空间) ==========
    // q_real = default_angles * sign_array + offset
    // 标定时 offset=0, sign_array=1, 所以 q_real = default_angles
    float target_real[10];
    memcpy(target_real, default_angles, sizeof(default_angles));

    cout << "\n目标姿态 (Real 空间):" << endl;
    cout << "  左腿: [";
    for (int i = 0; i < 5; i++) {
        cout << fixed << setprecision(4) << target_real[i];
        if (i < 4) cout << ", ";
    }
    cout << "]" << endl;
    cout << "  右腿: [";
    for (int i = 5; i < 10; i++) {
        cout << fixed << setprecision(4) << target_real[i];
        if (i < 9) cout << ", ";
    }
    cout << "]" << endl;

    // ========== 线性插值平滑移动 ==========
    const int STEPS = 500;
    cout << "\n正在平滑移动到目标姿态 (" << STEPS << " 步)..." << endl;

    for (int step = 0; step <= STEPS && g_running; step++) {
        float alpha = (float)step / STEPS;

        for (int i = 0; i < 10; i++) {
            response.q_exp[i] = current_pos[i] + alpha * (target_real[i] - current_pos[i]);
        }

        sendto(sock, (char*)&response, sizeof(response), 0,
               (struct sockaddr*)&remote_addr, sizeof(remote_addr));

        socklen_t addr_len = sizeof(remote_addr);
        recvfrom(sock, (char*)&request, sizeof(request), 0,
                 (struct sockaddr*)&remote_addr, &addr_len);

        if (step % 50 == 0) {
            cout << "  进度: " << (int)(alpha * 100) << "%" << endl;
        }

        usleep(2000);  // 2ms
    }

    if (!g_running) {
        cout << "标定已中断" << endl;
        close(sock);
        return 0;
    }

    cout << "已到达目标姿态" << endl;

    // ========== 等待稳定后读取编码器值 ==========
    cout << "\n等待姿态稳定 (1秒)..." << endl;
    usleep(1000000);

    // 读取稳定后的编码器值
    float q_encoder_stand[10] = {0};
    for (int i = 0; i < 10 && g_running; i++) {
        sendto(sock, (char*)&response, sizeof(response), 0,
               (struct sockaddr*)&remote_addr, sizeof(remote_addr));

        socklen_t addr_len = sizeof(remote_addr);
        if (recvfrom(sock, (char*)&request, sizeof(request), 0,
                     (struct sockaddr*)&remote_addr, &addr_len) > 0) {
            for (int j = 0; j < 10; j++) {
                q_encoder_stand[j] += request.q[j];
            }
        }
        usleep(2000);
    }

    if (!g_running) {
        cout << "标定已中断" << endl;
        close(sock);
        return 0;
    }

    // 取平均值
    for (int i = 0; i < 10; i++) {
        q_encoder_stand[i] /= 10.0f;
    }

    cout << "\n已读取编码器值 (q_encoder_stand):" << endl;
    cout << "  左腿: [";
    for (int i = 0; i < 5; i++) {
        cout << fixed << setprecision(6) << q_encoder_stand[i];
        if (i < 4) cout << ", ";
    }
    cout << "]" << endl;
    cout << "  右腿: [";
    for (int i = 5; i < 10; i++) {
        cout << fixed << setprecision(6) << q_encoder_stand[i];
        if (i < 9) cout << ", ";
    }
    cout << "]" << endl;

    // ========== 计算 offset ==========
    float offset[10];
    for (int i = 0; i < 10; i++) {
        offset[i] = q_encoder_stand[i] - default_angles[i];
    }

    cout << "\n计算 offset = q_encoder_stand - default_angles:" << endl;
    cout << "  左腿: [";
    for (int i = 0; i < 5; i++) {
        cout << fixed << setprecision(6) << offset[i];
        if (i < 4) cout << ", ";
    }
    cout << "]" << endl;
    cout << "  右腿: [";
    for (int i = 5; i < 10; i++) {
        cout << fixed << setprecision(6) << offset[i];
        if (i < 9) cout << ", ";
    }
    cout << "]" << endl;

    // ========== 输入 sign_array ==========
    cout << "\n【配置关节方向映射 (sign_array)】" << endl;
    int sign_array[10];
    if (!read_int_array("请输入 sign_array:", sign_array, 1)) {
        close(sock);
        return 1;
    }

    // ========== 保存配置文件 ==========
    string output_file = "robot.yaml";
    cout << "\n正在保存配置到: " << output_file << endl;

    if (save_yaml(default_angles, offset, sign_array, output_file)) {
        cout << "✓ 标定完成！配置已保存到: " << output_file << endl;
    } else {
        cerr << "✗ 保存配置文件失败" << endl;
        close(sock);
        return 1;
    }

    close(sock);
    return 0;
}
