/**
 * @file calibration_tool.cpp
 * @brief 双足机器人初始姿态标定工具
 * @author Zomnk
 * @date 2026-02-04
 *
 * @note 功能说明：
 *       1. 交互式标定10个关节的初始站立位置
 *       2. 实时显示当前关节角度
 *       3. 保存标定结果到robot.yaml文件
 *       4. 支持单独标定某个关节或全部标定
 *
 * @note 使用方法：
 *       ./calibration_tool              # 标定所有关节
 *       ./calibration_tool --joint 0    # 只标定指定关节
 */

#define _USE_MATH_DEFINES
#include <cmath>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <cstring>
#include <sys/socket.h>
#include <sys/select.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <csignal>
#include <cstdlib>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using namespace std;

/*
 * ============================================================
 * 消息结构体定义（与ODroid保持一致）
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

/*
 * ============================================================
 * 关节名称映射
 * ============================================================
 */
const char* JOINT_NAMES[10] = {
    "左腿Yaw (L_YAW)",
    "左腿Roll (L_ROLL)",
    "左腿Pitch (L_PITCH)",
    "左腿Knee (L_KNEE)",
    "左腿Ankle (L_ANKLE)",
    "右腿Yaw (R_YAW)",
    "右腿Roll (R_ROLL)",
    "右腿Pitch (R_PITCH)",
    "右腿Knee (R_KNEE)",
    "右腿Ankle (R_ANKLE)"
};

/*
 * ============================================================
 * 全局变量
 * ============================================================
 */
volatile bool g_running = true;
bool terminal_modified = false;

/**
 * @brief 信号处理函数
 */
void signal_handler(int sig) {
    cout << "\n\n收到信号 " << sig << ", 退出标定..." << endl;
    g_running = false;

    // 恢复终端设置
    if (terminal_modified) {
        system("stty icanon echo");
        terminal_modified = false;
    }
}

/**
 * @brief 启用终端原始模式（非阻塞输入）
 */
void enable_raw_mode() {
    system("stty -icanon -echo");
    terminal_modified = true;
}

/**
 * @brief 恢复终端设置
 */
void disable_raw_mode() {
    if (terminal_modified) {
        system("stty icanon echo");
        terminal_modified = false;
    }
}

/**
 * @brief 保存标定结果到YAML文件
 * @param init_pos 10个关节的初始位置
 * @param filename YAML文件路径
 */
bool save_yaml(const float init_pos[10], const string& filename) {
    ofstream f(filename);
    if (!f.is_open()) return false;

    f << "# 双足机器人初始姿态标定数据" << endl;
    f << "# 生成时间: " << __DATE__ << " " << __TIME__ << endl;
    f << "# 单位: 弧度 (rad)" << endl;
    f << endl;
    f << "robot_config:" << endl;
    f << "  init_pose:" << endl;

    // 左腿
    f << "    left_leg:" << endl;
    f << "      yaw:   " << fixed << setprecision(6) << init_pos[0] << "  # rad" << endl;
    f << "      roll:  " << init_pos[1] << "  # rad" << endl;
    f << "      pitch: " << init_pos[2] << "  # rad" << endl;
    f << "      knee:  " << init_pos[3] << "  # rad" << endl;
    f << "      ankle: " << init_pos[4] << "  # rad" << endl;

    // 右腿
    f << "    right_leg:" << endl;
    f << "      yaw:   " << init_pos[5] << "  # rad" << endl;
    f << "      roll:  " << init_pos[6] << "  # rad" << endl;
    f << "      pitch: " << init_pos[7] << "  # rad" << endl;
    f << "      knee:  " << init_pos[8] << "  # rad" << endl;
    f << "      ankle: " << init_pos[9] << "  # rad" << endl;

    // 备注说明
    f << endl;
    f << "  # 说明:" << endl;
    f << "  # - 此文件由 calibration_tool 工具生成" << endl;
    f << "  # - 请勿手动修改，除非了解参数含义" << endl;
    f << "  # - 重新标定会覆盖此文件" << endl;

    f.close();
    return true;
}

/**
 * @brief 标定单个关节
 * @param joint_id 关节ID (0-9)
 * @param sock_fd UDP socket
 * @param addr ODroid地址
 * @param addr_len 地址长度
 * @param calibrated_positions 已标定的关节位置数组（用于保持其他关节位置）
 * @return 标定的关节位置
 */
float calibrate_joint(int joint_id, int sock_fd, struct sockaddr_in& addr, socklen_t addr_len,
                     float calibrated_positions[10]) {
    cout << "\n========================================" << endl;
    cout << "正在标定: " << JOINT_NAMES[joint_id] << " [ID=" << joint_id << "]" << endl;
    cout << "========================================" << endl;
    cout << "操作说明:" << endl;
    cout << "  1. 手动调整机器人到期望的初始站立姿态" << endl;
    cout << "  2. 观察下方显示的当前关节角度" << endl;
    cout << "  3. 确认姿态合适后，按 Enter 键保存" << endl;
    cout << "  4. 按 's' 跳过此关节（使用默认值0.0）" << endl;
    cout << "========================================" << endl;
    cout << "提示: 标定期间该关节扭矩已卸载，可手动调整" << endl;

    MsgRequest request;
    MsgResponse response;
    memset(&response, 0, sizeof(response));

    float current_angle = 0.0f;
    int update_count = 0;

    cout << "\n实时角度监测中... (按Enter确认, 按's'跳过)" << endl;
    cout << "-------------------------------------------" << endl;

    enable_raw_mode();

    while (g_running) {
        char buf[512];
        int n = recvfrom(sock_fd, buf, sizeof(buf), 0, (struct sockaddr*)&addr, &addr_len);

        if (n > 0) {
            memcpy(&request, buf, sizeof(request));
            current_angle = request.q[joint_id];

            // 标定协议：
            // - dq_exp[0] = -999.0 表示标定模式
            // - tau_exp[0] = joint_id 表示当前标定的关节ID
            // - 当前标定关节：q_exp = 当前实际位置（卸载扭矩）
            // - 已标定关节：q_exp = 标定位置（保持位置）
            // - 未标定关节：q_exp = 当前实际位置（跟随）
            for (int i = 0; i < 10; i++) {
                if (i == joint_id) {
                    // 当前标定关节：跟随实际位置（卸载扭矩）
                    response.q_exp[i] = request.q[i];
                } else if (calibrated_positions[i] != -9999.0f) {
                    // 已标定关节：保持标定位置
                    response.q_exp[i] = calibrated_positions[i];
                } else {
                    // 未标定关节：跟随实际位置
                    response.q_exp[i] = request.q[i];
                }
                response.dq_exp[i] = 0.0f;
                response.tau_exp[i] = 0.0f;
            }
            response.dq_exp[0] = -999.0f;  // 标定模式标志
            response.tau_exp[0] = static_cast<float>(joint_id);  // 当前标定的关节ID

            memcpy(buf, &response, sizeof(response));
            sendto(sock_fd, buf, sizeof(response), 0, (struct sockaddr*)&addr, addr_len);

            // 更新显示
            if (update_count % 10 == 0) {
                cout << "\r当前角度: " << fixed << setprecision(4) << setw(8)
                     << current_angle << " rad (" << setw(7) << setprecision(2)
                     << (current_angle * 180.0 / M_PI) << " deg)   " << flush;
            }
            update_count++;
        }

        // 检查键盘输入（使用select实现非阻塞）
        fd_set readfds;
        struct timeval tv;
        FD_ZERO(&readfds);
        FD_SET(STDIN_FILENO, &readfds);
        tv.tv_sec = 0;
        tv.tv_usec = 1000;  // 1ms超时

        if (select(STDIN_FILENO + 1, &readfds, NULL, NULL, &tv) > 0) {
            char ch = getchar();
            if (ch == '\n') {
                // Enter键 - 确认标定
                break;
            } else if (ch == 's' || ch == 'S') {
                // 跳过此关节
                cout << "\n跳过标定，使用默认值 0.0 rad" << endl;
                current_angle = 0.0f;
                break;
            }
        }

        usleep(2000);
    }

    disable_raw_mode();

    if (!g_running) {
        cout << "\n标定被中断" << endl;
        return current_angle;
    }

    cout << "\n✓ 标定完成: " << current_angle << " rad" << endl;
    return current_angle;
}

/**
 * @brief 主函数
 */
int main(int argc, char** argv) {
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    cout << "========================================" << endl;
    cout << "   双足机器人初始姿态标定工具" << endl;
    cout << "========================================" << endl;

    // 解析命令行参数
    string ip = "192.168.5.159";
    int port = 10000;
    string output = "../robot.yaml";
    int target_joint = -1;

    for (int i = 1; i < argc; i++) {
        string arg = argv[i];
        if (arg == "--joint" && i + 1 < argc) {
            target_joint = atoi(argv[++i]);
            if (target_joint < 0 || target_joint > 9) {
                cerr << "错误: 关节ID必须在0-9之间" << endl;
                return 1;
            }
        }
        else if (arg == "--output" && i + 1 < argc) output = argv[++i];
        else if (arg == "--ip" && i + 1 < argc) ip = argv[++i];
        else if (arg == "--help" || arg == "-h") {
            cout << "使用方法:" << endl;
            cout << "  " << argv[0] << " [选项]" << endl;
            cout << "\n选项:" << endl;
            cout << "  --joint <ID>      只标定指定关节 (0-9)" << endl;
            cout << "  --output <FILE>   指定输出文件 (默认: ../robot.yaml)" << endl;
            cout << "  --ip <IP>         ODroid IP地址" << endl;
            cout << "  --help, -h        显示此帮助信息" << endl;
            cout << "\n关节ID映射:" << endl;
            for (int j = 0; j < 10; j++) {
                cout << "  " << j << " - " << JOINT_NAMES[j] << endl;
            }
            return 0;
        }
    }

    cout << "通信配置: " << ip << ":" << port << endl;
    cout << "输出文件: " << output << endl;

    if (target_joint >= 0) {
        cout << "标定模式: 单关节 [" << JOINT_NAMES[target_joint] << "]" << endl;
    } else {
        cout << "标定模式: 全部关节 (10个)" << endl;
    }
    cout << "========================================" << endl;

    // 创建UDP socket
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

    // 绑定本地端口
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

    cout << "\n等待与ODroid建立连接..." << endl;

    // 等待首次反馈
    socklen_t addr_len = sizeof(remote_addr);
    bool connected = false;
    while (!connected && g_running) {
        MsgResponse dummy;
        memset(&dummy, 0, sizeof(dummy));
        sendto(sock_fd, &dummy, sizeof(dummy), 0, (struct sockaddr*)&remote_addr, addr_len);

        char buf[512];
        if (recvfrom(sock_fd, buf, sizeof(buf), 0, (struct sockaddr*)&remote_addr, &addr_len) > 0) {
            connected = true;
            cout << "✓ 已连接到ODroid，开始标定..." << endl;
        }
        usleep(100000);
    }

    if (!g_running) {
        close(sock_fd);
        return 0;
    }

    // 标定数组
    float init_pos[10] = {0};

    // 已标定位置数组（-9999.0表示未标定）
    float calibrated_positions[10];
    for (int i = 0; i < 10; i++) {
        calibrated_positions[i] = -9999.0f;  // 未标定标志
    }

    // 读取现有配置（如果存在）
    ifstream existing_yaml(output);
    if (existing_yaml.is_open()) {
        cout << "\n检测到现有配置文件，将作为默认值..." << endl;
        string line;
        int idx = 0;
        while (getline(existing_yaml, line) && idx < 10) {
            size_t pos = line.find(':');
            if (pos != string::npos) {
                string value_str = line.substr(pos + 1);
                size_t comment_pos = value_str.find('#');
                if (comment_pos != string::npos) {
                    value_str = value_str.substr(0, comment_pos);
                }
                try {
                    float value = stof(value_str);
                    init_pos[idx++] = value;
                } catch (...) {}
            }
        }
        existing_yaml.close();
    }

    // 执行标定
    if (target_joint >= 0 && target_joint < 10) {
        // 单关节标定
        init_pos[target_joint] = calibrate_joint(target_joint, sock_fd, remote_addr, addr_len, calibrated_positions);
        calibrated_positions[target_joint] = init_pos[target_joint];  // 更新已标定位置
    } else {
        // 全部关节标定
        for (int i = 0; i < 10 && g_running; i++) {
            init_pos[i] = calibrate_joint(i, sock_fd, remote_addr, addr_len, calibrated_positions);
            calibrated_positions[i] = init_pos[i];  // 标定完成后立即更新，下一个关节时保持位置

            if (i < 9 && g_running) {
                cout << "\n按Enter继续标定下一个关节..." << endl;
                cin.ignore();  // 清除输入缓冲
                getchar();
            }
        }
    }

    if (g_running) {
        // 保存结果
        cout << "\n========================================" << endl;
        cout << "标定结果汇总:" << endl;
        cout << "========================================" << endl;

        for (int i = 0; i < 10; i++) {
            cout << "[" << i << "] " << setw(25) << left << JOINT_NAMES[i]
                 << ": " << fixed << setprecision(4) << setw(8) << init_pos[i]
                 << " rad (" << setprecision(2) << (init_pos[i] * 180.0 / M_PI)
                 << " deg)" << endl;
        }

        cout << "========================================" << endl;
        cout << "\n保存到文件: " << output << " ... ";

        if (save_yaml(init_pos, output)) {
            cout << "✓ 成功!" << endl;
            cout << "\n标定完成！配置文件已生成。" << endl;
            cout << "现在可以使用 test_motors 或其他程序读取此配置。" << endl;
        } else {
            cout << "✗ 失败!" << endl;
        }
    }

    disable_raw_mode();
    close(sock_fd);
    return 0;
}
