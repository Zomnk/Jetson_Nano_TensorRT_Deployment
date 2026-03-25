/**
 * @file calibration_tool.cpp
 * @brief 双足机器人 Sim-to-Real 标定工具（重构版）
 * @author Claude Code
 * @date 2026-03-25
 *
 * @note 功能说明：
 *       1. 输入算法层基准姿态（default_angles）
 *       2. 手动摆放机器人到期望站立姿态
 *       3. 读取编码器值作为硬件零点偏置（offset）
 *       4. 配置关节方向映射（sign_array）
 *       5. 保存完整的 robot.yaml 配置文件
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

/*
 * ============================================================
 * 关节名称映射
 * ============================================================
 */
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

const char* YAML_KEYS[10] = {
    "yaw", "roll", "pitch", "knee", "ankle",
    "yaw", "roll", "pitch", "knee", "ankle"
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
 * @brief 保存完整的 robot.yaml 配置文件
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

    // 获取当前时间
    time_t now = time(nullptr);
    char time_str[100];
    strftime(time_str, sizeof(time_str), "%Y-%m-%d %H:%M:%S", localtime(&now));

    f << "# 双足机器人 Sim-to-Real 配置文件" << endl;
    f << "# 生成时间: " << time_str << endl;
    f << "# 单位: 弧度 (rad) 和方向系数" << endl;
    f << endl;
    f << "robot_config:" << endl;

    // ========== default_angles ==========
    f << "  # ========================================" << endl;
    f << "  # 算法层基准姿态 (default_angles)" << endl;
    f << "  # ========================================" << endl;
    f << "  # 说明：这是训练环境中使用的基准姿态，通常来自训练时的 default_joint_angles" << endl;
    f << "  # 注意：这个值应该与训练环境保持一致，不是实机标定时读到的编码器值" << endl;
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

    // ========== offset ==========
    f << "  # ========================================" << endl;
    f << "  # 硬件零点偏置 (offset)" << endl;
    f << "  # ========================================" << endl;
    f << "  # 说明：每台机器独立标定的编码器零点偏置" << endl;
    f << "  # 这是让机器人站到期望姿态时，实际读取到的编码器值" << endl;
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

    // ========== sign_array ==========
    f << "  # ========================================" << endl;
    f << "  # 关节方向映射 (sign_array)" << endl;
    f << "  # ========================================" << endl;
    f << "  # 说明：定义每个关节的旋转方向" << endl;
    f << "  # 1 = 与训练环境方向一致" << endl;
    f << "  # -1 = 与训练环境方向相反" << endl;
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

    // ========== 使用说明 ==========
    f << "# ========================================" << endl;
    f << "# 使用说明" << endl;
    f << "# ========================================" << endl;
    f << "# 1. default_angles: 从训练环境获取，保持与训练时一致" << endl;
    f << "# 2. offset: 通过 calibration_tool 标定获得" << endl;
    f << "# 3. sign_array: 根据实际硬件装配确定，需要逐关节测试验证" << endl;
    f << "#" << endl;
    f << "# 坐标变换关系：" << endl;
    f << "# - 观测链路 (Real → Sim):" << endl;
    f << "#   q_sim = (q_real - offset) * sign_array" << endl;
    f << "#   dq_sim = dq_real * sign_array" << endl;
    f << "#" << endl;
    f << "# - 控制链路 (Sim → Real):" << endl;
    f << "#   q_real = q_sim * sign_array + offset" << endl;

    f.close();
    return true;
}

/**
 * @brief 从用户输入读取浮点数数组
 */
bool read_float_array(const string& prompt, float arr[10], float default_val = 0.0f) {
    cout << prompt << endl;
    cout << "输入格式: 10个浮点数，用空格分隔" << endl;
    cout << "或直接按 Enter 使用默认值 [" << default_val << " × 10]: ";

    string line;
    getline(cin, line);

    if (line.empty()) {
        for (int i = 0; i < 10; i++) {
            arr[i] = default_val;
        }
        return true;
    }

    istringstream iss(line);
    for (int i = 0; i < 10; i++) {
        if (!(iss >> arr[i])) {
            cerr << "输入格式错误，需要10个数值" << endl;
            return false;
        }
    }

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
        for (int i = 0; i < 10; i++) {
            arr[i] = default_val;
        }
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

/**
 * @brief 主函数
 */
int main(int argc, char** argv) {
    // 解析命令行参数
    string target_ip = "192.168.5.159";
    int port = 10000;

    for (int i = 1; i < argc; i++) {
        string arg = argv[i];
        if (arg == "--ip" && i + 1 < argc) {
            target_ip = argv[++i];
        } else if (arg == "--port" && i + 1 < argc) {
            port = atoi(argv[++i]);
        }
    }

    // 注册信号处理
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    cout << "========================================" << endl;
    cout << "  双足机器人 Sim-to-Real 标定工具" << endl;
    cout << "  目标: " << target_ip << ":" << port << endl;
    cout << "========================================" << endl;
    cout << endl;

    // ========== 步骤1: 输入 default_angles ==========
    cout << "【步骤 1/3】输入算法层基准姿态 (default_angles)" << endl;
    cout << "说明：这是训练环境中使用的基准姿态，通常来自训练配置文件" << endl;
    cout << "      如果训练使用对称站立姿态，通常全部为 0" << endl;
    cout << endl;

    float default_angles[10];
    if (!read_float_array("请输入 default_angles:", default_angles, 0.0f)) {
        return 1;
    }

    cout << "\n已设置 default_angles:" << endl;
    cout << "  左腿: [";
    for (int i = 0; i < 5; i++) {
        cout << fixed << setprecision(3) << default_angles[i];
        if (i < 4) cout << ", ";
    }
    cout << "]" << endl;
    cout << "  右腿: [";
    for (int i = 5; i < 10; i++) {
        cout << fixed << setprecision(3) << default_angles[i];
        if (i < 9) cout << ", ";
    }
    cout << "]" << endl;
    cout << endl;

    // ========== 步骤2: 读取 offset ==========
    cout << "【步骤 2/3】标定硬件零点偏置 (offset)" << endl;
    cout << "说明：请手动调整机器人到期望的站立姿态" << endl;
    cout << "      然后按 Enter 读取当前编码器值作为 offset" << endl;
    cout << endl;
    cout << "按 Enter 继续...";
    cin.get();

    // 创建UDP socket
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

    float offset[10] = {0};
    bool got_feedback = false;

    // 尝试获取反馈
    for (int i = 0; i < 50 && !got_feedback; i++) {
        sendto(sock, (char*)&response, sizeof(response), 0,
               (struct sockaddr*)&remote_addr, sizeof(remote_addr));

        socklen_t addr_len = sizeof(remote_addr);
        if (recvfrom(sock, (char*)&request, sizeof(request), 0,
                     (struct sockaddr*)&remote_addr, &addr_len) > 0) {
            for (int j = 0; j < 10; j++) {
                offset[j] = request.q[j];
            }
            got_feedback = true;
            cout << "已连接 ODroid" << endl;
        }
        usleep(10000);
    }

    close(sock);

    if (!got_feedback) {
        cerr << "错误: 无法连接到 ODroid，请检查网络连接" << endl;
        return 1;
    }

    cout << "\n已读取编码器值 (q_encoder_stand):" << endl;
    cout << "  左腿: [";
    for (int i = 0; i < 5; i++) {
        cout << fixed << setprecision(3) << offset[i];
        if (i < 4) cout << ", ";
    }
    cout << "]" << endl;
    cout << "  右腿: [";
    for (int i = 5; i < 10; i++) {
        cout << fixed << setprecision(3) << offset[i];
        if (i < 9) cout << ", ";
    }
    cout << "]" << endl;

    // ========== 计算 offset = q_encoder_stand - default_angles ==========
    cout << "\n计算 offset = q_encoder_stand - default_angles:" << endl;
    for (int i = 0; i < 10; i++) {
        offset[i] = offset[i] - default_angles[i];
    }
    cout << "  左腿: [";
    for (int i = 0; i < 5; i++) {
        cout << fixed << setprecision(3) << offset[i];
        if (i < 4) cout << ", ";
    }
    cout << "]" << endl;
    cout << "  右腿: [";
    for (int i = 5; i < 10; i++) {
        cout << fixed << setprecision(3) << offset[i];
        if (i < 9) cout << ", ";
    }
    cout << "]" << endl;
    cout << endl;

    // ========== 步骤3: 输入 sign_array ==========
    cout << "【步骤 3/3】配置关节方向映射 (sign_array)" << endl;
    cout << "说明：定义每个关节的旋转方向" << endl;
    cout << "      1 = 与训练环境方向一致" << endl;
    cout << "      -1 = 与训练环境方向相反" << endl;
    cout << "      建议：先使用默认值 [1 × 10]，然后通过测试工具逐关节验证" << endl;
    cout << endl;

    int sign_array[10];
    if (!read_int_array("请输入 sign_array:", sign_array, 1)) {
        return 1;
    }

    cout << "\n已设置 sign_array:" << endl;
    cout << "  左腿: [";
    for (int i = 0; i < 5; i++) {
        cout << setw(2) << sign_array[i];
        if (i < 4) cout << ", ";
    }
    cout << "]" << endl;
    cout << "  右腿: [";
    for (int i = 5; i < 10; i++) {
        cout << setw(2) << sign_array[i];
        if (i < 9) cout << ", ";
    }
    cout << "]" << endl;
    cout << endl;

    // ========== 保存配置文件 ==========
    string output_file = "robot.yaml";
    cout << "正在保存配置到: " << output_file << endl;

    if (save_yaml(default_angles, offset, sign_array, output_file)) {
        cout << "✓ 标定完成！配置已保存到: " << output_file << endl;
        cout << endl;
        cout << "下一步建议：" << endl;
        cout << "  1. 使用 test_init_pose 测试机器人是否能正确回到站立姿态" << endl;
        cout << "  2. 使用 test_motors 逐关节测试方向是否正确" << endl;
        cout << "  3. 根据测试结果调整 sign_array" << endl;
    } else {
        cerr << "✗ 保存配置文件失败" << endl;
        return 1;
    }

    return 0;
}
