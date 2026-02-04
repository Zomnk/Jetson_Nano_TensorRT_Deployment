/**
 * @file calibration_tool.cpp
 * @brief 机器人关节标定工具 - 交互式标定并保存到robot.yaml
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
#include <csignal>
#include <termios.h>

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
    "左腿Yaw", "左腿Roll", "左腿Pitch", "左腿Knee", "左腿Ankle",
    "右腿Yaw", "右腿Roll", "右腿Pitch", "右腿Knee", "右腿Ankle"
};

volatile bool g_running = true;
struct termios orig_termios;
bool term_modified = false;

void signal_handler(int sig) {
    cout << "\n退出标定..." << endl;
    g_running = false;
    if (term_modified) {
        tcsetattr(STDIN_FILENO, TCSANOW, &orig_termios);
    }
}

void enable_raw_mode() {
    tcgetattr(STDIN_FILENO, &orig_termios);
    struct termios raw = orig_termios;
    raw.c_lflag &= ~(ICANON | ECHO);
    raw.c_cc[VMIN] = 0;
    raw.c_cc[VTIME] = 0;
    tcsetattr(STDIN_FILENO, TCSANOW, &raw);
    term_modified = true;
}

void disable_raw_mode() {
    if (term_modified) {
        tcsetattr(STDIN_FILENO, TCSANOW, &orig_termios);
        term_modified = false;
    }
}

bool save_yaml(const float init_pos[10], const string& filename) {
    ofstream f(filename);
    if (!f.is_open()) return false;

    f << "# 机器人标定数据\n";
    f << "robot_config:\n";
    f << "  init_pose:\n";
    f << "    left_leg:\n";
    f << "      yaw:   " << fixed << setprecision(6) << init_pos[0] << "\n";
    f << "      roll:  " << init_pos[1] << "\n";
    f << "      pitch: " << init_pos[2] << "\n";
    f << "      knee:  " << init_pos[3] << "\n";
    f << "      ankle: " << init_pos[4] << "\n";
    f << "    right_leg:\n";
    f << "      yaw:   " << init_pos[5] << "\n";
    f << "      roll:  " << init_pos[6] << "\n";
    f << "      pitch: " << init_pos[7] << "\n";
    f << "      knee:  " << init_pos[8] << "\n";
    f << "      ankle: " << init_pos[9] << "\n";
    f.close();
    return true;
}

float calibrate_joint(int joint_id, int sock_fd, struct sockaddr_in& addr, socklen_t addr_len) {
    cout << "\n========================================" << endl;
    cout << "标定: " << JOINT_NAMES[joint_id] << " [ID=" << joint_id << "]" << endl;
    cout << "操作: 手动调整关节位置，按Enter确认，按's'跳过" << endl;
    cout << "========================================" << endl;

    MsgRequest request;
    MsgResponse response;
    memset(&response, 0, sizeof(response));

    float current_angle = 0.0f;
    enable_raw_mode();

    while (g_running) {
        char buf[512];
        int n = recvfrom(sock_fd, buf, sizeof(buf), 0, (struct sockaddr*)&addr, &addr_len);

        if (n > 0) {
            memcpy(&request, buf, sizeof(request));
            current_angle = request.q[joint_id];

            for (int i = 0; i < 10; i++) {
                response.q_exp[i] = request.q[i];
                response.dq_exp[i] = 0.0f;
                response.tau_exp[i] = 0.0f;
            }

            memcpy(buf, &response, sizeof(response));
            sendto(sock_fd, buf, sizeof(response), 0, (struct sockaddr*)&addr, addr_len);

            cout << "\r当前: " << fixed << setprecision(4) << current_angle
                 << " rad (" << setprecision(1) << (current_angle * 180.0 / M_PI)
                 << " deg)   " << flush;
        }

        char ch;
        if (read(STDIN_FILENO, &ch, 1) > 0) {
            if (ch == '\n' || ch == '\r') break;
            if (ch == 's' || ch == 'S') {
                cout << "\n跳过，使用0.0" << endl;
                current_angle = 0.0f;
                break;
            }
        }

        usleep(2000);
    }

    disable_raw_mode();
    cout << "\n确认: " << current_angle << " rad" << endl;
    return current_angle;
}

int main(int argc, char** argv) {
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    string ip = "192.168.5.159";
    int port = 10000;
    string output = "../robot.yaml";
    int target_joint = -1;

    for (int i = 1; i < argc; i++) {
        string arg = argv[i];
        if (arg == "--joint" && i + 1 < argc) target_joint = atoi(argv[++i]);
        else if (arg == "--output" && i + 1 < argc) output = argv[++i];
        else if (arg == "--ip" && i + 1 < argc) ip = argv[++i];
        else if (arg == "--help" || arg == "-h") {
            cout << "用法: " << argv[0] << " [选项]\n";
            cout << "  --joint <ID>    只标定指定关节(0-9)\n";
            cout << "  --output <FILE> 输出文件(默认: ../robot.yaml)\n";
            cout << "  --ip <IP>       ODroid IP\n";
            return 0;
        }
    }

    cout << "========================================" << endl;
    cout << "  机器人关节标定工具" << endl;
    cout << "  输出: " << output << endl;
    cout << "========================================" << endl;

    int sock_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock_fd < 0) {
        cerr << "创建socket失败!" << endl;
        return 1;
    }

    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 100000;
    setsockopt(sock_fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

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

    memset(&remote_addr, 0, sizeof(remote_addr));
    remote_addr.sin_family = AF_INET;
    remote_addr.sin_addr.s_addr = inet_addr(ip.c_str());
    remote_addr.sin_port = htons(port);

    cout << "等待ODroid连接..." << endl;

    socklen_t addr_len = sizeof(remote_addr);
    bool connected = false;
    while (!connected && g_running) {
        MsgResponse dummy;
        memset(&dummy, 0, sizeof(dummy));
        sendto(sock_fd, &dummy, sizeof(dummy), 0, (struct sockaddr*)&remote_addr, addr_len);

        char buf[512];
        if (recvfrom(sock_fd, buf, sizeof(buf), 0, (struct sockaddr*)&remote_addr, &addr_len) > 0) {
            connected = true;
            cout << "已连接!" << endl;
        }
        usleep(100000);
    }

    if (!g_running) {
        close(sock_fd);
        return 0;
    }

    float init_pos[10] = {0};

    if (target_joint >= 0 && target_joint < 10) {
        init_pos[target_joint] = calibrate_joint(target_joint, sock_fd, remote_addr, addr_len);
    } else {
        for (int i = 0; i < 10 && g_running; i++) {
            init_pos[i] = calibrate_joint(i, sock_fd, remote_addr, addr_len);
        }
    }

    if (g_running) {
        cout << "\n========================================" << endl;
        cout << "标定结果:" << endl;
        for (int i = 0; i < 10; i++) {
            cout << "  [" << i << "] " << setw(12) << left << JOINT_NAMES[i]
                 << ": " << fixed << setprecision(4) << init_pos[i] << " rad" << endl;
        }

        if (save_yaml(init_pos, output)) {
            cout << "\n已保存到: " << output << endl;
        } else {
            cerr << "\n保存失败!" << endl;
        }
    }

    disable_raw_mode();
    close(sock_fd);
    return 0;
}
