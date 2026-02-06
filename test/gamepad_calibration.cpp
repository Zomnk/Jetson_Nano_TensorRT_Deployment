/**
 * @file gamepad_calibration.cpp
 * @brief 交互式手柄键码获取工具
 * @details 用于获取当前手柄的轴值和按钮键码映射
 *
 * 编译: g++ -o gamepad_calibration gamepad_calibration.cpp
 * 运行: ./gamepad_calibration
 */

#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <cstring>
#include <linux/input.h>
#include <iomanip>
#include <map>

int main() {
    const char* device = "/dev/input/event2";
    int fd = open(device, O_RDONLY);

    if (fd < 0) {
        std::cerr << "无法打开设备: " << device << std::endl;
        std::cerr << "请确保手柄已连接，并且有读取权限" << std::endl;
        return 1;
    }

    std::cout << "========================================" << std::endl;
    std::cout << "  手柄键码获取工具" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "\n设备已打开: " << device << std::endl;
    std::cout << "\n请按照以下顺序操作手柄，记录对应的键码值：" << std::endl;
    std::cout << "1. 左摇杆 - 向左/右/上/下推动" << std::endl;
    std::cout << "2. 右摇杆 - 向左/右/上/下推动" << std::endl;
    std::cout << "3. 按钮 - 按下各个按钮" << std::endl;
    std::cout << "4. 肩键 - 按下LT/RT" << std::endl;
    std::cout << "\n按 Ctrl+C 退出\n" << std::endl;

    struct input_event event;
    std::map<int, int> axis_values;
    std::map<int, int> button_values;

    std::cout << std::string(80, '-') << std::endl;
    std::cout << "实时数据显示：" << std::endl;
    std::cout << std::string(80, '-') << std::endl;

    int event_count = 0;

    while (true) {
        ssize_t bytes = read(fd, &event, sizeof(event));

        if (bytes == sizeof(event)) {
            event_count++;

            // 处理轴事件 (EV_ABS)
            if (event.type == EV_ABS) {
                axis_values[event.code] = event.value;

                std::cout << "[轴事件 #" << event_count << "] 轴码: " << std::setw(3) << event.code
                          << " (0x" << std::hex << std::setw(2) << std::setfill('0') << event.code << std::dec << std::setfill(' ') << ")"
                          << " | 值: " << std::setw(6) << event.value
                          << " | 归一化: " << std::fixed << std::setprecision(3)
                          << (event.value / 32768.0f) << std::endl;
                std::cout.flush();
            }
            // 处理按钮事件 (EV_KEY)
            else if (event.type == EV_KEY) {
                button_values[event.code] = event.value;

                std::cout << "[按钮事件 #" << event_count << "] 按钮码: " << std::setw(3) << event.code
                          << " (0x" << std::hex << std::setw(2) << std::setfill('0') << event.code << std::dec << std::setfill(' ') << ")"
                          << " | 状态: " << (event.value ? "按下" : "释放") << std::endl;
                std::cout.flush();
            }
            // 处理同步事件 (EV_SYN)
            else if (event.type == EV_SYN) {
                // 忽略同步事件
            }
            else {
                std::cout << "[其他事件 #" << event_count << "] 类型: " << event.type
                          << " | 码: " << event.code
                          << " | 值: " << event.value << std::endl;
            }
        } else if (bytes < 0) {
            std::cerr << "读取错误" << std::endl;
            break;
        }
    }

    close(fd);
    return 0;
}
