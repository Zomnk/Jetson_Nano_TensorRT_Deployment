/**
 * @file communication.cpp
 * @brief UDP通信类实现文件
 * @author Zomnk
 * @date 2026-02-04
 *
 * @details 实现了UDPCommunication类的所有成员函数。
 *          负责Jetson与ODroid之间的UDP数据收发。
 */

#include "communication.h"
#include <iostream>
#include <cstring>
#include <unistd.h>
#include <sys/socket.h>
#include <arpa/inet.h>

/**
 * @brief 构造函数
 *
 * @details 初始化成员变量，将socket文件描述符设为-1（无效状态）。
 *          清零目标地址结构体。
 */
UDPCommunication::UDPCommunication(const std::string& target_ip, int port)
    : target_ip_(target_ip)          // 保存目标IP地址
    , port_(port)                    // 保存端口号
    , sock_fd_(-1)                   // socket初始化为无效
    , addr_len_(sizeof(target_addr_))// 地址结构体长度
    , initialized_(false) {          // 初始化状态为false
    // 清零目标地址结构体，避免未初始化的内存问题
    std::memset(&target_addr_, 0, sizeof(target_addr_));
}

/**
 * @brief 析构函数
 *
 * @details 自动调用close()关闭socket，释放系统资源。
 */
UDPCommunication::~UDPCommunication() {
    close();
}

/**
 * @brief 初始化UDP通信
 *
 * @details 完整的初始化流程：
 *          1. 创建UDP socket (SOCK_DGRAM)
 *          2. 设置接收超时为100ms，避免阻塞
 *          3. 绑定本地端口，使能接收数据
 *          4. 配置目标地址（ODroid）
 *
 * @return 初始化成功返回true，任何步骤失败返回false
 */
bool UDPCommunication::init() {
    // ========== 步骤1: 创建UDP socket ==========
    sock_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock_fd_ < 0) {
        std::cerr << "[UDP] Failed to create socket" << std::endl;
        return false;
    }

    // ========== 步骤2: 设置接收超时 ==========
    // 超时时间100ms，避免receiveRequest()永久阻塞
    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 100000;  // 100ms = 100000微秒
    setsockopt(sock_fd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    // ========== 步骤3: 绑定本地端口 ==========
    // 必须绑定端口才能接收数据，这是UDP通信的关键步骤
    struct sockaddr_in local_addr;
    std::memset(&local_addr, 0, sizeof(local_addr));
    local_addr.sin_family = AF_INET;         // IPv4
    local_addr.sin_addr.s_addr = INADDR_ANY; // 监听所有网卡
    local_addr.sin_port = htons(port_);      // 端口号（网络字节序）

    if (bind(sock_fd_, (struct sockaddr*)&local_addr, sizeof(local_addr)) < 0) {
        std::cerr << "[UDP] Failed to bind port " << port_ << std::endl;
        ::close(sock_fd_);  // 使用全局close()关闭socket
        sock_fd_ = -1;
        return false;
    }

    // ========== 步骤4: 配置目标地址 ==========
    target_addr_.sin_family = AF_INET;
    target_addr_.sin_port = htons(port_);
    target_addr_.sin_addr.s_addr = inet_addr(target_ip_.c_str());

    // 打印初始化信息
    std::cout << "[UDP] Initialized - Target: " << target_ip_ << ":" << port_ << std::endl;
    std::cout << "[UDP] Local port bound: " << port_ << std::endl;
    std::cout << "[UDP] Request size: " << sizeof(MsgRequest) << " bytes" << std::endl;
    std::cout << "[UDP] Response size: " << sizeof(MsgResponse) << " bytes" << std::endl;

    initialized_ = true;
    return true;
}

/**
 * @brief 发送响应消息给ODroid
 *
 * @details 将MsgResponse结构体通过UDP发送给ODroid。
 *          使用MSG_WAITALL标志确保数据完整发送。
 *
 * @param response 要发送的响应消息（包含关节目标位置）
 * @return 发送成功返回true，失败返回false
 */
bool UDPCommunication::sendResponse(const MsgResponse& response) {
    // 检查是否已初始化
    if (!initialized_) return false;

    // 将结构体拷贝到发送缓冲区
    char buffer[sizeof(MsgResponse)];
    std::memcpy(buffer, &response, sizeof(MsgResponse));

    // 发送数据到目标地址
    ssize_t sent = sendto(sock_fd_, buffer, sizeof(MsgResponse), MSG_WAITALL,
                          (struct sockaddr*)&target_addr_, addr_len_);
    return sent > 0;
}

/**
 * @brief 接收ODroid发送的请求消息
 *
 * @details 从UDP socket接收MsgRequest结构体。
 *          如果100ms内没有收到数据，函数会超时返回false。
 *          同时更新target_addr_为实际发送方的地址。
 *
 * @param request 用于存储接收到的请求消息
 * @return 接收成功返回true，超时或失败返回false
 */
bool UDPCommunication::receiveRequest(MsgRequest& request) {
    // 检查是否已初始化
    if (!initialized_) return false;

    // 接收缓冲区
    char buffer[sizeof(MsgRequest)];

    // 接收数据，同时获取发送方地址
    // MSG_WAITALL: 等待完整数据包
    ssize_t received = recvfrom(sock_fd_, buffer, sizeof(buffer), MSG_WAITALL,
                                (struct sockaddr*)&target_addr_, &addr_len_);

    if (received > 0) {
        // 将接收到的数据拷贝到输出结构体
        std::memcpy(&request, buffer, sizeof(MsgRequest));
        return true;
    }
    return false;
}

/**
 * @brief 关闭UDP连接
 *
 * @details 关闭socket文件描述符，释放系统资源。
 *          可以安全地多次调用。
 */
void UDPCommunication::close() {
    if (sock_fd_ >= 0) {
        ::close(sock_fd_);  // 使用全局close()避免与成员函数冲突
        sock_fd_ = -1;
    }
    initialized_ = false;
}
