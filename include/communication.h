/**
 * @file communication.h
 * @brief UDP通信类头文件
 * @author Zomnk
 * @date 2026-02-04
 *
 * @details 本文件定义了UDPCommunication类，用于Jetson与ODroid之间的UDP通信。
 *          通信采用请求-响应模式：
 *          1. ODroid发送MsgRequest（机器人状态）给Jetson
 *          2. Jetson进行推理后发送MsgResponse（控制指令）给ODroid
 */

#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include "types.h"
#include <string>
#include <netinet/in.h>

/**
 * @brief UDP通信类
 *
 * @details 封装了UDP socket的创建、绑定、发送和接收操作。
 *          使用非阻塞接收模式，超时时间为100ms。
 */
class UDPCommunication {
public:
    /**
     * @brief 构造函数
     * @param target_ip ODroid的IP地址
     * @param port UDP通信端口（本地和远程使用相同端口）
     */
    UDPCommunication(const std::string& target_ip, int port);

    /**
     * @brief 析构函数，自动关闭socket
     */
    ~UDPCommunication();

    /**
     * @brief 初始化UDP通信
     * @return 成功返回true，失败返回false
     *
     * @details 执行以下操作：
     *          1. 创建UDP socket
     *          2. 设置接收超时（100ms）
     *          3. 绑定本地端口
     *          4. 配置目标地址
     */
    bool init();

    /**
     * @brief 发送响应消息给ODroid
     * @param response 要发送的响应消息
     * @return 发送成功返回true，失败返回false
     */
    bool sendResponse(const MsgResponse& response);

    /**
     * @brief 接收ODroid发送的请求消息
     * @param request 用于存储接收到的请求消息
     * @return 接收成功返回true，超时或失败返回false
     */
    bool receiveRequest(MsgRequest& request);

    /**
     * @brief 关闭UDP连接
     */
    void close();

private:
    std::string target_ip_;         ///< 目标IP地址（ODroid）
    int port_;                      ///< UDP端口号
    int sock_fd_;                   ///< socket文件描述符
    struct sockaddr_in target_addr_;///< 目标地址结构体
    socklen_t addr_len_;            ///< 地址结构体长度
    bool initialized_;              ///< 初始化状态标志
};

#endif // COMMUNICATION_H
