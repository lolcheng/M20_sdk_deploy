/**
 * @file udp_server.hpp
 * @brief udp server
 * @author DeepRobotics
 * @version 1.0
 * @date 2026-01-07
 *
 * @copyright Copyright (c) 2025  DeepRobotics
 *
 */

#pragma once

#include <arpa/inet.h>
#include <fcntl.h>
#include <unistd.h>

#include <atomic>
#include <cstring>
#include <functional>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "common_types.h"
#include "gamepad_interface.hpp"
#include "thread_pool.hpp"

#define UDP_PORT 30000              // UDP端口号
#define IP "0.0.0.0"                // 监听所有本地 IPv4 地址
#define UDP_BUFFER_SIZE 1024 * 32   // 单次接收的缓冲区大小为 32KB

class ThreadPool;
class GamepadInterface;

// 接收手柄 UDP 报文，将解析和回复交给线程池处理
class UdpServer {
public:
    UdpServer(GamepadInterface* gp) : mGamepadInterface_(gp) {
        mThreadPool_ = std::make_unique<ThreadPool>(10);    // 创建一个包含10个线程的线程池

        if (!Init()) {
            std::cerr << "[UdpServer] Init Failed!" << std::endl;
        }

        // 启动独立的接收线程
        mIsRunning_ = true;
        mServerThread_ = std::thread(&UdpServer::RecvLoop, this);
        std::cout << "UDP Server started, listening on port: " << UDP_PORT << std::endl;
    }

    ~UdpServer() {
        mIsRunning_ = false;

        // 等待接收线程结束
        if (mServerThread_.joinable()) {
            mServerThread_.join();
        }

        if (mFd >= 0) {
            std::cout << "Closing UDP socket..." << std::endl;
            close(mFd);
            mFd = -1;
        }
    }

private:
    // 初始化UDP服务器
    bool Init() {
        mFd = socket(AF_INET, SOCK_DGRAM, 0);   // 创建UDP套接字
        if (mFd < 0) {
            perror("Create socket failed");
            return false;
        }

        int bufSize = 1024 * 1024 * 25;         // 向系统申请 25MB 的套接字收发缓冲区
        setsockopt(mFd, SOL_SOCKET, SO_RCVBUF, &bufSize, sizeof(bufSize));  // 设置接收缓冲区大小
        setsockopt(mFd, SOL_SOCKET, SO_SNDBUF, &bufSize, sizeof(bufSize));  // 设置发送缓冲区大小

        // 设置 5 秒接收超时，便于无报文时检查退出标志
        struct timeval timeout;
        timeout.tv_sec = 5;
        timeout.tv_usec = 0;
        if (setsockopt(mFd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)) < 0) {
            perror("Set timeout failed");
            close(mFd);
            return false;
        }

        // 设置套接字选项，允许地址重用
        int opt = 1;
        setsockopt(mFd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

        // 绑定套接字到对应的IP地址和端口
        sockaddr_in serverAddr{};
        serverAddr.sin_family = AF_INET;
        serverAddr.sin_addr.s_addr = inet_addr(IP);
        serverAddr.sin_port = htons(UDP_PORT);

        // 如果绑定失败，打印错误信息并关闭套接字
        if (bind(mFd, (struct sockaddr*)&serverAddr, sizeof(serverAddr)) < 0) {
            perror("Bind failed");
            close(mFd);
            return false;
        }

        return true;
    }

    // 接收数据循环
    void RecvLoop() {
        // 设置客户端地址结构
        struct sockaddr_in client_addr;
        socklen_t client_addr_len = sizeof(client_addr);

        std::vector<char> buffer(UDP_BUFFER_SIZE);

        // 运行时循环接收数据
        while (mIsRunning_) {
            ssize_t recvLen =
                recvfrom(mFd, buffer.data(), UDP_BUFFER_SIZE, 0, (struct sockaddr*)&client_addr, &client_addr_len);

            if (recvLen > 0) {
                // 复制报文
                auto data = std::make_shared<std::vector<char>>(buffer.begin(), buffer.begin() + recvLen);

                // 校验报文头，声明的总长度与实际接收长度一致时才调用dataHandle解析
                auto headerLen = mGamepadInterface_->ParseHeader(data->data(), recvLen);
                if (headerLen > 0 && headerLen == recvLen) {
                    dataHandle(mFd, data, headerLen, client_addr);
                } else {
                    std::cout<<"Invalid header"<<std::endl;
                }
            }
        }
    }

    // 在线程池中解析报文，执行对应的手柄命令并按需回复
    void dataHandle(int socket, std::shared_ptr<std::vector<char>> data, int dataLen, const sockaddr_in& clientAddr) {
        mThreadPool_->enqueue([=]() {
            FromType from_type;
            bool ret = false;
            int dataId = -1;
            std::string res;    // 回复
            DataInfo dataInfo;

            // 提取data报文编号和来源类型，将报文体写入dataInfo并解析
            if (mGamepadInterface_->ParseHeader(data->data(), dataLen, dataId, data->data(), from_type)) {
                std::string ip = inet_ntoa(clientAddr.sin_addr);
                int port = ntohs(clientAddr.sin_port);

                dataInfo.ip = ip;
                dataInfo.port = port;
                dataInfo.fromType = from_type;
                dataInfo.buffer = data->data();
                dataInfo.len = dataLen;

                // 结合客户端信息解析报文体并生成回复内容
                ret = mGamepadInterface_->ParseData(dataInfo, res);
            } else {
                res = "Protocol header error,error code:0x2";
            }

            // 为非空回复添加协议头，再发回客户端
            if (res.length() > 0) {
                std::string Response;
                mGamepadInterface_->AddDataHeader(res, Response, dataId, dataInfo);
                sendData(socket, Response.c_str(), Response.length(), clientAddr);
            }
        });
    }

    // 通过 UDP 套接字向指定客户端发送信息
    void sendData(int socket, const char* data, int dataLen, const sockaddr_in& clientAddr) {
        if (dataLen > 0) {
            ssize_t sentBytes = 0;
            while (sentBytes < dataLen) {
                ssize_t sendNum = sendto(socket, data + sentBytes, dataLen - sentBytes, 0,
                                         (struct sockaddr*)&clientAddr, sizeof(clientAddr));
                if (sendNum < 0) {
                    std::cout << "Send data failure: " << strerror(errno) << std::endl;
                    return;
                }
                sentBytes += sendNum;
            }
        }
    }

private:
    int mFd = -1;
    std::atomic<bool> mIsRunning_{false};
    std::thread mServerThread_;

    std::unique_ptr<ThreadPool> mThreadPool_;
    std::unique_ptr<GamepadInterface> mGamepadInterface_;   // 控制器接口
};