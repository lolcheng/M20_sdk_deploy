/**
 * @file heartbeat.hpp
 * @brief heartbeat
 * @author DeepRobotics
 * @version 1.0
 * @date 2026-01-07
 *
 * @copyright Copyright (c) 2025  DeepRobotics
 *
 */
#pragma once
#include <chrono>
#include <list>
#include <mutex>
#include <thread>

class Heartbeat {
public:
    Heartbeat() {
        mIsRunning_ = true;     // 心跳报文运行标志位
        heartbeatCheckThread_ = std::make_unique<std::thread>(&Heartbeat::heartbeatCheckThread, this);
    }
    ~Heartbeat() {
        mIsRunning_ = false;
        heartbeatCheckThread_.get()->join();
    }

    // TODO：唤醒客户端
    void wakeUp(const NetInfo& netInfo) {
        std::lock_guard<std::mutex> lock(mutex_);
        for (auto& clientInfo : clientInfoList_) {
            // 如果客户端存在于列表中，更新状态为活跃
            if (clientInfo.ip == netInfo.ip && clientInfo.port == netInfo.port) {
                clientInfo.isLive = true;
                return;
            }
        }

        // 如果客户端不存在，添加到列表中
        ClientInfo clientInfo;
        clientInfo.ip = netInfo.ip;
        clientInfo.port = netInfo.port;
        clientInfo.fromType = netInfo.fromType;
        clientInfo.isLive = true;
        clientInfoList_.push_back(clientInfo);
    }

    // 获取当前活跃客户端列表
    void GetLiveList(std::list<NetInfo>& netInfos) {
        netInfos.clear();
        std::lock_guard<std::mutex> lock(mutex_);
        for (auto& clientInfo : clientInfoList_) {
            NetInfo netInfo;
            netInfo.ip = clientInfo.ip;
            netInfo.port = clientInfo.port;
            netInfo.fromType = clientInfo.fromType;
            netInfos.push_back(netInfo);
        }
    }

    // 删除 netInfo 对应的客户端
    void DeleteClient(const NetInfo& netInfo) {
        std::lock_guard<std::mutex> lock(mutex_);
        for (auto it = clientInfoList_.begin(); it != clientInfoList_.end();) {
            if (it->ip == netInfo.ip && it->port == netInfo.port) {
                it = clientInfoList_.erase(it);
            } else {
                it++;
            }
        }
    }

private:
    // 心跳检测线程
    void heartbeatCheckThread() {
        bool flag = false;
        // 检测心跳报文是否超时和清理不活跃的客户端
        while (mIsRunning_) {
            std::this_thread::sleep_for(std::chrono::seconds(2));
            auto now = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - lastHeartbeatTime_).count();
            if (elapsed >= 2) {
                if (!flag) std::cout<<"Heartbeat timeout!"<<std::endl;
                isConntected_ = false;
                flag = true;

                clientInfoList_.clear();
            } else {
                isConntected_ = true;
                flag = false;
            }

            if (clientInfoList_.size() > 0) {
                std::lock_guard<std::mutex> lock(mutex_);
                for (auto it = clientInfoList_.begin(); it != clientInfoList_.end();) {
                    if (it->isLive == false) {
                        it = clientInfoList_.erase(it);
                    } else {
                        it->isLive = false;
                        it++;
                    }
                }
            }
        }
    }

public:
    bool mIsRunning_;
    std::chrono::time_point<std::chrono::steady_clock> lastHeartbeatTime_;
    std::unique_ptr<std::thread> heartbeatCheckThread_;
    bool isConntected_ = false;

    // 
    struct ClientInfo : public NetInfo {
        bool isLive;
    };

    std::mutex mutex_;
    std::list<ClientInfo> clientInfoList_;  // 存储客户端信息的列表
};
