// SPDX-License-Identifier: GPL-3.0
// Copyright (C) 2026 Luo1imasi

/**
 * @file socket_can.hpp
 * @brief SocketCAN interface declaration for CAN/CAN FD frame reception.
 * @details Provides the IMUSocketCAN singleton class that wraps Linux
 *          SocketCAN socket operations and delivers parsed frames
 *          to registered callback handlers.
 */

#pragma once

#include <linux/can.h>
#include <net/if.h>
#include <pthread.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>
#include <fcntl.h>

#include <atomic>
#include <cstdbool>
#include <cstdio>
#include <cstring>
#include <memory>
#include <string>
#include <thread>
#include <unordered_map>

constexpr const int INIT_FD = -1;
constexpr const int TIMEOUT_SEC = 0;
constexpr const int TIMEOUT_USEC = 1000;
using CanCbkFunc = std::function<void(const can_frame &)>;
using CanCbkId = uint16_t;
using CanCbkMap = std::unordered_map<CanCbkId, CanCbkFunc>;
using CanCbkKeyExtractor = std::function<CanCbkId(const can_frame &)>;

class IMUSocketCAN {
   private:
    std::string interface_;  // The network interface name
    int sockfd_ = -1;        // The file descriptor for the CAN socket
    std::atomic<bool> receiving_;

    sockaddr_can addr_;      // The address of the CAN socket
    ifreq if_request_;       // The network interface request

    /// Receiving
    std::thread receiver_thread_;
    CanCbkMap can_callback_list_;
    std::mutex can_callback_mutex_;
    CanCbkKeyExtractor key_extractor_ = [](const can_frame &frame) -> CanCbkId {
        return static_cast<CanCbkId>(frame.can_id);
    };

    IMUSocketCAN(std::string port_name);

    static std::shared_ptr<IMUSocketCAN> createInstance(const std::string &port_name) {
        return std::shared_ptr<IMUSocketCAN>(new IMUSocketCAN(port_name));
    }
    static std::shared_ptr<spdlog::logger> logger_;
    static std::unordered_map<std::string, std::shared_ptr<IMUSocketCAN>> instances_;

   public:
    IMUSocketCAN(const IMUSocketCAN &) = delete;
    IMUSocketCAN &operator=(const IMUSocketCAN &) = delete;
    ~IMUSocketCAN();
    static void init_logger(std::shared_ptr<spdlog::logger> logger) { logger_ = logger; }
    static std::shared_ptr<IMUSocketCAN> get_instance(std::string port_name) {
        if (!logger_) {
            logger_ = spdlog::get("imu");
            if (!logger_) {
                std::vector<spdlog::sink_ptr> sinks;
                sinks.push_back(std::make_shared<spdlog::sinks::stderr_color_sink_st>());
                logger_ = std::make_shared<spdlog::logger>("imu", std::begin(sinks), std::end(sinks));
                spdlog::register_logger(logger_);
            }
        }
        if (instances_.find(port_name) == instances_.end()) instances_[port_name] = createInstance(port_name);
        return instances_[port_name];
    }
    void open(std::string interface);
    void close();
    void add_can_callback(const CanCbkFunc callback, const CanCbkId id);
    void remove_can_callback(const CanCbkId id);
    void clear_can_callbacks();
    void set_key_extractor(CanCbkKeyExtractor extractor);
};
