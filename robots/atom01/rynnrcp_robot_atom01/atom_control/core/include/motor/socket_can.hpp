// SPDX-License-Identifier: GPL-3.0
// Copyright (C) 2025-2026 Luo1imasi
// Copyright (C) 2026 wentywenty

/**
 * @file
 * This file declares an interface to SocketCAN,
 * to facilitates frame transmission and reception.
 */

#pragma once

#include "can_iso.hpp"

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
#include <boost/lockfree/queue.hpp>
#include <condition_variable>
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
constexpr const int TX_QUEUE_SIZE = 4096;
constexpr const int MAX_RETRY_COUNT = 3;

using LFQueue = boost::lockfree::queue<can_frame, boost::lockfree::fixed_sized<true>>;

class MotorsSocketCAN : public MotorsCAN {
   private:
    std::string interface_;  // The network interface name
    int sockfd_ = -1;        // The file descriptor for the CAN socket
    std::atomic<bool> receiving_;
    LFQueue tx_queue_;
    std::mutex tx_mutex_;
    std::condition_variable tx_cv_;

    sockaddr_can addr_;      // The address of the CAN socket
    ifreq if_request_;       // The network interface request

    /// Receiving
    std::thread receiver_thread_;
    CanCbkMap can_callback_list_;
    std::mutex can_callback_mutex_;
    CanCbkKeyExtractor key_extractor_ = [](const can_frame &frame) -> CanCbkId {
        return static_cast<CanCbkId>(frame.can_id);
    };

    /// Transmitting
    std::thread sender_thread_;
    std::atomic<int> send_sleep_us_{0};

    MotorsSocketCAN(const std::string& interface);

    static std::shared_ptr<MotorsSocketCAN> createInstance(const std::string& interface) {
        return std::shared_ptr<MotorsSocketCAN>(new MotorsSocketCAN(interface));
    }
    static std::unordered_map<std::string, std::shared_ptr<MotorsSocketCAN>> instances_;

    void open(const std::string& interface);
    void close();

   public:
    MotorsSocketCAN(const MotorsSocketCAN &) = delete;
    MotorsSocketCAN &operator=(const MotorsSocketCAN &) = delete;
    ~MotorsSocketCAN() override;

    static std::shared_ptr<MotorsSocketCAN> get(const std::string& interface);

    void transmit(const can_frame& frame) override;
    void add_can_callback(const CanCbkFunc& callback, CanCbkId id) override;
    void remove_can_callback(CanCbkId id) override;
    void clear_can_callbacks() override;
    void set_can_key_extractor(CanCbkKeyExtractor extractor) override;

    void set_send_sleep(int us) { send_sleep_us_ = us; }

    /// SocketCAN-specific: direct socket fd access (for advanced use).
    int get_fd() const { return sockfd_; }
};
