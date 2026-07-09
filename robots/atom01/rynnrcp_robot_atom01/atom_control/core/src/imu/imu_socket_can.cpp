// SPDX-License-Identifier: GPL-3.0
// Copyright (C) 2026 Luo1imasi

/**
 * @file socket_can.cpp
 * @brief SocketCAN receiver implementation for CAN/CAN FD frames.
 * @details Contains socket initialization, epoll-based frame reception,
 *          and error frame filtering logic via Linux SocketCAN.
 */

#include "imu/imu_socket_can.hpp"

std::shared_ptr<spdlog::logger> IMUSocketCAN::logger_ = nullptr;
std::unordered_map<std::string, std::shared_ptr<IMUSocketCAN>> IMUSocketCAN::instances_;

IMUSocketCAN::IMUSocketCAN(std::string interface)
    : interface_(interface), sockfd_(INIT_FD), receiving_(false) {
    open(interface);
}

IMUSocketCAN::~IMUSocketCAN() { this->close(); }

void IMUSocketCAN::open(std::string interface) {
    sockfd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (sockfd_ == INIT_FD) {
        logger_->error("Failed to create CAN socket");
        throw std::runtime_error("Failed to create CAN socket");
    }

    strncpy(if_request_.ifr_name, interface.c_str(), IFNAMSIZ);
    if (ioctl(sockfd_, SIOCGIFINDEX, &if_request_) == -1) {
        logger_->error("Unable to detect CAN interface {}", interface);

        this->close();
        throw std::runtime_error("Unable to detect CAN interface " + interface);
    }

    // Bind the socket to the network interface
    addr_.can_family = AF_CAN;
    addr_.can_ifindex = if_request_.ifr_ifindex;
    int rc = ::bind(sockfd_, reinterpret_cast<struct sockaddr *>(&addr_), sizeof(addr_));
    if (rc == -1) {
        logger_->error("Failed to bind socket to network interface {}", interface);
        this->close();
        throw std::runtime_error("Failed to bind socket to network interface " + interface);
    }

    int flags = fcntl(sockfd_, F_GETFL, 0);
    if (flags == -1) {
        logger_->error("Failed to get socket flags");
        this->close();
        throw std::runtime_error("Failed to get socket flags");
    }
    if (fcntl(sockfd_, F_SETFL, flags | O_NONBLOCK) == -1) {
        logger_->error("Failed to set socket to non-blocking");
        this->close();
        throw std::runtime_error("Failed to set socket to non-blocking");
    }

    receiving_ = true;
    receiver_thread_ = std::thread([this]() {
        pthread_setname_np(pthread_self(), "can_rx");
        struct sched_param sp{}; sp.sched_priority = 80;
        if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp) != 0) {
            logger_->error("Failed to set realtime priority for IMU CAN RX thread");
        }
        fd_set descriptors;
        int maxfd = sockfd_;
        struct timeval timeout;
        can_frame rx_frame;

        while (receiving_) {
            FD_ZERO(&descriptors);
            FD_SET(sockfd_, &descriptors);

            timeout.tv_sec = TIMEOUT_SEC;
            timeout.tv_usec = TIMEOUT_USEC;

            int sel_ret = ::select(maxfd + 1, &descriptors, NULL, NULL, &timeout);
            if (sel_ret < 0) {
                if (errno == EINTR) continue;
                logger_->error("CAN select error: {}", strerror(errno));
                break;
            }
            if (sel_ret == 1) {
                while (true){
                    int len = ::read(sockfd_, &rx_frame, CAN_MTU);
                    if (len < 0) {
                        if (errno == EAGAIN || errno == EWOULDBLOCK) {
                            break; 
                        }
                        logger_->warn("CAN read error: {}", strerror(errno));
                        break;
                    }
                    if (len == 0){
                        break;
                    }
                    CanCbkFunc callback_to_run;
                    {
                        std::lock_guard<std::mutex> lock(can_callback_mutex_);
                        CanCbkId key = key_extractor_(rx_frame);
                        auto it = can_callback_list_.find(key);
                        if (it != can_callback_list_.end()) {
                            callback_to_run = it->second;
                        }
                    }
                    if (callback_to_run) {
                        callback_to_run(rx_frame);
                    }
                }
            }
        }
    });
}

void IMUSocketCAN::close() {
    receiving_ = false;
    if (receiver_thread_.joinable()) receiver_thread_.join();

    if (sockfd_ != INIT_FD) {
        if (::close(sockfd_) < 0) {
            logger_->warn("Failed to close socket {}: {}", interface_, strerror(errno));
        } else {
            logger_->info("CAN interface {} closed successfully.", interface_);
        }
    }
    sockfd_ = INIT_FD;
}

void IMUSocketCAN::add_can_callback(const CanCbkFunc callback, const CanCbkId id) {
    std::lock_guard<std::mutex> lock(can_callback_mutex_);
    can_callback_list_[id] = callback;
}

void IMUSocketCAN::remove_can_callback(CanCbkId id) {
    std::lock_guard<std::mutex> lock(can_callback_mutex_);
    can_callback_list_.erase(id);
}

void IMUSocketCAN::clear_can_callbacks() {
    std::lock_guard<std::mutex> lock(can_callback_mutex_);
    can_callback_list_.clear();
}

void IMUSocketCAN::set_key_extractor(CanCbkKeyExtractor extractor) {
    std::lock_guard<std::mutex> lock(can_callback_mutex_);
    key_extractor_ = std::move(extractor);
}
