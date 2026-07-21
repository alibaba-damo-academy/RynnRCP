// SPDX-License-Identifier: GPL-3.0
// Copyright (C) 2026 Luo1imasi

/**
 * @file serial_port.cpp
 * @brief Serial port implementation for IMU communication.
 * @details Implements UART serial port opening, configuration, and
 *          asynchronous reading with callback-based data delivery.
 */

#include "imu/serial_port.hpp"

std::shared_ptr<spdlog::logger> IMUSerialPort::logger_ = nullptr;

IMUSerialPort::IMUSerialPort(const std::string& interface, int baudrate) 
    : interface_(interface), baudrate_(baudrate), fd_(-1), running_(false) {
    init();
}

void IMUSerialPort::init() {
    fd_ = ::open(interface_.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd_ < 0) {
        if (logger_) logger_->error("Failed to open serial port: {}", interface_);
        throw std::runtime_error("Failed to open serial port: " + interface_);
    }

    struct termios tty;
    if (tcgetattr(fd_, &tty) != 0) {
        if (logger_) logger_->error("Failed to get serial attributes: {}", interface_);
        ::close(fd_);
        throw std::runtime_error("Failed to get serial attributes: " + interface_);
    }

    bool configured = false;

    if (!configured) {
        speed_t speed;
        switch (baudrate_) {
            case 9600: speed = B9600; break;
            case 19200: speed = B19200; break;
            case 38400: speed = B38400; break;
            case 57600: speed = B57600; break;
            case 115200: speed = B115200; break;
            case 230400: speed = B230400; break;
            case 460800: speed = B460800; break;
            case 921600: speed = B921600; break;
            default: speed = B115200; break; 
        }

        cfsetispeed(&tty, speed);
        cfsetospeed(&tty, speed);

        tty.c_cflag &= ~CSIZE;
        tty.c_cflag |= CS8;
        tty.c_cflag &= ~PARENB;
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag |= (CLOCAL | CREAD);
        tty.c_cflag &= ~CRTSCTS;

        tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
        tty.c_iflag &= ~(IXON | IXOFF | IXANY);
        tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
        tty.c_oflag &= ~OPOST;

        tty.c_cc[VMIN] = 0;
        tty.c_cc[VTIME] = 1;

        if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
            if (logger_) logger_->error("Failed to set serial attributes: {}", interface_);
            ::close(fd_);
            throw std::runtime_error("Failed to set serial attributes: " + interface_);
        }
    }

    running_ = true;
    rx_thread_ = std::thread([this]() {
        pthread_setname_np(pthread_self(), "serial_rx");
        struct sched_param sp{}; sp.sched_priority = 80;
        if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp) != 0) {
            if (logger_) logger_->error("Failed to set realtime priority for IMU serial RX");
        } 
        uint8_t buf[BUF_SIZE] = {0};
        
        while (running_) {
            fd_set readfds;
            struct timeval tv;
            
            FD_ZERO(&readfds);
            FD_SET(fd_, &readfds);
            tv.tv_sec = 0;
            tv.tv_usec = 1000;  // 1ms timeout

            int ret = select(fd_ + 1, &readfds, NULL, NULL, &tv);
            if (ret < 0) {
                if (errno == EINTR) continue;
                if (logger_) logger_->error("select error: {}", strerror(errno));
                break;
            } else if (ret == 0) {
                continue;  // timeout
            }
            
            int n = read(fd_, buf, BUF_SIZE);
            if (n < 0) {
                if (errno == EAGAIN || errno == EWOULDBLOCK) continue;
                if (logger_) logger_->error("read error: {}", strerror(errno));
                break; 
            } else if (n > 0) {
                if (callback_) {
                    callback_(buf, n);
                }
            }
        }
    });
}

IMUSerialPort::~IMUSerialPort() {
    close();
}

std::shared_ptr<IMUSerialPort> IMUSerialPort::open(const std::string& interface, int baudrate) {
    if (!logger_) {
        logger_ = spdlog::get("imu");
        if (!logger_) {
            std::vector<spdlog::sink_ptr> sinks;
            sinks.push_back(std::make_shared<spdlog::sinks::stderr_color_sink_st>());
            logger_ = std::make_shared<spdlog::logger>("imu", std::begin(sinks), std::end(sinks));
            spdlog::register_logger(logger_);
        }
    }
    return std::shared_ptr<IMUSerialPort>(new IMUSerialPort(interface, baudrate));
}

void IMUSerialPort::close() {
    running_ = false;
    if (rx_thread_.joinable()) {
        rx_thread_.join();
    }
    if (fd_ >= 0) {
        if (::close(fd_) < 0) {
            if (logger_) logger_->warn("Failed to close serial port {}: {}", interface_, strerror(errno));
        } else {
            if (logger_) logger_->info("Serial port {} closed successfully.", interface_);
        }
        fd_ = -1;
    }
}

void IMUSerialPort::set_serial_callback(SerialCbkFunc callback) {
    callback_ = callback;
}

