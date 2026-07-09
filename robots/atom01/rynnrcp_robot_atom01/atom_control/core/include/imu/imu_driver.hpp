// SPDX-License-Identifier: GPL-3.0
// Copyright (C) 2026 Luo1imasi

/**
 * @file imu_driver.hpp
 * @brief Abstract IMU driver interface and sensor data structures.
 * @details Defines the IMUDriver base class and packed sensor data structs.
 *          Supports multiple IMU backends via factory pattern.
 */

#pragma once

#include <stdint.h>
#include <string.h>
#include <spdlog/spdlog.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <iostream>
#include <cmath>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

class IMUDriver {
   public:

    IMUDriver() {
        std::vector<spdlog::sink_ptr> sinks;
        sinks.push_back(std::make_shared<spdlog::sinks::stderr_color_sink_st>());
        logger_ = spdlog::get("imu");
        if (!logger_) {
            logger_ = std::make_shared<spdlog::logger>("imu", std::begin(sinks), std::end(sinks));
            spdlog::register_logger(logger_);
        }
    }
    virtual ~IMUDriver() = default;

    static std::shared_ptr<IMUDriver> create_imu(uint16_t imu_id, const std::string& interface_type, const std::string& interface,
                                                const std::string& imu_type, const int baudrate=0);

    virtual uint16_t get_imu_id() { return imu_id_; }
    virtual std::vector<float> get_ang_vel() { return ang_vel_; }
    virtual std::vector<float> get_quat() { return quat_; }
    virtual std::vector<float> get_lin_acc() { return lin_acc_; }
    virtual float get_temperature() { return temperature_; }

   protected:
    std::shared_ptr<spdlog::logger> logger_;
    uint16_t imu_id_;

    std::vector<float> quat_{0.f, 0.f, 0.f, 0.f};       // w, x, y, z
    std::vector<float> ang_vel_{0.f, 0.f, 0.f};         // x, y, z
    std::vector<float> lin_acc_{0.f, 0.f, 0.f};         // x, y, z
    float temperature_{0.f}; // temperature
};