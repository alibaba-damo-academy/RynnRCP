// SPDX-License-Identifier: GPL-3.0
// Copyright (C) 2025-2026 Luo1imasi

#include "motor/motor_driver.hpp"

#include "motor/dm_motor_driver.hpp"

MotorDriver::MotorDriver() {
    std::vector<spdlog::sink_ptr> sinks;
    sinks.push_back(std::make_shared<spdlog::sinks::stderr_color_sink_st>());
    logger_ = setup_logger(sinks, "motors");
}
std::shared_ptr<MotorDriver> MotorDriver::create_motor(uint16_t motor_id, const std::string& interface_type, const std::string& interface,
                                                      const std::string& motor_type, int motor_model, uint16_t master_id_offset, double motor_zero_offset) {
    if (motor_type == "DM") {
        return std::make_shared<DmMotorDriver>(motor_id, interface_type, interface, master_id_offset,
                                               static_cast<DM_Motor_Model>(motor_model), motor_zero_offset);
    } else {
        throw std::runtime_error("Motor type not supported: " + motor_type);
    }
}
