// SPDX-License-Identifier: GPL-3.0
// Copyright (C) 2026 wentywenty

#pragma once

#include <linux/can.h>
#include <functional>
#include <memory>
#include <string>
#include <unordered_map>
#include <spdlog/spdlog.h>
#include <spdlog/sinks/stdout_color_sinks.h>

// Callback types shared by all CAN backends
using CanCbkFunc = std::function<void(const can_frame&)>;
using CanCbkId = uint16_t;
using CanCbkKeyExtractor = std::function<CanCbkId(const can_frame&)>;
using CanCbkMap = std::unordered_map<CanCbkId, CanCbkFunc>;

/**
 * @brief Abstract interface for classic CAN communication backends.
 *
 * Concrete implementations:
 *   - MotorsSocketCAN: Linux SocketCAN (direct CAN 2.0)
 *   - MotorsEthercatCAN: EtherCAT-to-CAN bridge
 *
 * All backends share the same transmit/callback API, allowing motor drivers
 * to switch transport transparently.
 */
class MotorsSocketCAN;

class MotorsCAN {
public:
    virtual ~MotorsCAN() = default;

    /// Send a CAN frame.
    virtual void transmit(const can_frame& frame) = 0;

    /// Register a receive callback keyed by CAN ID.
    virtual void add_can_callback(const CanCbkFunc& callback, CanCbkId id) = 0;

    /// Remove a previously registered callback.
    virtual void remove_can_callback(CanCbkId id) = 0;

    /// Remove all registered callbacks.
    virtual void clear_can_callbacks() = 0;

    /// Customize the key extractor (default: frame.can_id).
    virtual void set_can_key_extractor(CanCbkKeyExtractor extractor) = 0;

    /**
     * @brief Factory: create or retrieve a CAN backend instance.
     * @param interface  Interface name (e.g. "can0", or EtherCAT master name)
     * @param backend    Backend type: "socketcan" (default) | "ethercat"
     */
    static std::shared_ptr<MotorsCAN> get(
        const std::string& interface,
        const std::string& backend = "socketcan");

    static void init_logger(std::shared_ptr<spdlog::logger> logger);

protected:
    MotorsCAN() = default;
    inline static std::shared_ptr<spdlog::logger> logger_ = nullptr;

    inline static void ensure_logger() {
        if (!logger_) {
            logger_ = spdlog::get("motors");
            if (!logger_) {
                std::vector<spdlog::sink_ptr> sinks;
                sinks.push_back(std::make_shared<spdlog::sinks::stderr_color_sink_st>());
                logger_ = std::make_shared<spdlog::logger>("motors", std::begin(sinks), std::end(sinks));
                spdlog::register_logger(logger_);
            }
        }
    }
};

inline void MotorsCAN::init_logger(std::shared_ptr<spdlog::logger> logger) { logger_ = logger; }
