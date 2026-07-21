// SPDX-License-Identifier: GPL-3.0
// Copyright (C) 2026 Luo1imasi

#pragma once
// #include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/spdlog.h>
#include <stdint.h>
#include <string.h>
#include <iostream>
#include <cmath>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

enum class CommType {
    CAN = 0,         // Standard CAN 2.0 (8-byte payload)
    CANFD = 1,       // Physical CAN-FD (64-byte payload slot)
    ETHERCAT = 2     // EtherCAT communication
};

class MotorDriver {
   public:
    enum MotorControlMode_e {
        NONE = 0,
        MIT = 1,
        POS = 2,
        SPD = 3,
    };

    static constexpr double judgment_accuracy_threshold = 1e-2;
    static const int normal_sleep_time = 5;
    static const int setup_sleep_time = 500;

    MotorDriver();
    virtual ~MotorDriver() = default;

    static std::shared_ptr<MotorDriver> create_motor(uint16_t motor_id, const std::string& interface_type, const std::string& interface,
                                                    const std::string& motor_type, const int motor_model, uint16_t master_id_offset=0, const double motor_zero_offset=0.0);
    
    /**
     * @brief Locks the motor to prevent movement.
     *
     * This function locks the motor to prevent any movement.
     * Once locked, the motor will not respond to commands for movement.
     */
    virtual void lock_motor() = 0;

    /**
     * @brief Unlocks the motor to allow movement.
     *
     * This function unlocks the motor to enable movement.
     * After unlocking, the motor can respond to movement commands.
     */
    virtual void unlock_motor() = 0;

    /**
     * @brief Initializes the motor.
     *
     * This function initializes the motor for operation.
     * It performs necessary setup and configuration for motor control.
     *
     * @return True if motor initialization is successful; otherwise, false.
     */
    virtual uint8_t init_motor() = 0;

    /**
     * @brief Deinitializes the motor.
     *
     * This function deinitializes the motor.
     * It performs cleanup and releases resources associated with motor control.
     */
    virtual void deinit_motor() = 0;

    /**
     * @brief Sets the motor position to zero.
     *
     * This function sets the current motor position to zero.
     * It establishes a new reference point for position measurement.
     *
     * @return True if setting motor position to zero is successful; otherwise, false.
     */
    virtual bool set_motor_zero() = 0;

    /**
     * @brief Permanently burns current in-memory motor configuration parameters to Flash.
     *
     * This function triggers the motor's internal persistent storage logic, ensuring 
     * that parameters modified via write_register-style commands (such as ID, 
     * limits, gains, etc.) are retained after a power cycle or reboot.
     * * @note For certain motors (e.g., EVO series), this operation may be bundled 
     * with the set_motor_zero command.
     *
     * @return true if the burn command was successfully transmitted.
     * @return false if the burn command failed to transmit.
     */
    virtual bool write_motor_flash() = 0;

    /**
     * @brief Requests motor parameters based on a specific command.
     *
     * This function sends a request to retrieve specific parameters from the motor.
     * The parameter to be retrieved is identified by the `param_cmd` argument.
     *
     * @param param_cmd The command code specifying which parameter to retrieve.
     */
    virtual void get_motor_param(uint8_t param_cmd) = 0;
    // to enum and union

    /**
     * @brief Commands the motor to move to a specified position at a specified speed.
     *
     * This function is responsible for commanding the motor to move to a desired position
     * with a specified speed.
     *
     * @param pos The target position to move the motor to.
     * @param spd The speed at which the motor should move to the target position.
     * @param ignore_limit If true, ignores any position limits that may be set.
     */
    virtual void motor_pos_cmd(float pos, float spd, bool ignore_limit = false) = 0;

    /**
     * @brief Commands the motor to rotate at a specified speed.
     *
     * This function commands the motor to rotate at the specified speed.
     *
     * @param spd The speed at which the motor should rotate.
     */
    virtual void motor_spd_cmd(float spd) = 0;

    /**
     * @brief Commands the motor to operate in impedance mode with specific parameters.
     *
     * This function sets the motor to operate in an impedance control mode, where
     * it applies force based on the provided parameters.
     *
     * @param f_p Proportional force value.
     * @param f_v Velocity-based force value.
     * @param f_kp Proportional stiffness coefficient.
     * @param f_kd Damping coefficient.
     * @param f_t Desired torque value.
     */
    virtual void motor_mit_cmd(float f_p, float f_v, float f_kp, float f_kd, float f_t) = 0;

    /**
     * @brief Commands the motor to operate in impedance (MIT) mode using pointers/arrays.
     *
     * This overload is designed for efficient integration with multi-motor batch 
     * processing or inference engines. It allows passing arrays/pointers of control 
     * parameters. The specific hardware implementation handles how to extract and 
     * pack these contiguous memory blocks into bus-specific payloads (e.g. One-to-Many).
     *
     * @param f_p  Pointer to the proportional target position (rad).
     * @param f_v  Pointer to the target velocity (rad/s).
     * @param f_kp Pointer to the proportional stiffness coefficient (Position gain).
     * @param f_kd Pointer to the damping coefficient (Velocity gain).
     * @param f_t  Pointer to the desired feed-forward torque value (Nm).
     */
    virtual void motor_mit_cmd(float* f_p, float* f_v, float* f_kp, float* f_kd, float* f_t) = 0;

    /**
     * @brief Sets the control mode for the motor.
     *
     * This function specifies the control mode for the motor.
     *
     * @param motor_control_mode The control mode to be set for the motor.
     */
    virtual void set_motor_control_mode(uint8_t motor_control_mode) = 0;

    /**
     * @brief Retrieves the count of responses received from the motor.
     *
     * This function returns the number of responses received from the motor.
     *
     * @return The count of responses received from the motor.
     */
    virtual int get_response_count() const = 0;

    /**
     * @brief Requests a real-time status update from the physical motor.
     *
     * This function sends a query command to the motor to refresh its internal 
     * telemetry data, such as current position, velocity, and torque/current. 
     * The retrieved data is typically used to update the driver's internal 
     * state variables for subsequent getter calls.
     * * @note In a high-speed control loop, this is often called before 
     * retrieving feedback to ensure the controller is acting on the most 
     * recent hardware state.
     */
    virtual void refresh_motor_status() = 0;

    /**
     * @brief Clears active error flags and resets the motor's protection state.
     *
     * This function sends a command to the motor controller to acknowledge and 
     * clear internal faults (e.g., over-current, over-voltage, or stall errors). 
     * If the underlying physical condition causing the error has been resolved, 
     * the motor will transition from a fault/protection state back to a 
     * standby or ready state.
     * * @note This does not fix the root cause of hardware issues; it only attempts 
     * to reset the software error state of the motor's MCU.
     */
    virtual void clear_motor_error() = 0;

    /**
     * @brief Retrieves the unique identifier (ID) of the motor.
     *
     * This function returns the current motor ID stored in the driver instance. 
     * This ID is used as the primary addressing element on the CAN/CAN-FD bus.
     *
     * @return uint8_t The unique ID of the motor.
     */
    virtual uint8_t get_motor_id() { return motor_id_; }

    /**
     * @brief Configures a new ID for the physical motor hardware.
     *
     * This function triggers the hardware-specific protocol command to change the 
     * motor's internal ID. This is typically used during initial commissioning or 
     * when reconfiguring the robot topology.
     * * @note Depending on the implementation (e.g., EVO or LRO), this may require 
     * a subsequent call to write_motor_flash() to persist across power cycles.
     */
    virtual void set_motor_id(uint8_t old_id, uint8_t new_id) = 0;

    /**
     * @brief Resets the motor's hardware ID to the factory default value.
     *
     * Sends a protocol command to revert the motor's ID to its default state. 
     * This is useful for recovering motors with unknown IDs or clearing 
     * configuration errors during maintenance.
     */
    virtual void reset_motor_id() = 0;

    /**
     * @brief Retrieves the name of the CAN/CAN-FD bus interface associated with the motor.
     *
     * This function returns a string (e.g., "can0", "can9") indicating the specific 
     * physical bus hardware where the motor instance is connected. It is primarily 
     * used for debugging, logging, and fault isolation in multi-bus systems.
     *
     * @return std::string The name of the bus interface.
     */
    virtual std::string get_can_name() { return can_interface_; }

    /**
     * @brief Retrieves the control mode of the motor.
     *
     * This function returns the current control mode of the motor.
     *
     * @return The control mode of the motor.
     */
    virtual uint8_t get_motor_control_mode() { return motor_control_mode_; }

    /**
     * @brief Retrieves the error ID associated with the motor.
     *
     * This function returns the error ID that indicates any error condition of the motor.
     *
     * @return The error ID of the motor.
     */
    virtual uint8_t get_error_id() { return error_id_; }

    /**
     * @brief Retrieves the current position of the motor.
     *
     * This function returns the current position of the motor.
     *
     * @return The current position of the motor.
     */
    virtual float get_motor_pos() { return motor_pos_; }

    /**
     * @brief Retrieves the current speed of the motor.
     *
     * This function returns the current speed of the motor.
     *
     * @return The current speed of the motor.
     */
    virtual float get_motor_spd() { return motor_spd_; }

    /**
     * @brief Retrieves the current current (electric current) of the motor.
     *
     * This function returns the electric current flowing through the motor.
     *
     * @return The current (electric current) of the motor.
     */
    virtual float get_motor_current() { return motor_current_; }

    /**
     * @brief Retrieves the temperature of the motor.
     *
     * This function returns the current temperature of the motor.
     *
     * @return The temperature of the motor.
     */
    virtual float get_motor_temperature() { return motor_temperature_; }

   protected:
    std::shared_ptr<spdlog::logger> logger_;
    uint16_t motor_id_{0};

    uint8_t motor_control_mode_{0};  // 0:none 1:mit 2:pos 3:spd

    std::atomic<uint8_t> error_id_{0};

    double motor_zero_offset_{0.f};
    std::atomic<float> motor_pos_{0.f};
    std::atomic<float> motor_spd_{0.f};
    std::atomic<float> motor_current_{0.f};
    std::atomic<float> motor_temperature_{0.f};

    CommType comm_type_;
    std::string can_interface_;
};

using union32_t = union Union32 {
    float f;
    int32_t i;
    uint32_t u;
    uint8_t buf[4];
};
