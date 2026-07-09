// SPDX-License-Identifier: GPL-3.0
// Copyright (C) 2025-2026 Luo1imasi
// Copyright (C) 2026 wentywenty

#pragma once

#include <atomic>
#include <string>

#include "motor/motor_driver.hpp"
#include "motor/can_iso.hpp"
// CAN-FD not needed for this robot
#include "motor/utils.hpp"

enum DMError {
    DM_DOWN = 0x00,
    DM_UP = 0x01,
    DM_OVER_VOLT = 0x08,
    DM_UNDER_VOLT = 0x09,
    DM_OVER_CURRENT = 0x0A,
    DM_MOS_OVER_TEMP = 0x0B,
    DM_COIL_OVER_TEMP = 0x0C,
    DM_LOST_CONN = 0x0D,
    DM_OVER_LOAD = 0x0E
};

enum DM_Motor_Model { 
    DM4340P_48V, 
    DM10010L_48V, 
    DM_Num_Of_Motor 
};

enum DM_REG {
    DM_UV_VALUE = 0,
    DM_KT_VALUE = 1,
    DM_OT_VALUE = 2,
    DM_OC_VALUE = 3,
    DM_ACC = 4,
    DM_DEC = 5,
    DM_MAX_SPD = 6,
    DM_MST_ID = 7,
    DM_ESC_ID = 8,
    DM_TIMEOUT = 9,
    DM_CTRL_MODE = 10,
    DM_DAMP = 11,
    DM_INERTIA = 12,
    DM_HW_VER = 13,
    DM_SW_VER = 14,
    DM_SN = 15,
    DM_NPP = 16,
    DM_RS = 17,
    DM_LS = 18,
    DM_FLUX = 19,
    DM_GR = 20,
    DM_PMAX = 21,
    DM_VMAX = 22,
    DM_TMAX = 23,
    DM_I_BW = 24,
    DM_KP_ASR = 25,
    DM_KI_ASR = 26,
    DM_KP_APR = 27,
    DM_KI_APR = 28,
    DM_OV_VALUE = 29,
    DM_GREF = 30,
    DM_DETA = 31,
    DM_V_BW = 32,
    DM_IQ_C1 = 33,
    DM_VL_C1 = 34,
    DM_CAN_BR = 35,
    DM_SUB_VER = 36,
    DM_U_OFF = 50,
    DM_V_OFF = 51,
    DM_K1 = 52,
    DM_K2 = 53,
    DM_M_OFF = 54,
    DM_DIR = 55,
    DM_P_M = 80,
    DM_XOUT = 81
};

typedef struct {
    float PosMax;       ///< Maximum position limit (rad)
    float SpdMax;       ///< Maximum velocity limit (rad/s)
    float TauMax;       ///< Maximum torque limit (N·m)
    float OKpMax;       ///< Maximum outer-loop proportional gain
    float OKdMax;       ///< Maximum outer-loop derivative gain
} DM_Limit_Param;

class DmMotorDriver : public MotorDriver {
   public:
    DmMotorDriver(uint16_t motor_id, const std::string& interface_type, const std::string& can_interface, uint16_t master_id_offset,
                  DM_Motor_Model motor_model, double motor_zero_offset = 0.0);
    ~DmMotorDriver();

    virtual void lock_motor() override;
    virtual void unlock_motor() override;
    virtual uint8_t init_motor() override;
    virtual void deinit_motor() override;
    virtual bool set_motor_zero() override;
    virtual bool write_motor_flash() override;
    virtual void get_motor_param(uint8_t param_cmd) override;
    
    virtual void motor_pos_cmd(float pos, float spd, bool ignore_limit) override;
    virtual void motor_spd_cmd(float spd) override;
    virtual void motor_mit_cmd(float f_p, float f_v, float f_kp, float f_kd, float f_t) override;
    virtual void motor_mit_cmd(float* f_p, float* f_v, float* f_kp, float* f_kd, float* f_t) override {};
    virtual void set_motor_control_mode(uint8_t motor_control_mode) override;
    virtual int get_response_count() const {
        return response_count_;
    }
    virtual void set_motor_id(uint8_t old_id, uint8_t new_id) override {};
    virtual void reset_motor_id() override {};
    virtual void refresh_motor_status() override;
    virtual void clear_motor_error() override;

   private:
    uint16_t master_id_;
    bool param_cmd_flag_[30] = {false};

    std::atomic<int> response_count_{0};
    DM_Motor_Model motor_model_;
    DM_Limit_Param limit_param_;
    std::atomic<uint8_t> mos_temperature_{0};
    void set_motor_zero_dm();
    void clear_motor_error_dm();
    void write_register_dm(uint8_t rid, float value);
    void write_register_dm(uint8_t rid, int32_t value);
    void save_register_dm();

    virtual void can_rx_cbk(const can_frame& rx_frame);
    std::shared_ptr<MotorsCAN> can_;
};
