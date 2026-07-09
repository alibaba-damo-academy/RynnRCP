#pragma once

#include <string>
#include <vector>
#include <mutex>
#include <atomic>
#include <condition_variable>
#include <algorithm>
#include <memory>
#include <thread>
#include <Eigen/Geometry>
#include <cmath>
#include <chrono>
#include <filesystem>
#include <iostream>
#include <queue>
#include <sstream>
#include <yaml-cpp/yaml.h>
#include "utils/close_chain_mapping.hpp"
#include "utils/thread_pool.hpp"
#include "motor/motor_driver.hpp"
#include "imu/imu_driver.hpp"

class RobotInterface {
   public:
    RobotInterface(const std::string& config_file);
    ~RobotInterface() {
        stop_control_loop();
        deinit_motors();
        motors_.clear();
        imu_.reset();
    }
    struct IMUCfg{
        int imu_id_, baudrate_;
        std::string imu_type_, imu_interface_type_, imu_interface_;
    };
    struct MotorsCfg{
        int master_id_offset_;
        std::vector<std::string> motor_type_;
        std::vector<std::string> motor_interface_type_;
        std::vector<std::string> motor_interface_;
        std::vector<long int> motor_id_, motor_model_, motor_num_;
        std::vector<double> motor_zero_offset_;
    };
    struct RobotCfg{
        std::vector<long int> close_chain_motor_idx_, motor_sign_, urdf2motor_;
        std::vector<double> kp_, kd_, extrinsic_R_;
    };

    void apply_action(std::vector<float> action);
    void init_motors();
    void deinit_motors();
    void reset_joints(std::vector<double> joint_default_angle);
    void set_zeros();
    void set_zero_single(int motor_index);
    void clear_errors();
    void refresh_joints();

    // ── 阻尼 / 软失能 ──
    /// 进入纯阻尼模式：kp=0，所有关节以 kd 阻尼值抵抗运动
    void set_damping(float kd);
    /// 退出阻尼模式，恢复正常控制
    void clear_damping();
    /// 软失能：先切阻尼 → 逐步降低 kd → 最后 unlock
    void soft_deinit_motors(float damping_kd = 2.0f, float ramp_down_sec = 3.0f);
    /// 查询是否处于阻尼模式
    bool is_damping() const { return damping_enabled_.load(); }
    float get_damping_kd() const { return damping_kd_.load(); }
    /// 控制循环中调用：阻尼模式下发送 kp=0 指令
    void apply_damping();

    // ── Python 绑定用：内置控制循环 ──
    /// 启动 C++ 控制线程（250Hz, SCHED_FIFO）
    void start_control_loop();
    /// 停止控制线程
    void stop_control_loop();
    /// 线程安全地设置目标位置
    void set_targets(const std::vector<float>& targets);
    /// 获取关节数量
    size_t get_joint_num() const { return motors_cfg_ ? motors_cfg_->motor_id_.size() : 0; }
    std::vector<float> get_joint_q() {
        if (!is_init_.load()) {
            throw std::runtime_error("Motors not initialized");
        }
        std::unique_lock<std::mutex> lock(joint_mutex_);
        return joint_q_;
    }
    std::vector<float> get_joint_vel() {
        if (!is_init_.load()) {
            throw std::runtime_error("Motors not initialized");
        }
        std::unique_lock<std::mutex> lock(joint_mutex_);
        return joint_vel_;
    }
    std::vector<float> get_joint_tau() {
        if (!is_init_.load()) {
            throw std::runtime_error("Motors not initialized");
        }
        std::unique_lock<std::mutex> lock(joint_mutex_);
        return joint_tau_;
    }
    const std::vector<float>& get_quat() {
        if (!imu_) {
            throw std::runtime_error("IMU not initialized");
        }
        auto raw = imu_->get_quat();  // w, x, y, z
        q_body_ = Eigen::Quaternionf(raw[0], raw[1], raw[2], raw[3]) * extrinsic_q_inv_;
        q_body_.normalize();
        quat_buf_[0] = q_body_.w();
        quat_buf_[1] = q_body_.x();
        quat_buf_[2] = q_body_.y(); 
        quat_buf_[3] = q_body_.z();
        return quat_buf_;
    }
    const std::vector<float>& get_ang_vel() {
        if (!imu_) {
            throw std::runtime_error("IMU not initialized");
        }
        auto raw = imu_->get_ang_vel();  // in IMU frame
        Eigen::Map<const Eigen::Vector3f> omega_imu(raw.data());
        Eigen::Map<Eigen::Vector3f>(ang_vel_buf_.data()) = extrinsic_R_mat_ * omega_imu;
        return ang_vel_buf_;
    }
    const std::vector<float>& get_lin_acc() {
        if (!imu_) {
            throw std::runtime_error("IMU not initialized");
        }
        auto raw = imu_->get_lin_acc();  // in IMU frame
        Eigen::Map<const Eigen::Vector3f> acc_imu(raw.data());
        Eigen::Map<Eigen::Vector3f>(lin_acc_buf_.data()) = extrinsic_R_mat_ * acc_imu;
        return lin_acc_buf_;
    }

    std::atomic<bool> is_init_{false};

    // ── 阻尼状态 ──
    std::atomic<bool> damping_enabled_{false};
    std::atomic<float> damping_kd_{2.0f};

   private:
    std::shared_ptr<IMUCfg> imu_cfg_;
    std::shared_ptr<MotorsCfg> motors_cfg_;
    std::shared_ptr<RobotCfg> robot_cfg_;
    int offline_threshold_ = 25;
    std::shared_ptr<IMUDriver> imu_;
    std::shared_ptr<Decouple> ankle_decouple_;
    Eigen::Matrix3f extrinsic_R_mat_ = Eigen::Matrix3f::Identity();
    Eigen::Quaternionf extrinsic_q_inv_ = Eigen::Quaternionf::Identity();
    Eigen::Quaternionf q_body_;
    std::vector<float> quat_buf_{0.f, 0.f, 0.f, 0.f};
    std::vector<float> ang_vel_buf_{0.f, 0.f, 0.f};
    std::vector<float> lin_acc_buf_{0.f, 0.f, 0.f};
    std::vector<std::shared_ptr<MotorDriver>> motors_;
    std::unique_ptr<ThreadPool> thread_pool_;

    std::mutex motors_mutex_, joint_mutex_, targets_mutex_;
    std::vector<float> joint_q_, joint_vel_, joint_tau_, motor_target_;
    std::vector<int> close_chain_joint_idx_, motor2urdf_;

    // ── 控制循环线程 ──
    std::thread control_loop_thread_;
    std::atomic<bool> control_loop_running_{false};
    std::vector<float> control_targets_;

    void setup_motors();
    void setup_imu();

    void exec_motors_parallel(const std::function<void(std::shared_ptr<MotorDriver>&, int)>& cmd_func);
    void motors_mit_cmd(float kp_scale = 1.0f, float kd_scale = 1.0f, bool close_chain_tau = false);
    void motors_damping_cmd(float kd);
};
