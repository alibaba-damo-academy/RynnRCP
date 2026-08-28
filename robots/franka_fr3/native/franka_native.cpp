#include <franka/control_types.h>
#include <franka/exception.h>
#include <franka/gripper.h>
#include <franka/gripper_state.h>
#include <franka/model.h>
#include <franka/robot.h>
#include <franka/robot_state.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <ruckig/ruckig.hpp>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstddef>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace py = pybind11;
using Clock = std::chrono::steady_clock;

namespace {

constexpr std::size_t kJointCount = 7;
constexpr double kIkDamping = 0.05;
constexpr double kIkLookaheadSeconds = 0.1;
constexpr double kPositionTolerance = 0.0005;
constexpr double kRotationTolerance = 0.005;

enum class TargetMode { kJoint, kEndEffector };

template <std::size_t N>
std::array<double, N> to_array(const std::vector<double>& values, const char* name) {
  if (values.size() != N) {
    throw std::invalid_argument(std::string(name) + " must contain " + std::to_string(N) +
                                " values");
  }
  std::array<double, N> result{};
  for (std::size_t index = 0; index < N; ++index) {
    if (!std::isfinite(values[index])) {
      throw std::invalid_argument(std::string(name) + " values must be finite");
    }
    result[index] = values[index];
  }
  return result;
}

class FrankaHardware {
 public:
  FrankaHardware(std::string robot_ip,
                 bool with_gripper,
                 bool realtime_enforce,
                 double max_joint_velocity,
                 double max_joint_acceleration,
                 double max_joint_jerk,
                 double max_cartesian_velocity,
                 double max_cartesian_rotation,
                 double target_timeout,
                 const std::vector<double>& joint_lower_limits,
                 const std::vector<double>& joint_upper_limits,
                 const std::vector<double>& collision_torque_thresholds,
                 const std::vector<double>& collision_force_thresholds)
      : robot_ip_(std::move(robot_ip)),
        with_gripper_(with_gripper),
        realtime_enforce_(realtime_enforce),
        max_joint_velocity_(max_joint_velocity),
        max_joint_acceleration_(max_joint_acceleration),
        max_joint_jerk_(max_joint_jerk),
        max_cartesian_velocity_(max_cartesian_velocity),
        max_cartesian_rotation_(max_cartesian_rotation),
        target_timeout_(target_timeout),
        joint_lower_limits_(to_array<kJointCount>(joint_lower_limits, "joint_lower_limits")),
        joint_upper_limits_(to_array<kJointCount>(joint_upper_limits, "joint_upper_limits")),
        collision_torque_thresholds_(
            to_array<kJointCount>(collision_torque_thresholds, "collision_torque_thresholds")),
        collision_force_thresholds_(
            to_array<6>(collision_force_thresholds, "collision_force_thresholds")) {
    if (!(std::isfinite(max_joint_velocity_) && max_joint_velocity_ > 0.0)) {
      throw std::invalid_argument("max_joint_velocity must be positive");
    }
    if (!(std::isfinite(max_joint_acceleration_) && max_joint_acceleration_ > 0.0)) {
      throw std::invalid_argument("max_joint_acceleration must be positive");
    }
    if (!(std::isfinite(max_joint_jerk_) && max_joint_jerk_ > 0.0)) {
      throw std::invalid_argument("max_joint_jerk must be positive");
    }
    if (!(std::isfinite(max_cartesian_velocity_) && max_cartesian_velocity_ > 0.0)) {
      throw std::invalid_argument("max_cartesian_velocity must be positive");
    }
    if (!(std::isfinite(max_cartesian_rotation_) && max_cartesian_rotation_ > 0.0)) {
      throw std::invalid_argument("max_cartesian_rotation must be positive");
    }
    if (!(std::isfinite(target_timeout_) && target_timeout_ > 0.0)) {
      throw std::invalid_argument("target_timeout must be positive");
    }
    for (std::size_t index = 0; index < kJointCount; ++index) {
      if (joint_lower_limits_[index] >= joint_upper_limits_[index]) {
        throw std::invalid_argument("each lower joint limit must be below its upper limit");
      }
    }
  }

  ~FrankaHardware() {
    try {
      disconnect();
    } catch (...) {
    }
  }

  void connect() {
    std::lock_guard<std::mutex> lifecycle_lock(lifecycle_mutex_);
    if (connected_) {
      return;
    }

    const auto realtime_config = realtime_enforce_ ? franka::RealtimeConfig::kEnforce
                                                   : franka::RealtimeConfig::kIgnore;
    auto robot = std::make_unique<franka::Robot>(robot_ip_, realtime_config);
    const auto state = robot->readOnce();
    auto model = std::make_unique<franka::Model>(robot->loadModel());

    std::unique_ptr<franka::Gripper> gripper;
    franka::GripperState gripper_state{};
    if (with_gripper_) {
      gripper = std::make_unique<franka::Gripper>(robot_ip_);
      gripper_state = gripper->readOnce();
    }

    {
      std::lock_guard<std::mutex> state_lock(state_mutex_);
      robot_state_ = state;
      has_robot_state_ = true;
      gripper_state_ = gripper_state;
      has_gripper_state_ = with_gripper_;
      control_error_.clear();
    }
    {
      std::lock_guard<std::mutex> target_lock(target_mutex_);
      target_q_ = state.q;
      target_pose_ = state.O_T_EE;
      target_mode_ = TargetMode::kJoint;
      target_updated_at_ = Clock::now();
      has_received_target_ = false;
      watchdog_triggered_ = false;
    }
    robot_ = std::move(robot);
    model_ = std::move(model);
    gripper_ = std::move(gripper);
    gripper_connected_ = with_gripper_;
    connected_ = true;
  }

  void disconnect() {
    stop_control();
    std::lock_guard<std::mutex> lifecycle_lock(lifecycle_mutex_);
    if (gripper_) {
      try {
        gripper_->stop();
      } catch (...) {
      }
    }
    gripper_.reset();
    model_.reset();
    robot_.reset();
    gripper_connected_ = false;
    connected_ = false;
  }

  void start_control() {
    std::unique_lock<std::mutex> lifecycle_lock(lifecycle_mutex_);
    require_connected();
    if (control_thread_.joinable()) {
      if (control_running_) {
        return;
      }
      lifecycle_lock.unlock();
      control_thread_.join();
      lifecycle_lock.lock();
    }

    franka::RobotState state{};
    {
      std::lock_guard<std::mutex> state_lock(state_mutex_);
      if (!has_robot_state_) {
        throw std::runtime_error("Franka has no initial robot state");
      }
      state = robot_state_;
      control_error_.clear();
    }
    validate_joint_target(state.q);
    robot_->setCollisionBehavior(
        collision_torque_thresholds_, collision_torque_thresholds_,
        collision_torque_thresholds_, collision_torque_thresholds_,
        collision_force_thresholds_, collision_force_thresholds_,
        collision_force_thresholds_, collision_force_thresholds_);
    {
      std::lock_guard<std::mutex> target_lock(target_mutex_);
      target_q_ = state.q;
      target_pose_ = state.O_T_EE;
      target_mode_ = TargetMode::kJoint;
      target_updated_at_ = Clock::now();
      has_received_target_ = false;
      watchdog_triggered_ = false;
    }
    trajectory_input_ = ruckig::InputParameter<kJointCount>{};
    trajectory_output_ = ruckig::OutputParameter<kJointCount>{};
    trajectory_input_.current_position = state.q_d;
    trajectory_input_.current_velocity = state.dq_d;
    trajectory_input_.current_acceleration = state.ddq_d;
    trajectory_input_.target_position = state.q;
    trajectory_input_.target_velocity.fill(0.0);
    trajectory_input_.target_acceleration.fill(0.0);
    trajectory_input_.max_velocity.fill(max_joint_velocity_);
    trajectory_input_.max_acceleration.fill(max_joint_acceleration_);
    trajectory_input_.max_jerk.fill(max_joint_jerk_);
    trajectory_input_.min_position = joint_lower_limits_;
    trajectory_input_.max_position = joint_upper_limits_;
    {
      std::lock_guard<std::mutex> start_lock(start_mutex_);
      control_started_ = false;
      control_start_failed_ = false;
    }
    stop_requested_ = false;
    // Publish the loop state before launching the worker. start_control()
    // waits for the first libfranka callback or a startup failure before it
    // returns to Python.
    control_running_ = true;
    try {
      control_thread_ = std::thread(&FrankaHardware::control_worker, this);
    } catch (...) {
      control_running_ = false;
      throw;
    }
    lifecycle_lock.unlock();

    std::unique_lock<std::mutex> start_lock(start_mutex_);
    if (!start_cv_.wait_for(start_lock, std::chrono::seconds(5), [this] {
          return control_started_ || control_start_failed_;
        })) {
      start_lock.unlock();
      stop_control();
      throw std::runtime_error("Franka control session did not start within 5 seconds");
    }
    if (control_start_failed_) {
      std::string error;
      {
        std::lock_guard<std::mutex> state_lock(state_mutex_);
        error = control_error_;
      }
      start_lock.unlock();
      stop_control();
      throw std::runtime_error("Franka control session failed: " + error);
    }
  }

  void stop_control() {
    stop_requested_ = true;
    if (control_thread_.joinable() && control_thread_.get_id() != std::this_thread::get_id()) {
      control_thread_.join();
    }
    control_running_ = false;
  }

  void set_joint_target(const std::vector<double>& values) {
    const auto target = to_array<kJointCount>(values, "joint target");
    validate_joint_target(target);
    if (!control_running_) {
      throw std::runtime_error("Franka control is disabled");
    }
    std::lock_guard<std::mutex> target_lock(target_mutex_);
    target_q_ = target;
    target_mode_ = TargetMode::kJoint;
    target_updated_at_ = Clock::now();
    has_received_target_ = true;
    watchdog_triggered_ = false;
  }

  void set_ee_target(const std::vector<double>& position_values,
                     const std::vector<double>& quaternion_values) {
    const auto position = to_array<3>(position_values, "end-effector position");
    const auto quaternion = to_array<4>(quaternion_values, "end-effector quaternion");
    if (!control_running_) {
      throw std::runtime_error("Franka control is disabled");
    }

    Eigen::Quaterniond orientation(quaternion[3], quaternion[0], quaternion[1],
                                   quaternion[2]);
    if (orientation.norm() < 1e-12) {
      throw std::invalid_argument("end-effector quaternion must have a non-zero norm");
    }
    orientation.normalize();
    Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
    transform.block<3, 3>(0, 0) = orientation.toRotationMatrix();
    transform.block<3, 1>(0, 3) =
        Eigen::Vector3d(position[0], position[1], position[2]);

    std::array<double, 16> target{};
    Eigen::Map<Eigen::Matrix<double, 4, 4, Eigen::ColMajor>>(target.data()) = transform;
    std::lock_guard<std::mutex> target_lock(target_mutex_);
    target_pose_ = target;
    target_mode_ = TargetMode::kEndEffector;
    target_updated_at_ = Clock::now();
    has_received_target_ = true;
    watchdog_triggered_ = false;
  }

  py::dict get_state() {
    if (!connected_) {
      throw std::runtime_error("Franka is not connected");
    }

    franka::RobotState state{};
    {
      std::lock_guard<std::mutex> state_lock(state_mutex_);
      if (!has_robot_state_) {
        throw std::runtime_error("Franka has no robot state");
      }
      state = robot_state_;
    }
    py::dict result;
    result["q"] = state.q;
    result["dq"] = state.dq;
    result["tau_J"] = state.tau_J;
    result["O_T_EE"] = state.O_T_EE;
    return result;
  }

  py::dict get_gripper_state() {
    std::lock_guard<std::mutex> gripper_lock(gripper_mutex_);
    require_gripper();
    const auto state = gripper_->readOnce();
    {
      std::lock_guard<std::mutex> state_lock(state_mutex_);
      gripper_state_ = state;
      has_gripper_state_ = true;
    }
    return gripper_state_dict(state);
  }

  void gripper_homing() {
    std::lock_guard<std::mutex> gripper_lock(gripper_mutex_);
    require_gripper();
    if (!gripper_->homing()) {
      throw std::runtime_error("Franka gripper homing did not complete");
    }
  }

  void gripper_move(double width, double speed) {
    std::lock_guard<std::mutex> gripper_lock(gripper_mutex_);
    require_gripper();
    if (!gripper_->move(width, speed)) {
      throw std::runtime_error("Franka gripper move did not reach its target");
    }
  }

  void gripper_grasp(double width,
                     double speed,
                     double force,
                     double epsilon_inner,
                     double epsilon_outer) {
    std::lock_guard<std::mutex> gripper_lock(gripper_mutex_);
    require_gripper();
    if (!gripper_->grasp(width, speed, force, epsilon_inner, epsilon_outer)) {
      throw std::runtime_error("Franka gripper did not detect a successful grasp");
    }
  }

  void gripper_stop() {
    std::lock_guard<std::mutex> gripper_lock(gripper_mutex_);
    require_gripper();
    if (!gripper_->stop()) {
      throw std::runtime_error("Franka gripper stop command failed");
    }
  }

  void automatic_error_recovery() {
    std::lock_guard<std::mutex> lifecycle_lock(lifecycle_mutex_);
    require_connected();
    if (control_running_) {
      throw std::runtime_error("stop Franka control before automatic error recovery");
    }
    robot_->automaticErrorRecovery();
    std::lock_guard<std::mutex> state_lock(state_mutex_);
    control_error_.clear();
  }

  py::dict status() const {
    py::dict result;
    result["connected"] = connected_.load();
    result["control_enabled"] = control_running_.load();
    result["gripper_connected"] = gripper_connected_.load();
    result["realtime_enforce"] = realtime_enforce_;
    {
      std::lock_guard<std::mutex> target_lock(target_mutex_);
      result["watchdog_triggered"] = watchdog_triggered_;
    }
    {
      std::lock_guard<std::mutex> state_lock(state_mutex_);
      result["control_error"] = control_error_;
    }
    return result;
  }

 private:
  void control_worker() {
    try {
      robot_->control(
          [this](const franka::RobotState& state, franka::Duration) {
            {
              std::lock_guard<std::mutex> state_lock(state_mutex_);
              robot_state_ = state;
              has_robot_state_ = true;
            }
            {
              std::lock_guard<std::mutex> start_lock(start_mutex_);
              if (!control_started_) {
                control_started_ = true;
                start_cv_.notify_all();
              }
            }

            std::array<double, kJointCount> command{};
            {
              std::lock_guard<std::mutex> target_lock(target_mutex_);
              const bool target_timed_out =
                  has_received_target_ &&
                  Clock::now() - target_updated_at_ >
                  std::chrono::duration<double>(target_timeout_);
              if (target_timed_out && !watchdog_triggered_) {
                // Cancel the unfinished target once, then let Ruckig decelerate
                // continuously to the currently commanded joint position.
                target_q_ = state.q_d;
                target_mode_ = TargetMode::kJoint;
                watchdog_triggered_ = true;
              }
              trajectory_input_.control_interface =
                  ruckig::ControlInterface::Position;
              trajectory_input_.synchronization = ruckig::Synchronization::Time;
              if (target_mode_ == TargetMode::kEndEffector) {
                target_q_ = solve_ik_step(state, target_pose_);
              }
              trajectory_input_.target_position = target_q_;
              trajectory_input_.target_velocity.fill(0.0);
              trajectory_input_.target_acceleration.fill(0.0);

              const auto result =
                  trajectory_generator_.update(trajectory_input_, trajectory_output_);
              if (result < ruckig::Result::Working) {
                throw std::runtime_error("Ruckig trajectory generation failed: " +
                                         std::to_string(static_cast<int>(result)));
              }
              command = trajectory_output_.new_position;
              trajectory_output_.pass_to_input(trajectory_input_);
            }

            franka::JointPositions output(command);
            if (stop_requested_) {
              return franka::MotionFinished(output);
            }
            return output;
          },
          franka::ControllerMode::kJointImpedance, true);
    } catch (const std::exception& exception) {
      {
        std::lock_guard<std::mutex> state_lock(state_mutex_);
        control_error_ = exception.what();
      }
      {
        std::lock_guard<std::mutex> start_lock(start_mutex_);
        if (!control_started_) {
          control_start_failed_ = true;
          start_cv_.notify_all();
        }
      }
    }
    control_running_ = false;
  }

  void require_connected() const {
    if (!connected_ || !robot_) {
      throw std::runtime_error("Franka is not connected");
    }
  }

  void require_gripper() const {
    if (!with_gripper_ || !gripper_connected_ || !gripper_) {
      throw std::runtime_error("Franka gripper is not connected");
    }
  }

  std::array<double, kJointCount> solve_ik_step(
      const franka::RobotState& state,
      const std::array<double, 16>& target_pose) const {
    if (!model_) {
      throw std::runtime_error("Franka model is not loaded");
    }
    const auto jacobian_array = model_->zeroJacobian(franka::Frame::kEndEffector, state);
    const Eigen::Map<const Eigen::Matrix<double, 6, 7, Eigen::ColMajor>> jacobian(
        jacobian_array.data());
    const Eigen::Map<const Eigen::Matrix<double, 4, 4, Eigen::ColMajor>> current(
        state.O_T_EE.data());
    const Eigen::Map<const Eigen::Matrix<double, 4, 4, Eigen::ColMajor>> target(
        target_pose.data());

    Eigen::Vector3d translation_error =
        target.block<3, 1>(0, 3) - current.block<3, 1>(0, 3);
    Eigen::Quaterniond current_orientation(current.block<3, 3>(0, 0));
    Eigen::Quaterniond target_orientation(target.block<3, 3>(0, 0));
    Eigen::Quaterniond orientation_error =
        target_orientation * current_orientation.conjugate();
    if (orientation_error.w() < 0.0) {
      orientation_error.coeffs() *= -1.0;
    }
    const Eigen::Vector3d rotation_error = 2.0 * orientation_error.vec();

    Eigen::Matrix<double, 6, 1> desired_twist = Eigen::Matrix<double, 6, 1>::Zero();
    if (translation_error.norm() > kPositionTolerance) {
      desired_twist.head<3>() =
          translation_error.normalized() *
          std::min(max_cartesian_velocity_, translation_error.norm());
    }
    if (rotation_error.norm() > kRotationTolerance) {
      desired_twist.tail<3>() =
          rotation_error.normalized() *
          std::min(max_cartesian_rotation_, rotation_error.norm());
    }

    const Eigen::Matrix<double, 6, 6> regularized =
        jacobian * jacobian.transpose() +
        kIkDamping * kIkDamping * Eigen::Matrix<double, 6, 6>::Identity();
    Eigen::Matrix<double, 7, 1> joint_velocity =
        jacobian.transpose() * regularized.ldlt().solve(desired_twist);

    std::array<double, kJointCount> target_q{};
    for (std::size_t index = 0; index < kJointCount; ++index) {
      const double velocity =
          std::clamp(joint_velocity[static_cast<Eigen::Index>(index)],
                     -max_joint_velocity_, max_joint_velocity_);
      target_q[index] =
          std::clamp(state.q[index] + velocity * kIkLookaheadSeconds,
                     joint_lower_limits_[index], joint_upper_limits_[index]);
    }
    return target_q;
  }

  void validate_joint_target(const std::array<double, kJointCount>& target) const {
    for (std::size_t index = 0; index < kJointCount; ++index) {
      if (!std::isfinite(target[index])) {
        throw std::invalid_argument("Franka joint target values must be finite");
      }
      if (target[index] < joint_lower_limits_[index] ||
          target[index] > joint_upper_limits_[index]) {
        throw std::invalid_argument("Franka joint target is outside configured limits");
      }
    }
  }

  static py::dict gripper_state_dict(const franka::GripperState& state) {
    py::dict result;
    result["width"] = state.width;
    result["max_width"] = state.max_width;
    result["is_grasped"] = state.is_grasped;
    result["temperature"] = state.temperature;
    return result;
  }

  std::string robot_ip_;
  bool with_gripper_;
  bool realtime_enforce_;
  double max_joint_velocity_;
  double max_joint_acceleration_;
  double max_joint_jerk_;
  double max_cartesian_velocity_;
  double max_cartesian_rotation_;
  double target_timeout_;
  std::array<double, kJointCount> joint_lower_limits_{};
  std::array<double, kJointCount> joint_upper_limits_{};
  std::array<double, kJointCount> collision_torque_thresholds_{};
  std::array<double, 6> collision_force_thresholds_{};

  mutable std::mutex lifecycle_mutex_;
  mutable std::mutex state_mutex_;
  mutable std::mutex target_mutex_;
  mutable std::mutex gripper_mutex_;
  std::mutex start_mutex_;
  std::condition_variable start_cv_;

  std::unique_ptr<franka::Robot> robot_;
  std::unique_ptr<franka::Model> model_;
  std::unique_ptr<franka::Gripper> gripper_;
  std::thread control_thread_;
  std::atomic<bool> connected_{false};
  std::atomic<bool> gripper_connected_{false};
  std::atomic<bool> control_running_{false};
  std::atomic<bool> stop_requested_{false};
  bool control_started_{false};
  bool control_start_failed_{false};

  franka::RobotState robot_state_{};
  franka::GripperState gripper_state_{};
  bool has_robot_state_{false};
  bool has_gripper_state_{false};
  std::string control_error_;

  std::array<double, kJointCount> target_q_{};
  std::array<double, 16> target_pose_{};
  TargetMode target_mode_{TargetMode::kJoint};
  ruckig::Ruckig<kJointCount> trajectory_generator_{0.001};
  ruckig::InputParameter<kJointCount> trajectory_input_{};
  ruckig::OutputParameter<kJointCount> trajectory_output_{};
  Clock::time_point target_updated_at_{Clock::now()};
  bool has_received_target_{false};
  bool watchdog_triggered_{false};
};

}  // namespace

PYBIND11_MODULE(_franka_native, module) {
  module.doc() = "RynnRCP native driver for libfranka 0.13.3";
  py::class_<FrankaHardware>(module, "FrankaHardware")
      .def(py::init<std::string, bool, bool, double, double, double, double, double, double,
                    const std::vector<double>&, const std::vector<double>&,
                    const std::vector<double>&, const std::vector<double>&>())
      .def("connect", &FrankaHardware::connect, py::call_guard<py::gil_scoped_release>())
      .def("disconnect", &FrankaHardware::disconnect,
           py::call_guard<py::gil_scoped_release>())
      .def("start_control", &FrankaHardware::start_control,
           py::call_guard<py::gil_scoped_release>())
      .def("stop_control", &FrankaHardware::stop_control,
           py::call_guard<py::gil_scoped_release>())
      .def("set_joint_target", &FrankaHardware::set_joint_target)
      .def("set_ee_target", &FrankaHardware::set_ee_target)
      .def("get_state", &FrankaHardware::get_state)
      .def("get_gripper_state", &FrankaHardware::get_gripper_state)
      .def("gripper_homing", &FrankaHardware::gripper_homing,
           py::call_guard<py::gil_scoped_release>())
      .def("gripper_move", &FrankaHardware::gripper_move,
           py::call_guard<py::gil_scoped_release>())
      .def("gripper_grasp", &FrankaHardware::gripper_grasp,
           py::call_guard<py::gil_scoped_release>())
      .def("gripper_stop", &FrankaHardware::gripper_stop,
           py::call_guard<py::gil_scoped_release>())
      .def("automatic_error_recovery", &FrankaHardware::automatic_error_recovery,
           py::call_guard<py::gil_scoped_release>())
      .def("status", &FrankaHardware::status);
}
