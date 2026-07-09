// SPDX-License-Identifier: GPL-3.0
// Python bindings for RobotInterface via pybind11

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "robot_interface.hpp"

namespace py = pybind11;

PYBIND11_MODULE(atom01_py, m) {
    m.doc() = "Atom01 Robot Interface Python SDK";

    py::class_<RobotInterface>(m, "RobotInterface")
        .def(py::init<const std::string&>(), py::arg("config_file"))
        // 电机生命周期
        .def("init_motors", &RobotInterface::init_motors)
        .def("deinit_motors", &RobotInterface::deinit_motors)
        .def("soft_deinit_motors", &RobotInterface::soft_deinit_motors,
             py::arg("damping_kd") = 2.0f, py::arg("ramp_down_sec") = 3.0f)
        .def("reset_joints", &RobotInterface::reset_joints, py::arg("joint_default_angle"))
        .def("set_zeros", &RobotInterface::set_zeros)
        .def("set_zero_single", &RobotInterface::set_zero_single, py::arg("motor_index"))
        .def("clear_errors", &RobotInterface::clear_errors)
        .def("refresh_joints", &RobotInterface::refresh_joints)
        // 阻尼模式
        .def("set_damping", &RobotInterface::set_damping, py::arg("kd"))
        .def("clear_damping", &RobotInterface::clear_damping)
        .def("is_damping", &RobotInterface::is_damping)
        .def("get_damping_kd", &RobotInterface::get_damping_kd)
        // 控制循环（Python 绑定专用）
        .def("start_control_loop", &RobotInterface::start_control_loop)
        .def("stop_control_loop", &RobotInterface::stop_control_loop)
        .def("set_targets", &RobotInterface::set_targets, py::arg("targets"))
        .def("get_joint_num", &RobotInterface::get_joint_num)
        // 读取状态
        .def("apply_action", &RobotInterface::apply_action, py::arg("action"))
        .def("get_joint_q", &RobotInterface::get_joint_q)
        .def("get_joint_vel", &RobotInterface::get_joint_vel)
        .def("get_joint_tau", &RobotInterface::get_joint_tau)
        .def("get_quat", [](RobotInterface& r) {
            auto& v = r.get_quat();
            return std::vector<float>(v.begin(), v.end());
        })
        .def("get_ang_vel", [](RobotInterface& r) {
            auto& v = r.get_ang_vel();
            return std::vector<float>(v.begin(), v.end());
        })
        .def("get_lin_acc", [](RobotInterface& r) {
            auto& v = r.get_lin_acc();
            return std::vector<float>(v.begin(), v.end());
        })
        // 属性
        .def_property_readonly("is_init", [](const RobotInterface& r) {
            return r.is_init_.load();
        });
}
