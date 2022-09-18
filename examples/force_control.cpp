// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <array>
#include <iostream>

#include <Eigen/Core>

#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/model.h>
#include <franka/robot.h>

#include "examples_common.h"

/**
 * @example force_control.cpp
 * A simple PI force controller that renders in the Z axis the gravitational force corresponding
 * to a target mass of 1 kg.
 * 一个简单的 PI 力控制器, 在 Z 轴上呈现对应于 1 千克目标质量的重力
 *
 * @warning: make sure that no endeffector is mounted and that the robot's last joint is in contact
 * with a horizontal rigid surface before starting.
 * 开始之前, 确保没有安装末端执行器, 并且机器人的最后一个关节水平接触一个刚性平面
 */

int main(int argc, char** argv) {
  // Check whether the required arguments were passed
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " <robot-hostname>" << std::endl;
    return -1;
  }
  // parameters
  double desired_mass{0.0};
  constexpr double target_mass{1.0};    // NOLINT(readability-identifier-naming)
  constexpr double k_p{1.0};            // NOLINT(readability-identifier-naming)
  constexpr double k_i{2.0};            // NOLINT(readability-identifier-naming)
  constexpr double filter_gain{0.001};  // NOLINT(readability-identifier-naming)

  try {
    // connect to robot
    franka::Robot robot(argv[1]);
    setDefaultBehavior(robot);
    // load the kinematics and dynamics model 载入运动学和动力学模型
    franka::Model model = robot.loadModel();

    // set collision behavior 设置碰撞较高碰撞检测阈值
    robot.setCollisionBehavior({{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                               {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                               {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                               {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}});

    franka::RobotState initial_state = robot.readOnce();

    Eigen::VectorXd initial_tau_ext(7), tau_error_integral(7);
    // Bias torque sensor
    std::array<double, 7> gravity_array = model.gravity(initial_state);  // 初始自身重力力矩
    Eigen::Map<Eigen::Matrix<double, 7, 1>> initial_tau_measured(initial_state.tau_J.data());  // 测量的初始关节力矩
    Eigen::Map<Eigen::Matrix<double, 7, 1>> initial_gravity(gravity_array.data());
    initial_tau_ext = initial_tau_measured - initial_gravity;  // 初始外部力矩

    // init integrator 初始化积分器
    tau_error_integral.setZero();

    // define callback for the torque control loop  定义力控的回调函数
    Eigen::Vector3d initial_position;
    double time = 0.0;
    // 返回末端执行器的 xyz 坐标
    auto get_position = [](const franka::RobotState& robot_state) {
      return Eigen::Vector3d(robot_state.O_T_EE[12], robot_state.O_T_EE[13],
                             robot_state.O_T_EE[14]);
    };
    // 力控回调函数
    auto force_control_callback = [&](const franka::RobotState& robot_state,
                                      franka::Duration period) -> franka::Torques {
      time += period.toSec();

      if (time == 0.0) {
        initial_position = get_position(robot_state);
      }

      // 一旦检测到末端执行器位移大于 0.01 即停止
      if (time > 0 && (get_position(robot_state) - initial_position).norm() > 0.01) {
        throw std::runtime_error("Aborting; too far away from starting pose!");
      }

      // get state variables 获取状态变量
      std::array<double, 42> jacobian_array =
          model.zeroJacobian(franka::Frame::kEndEffector, robot_state);

      Eigen::Map<const Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
      Eigen::Map<const Eigen::Matrix<double, 7, 1>> tau_measured(robot_state.tau_J.data());
      Eigen::Map<const Eigen::Matrix<double, 7, 1>> gravity(gravity_array.data());

      Eigen::VectorXd tau_d(7), desired_force_torque(6), tau_cmd(7), tau_ext(7);
      desired_force_torque.setZero();  // 笛卡尔空间的期望输出力
      desired_force_torque(2) = desired_mass * -9.81;  // z 方向输出力
      tau_d << jacobian.transpose() * desired_force_torque;  // 关节空间, 期望输出力矩
      tau_ext << tau_measured - gravity - initial_tau_ext;  // 当前输出力矩 tau_d - tau_ext 为偏差值
      tau_error_integral += period.toSec() * (tau_d - tau_ext);  // 积分项
      // FF + PI control
      tau_cmd << tau_d + k_p * (tau_d - tau_ext) + k_i * tau_error_integral;

      // Smoothly update the mass to reach the desired target value
      // 平滑过渡重量 从 0 ~ 1 kg
      // x_n = 1 - (1-gain)^n 的递推形式  x_i+1 = (1-gain) x_i + gain
      desired_mass = filter_gain * target_mass + (1 - filter_gain) * desired_mass;

      std::array<double, 7> tau_d_array{};
      Eigen::VectorXd::Map(&tau_d_array[0], 7) = tau_cmd;
      return tau_d_array;
    };
    std::cout << "WARNING: Make sure sure that no endeffector is mounted and that the robot's last "
                 "joint is "
                 "in contact with a horizontal rigid surface before starting. Keep in mind that "
                 "collision thresholds are set to high values."
              << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();
    // start real-time control loop 开始运行
    robot.control(force_control_callback);

  } catch (const std::exception& ex) {
    // print exception
    std::cout << ex.what() << std::endl;
  }
  return 0;
}
