// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <cmath>
#include <iostream>

#include <franka/exception.h>
#include <franka/robot.h>

#include "examples_common.h"

/**
 * @example generate_cartesian_velocity_motion.cpp
 * An example showing how to generate a Cartesian velocity motion.
 * 展示笛卡尔速度运动生成器
 *
 * @warning Before executing this example, make sure there is enough space in front of the robot.
 * 开始前确保机器人前有足够空间
 */

int main(int argc, char** argv) {
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " <robot-hostname>" << std::endl;
    return -1;
  }
  try {
    franka::Robot robot(argv[1]);
    setDefaultBehavior(robot);

    // 首先将机器人复位, 这里直接使用了 examples_common.h 中定义的 motion_generator
    // 只要给定目标 q, 即可自动按照(S 型速度规划曲线)插值生成 关节空间 的轨迹
    std::array<double, 7> q_goal = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
    MotionGenerator motion_generator(0.5, q_goal);
    std::cout << "WARNING: This example will move the robot! "
              << "Please make sure to have the user stop button at hand!" << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();
    robot.control(motion_generator);
    std::cout << "Finished moving to initial joint configuration." << std::endl;

    // Set additional parameters always before the control loop, NEVER in the control loop!
    // 设置关节阻抗
    robot.setJointImpedance({{3000, 3000, 3000, 2500, 2500, 2000, 2000}});

    // 设置碰撞属性
    std::array<double, 7> lower_torque_thresholds_nominal{
        {25.0, 25.0, 22.0, 20.0, 19.0, 17.0, 14.}};
    std::array<double, 7> upper_torque_thresholds_nominal{
        {35.0, 35.0, 32.0, 30.0, 29.0, 27.0, 24.0}};
    std::array<double, 7> lower_torque_thresholds_acceleration{
        {25.0, 25.0, 22.0, 20.0, 19.0, 17.0, 14.0}};
    std::array<double, 7> upper_torque_thresholds_acceleration{
        {35.0, 35.0, 32.0, 30.0, 29.0, 27.0, 24.0}};
    std::array<double, 6> lower_force_thresholds_nominal{{30.0, 30.0, 30.0, 25.0, 25.0, 25.0}};
    std::array<double, 6> upper_force_thresholds_nominal{{40.0, 40.0, 40.0, 35.0, 35.0, 35.0}};
    std::array<double, 6> lower_force_thresholds_acceleration{{30.0, 30.0, 30.0, 25.0, 25.0, 25.0}};
    std::array<double, 6> upper_force_thresholds_acceleration{{40.0, 40.0, 40.0, 35.0, 35.0, 35.0}};
    robot.setCollisionBehavior(
        lower_torque_thresholds_acceleration, upper_torque_thresholds_acceleration,
        lower_torque_thresholds_nominal, upper_torque_thresholds_nominal,
        lower_force_thresholds_acceleration, upper_force_thresholds_acceleration,
        lower_force_thresholds_nominal, upper_force_thresholds_nominal);

    double time_max = 4.0;  // 三角函数周期
    double v_max = 0.1;
    double angle = M_PI / 4.0;  // 在 xz 平面的运动方向
    double time = 0.0;
    robot.control([=, &time](const franka::RobotState&,
                             franka::Duration period) -> franka::CartesianVelocities {
      time += period.toSec();
      // 周期时长为 time_max, 偶数周期为 1, 奇数周期为 -1
      double cycle = std::floor(pow(-1.0, (time - std::fmod(time, time_max)) / time_max));
      // 速度, 偶数周期 0 -> v_max -> 0, 奇数周期 0 -> -v_max -> 0
      double v = cycle * v_max / 2.0 * (1.0 - std::cos(2.0 * M_PI / time_max * time));
      double v_x = std::cos(angle) * v;
      double v_z = -std::sin(angle) * v;

      franka::CartesianVelocities output = {{v_x, 0.0, v_z, 0.0, 0.0, 0.0}};
      if (time >= 2 * time_max) {  // 两个周期后停止运动
        std::cout << std::endl << "Finished motion, shutting down example" << std::endl;
        return franka::MotionFinished(output);
      }
      return output;
    });
  } catch (const franka::Exception& e) {
    std::cout << e.what() << std::endl;
    return -1;
  }

  return 0;
}
