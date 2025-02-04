// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <cmath>
#include <iostream>

#include <franka/exception.h>
#include <franka/robot.h>

#include "examples_common.h"

/**
 * @example generate_consecutive_motions.cpp
 * An example showing how to execute consecutive motions with error recovery.
 * 展示执行连续运动, 采用关节速度运动生成器
 *
 * @warning Before executing this example, make sure there is enough space in front and to the side
 * of the robot.
 * 在运行之前, 保证机器人前方和侧面有足够空间
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
    // 设置碰撞参数
    robot.setCollisionBehavior(
        {{10.0, 10.0, 9.0, 9.0, 8.0, 7.0, 6.0}}, {{10.0, 10.0, 9.0, 9.0, 8.0, 7.0, 6.0}},
        {{10.0, 10.0, 9.0, 9.0, 8.0, 7.0, 6.0}}, {{10.0, 10.0, 9.0, 9.0, 8.0, 7.0, 6.0}},
        {{10.0, 10.0, 10.0, 12.5, 12.5, 12.5}}, {{10.0, 10.0, 10.0, 12.5, 12.5, 12.5}},
        {{10.0, 10.0, 10.0, 12.5, 12.5, 12.5}}, {{10.0, 10.0, 10.0, 12.5, 12.5, 12.5}});

    for (size_t i = 0; i < 5; i++) {  // 执行连续的五次运动
      std::cout << "Executing motion." << std::endl;
      try {
        double time_max = 4.0;
        double omega_max = 0.2;
        double time = 0.0;
        robot.control([=, &time](const franka::RobotState&,
                                 franka::Duration period) -> franka::JointVelocities {
          time += period.toSec();
          // 周期时长为 time_max, 偶数周期为 1, 奇数周期为 -1
          double cycle = std::floor(std::pow(-1.0, (time - std::fmod(time, time_max)) / time_max));
          // 角速度, 偶数周期 0 -> omega_max -> 0, 奇数周期 0 -> -omega_max -> 0
          double omega = cycle * omega_max / 2.0 * (1.0 - std::cos(2.0 * M_PI / time_max * time));

          franka::JointVelocities velocities = {{0.0, 0.0, omega, 0.0, 0.0, 0.0, 0.0}};
          if (time >= 2 * time_max) {  // 两个周期后结束
            std::cout << std::endl << "Finished motion." << std::endl;
            return franka::MotionFinished(velocities);
          }
          return velocities;
        });
      } catch (const franka::ControlException& e) {
        std::cout << e.what() << std::endl;
        std::cout << "Running error recovery..." << std::endl;
        robot.automaticErrorRecovery();
      }
    }
  } catch (const franka::Exception& e) {
    std::cout << e.what() << std::endl;
    return -1;
  }

  std::cout << "Finished." << std::endl;

  return 0;
}
