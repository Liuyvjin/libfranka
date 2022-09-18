// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <cmath>
#include <iostream>

#include <franka/exception.h>
#include <franka/robot.h>

#include "examples_common.h"

/**
 * @example generate_cartesian_pose_motion.cpp
 * 展示笛卡尔空间运动生成器的写法
 * 将末端执行器平滑运动到 (0.3, 0, 0.3) 再返回原位
 *
 * @warning 在运行这个示例之前, 确保机器人前有足够空间
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
    robot.control(motion_generator);  // 执行运动
    std::cout << "Finished moving to initial joint configuration." << std::endl;

    // 设置机器人参数
    // Set additional parameters always before the control loop, NEVER in the control loop!
    // Set collision behavior.
    robot.setCollisionBehavior(
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});

    std::array<double, 16> initial_pose;
    double time = 0.0;

    // 运行自定义的笛卡尔运动轨迹生成器
    // 运动时长 10 s
    robot.control([&time, &initial_pose](const franka::RobotState& robot_state,
                                         franka::Duration period) -> franka::CartesianPose {
      time += period.toSec();

      if (time == 0.0) {
        // 将第一次读取的机器人状态中的 O_T_EE_c 作为初始状态
        // O_T_EE_c: 运动轨迹生成器的上一条指令末端执行器相对于基座的位姿.
        // 参考: https://frankaemika.github.io/libfranka/structfranka_1_1RobotState.html
        initial_pose = robot_state.O_T_EE_c;
      }

      constexpr double kRadius = 0.3;
      double angle = M_PI / 4 * (1 - std::cos(M_PI / 5.0 * time)); // 光滑变化: 0 -> pi/2 -> 0
      double delta_x = kRadius * std::sin(angle);  // 0 -> 0.3 -> 0
      double delta_z = kRadius * (std::cos(angle) - 1);  // 0 -> -0.3 -> 0

      // Pose is represented as a 4x4 matrix in colMajor format.
      std::array<double, 16> new_pose = initial_pose;
      new_pose[12] += delta_x;
      new_pose[14] += delta_z;

      if (time >= 10.0) {
        std::cout << std::endl << "Finished motion, shutting down example" << std::endl;
        return franka::MotionFinished(new_pose);  // 发送运动结束信号
      }
      return new_pose;
    });
  } catch (const franka::Exception& e) {
    std::cout << e.what() << std::endl;
    return -1;
  }

  return 0;
}
