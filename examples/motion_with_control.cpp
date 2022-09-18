// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <vector>

#include <Poco/DateTimeFormatter.h>
#include <Poco/File.h>
#include <Poco/Path.h>

#include <franka/exception.h>
#include <franka/robot.h>

#include "examples_common.h"

/**
 * @example motion_with_control.cpp
 * An example showing how to use a joint velocity motion generator and torque control.
 * 展示 关节速度运动生成器 + 关节空间阻抗控制器. 在第 4 个关节控制速度按三角函数形式先加速, 再匀速, 再减速
 * Additionally, this example shows how to capture and write logs in case an exception is thrown
 * during a motion.
 * 此外, 该例程展示了如何在报错时获取并记录 logs
 *
 * @warning Before executing this example, make sure there is enough space in front of the robot.
 */

namespace {

// 节阻抗控制器, PD 控制, 微分项带滤波
class Controller {
 public:
  Controller(size_t dq_filter_size,
             const std::array<double, 7>& K_P,
             const std::array<double, 7>& K_D)
      : dq_current_filter_position_(0), dq_filter_size_(dq_filter_size), K_P_(K_P), K_D_(K_D) {
    std::fill(dq_d_.begin(), dq_d_.end(), 0);
    // dq 滤波器: 可以看成一维数组形式存储的二维循环数组 [dq_filter_size_, 7], 保存连续 dq_filter_size_ 个周期的 dq
    // 过滤后的 dq_filtered  为 dq_filter_size_ 个 dq 的平均值
    dq_buffer_ = std::make_unique<double[]>(dq_filter_size_ * 7);  // 动态数组 double[dq_filter_size_ * 7]
    std::fill(&dq_buffer_.get()[0], &dq_buffer_.get()[dq_filter_size_ * 7], 0); // &(unique_ptr.get()[i])
  }

  inline franka::Torques step(const franka::RobotState& state) {
    updateDQFilter(state);

    std::array<double, 7> tau_J_d;  // 期望输出力矩
    for (size_t i = 0; i < 7; i++) {
      tau_J_d[i] = K_P_[i] * (state.q_d[i] - state.q[i]) + K_D_[i] * (dq_d_[i] - getDQFiltered(i));  // dq_d_[i]==0
    }
    return tau_J_d;
  }

  void updateDQFilter(const franka::RobotState& state) {  // 更新 dq filter, 将最早的 dp 覆盖
    for (size_t i = 0; i < 7; i++) {
      dq_buffer_.get()[dq_current_filter_position_ * 7 + i] = state.dq[i];
    }
    dq_current_filter_position_ = (dq_current_filter_position_ + 1) % dq_filter_size_;
  }

  double getDQFiltered(size_t index) const {  // 返回第 index 个关节滤波后的 dq
    double value = 0;
    for (size_t i = index; i < 7 * dq_filter_size_; i += 7) {
      value += dq_buffer_.get()[i];
    }
    return value / dq_filter_size_;
  }

 private:
  size_t dq_current_filter_position_;
  size_t dq_filter_size_;

  const std::array<double, 7> K_P_;
  const std::array<double, 7> K_D_;

  std::array<double, 7> dq_d_;
  std::unique_ptr<double[]> dq_buffer_;
};


// 以三角函数生成具有光滑速度和加速度曲线的速度序列, 保存在一个数组中
std::vector<double> generateTrajectory(double a_max) {
  std::vector<double> trajectory;
  constexpr double kTimeStep = 0.001;          // [s]
  constexpr double kAccelerationTime = 1;      // time spend accelerating and decelerating [s]
  constexpr double kConstantVelocityTime = 1;  // time spend with constant speed [s]
  // obtained during the speed up and slow down [rad/s^2]
  double a = 0;  // [rad/s^2]
  double v = 0;  // [rad/s]
  double t = 0;  // [s]
  while (t < (2 * kAccelerationTime + kConstantVelocityTime)) {
    if (t <= kAccelerationTime) { // 加速
      a = pow(sin(t * M_PI / kAccelerationTime), 2) * a_max;  // sin(x)^2 = 1/2 * (1-cos(2*x)), 0->a_max->0
    } else if (t <= (kAccelerationTime + kConstantVelocityTime)) {
      a = 0;  // 匀速
    } else {  // 减速
      const double deceleration_time =
          (kAccelerationTime + kConstantVelocityTime) - t;  // time spent in the deceleration phase
      a = -pow(sin(deceleration_time * M_PI / kAccelerationTime), 2) * a_max;  // 0->-a_max->0
    }
    v += a * kTimeStep;
    t += kTimeStep;
    trajectory.push_back(v);
  }
  return trajectory;
}

}  // anonymous namespace

void writeLogToFile(const std::vector<franka::Record>& log);


int main(int argc, char** argv) {
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " <robot-hostname>" << std::endl;
    return -1;
  }

  // Parameters
  const size_t joint_number{3};
  const size_t filter_size{5};

  const std::array<double, 7> K_P{{200.0, 200.0, 200.0, 200.0, 200.0, 200.0, 200.0}};
  const std::array<double, 7> K_D{{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}};
  const double max_acceleration{1.0};

  Controller controller(filter_size, K_P, K_D);

  try {
    franka::Robot robot(argv[1]);
    setDefaultBehavior(robot);

    // 将机器人复位
    std::array<double, 7> q_goal = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
    MotionGenerator motion_generator(0.5, q_goal);
    std::cout << "WARNING: This example will move the robot! "
              << "Please make sure to have the user stop button at hand!" << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();
    robot.control(motion_generator);
    std::cout << "Finished moving to initial joint configuration." << std::endl;

    // Set additional parameters always before the control loop, NEVER in the control loop!
    // Set collision behavior.
    robot.setCollisionBehavior(
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});

    size_t index = 0;
    std::vector<double> trajectory = generateTrajectory(max_acceleration);

    robot.control(
        [&](const franka::RobotState& robot_state, franka::Duration) -> franka::Torques {  // 关节空间阻抗控制器
          return controller.step(robot_state);
        },
        [&](const franka::RobotState&, franka::Duration period) -> franka::JointVelocities {  // 关节速度运动生成器
          index += period.toMSec();

          if (index >= trajectory.size()) {
            index = trajectory.size() - 1;
          }

          franka::JointVelocities velocities{{0, 0, 0, 0, 0, 0, 0}};
          velocities.dq[joint_number] = trajectory[index];  // 设置第 4 个关节的速度

          if (index >= trajectory.size() - 1) {
            return franka::MotionFinished(velocities);
          }
          return velocities;
        });
  } catch (const franka::ControlException& e) {
    std::cout << e.what() << std::endl;
    writeLogToFile(e.log);
    return -1;
  } catch (const franka::Exception& e) {
    std::cout << e.what() << std::endl;
    return -1;
  }

  return 0;
}


// 记录 log
// POCO 参考: https://docs.pocoproject.org/current/package-Foundation.Filesystem.html
void writeLogToFile(const std::vector<franka::Record>& log) {
  if (log.empty()) {
    return;
  }
  try {
    Poco::Path temp_dir_path(Poco::Path::temp());
    temp_dir_path.pushDirectory("libfranka-logs");

    Poco::File temp_dir(temp_dir_path);
    temp_dir.createDirectories();

    std::string now_string =
        Poco::DateTimeFormatter::format(Poco::Timestamp{}, "%Y-%m-%d-%h-%m-%S-%i");
    std::string filename = std::string{"log-" + now_string + ".csv"};
    Poco::File log_file(Poco::Path(temp_dir_path, filename));
    if (!log_file.createFile()) {
      std::cout << "Failed to write log file." << std::endl;
      return;
    }
    std::ofstream log_stream(log_file.path().c_str());
    log_stream << franka::logToCSV(log);

    std::cout << "Log file written to: " << log_file.path() << std::endl;
  } catch (...) {
    std::cout << "Failed to write log file." << std::endl;
  }
}
