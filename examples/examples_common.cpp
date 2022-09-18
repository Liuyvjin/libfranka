// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include "examples_common.h"

#include <algorithm>
#include <array>
#include <cmath>

#include <franka/exception.h>
#include <franka/robot.h>

// 设置默认碰撞, 阻抗参数
void setDefaultBehavior(franka::Robot& robot) {
  robot.setCollisionBehavior(
      {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}}, {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
      {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}}, {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}},
      {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}}, {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
      {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0}}, {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0}});
  robot.setJointImpedance({{3000, 3000, 3000, 2500, 2500, 2000, 2000}});
  robot.setCartesianImpedance({{3000, 3000, 3000, 300, 300, 300}});
}

/**
 * @brief Construct a new Motion Generator:: Motion Generator object
 *
 * @param speed_factor 速度缩放因子 0 ~ 1
 * @param q_goal 目标关节位置
 */
MotionGenerator::MotionGenerator(double speed_factor, const std::array<double, 7> q_goal)
    : q_goal_(q_goal.data()) {
  dq_max_ *= speed_factor;
  ddq_max_start_ *= speed_factor;
  ddq_max_goal_ *= speed_factor;
  dq_max_sync_.setZero();
  q_start_.setZero();
  delta_q_.setZero();
  t_1_sync_.setZero();
  t_2_sync_.setZero();
  t_f_sync_.setZero();
  q_1_.setZero();
}

/**
 * @brief 插值计算 delta_q_d = f(t), 最终命令输出为  q_start_ + delta_q_d
 *        f(0)=0, f(t_f_sync_)=q_goal_-q_start_
 *
 * @param t
 * @param delta_q_d
 * @return true t==t_f_sync_ 时返回 true
 * @return false
 */
bool MotionGenerator::calculateDesiredValues(double t, Vector7d* delta_q_d) const {
  Vector7i sign_delta_q;
  sign_delta_q << delta_q_.cwiseSign().cast<int>();
  Vector7d t_d = t_2_sync_ - t_1_sync_;
  Vector7d delta_t_2_sync = t_f_sync_ - t_2_sync_;
  std::array<bool, 7> joint_motion_finished{};

  for (size_t i = 0; i < 7; i++) {
    if (std::abs(delta_q_[i]) < kDeltaQMotionFinished) {
      (*delta_q_d)[i] = 0;
      joint_motion_finished[i] = true;
    } else {
      if (t < t_1_sync_[i]) {  // 加速阶段
        (*delta_q_d)[i] = -1.0 / std::pow(t_1_sync_[i], 3.0) * dq_max_sync_[i] * sign_delta_q[i] *
                          (0.5 * t - t_1_sync_[i]) * std::pow(t, 3.0);
      } else if (t >= t_1_sync_[i] && t < t_2_sync_[i]) {  // 匀速阶段
        (*delta_q_d)[i] = q_1_[i] + (t - t_1_sync_[i]) * dq_max_sync_[i] * sign_delta_q[i];
      } else if (t >= t_2_sync_[i] && t < t_f_sync_[i]) {  // 减速阶段
        (*delta_q_d)[i] =
            delta_q_[i] + 0.5 *
                              (1.0 / std::pow(delta_t_2_sync[i], 3.0) *
                                   (t - t_1_sync_[i] - 2.0 * delta_t_2_sync[i] - t_d[i]) *
                                   std::pow((t - t_1_sync_[i] - t_d[i]), 3.0) +
                               (2.0 * t - 2.0 * t_1_sync_[i] - delta_t_2_sync[i] - 2.0 * t_d[i])) *
                              dq_max_sync_[i] * sign_delta_q[i];
      } else {  // 终止阶段
        (*delta_q_d)[i] = delta_q_[i];
        joint_motion_finished[i] = true;
      }
    }
  }
  return std::all_of(joint_motion_finished.cbegin(), joint_motion_finished.cend(), // 若所有关节都运动完成, 返回 true
                     [](bool x) { return x; });
}

/**
 * @brief 同步各个关节的运动时长
 * 参考: Wisama Khalil and Etienne Dombre. 2002. Modeling, Identification and Control of Robots 13.3.3-13.3.4
 * 与书中的区别是, 这里考虑了加速和减速阶段加速度绝对值不同的情况, 并且同步方法不是缩小加速度, 而是压扁速度梯形曲线
 */
void MotionGenerator::calculateSynchronizedValues() {
  // 各个关节按照 continuous trapeze velocity profile 能达到的最大速度
  Vector7d dq_max_reach(dq_max_);
  // 各个关节按照 continuous trapeze velocity profile 运动所需的总时长
  Vector7d t_f = Vector7d::Zero();
  Vector7d delta_t_2 = Vector7d::Zero();
  Vector7d t_1 = Vector7d::Zero();
  Vector7d delta_t_2_sync = Vector7d::Zero();
  Vector7i sign_delta_q;  // 关节旋转方向
  sign_delta_q << delta_q_.cwiseSign().cast<int>();

  for (size_t i = 0; i < 7; i++) {
    if (std::abs(delta_q_[i]) > kDeltaQMotionFinished) { // 若 delta_q 超过运动完成的阈值 1e-6
      // 若按照最大加速度无法达到额定最大速度即可走完全程, 更新最大速度为能达到的最大速度
      if (std::abs(delta_q_[i]) < (3.0 / 4.0 * (std::pow(dq_max_[i], 2.0) / ddq_max_start_[i]) +
                                   3.0 / 4.0 * (std::pow(dq_max_[i], 2.0) / ddq_max_goal_[i]))) {
        dq_max_reach[i] = std::sqrt(4.0 / 3.0 * delta_q_[i] * sign_delta_q[i] *
                                    (ddq_max_start_[i] * ddq_max_goal_[i]) /
                                    (ddq_max_start_[i] + ddq_max_goal_[i]));
      }
      // 二次函数加速阶段时长 tau_1
      t_1[i] = 1.5 * dq_max_reach[i] / ddq_max_start_[i];
      // 二次函数减速阶段时长 tau_2
      delta_t_2[i] = 1.5 * dq_max_reach[i] / ddq_max_goal_[i];
      // 总时长
      t_f[i] = t_1[i] / 2.0 + delta_t_2[i] / 2.0 + std::abs(delta_q_[i]) / dq_max_reach[i];
    }
  }
  double max_t_f = t_f.maxCoeff();  // 所有关节中的最大时长, 其他关节需要与此同步
  // 将所有关节的运动时长同步为 max_t_f
  // 压低其他关节的最大速度, 使得梯形变得更加矮胖, t_f 增大至 max_t_f
  for (size_t i = 0; i < 7; i++) {
    if (std::abs(delta_q_[i]) > kDeltaQMotionFinished) {
      double a = 1.5 / 2.0 * (ddq_max_goal_[i] + ddq_max_start_[i]);
      double b = -1.0 * max_t_f * ddq_max_goal_[i] * ddq_max_start_[i];
      double c = std::abs(delta_q_[i]) * ddq_max_goal_[i] * ddq_max_start_[i];
      double delta = b * b - 4.0 * a * c;
      if (delta < 0.0) {
        delta = 0.0;
      }
      dq_max_sync_[i] = (-1.0 * b - std::sqrt(delta)) / (2.0 * a);  // 同步后的最大速度
      t_1_sync_[i] = 1.5 * dq_max_sync_[i] / ddq_max_start_[i];  // 同步后的 t1 = tau1
      delta_t_2_sync[i] = 1.5 * dq_max_sync_[i] / ddq_max_goal_[i];  // 同步后的 tau2
      t_f_sync_[i] =
          (t_1_sync_)[i] / 2.0 + delta_t_2_sync[i] / 2.0 + std::abs(delta_q_[i] / dq_max_sync_[i]);  // 有解时 == max_t_f
      t_2_sync_[i] = (t_f_sync_)[i] - delta_t_2_sync[i];  // t2 = t_f - tau2
      q_1_[i] = (dq_max_sync_)[i] * sign_delta_q[i] * (0.5 * (t_1_sync_)[i]);  // 加速阶段的运动量
    }
  }
}

franka::JointPositions MotionGenerator::operator()(const franka::RobotState& robot_state,
                                                   franka::Duration period) {
  time_ += period.toSec();  // 运动过程经历时长 t

  if (time_ == 0.0) {
    q_start_ = Vector7d(robot_state.q_d.data());  // 上一个时刻的 q_d 作为关节初始状态
    delta_q_ = q_goal_ - q_start_;
    calculateSynchronizedValues();
  }

  Vector7d delta_q_d;
  bool motion_finished = calculateDesiredValues(time_, &delta_q_d);  // 更新 delta_q_d

  std::array<double, 7> joint_positions;
  Eigen::VectorXd::Map(&joint_positions[0], 7) = (q_start_ + delta_q_d);
  franka::JointPositions output(joint_positions);
  output.motion_finished = motion_finished;
  return output;
}
