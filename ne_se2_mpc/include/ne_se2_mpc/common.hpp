/*
 * @Author: ei_code_bash && 3080152159@qq.com
 * @Date: 2026-04-28 13:20:10
 * @LastEditors: ei_code_bash && 3080152159@qq.com
 * @LastEditTime: 2026-04-28 15:21:53
 * @FilePath: /ne_nav/src/ne_se2_mpc/include/ne_se2_mpc/common.hpp
 * @Description: 我永远喜欢雪之下雪乃
 * 
 * Copyright (c) 2026 by ei_code_bash, All Rights Reserved. 
 */
#ifndef COMMON_HPP
#define COMMON_HPP

#include <cmath>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"

namespace mpc_state
{
using state = Eigen::Matrix<double,6,1>;
using state_control = Eigen::Matrix<double,3,1>;
using MatrixXd = Eigen::MatrixXd;
using ref_sequence = std::vector<state>;
using control_sequence = std::vector<state_control>;

struct odom_measure
{
  rclcpp::Time odom_stamp;
  nav_msgs::msg::Odometry odom_msg;
};


// rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

}  // namespace mpc_state
#endif
