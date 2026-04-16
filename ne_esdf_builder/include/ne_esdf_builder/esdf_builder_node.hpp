/*
 * @Author: ei_code_bash && 3080152159@qq.com
 * @Date: 2026-04-16 12:39:04
 * @LastEditors: ei_code_bash && 3080152159@qq.com
 * @LastEditTime: 2026-04-16 17:40:59
 * @FilePath: /ne_nav/src/ne_esdf_builder/include/ne_esdf_builder/esdf_builder_node.hpp
 * @Description: 我永远喜欢雪之下雪乃
 * 
 * Copyright (c) 2026 by ei_code_bash, All Rights Reserved. 
 */
#ifndef ESDF_BUILDER_HPP
#define ESDF_BUILDER_HPP

#include <cstdint>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_interfaces/msg/esdf.hpp"

namespace esdf_builder
{
    class ESDF:public rclcpp::Node
    {
        public:
        explicit ESDF(const rclcpp::NodeOptions& option = rclcpp::NodeOptions());
        ~ESDF() = default;
        void init();
        void grid_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);

        private:
        rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr terrain_analysis_sub_;
        rclcpp::Publisher<nav_interfaces::msg::Esdf>::SharedPtr esdf_map_pub_;
        std::string input_topic_{"/terrain_analysis/grid"};
        std::string output_topic_{"/esdf_map"};
        int8_t obs_thresh_{50};
        bool treat_unknown_as_obstacle_{true};
    };
}
#endif
