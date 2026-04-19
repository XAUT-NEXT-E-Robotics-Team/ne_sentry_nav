/*
 * @Author: ei_code_bash && 3080152159@qq.com
 * @Date: 2026-04-16 20:55:00
 * @LastEditors: ei_code_bash && 3080152159@qq.com
 * @LastEditTime: 2026-04-16 20:55:00
 * @FilePath: /ne_nav/src/ne_esdf_builder/include/ne_esdf_builder/esdf_visualizer_node.hpp
 * @Description: ESDF visualization node for RViz
 *
 * Copyright (c) 2026 by ei_code_bash, All Rights Reserved.
 */
#ifndef ESDF_VISUALIZER_NODE_HPP
#define ESDF_VISUALIZER_NODE_HPP

#include <string>

#include "nav_interfaces/msg/esdf.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace esdf_builder
{

class ESDFVisualizer : public rclcpp::Node
{
public:
    explicit ESDFVisualizer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
    ~ESDFVisualizer() override = default;

private:
    void init();
    void esdfCallback(const nav_interfaces::msg::Esdf::SharedPtr msg);
    std_msgs::msg::ColorRGBA makeRainbowColor(float distance) const;

    rclcpp::Subscription<nav_interfaces::msg::Esdf>::SharedPtr esdf_sub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

    std::string input_topic_{"/esdf_map"};
    std::string output_topic_{"/esdf_markers"};
    double danger_distance_{0.2};
    double warn_distance_{0.6};
    double marker_alpha_{0.85};
    double marker_height_scale_{0.2};
    double rainbow_max_distance_{0.6};
};

}  // namespace esdf_builder

#endif
