/*
 * @Author: ei_code_bash && 3080152159@qq.com
 * @Date: 2026-04-16 20:55:00
 * @LastEditors: ei_code_bash && 3080152159@qq.com
 * @LastEditTime: 2026-04-16 19:08:57
 * @FilePath: /ne_nav/src/ne_esdf_builder/src/esdf_visualizer_node.cpp
 * @Description: ESDF visualization node for RViz
 *
 * Copyright (c) 2026 by ei_code_bash, All Rights Reserved.
 */
#include "ne_esdf_builder/esdf_visualizer_node.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <utility>

#include "geometry_msgs/msg/point.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include "visualization_msgs/msg/marker.hpp"

namespace esdf_builder
{

ESDFVisualizer::ESDFVisualizer(const rclcpp::NodeOptions & options)
: rclcpp::Node("esdf_visualizer_node", options)
{
    init();
    RCLCPP_INFO(this->get_logger(), "esdf visualizer node has been built");
}

void ESDFVisualizer::init()
{
    input_topic_ = this->declare_parameter<std::string>("input_topic", "/esdf_map");
    output_topic_ = this->declare_parameter<std::string>("output_topic", "/esdf_markers");
    danger_distance_ = this->declare_parameter<double>("danger_distance", 0.2);
    warn_distance_ = this->declare_parameter<double>("warn_distance", 0.4);
    marker_alpha_ = this->declare_parameter<double>("marker_alpha", 0.85);
    marker_height_scale_ = this->declare_parameter<double>("marker_height_scale", 0.2);
    rainbow_max_distance_ = this->declare_parameter<double>("rainbow_max_distance", 5.0);

    if (warn_distance_ < danger_distance_) {
        warn_distance_ = danger_distance_;
    }
    if (rainbow_max_distance_ <= 0.0) {
        rainbow_max_distance_ = std::max(warn_distance_ * 3.0, 0.1);
    }

    const auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();

    esdf_sub_ = this->create_subscription<nav_interfaces::msg::Esdf>(
        input_topic_,
        qos,
        std::bind(&ESDFVisualizer::esdfCallback, this, std::placeholders::_1));

    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        output_topic_,
        qos);
}

std_msgs::msg::ColorRGBA ESDFVisualizer::makeRainbowColor(float distance) const
{
    const float clipped_distance = std::max(0.0F, distance);
    const float max_distance = static_cast<float>(std::max(rainbow_max_distance_, 0.1));
    const float t = std::clamp(clipped_distance / max_distance, 0.0F, 1.0F);

    std_msgs::msg::ColorRGBA color;
    constexpr float kStops[7][3] = {
        {0.38F, 0.16F, 0.78F},
        {0.93F, 0.39F, 0.71F},
        {0.96F, 0.52F, 0.22F},
        {0.98F, 0.86F, 0.24F},
        {0.38F, 0.78F, 0.31F},
        {0.20F, 0.78F, 0.82F},
        {0.17F, 0.36F, 0.86F},
    };
    const float scaled = t * 6.0F;
    const int idx = static_cast<int>(std::floor(scaled));
    const int next_idx = std::min(idx + 1, 6);
    const float local_t = std::clamp(scaled - static_cast<float>(idx), 0.0F, 1.0F);
    const int base_idx = std::clamp(idx, 0, 6);

    color.r = kStops[base_idx][0] + (kStops[next_idx][0] - kStops[base_idx][0]) * local_t;
    color.g = kStops[base_idx][1] + (kStops[next_idx][1] - kStops[base_idx][1]) * local_t;
    color.b = kStops[base_idx][2] + (kStops[next_idx][2] - kStops[base_idx][2]) * local_t;
    color.a = static_cast<float>(marker_alpha_);
    return color;
}

void ESDFVisualizer::esdfCallback(const nav_interfaces::msg::Esdf::SharedPtr msg)
{
    const int width = static_cast<int>(msg->info.width);
    const int height = static_cast<int>(msg->info.height);
    if (width <= 0 || height <= 0) {
        return;
    }

    if (msg->data.size() != static_cast<std::size_t>(width * height)) {
        RCLCPP_WARN(this->get_logger(), "esdf data size does not match map meta data");
        return;
    }

    visualization_msgs::msg::MarkerArray marker_array;
    visualization_msgs::msg::Marker marker;
    marker.header = msg->header;
    marker.ns = "rainbow_esdf";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::CUBE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = msg->info.resolution;
    marker.scale.y = msg->info.resolution;
    marker.scale.z = std::max(
        static_cast<double>(msg->info.resolution) * marker_height_scale_,
        0.02);

    const double origin_x = msg->info.origin.position.x;
    const double origin_y = msg->info.origin.position.y;
    const double origin_z = msg->info.origin.position.z;
    const double resolution = msg->info.resolution;

    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            const int id = y * width + x;
            const float distance = msg->data[static_cast<std::size_t>(id)];

            geometry_msgs::msg::Point point;
            point.x = origin_x + (static_cast<double>(x) + 0.5) * resolution;
            point.y = origin_y + (static_cast<double>(y) + 0.5) * resolution;
            point.z = origin_z;

            marker.points.push_back(point);
            marker.colors.push_back(makeRainbowColor(distance));
        }
    }

    marker_array.markers.push_back(std::move(marker));
    marker_pub_->publish(marker_array);
}

}  // namespace esdf_builder

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<esdf_builder::ESDFVisualizer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
