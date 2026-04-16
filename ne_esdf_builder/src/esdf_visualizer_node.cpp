/*
 * @Author: ei_code_bash && 3080152159@qq.com
 * @Date: 2026-04-16 20:55:00
 * @LastEditors: ei_code_bash && 3080152159@qq.com
 * @LastEditTime: 2026-04-16 20:55:00
 * @FilePath: /ne_nav/src/ne_esdf_builder/src/esdf_visualizer_node.cpp
 * @Description: ESDF visualization node for RViz
 *
 * Copyright (c) 2026 by ei_code_bash, All Rights Reserved.
 */
#include "ne_esdf_builder/esdf_visualizer_node.hpp"

#include <algorithm>
#include <memory>
#include <utility>

#include "geometry_msgs/msg/point.hpp"
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

    if (warn_distance_ < danger_distance_) {
        warn_distance_ = danger_distance_;
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

    auto make_marker = [&](int id, const std::string & ns, float r, float g, float b) {
        visualization_msgs::msg::Marker marker;
        marker.header = msg->header;
        marker.ns = ns;
        marker.id = id;
        marker.type = visualization_msgs::msg::Marker::CUBE_LIST;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = msg->info.resolution;
        marker.scale.y = msg->info.resolution;
        marker.scale.z = std::max(
            static_cast<double>(msg->info.resolution) * marker_height_scale_,
            0.02);
        marker.color.r = r;
        marker.color.g = g;
        marker.color.b = b;
        marker.color.a = static_cast<float>(marker_alpha_);
        return marker;
    };

    auto red_marker = make_marker(0, "danger", 1.0F, 0.0F, 0.0F);
    auto yellow_marker = make_marker(1, "warning", 1.0F, 1.0F, 0.0F);
    auto green_marker = make_marker(2, "safe", 0.0F, 1.0F, 0.0F);

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

            if (distance <= 0.0F || distance <= static_cast<float>(danger_distance_)) {
                red_marker.points.push_back(point);
            } else if (distance <= static_cast<float>(warn_distance_)) {
                yellow_marker.points.push_back(point);
            } else {
                green_marker.points.push_back(point);
            }
        }
    }

    marker_array.markers.push_back(std::move(red_marker));
    marker_array.markers.push_back(std::move(yellow_marker));
    marker_array.markers.push_back(std::move(green_marker));
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
