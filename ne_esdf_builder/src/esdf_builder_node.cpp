/*
 * @Author: ei_code_bash && 3080152159@qq.com
 * @Date: 2026-04-16 15:51:13
 * @LastEditors: ei_code_bash && 3080152159@qq.com
 * @LastEditTime: 2026-04-16 17:43:42
 * @FilePath: /ne_nav/src/ne_esdf_builder/src/esdf_builder_node.cpp
 * @Description: 我永远喜欢雪之下雪乃
 * 
 * Copyright (c) 2026 by ei_code_bash, All Rights Reserved. 
 */
#include "ne_esdf_builder/esdf_builder_node.hpp"

#include <array>
#include <cmath>
#include <limits>
#include <queue>
#include <utility>
#include <vector>

namespace esdf_builder
{

namespace
{

constexpr float kInfDistance = std::numeric_limits<float>::infinity();

std::vector<float> compute_distance_field(
    const std::vector<uint8_t> &source_mask,
    int width,
    int height)
{
    using QueueItem = std::pair<float, int>;
    std::priority_queue<QueueItem, std::vector<QueueItem>, std::greater<QueueItem>> queue;

    std::vector<float> distances(
        static_cast<std::size_t>(width) * static_cast<std::size_t>(height),
        kInfDistance);

    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            const int id = y * width + x;
            if (source_mask[static_cast<std::size_t>(id)] == 0U) {
                continue;
            }
            distances[static_cast<std::size_t>(id)] = 0.0F;
            queue.emplace(0.0F, id);
        }
    }

    constexpr std::array<int, 8> kDx{{-1, 0, 1, -1, 1, -1, 0, 1}};
    constexpr std::array<int, 8> kDy{{-1, -1, -1, 0, 0, 1, 1, 1}};
    constexpr std::array<float, 8> kStepCost{
        {1.41421356F, 1.0F, 1.41421356F, 1.0F, 1.0F, 1.41421356F, 1.0F, 1.41421356F}};

    while (!queue.empty()) {
        const auto [current_distance, id] = queue.top();
        queue.pop();

        if (current_distance > distances[static_cast<std::size_t>(id)]) {
            continue;
        }

        const int x = id % width;
        const int y = id / width;
        for (std::size_t i = 0; i < kDx.size(); ++i) {
            const int nx = x + kDx[i];
            const int ny = y + kDy[i];
            if (nx < 0 || nx >= width || ny < 0 || ny >= height) {
                continue;
            }

            const int nid = ny * width + nx;
            const float candidate = current_distance + kStepCost[i];
            if (candidate >= distances[static_cast<std::size_t>(nid)]) {
                continue;
            }

            distances[static_cast<std::size_t>(nid)] = candidate;
            queue.emplace(candidate, nid);
        }
    }

    return distances;
}

}  // namespace

ESDF::ESDF(const rclcpp::NodeOptions& option):rclcpp::Node("esdf_builder_node",option)
{
    RCLCPP_INFO(this->get_logger(),"start build esdf map");
    init();
    RCLCPP_INFO(this->get_logger(),"esdf builder node is ready");
}

void ESDF::init()
{
    input_topic_ = this->declare_parameter<std::string>("input_topic", input_topic_);
    output_topic_ = this->declare_parameter<std::string>("output_topic", output_topic_);
    obs_thresh_ = static_cast<int8_t>(this->declare_parameter<int>("obs_thresh", obs_thresh_));
    treat_unknown_as_obstacle_ = this->declare_parameter<bool>(
        "treat_unknown_as_obstacle",
        treat_unknown_as_obstacle_);

    const auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();

    terrain_analysis_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        input_topic_,
        qos,
        std::bind(&ESDF::grid_callback, this, std::placeholders::_1));

    esdf_map_pub_ = this->create_publisher<nav_interfaces::msg::Esdf>(
        output_topic_,
        qos);
}

void ESDF::grid_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
    const int width = static_cast<int>(msg->info.width);
    const int height = static_cast<int>(msg->info.height);
    const float resolution = msg->info.resolution;

    if (width <= 0 || height <= 0 || resolution <= 0.0F) {
        RCLCPP_WARN(this->get_logger(), "invalid occupancy grid metadata");
        return;
    }

    const auto cell_count = static_cast<std::size_t>(width) * static_cast<std::size_t>(height);
    if (msg->data.size() != cell_count) {
        RCLCPP_WARN(this->get_logger(), "occupancy grid data size does not match metadata");
        return;
    }

    std::vector<uint8_t> obstacle_map(cell_count, 0U);
    std::vector<uint8_t> free_map(cell_count, 0U);

    for (std::size_t i = 0; i < cell_count; ++i) {
        const int8_t value = msg->data[i];
        if (value < 0) {
            obstacle_map[i] = treat_unknown_as_obstacle_ ? 1U : 0U;
            free_map[i] = treat_unknown_as_obstacle_ ? 0U : 1U;
            continue;
        }

        const bool occupied = value >= obs_thresh_;
        obstacle_map[i] = occupied ? 1U : 0U;
        free_map[i] = occupied ? 0U : 1U;
    }

    const auto distance_to_obstacle = compute_distance_field(obstacle_map, width, height);
    const auto distance_to_free = compute_distance_field(free_map, width, height);

    nav_interfaces::msg::Esdf esdf_msg;
    esdf_msg.header = msg->header;
    esdf_msg.info = msg->info;
    esdf_msg.data.resize(cell_count, 0.0F);

    for (std::size_t i = 0; i < cell_count; ++i) {
        const bool occupied = obstacle_map[i] != 0U;
        const float grid_distance = occupied ? distance_to_free[i] : distance_to_obstacle[i];
        const float metric_distance = std::isfinite(grid_distance) ? grid_distance * resolution
                                                                   : kInfDistance;
        esdf_msg.data[i] = occupied ? -metric_distance : metric_distance;
    }

    esdf_map_pub_->publish(esdf_msg);
}

}  // namespace esdf_builder

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<esdf_builder::ESDF>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
