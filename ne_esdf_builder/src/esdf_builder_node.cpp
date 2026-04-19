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

#include <cmath>
#include <limits>
#include <vector>

namespace esdf_builder
{

namespace
{

constexpr float kInfDistance = std::numeric_limits<float>::infinity();

inline std::size_t to_address(int x, int y, int cols)
{
    return static_cast<std::size_t>(x) * static_cast<std::size_t>(cols) +
           static_cast<std::size_t>(y);
}

template<typename FGetVal, typename FSetVal>
void fill_esdf(FGetVal f_get_val, FSetVal f_set_val, int start, int end, int size)
{
    std::vector<int> v(static_cast<std::size_t>(size));
    std::vector<double> z(static_cast<std::size_t>(size + 1));

    int k = start;
    v[static_cast<std::size_t>(start)] = start;
    z[static_cast<std::size_t>(start)] = -std::numeric_limits<double>::max();
    z[static_cast<std::size_t>(start + 1)] = std::numeric_limits<double>::max();

    for (int q = start + 1; q <= end; ++q) {
        ++k;
        double s = 0.0;
        do {
            --k;
            s = ((f_get_val(q) + static_cast<double>(q * q)) -
                 (f_get_val(v[static_cast<std::size_t>(k)]) +
                  static_cast<double>(v[static_cast<std::size_t>(k)] * v[static_cast<std::size_t>(k)]))) /
                static_cast<double>(2 * q - 2 * v[static_cast<std::size_t>(k)]);
        } while (s <= z[static_cast<std::size_t>(k)]);

        ++k;
        v[static_cast<std::size_t>(k)] = q;
        z[static_cast<std::size_t>(k)] = s;
        z[static_cast<std::size_t>(k + 1)] = std::numeric_limits<double>::max();
    }

    k = start;
    for (int q = start; q <= end; ++q) {
        while (z[static_cast<std::size_t>(k + 1)] < static_cast<double>(q)) {
            ++k;
        }
        const double val = static_cast<double>((q - v[static_cast<std::size_t>(k)]) *
                                               (q - v[static_cast<std::size_t>(k)])) +
                           f_get_val(v[static_cast<std::size_t>(k)]);
        f_set_val(q, val);
    }
}

std::vector<float> compute_signed_esdf(
    const std::vector<uint8_t> &occupancy_map,
    int width,
    int height,
    float resolution)
{
    const int rows = width;
    const int cols = height;
    const std::size_t cell_count = static_cast<std::size_t>(rows) * static_cast<std::size_t>(cols);
    const double inf = std::numeric_limits<double>::max();

    std::vector<double> tmp_buffer(cell_count, inf);
    std::vector<double> pos_buffer(cell_count, inf);
    std::vector<double> neg_buffer(cell_count, inf);
    std::vector<uint8_t> free_map(cell_count, 0U);
    std::vector<float> esdf_buffer(cell_count, kInfDistance);

    for (std::size_t i = 0; i < cell_count; ++i) {
        free_map[i] = occupancy_map[i] == 0U ? 1U : 0U;
    }

    for (int x = 0; x < rows; ++x) {
        fill_esdf(
            [&](int y) {
                return occupancy_map[to_address(x, y, cols)] != 0U ? 0.0 : inf;
            },
            [&](int y, double val) {
                tmp_buffer[to_address(x, y, cols)] = val;
            },
            0, cols - 1, cols);
    }

    for (int y = 0; y < cols; ++y) {
        fill_esdf(
            [&](int x) {
                return tmp_buffer[to_address(x, y, cols)];
            },
            [&](int x, double val) {
                pos_buffer[to_address(x, y, cols)] =
                    static_cast<double>(resolution) * std::sqrt(val);
            },
            0, rows - 1, rows);
    }

    for (int x = 0; x < rows; ++x) {
        fill_esdf(
            [&](int y) {
                return free_map[to_address(x, y, cols)] != 0U ? 0.0 : inf;
            },
            [&](int y, double val) {
                tmp_buffer[to_address(x, y, cols)] = val;
            },
            0, cols - 1, cols);
    }

    for (int y = 0; y < cols; ++y) {
        fill_esdf(
            [&](int x) {
                return tmp_buffer[to_address(x, y, cols)];
            },
            [&](int x, double val) {
                neg_buffer[to_address(x, y, cols)] =
                    static_cast<double>(resolution) * std::sqrt(val);
            },
            0, rows - 1, rows);
    }

    for (int x = 0; x < rows; ++x) {
        for (int y = 0; y < cols; ++y) {
            const std::size_t id = to_address(x, y, cols);
            const double pos_dist = pos_buffer[id];
            const double neg_dist = std::abs(neg_buffer[id]);

            if (neg_dist > 0.0) {
                esdf_buffer[id] = static_cast<float>(-neg_dist + static_cast<double>(resolution));
            } else {
                esdf_buffer[id] = static_cast<float>(pos_dist);
            }
        }
    }

    return esdf_buffer;
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

    std::vector<uint8_t> occupancy_map(cell_count, 0U);

    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            const std::size_t ros_id =
                static_cast<std::size_t>(y) * static_cast<std::size_t>(width) +
                static_cast<std::size_t>(x);
            const std::size_t esdf_id = to_address(x, y, height);
            const int8_t value = msg->data[ros_id];

            if (value < 0) {
                occupancy_map[esdf_id] = treat_unknown_as_obstacle_ ? 1U : 0U;
                continue;
            }

            const bool occupied = value >= obs_thresh_;
            occupancy_map[esdf_id] = occupied ? 1U : 0U;
        }
    }

    const auto esdf_buffer = compute_signed_esdf(occupancy_map, width, height, resolution);

    nav_interfaces::msg::Esdf esdf_msg;
    esdf_msg.header = msg->header;
    esdf_msg.info = msg->info;
    esdf_msg.data.resize(cell_count, 0.0F);

    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            const std::size_t ros_id =
                static_cast<std::size_t>(y) * static_cast<std::size_t>(width) +
                static_cast<std::size_t>(x);
            esdf_msg.data[ros_id] = esdf_buffer[to_address(x, y, height)];
        }
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
