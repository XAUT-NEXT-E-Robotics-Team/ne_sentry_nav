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

namespace esdf_builder
{

namespace
{

constexpr double kMaxDistance = std::numeric_limits<double>::max();

}  // namespace

void ESDFMap2D::reset(
    const nav_msgs::msg::MapMetaData & info,
    const std::vector<int8_t> & occupancy_data,
    int8_t obs_thresh,
    bool treat_unknown_as_obstacle)
{
    voxel_num_ << static_cast<int>(info.width), static_cast<int>(info.height);
    resolution_ = static_cast<double>(info.resolution);
    resolution_inv_ = resolution_ > 0.0 ? 1.0 / resolution_ : 0.0;
    map_origin_ << info.origin.position.x, info.origin.position.y;
    map_size_ << voxel_num_.x() * resolution_, voxel_num_.y() * resolution_;
    min_boundary_ = map_origin_;
    max_boundary_ = map_origin_ + map_size_;
    buffer_size_ = voxel_num_.x() * voxel_num_.y();

    occ_buffer_.assign(static_cast<std::size_t>(buffer_size_), 0);
    esdf_buffer_.assign(static_cast<std::size_t>(buffer_size_), kMaxDistance);

    for (int y = 0; y < voxel_num_.y(); ++y) {
        for (int x = 0; x < voxel_num_.x(); ++x) {
            const std::size_t ros_id =
                static_cast<std::size_t>(y) * static_cast<std::size_t>(voxel_num_.x()) +
                static_cast<std::size_t>(x);
            const int8_t value = occupancy_data[ros_id];

            bool occupied = false;
            if (value < 0) {
                occupied = treat_unknown_as_obstacle;
            } else {
                occupied = value >= obs_thresh;
            }

            occ_buffer_[static_cast<std::size_t>(toAddress(x, y))] = occupied ? 1 : 0;
        }
    }

    updateESDF();
}

void ESDFMap2D::updateESDF()
{
    const int rows = voxel_num_.x();
    const int cols = voxel_num_.y();
    const double inf = std::numeric_limits<double>::max();
    const std::size_t cell_count = static_cast<std::size_t>(buffer_size_);

    if (rows <= 0 || cols <= 0 || resolution_ <= 0.0) {
        return;
    }

    bool has_occupied = false;
    bool has_free = false;
    for (const char value : occ_buffer_) {
        has_occupied = has_occupied || value == 1;
        has_free = has_free || value == 0;
        if (has_occupied && has_free) {
            break;
        }
    }

    std::vector<double> tmp_buffer(cell_count, inf);
    std::vector<double> pos_buffer(cell_count, inf);
    std::vector<double> neg_buffer(cell_count, inf);
    RowMatrixXi neg_map(rows, cols);

    if (has_occupied) {
        for (int x = 0; x < rows; ++x) {
            fillESDF(
                [&](int y) {
                    return occ_buffer_[static_cast<std::size_t>(toAddress(Eigen::Vector2i(x, y)))] ?
                        0.0 : inf;
                },
                [&](int y, double val) {
                    tmp_buffer[static_cast<std::size_t>(toAddress(x, y))] = val;
                },
                0, cols - 1, cols);
        }

        for (int y = 0; y < cols; ++y) {
            fillESDF(
                [&](int x) {
                    return tmp_buffer[static_cast<std::size_t>(toAddress(x, y))];
                },
                [&](int x, double val) {
                    pos_buffer[static_cast<std::size_t>(toAddress(x, y))] = resolution_ * std::sqrt(val);
                },
                0, rows - 1, rows);
        }
    }

    for (int x = 0; x < rows; ++x) {
        for (int y = 0; y < cols; ++y) {
            neg_map(x, y) = (occ_buffer_[static_cast<std::size_t>(toAddress(Eigen::Vector2i(x, y)))] == 0) ? 1 : 0;
        }
    }

    if (has_free) {
        for (int x = 0; x < rows; ++x) {
            fillESDF(
                [&](int y) {
                    return neg_map(x, y) ? 0.0 : inf;
                },
                [&](int y, double val) {
                    tmp_buffer[static_cast<std::size_t>(toAddress(x, y))] = val;
                },
                0, cols - 1, cols);
        }

        for (int y = 0; y < cols; ++y) {
            fillESDF(
                [&](int x) {
                    return tmp_buffer[static_cast<std::size_t>(toAddress(x, y))];
                },
                [&](int x, double val) {
                    neg_buffer[static_cast<std::size_t>(toAddress(x, y))] = resolution_ * std::sqrt(val);
                },
                0, rows - 1, rows);
        }
    }

    for (int x = 0; x < rows; ++x) {
        for (int y = 0; y < cols; ++y) {
            const std::size_t id = static_cast<std::size_t>(toAddress(x, y));
            const double pos_dist = pos_buffer[id];
            const double neg_dist = std::abs(neg_buffer[id]);

            if (std::isfinite(neg_dist) && neg_dist > 0.0) {
                esdf_buffer_[id] = -neg_dist + resolution_;
            } else if (std::isfinite(pos_dist)) {
                esdf_buffer_[id] = pos_dist;
            } else if (occ_buffer_[id] == 1) {
                esdf_buffer_[id] = -kMaxDistance;
            } else {
                esdf_buffer_[id] = kMaxDistance;
            }
        }
    }
}

std::vector<float> ESDFMap2D::getRosMessageData() const
{
    std::vector<float> data(static_cast<std::size_t>(buffer_size_), 0.0F);
    for (int y = 0; y < voxel_num_.y(); ++y) {
        for (int x = 0; x < voxel_num_.x(); ++x) {
            const std::size_t ros_id =
                static_cast<std::size_t>(y) * static_cast<std::size_t>(voxel_num_.x()) +
                static_cast<std::size_t>(x);
            const double value = esdf_buffer_[static_cast<std::size_t>(toAddress(x, y))];
            data[ros_id] = std::isfinite(value) ?
                static_cast<float>(value) :
                (value < 0.0 ? -std::numeric_limits<float>::infinity() :
                               std::numeric_limits<float>::infinity());
        }
    }
    return data;
}

ESDF::ESDF(const rclcpp::NodeOptions & option) : rclcpp::Node("esdf_builder_node", option)
{
    RCLCPP_INFO(this->get_logger(), "start build esdf map");
    init();
    RCLCPP_INFO(this->get_logger(), "esdf builder node is ready");
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

    esdf_map_.reset(msg->info, msg->data, obs_thresh_, treat_unknown_as_obstacle_);

    nav_interfaces::msg::Esdf esdf_msg;
    esdf_msg.header = msg->header;
    esdf_msg.info = msg->info;
    esdf_msg.data = esdf_map_.getRosMessageData();

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
