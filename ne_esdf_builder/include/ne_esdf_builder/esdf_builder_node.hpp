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

#include <algorithm>
#include <cstdint>
#include <limits>
#include <vector>

#include <Eigen/Dense>

#include "nav_interfaces/msg/esdf.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"

namespace esdf_builder
{

using RowMatrixXd = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
using RowMatrixXi = Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;

class ESDFMap2D
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    ESDFMap2D() = default;

    void reset(
        const nav_msgs::msg::MapMetaData & info,
        const std::vector<int8_t> & occupancy_data,
        int8_t obs_thresh,
        bool treat_unknown_as_obstacle);

    double getDistance(const Eigen::Vector2d & pos) const
    {
        if (!isInMap(pos)) {
            return std::numeric_limits<double>::max();
        }

        Eigen::Vector2d pos_m = pos;
        pos_m(0) -= 0.5 * resolution_;
        pos_m(1) -= 0.5 * resolution_;

        Eigen::Vector2i idx;
        posToIndex(pos_m, idx);

        Eigen::Vector2d idx_pos;
        indexToPos(idx, idx_pos);

        Eigen::Vector2d diff = pos - idx_pos;
        diff(0) *= resolution_inv_;
        diff(1) *= resolution_inv_;

        double values[2][2];
        for (int x = 0; x < 2; ++x) {
            for (int y = 0; y < 2; ++y) {
                Eigen::Vector2i current_idx = idx + Eigen::Vector2i(x, y);
                boundIndex(current_idx);
                values[x][y] = esdf_buffer_[toAddress(current_idx)];
            }
        }

        const double v0 = values[0][0] * (1.0 - diff[0]) + values[1][0] * diff[0];
        const double v1 = values[0][1] * (1.0 - diff[0]) + values[1][1] * diff[0];
        return v0 * (1.0 - diff[1]) + v1 * diff[1];
    }

    bool getDistanceAndGradient(
        const Eigen::Vector2d & pos,
        double & distance,
        Eigen::Vector2d & gradient) const
    {
        if (!isInMap(pos)) {
            distance = std::numeric_limits<double>::max();
            gradient.setZero();
            return false;
        }

        Eigen::Vector2d pos_m = pos;
        pos_m(0) -= 0.5 * resolution_;
        pos_m(1) -= 0.5 * resolution_;

        Eigen::Vector2i idx;
        posToIndex(pos_m, idx);

        Eigen::Vector2d idx_pos;
        indexToPos(idx, idx_pos);

        Eigen::Vector2d diff = pos - idx_pos;
        diff(0) *= resolution_inv_;
        diff(1) *= resolution_inv_;

        double values[2][2];
        for (int x = 0; x < 2; ++x) {
            for (int y = 0; y < 2; ++y) {
                Eigen::Vector2i current_idx = idx + Eigen::Vector2i(x, y);
                boundIndex(current_idx);
                values[x][y] = esdf_buffer_[toAddress(current_idx)];
            }
        }

        const double v0 = values[0][0] * (1.0 - diff[0]) + values[1][0] * diff[0];
        const double v1 = values[0][1] * (1.0 - diff[0]) + values[1][1] * diff[0];
        distance = v0 * (1.0 - diff[1]) + v1 * diff[1];

        gradient(1) = (v1 - v0) * resolution_inv_;
        gradient(0) =
            (1.0 - diff[1]) * (values[1][0] - values[0][0]) +
            diff[1] * (values[1][1] - values[0][1]);
        gradient(0) *= resolution_inv_;
        return true;
    }

    bool isCollision(const Eigen::Vector2d & pos, double safe_threshold = 0.0) const
    {
        return !isInMap(pos) || getDistance(pos) < safe_threshold;
    }

    bool isLineOccupancy(const Eigen::Vector2d & p1, const Eigen::Vector2d & p2) const
    {
        const Eigen::Vector2d diff = p2 - p1;
        const double max_dist = diff.norm();
        if (max_dist < 1e-6) {
            return isCollision(p1);
        }

        const Eigen::Vector2d dir = diff.normalized();
        const double step = resolution_ * 0.1;
        for (double d = 0.0; d <= max_dist; d += step) {
            const Eigen::Vector2d pt = p1 + dir * d;
            if (isCollision(pt)) {
                return true;
            }
        }
        return false;
    }

    RowMatrixXd getMap() const
    {
        RowMatrixXd map(voxel_num_.x(), voxel_num_.y());
        for (int x = 0; x < voxel_num_.x(); ++x) {
            for (int y = 0; y < voxel_num_.y(); ++y) {
                map(x, y) = esdf_buffer_[toAddress(Eigen::Vector2i(x, y))];
            }
        }
        return map;
    }

    std::vector<float> getRosMessageData() const;
    Eigen::Vector2i getVoxelNum() const { return voxel_num_; }
    bool isOccupied(const Eigen::Vector2i & id) const
    {
        return isInMap(id) ? occ_buffer_[toAddress(id)] == 1 : true;
    }

    void posToIndex(const Eigen::Vector2d & pos, Eigen::Vector2i & id) const
    {
        id << static_cast<int>((pos.x() - map_origin_.x()) * resolution_inv_),
            static_cast<int>((pos.y() - map_origin_.y()) * resolution_inv_);
    }

    void indexToPos(const Eigen::Vector2i & id, Eigen::Vector2d & pos) const
    {
        pos << (id.x() + 0.5) * resolution_ + map_origin_.x(),
            (id.y() + 0.5) * resolution_ + map_origin_.y();
    }

protected:
    inline int toAddress(int x, int y) const
    {
        return x * voxel_num_.y() + y;
    }

    void updateESDF();

private:
    template<typename FGetVal, typename FSetVal>
    static void fillESDF(FGetVal f_get_val, FSetVal f_set_val, int start, int end, int size)
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
                s = ((f_get_val(q) + q * q) -
                     (f_get_val(v[static_cast<std::size_t>(k)]) +
                      v[static_cast<std::size_t>(k)] * v[static_cast<std::size_t>(k)])) /
                    (2 * q - 2 * v[static_cast<std::size_t>(k)]);
            } while (s <= z[static_cast<std::size_t>(k)]);

            ++k;
            v[static_cast<std::size_t>(k)] = q;
            z[static_cast<std::size_t>(k)] = s;
            z[static_cast<std::size_t>(k + 1)] = std::numeric_limits<double>::max();
        }

        k = start;
        for (int q = start; q <= end; ++q) {
            while (z[static_cast<std::size_t>(k + 1)] < q) {
                ++k;
            }
            const double val =
                (q - v[static_cast<std::size_t>(k)]) * (q - v[static_cast<std::size_t>(k)]) +
                f_get_val(v[static_cast<std::size_t>(k)]);
            f_set_val(q, val);
        }
    }

    inline int toAddress(const Eigen::Vector2i & id) const
    {
        return id.x() * voxel_num_.y() + id.y();
    }

    inline void boundIndex(Eigen::Vector2i & id) const
    {
        id(0) = std::max(std::min(id(0), voxel_num_.x() - 1), 0);
        id(1) = std::max(std::min(id(1), voxel_num_.y() - 1), 0);
    }

    inline bool isInMap(const Eigen::Vector2d & pos) const
    {
        return (pos.array() >= min_boundary_.array()).all() &&
               (pos.array() <= max_boundary_.array()).all();
    }

    inline bool isInMap(const Eigen::Vector2i & idx) const
    {
        return idx.x() >= 0 && idx.y() >= 0 &&
               idx.x() < voxel_num_.x() && idx.y() < voxel_num_.y();
    }

    double resolution_{0.0};
    double resolution_inv_{0.0};
    Eigen::Vector2d map_size_{Eigen::Vector2d::Zero()};
    Eigen::Vector2d map_origin_{Eigen::Vector2d::Zero()};
    Eigen::Vector2d min_boundary_{Eigen::Vector2d::Zero()};
    Eigen::Vector2d max_boundary_{Eigen::Vector2d::Zero()};
    Eigen::Vector2i voxel_num_{Eigen::Vector2i::Zero()};
    int buffer_size_{0};
    std::vector<char> occ_buffer_;
    std::vector<double> esdf_buffer_;
};

class ESDF : public rclcpp::Node
{
public:
    explicit ESDF(const rclcpp::NodeOptions & option = rclcpp::NodeOptions());
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
    ESDFMap2D esdf_map_;
};
}
#endif
