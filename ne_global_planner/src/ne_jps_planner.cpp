#include "ne_global_planner/ne_jps_planner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <utility>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"

namespace jps_planner
{

namespace
{

struct PathSafetyStats
{
    double min_distance{std::numeric_limits<double>::max()};
    bool violates_safe_distance{false};
    Eigen::Vector2d first_violation_point{Eigen::Vector2d::Zero()};
};

PathSafetyStats evaluatePathSafety(
    const std::shared_ptr<esdf_builder::ESDFMap2D> & map,
    const std::vector<Eigen::Vector2d> & path,
    double safe_distance)
{
    PathSafetyStats stats;
    if (!map || path.empty()) {
        return stats;
    }

    for (const auto & point : path) {
        const double distance = map->getDistance(point);
        stats.min_distance = std::min(stats.min_distance, distance);
        if (!stats.violates_safe_distance && map->isCollision(point, safe_distance)) {
            stats.violates_safe_distance = true;
            stats.first_violation_point = point;
        }
    }

    return stats;
}

}  // namespace

JpsPlannerNode::JpsPlannerNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("jps_planner_node", options),
  map_(std::make_shared<esdf_builder::ESDFMap2D>())
{
    init();
    RCLCPP_INFO(this->get_logger(), "jps planner node is ready");
}

void JpsPlannerNode::init()
{
    esdf_topic_ = this->declare_parameter<std::string>("esdf_topic", esdf_topic_);
    jps_path_topic_ = this->declare_parameter<std::string>("jps_path_topic", jps_path_topic_);
    optimized_path_topic_ = this->declare_parameter<std::string>(
        "optimized_path_topic",
        optimized_path_topic_);
    path_marker_topic_ = this->declare_parameter<std::string>("path_marker_topic", path_marker_topic_);
    clicked_point_topic_ = this->declare_parameter<std::string>("clicked_point_topic", clicked_point_topic_);
    initial_pose_topic_ = this->declare_parameter<std::string>("initial_pose_topic", initial_pose_topic_);
    goal_pose_topic_ = this->declare_parameter<std::string>("goal_pose_topic", goal_pose_topic_);

    start_pos_.x() = this->declare_parameter<double>("start_x", start_pos_.x());
    start_pos_.y() = this->declare_parameter<double>("start_y", start_pos_.y());
    goal_pos_.x() = this->declare_parameter<double>("goal_x", goal_pos_.x());
    goal_pos_.y() = this->declare_parameter<double>("goal_y", goal_pos_.y());
    has_start_ = true;
    has_goal_ = true;

    safe_distance_ = this->declare_parameter<double>("safe_distance", safe_distance_);
    traj_total_time_ = this->declare_parameter<double>("traj_total_time", traj_total_time_);
    traj_piece_len_ = this->declare_parameter<double>("traj_piece_len", traj_piece_len_);
    traj_sample_dt_ = this->declare_parameter<double>("traj_sample_dt", traj_sample_dt_);
    path_z_offset_ = this->declare_parameter<double>("path_z_offset", path_z_offset_);
    marker_line_width_ = this->declare_parameter<double>("marker_line_width", marker_line_width_);
    optimize_trajectory_ = this->declare_parameter<bool>("optimize_trajectory", optimize_trajectory_);
    plan_on_map_update_ = this->declare_parameter<bool>("plan_on_map_update", plan_on_map_update_);
    max_expand_ = this->declare_parameter<int>("max_expand", max_expand_);

    const auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
    esdf_sub_ = this->create_subscription<nav_interfaces::msg::Esdf>(
        esdf_topic_,
        qos,
        std::bind(&JpsPlannerNode::esdfCallback, this, std::placeholders::_1));
    clicked_point_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
        clicked_point_topic_,
        qos,
        std::bind(&JpsPlannerNode::clickedPointCallback, this, std::placeholders::_1));
    initial_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        initial_pose_topic_,
        qos,
        std::bind(&JpsPlannerNode::initialPoseCallback, this, std::placeholders::_1));
    goal_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        goal_pose_topic_,
        qos,
        std::bind(&JpsPlannerNode::goalPoseCallback, this, std::placeholders::_1));

    jps_path_pub_ = this->create_publisher<nav_msgs::msg::Path>(jps_path_topic_, qos);
    optimized_path_pub_ = this->create_publisher<nav_msgs::msg::Path>(optimized_path_topic_, qos);
    path_marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        path_marker_topic_,
        qos);
}

void JpsPlannerNode::esdfCallback(const nav_interfaces::msg::Esdf::SharedPtr msg)
{
    if (!msg) {
        return;
    }

    if (msg->info.width == 0 || msg->info.height == 0 || msg->info.resolution <= 0.0F) {
        RCLCPP_WARN(this->get_logger(), "invalid ESDF map metadata");
        return;
    }

    const auto expected_size =
        static_cast<std::size_t>(msg->info.width) * static_cast<std::size_t>(msg->info.height);
    if (msg->data.size() != expected_size) {
        RCLCPP_WARN(this->get_logger(), "ESDF data size does not match map metadata");
        return;
    }

    latest_esdf_msg_ = msg;
    map_->reset(msg->info, msg->data);
    if (plan_on_map_update_) {
        makePlan(*msg);
    }
}

void JpsPlannerNode::clickedPointCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
{
    if (!msg) {
        return;
    }

    const Vector2d pos(msg->point.x, msg->point.y);
    if (!awaiting_clicked_goal_) {
        start_pos_ = pos;
        has_start_ = true;
        awaiting_clicked_goal_ = true;
        RCLCPP_INFO(
            this->get_logger(),
            "clicked start set to (%.3f, %.3f); click another point to set the goal",
            start_pos_.x(),
            start_pos_.y());
        return;
    }

    goal_pos_ = pos;
    has_goal_ = true;
    awaiting_clicked_goal_ = false;
    RCLCPP_INFO(
        this->get_logger(),
        "clicked goal set to (%.3f, %.3f); planning from clicked start",
        goal_pos_.x(),
        goal_pos_.y());
    tryPlanFromCurrentMap();
}

void JpsPlannerNode::initialPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
    if (!msg) {
        return;
    }

    start_pos_.x() = msg->pose.pose.position.x;
    start_pos_.y() = msg->pose.pose.position.y;
    has_start_ = true;
    awaiting_clicked_goal_ = false;
    RCLCPP_INFO(
        this->get_logger(),
        "start pose updated from RViz initial pose: (%.3f, %.3f)",
        start_pos_.x(),
        start_pos_.y());
}

void JpsPlannerNode::goalPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    if (!msg) {
        return;
    }

    goal_pos_.x() = msg->pose.position.x;
    goal_pos_.y() = msg->pose.position.y;
    has_goal_ = true;
    awaiting_clicked_goal_ = false;
    RCLCPP_INFO(
        this->get_logger(),
        "goal pose updated from RViz goal tool: (%.3f, %.3f)",
        goal_pos_.x(),
        goal_pos_.y());
    tryPlanFromCurrentMap();
}

bool JpsPlannerNode::tryPlanFromCurrentMap()
{
    if (!latest_esdf_msg_) {
        RCLCPP_WARN(this->get_logger(), "cannot plan yet: no ESDF map has been received");
        return false;
    }
    if (!has_start_ || !has_goal_) {
        RCLCPP_WARN(this->get_logger(), "cannot plan yet: start or goal has not been set");
        return false;
    }

    map_->reset(latest_esdf_msg_->info, latest_esdf_msg_->data);
    return makePlan(*latest_esdf_msg_);
}

bool JpsPlannerNode::makePlan(const nav_interfaces::msg::Esdf & msg)
{
    if (!has_start_ || !has_goal_) {
        RCLCPP_WARN(this->get_logger(), "start or goal is not available yet");
        return false;
    }

    Eigen::Vector2i start_idx;
    Eigen::Vector2i goal_idx;
    if (!worldToIndex(start_pos_, start_idx) || !worldToIndex(goal_pos_, goal_idx)) {
        const double origin_x = static_cast<double>(msg.info.origin.position.x);
        const double origin_y = static_cast<double>(msg.info.origin.position.y);
        const double resolution = static_cast<double>(msg.info.resolution);
        const double map_max_x = origin_x + static_cast<double>(msg.info.width) * resolution;
        const double map_max_y = origin_y + static_cast<double>(msg.info.height) * resolution;
        RCLCPP_WARN(
            this->get_logger(),
            "start or goal is outside the ESDF map: start=(%.3f, %.3f), goal=(%.3f, %.3f), "
            "map_origin=(%.3f, %.3f), map_size=(%u x %u), resolution=%.3f, "
            "map_bounds_x=[%.3f, %.3f), map_bounds_y=[%.3f, %.3f)",
            start_pos_.x(),
            start_pos_.y(),
            goal_pos_.x(),
            goal_pos_.y(),
            origin_x,
            origin_y,
            msg.info.width,
            msg.info.height,
            resolution,
            origin_x,
            map_max_x,
            origin_y,
            map_max_y);
        return false;
    }

    const double start_distance = map_->getDistance(start_pos_);
    const double goal_distance = map_->getDistance(goal_pos_);
    const bool start_occupied = map_->isOccupied(start_idx);
    const bool goal_occupied = map_->isOccupied(goal_idx);
    const bool start_collision = map_->isCollision(start_pos_, safe_distance_);
    const bool goal_collision = map_->isCollision(goal_pos_, safe_distance_);
    const bool direct_line_collision = checkLineCollision(start_pos_, goal_pos_);

    JPS::EsdfSearch search(map_, safe_distance_);
    const bool success = search.plan(start_idx.x(), start_idx.y(), goal_idx.x(), goal_idx.y(), max_expand_);
    if (!success) {
        const auto close_set = search.getCloseSet();
        const auto open_set = search.getOpenSet();
        const auto all_set = search.getAllSet();
        RCLCPP_WARN(
            this->get_logger(),
            "JPS failed from index (%d, %d) to (%d, %d), "
            "start_world=(%.3f, %.3f), goal_world=(%.3f, %.3f), "
            "start_esdf=%.3f, goal_esdf=%.3f, "
            "start_occupied=%s, goal_occupied=%s, "
            "start_collision=%s, goal_collision=%s, "
            "direct_line_collision=%s, "
            "expanded_nodes=%zu, open_nodes=%zu, all_seen_nodes=%zu, "
            "safe_distance=%.3f",
            start_idx.x(),
            start_idx.y(),
            goal_idx.x(),
            goal_idx.y(),
            start_pos_.x(),
            start_pos_.y(),
            goal_pos_.x(),
            goal_pos_.y(),
            start_distance,
            goal_distance,
            start_occupied ? "true" : "false",
            goal_occupied ? "true" : "false",
            start_collision ? "true" : "false",
            goal_collision ? "true" : "false",
            direct_line_collision ? "true" : "false",
            close_set.size(),
            open_set.size(),
            all_set.size(),
            safe_distance_);
        return false;
    }
    RCLCPP_INFO(this -> get_logger(),"jps successfully index");
    const auto raw_path = optimizePath(expandGridPath(search.getPath()));
    RCLCPP_INFO(this -> get_logger(),"成功获得原始的路径");
    const auto raw_stats = evaluatePathSafety(map_, raw_path, safe_distance_);
    RCLCPP_INFO(
        this->get_logger(),
        "raw JPS path stats: points=%zu, min_esdf_distance=%.3f, violates_safe_distance=%s%s",
        raw_path.size(),
        raw_stats.min_distance,
        raw_stats.violates_safe_distance ? "true" : "false",
        raw_stats.violates_safe_distance ?
            (std::string(", first_violation=(") +
             std::to_string(raw_stats.first_violation_point.x()) + ", " +
             std::to_string(raw_stats.first_violation_point.y()) + ")").c_str() :
            "");

    jps_path_pub_->publish(makePathMsg(raw_path, msg.header));

    if (!optimize_trajectory_) {
        optimized_path_pub_->publish(makePathMsg(raw_path, msg.header));
        visualization_msgs::msg::MarkerArray marker_array;
        marker_array.markers.push_back(
            makeLineStripMarker(
                raw_path, msg.header, "jps_raw_path", 0, 0.95F, 0.35F, 0.30F, 0.35F,
                marker_line_width_ * 2.2, path_z_offset_));
        marker_array.markers.push_back(
            makeLineStripMarker(
                raw_path, msg.header, "jps_raw_path", 1, 1.0F, 0.45F, 0.35F, 0.95F,
                marker_line_width_, path_z_offset_));
        marker_array.markers.push_back(
            makePointListMarker(
                raw_path, msg.header, "jps_raw_path_points", 2, 1.0F, 0.82F, 0.25F, 0.95F,
                marker_line_width_ * 2.4, path_z_offset_));
        marker_array.markers.push_back(
            makeLineStripMarker(
                raw_path, msg.header, "jps_optimized_path", 3, 0.30F, 1.0F, 0.55F, 0.28F,
                marker_line_width_ * 2.8, path_z_offset_ * 1.5));
        marker_array.markers.push_back(
            makeLineStripMarker(
                raw_path, msg.header, "jps_optimized_path", 4, 0.12F, 0.95F, 0.42F, 0.98F,
                marker_line_width_ * 1.25, path_z_offset_ * 1.5));
        marker_array.markers.push_back(
            makeEndpointMarker(
                raw_path.front(), msg.header, "planner_endpoints", 5, 0.25F, 0.65F, 1.0F, 0.98F,
                marker_line_width_ * 4.5, path_z_offset_ * 1.8));
        marker_array.markers.push_back(
            makeEndpointMarker(
                raw_path.back(), msg.header, "planner_endpoints", 6, 1.0F, 0.30F, 0.30F, 0.98F,
                marker_line_width_ * 4.5, path_z_offset_ * 1.8));
        path_marker_pub_->publish(marker_array);
        return true;
    }

    auto traj_params = makeTrajParams();
    ne_traj_opt::TrajectoryOptimize optimizer(map_, raw_path, traj_params);
    if (!optimizer.jps_plan()) {
        RCLCPP_WARN(this->get_logger(), "trajectory optimization failed, publishing raw JPS path as fallback");
        optimized_path_pub_->publish(makePathMsg(raw_path, msg.header));
        visualization_msgs::msg::MarkerArray marker_array;
        marker_array.markers.push_back(
            makeLineStripMarker(
                raw_path, msg.header, "jps_raw_path", 0, 0.95F, 0.35F, 0.30F, 0.35F,
                marker_line_width_ * 2.2, path_z_offset_));
        marker_array.markers.push_back(
            makeLineStripMarker(
                raw_path, msg.header, "jps_raw_path", 1, 1.0F, 0.45F, 0.35F, 0.95F,
                marker_line_width_, path_z_offset_));
        marker_array.markers.push_back(
            makePointListMarker(
                raw_path, msg.header, "jps_raw_path_points", 2, 1.0F, 0.82F, 0.25F, 0.95F,
                marker_line_width_ * 2.4, path_z_offset_));
        marker_array.markers.push_back(
            makeLineStripMarker(
                raw_path, msg.header, "jps_optimized_path", 3, 0.30F, 1.0F, 0.55F, 0.28F,
                marker_line_width_ * 2.8, path_z_offset_ * 1.5));
        marker_array.markers.push_back(
            makeLineStripMarker(
                raw_path, msg.header, "jps_optimized_path", 4, 0.12F, 0.95F, 0.42F, 0.98F,
                marker_line_width_ * 1.25, path_z_offset_ * 1.5));
        marker_array.markers.push_back(
            makeEndpointMarker(
                raw_path.front(), msg.header, "planner_endpoints", 5, 0.25F, 0.65F, 1.0F, 0.98F,
                marker_line_width_ * 4.5, path_z_offset_ * 1.8));
        marker_array.markers.push_back(
            makeEndpointMarker(
                raw_path.back(), msg.header, "planner_endpoints", 6, 1.0F, 0.30F, 0.30F, 0.98F,
                marker_line_width_ * 4.5, path_z_offset_ * 1.8));
        path_marker_pub_->publish(marker_array);
        return false;
    }

    const auto optimized_path = optimizer.sampleTrajectory(traj_sample_dt_);
    const double collision_check_dt = std::min(std::max(traj_sample_dt_ * 0.5, 0.01), 0.02);
    const auto collision_check_path = optimizer.sampleTrajectory(collision_check_dt);
    const auto optimized_stats = evaluatePathSafety(map_, collision_check_path, safe_distance_);
    RCLCPP_INFO(
        this->get_logger(),
        "optimized trajectory stats: samples=%zu, min_esdf_distance=%.3f, violates_safe_distance=%s%s",
        collision_check_path.size(),
        optimized_stats.min_distance,
        optimized_stats.violates_safe_distance ? "true" : "false",
        optimized_stats.violates_safe_distance ?
            (std::string(", first_violation=(") +
             std::to_string(optimized_stats.first_violation_point.x()) + ", " +
             std::to_string(optimized_stats.first_violation_point.y()) + ")").c_str() :
            "");

    const bool optimized_safe = !optimized_stats.violates_safe_distance;
    if (!optimized_safe) {
        RCLCPP_WARN(
            this->get_logger(),
            "optimized trajectory intersects obstacle or violates safe_distance, publishing optimized path anyway");
    }
    optimized_path_pub_->publish(makePathMsg(optimized_path, msg.header));

    visualization_msgs::msg::MarkerArray marker_array;
    marker_array.markers.push_back(
        makeLineStripMarker(
            raw_path, msg.header, "jps_raw_path", 0, 0.95F, 0.35F, 0.30F, 0.32F,
            marker_line_width_ * 1.9, path_z_offset_));
    marker_array.markers.push_back(
        makeLineStripMarker(
            raw_path, msg.header, "jps_raw_path", 1, 1.0F, 0.45F, 0.35F, 0.90F,
            marker_line_width_ * 0.9, path_z_offset_));
    marker_array.markers.push_back(
        makePointListMarker(
            raw_path, msg.header, "jps_raw_path_points", 2, 1.0F, 0.82F, 0.25F, 0.92F,
            marker_line_width_ * 2.1, path_z_offset_));
    marker_array.markers.push_back(
        makeLineStripMarker(
            optimized_path, msg.header, "jps_optimized_path", 3, 0.30F, 1.0F, 0.55F, 0.30F,
            marker_line_width_ * 3.0, path_z_offset_ * 1.5));
    marker_array.markers.push_back(
        makeLineStripMarker(
            optimized_path, msg.header, "jps_optimized_path", 4, 0.12F, 0.95F, 0.42F, 0.98F,
            marker_line_width_ * 1.35, path_z_offset_ * 1.5));
    marker_array.markers.push_back(
        makePointListMarker(
            optimized_path, msg.header, "jps_optimized_path_points", 5, 0.75F, 1.0F, 0.82F, 0.90F,
            marker_line_width_ * 1.6, path_z_offset_ * 1.5));
    marker_array.markers.push_back(
        makeEndpointMarker(
            raw_path.front(), msg.header, "planner_endpoints", 6, 0.25F, 0.65F, 1.0F, 0.98F,
            marker_line_width_ * 4.5, path_z_offset_ * 1.8));
    marker_array.markers.push_back(
        makeEndpointMarker(
            optimized_path.back(), msg.header, "planner_endpoints", 7, 1.0F, 0.30F, 0.30F, 0.98F,
            marker_line_width_ * 4.5, path_z_offset_ * 1.8));
    path_marker_pub_->publish(marker_array);

    RCLCPP_INFO(
        this->get_logger(),
        "published JPS path with %zu points and optimized trajectory with %zu points",
        raw_path.size(),
        optimized_path.size());
    return true;
}

bool JpsPlannerNode::worldToIndex(const Vector2d & pos, Eigen::Vector2i & index) const
{
    if (!map_) {
        return false;
    }

    map_->posToIndex(pos, index);
    const Eigen::Vector2i voxel_num = map_->getVoxelNum();
    return index.x() >= 0 && index.y() >= 0 &&
           index.x() < voxel_num.x() && index.y() < voxel_num.y();
}

std::vector<JpsPlannerNode::Vector2d> JpsPlannerNode::expandGridPath(
    const std::vector<JPS::StatePtr> & grid_path) const
{
    std::vector<Vector2d> expanded_path;
    if (!map_ || grid_path.empty()) {
        return expanded_path;
    }

    expanded_path.reserve(grid_path.size() * 4);
    for (std::size_t i = 0; i < grid_path.size(); ++i) {
        const auto & node = grid_path[i];
        if (!node) {
            continue;
        }

        if (expanded_path.empty()) {
            Vector2d pos;
            map_->indexToPos(Eigen::Vector2i(node->x, node->y), pos);
            expanded_path.push_back(pos);
        }

        if (i == 0) {
            continue;
        }

        const auto & prev = grid_path[i - 1];
        if (!prev) {
            continue;
        }

        const int delta_x = node->x - prev->x;
        const int delta_y = node->y - prev->y;
        const int step_count = std::max(std::abs(delta_x), std::abs(delta_y));
        if (step_count <= 0) {
            continue;
        }

        const int step_x = (delta_x == 0) ? 0 : (delta_x > 0 ? 1 : -1);
        const int step_y = (delta_y == 0) ? 0 : (delta_y > 0 ? 1 : -1);

        for (int step = 1; step <= step_count; ++step) {
            Vector2d pos;
            map_->indexToPos(
                Eigen::Vector2i(prev->x + step * step_x, prev->y + step * step_y),
                pos);
            if (expanded_path.empty() || (pos - expanded_path.back()).norm() > 1e-6) {
                expanded_path.push_back(pos);
            }
        }
    }

    return expanded_path;
}

bool JpsPlannerNode::checkLineCollision(const Vector2d & start, const Vector2d & end) const
{
    if (!map_) {
        return true;
    }

    Eigen::Vector2i start_idx;
    Eigen::Vector2i end_idx;
    map_->posToIndex(start, start_idx);
    map_->posToIndex(end, end_idx);

    int dx = std::abs(end_idx.x() - start_idx.x());
    int dy = std::abs(end_idx.y() - start_idx.y());
    const int sx = start_idx.x() < end_idx.x() ? 1 : -1;
    const int sy = start_idx.y() < end_idx.y() ? 1 : -1;
    int err = dx - dy;

    while (true) {
        Vector2d pos;
        map_->indexToPos(start_idx, pos);
        if (map_->isCollision(pos, safe_distance_)) {
            return true;
        }
        if (start_idx == end_idx) {
            break;
        }

        const int e2 = 2 * err;
        if (e2 > -dy) {
            err -= dy;
            start_idx.x() += sx;
        }
        if (e2 < dx) {
            err += dx;
            start_idx.y() += sy;
        }
    }

    return false;
}

std::vector<JpsPlannerNode::Vector2d> JpsPlannerNode::optimizePath(
    const std::vector<Vector2d> & path) const
{
    if (path.size() < 2) {
        return path;
    }

    std::vector<Vector2d> optimized_path;
    optimized_path.reserve(path.size());
    optimized_path.push_back(path.front());

    Vector2d prev_pose = path.front();
    double cost1 = checkLineCollision(path[0], path[1]) ?
        std::numeric_limits<double>::infinity() :
        (path[0] - path[1]).norm();

    for (std::size_t i = 1; i + 1 < path.size(); ++i) {
        const auto & pose1 = path[i];
        const auto & pose2 = path[i + 1];

        const double cost2 = checkLineCollision(pose1, pose2) ?
            std::numeric_limits<double>::infinity() :
            (pose1 - pose2).norm();
        const double cost3 = checkLineCollision(prev_pose, pose2) ?
            std::numeric_limits<double>::infinity() :
            (prev_pose - pose2).norm();

        if (cost3 < cost1 + cost2) {
            cost1 = cost3;
        } else {
            optimized_path.push_back(pose1);
            prev_pose = pose1;
            cost1 = cost2;
        }
    }

    if ((optimized_path.back() - path.back()).norm() > 1e-6) {
        optimized_path.push_back(path.back());
    }

    return optimized_path;
}

nav_msgs::msg::Path JpsPlannerNode::makePathMsg(
    const std::vector<Vector2d> & path,
    const std_msgs::msg::Header & header) const
{
    nav_msgs::msg::Path msg;
    msg.header = header;
    msg.poses.reserve(path.size());

    for (const auto & point : path) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header = header;
        pose.pose.position.x = point.x();
        pose.pose.position.y = point.y();
        pose.pose.position.z = path_z_offset_;
        pose.pose.orientation.w = 1.0;
        msg.poses.push_back(std::move(pose));
    }

    return msg;
}

visualization_msgs::msg::Marker JpsPlannerNode::makeLineStripMarker(
    const std::vector<Vector2d> & path,
    const std_msgs::msg::Header & header,
    const std::string & ns,
    int id,
    float r,
    float g,
    float b,
    float a,
    double line_width,
    double z_offset) const
{
    visualization_msgs::msg::Marker marker;
    marker.header = header;
    marker.ns = ns;
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = std::max(line_width, 1e-3);
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = a;

    marker.points.reserve(path.size());
    for (const auto & point : path) {
        geometry_msgs::msg::Point marker_point;
        marker_point.x = point.x();
        marker_point.y = point.y();
        marker_point.z = z_offset;
        marker.points.push_back(std::move(marker_point));
    }

    return marker;
}

visualization_msgs::msg::Marker JpsPlannerNode::makePointListMarker(
    const std::vector<Vector2d> & path,
    const std_msgs::msg::Header & header,
    const std::string & ns,
    int id,
    float r,
    float g,
    float b,
    float a,
    double point_size,
    double z_offset) const
{
    visualization_msgs::msg::Marker marker;
    marker.header = header;
    marker.ns = ns;
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = std::max(point_size, 1e-3);
    marker.scale.y = std::max(point_size, 1e-3);
    marker.scale.z = std::max(point_size * 0.45, 1e-3);
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = a;

    marker.points.reserve(path.size());
    for (const auto & point : path) {
        geometry_msgs::msg::Point marker_point;
        marker_point.x = point.x();
        marker_point.y = point.y();
        marker_point.z = z_offset;
        marker.points.push_back(std::move(marker_point));
    }

    return marker;
}

visualization_msgs::msg::Marker JpsPlannerNode::makeEndpointMarker(
    const Vector2d & point,
    const std_msgs::msg::Header & header,
    const std::string & ns,
    int id,
    float r,
    float g,
    float b,
    float a,
    double point_size,
    double z_offset) const
{
    visualization_msgs::msg::Marker marker;
    marker.header = header;
    marker.ns = ns;
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = point.x();
    marker.pose.position.y = point.y();
    marker.pose.position.z = z_offset;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = std::max(point_size, 1e-3);
    marker.scale.y = std::max(point_size, 1e-3);
    marker.scale.z = std::max(point_size, 1e-3);
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = a;
    return marker;
}

ne_traj_opt::TrajectoryParams JpsPlannerNode::makeTrajParams() const
{
    ne_traj_opt::TrajectoryParams params;
    params.total_time = std::max(traj_total_time_, 1e-3);
    params.piece_len = std::max(traj_piece_len_, 1e-3);
    params.safe_threshold = safe_distance_;
    return params;
}

}  // namespace jps_planner

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<jps_planner::JpsPlannerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
