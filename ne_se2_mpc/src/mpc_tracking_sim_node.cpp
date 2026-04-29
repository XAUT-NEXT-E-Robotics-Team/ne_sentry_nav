#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "ne_global_planner/SplineTrajectory.hpp"
#include "ne_se2_mpc/ne_mpc_controller.hpp"

namespace
{

double normalizeAngle(double angle)
{
    while (angle > M_PI) {
        angle -= 2.0 * M_PI;
    }
    while (angle < -M_PI) {
        angle += 2.0 * M_PI;
    }
    return angle;
}

geometry_msgs::msg::Quaternion quaternionFromYaw(double yaw)
{
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw);
    geometry_msgs::msg::Quaternion msg;
    msg.x = q.x();
    msg.y = q.y();
    msg.z = q.z();
    msg.w = q.w();
    return msg;
}

class MpcTrackingSimNode : public rclcpp::Node
{
public:
    MpcTrackingSimNode()
    : rclcpp::Node("mpc_tracking_sim_node")
    {
        declareParameters();
        createInterfaces();
        initializeSolver();
    }

private:
    using Trajectory2D = mpc_slover::MpcSlover::Trajectory2D;

    void declareParameters()
    {
        map_frame_ = this->declare_parameter<std::string>("map_frame", "world");
        base_frame_ = this->declare_parameter<std::string>("base_frame", "mpc_base_link");
        optimized_path_topic_ = this->declare_parameter<std::string>("optimized_path_topic", "/optimized_trajectory");
        odom_topic_ = this->declare_parameter<std::string>("odom_topic", "/mpc_sim/odom");
        actual_path_topic_ = this->declare_parameter<std::string>("actual_path_topic", "/mpc_sim/actual_path");
        reference_path_topic_ = this->declare_parameter<std::string>("reference_path_topic", "/mpc_sim/reference_path");
        preview_path_topic_ = this->declare_parameter<std::string>("preview_path_topic", "/mpc_sim/preview_path");
        marker_topic_ = this->declare_parameter<std::string>("marker_topic", "/mpc_sim/markers");
        spline_speed_ = this->declare_parameter<double>("spline_speed", 1.2);
        sim_dt_ = this->declare_parameter<double>("sim_dt", 0.05);
        status_rate_hz_ = this->declare_parameter<double>("status_rate_hz", 5.0);
        reference_sample_dt_ = this->declare_parameter<double>("reference_sample_dt", 0.05);
        reset_on_new_path_ = this->declare_parameter<bool>("reset_on_new_path", true);
        duplicate_path_position_tol_ = this->declare_parameter<double>("duplicate_path_position_tol", 0.05);
        stop_position_tolerance_ = this->declare_parameter<double>("stop_position_tolerance", 0.15);
        stop_velocity_tolerance_ = this->declare_parameter<double>("stop_velocity_tolerance", 0.12);

        mpc_config_.dt = this->declare_parameter<double>("mpc.dt", 0.05);
        mpc_config_.horizon = this->declare_parameter<int>("mpc.horizon", 40);
        mpc_config_.mass = this->declare_parameter<double>("mpc.mass", 20.0);
        mpc_config_.iz = this->declare_parameter<double>("mpc.iz", 5.0);
        mpc_config_.gravity = this->declare_parameter<double>("mpc.gravity", 9.81);
        mpc_config_.max_preview_time = this->declare_parameter<double>("mpc.max_preview_time", 2.0);
        mpc_config_.projection_sample_dt = this->declare_parameter<double>("mpc.projection_sample_dt", 0.02);

        const auto q_diag = this->declare_parameter<std::vector<double>>(
            "mpc.q_diag",
            std::vector<double>{50.0, 50.0, 20.0, 5.0, 5.0, 2.0});
        const auto r_diag = this->declare_parameter<std::vector<double>>(
            "mpc.r_diag",
            std::vector<double>{0.2, 0.2, 0.1});
        const auto u_min = this->declare_parameter<std::vector<double>>(
            "mpc.u_min",
            std::vector<double>{-30.0, -30.0, -10.0});
        const auto u_max = this->declare_parameter<std::vector<double>>(
            "mpc.u_max",
            std::vector<double>{30.0, 30.0, 10.0});

        for (int i = 0; i < 6 && i < static_cast<int>(q_diag.size()); ++i) {
            mpc_config_.q_diag(i) = q_diag[static_cast<std::size_t>(i)];
        }
        for (int i = 0; i < 3 && i < static_cast<int>(r_diag.size()); ++i) {
            mpc_config_.r_diag(i) = r_diag[static_cast<std::size_t>(i)];
            mpc_config_.u_min(i) = u_min[static_cast<std::size_t>(i)];
            mpc_config_.u_max(i) = u_max[static_cast<std::size_t>(i)];
        }
    }

    void createInterfaces()
    {
        path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            optimized_path_topic_,
            rclcpp::QoS(1).reliable(),
            std::bind(&MpcTrackingSimNode::pathCallback, this, std::placeholders::_1));

        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(odom_topic_, 10);
        actual_path_pub_ = this->create_publisher<nav_msgs::msg::Path>(actual_path_topic_, 10);
        reference_path_pub_ = this->create_publisher<nav_msgs::msg::Path>(reference_path_topic_, 1);
        preview_path_pub_ = this->create_publisher<nav_msgs::msg::Path>(preview_path_topic_, 10);
        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(marker_topic_, 10);
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        sim_timer_ = this->create_wall_timer(
            std::chrono::duration<double>(sim_dt_),
            std::bind(&MpcTrackingSimNode::stepSimulation, this));

        status_timer_ = this->create_wall_timer(
            std::chrono::duration<double>(1.0 / std::max(status_rate_hz_, 0.2)),
            std::bind(&MpcTrackingSimNode::printStatus, this));
    }

    void initializeSolver()
    {
        solver_ready_ = solver_.initialize(mpc_config_);
        if (!solver_ready_) {
            RCLCPP_ERROR(this->get_logger(), "failed to initialize MPC solver");
            return;
        }

        RCLCPP_INFO(
            this->get_logger(),
            "MPC solver ready: dt=%.3f horizon=%d mass=%.3f iz=%.3f",
            mpc_config_.dt,
            mpc_config_.horizon,
            mpc_config_.mass,
            mpc_config_.iz);
    }

    void pathCallback(const nav_msgs::msg::Path::SharedPtr msg)
    {
        if (!msg || msg->poses.size() < 2U) {
            RCLCPP_WARN(this->get_logger(), "received optimized path with insufficient poses");
            return;
        }

        std::vector<Eigen::Vector2d> waypoints;
        waypoints.reserve(msg->poses.size());
        for (const auto & pose : msg->poses) {
            const Eigen::Vector2d pt(pose.pose.position.x, pose.pose.position.y);
            if (waypoints.empty() || (pt - waypoints.back()).norm() > 1e-6) {
                waypoints.push_back(pt);
            }
        }

        if (waypoints.size() < 2U) {
            RCLCPP_WARN(this->get_logger(), "optimized path collapsed to fewer than 2 unique points");
            return;
        }

        if (trajectory_ready_ && isSimilarPath(waypoints, last_waypoints_)) {
            RCLCPP_DEBUG(
                this->get_logger(),
                "ignoring duplicate optimized path with %zu samples",
                waypoints.size());
            return;
        }

        if (!fitReferenceSpline(waypoints)) {
            RCLCPP_WARN(this->get_logger(), "failed to fit spline from optimized path");
            return;
        }

        solver_.setReferenceTrajectory(reference_trajectory_);
        trajectory_ready_ = solver_.state_init();
        if (!trajectory_ready_) {
            RCLCPP_WARN(this->get_logger(), "solver could not initialize from fitted trajectory");
            return;
        }

        if (!trajectory_ready_ || reset_on_new_path_) {
            sim_state_ = sampleTrajectoryState(reference_trajectory_, reference_trajectory_.getStartTime());
            actual_path_msg_.poses.clear();
        }

        last_waypoints_ = waypoints;
        publishReferencePath();
        RCLCPP_INFO(
            this->get_logger(),
            "received optimized path: samples=%zu duration=%.3f reset=%s",
            waypoints.size(),
            reference_trajectory_.getDuration(),
            reset_on_new_path_ ? "true" : "false");
    }

    bool fitReferenceSpline(const std::vector<Eigen::Vector2d> & waypoints)
    {
        std::vector<double> times;
        times.reserve(waypoints.size());
        times.push_back(0.0);

        for (std::size_t i = 1; i < waypoints.size(); ++i) {
            const double ds = (waypoints[i] - waypoints[i - 1]).norm();
            times.push_back(times.back() + std::max(ds / std::max(spline_speed_, 0.1), sim_dt_));
        }

        SplineTrajectory::SplineVector2D spline_points;
        spline_points.reserve(waypoints.size());
        for (const auto & waypoint : waypoints) {
            spline_points.push_back(waypoint);
        }

        const Eigen::Vector2d start_dir = (waypoints[1] - waypoints[0]).normalized();
        const Eigen::Vector2d end_dir = (waypoints.back() - waypoints[waypoints.size() - 2]).normalized();
        const SplineTrajectory::BoundaryConditions<2> bc(
            start_dir * spline_speed_,
            end_dir * spline_speed_);

        reference_spline_.update(times, spline_points, bc);
        reference_trajectory_ = reference_spline_.getTrajectory();
        return reference_trajectory_.isInitialized();
    }

    bool isSimilarPath(
        const std::vector<Eigen::Vector2d> & lhs,
        const std::vector<Eigen::Vector2d> & rhs) const
    {
        if (lhs.size() != rhs.size() || lhs.empty()) {
            return false;
        }

        double max_delta = 0.0;
        for (std::size_t i = 0; i < lhs.size(); ++i) {
            max_delta = std::max(max_delta, (lhs[i] - rhs[i]).norm());
            if (max_delta > duplicate_path_position_tol_) {
                return false;
            }
        }
        return true;
    }

    mpc_state::state sampleTrajectoryState(const Trajectory2D & trajectory, double t) const
    {
        const Eigen::Vector2d pos = trajectory.getPos(t);
        const Eigen::Vector2d vel = trajectory.getVel(t);
        const Eigen::Vector2d acc = trajectory.getAcc(t);

        double yaw = 0.0;
        if (vel.norm() > 1e-6) {
            yaw = std::atan2(vel.y(), vel.x());
        }

        double omega = 0.0;
        const double vel_sq = vel.squaredNorm();
        if (vel_sq > 1e-6) {
            omega = (vel.x() * acc.y() - vel.y() * acc.x()) / vel_sq;
        }

        mpc_state::state state = mpc_state::state::Zero();
        state << pos.x(), pos.y(), yaw, vel.x(), vel.y(), omega;
        return state;
    }

    void publishReferencePath()
    {
        if (!reference_trajectory_.isInitialized()) {
            return;
        }

        nav_msgs::msg::Path path_msg;
        path_msg.header.stamp = this->now();
        path_msg.header.frame_id = map_frame_;

        const double total_time = reference_trajectory_.getDuration();
        for (double t = 0.0; t < total_time + 1e-9; t += std::max(reference_sample_dt_, 0.02)) {
            const auto ref_state = sampleTrajectoryState(reference_trajectory_, t);
            geometry_msgs::msg::PoseStamped pose;
            pose.header = path_msg.header;
            pose.pose.position.x = ref_state(0);
            pose.pose.position.y = ref_state(1);
            pose.pose.orientation = quaternionFromYaw(ref_state(2));
            path_msg.poses.push_back(pose);
        }

        reference_path_pub_->publish(path_msg);
    }

    void stepSimulation()
    {
        if (!solver_ready_ || !trajectory_ready_) {
            return;
        }

        solver_.setCurrentState(sim_state_);

        mpc_state::control_sequence optimal_sequence;
        if (!solver_.solve(&optimal_sequence) || optimal_sequence.empty()) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                1000,
                "MPC solve failed or returned empty sequence");
            return;
        }

        const auto preview = solver_.generatePreviewReference(mpc_config_.dt, mpc_config_.max_preview_time);
        if (!preview.empty()) {
            current_reference_ = preview.front();
            publishPreviewPath(preview);
        }

        const mpc_state::state_control control = optimal_sequence.front();
        sim_state_(0) += sim_state_(3) * sim_dt_;
        sim_state_(1) += sim_state_(4) * sim_dt_;
        sim_state_(2) = normalizeAngle(sim_state_(2) + sim_state_(5) * sim_dt_);
        sim_state_(3) += (control(0) / std::max(mpc_config_.mass, 1e-6)) * sim_dt_;
        sim_state_(4) += (control(1) / std::max(mpc_config_.mass, 1e-6)) * sim_dt_;
        sim_state_(5) += (control(2) / std::max(mpc_config_.iz, 1e-6)) * sim_dt_;

        last_control_ = control;
        last_position_error_ = std::hypot(sim_state_(0) - current_reference_(0), sim_state_(1) - current_reference_(1));
        last_yaw_error_ = normalizeAngle(sim_state_(2) - current_reference_(2));
        publishState();
    }

    void publishPreviewPath(const mpc_state::ref_sequence & preview)
    {
        nav_msgs::msg::Path path_msg;
        path_msg.header.stamp = this->now();
        path_msg.header.frame_id = map_frame_;

        for (const auto & state : preview) {
            geometry_msgs::msg::PoseStamped pose;
            pose.header = path_msg.header;
            pose.pose.position.x = state(0);
            pose.pose.position.y = state(1);
            pose.pose.orientation = quaternionFromYaw(state(2));
            path_msg.poses.push_back(pose);
        }

        preview_path_pub_->publish(path_msg);
    }

    void publishState()
    {
        const auto stamp = this->now();

        nav_msgs::msg::Odometry odom;
        odom.header.stamp = stamp;
        odom.header.frame_id = map_frame_;
        odom.child_frame_id = base_frame_;
        odom.pose.pose.position.x = sim_state_(0);
        odom.pose.pose.position.y = sim_state_(1);
        odom.pose.pose.orientation = quaternionFromYaw(sim_state_(2));
        odom.twist.twist.linear.x = sim_state_(3);
        odom.twist.twist.linear.y = sim_state_(4);
        odom.twist.twist.angular.z = sim_state_(5);
        odom_pub_->publish(odom);

        geometry_msgs::msg::TransformStamped tf_msg;
        tf_msg.header = odom.header;
        tf_msg.child_frame_id = base_frame_;
        tf_msg.transform.translation.x = sim_state_(0);
        tf_msg.transform.translation.y = sim_state_(1);
        tf_msg.transform.rotation = odom.pose.pose.orientation;
        tf_broadcaster_->sendTransform(tf_msg);

        geometry_msgs::msg::PoseStamped pose;
        pose.header = odom.header;
        pose.pose = odom.pose.pose;
        actual_path_msg_.header = odom.header;
        actual_path_msg_.poses.push_back(pose);
        actual_path_pub_->publish(actual_path_msg_);

        publishMarkers(stamp);
    }

    void publishMarkers(const rclcpp::Time & stamp)
    {
        visualization_msgs::msg::MarkerArray markers;

        visualization_msgs::msg::Marker body;
        body.header.frame_id = map_frame_;
        body.header.stamp = stamp;
        body.ns = "mpc_sim";
        body.id = 0;
        body.type = visualization_msgs::msg::Marker::ARROW;
        body.action = visualization_msgs::msg::Marker::ADD;
        body.pose.position.x = sim_state_(0);
        body.pose.position.y = sim_state_(1);
        body.pose.position.z = 0.15;
        body.pose.orientation = quaternionFromYaw(sim_state_(2));
        body.scale.x = 0.7;
        body.scale.y = 0.18;
        body.scale.z = 0.18;
        body.color.r = 0.12F;
        body.color.g = 0.65F;
        body.color.b = 1.0F;
        body.color.a = 0.95F;
        markers.markers.push_back(body);

        visualization_msgs::msg::Marker text;
        text.header = body.header;
        text.ns = "mpc_status";
        text.id = 1;
        text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        text.action = visualization_msgs::msg::Marker::ADD;
        text.pose.position.x = sim_state_(0);
        text.pose.position.y = sim_state_(1);
        text.pose.position.z = 0.8;
        text.scale.z = 0.22;
        text.color.r = 1.0F;
        text.color.g = 1.0F;
        text.color.b = 1.0F;
        text.color.a = 0.95F;
        text.text =
            "e_p=" + std::to_string(last_position_error_).substr(0, 5) +
            " e_yaw=" + std::to_string(last_yaw_error_).substr(0, 5) +
            " u=[" + std::to_string(last_control_(0)).substr(0, 5) + "," +
            std::to_string(last_control_(1)).substr(0, 5) + "," +
            std::to_string(last_control_(2)).substr(0, 5) + "]";
        markers.markers.push_back(text);

        marker_pub_->publish(markers);
    }

    void printStatus()
    {
        if (!trajectory_ready_) {
            RCLCPP_INFO_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                2000,
                "waiting for optimized trajectory on %s",
                optimized_path_topic_.c_str());
            return;
        }

        const bool reached_goal =
            last_position_error_ < stop_position_tolerance_ &&
            std::hypot(sim_state_(3), sim_state_(4)) < stop_velocity_tolerance_;

        RCLCPP_INFO(
            this->get_logger(),
            "MPC status | pos=(%.3f, %.3f) yaw=%.3f ref=(%.3f, %.3f) err=%.3f yaw_err=%.3f "
            "vel=(%.3f, %.3f, %.3f) u=(%.3f, %.3f, %.3f) grad_norm=%.3f proj_t=%.3f reached=%s",
            sim_state_(0),
            sim_state_(1),
            sim_state_(2),
            current_reference_(0),
            current_reference_(1),
            last_position_error_,
            last_yaw_error_,
            sim_state_(3),
            sim_state_(4),
            sim_state_(5),
            last_control_(0),
            last_control_(1),
            last_control_(2),
            solver_.getGradient().norm(),
            solver_.getProjectedTime(),
            reached_goal ? "true" : "false");
    }

    std::string map_frame_;
    std::string base_frame_;
    std::string optimized_path_topic_;
    std::string odom_topic_;
    std::string actual_path_topic_;
    std::string reference_path_topic_;
    std::string preview_path_topic_;
    std::string marker_topic_;
    double spline_speed_{1.2};
    double sim_dt_{0.05};
    double status_rate_hz_{5.0};
    double reference_sample_dt_{0.05};
    bool reset_on_new_path_{true};
    double duplicate_path_position_tol_{0.05};
    double stop_position_tolerance_{0.15};
    double stop_velocity_tolerance_{0.12};

    mpc_slover::MpcConfig mpc_config_;
    mpc_slover::MpcSlover solver_;
    bool solver_ready_{false};
    bool trajectory_ready_{false};
    SplineTrajectory::CubicSpline2D reference_spline_;
    Trajectory2D reference_trajectory_;
    std::vector<Eigen::Vector2d> last_waypoints_;
    mpc_state::state sim_state_ = mpc_state::state::Zero();
    mpc_state::state current_reference_ = mpc_state::state::Zero();
    mpc_state::state_control last_control_ = mpc_state::state_control::Zero();
    double last_position_error_{0.0};
    double last_yaw_error_{0.0};
    nav_msgs::msg::Path actual_path_msg_;

    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr actual_path_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr reference_path_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr preview_path_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr sim_timer_;
    rclcpp::TimerBase::SharedPtr status_timer_;
};

}  // namespace

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MpcTrackingSimNode>());
    rclcpp::shutdown();
    return 0;
}
