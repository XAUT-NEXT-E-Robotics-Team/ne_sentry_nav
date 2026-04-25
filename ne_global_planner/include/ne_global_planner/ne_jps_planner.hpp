#ifndef NE_JPS_PLANNER_HPP
#define NE_JPS_PLANNER_HPP

#include <memory>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "nav_interfaces/msg/esdf.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "ne_esdf_builder/esdf_builder_node.hpp"
#include "ne_global_planner/ne_grid_search.hpp"
#include "ne_global_planner/ne_traj_optimize.hpp"

namespace jps_planner
{

class JpsPlannerNode : public rclcpp::Node
{
public:
    explicit JpsPlannerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
    ~JpsPlannerNode() override = default;

private:
    using Vector2d = Eigen::Vector2d;

    void init();
    void esdfCallback(const nav_interfaces::msg::Esdf::SharedPtr msg);
    bool makePlan(const nav_interfaces::msg::Esdf & msg);
    bool worldToIndex(const Vector2d & pos, Eigen::Vector2i & index) const;
    std::vector<Vector2d> expandGridPath(const std::vector<JPS::StatePtr> & grid_path) const;
    bool checkLineCollision(const Vector2d & start, const Vector2d & end) const;
    std::vector<Vector2d> optimizePath(const std::vector<Vector2d> & path) const;
    nav_msgs::msg::Path makePathMsg(
        const std::vector<Vector2d> & path,
        const std_msgs::msg::Header & header) const;
    visualization_msgs::msg::Marker makePathMarker(
        const std::vector<Vector2d> & path,
        const std_msgs::msg::Header & header,
        const std::string & ns,
        int id,
        float r,
        float g,
        float b,
        double z_offset) const;
    ne_traj_opt::TrajectoryParams makeTrajParams() const;

    rclcpp::Subscription<nav_interfaces::msg::Esdf>::SharedPtr esdf_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr jps_path_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr optimized_path_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr path_marker_pub_;

    std::shared_ptr<esdf_builder::ESDFMap2D> map_;

    std::string esdf_topic_{"/esdf_map"};
    std::string jps_path_topic_{"/jps_path"};
    std::string optimized_path_topic_{"/optimized_trajectory"};
    std::string path_marker_topic_{"/planner_path_markers"};

    Vector2d start_pos_{Vector2d::Zero()};
    Vector2d goal_pos_{Vector2d::Zero()};

    double safe_distance_{0.3};
    double traj_total_time_{10.0};
    double traj_piece_len_{4.0};
    double traj_sample_dt_{0.1};
    double path_z_offset_{0.08};
    double marker_line_width_{0.06};
    bool optimize_trajectory_{true};
    bool plan_on_map_update_{true};
    int max_expand_{-1};
};

}  // namespace jps_planner

#endif
