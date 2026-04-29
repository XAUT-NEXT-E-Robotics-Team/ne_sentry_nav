#include <algorithm>
#include <chrono>
#include <string>
#include <vector>

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{

class OccupancyGridSimNode : public rclcpp::Node
{
public:
    OccupancyGridSimNode()
    : rclcpp::Node("occupancy_grid_sim_node")
    {
        topic_ = this->declare_parameter<std::string>("topic", "/terrain_analysis/grid");
        frame_id_ = this->declare_parameter<std::string>("frame_id", "world");
        resolution_ = this->declare_parameter<double>("resolution", 0.1);
        width_ = this->declare_parameter<int>("width", 120);
        height_ = this->declare_parameter<int>("height", 80);
        origin_x_ = this->declare_parameter<double>("origin_x", -2.0);
        origin_y_ = this->declare_parameter<double>("origin_y", -4.0);
        publish_rate_hz_ = this->declare_parameter<double>("publish_rate_hz", 2.0);
        add_border_walls_ = this->declare_parameter<bool>("add_border_walls", true);
        publish_once_ = this->declare_parameter<bool>("publish_once", true);
        obstacle_rects_ = this->declare_parameter<std::vector<double>>(
            "obstacle_rects",
            std::vector<double>{2.0, -0.8, 2.6, 2.8, 4.1, -2.8, 4.7, 0.6, 6.1, 1.1, 6.8, 4.2});

        grid_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(topic_, rclcpp::QoS(1).transient_local());
        timer_ = this->create_wall_timer(
            std::chrono::duration<double>(1.0 / std::max(publish_rate_hz_, 0.1)),
            std::bind(&OccupancyGridSimNode::publishGrid, this));

        RCLCPP_INFO(
            this->get_logger(),
            "occupancy grid sim ready: topic=%s frame=%s size=%dx%d resolution=%.3f",
            topic_.c_str(),
            frame_id_.c_str(),
            width_,
            height_,
            resolution_);
    }

private:
    void setOccupied(nav_msgs::msg::OccupancyGrid & grid, int x, int y) const
    {
        if (x < 0 || y < 0 || x >= width_ || y >= height_) {
            return;
        }
        const std::size_t idx = static_cast<std::size_t>(y) * static_cast<std::size_t>(width_) +
            static_cast<std::size_t>(x);
        grid.data[idx] = 100;
    }

    void fillRectangle(nav_msgs::msg::OccupancyGrid & grid, double xmin, double ymin, double xmax, double ymax) const
    {
        const int x0 = static_cast<int>(std::floor((std::min(xmin, xmax) - origin_x_) / resolution_));
        const int x1 = static_cast<int>(std::ceil((std::max(xmin, xmax) - origin_x_) / resolution_));
        const int y0 = static_cast<int>(std::floor((std::min(ymin, ymax) - origin_y_) / resolution_));
        const int y1 = static_cast<int>(std::ceil((std::max(ymin, ymax) - origin_y_) / resolution_));

        for (int x = x0; x <= x1; ++x) {
            for (int y = y0; y <= y1; ++y) {
                setOccupied(grid, x, y);
            }
        }
    }

    void publishGrid()
    {
        nav_msgs::msg::OccupancyGrid grid;
        grid.header.stamp = this->now();
        grid.header.frame_id = frame_id_;
        grid.info.map_load_time = grid.header.stamp;
        grid.info.resolution = static_cast<float>(resolution_);
        grid.info.width = static_cast<uint32_t>(width_);
        grid.info.height = static_cast<uint32_t>(height_);
        grid.info.origin.position.x = origin_x_;
        grid.info.origin.position.y = origin_y_;
        grid.info.origin.orientation.w = 1.0;
        grid.data.assign(static_cast<std::size_t>(width_) * static_cast<std::size_t>(height_), 0);

        if (add_border_walls_) {
            for (int x = 0; x < width_; ++x) {
                setOccupied(grid, x, 0);
                setOccupied(grid, x, height_ - 1);
            }
            for (int y = 0; y < height_; ++y) {
                setOccupied(grid, 0, y);
                setOccupied(grid, width_ - 1, y);
            }
        }

        for (std::size_t i = 0; i + 3 < obstacle_rects_.size(); i += 4) {
            fillRectangle(
                grid,
                obstacle_rects_[i + 0],
                obstacle_rects_[i + 1],
                obstacle_rects_[i + 2],
                obstacle_rects_[i + 3]);
        }

        grid_pub_->publish(grid);

        if (publish_once_ && timer_) {
            timer_->cancel();
        }
    }

    std::string topic_;
    std::string frame_id_;
    double resolution_{0.1};
    int width_{120};
    int height_{80};
    double origin_x_{-2.0};
    double origin_y_{-4.0};
    double publish_rate_hz_{2.0};
    bool add_border_walls_{true};
    bool publish_once_{true};
    std::vector<double> obstacle_rects_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OccupancyGridSimNode>());
    rclcpp::shutdown();
    return 0;
}
