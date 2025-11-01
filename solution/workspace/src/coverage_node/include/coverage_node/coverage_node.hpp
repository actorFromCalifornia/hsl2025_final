#ifndef COVERAGE_NODE__COVERAGE_NODE_HPP_
#define COVERAGE_NODE__COVERAGE_NODE_HPP_

#include <atomic>
#include <unordered_set>
#include <utility>
#include <optional>
#include <limits>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>

namespace coverage_node
{

struct CellCoord {
  int x;
  int y;
  
  CellCoord() : x(0), y(0) {}
  CellCoord(int x_val, int y_val) : x(x_val), y(y_val) {}
  
  bool operator==(const CellCoord& other) const {
    return x == other.x && y == other.y;
  }
  
  bool operator!=(const CellCoord& other) const {
    return !(*this == other);
  }
};

struct CellHash {
  std::size_t operator()(const CellCoord& c) const {
    return std::hash<int>()(c.x) ^ (std::hash<int>()(c.y) << 1);
  }
};

class CoverageNode : public rclcpp::Node
{
public:
  CoverageNode();

private:
  void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void publish_grid_visualization(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void select_next_cell();
  void find_explored_region_corner(const nav_msgs::msg::OccupancyGrid::SharedPtr msg, double& corner_x, double& corner_y);
  void send_nav2_goal(const CellCoord& cell);
  void nav2_goal_response_callback(
    const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr & goal_handle);
  void nav2_feedback_callback(
    rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr,
    const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback>);
  void nav2_result_callback(
    const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult & result);
  CellCoord world_to_cell(double x, double y, double origin_x, double origin_y) const;
  geometry_msgs::msg::PoseStamped cell_to_pose(const CellCoord& cell) const;
  double cell_distance(const CellCoord& cell1, const CellCoord& cell2) const;

  // Subscribers and publishers
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr grid_pub_;
  
  // Nav2 action client
  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr nav2_client_;
  rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr current_goal_handle_;

  // Grid parameters
  double cell_size_;
  int grid_width_cells_;
  int grid_height_cells_;

  // Coverage tracking
  std::unordered_set<CellCoord, CellHash> visited_cells_;
  std::optional<CellCoord> selected_cell_;
  std::optional<CellCoord> current_robot_cell_;
  nav_msgs::msg::OccupancyGrid::SharedPtr last_map_;
  
  // Grid origin - aligned to explored region corner (set once on first map)
  bool grid_origin_set_;
  double grid_origin_x_;
  double grid_origin_y_;
  
  // Coverage algorithm parameters
  double cell_selection_interval_;
  rclcpp::Time last_selection_time_;

  // Throttle updates
  rclcpp::Time last_update_time_;
  double min_update_interval_;
  std::atomic<bool> is_publishing_;
};

}  // namespace coverage_node

#endif  // COVERAGE_NODE__COVERAGE_NODE_HPP_

