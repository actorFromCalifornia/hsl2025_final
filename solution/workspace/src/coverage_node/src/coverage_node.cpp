#include "coverage_node/coverage_node.hpp"
#include <cmath>
#include <thread>
#include <chrono>
#include <tf2/LinearMath/Quaternion.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace coverage_node
{

CoverageNode::CoverageNode()
: Node("coverage_node"),
  cell_size_(1.0),
  grid_width_cells_(8),
  grid_height_cells_(4),
  last_update_time_(this->now()),
  min_update_interval_(1.0),
  cell_selection_interval_(2.0),
  last_selection_time_(this->now()),
  is_publishing_(false),
  grid_origin_set_(false),
  grid_origin_x_(0.0),
  grid_origin_y_(0.0)
{
  // Declare parameters
  this->declare_parameter("grid_width_cells", 8);
  this->declare_parameter("grid_height_cells", 4);
  this->declare_parameter("cell_size", 1.0);
  
  // Get parameters
  grid_width_cells_ = this->get_parameter("grid_width_cells").as_int();
  grid_height_cells_ = this->get_parameter("grid_height_cells").as_int();
  cell_size_ = this->get_parameter("cell_size").as_double();
  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/rtabmap_map", 1,
    std::bind(&CoverageNode::map_callback, this, std::placeholders::_1));
  
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/icp_odom", 10,
    std::bind(&CoverageNode::odom_callback, this, std::placeholders::_1));
  
  grid_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
    "/coverage_node/grid", 1);
  
  // Nav2 action client
  nav2_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
    this, "navigate_to_pose");
}

CellCoord CoverageNode::world_to_cell(double x, double y, double origin_x, double origin_y) const
{
  CellCoord cell;
  
  if (!grid_origin_set_) {
    // Fallback to provided origin if grid origin not set yet
    double local_x = y - origin_y;
    double local_y = -(x - origin_x);
    cell.x = static_cast<int>(std::floor(local_x / cell_size_));
    cell.y = static_cast<int>(std::floor(local_y / cell_size_));
    return cell;
  }
  
  // Rotate world coordinates back by -90 degrees to get grid-local coordinates
  // Rotation: (x, y) -> (y, -x) for -90 degrees
  double local_x = y - grid_origin_y_;
  double local_y = -(x - grid_origin_x_);
  
  cell.x = static_cast<int>(std::floor(local_x / cell_size_));
  cell.y = static_cast<int>(std::floor(local_y / cell_size_));
  return cell;
}

void CoverageNode::find_explored_region_corner(const nav_msgs::msg::OccupancyGrid::SharedPtr msg, double& corner_x, double& corner_y)
{
  double resolution = msg->info.resolution;
  int width = msg->info.width;
  int height = msg->info.height;
  
  // Find the explored region bounds (min and max)
  int min_x = width;
  int min_y = height;
  bool has_explored = false;
  
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      int index = y * width + x;
      int8_t cell_value = msg->data[index];
      
      // Check if cell is explored (not unknown, which is -1)
      if (cell_value != -1) {
        has_explored = true;
        if (x < min_x) min_x = x;
        if (y < min_y) min_y = y;
      }
    }
  }
  
  // Convert to world coordinates - this is the bottom-left corner of explored region
  if (has_explored && min_x < width && min_y < height) {
    // Convert grid coordinates to world coordinates
    corner_x = msg->info.origin.position.x + min_x * resolution;
    corner_y = msg->info.origin.position.y + min_y * resolution;
  } else {
    // Fallback to map origin position
    corner_x = msg->info.origin.position.x;
    corner_y = msg->info.origin.position.y;
  }
}

void CoverageNode::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  if (!last_map_) {
    return;
  }

  double robot_x = msg->pose.pose.position.x;
  double robot_y = msg->pose.pose.position.y;
  double origin_x = last_map_->info.origin.position.x;
  double origin_y = last_map_->info.origin.position.y;

  CellCoord cell = world_to_cell(robot_x, robot_y, origin_x, origin_y);
  visited_cells_.insert(cell);
  current_robot_cell_ = cell;
  
  // Periodically select next cell for coverage
  rclcpp::Time current_time = this->now();
  if ((current_time - last_selection_time_).seconds() >= cell_selection_interval_) {
    select_next_cell();
    last_selection_time_ = current_time;
  }
}

double CoverageNode::cell_distance(const CellCoord& cell1, const CellCoord& cell2) const
{
  double dx = cell1.x - cell2.x;
  double dy = cell1.y - cell2.y;
  return std::sqrt(dx * dx + dy * dy);
}

geometry_msgs::msg::PoseStamped CoverageNode::cell_to_pose(const CellCoord& cell) const
{
  if (!last_map_) {
    geometry_msgs::msg::PoseStamped pose;
    return pose;
  }
  
  if (!grid_origin_set_) {
    geometry_msgs::msg::PoseStamped pose;
    return pose;
  }
  
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "map";
  pose.header.stamp = this->now();
  
  // Grid-local coordinates
  double local_x = (cell.x + 0.5) * cell_size_;
  double local_y = (cell.y + 0.5) * cell_size_;
  
  // Rotate by 90 degrees counter-clockwise: (x, y) -> (-y, x)
  double rot_x = -local_y;
  double rot_y = local_x;
  
  // Center of the cell in world coordinates (using grid origin)
  pose.pose.position.x = grid_origin_x_ + rot_x;
  pose.pose.position.y = grid_origin_y_ + rot_y;
  pose.pose.position.z = 0.0;
  
  // Orientation - 90 degrees rotation
  tf2::Quaternion rot_90;
  rot_90.setRPY(0, 0, M_PI / 2.0);
  pose.pose.orientation.x = rot_90.x();
  pose.pose.orientation.y = rot_90.y();
  pose.pose.orientation.z = rot_90.z();
  pose.pose.orientation.w = rot_90.w();
  
  return pose;
}

void CoverageNode::send_nav2_goal(const CellCoord& cell)
{
  if (!nav2_client_->wait_for_action_server(std::chrono::seconds(1))) {
    return;
  }
  
  // Cancel previous goal if exists
  if (current_goal_handle_) {
    nav2_client_->async_cancel_goal(current_goal_handle_);
  }
  
  auto goal_msg = nav2_msgs::action::NavigateToPose::Goal();
  goal_msg.pose = cell_to_pose(cell);
  
  auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
  send_goal_options.goal_response_callback =
    std::bind(&CoverageNode::nav2_goal_response_callback, this, std::placeholders::_1);
  send_goal_options.feedback_callback =
    std::bind(&CoverageNode::nav2_feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
  send_goal_options.result_callback =
    std::bind(&CoverageNode::nav2_result_callback, this, std::placeholders::_1);
  
  nav2_client_->async_send_goal(goal_msg, send_goal_options);
}

void CoverageNode::nav2_goal_response_callback(
  const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr & goal_handle)
{
  if (!goal_handle) {
    return;
  }
  current_goal_handle_ = goal_handle;
}

void CoverageNode::nav2_feedback_callback(
  rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr,
  const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback>)
{
  // Feedback can be processed here if needed
}

void CoverageNode::nav2_result_callback(
  const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult & result)
{
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      // Goal reached - mark cell as visited if needed
      break;
    case rclcpp_action::ResultCode::ABORTED:
    case rclcpp_action::ResultCode::CANCELED:
      // Goal failed or canceled
      break;
    default:
      break;
  }
  current_goal_handle_.reset();
}

void CoverageNode::select_next_cell()
{
  if (!last_map_ || !current_robot_cell_.has_value() || !grid_origin_set_) {
    return;
  }

  // Use fixed grid size: 8x4 cells
  int cells_x = grid_width_cells_;
  int cells_y = grid_height_cells_;

  CellCoord robot_cell = current_robot_cell_.value();
  double min_distance = std::numeric_limits<double>::max();
  std::optional<CellCoord> best_cell;

  // Find nearest unvisited cell
  for (int x = 0; x < cells_x; ++x) {
    for (int y = 0; y < cells_y; ++y) {
      CellCoord cell(x, y);
      
      // Skip visited cells
      if (visited_cells_.find(cell) != visited_cells_.end()) {
        continue;
      }
      
      // Calculate distance to robot's current cell
      double distance = cell_distance(cell, robot_cell);
      
      // Check if this cell is closer than current best
      if (distance < min_distance) {
        min_distance = distance;
        best_cell = cell;
      }
    }
  }

  if (best_cell.has_value()) {
    CellCoord previous_selected = selected_cell_.value_or(CellCoord(-1, -1));
    selected_cell_ = best_cell;
    
    // Send goal to Nav2 if this is a new cell
    if (previous_selected != best_cell.value()) {
      send_nav2_goal(best_cell.value());
    }
  } else {
    selected_cell_.reset();
  }
}

void CoverageNode::map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  bool expected = false;
  if (!is_publishing_.compare_exchange_strong(expected, true)) {
    return;
  }

  rclcpp::Time current_time = this->now();
  if ((current_time - last_update_time_).seconds() < min_update_interval_) {
    is_publishing_ = false;
    return;
  }

  last_update_time_ = current_time;
  last_map_ = msg;
  
  // Set grid origin to map origin only once on first map reception
  if (!grid_origin_set_) {
    // Use map origin directly, shift by 5 cells on X axis
    grid_origin_x_ = msg->info.origin.position.x + 5.3 * cell_size_;
    grid_origin_y_ = msg->info.origin.position.y;
    grid_origin_set_ = true;
  }
  
  publish_grid_visualization(msg);
  is_publishing_ = false;
}

void CoverageNode::publish_grid_visualization(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  if (!grid_origin_set_) {
    return;
  }
  
  rclcpp::Time now = this->now();
  
  // First, delete all previous markers
  auto delete_msg = std::make_shared<visualization_msgs::msg::MarkerArray>();
  visualization_msgs::msg::Marker delete_all;
  delete_all.header.frame_id = "map";
  delete_all.header.stamp = now;
  delete_all.action = visualization_msgs::msg::Marker::DELETEALL;
  delete_msg->markers.push_back(delete_all);
  grid_pub_->publish(*delete_msg);
  
  // Small delay to ensure DELETEALL is processed
  std::this_thread::sleep_for(std::chrono::milliseconds(10));

  // Use fixed grid size: 8x4 cells
  int cells_x = grid_width_cells_;
  int cells_y = grid_height_cells_;
  
  auto grid_msg = std::make_shared<visualization_msgs::msg::MarkerArray>();
  grid_msg->markers.reserve(cells_y + 1 + cells_x + 1 + cells_x * cells_y);  // Lines + cells

  int marker_id = 0;

  // Cell status markers (unknown, visited, and selected cells)
  for (int x = 0; x < cells_x; ++x) {
    for (int y = 0; y < cells_y; ++y) {
      CellCoord cell(x, y);
      bool is_visited = visited_cells_.find(cell) != visited_cells_.end();
      bool is_selected = selected_cell_.has_value() && selected_cell_.value() == cell;
      
      visualization_msgs::msg::Marker cell_marker;
      cell_marker.header.frame_id = "map";
      cell_marker.header.stamp = now;
      cell_marker.ns = "coverage_cells";
      cell_marker.id = marker_id++;
      cell_marker.type = visualization_msgs::msg::Marker::CUBE;
      cell_marker.action = visualization_msgs::msg::Marker::ADD;
      
      // Grid-local coordinates
      double local_x = (x + 0.5) * cell_size_;
      double local_y = (y + 0.5) * cell_size_;
      
      // Rotate by 90 degrees counter-clockwise: (x, y) -> (-y, x)
      double rot_x = -local_y;
      double rot_y = local_x;
      
      // Use grid origin (explored region corner)
      cell_marker.pose.position.x = grid_origin_x_ + rot_x;
      cell_marker.pose.position.y = grid_origin_y_ + rot_y;
      cell_marker.pose.position.z = 0.01;
      
      // Orientation - 90 degrees rotation
      tf2::Quaternion rot_90;
      rot_90.setRPY(0, 0, M_PI / 2.0);
      cell_marker.pose.orientation.x = rot_90.x();
      cell_marker.pose.orientation.y = rot_90.y();
      cell_marker.pose.orientation.z = rot_90.z();
      cell_marker.pose.orientation.w = rot_90.w();
      
      cell_marker.scale.x = cell_size_ * 0.9;
      cell_marker.scale.y = cell_size_ * 0.9;
      cell_marker.scale.z = 0.01;
      
      if (is_selected) {
        // Selected cell - red
        cell_marker.color.r = 1.0;
        cell_marker.color.g = 0.0;
        cell_marker.color.b = 0.0;
        cell_marker.color.a = 0.6;
      } else if (is_visited) {
        // Visited cells - green
        cell_marker.color.r = 0.0;
        cell_marker.color.g = 1.0;
        cell_marker.color.b = 0.0;
        cell_marker.color.a = 0.3;
      } else {
        // Unknown cells - yellow
        cell_marker.color.r = 1.0;
        cell_marker.color.g = 1.0;
        cell_marker.color.b = 0.0;
        cell_marker.color.a = 0.3;
      }
      
      grid_msg->markers.push_back(cell_marker);
    }
  }

  // Horizontal lines
  for (int i = 0; i <= cells_y; ++i) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = now;
    marker.ns = "grid_h";
    marker.id = marker_id++;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.05;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;
    marker.color.a = 0.5;
    
    geometry_msgs::msg::Point p1, p2;
    
    // Grid-local coordinates for horizontal line
    double local_x1 = 0.0;
    double local_y1 = i * cell_size_;
    double local_x2 = cells_x * cell_size_;
    double local_y2 = i * cell_size_;
    
    // Rotate by 90 degrees counter-clockwise: (x, y) -> (-y, x)
    p1.x = grid_origin_x_ + (-local_y1);
    p1.y = grid_origin_y_ + local_x1;
    p1.z = 0.0;
    p2.x = grid_origin_x_ + (-local_y2);
    p2.y = grid_origin_y_ + local_x2;
    p2.z = 0.0;
    marker.points.push_back(p1);
    marker.points.push_back(p2);
    grid_msg->markers.push_back(marker);
  }

  // Vertical lines
  for (int i = 0; i <= cells_x; ++i) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = now;
    marker.ns = "grid_v";
    marker.id = marker_id++;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.05;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;
    marker.color.a = 0.5;
    
    geometry_msgs::msg::Point p1, p2;
    
    // Grid-local coordinates for vertical line
    double local_x1 = i * cell_size_;
    double local_y1 = 0.0;
    double local_x2 = i * cell_size_;
    double local_y2 = cells_y * cell_size_;
    
    // Rotate by 90 degrees counter-clockwise: (x, y) -> (-y, x)
    p1.x = grid_origin_x_ + (-local_y1);
    p1.y = grid_origin_y_ + local_x1;
    p1.z = 0.0;
    p2.x = grid_origin_x_ + (-local_y2);
    p2.y = grid_origin_y_ + local_x2;
    p2.z = 0.0;
    marker.points.push_back(p1);
    marker.points.push_back(p2);
    grid_msg->markers.push_back(marker);
  }

  grid_pub_->publish(*grid_msg);
}

}  // namespace coverage_node

