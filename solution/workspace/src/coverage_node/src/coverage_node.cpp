#include "coverage_node/coverage_node.hpp"
#include <cmath>
#include <thread>
#include <chrono>

namespace coverage_node
{

CoverageNode::CoverageNode()
: Node("coverage_node"),
  cell_size_(1.0),
  last_update_time_(this->now()),
  min_update_interval_(1.0),
  cell_selection_interval_(2.0),
  last_selection_time_(this->now()),
  is_publishing_(false)
{
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
  cell.x = static_cast<int>(std::floor((x - origin_x) / cell_size_));
  cell.y = static_cast<int>(std::floor((y - origin_y) / cell_size_));
  return cell;
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
  
  double origin_x = last_map_->info.origin.position.x;
  double origin_y = last_map_->info.origin.position.y;
  
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "map";
  pose.header.stamp = this->now();
  
  // Center of the cell
  pose.pose.position.x = origin_x + (cell.x + 0.5) * cell_size_;
  pose.pose.position.y = origin_y + (cell.y + 0.5) * cell_size_;
  pose.pose.position.z = 0.0;
  
  // Orientation - facing forward (no rotation)
  pose.pose.orientation.w = 1.0;
  
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
  if (!last_map_ || !current_robot_cell_.has_value()) {
    return;
  }

  double resolution = last_map_->info.resolution;
  double origin_x = last_map_->info.origin.position.x;
  double origin_y = last_map_->info.origin.position.y;
  double map_width_m = last_map_->info.width * resolution;
  double map_height_m = last_map_->info.height * resolution;
  
  int cells_x = static_cast<int>(std::floor(map_width_m / cell_size_));
  int cells_y = static_cast<int>(std::floor(map_height_m / cell_size_));

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
  publish_grid_visualization(msg);
  is_publishing_ = false;
}

void CoverageNode::publish_grid_visualization(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
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

  // Now create new grid
  double resolution = msg->info.resolution;
  double origin_x = msg->info.origin.position.x;
  double origin_y = msg->info.origin.position.y;
  double map_width_m = msg->info.width * resolution;
  double map_height_m = msg->info.height * resolution;
  
  int cells_x = static_cast<int>(std::floor(map_width_m / cell_size_));
  int cells_y = static_cast<int>(std::floor(map_height_m / cell_size_));

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
      
      cell_marker.pose.position.x = origin_x + (x + 0.5) * cell_size_;
      cell_marker.pose.position.y = origin_y + (y + 0.5) * cell_size_;
      cell_marker.pose.position.z = 0.01;
      cell_marker.pose.orientation.w = 1.0;
      
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
    p1.x = origin_x;
    p1.y = origin_y + i * cell_size_;
    p1.z = 0.0;
    p2.x = origin_x + map_width_m;
    p2.y = origin_y + i * cell_size_;
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
    p1.x = origin_x + i * cell_size_;
    p1.y = origin_y;
    p1.z = 0.0;
    p2.x = origin_x + i * cell_size_;
    p2.y = origin_y + map_height_m;
    p2.z = 0.0;
    marker.points.push_back(p1);
    marker.points.push_back(p2);
    grid_msg->markers.push_back(marker);
  }

  grid_pub_->publish(*grid_msg);
}

}  // namespace coverage_node

