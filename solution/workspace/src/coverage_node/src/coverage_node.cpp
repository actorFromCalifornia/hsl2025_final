#include "coverage_node/coverage_node.hpp"
#include <cmath>
#include <thread>
#include <chrono>

namespace coverage_node
{

CoverageNode::CoverageNode()
: Node("coverage_node"),
  cell_size_(1.0),
  grid_width_cells_(4),
  grid_height_cells_(8),
  cell_selection_interval_(2.0),
  last_selection_time_(this->now()),
  visualization_interval_(0.5),
  last_visualization_time_(this->now()),
  grid_origin_x_(0.0),
  grid_origin_y_(0.0)
{
  // Declare parameters
  this->declare_parameter("grid_width_cells", 4);
  this->declare_parameter("grid_height_cells", 8);
  this->declare_parameter("cell_size", 1.0);
  
  // Get parameters
  grid_width_cells_ = this->get_parameter("grid_width_cells").as_int();
  grid_height_cells_ = this->get_parameter("grid_height_cells").as_int();
  cell_size_ = this->get_parameter("cell_size").as_double();
  
  // Grid origin is fixed - bottom-left corner of cell (0, 0) will be at world (0, 0)
  // No rotation needed - simple direct coordinates
  grid_origin_x_ = 0.0 - 0.25 * cell_size_;
  grid_origin_y_ = 0.0 - 1.5 * cell_size_;
  
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
  
  // Convert world coordinates to grid-local coordinates (no rotation)
  // Bottom-left corner of cell (0, 0) is at (0, 0)
  double local_x = x - grid_origin_x_;
  double local_y = y - grid_origin_y_;
  
  cell.x = static_cast<int>(std::floor(local_x / cell_size_));
  cell.y = static_cast<int>(std::floor(local_y / cell_size_));
  return cell;
}

// find_explored_region_corner removed - not used

void CoverageNode::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  double robot_x = msg->pose.pose.position.x;
  double robot_y = msg->pose.pose.position.y;

  // Use fixed grid origin (0, 0)
  CellCoord cell = world_to_cell(robot_x, robot_y, 0.0, 0.0);
  visited_cells_.insert(cell);
  current_robot_cell_ = cell;
  
  // Periodically select next cell for coverage
  rclcpp::Time current_time = this->now();
  if ((current_time - last_selection_time_).seconds() >= cell_selection_interval_) {
    select_next_cell();
    last_selection_time_ = current_time;
  }
  
  // Publish grid visualization (throttled)
  if ((current_time - last_visualization_time_).seconds() >= visualization_interval_) {
    publish_grid_visualization();
    last_visualization_time_ = current_time;
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
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "map";
  pose.header.stamp = this->now();
  
  // Grid-local coordinates (center of cell)
  double local_x = (cell.x + 0.5) * cell_size_;
  double local_y = (cell.y + 0.5) * cell_size_;
  
  // Convert to world coordinates (no rotation)
  pose.pose.position.x = grid_origin_x_ + local_x;
  pose.pose.position.y = grid_origin_y_ + local_y;
  pose.pose.position.z = 0.0;
  
  // No rotation - orientation stays at identity
  pose.pose.orientation.x = 0.0;
  pose.pose.orientation.y = 0.0;
  pose.pose.orientation.z = 0.0;
  pose.pose.orientation.w = 1.0;
  
  return pose;
}

void CoverageNode::send_nav2_goal(const CellCoord& cell)
{
  if (!nav2_client_->wait_for_action_server(std::chrono::seconds(1))) {
    RCLCPP_WARN(this->get_logger(), "Nav2 action server not available, skipping goal for cell (%d, %d)", cell.x, cell.y);
    return;
  }
  
  // Cancel previous goal if exists
  if (current_goal_handle_) {
    RCLCPP_DEBUG(this->get_logger(), "Cancelling previous goal");
    nav2_client_->async_cancel_goal(current_goal_handle_);
  }
  
  auto goal_msg = nav2_msgs::action::NavigateToPose::Goal();
  goal_msg.pose = cell_to_pose(cell);
  
  RCLCPP_INFO(this->get_logger(), "Setting Nav2 goal: cell (%d, %d) -> pose (%.2f, %.2f, %.2f)", 
              cell.x, cell.y, 
              goal_msg.pose.pose.position.x, 
              goal_msg.pose.pose.position.y,
              goal_msg.pose.pose.position.z);
  
  auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
  send_goal_options.goal_response_callback =
    std::bind(&CoverageNode::nav2_goal_response_callback, this, std::placeholders::_1);
  send_goal_options.feedback_callback =
    std::bind(&CoverageNode::nav2_feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
  send_goal_options.result_callback =
    std::bind(&CoverageNode::nav2_result_callback, this, std::placeholders::_1);
  
  nav2_client_->async_send_goal(goal_msg, send_goal_options);
  RCLCPP_INFO(this->get_logger(), "Nav2 goal sent asynchronously for cell (%d, %d)", cell.x, cell.y);
}

void CoverageNode::nav2_goal_response_callback(
  const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr & goal_handle)
{
  if (!goal_handle) {
    RCLCPP_WARN(this->get_logger(), "Nav2 goal was rejected");
    return;
  }
  current_goal_handle_ = goal_handle;
  RCLCPP_INFO(this->get_logger(), "Nav2 goal accepted");
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
      RCLCPP_INFO(this->get_logger(), "Nav2 goal succeeded - reached target");
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_WARN(this->get_logger(), "Nav2 goal aborted");
      // Mark current selected cell as aborted
      if (selected_cell_.has_value()) {
        aborted_cells_.insert(selected_cell_.value());
        RCLCPP_INFO(this->get_logger(), "Cell (%d, %d) marked as aborted", 
                    selected_cell_.value().x, selected_cell_.value().y);
      }
      // Try to select a new cell
      select_next_cell();
      break;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_INFO(this->get_logger(), "Nav2 goal canceled");
      // Mark current selected cell as aborted
      if (selected_cell_.has_value()) {
        aborted_cells_.insert(selected_cell_.value());
        RCLCPP_INFO(this->get_logger(), "Cell (%d, %d) marked as canceled", 
                    selected_cell_.value().x, selected_cell_.value().y);
      }
      // Try to select a new cell
      select_next_cell();
      break;
    default:
      RCLCPP_WARN(this->get_logger(), "Nav2 goal ended with unknown result code");
      break;
  }
  current_goal_handle_.reset();
}

void CoverageNode::select_next_cell()
{
  if (!current_robot_cell_.has_value()) {
    return;
  }

  // Use fixed grid size: 8x4 cells
  int cells_x = grid_width_cells_;
  int cells_y = grid_height_cells_;

  CellCoord robot_cell = current_robot_cell_.value();
  double min_distance = std::numeric_limits<double>::max();
  std::optional<CellCoord> best_cell;

  // Find nearest unvisited cell (skip visited and aborted cells)
  for (int x = 0; x < cells_x; ++x) {
    for (int y = 0; y < cells_y; ++y) {
      CellCoord cell(x, y);
      
      // Skip visited cells
      if (visited_cells_.find(cell) != visited_cells_.end()) {
        continue;
      }
      
      // Skip aborted cells (they will be shown in a different color)
      if (aborted_cells_.find(cell) != aborted_cells_.end()) {
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
    CellCoord new_cell = best_cell.value();
    
    RCLCPP_INFO(this->get_logger(), "Selected next cell: (%d, %d), distance: %.2f, robot at cell: (%d, %d)", 
                new_cell.x, new_cell.y, min_distance, robot_cell.x, robot_cell.y);
    
    selected_cell_ = best_cell;
    
    // Send goal to Nav2 if this is a new cell
    if (previous_selected != new_cell) {
      RCLCPP_INFO(this->get_logger(), "New cell selected, sending Nav2 goal");
      send_nav2_goal(new_cell);
    } else {
      RCLCPP_DEBUG(this->get_logger(), "Same cell selected again, not sending new goal");
    }
  } else {
    RCLCPP_INFO(this->get_logger(), "All cells visited! Coverage complete.");
    selected_cell_.reset();
  }
}

// Map callback removed - grid visualization published from odom_callback

void CoverageNode::publish_grid_visualization()
{
  // No DELETEALL - use unique marker IDs to update markers smoothly
  rclcpp::Time now = this->now();

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
      
      // Grid-local coordinates (center of cell)
      double local_x = (x + 0.5) * cell_size_;
      double local_y = (y + 0.5) * cell_size_;
      
      // Convert to world coordinates (no rotation)
      cell_marker.pose.position.x = grid_origin_x_ + local_x;
      cell_marker.pose.position.y = grid_origin_y_ + local_y;
      cell_marker.pose.position.z = 0.01;
      
      // No rotation - orientation stays at identity
      cell_marker.pose.orientation.x = 0.0;
      cell_marker.pose.orientation.y = 0.0;
      cell_marker.pose.orientation.z = 0.0;
      cell_marker.pose.orientation.w = 1.0;
      
      cell_marker.scale.x = cell_size_ * 0.9;
      cell_marker.scale.y = cell_size_ * 0.9;
      cell_marker.scale.z = 0.01;
      
      bool is_aborted = aborted_cells_.find(cell) != aborted_cells_.end();
      
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
      } else if (is_aborted) {
        // Aborted/canceled cells - orange/magenta
        cell_marker.color.r = 1.0;
        cell_marker.color.g = 0.5;
        cell_marker.color.b = 0.0;
        cell_marker.color.a = 0.5;
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
    
    // Grid-local coordinates for horizontal line (no rotation)
    double local_x1 = 0.0;
    double local_y1 = i * cell_size_;
    double local_x2 = cells_x * cell_size_;
    double local_y2 = i * cell_size_;
    
    // Convert to world coordinates (no rotation)
    p1.x = grid_origin_x_ + local_x1;
    p1.y = grid_origin_y_ + local_y1;
    p1.z = 0.0;
    p2.x = grid_origin_x_ + local_x2;
    p2.y = grid_origin_y_ + local_y2;
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
    
    // Grid-local coordinates for vertical line (no rotation)
    double local_x1 = i * cell_size_;
    double local_y1 = 0.0;
    double local_x2 = i * cell_size_;
    double local_y2 = cells_y * cell_size_;
    
    // Convert to world coordinates (no rotation)
    p1.x = grid_origin_x_ + local_x1;
    p1.y = grid_origin_y_ + local_y1;
    p1.z = 0.0;
    p2.x = grid_origin_x_ + local_x2;
    p2.y = grid_origin_y_ + local_y2;
    p2.z = 0.0;
      marker.points.push_back(p1);
      marker.points.push_back(p2);
      grid_msg->markers.push_back(marker);
    }

    grid_pub_->publish(*grid_msg);
}

}  // namespace coverage_node
