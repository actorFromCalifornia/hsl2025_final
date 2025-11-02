#include "coverage_node/coverage_node.hpp"
#include <cmath>
#include <thread>
#include <chrono>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Quaternion.h>

namespace coverage_node
{

// Helper function to calculate goal position relative to marker (same as in mark_detector_node)
tf2::Vector3 getGoal(tf2::Vector3 center, tf2::Quaternion rotation_quat) {
    tf2::Vector3 dirVector;
    dirVector.setX(0.5);
    dirVector.setY(0);
    dirVector.setZ(0);

    // Нормализация кватерниона (рекомендуется)
    rotation_quat.normalize();

    // Поворот вектора
    dirVector = tf2::quatRotate(rotation_quat, dirVector);
    return center + dirVector;
}

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
  grid_origin_y_(0.0),
  node_state_(NodeState::COVERAGE),
  pending_marker_id_(-1),
  canceling_for_marker_(false),
  canceling_for_new_cell_(false)
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
  grid_origin_y_ = 0.0 - 0.5 * cell_size_;
  
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/icp_odom", 10,
    std::bind(&CoverageNode::odom_callback, this, std::placeholders::_1));

  // Marker subscription
  marker_sub_ = this->create_subscription<visualization_msgs::msg::Marker>(
    "detected_marks", 10,
    std::bind(&CoverageNode::marker_callback, this, std::placeholders::_1));

  grid_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
    "/coverage_node/grid", 1);
  
  // Nav2 action client
  nav2_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
    this, "navigate_to_pose");
  
  // Timer for waiting at marker (5 seconds)
  marker_wait_timer_ = this->create_wall_timer(
    std::chrono::seconds(5),
    std::bind(&CoverageNode::marker_wait_timer_callback, this));
  marker_wait_timer_->cancel(); // Start canceled, will be activated when needed
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
  
  // Cancel previous goal if exists (mark that we're canceling for new cell, not error)
  if (current_goal_handle_) {
    // Save the cell that will be canceled before we change selected_cell_
    if (selected_cell_.has_value()) {
      canceled_cell_ = selected_cell_.value();
    }
    RCLCPP_DEBUG(this->get_logger(), "Cancelling previous goal for new cell (%d, %d)", cell.x, cell.y);
    canceling_for_new_cell_ = true;
    nav2_client_->async_cancel_goal(current_goal_handle_);
  }
  
  // Update selected cell before sending new goal
  selected_cell_ = cell;
  
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
      
      // If we just reached the marker, start waiting
      if (node_state_ == NodeState::GOING_TO_MARKER) {
        RCLCPP_INFO(this->get_logger(), "Marker ID %d reached, starting 5 second wait", pending_marker_id_);
        // Mark this marker as visited
        visited_marker_ids_.insert(pending_marker_id_);
        node_state_ = NodeState::WAITING_AT_MARKER;
        marker_wait_timer_->reset(); // Start the 5 second timer
        // Clear pending marker
        pending_marker_pose_.reset();
        pending_marker_id_ = -1;
      }
      // If we reached a cell goal and we're in coverage mode, continue coverage
      else if (node_state_ == NodeState::COVERAGE) {
        // Continue coverage - select next cell will be called by periodic timer
        RCLCPP_DEBUG(this->get_logger(), "Cell goal reached, continuing coverage");
      }
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_WARN(this->get_logger(), "Nav2 goal aborted");
      // Mark cell as aborted only if it was a real error (not canceled for marker or new cell)
      if (!canceling_for_marker_ && !canceling_for_new_cell_) {
        // Use canceled_cell_ if available (the cell that was actually canceled), otherwise use selected_cell_
        CellCoord cell_to_mark = canceled_cell_.has_value() ? canceled_cell_.value() : 
                                 (selected_cell_.has_value() ? selected_cell_.value() : CellCoord(-1, -1));
        if (cell_to_mark.x >= 0 && cell_to_mark.y >= 0) {
          aborted_cells_.insert(cell_to_mark);
          RCLCPP_INFO(this->get_logger(), "Cell (%d, %d) marked as aborted", 
                      cell_to_mark.x, cell_to_mark.y);
        }
      } else if (canceling_for_new_cell_) {
        RCLCPP_DEBUG(this->get_logger(), "Goal canceled for new cell - previous cell not marked as aborted");
      }
      // Clear canceled_cell_ and reset canceling flags
      canceled_cell_.reset();
      canceling_for_marker_ = false;
      canceling_for_new_cell_ = false;
      // If aborted while going to marker, return to coverage mode
      if (node_state_ == NodeState::GOING_TO_MARKER) {
        RCLCPP_WARN(this->get_logger(), "Marker ID %d goal aborted, returning to coverage mode", pending_marker_id_);
        node_state_ = NodeState::COVERAGE;
        pending_marker_pose_.reset();
        pending_marker_id_ = -1;
      }
      // Try to select a new cell if in coverage mode
      if (node_state_ == NodeState::COVERAGE) {
        select_next_cell();
      }
      break;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_INFO(this->get_logger(), "Nav2 goal canceled");
      // Mark cell as aborted only if it was a real error (not canceled for marker or new cell)
      if (!canceling_for_marker_ && !canceling_for_new_cell_) {
        // Use canceled_cell_ if available (the cell that was actually canceled), otherwise use selected_cell_
        CellCoord cell_to_mark = canceled_cell_.has_value() ? canceled_cell_.value() : 
                                 (selected_cell_.has_value() ? selected_cell_.value() : CellCoord(-1, -1));
        if (cell_to_mark.x >= 0 && cell_to_mark.y >= 0) {
          aborted_cells_.insert(cell_to_mark);
          RCLCPP_INFO(this->get_logger(), "Cell (%d, %d) marked as canceled", 
                      cell_to_mark.x, cell_to_mark.y);
        }
      } else if (canceling_for_marker_) {
        RCLCPP_DEBUG(this->get_logger(), "Goal canceled for marker - cell not marked as aborted");
      } else if (canceling_for_new_cell_) {
        RCLCPP_DEBUG(this->get_logger(), "Goal canceled for new cell - previous cell not marked as aborted");
      }
      // Clear canceled_cell_ and reset canceling flags
      canceled_cell_.reset();
      canceling_for_marker_ = false;
      canceling_for_new_cell_ = false;
      // If canceled while going to marker, return to coverage mode
      if (node_state_ == NodeState::GOING_TO_MARKER) {
        RCLCPP_INFO(this->get_logger(), "Marker ID %d goal canceled, returning to coverage mode", pending_marker_id_);
        node_state_ = NodeState::COVERAGE;
        pending_marker_pose_.reset();
        pending_marker_id_ = -1;
      }
      // Try to select a new cell if in coverage mode
      if (node_state_ == NodeState::COVERAGE) {
        select_next_cell();
      }
      break;
    default:
      RCLCPP_WARN(this->get_logger(), "Nav2 goal ended with unknown result code");
      break;
  }
  current_goal_handle_.reset();
}

void CoverageNode::select_next_cell()
{
  // Only select cells when in COVERAGE mode
  if (node_state_ != NodeState::COVERAGE) {
    return;
  }
  
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
    
    // Send goal to Nav2 if this is a new cell
    if (previous_selected != new_cell) {
      RCLCPP_INFO(this->get_logger(), "New cell selected, sending Nav2 goal");
      send_nav2_goal(new_cell);
    } else {
      RCLCPP_DEBUG(this->get_logger(), "Same cell selected again, not sending new goal");
    }
    // Note: selected_cell_ is updated inside send_nav2_goal after canceling previous goal
  } else {
    RCLCPP_INFO(this->get_logger(), "All cells visited! Coverage complete.");
    selected_cell_.reset();
  }
}

void CoverageNode::marker_callback(const visualization_msgs::msg::Marker::SharedPtr msg)
{
  // Process marker detection - only process CUBE markers (not SPHERE)
  if (msg->action == visualization_msgs::msg::Marker::ADD && 
      msg->ns == "marks" &&
      msg->type == visualization_msgs::msg::Marker::CUBE) {
    
    int marker_id = msg->id;
    
    // Skip if this marker has already been visited
    if (visited_marker_ids_.find(marker_id) != visited_marker_ids_.end()) {
      RCLCPP_DEBUG(this->get_logger(), "Marker ID %d already visited, ignoring", marker_id);
      return;
    }
    
    // Skip if we're already going to a marker (or waiting at one)
    if (node_state_ == NodeState::GOING_TO_MARKER || node_state_ == NodeState::WAITING_AT_MARKER) {
      RCLCPP_DEBUG(this->get_logger(), "Marker ID %d detected but already processing marker ID %d, ignoring", 
                   marker_id, pending_marker_id_);
      return;
    }
    
    RCLCPP_INFO(this->get_logger(), "Marker ID %d detected at (%.2f, %.2f, %.2f)", 
                marker_id, msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    
    // Cancel current goal if exists (mark that we're canceling for marker, not error)
    if (current_goal_handle_) {
      // Save the cell that will be canceled before we clear selected_cell_
      if (selected_cell_.has_value()) {
        canceled_cell_ = selected_cell_.value();
      }
      RCLCPP_INFO(this->get_logger(), "Cancelling current goal to go to marker ID %d", marker_id);
      canceling_for_marker_ = true;
      nav2_client_->async_cancel_goal(current_goal_handle_);
      // Don't reset current_goal_handle_ here - let the result callback handle it
    }
    
    // Clear selected cell since we're interrupting coverage (but don't mark as aborted)
    if (selected_cell_.has_value()) {
      RCLCPP_DEBUG(this->get_logger(), "Clearing selected cell to go to marker");
      selected_cell_.reset();
    }
    
    // Calculate goal position using getGoal method (offset 0.5m from marker center in marker direction)
    tf2::Vector3 center;
    center.setX(msg->pose.position.x);
    center.setY(msg->pose.position.y);
    center.setZ(msg->pose.position.z);
    
    tf2::Quaternion rotation_quat;
    rotation_quat.setX(msg->pose.orientation.x);
    rotation_quat.setY(msg->pose.orientation.y);
    rotation_quat.setZ(msg->pose.orientation.z);
    rotation_quat.setW(msg->pose.orientation.w);
    
    tf2::Vector3 goal_position = getGoal(center, rotation_quat);
    
    // Save marker goal position and ID
    geometry_msgs::msg::PoseStamped marker_pose;
    marker_pose.header = msg->header;
    marker_pose.pose.position.x = goal_position.x();
    marker_pose.pose.position.y = goal_position.y();
    marker_pose.pose.position.z = goal_position.z();
    // Use marker orientation for goal
    marker_pose.pose.orientation = msg->pose.orientation;
    
    pending_marker_pose_ = marker_pose;
    pending_marker_id_ = marker_id;
    
    RCLCPP_INFO(this->get_logger(), "Marker ID %d: center (%.2f, %.2f, %.2f) -> goal (%.2f, %.2f, %.2f)", 
                marker_id, 
                center.x(), center.y(), center.z(),
                goal_position.x(), goal_position.y(), goal_position.z());
    
    // Switch to marker mode and send goal immediately
    node_state_ = NodeState::GOING_TO_MARKER;
    RCLCPP_INFO(this->get_logger(), "Switching to marker mode, sending goal to marker ID %d", marker_id);
    send_nav2_goal_to_marker(marker_pose);
  }
}

void CoverageNode::send_nav2_goal_to_marker(const geometry_msgs::msg::PoseStamped& pose)
{
  if (!nav2_client_->wait_for_action_server(std::chrono::seconds(1))) {
    RCLCPP_WARN(this->get_logger(), "Nav2 action server not available, cannot send marker goal");
    node_state_ = NodeState::COVERAGE;
    return;
  }
  
  // Cancel previous goal if exists
  if (current_goal_handle_) {
    RCLCPP_DEBUG(this->get_logger(), "Cancelling previous goal before marker");
    nav2_client_->async_cancel_goal(current_goal_handle_);
  }
  
  auto goal_msg = nav2_msgs::action::NavigateToPose::Goal();
  goal_msg.pose = pose;
  
  RCLCPP_INFO(this->get_logger(), "Sending Nav2 goal to marker: pose (%.2f, %.2f, %.2f)", 
              pose.pose.position.x, 
              pose.pose.position.y,
              pose.pose.position.z);
  
  auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
  send_goal_options.goal_response_callback =
    std::bind(&CoverageNode::nav2_goal_response_callback, this, std::placeholders::_1);
  send_goal_options.feedback_callback =
    std::bind(&CoverageNode::nav2_feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
  send_goal_options.result_callback =
    std::bind(&CoverageNode::nav2_result_callback, this, std::placeholders::_1);
  
  nav2_client_->async_send_goal(goal_msg, send_goal_options);
  RCLCPP_INFO(this->get_logger(), "Nav2 goal to marker sent asynchronously");
}

void CoverageNode::marker_wait_timer_callback()
{
  RCLCPP_INFO(this->get_logger(), "Marker wait completed, returning to coverage mode");
  marker_wait_timer_->cancel();
  node_state_ = NodeState::COVERAGE;
  // Resume coverage algorithm
  select_next_cell();
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
