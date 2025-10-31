cmake_minimum_required(VERSION 3.8)
project(mark_detector)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(OpenCV 4.12 REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common segmentation kdtree filters)

# Include directories
include_directories(include)

# Node executable
add_executable(mark_detector_node
  src/mark_detector_node.cpp
  src/geom.cpp
  src/mark_recognizer.cpp
)

link_directories(${PCL_LIBRARY_DIRS})
include_directories(${PCL_INCLUDE_DIRS})

ament_target_dependencies(mark_detector_node 
  rclcpp 
  sensor_msgs 
  nav_msgs 
  tf2_msgs 
  tf2_ros 
  tf2_geometry_msgs 
  visualization_msgs
  pcl_conversions  
)
target_link_libraries (mark_detector_node 
  ${rclcpp_LIBRARIES}
  ${sensor_msgs_LIBRARIES}
  ${geometry_msgs_LIBRARIES}
  ${tf2_geometry_msgs_LIBRARIES}
  ${std_msgs_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
)

# Link dependencies
ament_target_dependencies(mark_detector_node
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs
  pcl_conversions
  tf2_geometry_msgs
)

# Install
install(TARGETS
  mark_detector_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install header files
install(DIRECTORY include/
  DESTINATION include/
)

ament_package()
