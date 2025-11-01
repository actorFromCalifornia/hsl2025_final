#include <rclcpp/rclcpp.hpp>
#include "coverage_node/coverage_node.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<coverage_node::CoverageNode>());
  rclcpp::shutdown();
  return 0;
}

