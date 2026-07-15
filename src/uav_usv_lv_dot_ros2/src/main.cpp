#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "uav_usv_lv_dot_ros2/detector_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<uav_usv_lv_dot_ros2::DetectorNode>();
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
