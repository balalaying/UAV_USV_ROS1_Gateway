#include <ros/ros.h>

#include "uav_usv_lv_dot/detector_node.hpp"

int main(int argc, char **argv) {
  ros::init(argc, argv, "lv_dot_detector_node");
  uav_usv_lv_dot::DetectorNode node;
  ros::spin();
  return 0;
}
