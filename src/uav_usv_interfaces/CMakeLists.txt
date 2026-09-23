cmake_minimum_required(VERSION 3.0.2)
project(uav_usv_interfaces)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  geographic_msgs
  geometry_msgs
  std_msgs
  uuid_msgs
)

add_message_files(
  FILES
  AisContact.msg
  AisContactArray.msg
  TrackedObject.msg
  TrackedObjectArray.msg
  AffiliatedDetection2D.msg
  AffiliatedDetection2DArray.msg
  CollisionRisk.msg
  ColregsDecision.msg
  VehicleState.msg
  ControlLease.msg
  FleetCommand.msg
  CommandAck.msg
  SensorStatus.msg
  CaptureState.msg
  CaptureAssignment.msg
  CaptureAssignmentArray.msg
  CaptureTargetStatus.msg
)

generate_messages(
  DEPENDENCIES
  geographic_msgs
  geometry_msgs
  std_msgs
  uuid_msgs
)

catkin_package(
  CATKIN_DEPENDS
  message_runtime
  geographic_msgs
  geometry_msgs
  std_msgs
  uuid_msgs
)
