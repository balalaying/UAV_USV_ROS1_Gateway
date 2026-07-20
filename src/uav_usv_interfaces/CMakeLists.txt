cmake_minimum_required(VERSION 3.8)
project(uav_usv_interfaces)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geographic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(unique_identifier_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/AisContact.msg"
  "msg/AisContactArray.msg"
  "msg/TrackedObject.msg"
  "msg/TrackedObjectArray.msg"
  "msg/AffiliatedDetection2D.msg"
  "msg/AffiliatedDetection2DArray.msg"
  "msg/CollisionRisk.msg"
  "msg/ColregsDecision.msg"
  "msg/VehicleState.msg"
  "msg/ControlLease.msg"
  "msg/FleetCommand.msg"
  "msg/CommandAck.msg"
  "msg/SensorStatus.msg"
  "msg/CaptureState.msg"
  "msg/CaptureAssignment.msg"
  "msg/CaptureAssignmentArray.msg"
  "msg/CaptureTargetStatus.msg"
  DEPENDENCIES
    builtin_interfaces
    geographic_msgs
    geometry_msgs
    std_msgs
    unique_identifier_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
