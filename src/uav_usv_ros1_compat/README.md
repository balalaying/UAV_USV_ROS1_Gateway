# UAV-USV ROS 1 compatibility layer

This package provides a small ROS 1 node helper API backed only by native
`rospy`. It does not serialize, translate, or alter sensor messages.
Camera, camera-info, LaserScan, PointCloud2, custom interfaces, topic names,
timestamps, and frame IDs remain ROS 1 message objects end to end.
