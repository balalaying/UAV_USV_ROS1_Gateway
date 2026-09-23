#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

#include <gz/transport/Node.hh>
#include <gz/msgs/laserscan.pb.h>

#include <string>

class GzLaserScanBridge
{
public:
  GzLaserScanBridge()
  {
    ros::NodeHandle pnh("~");

    pnh.param<std::string>(
        "gz_topic",
        gz_topic_,
        "/boat/scan");

    pnh.param<std::string>(
        "ros_topic",
        ros_topic_,
        "/boat/scan_raw");

    pnh.param<std::string>(
        "frame_id",
        frame_id_,
        "landing_boat/hull/front_lidar");

    pub_ = nh_.advertise<sensor_msgs::LaserScan>(
        ros_topic_,
        10);

    if (!gz_node_.Subscribe(
            gz_topic_,
            &GzLaserScanBridge::OnScan,
            this))
    {
      ROS_FATAL_STREAM(
          "Failed to subscribe Gazebo LaserScan topic: "
          << gz_topic_);
      ros::shutdown();
      return;
    }

    ROS_INFO_STREAM(
        "Gazebo LaserScan bridge: "
        << gz_topic_
        << " -> "
        << ros_topic_);
  }

private:
  void OnScan(const gz::msgs::LaserScan &_msg)
  {
    sensor_msgs::LaserScan out;

    if (_msg.has_header() &&
        _msg.header().has_stamp())
    {
      out.header.stamp.sec =
          _msg.header().stamp().sec();
      out.header.stamp.nsec =
          _msg.header().stamp().nsec();
    }
    else
    {
      out.header.stamp = ros::Time::now();
    }

    out.header.frame_id = frame_id_;

    out.angle_min = _msg.angle_min();
    out.angle_max = _msg.angle_max();
    out.angle_increment = _msg.angle_step();

    out.range_min = _msg.range_min();
    out.range_max = _msg.range_max();

    out.ranges.reserve(_msg.ranges_size());
    for (int i = 0; i < _msg.ranges_size(); ++i)
    {
      out.ranges.push_back(_msg.ranges(i));
    }

    out.intensities.reserve(_msg.intensities_size());
    for (int i = 0; i < _msg.intensities_size(); ++i)
    {
      out.intensities.push_back(_msg.intensities(i));
    }

    pub_.publish(out);
  }

  ros::NodeHandle nh_;
  ros::Publisher pub_;

  gz::transport::Node gz_node_;

  std::string gz_topic_;
  std::string ros_topic_;
  std::string frame_id_;
};

int main(int argc, char **argv)
{
  ros::init(
      argc,
      argv,
      "gz_laserscan_bridge");

  GzLaserScanBridge bridge;

  ros::spin();
  return 0;
}
