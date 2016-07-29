#pragma once

#include <ros/ros.h>

#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

namespace rover12_description {

class Broadcaster {
public:
  explicit Broadcaster(ros::NodeHandle& node);

private:
  void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg);

  ros::NodeHandle& node_;
  ros::Subscriber odometry_sub_;
  tf::TransformBroadcaster broadcaster_;
  tf::Transform map_to_base_link_;
};

} // namespace rover12_description
