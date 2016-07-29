#pragma once

#include <ros/ros.h>

#include <ackermann_msgs/AckermannDriveStamped.h>

class Simulator {
public:
  explicit Simulator(ros::NodeHandle& node);

private:
  void controlCallback(const ackermann_msgs::AckermannDriveStamped::ConstPtr& msg);

  ros::NodeHandle& node_;
  ros::Subscriber control_sub_;
};
