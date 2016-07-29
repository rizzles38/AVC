#pragma once

#include <ros/ros.h>

#include <ackermann_msgs/AckermannDriveStamped.h>
#include <nav_msgs/Odometry.h>

class Simulator {
public:
  explicit Simulator(ros::NodeHandle& node);

  void tick();

private:
  void controlCallback(const ackermann_msgs::AckermannDriveStamped::ConstPtr& msg);

  ros::NodeHandle& node_;
  ros::Publisher odometry_pub_;
  ros::Subscriber control_sub_;
  ackermann_msgs::AckermannDriveStamped control_;
  nav_msgs::Odometry odometry_;
};
