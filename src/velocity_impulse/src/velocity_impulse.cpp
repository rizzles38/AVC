#include <cstdlib>

#include <ros/ros.h>

#include <ackermann_msgs/AckermannDriveStamped.h>

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "velocity_impulse");

  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<ackermann_msgs::AckermannDriveStamped>("/control/command", 0);

  ackermann_msgs::AckermannDriveStamped msg;
  msg.header.frame_id = "base_link";
  msg.drive.steering_angle = 0.0;
  msg.drive.speed = 0.0;

  auto start_time = ros::Time::now();
  auto go_time = start_time + ros::Duration(3.0);
  auto stop_time = go_time + ros::Duration(3.0);

  const double magnitude = 3.0; // m/s

  ros::Rate rate(50); // 50 Hz
  while (ros::ok()) {
    auto now = ros::Time::now();
    if (go_time <= now && now <= stop_time) {
      msg.drive.speed = magnitude;
    } else {
      msg.drive.speed = 0.0;
    }

    msg.header.stamp = ros::Time::now();
    pub.publish(msg);

    ros::spinOnce();
    rate.sleep();
  }

  return EXIT_SUCCESS;
}
