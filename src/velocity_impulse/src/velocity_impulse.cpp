#include <cstdlib>
#include <string>

#include <ros/ros.h>

#include <ackermann_msgs/AckermannDriveStamped.h>

int main(int argc, char* argv[]) {
  if (argc < 3) {
    ROS_ERROR_STREAM("Usage: " << argv[0] << " <speed in m/s> <time in sec>");
    return EXIT_FAILURE;
  }

  const double speed = std::stod(argv[1]);
  const double length = std::stod(argv[2]);

  ros::init(argc, argv, "velocity_impulse");

  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<ackermann_msgs::AckermannDriveStamped>("/control/command", 0);

  ROS_INFO_STREAM("Using speed = " << speed << " m/s");
  ROS_INFO_STREAM("Using length = " << length << " sec");

  ackermann_msgs::AckermannDriveStamped msg;
  msg.header.frame_id = "base_link";
  msg.drive.steering_angle = 0.0;
  msg.drive.speed = 0.0;

  auto start_time = ros::Time::now();
  auto go_time = start_time + ros::Duration(3.0);
  auto stop_time = go_time + ros::Duration(length);

  ros::Rate rate(50); // 50 Hz
  while (ros::ok()) {
    auto now = ros::Time::now();
    if (go_time <= now && now <= stop_time) {
      msg.drive.speed = speed;
    } else {
      msg.drive.speed = 0.0;
    }

    msg.header.stamp = ros::Time::now();
    pub.publish(msg);

    ros::spinOnce();
    rate.sleep();

    if (now > stop_time + ros::Duration(3.0)) {
      break;
    }
  }

  return EXIT_SUCCESS;
}
