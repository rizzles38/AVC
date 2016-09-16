#include <cstdlib>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

int main(int argc, char* argv[]) {
  if (argc < 3) {
    ROS_ERROR_STREAM("Usage: " << argv[0] << " <input odometry bag> <output path bag> [velocity scale factor = 1.0]");
    return EXIT_FAILURE;
  }

  // Topic we're extracting to a path.
  const std::string odom_topic("/odom/filtered");

  // Topic we're writing a path to.
  const std::string path_topic("/trajectory");

  // Velocity scale factor, if specified.
  double velocity_scale = 1.0;
  if (argc > 3) {
    velocity_scale = std::stod(argv[3]);
  }

  ROS_INFO_STREAM("Reading odometry from " << argv[1]);
  ROS_INFO_STREAM("Writing path to " << argv[2]);
  ROS_INFO_STREAM("Using velocity scale " << velocity_scale);

  // Open odometry bag for reading.
  rosbag::Bag odom_bag;
  odom_bag.open(argv[1], rosbag::bagmode::Read);
  std::vector<std::string> topics;
  topics.push_back(odom_topic);
  rosbag::View view(odom_bag, rosbag::TopicQuery(topics));

  // Open path bag for writing.
  rosbag::Bag path_bag;
  path_bag.open(argv[2], rosbag::bagmode::Write);
  nav_msgs::Path path_msg;
  path_msg.header.stamp = ros::TIME_MIN;
  path_msg.header.frame_id = "odom";

  double start_time = -1.0;
  for (auto msg_iter = view.begin(); msg_iter != view.end(); ++msg_iter) {
    auto odom_msg_ptr = msg_iter->instantiate<nav_msgs::Odometry>();
    if (!odom_msg_ptr) {
      ROS_ERROR_STREAM("odom_msg_ptr NULL when reading bag!");
      return EXIT_FAILURE;
    }

    // Extract useful data.
    const double time = odom_msg_ptr->header.stamp.toSec();
    const double x = odom_msg_ptr->pose.pose.position.x;
    const double y = odom_msg_ptr->pose.pose.position.y;
    const double speed = odom_msg_ptr->twist.twist.linear.x * velocity_scale;

    // Skip data points with low speed to remove the leading and trailing parts
    // of the recording.
    if (speed < 0.0001) {
      continue;
    }

    // Record start time and compute t from start time from here on out.
    if (start_time < 0.0) {
      start_time = time;
    }
    const double t = (time - start_time) + ros::TIME_MIN.toSec();

    // Populate the pose in the path. We map base_link x axis speed to the z
    // axis of the pose (for display, and as a hack to get it through to the
    // planner).
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.stamp = ros::Time(t);
    pose_stamped.header.frame_id = "odom";
    geometry_msgs::Pose pose;
    geometry_msgs::Point point;
    point.x = x;
    point.y = y;
    point.z = speed;
    pose.position = point;
    pose_stamped.pose = pose;
    path_msg.poses.push_back(pose_stamped);

    ROS_INFO_STREAM(t << ", " << x << ", " << y << ", " << speed);
  }

  path_bag.write(path_topic, ros::TIME_MIN, path_msg);

  odom_bag.close();
  path_bag.close();

  return EXIT_SUCCESS;
}
