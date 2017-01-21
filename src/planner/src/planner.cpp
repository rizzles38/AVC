#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <string>
#include <vector>
#include <limits>

#include <Eigen/Dense>

#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <rover12_drivers/Waypoint.h>
#include <rover12_drivers/Trajectory.h>

using Eigen::Vector2d;

namespace {

// Estimates the discrete curvature at point p2 given neighboring points p1 and
// p3.
double discreteCurvature(const Vector2d& p1, const Vector2d& p2, const Vector2d& p3) {
  Vector2d v21 = p2 - p1;
  Vector2d v32 = p3 - p2;
  Vector2d v31 = p3 - p1;

  // Triangle circumcircle:
  // r = abc / 4A
  // k = 1 / r = 4A / abc
  double det = v21[0] * v32[1] - v21[1] * v32[0]; // A = 1/2 * det
  return 2.0 * det / (v21.norm() * v32.norm() * v31.norm());
}

// Estimates the tangent at a point between p1 and p2.
Vector2d discreteTangent(const Vector2d& p1, const Vector2d& p2) {
  Vector2d v = p2 - p1;
  return v / v.norm();
}

// A point on the trajectory with all its information.
struct Waypoint {
  Waypoint(double time, double x, double y, double speed)
    : position(x, y),
      tangent(0.0, 0.0),
      speed(speed),
      curvature(0.0),
      time(time) {}

  Vector2d position;
  Vector2d tangent;
  double speed;
  double curvature;
  double time;
};

class Planner {
public:
  Planner(ros::NodeHandle& nh, const std::string& bag_file) : nh_(nh) {
    // Set up publishers.
    traj_pub_ = nh_.advertise<rover12_drivers::Trajectory>("/planner/trajectory", 0, true);
    path_pub_ = nh_.advertise<nav_msgs::Path>("/planner/path", 0, true);

    // Load the /trajectory topic out of the bag file.
    const std::string traj_topic("/trajectory");
    ROS_INFO_STREAM("Loading trajectory from " << bag_file);
    rosbag::Bag path_bag;
    path_bag.open(bag_file, rosbag::bagmode::Read);
    std::vector<std::string> topics;
    topics.push_back(traj_topic);
    rosbag::View view(path_bag, rosbag::TopicQuery(topics));

    // Get the message out of the topic.
    auto msg_iter = view.begin();
    if (msg_iter == view.end()) {
      ROS_ERROR_STREAM("No trajectory message in bag!");
      std::exit(EXIT_FAILURE);
    }
    auto msg_ptr = msg_iter->instantiate<nav_msgs::Path>();
    if (!msg_ptr) {
      ROS_ERROR_STREAM("msg_ptr NULL when reading bag!");
      std::exit(EXIT_FAILURE);
    }

    // Create a list of temporary waypoints as we extract them.
    std::vector<Waypoint> wps;
    for (auto pose_iter = msg_ptr->poses.begin(); pose_iter != msg_ptr->poses.end(); ++pose_iter) {
      const double time = pose_iter->header.stamp.toSec();
      const double x = pose_iter->pose.position.x;
      const double y = pose_iter->pose.position.y;
      const double speed = pose_iter->pose.position.z;
      wps.emplace_back(time, x, y, speed);
    }

    // Compute curvature and tangent for each waypoint.
    wps.front().curvature = 0.0;
    wps.front().tangent = discreteTangent(wps[0].position, wps[1].position);
    wps.back().curvature = 0.0;
    wps.back().tangent = discreteTangent(wps[wps.size() - 2].position,
                                         wps.back().position);
    for (std::size_t i = 1; i < wps.size() - 1; ++i) {
      wps[i].curvature = discreteCurvature(wps[i - 1].position,
                                           wps[i].position,
                                           wps[i + 1].position);
      wps[i].tangent = discreteTangent(wps[i - 1].position,
                                       wps[i + 1].position);
    }

    ROS_INFO_STREAM("Trajectory has " << wps.size() << " points.");

    // Path message is good, so publish it.
    path_ = *msg_ptr;
    path_.header.stamp = ros::Time::now();
    path_.header.frame_id = "odom";
    path_pub_.publish(path_);
    ROS_INFO_STREAM("Published path for visualization.");

    // Build a trajectory message and publish it.
    for (const auto& wp : wps) {
      rover12_drivers::Waypoint waypoint;
      waypoint.x = wp.position.x();
      waypoint.y = wp.position.y();
      waypoint.tx = wp.tangent.x();
      waypoint.ty = wp.tangent.y();
      waypoint.v = wp.speed;
      waypoint.k = wp.curvature;
      waypoint.t = wp.time;
      traj_.waypoints.push_back(waypoint);
    }
    traj_.header.stamp = ros::Time::now();
    traj_.header.frame_id = "odom";
    traj_pub_.publish(traj_);
    ROS_INFO_STREAM("Published trajectory for control.");
  }

private:
  ros::NodeHandle& nh_;
  ros::Publisher traj_pub_;
  ros::Publisher path_pub_;
  rover12_drivers::Trajectory traj_;
  nav_msgs::Path path_;
};

} // namespace

int main(int argc, char* argv[]) {
  if (argc < 2) {
    ROS_ERROR_STREAM("Usage: " << argv[0] << " <trajectory bag>");
    return EXIT_FAILURE;
  }

  ros::init(argc, argv, "planner");
  ros::NodeHandle nh;

  Planner planner(nh, argv[1]);

  ros::spin();

  return EXIT_SUCCESS;
}
