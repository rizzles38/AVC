#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <string>
#include <vector>
#include <limits>

#include <Eigen/Dense>

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

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

// A sequence of waypoints.
class Trajectory {
public:
  explicit Trajectory(const std::string& bag_file)
    : start_idx_(0),
      forward_search_limit_(50),
      backward_search_limit_(50),
      goal_tolerance_(25) {
    const std::string traj_topic("/trajectory");
    nav_msgs::Path path_msg;

    ROS_INFO_STREAM("Loading trajectory from " << bag_file);
    rosbag::Bag path_bag;
    path_bag.open(bag_file, rosbag::bagmode::Read);
    std::vector<std::string> topics;
    topics.push_back(traj_topic);
    rosbag::View view(path_bag, rosbag::TopicQuery(topics));

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

    for (auto pose_iter = msg_ptr->poses.begin(); pose_iter != msg_ptr->poses.end(); ++pose_iter) {
      const double time = pose_iter->header.stamp.toSec();
      const double x = pose_iter->pose.position.x;
      const double y = pose_iter->pose.position.y;
      const double speed = pose_iter->pose.position.z;
      waypoints_.emplace_back(time, x, y, speed);
    }

    // Compute curvature and tangent for each point.
    waypoints_.front().curvature = 0.0;
    waypoints_.front().tangent = discreteTangent(waypoints_[0].position,
                                                 waypoints_[1].position);
    waypoints_.back().curvature = 0.0;
    waypoints_.back().tangent = discreteTangent(waypoints_[waypoints_.size() - 2].position,
                                                waypoints_.back().position);
    for (std::size_t i = 1; i < waypoints_.size() - 1; ++i) {
      waypoints_[i].curvature = discreteCurvature(waypoints_[i - 1].position,
                                                  waypoints_[i].position,
                                                  waypoints_[i + 1].position);
      waypoints_[i].tangent = discreteTangent(waypoints_[i - 1].position,
                                              waypoints_[i + 1].position);
    }

    ROS_INFO_STREAM("Trajectory has " << waypoints_.size() << " points.");

    ros::NodeHandle pnh("~");
    if (!pnh.getParam("forward_search_limit", forward_search_limit_)) {
      ROS_ERROR_STREAM("No forward_search_limit parameter found!");
      std::exit(EXIT_FAILURE);
    }
    if (!pnh.getParam("backward_search_limit", backward_search_limit_)) {
      ROS_ERROR_STREAM("No backward_search_limit parameter found!");
      std::exit(EXIT_FAILURE);
    }
    if (!pnh.getParam("goal_tolerance", goal_tolerance_)) {
      ROS_ERROR_STREAM("No goal_tolerance parameter found!");
      std::exit(EXIT_FAILURE);
    }
    ROS_INFO_STREAM("Using forward_search_limit = " << forward_search_limit_);
    ROS_INFO_STREAM("Using backward_search_limit = " << backward_search_limit_);
    ROS_INFO_STREAM("Using goal_tolerance = " << goal_tolerance_);
  }

  // Matches the given position to the best point on the trajectory (based on
  // Euclidean distance).
  int match(Vector2d position) {
    int best_idx_ = start_idx_;
    double best_dist_ = (position - waypoints_[start_idx_].position).squaredNorm();
    int size = static_cast<int>(waypoints_.size());
    int i = start_idx_;
    int worse_count = 0;
    while (i < size) {
      const double d2 = (position - waypoints_[i].position).squaredNorm();
      if (d2 <= best_dist_) {
        best_idx_ = i;
        worse_count = 0;
      } else {
        ++worse_count;
        if (worse_count >= forward_search_limit_) {
          break;
        }
      }
      ++i;
    }

    // best_idx_ now points to the closest point on the trajectory, so reset
    // our search window around this point
    start_idx_ = std::max(best_idx_ - backward_search_limit_, 0);

    return best_idx_;
  }

  // Returns true if the search range starts within tolerance of the last
  // point in the trajectory.
  bool reachedGoal() const {
    return static_cast<int>(waypoints_.size()) - start_idx_ < goal_tolerance_;
  }

  const Waypoint& operator[](int index) const {
    if (index < 0 || std::size_t(index) >= waypoints_.size()) {
      ROS_ERROR_STREAM("invalid traj index " << index);
      std::exit(EXIT_FAILURE);
    }
    return waypoints_[static_cast<std::size_t>(index)];
  }

private:
  std::vector<Waypoint> waypoints_;
  int start_idx_;
  int forward_search_limit_;
  int backward_search_limit_;
  int goal_tolerance_;
};

class LaneKeeping {
public:
  LaneKeeping()
    : lookahead_(0.25),
      feedback_gain_(0.0) {
    ros::NodeHandle pnh("~");
    if (!pnh.getParam("lookahead", lookahead_)) {
      ROS_ERROR_STREAM("No lookahead parameter found!");
      std::exit(EXIT_FAILURE);
    }
    if (!pnh.getParam("feedback_gain", feedback_gain_)) {
      ROS_ERROR_STREAM("No feedback_gain parameter found!");
      std::exit(EXIT_FAILURE);
    }
  }

  ackermann_msgs::AckermannDrive process(Trajectory& traj, const Vector2d& position) {
    const double wheelbase = 0.33; // 33cm, this should be a param, lol.

    // Initialize output message.
    ackermann_msgs::AckermannDrive drive;
    drive.steering_angle = 0.0;
    drive.steering_angle_velocity = -1.0;
    drive.speed = 0.0;
    drive.acceleration = -1.0;
    drive.jerk = -1.0;

    // If we've reached the goal, send zero steering angle and zero speed.
    if (traj.reachedGoal()) {
      return drive;
    }

    // Project our position onto the trajectory.
    const auto& wp = traj[traj.match(position)];

    // Compute lateral error.
    const double lateral_error = (position - wp.position).norm();

    // Compute heading error.
    const double heading_error = 0.0; // TODO

    // Compute lookahead error.
    const double lookahead_error = lateral_error + lookahead_ * std::sin(heading_error);

    // Compute feedforward based on path curvature.
    const double curvature = wp.curvature;
    const double feedforward = std::atan(wheelbase * curvature);

    // Compute feedback based on lookahead error.
    const double feedback = -feedback_gain_ * lookahead_error;

    // Combine into steering angle!
    const double steering_angle = feedforward + feedback;

    // Set the steering angle based on lane keeping control and the velocity
    // based on the trajectory projection.
    drive.steering_angle = steering_angle;
    drive.speed = wp.speed;

    return drive;
  }

private:
  double lookahead_;
  double feedback_gain_;
};

class Planner {
public:
  Planner(ros::NodeHandle& nh, const std::string& traj_bag)
    : nh_(nh),
      traj_(traj_bag) {
    drive_pub_ = nh_.advertise<ackermann_msgs::AckermannDriveStamped>("/control/command", 0);
    traj_pub_ = nh_.advertise<nav_msgs::Path>("/trajectory", 0);
    pose_sub_ = nh_.subscribe("/odom/filtered", 0, &Planner::poseCallback, this);
  }

  void poseCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    ackermann_msgs::AckermannDriveStamped drive_msg;
    drive_msg.header.stamp == ros::Time::now();
    drive_msg.header.frame_id = "base_link";

    Vector2d position = Vector2d(msg->pose.pose.position.x,
                                 msg->pose.pose.position.y);
    // TODO: Orientation is a quaternion, so either extract yaw angle or heading
    // vector here and pass it to the lane keeping controller.
    drive_msg.drive = controller_.process(traj_, position);

    drive_pub_.publish(drive_msg);
  }

private:
  ros::NodeHandle& nh_;
  ros::Publisher drive_pub_;
  ros::Publisher traj_pub_;
  ros::Subscriber pose_sub_;
  Trajectory traj_;
  LaneKeeping controller_;
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
