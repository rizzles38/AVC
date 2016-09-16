#include <algorithm>
#include <cstdlib>
#include <string>
#include <vector>
#include <limits>

#include <Eigen/Dense>

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
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
    : start_idx_(2960) {
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
  }

  // Matches the given position to the best point on the trajectory (based on
  // Euclidean distance).
  int match(Vector2d position) {
    const int forward_search_limit = 50;
    const int backward_search_limit = 50;

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
        if (worse_count >= forward_search_limit) {
          break;
        }
      }
      ++i;
    };

    // best_idx_ now points to the closest point on the trajectory, so reset
    // our search window around this point
    start_idx_ = std::max(best_idx_ - backward_search_limit, 0);

    return best_idx_;
  }

  Waypoint operator[](int index) {
    if (index < 0 || std::size_t(index) >= waypoints_.size()) {
      ROS_ERROR_STREAM("invalid traj index " << index);
      std::exit(EXIT_FAILURE);
    }
    return waypoints_[static_cast<std::size_t>(index)];
  }

private:
  std::vector<Waypoint> waypoints_;
  int start_idx_;
};

} // namespace

int main(int argc, char* argv[]) {
  if (argc != 2) {
    ROS_ERROR_STREAM("Usage: " << argv[0] << " <trajectory bag>");
    return EXIT_FAILURE;
  }

  Trajectory traj(argv[1]);

  /*
  for (double y = -1.57; y >= -1.61; y -= 0.001) {
    Vector2d p(-0.75, y);
    auto match_idx = traj.match(p);
    auto wp = traj[match_idx];
    ROS_INFO_STREAM("Match: " << p.x() << ", " << p.y() << " [" << match_idx << "] " << wp.position.x() << ", " << wp.position.y());
  }
  */

  return EXIT_SUCCESS;
}
