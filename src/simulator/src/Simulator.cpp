#include <simulator/Simulator.h>

#include <cmath>
#include <string>

#include <tf/transform_datatypes.h>

namespace simulator {

Simulator::Simulator(ros::NodeHandle& node)
  : node_(node),
    world_(node) {
  odometry_pub_ = node_.advertise<nav_msgs::Odometry>("odometry", 0);
  control_sub_ = node_.subscribe("control", 0, &Simulator::controlCallback, this);
}

void Simulator::controlCallback(const ackermann_msgs::AckermannDriveStamped::ConstPtr& msg) {
  control_ = *msg;
}

void Simulator::tick() {
  double now = ros::Time::now().toSec();
  double theta = std::fmod(2.0 * M_PI * 0.1 * now, 2.0 * M_PI);
  double x = std::cos(theta);
  double y = std::sin(theta);

  tf::pointTFToMsg(tf::Point(x, y, 0.0), odometry_.pose.pose.position);
  odometry_.pose.pose.orientation = tf::createQuaternionMsgFromYaw(theta + M_PI / 2.0);
  odometry_.header.stamp = ros::Time::now();
  odometry_.header.frame_id = "map";

  // TODO: more

  odometry_pub_.publish(odometry_);
}

} // namespace simulator
