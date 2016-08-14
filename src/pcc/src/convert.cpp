#include <pcc/convert.h>

#include <cmath>

#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>

namespace pcc {

nav_msgs::Path convertToPath(const Curve& curve, std::string frame_id) {
  nav_msgs::Path path;

  auto now = ros::Time::now();

  path.header.stamp = now;
  path.header.frame_id = frame_id;

  geometry_msgs::PoseStamped pose;
  pose.header.stamp = now;
  pose.header.frame_id = frame_id;

  for (int i = 0; i < curve.size(); ++i) {
    auto position = curve.position(i);
    auto tangent = curve.tangent(i);

    tf::pointTFToMsg(tf::Point(position.x(), position.y(), 0.0),
        pose.pose.position);

    double theta = std::atan2(tangent.y(), tangent.x());
    pose.pose.orientation = tf::createQuaternionMsgFromYaw(theta);

    path.poses.push_back(pose);
  }

  return path;
}

} // namespace pcc
