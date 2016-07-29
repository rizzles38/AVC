#include <rover12_description/Broadcaster.h>

#include <tf/transform_datatypes.h>

namespace rover12_description {

Broadcaster::Broadcaster(ros::NodeHandle& node)
  : node_(node) {
  odometry_sub_ = node_.subscribe("odometry", 0, &Broadcaster::odometryCallback, this);
}

void Broadcaster::odometryCallback(const nav_msgs::Odometry::ConstPtr& msg) {
  // Place our robot in the world by setting the map to base_link transform.
  tf::Point position;
  tf::pointMsgToTF(msg->pose.pose.position, position);
  map_to_base_link_.setOrigin(position);

  tf::Quaternion orientation;
  tf::quaternionMsgToTF(msg->pose.pose.orientation, orientation);
  map_to_base_link_.setRotation(orientation);

  broadcaster_.sendTransform(
      tf::StampedTransform(map_to_base_link_, ros::Time::now(), "map", "base_link"));
}

} // namespace rover12_description
