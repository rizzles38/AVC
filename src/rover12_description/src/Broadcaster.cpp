#include <rover12_description/Broadcaster.h>

namespace rover12_description {

Broadcaster::Broadcaster(ros::NodeHandle& node)
  : node_(node) {
  odometry_sub_ = node_.subscribe("odometry", 0, &Broadcaster::odometryCallback, this);
}

void Broadcaster::odometryCallback(const nav_msgs::Odometry::ConstPtr& msg) {
  // Place our robot in the world by setting the map to base_link transform.
  const geometry_msgs::Point& p = msg->pose.pose.position;
  const geometry_msgs::Quaternion& o = msg->pose.pose.orientation;
  map_to_base_link_.setOrigin(tf::Vector3(p.x, p.y, p.z));
  map_to_base_link_.setRotation(tf::Quaternion(o.x, o.y, o.z, o.w));
  broadcaster_.sendTransform(
      tf::StampedTransform(map_to_base_link_, ros::Time::now(), "map", "base_link"));
}

} // namespace rover12_description
