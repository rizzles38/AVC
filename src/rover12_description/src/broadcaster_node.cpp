#include <ros/ros.h>

#include <rover12_description/Broadcaster.h>

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "broadcaster_node");

  ros::NodeHandle node;
  rover12_description::Broadcaster broadcaster(node);

  ros::spin();

  return 0;
}
