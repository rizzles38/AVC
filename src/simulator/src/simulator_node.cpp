#include <ros/ros.h>

#include <simulator/Simulator.h>

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "simulator_node");

  ros::NodeHandle node;
  Simulator simulator(node);

  ros::spin();

  return 0;
}
