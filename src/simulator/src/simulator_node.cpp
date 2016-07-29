#include <ros/ros.h>

#include <simulator/Simulator.h>

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "simulator_node");

  ros::NodeHandle node;
  Simulator simulator(node);

  ros::Rate rate(30.0);
  while(ros::ok()) {
    simulator.tick();
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
