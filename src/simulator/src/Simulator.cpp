#include <simulator/Simulator.h>

Simulator::Simulator(ros::NodeHandle& node)
  : node_(node) {
  control_sub_ = node_.subscribe("control", 0, &Simulator::controlCallback, this);
}

void Simulator::controlCallback(const ackermann_msgs::AckermannDriveStamped::ConstPtr& msg) {

}
