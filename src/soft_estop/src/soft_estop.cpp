#include <cstdlib>
#include <string>

#include <ros/ros.h>

#include <rover12_drivers/AutonomousMode.h>

class AutonomousPublisher {
public:
  explicit AutonomousPublisher(ros::NodeHandle& nh) :
    nh_(nh) {
    auto_pub_ = nh_.advertise<rover12_drivers::AutonomousMode>("/control/autonomous", 0);
    timer_ = nh_.createTimer(ros::Duration(0.1), &AutonomousPublisher::callback, this);
  }

  void callback(const ros::TimerEvent&) {
    rover12_drivers::AutonomousMode msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "base_link";
    msg.autonomous = true;
    auto_pub_.publish(msg);
  }

private:
  ros::NodeHandle nh_;
  ros::Publisher auto_pub_;
  ros::Timer timer_;
};

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "soft_estop");

  ros::NodeHandle nh;
  AutonomousPublisher autonomous_publisher(nh);

  while (ros::ok()) {
    ros::spin();
  }

  return EXIT_SUCCESS;
}
