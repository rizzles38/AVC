#include <cstdlib>
#include <string>

#include <ros/ros.h>

#include <rover12_comm/rover12_comm.h>
#include <rover12_drivers/messenger.h>

class EncoderPublisher {
public:
  explicit EncoderPublisher(ros::NodeHandle& nh)
    : nh_(nh) {
    // TODO: publishers advertise
  }

  void wheelEncCallback(const rover12_comm::WheelEncMsg& msg) {
    ROS_INFO_STREAM("RL=" << msg.data.count_rear_left << " RR=" << msg.data.count_rear_right);
    // TODO: ROS odometry message
  }

private:
  ros::NodeHandle& nh_;
  // TODO: ros::Publisher
};

int main(int argc, char* argv[]) {
  if (argc < 2) {
    ROS_ERROR_STREAM("Usage: " << argv[0] << " [serial devices]");
    return EXIT_FAILURE;
  }

  // Initialize ROS.
  ros::init(argc, argv, "control_board_node");
  ros::NodeHandle nh;

  // Create encoder publisher.
  EncoderPublisher encoder_publisher(nh);

  // Add each serial device to the messenger.
  rover12_drivers::Messenger messenger;
  for (int i = 1; i < argc; ++i) {
    messenger.addDevice(argv[i]);
  }

  // Set callbacks on message type.
  messenger.setWheelEncCallback([&encoder_publisher](const rover12_comm::WheelEncMsg& msg) {
    encoder_publisher.wheelEncCallback(msg);
  });

  // Connect to the control board.
  messenger.connect(rover12_drivers::Messenger::Board::CONTROL);

  // Spin on the messenger and ROS.
  while (ros::ok()) {
    messenger.spin();
    ros::spinOnce();
  }

  return EXIT_SUCCESS;
};
