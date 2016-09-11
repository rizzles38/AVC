#include <cstdlib>
#include <iostream>
#include <string>

#include <ros/ros.h>

#include <rover12_comm/rover12_comm.h>
#include <rover12_drivers/messenger.h>

void gpsCallback(const rover12_comm::GpsMsg& msg) {
  std::cout << "Got GPS message!\n";
}

void imuCallback(const rover12_comm::ImuMsg& msg) {
  std::cout << "Got IMU message!\n";
}

int main(int argc, char* argv[]) {
  if (argc < 2) {
    ROS_ERROR_STREAM("Usage: " << argv[0] << " [serial devices]");
    return EXIT_FAILURE;
  }

  // Initialize ROS.
  ros::init(argc, argv, "sensor_board_node");

  // Add each serial device to the messenger.
  rover12_drivers::Messenger messenger;
  for (int i = 1; i < argc; ++i) {
    messenger.addDevice(argv[i]);
  }

  // Set callbacks on message type.
  messenger.setGpsCallback(gpsCallback);
  messenger.setImuCallback(imuCallback);

  // Connect to the sensor board.
  messenger.connect(rover12_drivers::Messenger::Board::SENSOR);

  // Spin on the messenger and ROS.
  while (ros::ok()) {
    messenger.spin();
    ros::spinOnce();
  }

  return EXIT_SUCCESS;
};
