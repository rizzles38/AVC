#include <cstdlib>
#include <iostream>

#include <rover12_drivers/serial_msg.h>

int main(int argc, char* argv[]) {
  rover12_drivers::ControlMsg control_msg;

  control_msg.data.steering_angle = 15.0;
  control_msg.data.velocity = 5.0;
  std::cout << "[I] steering_angle = " << control_msg.data.steering_angle << "\n";
  std::cout << "[I] velocity = " << control_msg.data.velocity << "\n";

  control_msg.encode();
  std::cout << "[E] steering_angle = " << control_msg.data.steering_angle << "\n";
  std::cout << "[E] velocity = " << control_msg.data.velocity << "\n";

  control_msg.decode();
  std::cout << "[D] steering_angle = " << control_msg.data.steering_angle << "\n";
  std::cout << "[D] velocity = " << control_msg.data.velocity << "\n";

  return EXIT_SUCCESS;
};
