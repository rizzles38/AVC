#include <cstdlib>
#include <iostream>

#include <rover12_drivers/serial_msg.h>

int main(int argc, char* argv[]) {
  std::cout << "Hello control board node!\n";

  rover12_drivers::ControlMsg control_msg;
  control_msg.data.steering_angle = 15.0;
  control_msg.data.velocity = 5.0;

  return EXIT_SUCCESS;
};
