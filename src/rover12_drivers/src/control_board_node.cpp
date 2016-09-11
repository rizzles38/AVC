#include <cstdlib>
#include <iostream>

#include <rover12_comm/rover12_comm.h>

template <typename T>
void hexPrint(const rover12_comm::SerialMsg<T>* msg) {
  uint8_t* bytes = (uint8_t*)msg;
  std::cout << "type = 0x" << std::hex << (uint32_t)bytes[0] << "\n";
  std::cout << "checksum = 0x" << std::hex << *(uint32_t*)&bytes[1] << "\n";
  std::cout << "overhead = 0x" << std::hex << (uint32_t)*(uint8_t*)&bytes[5] << "\n";
  for (size_t i = 0; i < sizeof(T); ++i) {
    std::cout << "payload[" << i << "] = 0x" << std::hex << (uint32_t)*(uint8_t*)&bytes[6 + i] << "\n";
  }
  std::cout << "trailer = 0x" << std::hex << (uint32_t)*(uint8_t*)&bytes[6 + sizeof(T)] << "\n";
}

int main(int argc, char* argv[]) {
  rover12_comm::ControlMsg control_msg;

  std::cout << "sizeof(ControlMsg) = " << sizeof(rover12_comm::ControlMsg) << "\n\n";

  control_msg.data.steering_angle = 15.0;
  control_msg.data.velocity = 5.0;
  std::cout << "[I] steering_angle = " << control_msg.data.steering_angle << "\n";
  std::cout << "[I] velocity = " << control_msg.data.velocity << "\n";
  hexPrint(&control_msg);
  std::cout << "\n";

  control_msg.encode();
  std::cout << "[E] steering_angle = " << control_msg.data.steering_angle << "\n";
  std::cout << "[E] velocity = " << control_msg.data.velocity << "\n";
  std::cout << "checksum = 0x" << std::hex << control_msg.checksum() << "\n";
  hexPrint(&control_msg);
  std::cout << "\n";

  bool valid = control_msg.decode();
  std::cout << "[D] steering_angle = " << control_msg.data.steering_angle << "\n";
  std::cout << "[D] velocity = " << control_msg.data.velocity << "\n";
  std::cout << "valid = " << std::boolalpha << valid << "\n";
  hexPrint(&control_msg);
  std::cout << "\n";

  rover12_comm::IdRequestMsg id_request_msg;

  std::cout << "sizeof(IdRequestMsg) = " << sizeof(rover12_comm::IdRequestMsg) << "\n\n";
  hexPrint(&id_request_msg);
  std::cout << "\n";

  id_request_msg.encode();
  std::cout << "checksum = 0x" << std::hex << id_request_msg.checksum() << "\n";
  hexPrint(&id_request_msg);
  std::cout << "\n";

  bool id_req_valid = id_request_msg.decode();
  std::cout << "valid = " << std::boolalpha << id_req_valid << "\n";
  hexPrint(&id_request_msg);

  return EXIT_SUCCESS;
};
