#pragma once

#include <algorithm>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <rover12_comm/rover12_comm.h>
#include <rover12_drivers/serial_port.h>

namespace rover12_drivers {

class Messenger {
public:
  using GpsCallback = std::function<void(const rover12_comm::GpsMsg&)>;
  using ImuCallback = std::function<void(const rover12_comm::ImuMsg&)>;

  enum class Board {
    CONTROL,
    SENSOR
  };

  explicit Messenger();

  void addDevice(std::string device);

  void setGpsCallback(GpsCallback callback) { gps_callback_ = callback; }
  void setImuCallback(ImuCallback callback) { imu_callback_ = callback; }

  void connect(Board board);

  void spin();

private:
  void dispatchMessage(int len);

  std::unique_ptr<SerialPort> sp_;
  uint8_t buffer_[256];
  int buffer_idx_;
  GpsCallback gps_callback_;
  ImuCallback imu_callback_;
  std::vector<std::string> device_list_;
};

} // namespace rover12_drivers
