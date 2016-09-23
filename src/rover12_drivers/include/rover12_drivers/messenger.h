#pragma once

#include <algorithm>
#include <functional>
#include <memory>
#include <string>

#include <serial/serial.h>

#include <rover12_comm/rover12_comm.h>

namespace rover12_drivers {

class Messenger {
public:
  using GpsCallback = std::function<void(const rover12_comm::GpsMsg&)>;
  using ImuCallback = std::function<void(const rover12_comm::ImuMsg&)>;
  using ImuCalCallback = std::function<void(const rover12_comm::ImuCalMsg&)>;
  using WheelEncCallback = std::function<void(const rover12_comm::WheelEncMsg&)>;
  using EstopCallback = std::function<void(const rover12_comm::EstopMsg&)>;

  explicit Messenger(const std::string& device);

  void setGpsCallback(GpsCallback callback) { gps_callback_ = callback; }
  void setImuCallback(ImuCallback callback) { imu_callback_ = callback; }
  void setImuCalCallback(ImuCalCallback callback) { imu_cal_callback_ = callback; }
  void setWheelEncCallback(WheelEncCallback callback) { wheel_enc_callback_ = callback; }
  void setEstopCallback(EstopCallback callback) { estop_callback_ = callback; }

  template <typename MsgType>
  void send(MsgType& msg) {
    msg.encode();
    size_t bytes_to_write = sizeof(MsgType);
    do {
      auto offset = sizeof(MsgType) - bytes_to_write;
      auto buf = reinterpret_cast<const uint8_t*>(&msg) + offset;
      auto nwritten = sp_.write(buf, bytes_to_write);
      bytes_to_write -= nwritten;
    } while (bytes_to_write > 0);
  }

  void spin();

private:
  void dispatchMessage();

  serial::Serial sp_;
  uint8_t buffer_[256];
  int buffer_idx_;
  GpsCallback gps_callback_;
  ImuCallback imu_callback_;
  ImuCalCallback imu_cal_callback_;
  WheelEncCallback wheel_enc_callback_;
  EstopCallback estop_callback_;
};

} // namespace rover12_drivers
