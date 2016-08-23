#pragma once

#include <cstdint>

namespace rover12_drivers {

enum class MsgType : uint32_t {
  UNKNOWN = 0,
  CONTROL = 1,
  ESTOP = 2,
  GPS = 3,
  IMU = 4,
  BATTERY = 5,
};

struct Header {
  explicit Header(MsgType type)
    : type(type),
      checksum(0) {}

  const MsgType type;
  uint32_t checksum;
};

struct ControlMsg {
  ControlMsg(float steering_angle, float velocity)
    : header(MsgType::CONTROL),
      steering_angle(steering_angle),
      velocity(velocity) {}

  Header header;
  float steering_angle;
  float velocity;
};

struct EstopMsg {
  EstopMsg(bool autonomous)
    : header(MsgType::ESTOP),
      autonomous(autonomous) {}

  Header header;
  bool autonomous;
};

// TODO: GpsMsg, ImuMsg, BatteryMsg

} // namespace rover12_drivers
