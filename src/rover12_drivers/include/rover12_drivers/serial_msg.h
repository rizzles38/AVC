#pragma once

#include <cstdint>
#include <type_traits>

namespace rover12_drivers {

enum class MsgType : int8_t {
  UNKNOWN = 0,
  ID_REQUEST = 1,
  ID_RESPONSE = 2,
  CONTROL = 3,
  ESTOP = 4,
  GPS = 5,
  IMU = 6,
  BATTERY = 7,
};

#define SET_MSG_TYPE(msg_type) \
  enum { Type = static_cast<typename std::underlying_type<MsgType>::type>(MsgType::msg_type) };

template <typename PayloadType>
class SerialMsg {
public:
  explicit SerialMsg()
    : type(static_cast<MsgType>(PayloadType::Type)),
      checksum(0),
      overhead(0),
      data(PayloadType()),
      trailer(0) {}

private:
  const MsgType type;
  uint32_t checksum;
  uint8_t overhead;

public:
  PayloadType data;

private:
  const uint8_t trailer;
} __attribute__((packed));

// Sent from the computer to the control board or sensor board. Requests an
// identification response to know what kind of board is located at this serial
// endpoint.
struct IdRequest {
  SET_MSG_TYPE(ID_REQUEST);

  IdRequest() {}
} __attribute__((packed));

using IdRequestMsg = SerialMsg<IdRequest>;

// Sent from the control board or sensor board to the computer. Responds with
// the type of board that is located at this serial endpoint for identification
// purposes.
struct IdResponse {
  SET_MSG_TYPE(ID_RESPONSE);

  enum class Board : int8_t {
    UNKNOWN = 0,
    CONTROL = 1,
    SENSOR = 2,
  };

  IdResponse()
    : board(Board::UNKNOWN) {}

  Board board;
} __attribute__((packed));

using IdResponseMsg = SerialMsg<IdResponse>;

// Sent from the computer to the control board. Contains the desired steering
// angle (in degrees) and speed (in meters per second).
struct Control {
  SET_MSG_TYPE(CONTROL);

  Control()
    : steering_angle(0.0),
      velocity(0.0) {}

  float steering_angle;
  float velocity;
} __attribute__((packed));

using ControlMsg = SerialMsg<Control>;

// Sent from the computer to the control board. If true, we authorize the
// hardware to be in autonomous mode.
struct Estop {
  SET_MSG_TYPE(ESTOP);

  Estop(bool autonomous)
    : autonomous(false) {}

  bool autonomous;
} __attribute__((packed));

using EstopMsg = SerialMsg<Estop>;

// TODO: GpsMsg, ImuMsg, BatteryMsg

#undef SET_MSG_TYPE

} // namespace rover12_drivers
