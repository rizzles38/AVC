#pragma once

#include <cstddef>
#include <cstdint>
#include <type_traits>

namespace rover12_drivers {

// Computes a 32-bit checksum over the data and returns it.
uint32_t crc32(const uint8_t* data, size_t len);

// Applies consistent overhead byte stuffing (COBS) in place to the given data.
// This expects that the data you want to stuff starts at the second byte and
// the first byte is available for overhead. It does not add a trailing byte.
// The length should be the length of the data, not including the overhead byte.
void cobs(uint8_t* data, size_t len);

// Decodes a buffer encoded with consistent overhead byte stuffing (COBS) in
// place. The data pointer should point to the beginning of the stuffed data
// (first byte in the overhead byte). The length should be the length of the
// data, not including the overhead byte. The trailing byte is ignored.
void uncobs(uint8_t* data, size_t len);

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
    : type_(static_cast<MsgType>(PayloadType::Type)),
      checksum_(0),
      overhead_(0),
      data(PayloadType()),
      trailer_(0) {}

  // Encodes the message for sending, computing the CRC32 checksum and stuffing
  // the data with consistent overhead byte stuffing. Once encoded, you can
  // send the data starting at the first byte of this object and sending
  // sizeof(SerialMsg<PayloadType>) bytes.
  void encode() {
    checksum_ = crc32(reinterpret_cast<uint8_t*>(&data), sizeof(PayloadType));
    cobs(&overhead_, sizeof(PayloadType));
  }

  // Returns true if the message was successfully decoded. To be successfully
  // decoded, the message type must match and the checksum must validate. Once
  // decoded, the payload data can be accessed directly via the public .data
  // member.
  bool decode() {
    if (type_ != static_cast<MsgType>(PayloadType::Type)) {
      return false;
    }
    uncobs(&overhead_, sizeof(PayloadType));
    return checksum_ == crc32(reinterpret_cast<uint8_t*>(&data), sizeof(PayloadType));
  }

private:
  const MsgType type_;
  uint32_t checksum_;
  uint8_t overhead_;

public:
  PayloadType data;

private:
  const uint8_t trailer_;
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