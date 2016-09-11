#pragma once

#include <stddef.h>
#include <stdint.h>

namespace rover12_comm {

// Computes a 32-bit checksum over the data and returns it.
uint32_t crc32(const uint8_t* data, size_t len);

// Applies consistent overhead byte stuffing (COBS) in place to the given data.
// This expects that the data you want to stuff starts at the second byte and
// the first byte is available for overhead. It does not add a trailing byte.
// The length should be the length of the data, not including the overhead byte.
// Only supports data up to 254 bytes in length.
void cobs(uint8_t* data, size_t len);

// Decodes a buffer encoded with consistent overhead byte stuffing (COBS) in
// place. The data pointer should point to the beginning of the stuffed data
// (first byte in the overhead byte). The length should be the length of the
// data, not including the overhead byte. The trailing byte is ignored.
// Only supports data up to 254 bytes in length.
void uncobs(uint8_t* data, size_t len);

#define MSG_TYPE_UNDERLYING_TYPE int8_t

enum class MsgType : MSG_TYPE_UNDERLYING_TYPE {
  UNKNOWN = 0,
  ID_REQUEST = 1,
  ID_RESPONSE = 2,
  CONTROL = 3,
  ESTOP = 4,
  GPS = 5,
  IMU = 6,
};

#define SET_MSG_TYPE(msg_type) \
  enum { Type = static_cast<MSG_TYPE_UNDERLYING_TYPE>(MsgType::msg_type) };

template <typename PayloadType>
class SerialMsg {
public:
  explicit SerialMsg()
    : type_(static_cast<MsgType>(PayloadType::Type)),
      overhead_(0xff),
      checksum_(0xdeadbeef),
      data(PayloadType()),
      trailer_(0) {}

  // Encodes the message for sending, computing the CRC32 checksum and stuffing
  // the data with consistent overhead byte stuffing. Once encoded, you can
  // send the data starting at the first byte of this object and sending
  // sizeof(SerialMsg<PayloadType>) bytes.
  void encode() {
    checksum_ = crc32(reinterpret_cast<uint8_t*>(&data), sizeof(data));
    cobs(&overhead_, sizeof(checksum_) + sizeof(data));
  }

  // Returns true if the message was successfully decoded. To be successfully
  // decoded, the message type must match and the checksum must validate. Once
  // decoded, the payload data can be accessed directly via the public .data
  // member.
  bool decode() {
    if (type_ != static_cast<MsgType>(PayloadType::Type)) {
      return false;
    }
    uncobs(&overhead_, sizeof(checksum_) + sizeof(data));
    return checksum_ == crc32(reinterpret_cast<uint8_t*>(&data), sizeof(data));
  }

  // Returns the checksum value if you care about it for some reason.
  uint32_t checksum() const {
    return checksum_;
  }

private:
  const MsgType type_;
  uint8_t overhead_;
  uint32_t checksum_;

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

  IdRequest()
    : ignored(42) {}

  int8_t ignored;
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

struct Gps {
  SET_MSG_TYPE(GPS);

  Gps()
    : bytes{0} {}

  uint8_t bytes[59];
} __attribute__((packed));

using GpsMsg = SerialMsg<Gps>;

struct Imu {
  SET_MSG_TYPE(IMU);

  Imu()
    : abs_orient_x(0.0f),
      abs_orient_y(0.0f),
      abs_orient_z(0.0f),
      raw_accel_x(0.0f),
      raw_accel_y(0.0f),
      raw_accel_z(0.0f),
      cal_system(0),
      cal_gyro(0),
      cal_accel(0),
      cal_mag(0) {}

  float abs_orient_x;
  float abs_orient_y;
  float abs_orient_z;
  float raw_accel_x;
  float raw_accel_y;
  float raw_accel_z;
  uint8_t cal_system;
  uint8_t cal_gyro;
  uint8_t cal_accel;
  uint8_t cal_mag;
} __attribute__((packed));

using ImuMsg = SerialMsg<Imu>;

#undef SET_MSG_TYPE

} // namespace rover12_comm