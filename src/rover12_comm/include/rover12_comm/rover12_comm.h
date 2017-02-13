#pragma once

#include <stddef.h>
#include <stdint.h>
#include <string.h>

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
  CONTROL = 1,
  ESTOP = 2,
  GPS = 3,
  IMU = 4,
  IMU_CAL = 5,
  WHEEL_ENC = 6,
  DEBUG = 7,
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

// Sent from the computer to the control board. Contains the desired steering
// and throttle servo durations in microseconds. These should range from 1000
// to 2000 (with an approximate center point at 1500).
struct Control {
  SET_MSG_TYPE(CONTROL);

  Control()
    : steering_us(1556),
      throttle_us(1500) {}

  int16_t steering_us;
  int16_t throttle_us;
} __attribute__((packed));

using ControlMsg = SerialMsg<Control>;

// Sent from the computer to the control board. If true, we authorize the
// hardware to be in autonomous mode. Also sent from control board to computer
// reporting the actual status of autonomous mode.
struct Estop {
  SET_MSG_TYPE(ESTOP);

  Estop()
    : autonomous(false) {}

  bool autonomous;
} __attribute__((packed));

using EstopMsg = SerialMsg<Estop>;

// Sent from the sensor board to the computer. Contains the raw GPS data packet.
struct Gps {
  SET_MSG_TYPE(GPS);

  Gps()
    : bytes{0} {}

  uint8_t bytes[59];
} __attribute__((packed));

using GpsMsg = SerialMsg<Gps>;

// Sent from the sensor board to the computer. Contains the orientation estimate
// quaternion, linear accelerations, and angular velocities.
struct Imu {
  SET_MSG_TYPE(IMU);

  Imu()
    : orient_x(0.0f),
      orient_y(0.0f),
      orient_z(0.0f),
      orient_w(1.0f),
      lin_accel_x(0.0f),
      lin_accel_y(0.0f),
      lin_accel_z(0.0f),
      ang_vel_x(0.0f),
      ang_vel_y(0.0f),
      ang_vel_z(0.0f) {}

  float orient_x;
  float orient_y;
  float orient_z;
  float orient_w;
  float lin_accel_x;
  float lin_accel_y;
  float lin_accel_z;
  float ang_vel_x;
  float ang_vel_y;
  float ang_vel_z;
} __attribute__((packed));

using ImuMsg = SerialMsg<Imu>;

// Sent from the sensor board to the computer. Contains the IMU calibration
// status for the sensor fusion system, gyro, accelerometer, and magnetometer.
struct ImuCal {
  SET_MSG_TYPE(IMU_CAL);

  ImuCal()
    : system(0),
      gyro(0),
      accel(0),
      mag(0) {}

  uint8_t system;
  uint8_t gyro;
  uint8_t accel;
  uint8_t mag;

} __attribute__((packed));

using ImuCalMsg = SerialMsg<ImuCal>;

// Sent from the control board to the computer. Contains the raw tick counts
// from the wheel encoders.
struct WheelEnc {
  SET_MSG_TYPE(WHEEL_ENC);

  WheelEnc()
    : count_rear_left(0),
      count_rear_right(0) {}

  uint32_t hw_timestamp_ms;
  int32_t count_rear_left;
  int32_t count_rear_right;
} __attribute__((packed));

using WheelEncMsg = SerialMsg<WheelEnc>;

// Sent from either the sensor or control board to the computer. Contains debug
// information for the purpose of debugging firmware. Contains a short string
// message and optionally an int or a float value.
struct Debug {
  SET_MSG_TYPE(DEBUG);

  enum class DataType : int8_t {
    NONE,
    INT,
    FLOAT,
  };

  Debug()
    : data_type(DataType::NONE) {
    for (int i = 0; i < sizeof(name); ++i) {
      name[i] = '\0';
    }
    value.i = 0;
  }

  void setInt(const char* s, int32_t i) {
    value.i = i;
    data_type = DataType::INT;
    strncpy(name, s, 30);
  }

  void setFloat(const char* s, float f) {
    value.f = f;
    data_type = DataType::FLOAT;
    strncpy(name, s, 30);
  }

  void setName(const char* s) {
    strncpy(name, s, 30);
  }

  DataType data_type;
  char name[31];
  union {
    int32_t i;
    float f;
  } value;
} __attribute__((packed));

using DebugMsg = SerialMsg<Debug>;

#undef SET_MSG_TYPE

} // namespace rover12_comm
