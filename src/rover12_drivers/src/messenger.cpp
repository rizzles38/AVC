#include <rover12_drivers/messenger.h>

#include <utility>

#include <ros/ros.h>

namespace rover12_drivers {

Messenger::Messenger(const std::string& device)
  : sp_(device, 115200, serial::Timeout::simpleTimeout(5)),
    buffer_{0},
    buffer_idx_(-1) {
  if (sp_.isOpen()) {
    ROS_INFO_STREAM("Connected to " << device);
  } else {
    ROS_ERROR_STREAM("Serial port not open!");
  }
}

void Messenger::spin() {
  if (!sp_.isOpen()) return;

  uint8_t buf[1024];
  auto nread = sp_.read(buf, sizeof(buf));
  int i = 0;
  while (i < nread) {
    // Is this byte a message terminator?
    if (buf[i] == 0x0) {
      // If we've already synchronized, dispatch the message.
      if (buffer_idx_ > 0) {
        dispatchMessage();
      }

      // Reset message buffer index.
      buffer_idx_ = 0;
      ++i;
      continue;
    }

    // If we haven't synchronized yet, skip these bytes.
    if (buffer_idx_ < 0) {
      ++i;
      continue;
    }

    // Otherwise copy them into the message buffer.
    buffer_[buffer_idx_] = buf[i];
    ++buffer_idx_;
    ++i;
  }
}

void Messenger::dispatchMessage() {
  int8_t type = static_cast<int8_t>(buffer_[0]);
  switch (static_cast<rover12_comm::MsgType>(type)) {
    case rover12_comm::MsgType::GPS: {
      auto ptr = reinterpret_cast<rover12_comm::GpsMsg*>(buffer_);
      if (gps_callback_ && ptr->decode()) {
        gps_callback_(*ptr);
      } else {
        ROS_WARN_STREAM("Bad GPS message, skipping...");
      }
    } break;

    case rover12_comm::MsgType::IMU: {
      auto ptr = reinterpret_cast<rover12_comm::ImuMsg*>(buffer_);
      if (imu_callback_ && ptr->decode()) {
        imu_callback_(*ptr);
      } else {
        ROS_WARN_STREAM("Bad IMU message, skipping...");
      }
    } break;

    case rover12_comm::MsgType::IMU_CAL: {
      auto ptr = reinterpret_cast<rover12_comm::ImuCalMsg*>(buffer_);
      if (imu_cal_callback_ && ptr->decode()) {
        imu_cal_callback_(*ptr);
      } else {
        ROS_WARN_STREAM("Bad IMU calibration message, skipping...");
      }
    } break;

    case rover12_comm::MsgType::WHEEL_ENC: {
      auto ptr = reinterpret_cast<rover12_comm::WheelEncMsg*>(buffer_);
      if (wheel_enc_callback_ && ptr->decode()) {
        wheel_enc_callback_(*ptr);
      } else {
        ROS_WARN_STREAM("Bad wheel encoder message, skipping...");
      }
    } break;

    case rover12_comm::MsgType::ESTOP: {
      auto ptr = reinterpret_cast<rover12_comm::EstopMsg*>(buffer_);
      if (estop_callback_ && ptr->decode()) {
        estop_callback_(*ptr);
      } else {
        ROS_WARN_STREAM("Bad estop message, skipping...");
      }
    } break;

    case rover12_comm::MsgType::DEBUG: {
      auto ptr = reinterpret_cast<rover12_comm::DebugMsg*>(buffer_);
      if (ptr->decode()) {
        switch (ptr->data.data_type) {
          case rover12_comm::Debug::DataType::NONE:
            ROS_INFO_STREAM(ptr->data.name);
            break;

          case rover12_comm::Debug::DataType::INT:
            ROS_INFO_STREAM(ptr->data.name << ptr->data.value.i);
            break;

          case rover12_comm::Debug::DataType::FLOAT:
            ROS_INFO_STREAM(ptr->data.name << ptr->data.value.f);
            break;

          default:
            ROS_WARN_STREAM("Unknown debug message data type: " <<
                static_cast<int>(ptr->data.data_type));
            break;
        }
      }
    } break;

    default:
      ROS_WARN_STREAM("Unknown message from serial port! [type = 0x" << std::hex << (int)type << "]");
  }
}

} // namespace rover12_drivers
