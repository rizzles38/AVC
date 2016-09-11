#include <rover12_drivers/messenger.h>

#include <utility>

#include <ros/ros.h>

namespace rover12_drivers {

Messenger::Messenger()
  : sp_(nullptr),
    buffer_{0},
    buffer_idx_(-1) {}

void Messenger::addDevice(std::string device) {
  device_list_.push_back(std::move(device));
}

void Messenger::connect(Board board) {
  if (device_list_.size() == 0) return;

  // TODO: identify board and keep connection to first one of correct type
  sp_.reset(new SerialPort(device_list_[0]));

  ROS_INFO_STREAM("Connected to " << device_list_[0]);
}

void Messenger::spin() {
  if (!sp_) return;

  uint8_t buf[1024];
  auto nread = sp_->read(buf, sizeof(buf));
  int i = 0;
  //ROS_INFO_STREAM("nread = " << nread);
  while (i < nread) {
    //ROS_INFO_STREAM("buf[" << i << "] = 0x" << std::hex << (int)buf[i]);

    // Is this byte a message terminator?
    if (buf[i] == 0x0) {
      // If we've already synchronized, dispatch the message.
      if (buffer_idx_ > 0) {
        //ROS_INFO_STREAM("buffer_idx_ = " << buffer_idx_);
        dispatchMessage(buffer_idx_);
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

void Messenger::dispatchMessage(int len) {
  int8_t type = static_cast<int8_t>(buffer_[0]);
  switch (static_cast<rover12_comm::MsgType>(type)) {
    case rover12_comm::MsgType::GPS: {
      rover12_comm::GpsMsg* ptr = reinterpret_cast<rover12_comm::GpsMsg*>(buffer_);
      ptr->decode();
      gps_callback_(*ptr);
    } break;

    case rover12_comm::MsgType::IMU: {
      rover12_comm::ImuMsg* ptr = reinterpret_cast<rover12_comm::ImuMsg*>(buffer_);
      ptr->decode();
      imu_callback_(*ptr);
    } break;

    default:
      ROS_WARN_STREAM("Unknown message type from serial port: [0x" << std::hex << (int)type << "]");
  }
}

} // namespace rover12_drivers