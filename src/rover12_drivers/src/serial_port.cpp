#include <rover12_drivers/serial_port.h>

#include <cerrno>
#include <cstdlib>
#include <cstring>

#include <ros/ros.h>

namespace rover12_drivers {

SerialPort::SerialPort(const std::string& device) {
  fd_ = open(device.c_str(), O_RDWR | O_NOCTTY);

  termios tty;
  std::memset(&tty, 0, sizeof(tty));

  if (tcgetattr(fd_, &tty) != 0) {
    ROS_ERROR_STREAM("Error " << errno << " from tcgetattr: " << std::strerror(errno));
    std::exit(EXIT_FAILURE);
  }

  tty_old_ = tty;

  cfsetospeed(&tty, (speed_t)B115200);
  cfsetispeed(&tty, (speed_t)B115200);

  tty.c_cflag &= ~PARENB;            // Make 8n1
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8;

  tty.c_cflag &= ~CRTSCTS;        // no flow control
  tty.c_cc[VMIN] =  1;            // read doesn't block
  tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout
  tty.c_cflag |= CREAD | CLOCAL;  // turn on READ & ignore ctrl lines

  cfmakeraw(&tty);

  tcflush(fd_, TCIFLUSH);
  if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
    ROS_ERROR_STREAM("Error " << errno << " from tcsetattr: " << std::strerror(errno));
  }
}

SerialPort::~SerialPort() {
  tcflush(fd_, TCIFLUSH);
  if (tcsetattr(fd_, TCSANOW, &tty_old_) != 0) {
    ROS_ERROR_STREAM("Error " << errno << " from tcsetattr: " << std::strerror(errno));
  }
  close(fd_);
}

ssize_t SerialPort::read(void* buf, size_t count) {
  return ::read(fd_, buf, count);
}

ssize_t SerialPort::write(const void* buf, size_t count) {
  return ::write(fd_, buf, count);
}

} // namespace rover12_drivers
