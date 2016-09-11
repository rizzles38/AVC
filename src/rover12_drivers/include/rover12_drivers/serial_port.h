#pragma once

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <cstddef>
#include <string>

namespace rover12_drivers {

class SerialPort {
public:
  explicit SerialPort(const std::string& device);
  ~SerialPort();

  ssize_t read(void* buf, size_t count);
  ssize_t write(void* buf, size_t count);

private:
  int fd_;
  termios tty_old_;
};

} // namespace rover12_drivers
