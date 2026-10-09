// Copyright 2026 Thornbots
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#pragma once
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace dji_diagnostic
{
inline speed_t baud_speed(int baud)
{
  switch (baud) {
    case 50:
      return B50;
    case 75:
      return B75;
    case 110:
      return B110;
    case 134:
      return B134;
    case 150:
      return B150;
    case 200:
      return B200;
    case 300:
      return B300;
    case 600:
      return B600;
    case 1200:
      return B1200;
    case 1800:
      return B1800;
    case 2400:
      return B2400;
    case 4800:
      return B4800;
    case 9600:
      return B9600;
    case 19200:
      return B19200;
    case 38400:
      return B38400;
    case 57600:
      return B57600;
    case 115200:
      return B115200;
    case 230400:
      return B230400;
#ifdef B460800
    case 460800:
      return B460800;
#endif
#ifdef B500000
    case 500000:
      return B500000;
#endif
#ifdef B576000
    case 576000:
      return B576000;
#endif
#ifdef B921600
    case 921600:
      return B921600;
#endif
#ifdef B1000000
    case 1000000:
      return B1000000;
#endif
#ifdef B1152000
    case 1152000:
      return B1152000;
#endif
#ifdef B1500000
    case 1500000:
      return B1500000;
#endif
#ifdef B2000000
    case 2000000:
      return B2000000;
#endif
#ifdef B2500000
    case 2500000:
      return B2500000;
#endif
#ifdef B3000000
    case 3000000:
      return B3000000;
#endif
#ifdef B3500000
    case 3500000:
      return B3500000;
#endif
#ifdef B4000000
    case 4000000:
      return B4000000;
#endif
    default:
      throw std::runtime_error("unsupported baudrate " + std::to_string(baud));
  }
}
class SerialPort {
public:
  SerialPort(const std::string & device, int baud)
  {
    fd_ = open(device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) {
      throw std::runtime_error(std::strerror(errno));
    }
    try {
      termios options{};
      if (tcgetattr(fd_, &options) != 0) {
        throw std::runtime_error(std::strerror(errno));
      }
      cfmakeraw(&options);
      options.c_cflag |= CLOCAL | CREAD;
      options.c_cflag &= ~(PARENB | CSTOPB | CSIZE | CRTSCTS);
      options.c_cflag |= CS8;
      options.c_cc[VMIN] = 0;
      options.c_cc[VTIME] = 0;
      cfsetispeed(&options, baud_speed(baud));
      cfsetospeed(&options, baud_speed(baud));
      if (tcsetattr(fd_, TCSANOW, &options) != 0) {
        throw std::runtime_error(std::strerror(errno));
      }
    } catch (...) {
      close(fd_);
      throw;
    }
  }
  ~SerialPort() {close(fd_);}
  SerialPort(const SerialPort &) = delete;
  SerialPort & operator=(const SerialPort &) = delete;
  int fd() const {return fd_;}

private:
  int fd_;
};
inline std::string hex(
  const unsigned char * data, std::size_t size,
  bool spaces = true)
{
  std::ostringstream out;
  out << std::hex << std::setfill('0');
  for (std::size_t i = 0; i < size; ++i) {
    if (spaces && i) {
      out << ' ';
    }
    out << std::setw(2) << static_cast<unsigned>(data[i]);
  }
  return out.str();
}
}  // namespace dji_diagnostic
