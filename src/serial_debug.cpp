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
#include <poll.h>

#include <chrono>
#include <csignal>
#include <fstream>
#include <iostream>

#include "dji_serial_bridge/diagnostic_frames.hpp"
namespace
{
volatile std::sig_atomic_t running = 1;
void stop(int) {running = 0;}
}  // namespace

int main(int argc, char ** argv)
{
  std::string device, log_path;
  int baud = 921600;
  double rate = 1.0;
  bool tx = false;
  try {
    for (int i = 1; i < argc; ++i) {
      const std::string arg = argv[i];
      if (arg == "--help" || arg == "-h") {
        std::puts(
            "Usage: serial_debug PORT [--baud 921600] [--log FILE] [--tx] "
            "[--tx-rate 1.0]");
        return 0;
      }
      if (arg == "--tx") {
        tx = true;
      } else if (arg == "--baud" && i + 1 < argc) {
        baud = std::stoi(argv[++i]);
      } else if (arg == "--log" && i + 1 < argc) {
        log_path = argv[++i];
      } else if (arg == "--tx-rate" && i + 1 < argc) {
        rate = std::stod(argv[++i]);
      } else if (device.empty() && !arg.empty() && arg[0] != '-') {
        device = arg;
      } else {
        throw std::runtime_error("unknown or incomplete argument " + arg);
      }
    }
    if (device.empty()) {
      throw std::runtime_error("serial port is required");
    }
    dji_diagnostic::SerialPort port(device, baud);
    std::ofstream log;
    if (!log_path.empty()) {
      log.open(log_path, std::ios::binary);
      if (!log) {
        throw std::runtime_error("cannot open " + log_path);
      }
    }
    std::printf("Opened %s @ %d\n", device.c_str(), baud);
    std::signal(SIGINT, stop);
    std::signal(SIGTERM, stop);
    dji_diagnostic::FrameParser parser;
    uint8_t sequence = 0;
    using Clock = std::chrono::system_clock;
    auto next_tx = Clock::now();
    while (running) {
      pollfd reader{port.fd(), POLLIN, 0};
      poll(&reader, 1, 50);
      uint8_t buffer[4096];
      const auto count = read(port.fd(), buffer, sizeof(buffer));
      if (count > 0) {
        if (log.is_open()) {
          log.write(reinterpret_cast<char *>(buffer), count);
          log.flush();
        }
        std::printf("[RX %4zd] %s\n", count,
                    dji_diagnostic::hex(buffer, count).c_str());
        parser.append(buffer, count);
      }
      if (tx && Clock::now() >= next_tx) {
        const auto frame = dji_diagnostic::test_frame(sequence++);
        std::size_t sent = 0;
        while (sent < frame.size()) {
          const auto count_written =
            write(port.fd(), frame.data() + sent, frame.size() - sent);
          if (count_written > 0) {
            sent += count_written;
          } else if (errno == EAGAIN || errno == EINTR) {
            pollfd writer{port.fd(), POLLOUT, 0};
            poll(&writer, 1, 50);
          } else {
            throw std::runtime_error(std::strerror(errno));
          }
        }
        std::printf("[TX] %s\n",
                    dji_diagnostic::hex(frame.data(), frame.size()).c_str());
        if (rate == 0) {
          throw std::runtime_error("division by zero in --tx-rate");
        }
        next_tx = Clock::now() + std::chrono::duration_cast<Clock::duration>(
                                     std::chrono::duration<double>(1.0 / rate));
      }
      std::fflush(stdout);
    }
    std::printf(
        "\nStopping...\n\n===== Statistics =====\nValid frames : %zu\nBad CRC8 "
        "    : %zu\n"
        "Bad CRC16    : %zu\nDesyncs      : %zu\n",
        parser.frames, parser.bad_crc8, parser.bad_crc16, parser.desyncs);
  } catch (const std::exception & error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
