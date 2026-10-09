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
#include <algorithm>
#include <array>
#include <cstdio>
#include <vector>

#include "dji_serial_bridge/diagnostic_serial.hpp"

namespace dji_diagnostic
{
inline uint8_t crc8(const uint8_t * data, std::size_t size)
{
  uint8_t crc = 0xff;
  for (std::size_t i = 0; i < size; ++i) {
    crc ^= data[i];
    for (int bit = 0; bit < 8; ++bit) {
      crc = (crc >> 1) ^ ((crc & 1) ? 0x8c : 0);
    }
  }
  return crc;
}
// Preserve serial_debug.py's repeated 16-entry table for diagnostic parity.
// This is not the production bridge's DJI CRC16; see README.md#usage.
inline uint16_t crc16(const uint8_t * data, std::size_t size)
{
  constexpr std::array<uint16_t, 16> table = {
    0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf,
    0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e, 0xf8f7};
  uint16_t crc = 0xffff;
  for (std::size_t i = 0; i < size; ++i) {
    crc = (crc >> 8) ^ table[(crc ^ data[i]) & 15];
  }
  return crc;
}
inline uint16_t read16(const uint8_t * bytes)
{
  return bytes[0] | (static_cast<uint16_t>(bytes[1]) << 8);
}
inline std::vector<uint8_t> test_frame(uint8_t sequence)
{
  std::vector<uint8_t> frame = {0xa5, 8, 0, sequence, 0, 4, 0};
  frame[4] = crc8(frame.data(), 4);
  const float payload[] = {5.0f, 3.0f};
  const auto * bytes = reinterpret_cast<const uint8_t *>(payload);
  frame.insert(frame.end(), bytes, bytes + sizeof(payload));
  const auto crc = crc16(frame.data(), frame.size());
  frame.push_back(crc & 255);
  frame.push_back(crc >> 8);
  return frame;
}
class FrameParser {
public:
  std::size_t frames = 0, bad_crc8 = 0, bad_crc16 = 0, desyncs = 0;
  void append(const uint8_t * bytes, std::size_t count)
  {
    buffer_.insert(buffer_.end(), bytes, bytes + count);
    while (buffer_.size() >= 7) {
      const auto sync = std::find(buffer_.begin(), buffer_.end(), 0xa5);
      if (sync == buffer_.end()) {
        std::printf("[DESYNC] discarded %zu bytes\n", buffer_.size());
        ++desyncs;
        buffer_.clear();
        return;
      }
      if (sync != buffer_.begin()) {
        const auto size = static_cast<std::size_t>(sync - buffer_.begin());
        std::printf("[GARBAGE] skipped %zu bytes: %s\n", size,
                    dji_diagnostic::hex(buffer_.data(), size).c_str());
        ++desyncs;
        buffer_.erase(buffer_.begin(), sync);
      }
      if (buffer_.size() < 7) {
        return;
      }
      const auto length = read16(buffer_.data() + 1);
      const auto crc_header = crc8(buffer_.data(), 4);
      if (crc_header != buffer_[4]) {
        std::printf("[BAD CRC8] recv=0x%02X calc=0x%02X header=%s\n",
                    buffer_[4], crc_header,
                    dji_diagnostic::hex(buffer_.data(), 7).c_str());
        ++bad_crc8;
        buffer_.erase(buffer_.begin());
        continue;
      }
      const std::size_t total = 9 + length;
      if (total > 4096) {
        std::printf("[BAD LENGTH] %zu\n", total);
        buffer_.erase(buffer_.begin());
        continue;
      }
      if (buffer_.size() < total) {
        return;
      }
      const auto received = read16(buffer_.data() + total - 2);
      const auto calculated = crc16(buffer_.data(), total - 2);
      if (received != calculated) {
        std::printf("[BAD CRC16] recv=0x%04X calc=0x%04X\nframe: %s\n",
                    received, calculated,
                    dji_diagnostic::hex(buffer_.data(), total).c_str());
        ++bad_crc16;
        buffer_.erase(buffer_.begin());
        continue;
      }
      ++frames;
      std::printf("[FRAME OK] seq=%3u msg=0x%04X len=%3u payload=%s\n",
                  buffer_[3], read16(buffer_.data() + 5), length,
                  dji_diagnostic::hex(buffer_.data() + 7, length).c_str());
      buffer_.erase(buffer_.begin(), buffer_.begin() + total);
    }
  }

private:
  std::vector<uint8_t> buffer_;
};
}  // namespace dji_diagnostic
