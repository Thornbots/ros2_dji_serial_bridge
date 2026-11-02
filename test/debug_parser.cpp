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
#include <atomic>
#include <cassert>
#include <cstring>
#include <memory>
#include <vector>
#include "dji_serial_bridge/crc_dji.hpp"
#include "dji_serial_bridge/dji_protocol.hpp"

#define RCLCPP_DEBUG(...) ((void)0)
#define RCLCPP_WARN(...) ((void)0)
#define RCLCPP_WARN_THROTTLE(...) ((void)0)
namespace rclcpp
{
struct Time
{
  int64_t ns;
  Time(int64_t value, int)
  : ns(value) {}
};
}
namespace dji_serial_bridge::msg
{
struct DebugBlob
{
  struct {rclcpp::Time stamp{0, 0};} header;
  std::vector<uint8_t> data;
};
}
struct Publisher
{
  std::vector<dji_serial_bridge::msg::DebugBlob> messages;
  void publish(const dji_serial_bridge::msg::DebugBlob & msg) {messages.push_back(msg);}
};
struct Bridge
{
  std::atomic<size_t> frames_rx_{0}, crc8_errors_{0}, crc16_errors_{0}, debug_msgs_pub_{0};
  bool enforce_crc_ = true;
  int64_t byte_wire_ns_ = 86805;
  Publisher publisher;
  Publisher * debug_pub_ = &publisher;
  struct Clock {int get_clock_type() const {return 0;}} clock;
  const Clock * get_clock() const {return &clock;}
  size_t poses = 0;
  void handle_pose(const uint8_t *, uint16_t, const rclcpp::Time &) {++poses;}
  void handle_ref_sys(const uint8_t *, uint16_t, const rclcpp::Time &) {}
  void handle_ping(const uint8_t *, uint16_t, const rclcpp::Time &) {}
#include "bridge_debug_methods.inc"
};
std::vector<uint8_t> frame(const std::vector<uint8_t> & payload, uint16_t id = 6)
{
  std::vector<uint8_t> bytes(sizeof(FrameHeader) + payload.size() + 2);
  FrameHeader header{FRAME_HEAD, static_cast<uint16_t>(payload.size()), 0, 0, id};
  std::memcpy(bytes.data(), &header, sizeof(header));
  bytes[4] = calculateCRC8(bytes.data(), CRC8_COVERAGE);
  std::memcpy(bytes.data() + sizeof(header), payload.data(), payload.size());
  const uint16_t crc = calculateCRC16(bytes.data(), bytes.size() - 2);
  bytes[bytes.size() - 2] = crc & 0xff;
  bytes.back() = crc >> 8;
  return bytes;
}
int main()
{
  Bridge bridge;
  std::vector<uint8_t> buffer;
  std::vector<int64_t> stamps;
  const auto feed = [&](const std::vector<uint8_t> & bytes) {
      buffer.insert(buffer.end(), bytes.begin(), bytes.end());
      stamps.insert(stamps.end(), bytes.size(), 1000000000);
      bridge.process_rx_buffer(buffer, stamps);
    };
  const std::vector<uint8_t> payload{0, 0xa5, 0xff, 0, 6};
  const auto bytes = frame(payload);
  feed({bytes.begin(), bytes.begin() + 8});
  assert(bridge.publisher.messages.empty());
  feed({bytes.begin() + 8, bytes.end()});
  assert(bridge.publisher.messages.back().data == payload);
  assert(bridge.publisher.messages.back().header.stamp.ns ==
    1000000000 - static_cast<int64_t>(bytes.size()) * bridge.byte_wire_ns_);
  feed(frame({}));
  assert(bridge.publisher.messages.back().data.empty());
  const std::vector<uint8_t> maximum(DEBUG_MAX_PAYLOAD, 0x81);
  feed(frame(maximum));
  assert(bridge.publisher.messages.back().data == maximum);
  assert(bridge.publisher.messages.size() == 3);
  feed(frame(std::vector<uint8_t>(DEBUG_MAX_PAYLOAD + 1, 0x81)));
  assert(bridge.publisher.messages.size() == 3);
  // A huge declared length must not block the following valid frame.
  auto oversized = frame({});
  oversized[1] = 0xff;
  oversized[2] = 0xff;
  oversized[4] = calculateCRC8(oversized.data(), CRC8_COVERAGE);
  feed(oversized);
  feed(frame(payload));
  assert(bridge.publisher.messages.size() == 4);
  auto bad_crc = frame(payload);
  bad_crc.back() ^= 1;
  feed(bad_crc);
  feed(frame(std::vector<uint8_t>(sizeof(PosePayload)), 2));
  assert(bridge.publisher.messages.size() == 4);
  assert(bridge.crc16_errors_ == 1);
  assert(bridge.poses == 1);
  assert(DEBUG_MAX_PAYLOAD == 128);
  assert(bridge.debug_msgs_pub_ == 4);
}
