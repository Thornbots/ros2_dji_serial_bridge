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


#include <pty.h>
#include <unistd.h>
#include <gtest/gtest.h>
#include <chrono>
#include <vector>
#include "dji_serial_bridge/node_factory.hpp"
#include "dji_serial_bridge/dji_protocol.hpp"
#include "dji_serial_bridge/crc_dji.hpp"
#include "dji_serial_bridge/msg/mcb_data.hpp"

TEST(McbData, RealParserValidatesFramesAndPreservesOpaqueBytes)
{
  int master, slave;
  char name[128];
  ASSERT_EQ(openpty(&master, &slave, name, nullptr, nullptr), 0);
  struct Fds
  {
    int master, slave;
    ~Fds() {close(master); close(slave);}
  } fds{master, slave};
  auto context = std::make_shared<rclcpp::Context>();
  rclcpp::InitOptions init;
  init.set_domain_id(89);
  context->init(0, nullptr, init);
  struct Shutdown
  {
    std::shared_ptr<rclcpp::Context> context;
    ~Shutdown() {context->shutdown("MCB data test complete");}
  } shutdown{context};
  auto options = rclcpp::NodeOptions().context(context).parameter_overrides({
    rclcpp::Parameter("device", name), rclcpp::Parameter("debug_log", false),
    rclcpp::Parameter("diag_interval_s", 0), rclcpp::Parameter("read_poll_ms", 1)});
  auto bridge = make_serial_bridge(options);
  auto receiver = std::make_shared<rclcpp::Node>(
    "data_receiver", rclcpp::NodeOptions().context(context));
  std::vector<dji_serial_bridge::msg::MCBData> received;
  auto sub = receiver->create_subscription<dji_serial_bridge::msg::MCBData>(
    "/dji_serial_bridge/mcb_data", 100,
    [&](dji_serial_bridge::msg::MCBData::ConstSharedPtr msg) {received.push_back(*msg);});
  rclcpp::ExecutorOptions executor_options;
  executor_options.context = context;
  rclcpp::executors::SingleThreadedExecutor executor(executor_options);
  executor.add_node(receiver);
  auto wait = [&](const auto & predicate) {
      const auto end = std::chrono::steady_clock::now() + std::chrono::seconds(5);
      while (!predicate() && std::chrono::steady_clock::now() < end) {
        executor.spin_once(std::chrono::milliseconds(5));
      }
      return predicate();
    };
  ASSERT_TRUE(wait([&] {return sub->get_publisher_count() == 1;}));
  auto frame = [](const std::vector<uint8_t> & payload) {
      FrameHeader header{FRAME_HEAD, static_cast<uint16_t>(payload.size()), 0, 0, 6};
      header.crc8 = calculateCRC8(reinterpret_cast<uint8_t *>(&header), CRC8_COVERAGE);
      const auto bytes = reinterpret_cast<uint8_t *>(&header);
      std::vector<uint8_t> result(bytes, bytes + sizeof(header));
      result.insert(result.end(), payload.begin(), payload.end());
      const auto crc = calculateCRC16(result.data(), result.size());
      result.push_back(crc & 0xff);
      result.push_back(crc >> 8);
      return result;
    };
  auto send = [&](const std::vector<uint8_t> & bytes) {
      ASSERT_EQ(write(master, bytes.data(), bytes.size()), static_cast<ssize_t>(bytes.size()));
    };
  auto crc8_bad = frame({42, 0x55});
  crc8_bad[4] ^= 1;
  send(crc8_bad);
  auto crc16_bad = frame({42, 0x55});
  crc16_bad.back() ^= 1;
  send(crc16_bad);
  send(frame({}));
  send(frame(std::vector<uint8_t>(50, 0x55)));
  // Deliver a truncated frame in two reads: no publication before its final byte.
  auto split = frame({42, 0, 0xa5, 0xff});
  send(std::vector<uint8_t>(split.begin(), split.end() - 1));
  const auto end = std::chrono::steady_clock::now() + std::chrono::milliseconds(100);
  while (std::chrono::steady_clock::now() < end) {
    executor.spin_once(std::chrono::milliseconds(5));
  }
  EXPECT_TRUE(received.empty());
  const auto before = receiver->now().nanoseconds();
  send({split.back()});
  send(frame({7}));
  std::vector<uint8_t> maximum(49, 0xff);
  maximum[0] = 254;
  send(frame(maximum));
  ASSERT_TRUE(wait([&] {return received.size() == 3;}));
  EXPECT_EQ(received[0].channel, 42);
  EXPECT_EQ(received[0].data, (std::vector<uint8_t>{0, 0xa5, 0xff}));
  EXPECT_TRUE(received[0].header.frame_id.empty());
  const auto stamp = rclcpp::Time(received[0].header.stamp).nanoseconds();
  EXPECT_GE(stamp, before - static_cast<int64_t>(split.size()) * (10000000000LL / 115200));
  EXPECT_LE(stamp, receiver->now().nanoseconds());
  EXPECT_EQ(received[1].channel, 7);
  EXPECT_TRUE(received[1].data.empty());
  EXPECT_EQ(received[2].channel, 254);
  EXPECT_EQ(received[2].data, std::vector<uint8_t>(48, 0xff));
}
