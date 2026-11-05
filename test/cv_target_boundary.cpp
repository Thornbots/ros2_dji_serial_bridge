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
#include <sys/ioctl.h>
#include <unistd.h>

#include <atomic>
#include <array>
#include <cassert>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <future>
#include <memory>
#include <mutex>
#include <vector>

#include "dji_serial_bridge/crc_dji.hpp"
#include "dji_serial_bridge/cv_target_freshness.hpp"
#include "dji_serial_bridge/dji_protocol.hpp"
#include "dji_serial_bridge/target_state_wire.hpp"

namespace builtin_interfaces::msg
{
struct Time {int32_t sec{0}; uint32_t nanosec{0};};
}
namespace dji_serial_bridge::msg
{
struct CVTarget
{
  using SharedPtr = std::shared_ptr<CVTarget>;
  struct {builtin_interfaces::msg::Time stamp;} header;
  float x{1.25F}, y{-2.5F}, z{0.75F};
  bool type_c_based_patrol{true}, turn_to_hit{true};
};
struct Point {double x{0}, y{0}, z{0};};
struct TargetState
{
  using SharedPtr = std::shared_ptr<TargetState>;
  struct {builtin_interfaces::msg::Time stamp;} header;
  builtin_interfaces::msg::Time detection_stamp;
  uint32_t robot_track_id{0x12345};
  float confidence{0.5F};
  Point center{1.0, 2.0, 3.0}, velocity{4.0, 5.0, 6.0}, acceleration{7.0, 8.0, 9.0};
  Point panel{10.0, 11.0, 12.0};
  float yaw{0.25F}, yaw_rate{-0.75F};
  std::array<float, 2> radius{0.2F, 0.3F}, z_offset{0.0F, 0.05F};
  bool valid{true}, type_c_based_patrol{true};
};
}  // namespace dji_serial_bridge::msg

static int pending_bytes = 0;
static int ioctl_rc = 0;
static bool write_blocked = false;
static bool short_write = false;
static std::vector<uint8_t> emitted;
static int writes = 0;

int test_ioctl(int, decltype(TIOCOUTQ), int * pending)
{
  *pending = pending_bytes;
  return ioctl_rc;
}
ssize_t test_write(int, const void * data, size_t size)
{
  ++writes;
  if (write_blocked) {errno = EAGAIN; return -1;}
  const auto * bytes = static_cast<const uint8_t *>(data);
  emitted.assign(bytes, bytes + size);
  return static_cast<ssize_t>(size - (short_write ? 1 : 0));
}

#define ioctl test_ioctl
#define write test_write
#define RCLCPP_WARN_THROTTLE(...) do {} while (false)
#define RCLCPP_ERROR_THROTTLE(...) do {} while (false)
#define RCLCPP_ERROR(...) do {} while (false)
#define RCLCPP_DEBUG(...) do {} while (false)

class Bridge
{
public:
  std::mutex write_mutex_;
  std::atomic<int64_t> clock_ns{1000000000LL};
  std::atomic<int> clock_reads{0};
  std::atomic<uint64_t> cv_target_msgs_tx_{0};
  std::atomic<uint64_t> target_state_msgs_tx_{0};
  int serial_fd_{-1};
  uint8_t tx_seq_{0};
  struct Now {int64_t ns; int64_t nanoseconds() const {return ns;}};
  Now now() {++clock_reads; return {clock_ns.load()};}

#include "bridge_write_methods.inc"
};
#undef ioctl
#undef write

static void reset()
{
  emitted.clear();
  writes = 0;
  pending_bytes = 0;
  ioctl_rc = 0;
  write_blocked = false;
  short_write = false;
}

static uint16_t u16_at(size_t off)
{
  uint16_t v;
  std::memcpy(&v, emitted.data() + sizeof(FrameHeader) + off, sizeof(v));
  return v;
}
static float f32_at(size_t off)
{
  float v;
  std::memcpy(&v, emitted.data() + sizeof(FrameHeader) + off, sizeof(v));
  return v;
}

static void test_wire_helpers()
{
  constexpr double kPi = 3.14159265358979323846;
  assert(dji_bridge::wrap_yaw(0.0) == 0.0F);
  assert(dji_bridge::wrap_yaw(-kPi) == static_cast<float>(-kPi));
  assert(dji_bridge::wrap_yaw(kPi) == static_cast<float>(-kPi));  // [-pi, pi)
  assert(dji_bridge::wrap_yaw(kPi - 1e-12) == static_cast<float>(-kPi));
  assert(std::fabs(dji_bridge::wrap_yaw(3.0 * kPi / 2.0) + kPi / 2.0) < 1e-6);
  assert(std::fabs(dji_bridge::wrap_yaw(-3.0 * kPi / 2.0) - kPi / 2.0) < 1e-6);
  assert(std::fabs(dji_bridge::wrap_yaw(20.0 * kPi + 0.5) - 0.5) < 1e-5);
  assert(std::fabs(dji_bridge::wrap_yaw(-20.0 * kPi - 0.5) + 0.5) < 1e-5);
  assert(dji_bridge::wrap_yaw(10.0) < static_cast<float>(kPi));
  assert(dji_bridge::wrap_yaw(10.0) >= static_cast<float>(-kPi));

  assert(dji_bridge::confidence_to_u8(-1.0) == 0);
  assert(dji_bridge::confidence_to_u8(0.0) == 0);
  assert(dji_bridge::confidence_to_u8(std::nan("")) == 0);
  assert(dji_bridge::confidence_to_u8(0.5) == 128);   // round(127.5)
  assert(dji_bridge::confidence_to_u8(0.2) == 51);
  assert(dji_bridge::confidence_to_u8(1.0) == 255);
  assert(dji_bridge::confidence_to_u8(7.0) == 255);

  assert(dji_bridge::age_ms_saturated(0) == 0);
  assert(dji_bridge::age_ms_saturated(1499999) == 1);
  assert(dji_bridge::age_ms_saturated(1500000) == 2);
  assert(dji_bridge::age_ms_saturated(65535LL * 1000000LL) == 65535);
  assert(dji_bridge::age_ms_saturated(65536LL * 1000000LL) == 65535);
  assert(dji_bridge::age_ms_saturated(1000000000000LL) == 65535);

  int64_t age = -1;
  assert(dji_bridge::stamp_age_ns(1, 0, 1050000000LL, &age) && age == 50000000LL);
  assert(!dji_bridge::stamp_age_ns(1, 1, 1000000000LL, &age));  // future
  assert(!dji_bridge::stamp_age_ns(0, 0, 1000000000LL, &age));  // zero
}

static void test_target_state()
{
  test_wire_helpers();
  static_assert(sizeof(TargetStatePayload) == 80, "");
  Bridge bridge;
  auto msg = std::make_shared<dji_serial_bridge::msg::TargetState>();
  bridge.clock_ns = 1100000000LL;
  msg->header.stamp = {1, 50000000U};       // 50 ms old
  msg->detection_stamp = {1, 20000000U};    // 80 ms old

  // Byte-exact payload at the contract's offsets; ages from one clock read.
  reset();
  bridge.clock_reads = 0;
  bridge.target_state_callback(msg);
  assert(writes == 1 && emitted.size() == 89 && bridge.clock_reads == 1);
  assert(emitted[0] == FRAME_HEAD && emitted[1] == 80 && emitted[2] == 0);
  assert(emitted[5] == 7 && emitted[6] == 0);  // msgType little-endian
  const float expect[] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 0.25F, -0.75F, 0.2F, 0.3F, 0.0F, 0.05F,
    10, 11, 12};
  for (size_t i = 0; i < 18; ++i) {
    assert(f32_at(4 * i) == expect[i]);
  }
  assert(u16_at(72) == 50 && u16_at(74) == 80);
  assert(u16_at(76) == 0x2345);
  assert(emitted[7 + 78] == 128);
  assert(emitted[7 + 79] == 0x03);
  const uint16_t crc = calculateCRC16(emitted.data(), emitted.size() - 2);
  assert(emitted[87] == (crc & 0xff) && emitted[88] == (crc >> 8));
  assert(emitted[4] == calculateCRC8(emitted.data(), CRC8_COVERAGE));
  assert(bridge.target_state_msgs_tx_ == 1);

  // Flags: valid and patrol are independent bits.
  for (int bits = 0; bits < 4; ++bits) {
    reset();
    msg->valid = (bits & 1) != 0;
    msg->type_c_based_patrol = (bits & 2) != 0;
    bridge.target_state_callback(msg);
    assert(writes == 1 && emitted[7 + 79] == bits);
  }
  msg->valid = true;
  msg->type_c_based_patrol = true;

  // Yaw wrap, confidence clamp and track id truncation on the wire.
  reset();
  msg->yaw = 7.0F;
  msg->confidence = 3.0F;
  msg->robot_track_id = 0xABCD1234U;
  bridge.target_state_callback(msg);
  assert(std::fabs(f32_at(36) - (7.0F - 6.2831853F)) < 1e-5F);
  assert(emitted[7 + 78] == 255 && u16_at(76) == 0x1234);
  msg->yaw = 0.25F;
  msg->confidence = 0.5F;
  msg->robot_track_id = 0x12345U;

  // Saturation: a detection older than 65.535 s still sends, at 65535.
  bridge.clock_ns = 100000000000LL;
  msg->header.stamp = {99, 900000000U};
  msg->detection_stamp = {1, 0};
  reset();
  bridge.target_state_callback(msg);
  assert(writes == 1 && u16_at(72) == 100 && u16_at(74) == 65535);
  bridge.clock_ns = 1100000000LL;
  msg->header.stamp = {1, 50000000U};
  msg->detection_stamp = {1, 20000000U};

  // Freshness: state stamp exactly 200 ms old, expired or malformed drops.
  for (auto stamp : {builtin_interfaces::msg::Time{0, 0}, {-1, 0}, {1, 1000000000U},
      {1, 100000001U} /* future */, {0, 900000000U} /* 200 ms */, {0, 900000001U} /* 199.9 */})
  {
    reset();
    msg->header.stamp = stamp;
    bridge.target_state_callback(msg);
    const bool fresh = stamp.sec == 0 && stamp.nanosec == 900000001U;
    assert(writes == (fresh ? 1 : 0));
    if (fresh) {assert(u16_at(72) == 200);}
  }
  msg->header.stamp = {1, 50000000U};

  // Zero, future or malformed detection stamps drop the whole frame.
  for (auto stamp : {builtin_interfaces::msg::Time{0, 0}, {-1, 0}, {1, 100000001U},
      {1, 1000000000U}})
  {
    reset();
    msg->detection_stamp = stamp;
    bridge.target_state_callback(msg);
    assert(writes == 0);
  }
  msg->detection_stamp = {1, 20000000U};

  // Backlog, unreadable queue and failed writes never retry.
  reset();
  pending_bytes = 89;
  bridge.target_state_callback(msg);
  assert(writes == 0);
  reset();
  ioctl_rc = -1;
  bridge.target_state_callback(msg);
  assert(writes == 0);
  for (bool blocked : {true, false}) {
    reset();
    write_blocked = blocked;
    short_write = !blocked;
    const uint64_t before = bridge.target_state_msgs_tx_;
    bridge.target_state_callback(msg);
    assert(writes == 1 && bridge.target_state_msgs_tx_ == before);
  }

  // A caller queued behind another writer ages after acquiring the lock.
  reset();
  bridge.clock_reads = 0;
  std::unique_lock<std::mutex> hold(bridge.write_mutex_);
  std::promise<void> started;
  auto ready = started.get_future();
  auto queued = std::async(std::launch::async, [&]() {
        started.set_value();
        bridge.target_state_callback(msg);
    });
  ready.wait();
  assert(bridge.clock_reads == 0);
  bridge.clock_ns = 1130000000LL;  // 80 ms state age, 110 ms detection age
  hold.unlock();
  queued.get();
  assert(writes == 1 && bridge.clock_reads == 1);
  assert(u16_at(72) == 80 && u16_at(74) == 110);
  bridge.clock_ns = 1250000000LL;  // 200 ms state age: expired after the wait
  reset();
  bridge.target_state_callback(msg);
  assert(writes == 0);

  // No path may send a TARGET_STATE without both stamps or with a wrong size.
  reset();
  TargetStatePayload payload{};
  bridge.clock_ns = 1100000000LL;
  assert(!bridge.send_frame(McbMsgType::TARGET_STATE,
    reinterpret_cast<const uint8_t *>(&payload), sizeof(payload)));
  assert(!bridge.send_frame(McbMsgType::TARGET_STATE,
    reinterpret_cast<const uint8_t *>(&payload), sizeof(payload), &msg->header.stamp));
  assert(!bridge.send_frame(McbMsgType::TARGET_STATE,
    reinterpret_cast<const uint8_t *>(&payload), 13, &msg->header.stamp, &msg->detection_stamp));
  assert(writes == 0);
}

int main()
{
  using dji_bridge::cv_target_is_fresh;
  assert(cv_target_is_fresh(1, 0, 1000000000LL));
  assert(cv_target_is_fresh(1, 0, 1199999999LL));
  assert(!cv_target_is_fresh(1, 0, 1200000000LL));
  assert(!cv_target_is_fresh(1, 0, 2000000000LL));
  assert(!cv_target_is_fresh(0, 0, 1000000000LL));
  assert(!cv_target_is_fresh(-1, 0, 1000000000LL));
  assert(!cv_target_is_fresh(1, 1000000000U, 1000000000LL));
  assert(!cv_target_is_fresh(1, 1, 1000000000LL));
  assert(!cv_target_is_fresh(1, 0, 0));
  assert(!cv_target_is_fresh(1, 0, -1));
  assert(cv_target_is_fresh(0, 1, 1));  // Positive simulation time below one second.
  assert(!cv_target_is_fresh(1, 0, 1700000000000000000LL));  // Different epoch.
  assert(!cv_target_is_fresh(1700000000, 0, 1000000000LL));

  Bridge bridge;
  auto msg = std::make_shared<dji_serial_bridge::msg::CVTarget>();
  msg->header.stamp.sec = 1;
  // Fresh idle points retain the exact 13-byte payload, flags and CRC.
  for (bool turn : {true, false}) {
    reset();
    msg->turn_to_hit = turn;
    bridge.cv_target_callback(msg);
    assert(writes == 1 && emitted.size() == 22);
    CvTargetPayload payload{};
    std::memcpy(&payload, emitted.data() + sizeof(FrameHeader), sizeof(payload));
    assert(payload.x == msg->x && payload.y == msg->y && payload.z == msg->z);
    assert(sizeof(payload) == 13 && emitted[7 + 12] == payload.flags);
    assert(payload.flags == (turn ? 6 : 2));
    assert(emitted[0] == FRAME_HEAD && emitted[1] == sizeof(payload));
    assert(emitted[4] == calculateCRC8(emitted.data(), CRC8_COVERAGE));
    const uint16_t crc = calculateCRC16(emitted.data(), emitted.size() - 2);
    assert(emitted[20] == (crc & 0xff) && emitted[21] == (crc >> 8));
    assert(msg->header.stamp.sec == 1 && msg->header.stamp.nanosec == 0);
  }
  assert(bridge.cv_target_msgs_tx_ == 2);

  // Model a stopped executor: receipt does not freshen the original stamp.
  for (bool turn : {true, false}) {
    reset();
    msg->turn_to_hit = turn;
    bridge.clock_ns = 2000000000LL;
    bridge.cv_target_callback(msg);
    assert(writes == 0 && emitted.empty() && bridge.cv_target_msgs_tx_ == 2);
  }
  bridge.clock_ns = 1000000000LL;
  for (auto stamp : {builtin_interfaces::msg::Time{0, 0}, {-1, 0}, {1, 1}, {1, 1000000000U}}) {
    reset();
    msg->header.stamp = stamp;
    bridge.cv_target_callback(msg);
    assert(writes == 0);
  }
  msg->header.stamp = {1, 0};
  reset();
  pending_bytes = 22;
  bridge.cv_target_callback(msg);
  assert(writes == 0);
  reset();
  ioctl_rc = -1;
  bridge.cv_target_callback(msg);
  assert(writes == 0);

  // A caller queued behind another writer must expire after acquiring the lock.
  reset();
  bridge.clock_reads = 0;
  std::unique_lock<std::mutex> hold(bridge.write_mutex_);
  std::promise<void> started;
  auto ready = started.get_future();
  auto queued = std::async(std::launch::async, [&]() {
        started.set_value();
        bridge.cv_target_callback(msg);
    });
  ready.wait();
  assert(bridge.clock_reads == 0);
  bridge.clock_ns = 1200000000LL;
  hold.unlock();
  queued.get();
  assert(writes == 0 && bridge.clock_reads == 1);

  // Nonblocking failure/short write is never retried with a stale decision.
  bridge.clock_ns = 1000000000LL;
  for (bool blocked : {true, false}) {
    reset();
    write_blocked = blocked;
    short_write = !blocked;
    bridge.cv_target_callback(msg);
    assert(writes == 1 && bridge.cv_target_msgs_tx_ == 2);
  }
  reset();
  CvTargetPayload payload{};
  assert(!bridge.send_frame(McbMsgType::CV_TARGET,
    reinterpret_cast<const uint8_t *>(&payload), sizeof(payload)));
  assert(writes == 0);  // No path may omit the decision stamp.

  // Other wire types retain their existing unstamped send behavior.
  pending_bytes = 22;
  assert(bridge.send_frame(McbMsgType::PING,
    reinterpret_cast<const uint8_t *>(&payload), 1));
  assert(writes == 1);

  test_target_state();
}
