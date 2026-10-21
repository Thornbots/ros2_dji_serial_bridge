// Copyright 2026 Thornbots
// SPDX-License-Identifier: Apache-2.0
#include <sys/ioctl.h>
#include <unistd.h>

#include <atomic>
#include <cassert>
#include <cerrno>
#include <cstring>
#include <future>
#include <memory>
#include <mutex>
#include <vector>

#include "dji_serial_bridge/crc_dji.hpp"
#include "dji_serial_bridge/cv_target_freshness.hpp"
#include "dji_serial_bridge/dji_protocol.hpp"

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
  uint16_t delay_ms{119};
  bool fire{true}, type_c_based_patrol{true}, turn_to_hit{true}, patrol{true};
};
}

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
  // Fresh fire and aim-only retain the exact payload, flags, delay and CRC.
  for (bool fire : {true, false}) {
    reset();
    msg->fire = fire;
    bridge.cv_target_callback(msg);
    assert(writes == 1 && emitted.size() == 24);
    CvTargetPayload payload{};
    std::memcpy(&payload, emitted.data() + sizeof(FrameHeader), sizeof(payload));
    assert(payload.x == msg->x && payload.y == msg->y && payload.z == msg->z);
    assert(payload.delay_ms == msg->delay_ms);
    assert(payload.flags == (fire ? 15 : 14));
    assert(emitted[0] == FRAME_HEAD && emitted[1] == sizeof(payload));
    assert(emitted[4] == calculateCRC8(emitted.data(), CRC8_COVERAGE));
    const uint16_t crc = calculateCRC16(emitted.data(), emitted.size() - 2);
    assert(emitted[22] == (crc & 0xff) && emitted[23] == (crc >> 8));
    assert(msg->header.stamp.sec == 1 && msg->header.stamp.nanosec == 0);
  }
  assert(bridge.cv_target_msgs_tx_ == 2);

  // Model a stopped executor: receipt does not freshen the original stamp.
  for (bool fire : {true, false}) {
    reset();
    msg->fire = fire;
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
  pending_bytes = 24;
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
  pending_bytes = 24;
  assert(bridge.send_frame(McbMsgType::PING,
    reinterpret_cast<const uint8_t *>(&payload), 1));
  assert(writes == 1);
}
