// Copyright 2026 Thornbots
// SPDX-License-Identifier: Apache-2.0
#ifndef DJI_SERIAL_BRIDGE__CV_TARGET_FRESHNESS_HPP_
#define DJI_SERIAL_BRIDGE__CV_TARGET_FRESHNESS_HPP_

#include <cstdint>

namespace dji_bridge
{
// Decision stamps and now use the same ROS clock, including use_sim_time.
// Reject unknown, malformed and future stamps; never re-stamp a decision.
inline bool cv_target_is_fresh(int32_t sec, uint32_t nanosec, int64_t now_ns)
{
  constexpr int64_t kSecondNs = 1000000000LL;
  constexpr int64_t kMaxAgeNs = 200000000LL;
  if (sec < 0 || nanosec >= kSecondNs || now_ns <= 0) {
    return false;
  }
  const int64_t stamp_ns = static_cast<int64_t>(sec) * kSecondNs + nanosec;
  return stamp_ns > 0 && stamp_ns <= now_ns && now_ns - stamp_ns < kMaxAgeNs;
}
}  // namespace dji_bridge

#endif  // DJI_SERIAL_BRIDGE__CV_TARGET_FRESHNESS_HPP_
