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
#ifndef DJI_SERIAL_BRIDGE__CV_TARGET_FRESHNESS_HPP_
#define DJI_SERIAL_BRIDGE__CV_TARGET_FRESHNESS_HPP_

#include <cstdint>

namespace dji_bridge
{
// Decision stamps and now use the same ROS clock, including use_sim_time.
// Reject unknown, malformed and future stamps; never re-stamp a decision.
// On success age_ns is now - stamp (>= 0).
inline bool stamp_age_ns(int32_t sec, uint32_t nanosec, int64_t now_ns, int64_t * age_ns)
{
  constexpr int64_t kSecondNs = 1000000000LL;
  if (sec < 0 || nanosec >= kSecondNs || now_ns <= 0) {
    return false;
  }
  const int64_t stamp_ns = static_cast<int64_t>(sec) * kSecondNs + nanosec;
  if (stamp_ns <= 0 || stamp_ns > now_ns) {
    return false;
  }
  *age_ns = now_ns - stamp_ns;
  return true;
}

// Fresh means younger than 200 ms; exactly 200 ms is expired.
inline bool cv_target_is_fresh(int32_t sec, uint32_t nanosec, int64_t now_ns)
{
  constexpr int64_t kMaxAgeNs = 200000000LL;
  int64_t age_ns = 0;
  return stamp_age_ns(sec, nanosec, now_ns, &age_ns) && age_ns < kMaxAgeNs;
}

// Wire age: nearest millisecond, saturating at 65535.
inline uint16_t age_ms_saturated(int64_t age_ns)
{
  constexpr int64_t kMs = 1000000LL;
  const int64_t ms = (age_ns + kMs / 2) / kMs;
  return ms > 65535 ? static_cast<uint16_t>(65535) : static_cast<uint16_t>(ms);
}
}  // namespace dji_bridge

#endif  // DJI_SERIAL_BRIDGE__CV_TARGET_FRESHNESS_HPP_
