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
#ifndef DJI_SERIAL_BRIDGE__TARGET_STATE_WIRE_HPP_
#define DJI_SERIAL_BRIDGE__TARGET_STATE_WIRE_HPP_

#include <cmath>
#include <cstdint>

#include "dji_serial_bridge/dji_protocol.hpp"

namespace dji_bridge
{
// Wrap to [-pi, pi) as a float: a double just below pi must not round up to
// float pi, which is outside the contract's interval.
inline float wrap_yaw(double yaw)
{
  constexpr double kPi = 3.14159265358979323846;
  if (!std::isfinite(yaw)) {
    return static_cast<float>(yaw);
  }
  double w = std::fmod(yaw + kPi, 2.0 * kPi);
  if (w < 0.0) {
    w += 2.0 * kPi;
  }
  const float f = static_cast<float>(w - kPi);
  return f >= static_cast<float>(kPi) ? -static_cast<float>(kPi) : f;
}

// round(clamp(c, 0, 1) * 255); NaN maps to 0.
inline uint8_t confidence_to_u8(double c)
{
  if (!(c > 0.0)) {
    return 0;
  }
  if (c >= 1.0) {
    return 255;
  }
  return static_cast<uint8_t>(std::lround(c * 255.0));
}

// Field copy of a dji_serial_bridge/msg/TargetState; both ages stay 0 until
// the final write boundary sets them from one clock read.
template<typename Msg>
inline TargetStatePayload pack_target_state(const Msg & m)
{
  TargetStatePayload p{};
  p.center[0] = static_cast<float>(m.center.x);
  p.center[1] = static_cast<float>(m.center.y);
  p.center[2] = static_cast<float>(m.center.z);
  p.velocity[0] = static_cast<float>(m.velocity.x);
  p.velocity[1] = static_cast<float>(m.velocity.y);
  p.velocity[2] = static_cast<float>(m.velocity.z);
  p.acceleration[0] = static_cast<float>(m.acceleration.x);
  p.acceleration[1] = static_cast<float>(m.acceleration.y);
  p.acceleration[2] = static_cast<float>(m.acceleration.z);
  p.yaw = wrap_yaw(m.yaw);
  p.yaw_rate = m.yaw_rate;
  for (int i = 0; i < 2; ++i) {
    p.radius[i] = m.radius[i];
    p.z_offset[i] = m.z_offset[i];
  }
  p.panel[0] = static_cast<float>(m.panel.x);
  p.panel[1] = static_cast<float>(m.panel.y);
  p.panel[2] = static_cast<float>(m.panel.z);
  p.track_id = static_cast<uint16_t>(m.robot_track_id & 0xFFFFu);
  p.confidence = confidence_to_u8(m.confidence);
  p.flags = static_cast<uint8_t>((m.valid ? 0x01 : 0x00) | (m.type_c_based_patrol ? 0x02 : 0x00));
  return p;
}
}  // namespace dji_bridge

#endif  // DJI_SERIAL_BRIDGE__TARGET_STATE_WIRE_HPP_
