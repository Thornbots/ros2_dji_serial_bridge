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

// dji_protocol.hpp
//
// Packed struct definitions that mirror the wire layout used by the MCB
// (JetsonSubsystem.hpp). All multi-byte fields
// are little-endian, matching the ARM Cortex-M running modm.
// see UART_PROTOCOL.md for the frame diagram and every payload table

#include <cstdint>
#include <cstddef>  // offsetof

// ─── framing constant ────────────────────────────────────────────────────────
static constexpr uint8_t FRAME_HEAD = 0xA5;

// ─── message IDs ─────────────────────────────────────────────────────────────
// Named after the ROS topic each one carries. IDs must stay in sync with
// enum UartMessage in JetsonSubsystem.hpp (old names at MCBV3 708b8d6; this
// file's names on its position-based-cv branch, Thornbots/MCBV3#74). Id 5 is
// Ping there, which echoes it.
enum class McbMsgType : uint16_t
{
  NAV_GOAL   = 0,    // Jetson → MCB : ~/nav_goal    (NavGoalPayload)
  CV_TARGET  = 1,    // Jetson → MCB : ~/cv_target   (CvTargetPayload)
  POSE       = 2,    // MCB → Jetson : ~/pose        (PosePayload)
  REF_SYS    = 3,    // MCB → Jetson : ~/ref_sys     (RefSysPayload)
  RELOCALIZE = 4,    // Jetson → MCB : ~/relocalize  (RelocalizePayload)
  DEBUG      = 6,    // MCB → Jetson : ~/debug (DebugBlob), opaque bytes
  PING       = 5,    // both ways    : ~/ping_to_mcb, ~/ping_from_mcb  (PingPayload)
  TARGET_STATE = 7,  // Jetson → MCB : ~/target_state (TargetStatePayload)
};

static constexpr size_t DEBUG_MAX_PAYLOAD = 128;

// ─── DJI wire header  (exactly 7 bytes) ─────────────────────────────────────
struct __attribute__((packed)) FrameHeader
{
  uint8_t  head;          // 0xA5
  uint16_t dataLength;    // payload byte count
  uint8_t  seq;
  uint8_t  crc8;          // CRC-8 of bytes [0 .. offsetof(crc8))
  uint16_t msgType;       // McbMsgType cast to uint16_t
};
static_assert(sizeof(FrameHeader) == 7, "FrameHeader must be exactly 7 bytes");

// Bytes covered by CRC-8: everything before the crc8 field (head + dataLength + seq = 4 bytes).
static constexpr size_t CRC8_COVERAGE = offsetof(FrameHeader, crc8);  // == 4

// ─── MCB → Jetson payloads ───────────────────────────────────────────────────

// POSE (id=2) — sent at 90 Hz by the MCB, published on ~/pose (RobotPose).
// Mirror of struct Pose (modm_packed) in JetsonSubsystem.hpp.
// Trailing odom_status byte rides along with every pose rather than arriving
// as its own message, so the verdict can never be newer or older than the
// x/y it applies to. see UART_PROTOCOL.md for the status code table
struct __attribute__((packed)) PosePayload
{
  float   x;             // chassis X  (raw aim odom, metres)
  float   y;             // chassis Y  (raw aim odom, metres)
  float   vel_x;         // chassis vX (m/s)
  float   vel_y;         // chassis vY (m/s)
  float   head_pitch;    // gimbal pitch encoder value (radians)
  float   head_yaw;      // gimbal yaw, field frame (radians)
  uint8_t odom_status;   // odometry source: 0 pods, 1 drivetrain, 2 i2c dead
  // (no data), 3 i2c dead using drivetrain
};
static_assert(sizeof(PosePayload) == 25, "PosePayload size mismatch");

// REF_SYS (id=3) — sent at 10 Hz by the MCB, interleaved with POSE, published
// on ~/ref_sys (RefSysStatus).
// Mirror of struct RefSys (modm_packed) in JetsonSubsystem.hpp.
// Booleans byte bit layout (MSB first): see UART_PROTOCOL.md for the full
// bit-to-flag table (team/health/zone RFIDs/power flags).
struct __attribute__((packed)) RefSysPayload
{
  uint8_t  game_stage;
  uint16_t stage_time_remaining;
  uint16_t robot_hp;
  uint8_t  robot_id;                // normalised to red-team numbering (hero == 1)
  float    delta_angle_got_hit_in;  // radians from current heading
  uint8_t  booleans;
};
static_assert(sizeof(RefSysPayload) == 11, "RefSysPayload size mismatch");

// ─── Jetson → MCB payloads ───────────────────────────────────────────────────

// NAV_GOAL (id=0) — navigation goal for the autonomous drive controller,
// from ~/nav_goal (PointStamped). Mirror of struct NavGoal in JetsonSubsystem.hpp.
struct __attribute__((packed)) NavGoalPayload
{
  float x;
  float y;
};
static_assert(sizeof(NavGoalPayload) == 8, "NavGoalPayload size mismatch");

// CV_TARGET (id=1) — from ~/cv_target (CVTarget): idle look point only.
// x/y/z is a WORLD-FRAME POSITION in raw aim odom (not a root- or
// camera-frame offset, not a barrel attitude). Field frame: UART_PROTOCOL.md.
// Aiming and firing ride TARGET_STATE. No stamp crosses the wire.
// Mirror of struct CvTarget in the firmware's JetsonSubsystem.hpp.
struct __attribute__((packed)) CvTargetPayload
{
  float    x;            // position, odom x (metres)
  float    y;            // position, odom y (metres)
  float    z;            // position, odom z, up (metres)
  uint8_t  flags;        // bit1 type_c_based_patrol, bit2 turn_to_hit,
                         // bits 0 and 3-7 reserved (0)
};
static_assert(sizeof(CvTargetPayload) == 13, "CvTargetPayload size mismatch");
static_assert(offsetof(CvTargetPayload, flags) == 12, "CvTargetPayload flags offset");

// TARGET_STATE (id=7) — from ~/target_state (TargetState): the tracker's
// armor model. The MCB plans from it. Ages are measured at the final write
// boundary, so no clock is shared. Table: UART_PROTOCOL.md.
struct __attribute__((packed)) TargetStatePayload
{
  float    center[3];         // m, raw aim odom
  float    velocity[3];       // m/s
  float    acceleration[3];   // m/s^2
  float    yaw;               // rad, wrapped to [-pi, pi)
  float    yaw_rate;          // rad/s, + CCW
  float    radius[2];         // m
  float    z_offset[2];       // m
  float    panel[3];          // m
  uint16_t state_age_ms;      // write time - header.stamp
  uint16_t detection_age_ms;  // write time - detection_stamp
  uint16_t track_id;          // low 16 bits of robot_track_id
  uint8_t  confidence;        // round(confidence * 255), clamped
  uint8_t  flags;             // bit0 valid, bit1 type_c_based_patrol, 2-7 reserved (0)
};
static_assert(sizeof(TargetStatePayload) == 80, "TargetStatePayload size mismatch");
static_assert(offsetof(TargetStatePayload, yaw) == 36, "TargetStatePayload yaw offset");
static_assert(offsetof(TargetStatePayload, yaw_rate) == 40, "TargetStatePayload yaw_rate offset");
static_assert(offsetof(TargetStatePayload, radius) == 44, "TargetStatePayload radius offset");
static_assert(offsetof(TargetStatePayload, z_offset) == 52, "TargetStatePayload z_offset offset");
static_assert(offsetof(TargetStatePayload, panel) == 60, "TargetStatePayload panel offset");
static_assert(offsetof(TargetStatePayload, state_age_ms) == 72,
  "TargetStatePayload state_age offset");
static_assert(offsetof(TargetStatePayload, detection_age_ms) == 74,
  "TargetStatePayload detection_age offset");
static_assert(offsetof(TargetStatePayload, track_id) == 76, "TargetStatePayload track_id offset");
static_assert(offsetof(TargetStatePayload, confidence) == 78,
  "TargetStatePayload confidence offset");
static_assert(offsetof(TargetStatePayload, flags) == 79, "TargetStatePayload flags offset");

// RELOCALIZE (id=4) — lidar-estimated robot position from ~/relocalize
// (PointStamped), updates only the MCB navigation offset, never raw aim odom.
// Mirror of struct Relocalize in JetsonSubsystem.hpp.
struct __attribute__((packed)) RelocalizePayload
{
  float x;
  float y;
};
static_assert(sizeof(RelocalizePayload) == 8, "RelocalizePayload size mismatch");

// ─── Either direction ────────────────────────────────────────────────────────

// PING (id=5) — Jetson → MCB from ~/ping_to_mcb; the MCB echoes it back,
// published on ~/ping_from_mcb (both Ping). Firmware: struct Ping.
struct __attribute__((packed)) PingPayload
{
  uint8_t number;
};
static_assert(sizeof(PingPayload) == 1, "PingPayload size mismatch");
