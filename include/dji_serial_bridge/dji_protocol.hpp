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
// file's names on its uart-names-from-ros-topics branch, Thornbots/MCBV3#74).
enum class McbMsgType : uint16_t
{
  NAV_GOAL   = 0,    // Jetson → MCB : ~/nav_goal    (NavGoalPayload)
  CV_TARGET  = 1,    // Jetson → MCB : ~/cv_target   (CvTargetPayload)
  POSE       = 2,    // MCB → Jetson : ~/pose        (PosePayload)
  REF_SYS    = 3,    // MCB → Jetson : ~/ref_sys     (RefSysPayload)
  RELOCALIZE = 4,    // Jetson → MCB : ~/relocalize  (RelocalizePayload)
  BYTE       = 5,    // both ways    : ~/byte_to_mcb, ~/byte_from_mcb  (BytePayload)
};

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

// POSE (id=2) — sent at 100 Hz by the MCB, published on ~/pose (RobotPose).
// Mirror of struct Pose (modm_packed) in JetsonSubsystem.hpp.
// Trailing odom_status byte rides along with every pose rather than arriving
// as its own message, so the verdict can never be newer or older than the
// x/y it applies to. see UART_PROTOCOL.md for the status code table
struct __attribute__((packed)) PosePayload
{
  float   x;             // chassis X  (odometry, metres)
  float   y;             // chassis Y  (odometry, metres)
  float   vel_x;         // chassis vX (m/s)
  float   vel_y;         // chassis vY (m/s)
  float   head_pitch;    // gimbal pitch encoder value (radians)
  float   head_yaw;      // gimbal yaw relative to world (radians)
  uint8_t odom_status;   // odometry source: 0 pods, 1 drivetrain, 2 i2c dead
  // (no data), 3 i2c dead using drivetrain
};
static_assert(sizeof(PosePayload) == 25, "PosePayload size mismatch");

// REF_SYS (id=3) — sent at ~5 Hz by the MCB, interleaved with POSE, published
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

// CV_TARGET (id=1) — from ~/cv_target (CVTarget): aim point and fire decision
// in one frame, so the delay can never pair with an aim point it was not
// solved for.
// x/y/z is a WORLD-FRAME POSITION in odom, POSE's frame (not a root- or
// camera-frame offset, not a barrel attitude -- Type-C applies its own
// ballistics on top).
// No stamp: the MCB runs delay_ms from frame receipt.
// Mirror of struct CvTarget in the firmware's JetsonSubsystem.hpp.
struct __attribute__((packed)) CvTargetPayload
{
  float    x;            // position, odom x (metres)
  float    y;            // position, odom y (metres)
  float    z;            // position, odom z, up (metres)
  uint16_t delay_ms;     // fire this many ms after frame receipt (0 = immediate)
  uint8_t  flags;        // CVTarget booleans: bit0 fire, bit1 type_c_based_patrol,
                         // bit2 turn_to_hit, bits 3-7 reserved (0)
};
static_assert(sizeof(CvTargetPayload) == 15, "CvTargetPayload size mismatch");

// RELOCALIZE (id=4) — lidar-estimated robot position from ~/relocalize
// (PointStamped), sent back to the MCB so it can update its odometry origin.
// Mirror of struct Relocalize in JetsonSubsystem.hpp.
struct __attribute__((packed)) RelocalizePayload
{
  float x;
  float y;
};
static_assert(sizeof(RelocalizePayload) == 8, "RelocalizePayload size mismatch");

// ─── Either direction ────────────────────────────────────────────────────────

// BYTE (id=5) — one raw byte, Jetson → MCB from ~/byte_to_mcb and
// MCB → Jetson on ~/byte_from_mcb (both McbByte). Meaning is up to the
// sender and receiver. Mirror of struct McbByte in JetsonSubsystem.hpp.
struct __attribute__((packed)) BytePayload
{
  uint8_t data;
};
static_assert(sizeof(BytePayload) == 1, "BytePayload size mismatch");
