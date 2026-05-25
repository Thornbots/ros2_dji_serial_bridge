#pragma once
// mcb_protocol.hpp
//
// Packed struct definitions that mirror the wire layout used by the MCB
// (JetsonSubsystem.hpp / type_c_serial_test.hpp).  All multi-byte fields
// are little-endian, matching the ARM Cortex-M running modm.
//
// DJI UART frame layout (all offsets in bytes):
//   [0]      0xA5         frame head
//   [1-2]    dataLength   payload byte count (uint16_t LE)
//   [3]      seq          rolling sequence counter
//   [4]      crc8         CRC-8 over bytes [0..3]
//   [5-6]    msgType      message ID (uint16_t LE)  ← enum McbMsgType
//   [7..N]   payload      dataLength raw bytes
//   [N+1,2]  crc16        CRC-16 over bytes [0..N]  (uint16_t LE)

#include <cstdint>
#include <cstddef>  // offsetof

// ─── framing constant ────────────────────────────────────────────────────────
static constexpr uint8_t FRAME_HEAD = 0xA5;

// ─── message IDs ─────────────────────────────────────────────────────────────
// Must stay in sync with enum UartMessage in JetsonSubsystem.hpp.
enum class McbMsgType : uint16_t {
    ROS_MSG    = 0,  // Jetson → MCB : navigation goal  (ROSDataPayload)
    CV_MSG     = 1,  // Jetson → MCB : CV target         (CVDataPayload)
    POSE_MSG   = 2,  // MCB → Jetson : robot pose        (PoseDataPayload)
    REF_SYS    = 3,  // MCB → Jetson : referee system    (RefSysMsgPayload)
    RELOCALIZE = 4,  // Jetson → MCB : lidar position    (RelocalizePayload)
};

// ─── DJI wire header  (exactly 7 bytes) ─────────────────────────────────────
struct __attribute__((packed)) FrameHeader {
    uint8_t  head;        // 0xA5
    uint16_t dataLength;  // payload byte count
    uint8_t  seq;
    uint8_t  crc8;        // CRC-8 of bytes [0 .. offsetof(crc8))
    uint16_t msgType;     // McbMsgType cast to uint16_t
};
static_assert(sizeof(FrameHeader) == 7, "FrameHeader must be exactly 7 bytes");

// Bytes covered by CRC-8: everything before the crc8 field (head + dataLength + seq = 4 bytes).
static constexpr size_t CRC8_COVERAGE = offsetof(FrameHeader, crc8);  // == 4

// ─── MCB → Jetson payloads ───────────────────────────────────────────────────

// POSE_MSG (id=2) — sent at 100 Hz by the MCB.
// Mirror of struct PoseData (modm_packed) in JetsonSubsystem.hpp.
struct __attribute__((packed)) PoseDataPayload {
    float x;           // chassis X  (odometry, metres)
    float y;           // chassis Y  (odometry, metres)
    float vel_x;       // chassis vX (m/s)
    float vel_y;       // chassis vY (m/s)
    float head_pitch;  // gimbal pitch encoder value (radians)
    float head_yaw;    // gimbal yaw relative to world (radians)
};
static_assert(sizeof(PoseDataPayload) == 24, "PoseDataPayload size mismatch");

// REF_SYS_MSG (id=3) — sent at ~5 Hz by the MCB, interleaved with POSE_MSG.
// Mirror of struct RefSysMsg (modm_packed) in JetsonSubsystem.hpp.
//
// booleans bit layout (packed by MCB, MSB first):
//   bit 7 : isOnBlueTeam
//   bit 6 : isHealing
//   bit 5 : isInReloadZone   (restoration OR exchange zone RFID)
//   bit 4 : isInCenterZone   (central buff RFID)
//   bit 3 : teamOccupiesCenter
//   bit 2 : opponentOccupiesCenter
//   bit 1 : chassisHasPower
//   bit 0 : gimbalHasPower
struct __attribute__((packed)) RefSysMsgPayload {
    uint8_t  gameStage;
    uint16_t stageTimeRemaining;
    uint16_t robotHp;
    uint8_t  robotID;              // normalised to red-team numbering (hero == 1)
    float    deltaAngleGotHitIn;   // radians from current heading
    uint8_t  booleans;
};
static_assert(sizeof(RefSysMsgPayload) == 11, "RefSysMsgPayload size mismatch");

// ─── Jetson → MCB payloads ───────────────────────────────────────────────────

// ROS_MSG (id=0) — navigation goal for the autonomous drive controller.
// Mirror of struct ROSData in JetsonSubsystem.hpp.
struct __attribute__((packed)) ROSDataPayload {
    float targetX;
    float targetY;
};
static_assert(sizeof(ROSDataPayload) == 8, "ROSDataPayload size mismatch");

// CV_MSG (id=1) — computer-vision target state in the camera frame.
// Mirror of struct CVData in JetsonSubsystem.hpp.
struct __attribute__((packed)) CVDataPayload {
    float x;          // position  (metres)
    float y;
    float z;
    float v_x;        // velocity  (m/s)
    float v_y;
    float v_z;
    float a_x;        // acceleration (m/s²)
    float a_y;
    float a_z;
    float confidence; // [0.0, 1.0]
};
static_assert(sizeof(CVDataPayload) == 40, "CVDataPayload size mismatch");

// RELOCALIZE (id=4) — lidar-estimated robot position sent back to the MCB
// so it can update its odometry origin.
// Mirror of struct Relocalize in JetsonSubsystem.hpp.
struct __attribute__((packed)) RelocalizePayload {
    float expectedX;
    float expectedY;
};
static_assert(sizeof(RelocalizePayload) == 8, "RelocalizePayload size mismatch");
