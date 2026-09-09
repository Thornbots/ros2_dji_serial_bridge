# MCB ↔ Jetson UART protocol

Reference for every message crossing the serial link. Jetson side:
`include/dji_serial_bridge/dji_protocol.hpp`. MCB side: `JetsonSubsystem.hpp`
in the firmware repo.

The MCB casts these bytes into packed structs, so any change here is also a
firmware change. Size mismatches fail the receiver's length check; a field
that changes meaning under a stable layout does not.

## Framing

All multi-byte fields little-endian. Structs are `__attribute__((packed))`;
no padding.

```
[0]      0xA5         frame head
[1-2]    dataLength   payload byte count (uint16_t LE)
[3]      seq          rolling sequence counter, wraps at 256
[4]      crc8         CRC-8 over bytes [0..3]
[5-6]    msgType      message ID (uint16_t LE)  ← enum McbMsgType
[7..N]   payload      dataLength raw bytes
[N+1,2]  crc16        CRC-16 over bytes [0..N]  (uint16_t LE)
```

Header 7 bytes, trailer 2, frame size `payload + 9`. Payload offsets in the
tables below are relative to the payload start; add 7 for the frame offset.

`seq` increments per transmitted frame and is not checked on receive. No
retransmission, acknowledgement or flow control in either direction.

## Message types

| ID | Direction     | Payload struct      | Bytes | ROS topic     | ROS type                           | Rate         |
|----|---------------|---------------------|-------|---------------|------------------------------------|--------------|
| 0  | Jetson → MCB  | `ROSDataPayload`    | 8     | `~/nav_goal`  | `geometry_msgs/msg/Point`          | on publish   |
| 1  | Jetson → MCB  | `CVDataPayload`     | 17    | `~/cv_target` | `dji_serial_bridge/msg/CVTarget`   | on publish   |
| 2  | MCB → Jetson  | `PoseDataPayload`   | 25    | `~/pose`      | `dji_serial_bridge/msg/RobotPose`  | 100 Hz       |
| 3  | MCB → Jetson  | `RefSysMsgPayload`  | 11    | `~/ref_sys`   | `dji_serial_bridge/msg/RefSysStatus` | ~5 Hz      |
| 4  | Jetson → MCB  | `RelocalizePayload` | 8     | `~/relocalize`| `geometry_msgs/msg/Point`          | on correction|

IDs match `enum UartMessage` in the firmware's `JetsonSubsystem.hpp`. Topics
are in the node's private namespace: `~/nav_goal` is
`/dji_serial_bridge/nav_goal` unless remapped.

An inbound frame with any other `msgType` is counted, logged at WARN, dropped.

---

## Jetson → MCB

Transmitted from the subscriber callback: one ROS message in, one frame out.
No timer, no retransmission; the wire rate is the publisher's rate. Fields are
copied without clamping or validation.

### ROS_MSG (id=0) — navigation goal

8-byte payload, 17-byte frame. Subscribed on `~/nav_goal`
(`geometry_msgs/msg/Point`, depth-10 reliable). `Point.z` is discarded.

| Off | Size | Type    | Field     | From      |
|-----|------|---------|-----------|-----------|
| 0   | 4    | float32 | `targetX` | `Point.x` |
| 4   | 4    | float32 | `targetY` | `Point.y` |

A field goal in the MCB's odometry frame, metres, consumed by its autonomous
drive controller. No publisher in this workspace: `mcb_relay` wires up
`~/cv_target` and `~/relocalize` only.

### CV_MSG (id=1) — aim point

17-byte payload, 26-byte frame. Subscribed on `~/cv_target`
(`dji_serial_bridge/msg/CVTarget`, SensorDataQoS, best-effort).

| Off | Size | Type    | Field        | From                  |
|-----|------|---------|--------------|-----------------------|
| 0   | 4    | float32 | `x`          | `CVTarget.x`          |
| 4   | 4    | float32 | `y`          | `CVTarget.y`          |
| 8   | 4    | float32 | `z`          | `CVTarget.z`          |
| 12  | 4    | float32 | `confidence` | `CVTarget.confidence` |
| 16  | 1    | uint8   | `flags`      | packed, below         |

`flags` bit0 = `lead_applied` (`x/y/z` already includes the intercept solve),
bit1 = `track_valid` (backed by a converged `target_tracker` estimate rather
than a raw panel position), bits 2-7 reserved, sent as 0.

`x/y/z` is a root-frame position in metres, REP-103 (x forward, y left, z up).
Type-C aims at it directly and applies its own gravity, drag and muzzle
geometry. It is not a camera-frame offset and not a barrel attitude. Velocity
and spin are not on the wire; they are ROS-internal on `/cv/target_state`
(`TargetState.msg`). `header` is not on the wire, so the MCB has no detection
timestamp and cannot age the point.

Layout history, both requiring matching firmware changes:

- 2026-07-28: `v_x/v_y/v_z` and `a_x/a_y/a_z` dropped, 40 → 16 bytes,
  `confidence` moved from offset 36 to 12.
- Later: `x/y/z` changed from a camera-frame offset to a root-frame position,
  and the `flags` byte took the struct to 17 bytes.

### RELOCALIZE (id=4) — lidar position fix

8-byte payload, 17-byte frame. Subscribed on `~/relocalize`
(`geometry_msgs/msg/Point`, depth-10 reliable). `Point.z` is discarded.

| Off | Size | Type    | Field       | From      |
|-----|------|---------|-------------|-----------|
| 0   | 4    | float32 | `expectedX` | `Point.x` |
| 4   | 4    | float32 | `expectedY` | `Point.y` |

The lidar-estimated position the MCB adopts as its odometry origin, same frame
and units as POSE_MSG's `x/y`. It overwrites MCB odometry. The node logs every
transmission at INFO with the last received `~/pose` beside the new coordinate.

---

## MCB → Jetson

A dedicated read thread polls the port, scans for `0xA5`, checks CRC-8 over the
header and CRC-16 over the frame (both dropped on mismatch while `enforce_crc`
is true), then checks payload length against the struct size before publishing.
`header.stamp` on both published messages is the Jetson's arrival time; no MCB
clock is on the wire.

### POSE_MSG (id=2) — chassis pose and gimbal angles

25-byte payload, 34-byte frame, 100 Hz. Published on `~/pose`
(`dji_serial_bridge/msg/RobotPose`, SensorDataQoS).

| Off | Size | Type    | Field        | Meaning                                  |
|-----|------|---------|--------------|------------------------------------------|
| 0   | 4    | float32 | `x`          | chassis X, odometry frame, metres        |
| 4   | 4    | float32 | `y`          | chassis Y, odometry frame, metres        |
| 8   | 4    | float32 | `vel_x`      | chassis X velocity, m/s                  |
| 12  | 4    | float32 | `vel_y`      | chassis Y velocity, m/s                  |
| 16  | 4    | float32 | `head_pitch` | gimbal pitch encoder value, radians      |
| 20  | 4    | float32 | `head_yaw`   | gimbal yaw relative to world, radians    |
| 24  | 1    | uint8   | `odomStatus` | odometry health, table below             |

| Code | `RobotPose` constant | Meaning                                      |
|------|----------------------|----------------------------------------------|
| 0    | `ODOM_OK`            | dead reckoning is trustworthy                |
| 1    | `ODOM_ENCODER_FAULT` | wheel encoder fault or implausible reading   |
| 2    | `ODOM_IMU_FAULT`     | IMU fault, saturation, or failed calibration |
| 3    | `ODOM_SLIP`          | wheels turning without matching motion       |
| 4    | `ODOM_UNKNOWN`       | MCB knows odom is bad, not why               |

Non-zero means `x`, `y`, `vel_x` and `vel_y` are unusable; `head_pitch` and
`head_yaw` are gimbal encoder values and remain valid. The node copies the byte
through without validating it against the table. Transitions are logged: WARN
into a fault, INFO on recovery.

`odomStatus` was added 2026-09-09, taking the payload from 24 to 25 bytes.
Until the firmware's `PoseData` matches, every pose frame fails the length
check and `~/pose` publishes nothing.

### REF_SYS_MSG (id=3) — referee system status

11-byte payload, 20-byte frame, ~5 Hz, interleaved with POSE_MSG. Published on
`~/ref_sys` (`dji_serial_bridge/msg/RefSysStatus`, SensorDataQoS).

| Off | Size | Type    | Field                | Meaning                                   |
|-----|------|---------|----------------------|-------------------------------------------|
| 0   | 1    | uint8   | `gameStage`          | referee game stage enum value             |
| 1   | 2    | uint16  | `stageTimeRemaining` | seconds left in the current stage         |
| 3   | 2    | uint16  | `robotHp`            | current robot HP                          |
| 5   | 1    | uint8   | `robotID`            | normalised to red-team numbering (hero=1) |
| 6   | 4    | float32 | `deltaAngleGotHitIn` | radians from current heading of last hit  |
| 10  | 1    | uint8   | `booleans`           | 8 flags, MSB first, table below           |

| Bit | Firmware name            | `RefSysStatus` field       | Meaning                           |
|-----|--------------------------|----------------------------|-----------------------------------|
| 7   | `isOnBlueTeam`           | `is_on_blue_team`          | blue team, else red               |
| 6   | `isHealing`              | `is_healing`               | HP currently restoring            |
| 5   | `isInReloadZone`         | `is_in_reload_zone`        | restoration or exchange zone RFID |
| 4   | `isInCenterZone`         | `is_in_center_zone`        | central buff RFID                 |
| 3   | `teamOccupiesCenter`     | `team_occupies_center`     | our team holds the central buff   |
| 2   | `opponentOccupiesCenter` | `opponent_occupies_center` | opponent holds it                 |
| 1   | `chassisHasPower`        | `chassis_has_power`        | chassis power output on           |
| 0   | `gimbalHasPower`         | `gimbal_has_power`         | gimbal power output on            |

The node unpacks the byte into the eight named booleans.

---

## Not on the wire

`PanelDetection`, `PanelDetectionArray` and `TargetState` are ROS-internal,
travelling between `thornbots_pkg` and the CV pipeline.

`FireCommand` has no message ID and no subscriber in this package.
`mcb_relay` republishes `/sentry/fire_command` onto
`/dji_serial_bridge/fire_command`, where nothing reads it. Putting it on the
wire needs an ID, a payload struct and the matching firmware side.
