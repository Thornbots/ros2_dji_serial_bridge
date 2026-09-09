# MCB ↔ Jetson UART protocol

Every message that crosses the serial link, both directions, in one place.
The C++ side of this is `include/dji_serial_bridge/dji_protocol.hpp`; the MCB
side is `struct` definitions in its `JetsonSubsystem.hpp`, which lives in the
firmware repo, not here.

**Changing anything on this page is a two-repo change.** The MCB parses these
bytes by casting into a packed struct, so a layout the firmware disagrees with
either fails the node's length check (topic goes silent) or is read as
garbage. The size mismatches are caught; a field whose *meaning* changed under
a stable layout is not, and CV_MSG has been through exactly that.

## Framing

All multi-byte fields little-endian, matching the ARM Cortex-M running modm.
Structs are `__attribute__((packed))`, so there is no padding anywhere.

```
[0]      0xA5         frame head
[1-2]    dataLength   payload byte count (uint16_t LE)
[3]      seq          rolling sequence counter, wraps at 256
[4]      crc8         CRC-8 over bytes [0..3]
[5-6]    msgType      message ID (uint16_t LE)  ← enum McbMsgType
[7..N]   payload      dataLength raw bytes
[N+1,2]  crc16        CRC-16 over bytes [0..N]  (uint16_t LE)
```

Header is 7 bytes, trailer 2, so a frame is `payload + 9`. Payload offsets in
the tables below are relative to the payload start; add 7 for the offset
within the frame.

`seq` increments per transmitted frame and is not checked on receive: there is
no retransmission, no acknowledgement and no flow control in either direction.
A frame that arrives corrupt is dropped and gone.

## Message types

| ID | Direction     | Payload struct      | Bytes | ROS topic     | ROS type                           | Rate         |
|----|---------------|---------------------|-------|---------------|------------------------------------|--------------|
| 0  | Jetson → MCB  | `ROSDataPayload`    | 8     | `~/nav_goal`  | `geometry_msgs/msg/Point`          | on publish   |
| 1  | Jetson → MCB  | `CVDataPayload`     | 17    | `~/cv_target` | `dji_serial_bridge/msg/CVTarget`   | on publish   |
| 2  | MCB → Jetson  | `PoseDataPayload`   | 25    | `~/pose`      | `dji_serial_bridge/msg/RobotPose`  | 100 Hz       |
| 3  | MCB → Jetson  | `RefSysMsgPayload`  | 11    | `~/ref_sys`   | `dji_serial_bridge/msg/RefSysStatus` | ~5 Hz      |
| 4  | Jetson → MCB  | `RelocalizePayload` | 8     | `~/relocalize`| `geometry_msgs/msg/Point`          | on correction|

Topics live in the node's private namespace, so `~/nav_goal` is
`/dji_serial_bridge/nav_goal` unless a launch file remaps it. The IDs must
match `enum UartMessage` in the firmware's `JetsonSubsystem.hpp`.

An inbound frame with any other `msgType` is counted, logged at WARN and
dropped.

---

## Jetson → MCB

The node transmits only from a subscriber callback: one ROS message in, one
frame out, no timer and no retransmission, so the wire rate is whatever
`thornbots_pkg`'s `mcb_relay` publishes at. Nothing is clamped or validated on
the way out, so a NaN in the ROS message reaches the MCB as a NaN.

### ROS_MSG (id=0) — navigation goal

8-byte payload, 17-byte frame. Subscribed on `~/nav_goal`
(`geometry_msgs/msg/Point`, depth-10 reliable). `Point.z` is discarded.

| Off | Size | Type    | Field     | From      |
|-----|------|---------|-----------|-----------|
| 0   | 4    | float32 | `targetX` | `Point.x` |
| 4   | 4    | float32 | `targetY` | `Point.y` |

A field goal in the MCB's odometry frame, metres, consumed by its autonomous
drive controller. Nothing in this workspace publishes it yet: `mcb_relay`
wires up `~/cv_target` and `~/relocalize` only, so the path is implemented but
dormant.

### CV_MSG (id=1) — aim point

17-byte payload, 26-byte frame. Subscribed on `~/cv_target`
(`dji_serial_bridge/msg/CVTarget`, SensorDataQoS, so best-effort: a dropped
frame at CV rates is expected and fine).

| Off | Size | Type    | Field        | From                  |
|-----|------|---------|--------------|-----------------------|
| 0   | 4    | float32 | `x`          | `CVTarget.x`          |
| 4   | 4    | float32 | `y`          | `CVTarget.y`          |
| 8   | 4    | float32 | `z`          | `CVTarget.z`          |
| 12  | 4    | float32 | `confidence` | `CVTarget.confidence` |
| 16  | 1    | uint8   | `flags`      | packed, below         |

`flags` bit0 = `lead_applied` (does `x/y/z` already include the intercept
solve), bit1 = `track_valid` (is it backed by a converged `target_tracker`
estimate rather than a raw unfiltered panel position), bits 2-7 reserved and
sent as 0.

`x/y/z` is a **root-frame position** in metres, REP-103 (x forward, y left,
z up), which Type-C aims at directly by applying its own gravity, drag and
muzzle geometry. It is not a camera-frame offset and not a barrel attitude.
Velocity and spin stay off the wire by design and live ROS-internal on
`/cv/target_state` (`TargetState.msg`). `header` does not cross either, so the
MCB has no detection timestamp and cannot age the point itself; staleness is
the sender's problem.

Two changes this struct has already been through, both needing the firmware
struct updated to match:

- 2026-07-28: `v_x/v_y/v_z` and `a_x/a_y/a_z` dropped, 40 → 16 bytes, moving
  `confidence` from offset 36 to 12.
- Later: `x/y/z` changed meaning from a camera-frame offset to a root-frame
  position, and the `flags` byte took the struct to 17. A firmware that parses
  the new layout correctly still aims wrong if it treats `x/y/z` as
  camera-relative.

### RELOCALIZE (id=4) — lidar position fix

8-byte payload, 17-byte frame. Subscribed on `~/relocalize`
(`geometry_msgs/msg/Point`, depth-10 reliable). `Point.z` is discarded.

| Off | Size | Type    | Field       | From      |
|-----|------|---------|-------------|-----------|
| 0   | 4    | float32 | `expectedX` | `Point.x` |
| 4   | 4    | float32 | `expectedY` | `Point.y` |

The lidar-estimated position the MCB should adopt as its odometry origin, same
frame and units as POSE_MSG's `x/y`. Each one is destructive to MCB odometry,
so the node logs every transmission at INFO with the last `~/pose` it received
beside the new coordinate.

---

## MCB → Jetson

A dedicated read thread polls the port, scans for `0xA5`, checks CRC-8 over
the header and CRC-16 over the whole frame (both dropping the frame on
mismatch while `enforce_crc` is true), then checks the payload length against
the struct size before publishing. `header.stamp` on both published messages
is the Jetson's arrival time, not an MCB timestamp, and no MCB clock crosses
the link.

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

`odomStatus` rides along with every pose rather than arriving as its own
message, so the verdict can never be newer or older than the `x/y` it applies
to.

| Code | `RobotPose` constant | Meaning                                      |
|------|----------------------|----------------------------------------------|
| 0    | `ODOM_OK`            | dead reckoning is trustworthy                |
| 1    | `ODOM_ENCODER_FAULT` | wheel encoder fault or implausible reading   |
| 2    | `ODOM_IMU_FAULT`     | IMU fault, saturation, or failed calibration |
| 3    | `ODOM_SLIP`          | wheels turning without matching motion       |
| 4    | `ODOM_UNKNOWN`       | MCB knows odom is bad, not why               |

Non-zero means `x`, `y`, `vel_x` and `vel_y` are unusable. `head_pitch` and
`head_yaw` come off the gimbal encoders and stay good either way. The node
copies the byte through unvalidated, so a code the firmware invents later
reaches consumers instead of being swallowed. At 100 Hz it logs transitions
only: WARN on the edge into a fault, INFO on recovery.

The byte was added 2026-09-09, taking the payload from 24 to 25 bytes. Until
the firmware's `PoseData` matches, every pose frame fails the length check and
`~/pose` goes silent, which at least fails loudly.

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

| Bit | Firmware name            | `RefSysStatus` field        | Meaning                            |
|-----|--------------------------|-----------------------------|------------------------------------|
| 7   | `isOnBlueTeam`           | `is_on_blue_team`           | blue team, else red                |
| 6   | `isHealing`              | `is_healing`                | HP currently restoring             |
| 5   | `isInReloadZone`         | `is_in_reload_zone`         | restoration or exchange zone RFID  |
| 4   | `isInCenterZone`         | `is_in_center_zone`         | central buff RFID                  |
| 3   | `teamOccupiesCenter`     | `team_occupies_center`      | our team holds the central buff    |
| 2   | `opponentOccupiesCenter` | `opponent_occupies_center`  | opponent holds it                  |
| 1   | `chassisHasPower`        | `chassis_has_power`         | chassis power output on            |
| 0   | `gimbalHasPower`         | `gimbal_has_power`          | gimbal power output on             |

The node unpacks the byte into the eight named booleans; nothing downstream
should be shifting bits.

---

## Not on the wire

`PanelDetection`, `PanelDetectionArray` and `TargetState` are built by this
package but stay ROS-internal, travelling between `thornbots_pkg` and the CV
pipeline. `TargetState` is the rich counterpart to the deliberately lean
`CVTarget`: velocity, spin and covariance stay on that topic rather than
burning UART bandwidth.

`FireCommand` is a loose end rather than a decision. `mcb_relay` republishes
`/sentry/fire_command` onto `/dji_serial_bridge/fire_command`, but this node
has no subscriber on that topic and no message ID for it, so the firing
decision reaches a topic nobody reads and stops there. Making it real needs an
ID, a payload struct, and the firmware side to match.
