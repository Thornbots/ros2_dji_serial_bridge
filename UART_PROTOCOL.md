# MCB ↔ Jetson UART protocol

Reference for every message crossing the serial link. Jetson side:
`include/dji_serial_bridge/dji_protocol.hpp`. MCB side: `JetsonSubsystem.hpp`
in the firmware repo.

Changing anything here is also a firmware change; see README.md for the
coordination notes and the currently pending ones.

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

| ID | Name         | Direction     | Payload struct      | Bytes | ROS topic      | ROS type                             | Rate          |
|----|--------------|---------------|---------------------|-------|----------------|--------------------------------------|---------------|
| 0  | `NAV_GOAL`   | Jetson → MCB  | `NavGoalPayload`    | 8     | `~/nav_goal`   | `geometry_msgs/msg/PointStamped`     | on publish    |
| 1  | `CV_TARGET`  | Jetson → MCB  | `CvTargetPayload`   | 15    | `~/cv_target`  | `dji_serial_bridge/msg/CVTarget`     | on publish    |
| 2  | `POSE`       | MCB → Jetson  | `PosePayload`       | 25    | `~/pose`       | `dji_serial_bridge/msg/RobotPose`    | 100 Hz        |
| 3  | `REF_SYS`    | MCB → Jetson  | `RefSysPayload`     | 11    | `~/ref_sys`    | `dji_serial_bridge/msg/RefSysStatus` | ~5 Hz         |
| 4  | `RELOCALIZE` | Jetson → MCB  | `RelocalizePayload` | 8     | `~/relocalize` | `geometry_msgs/msg/PointStamped`     | on correction |
| 5  | `BYTE`       | both          | `BytePayload`       | 1     | `~/byte_to_mcb`, `~/byte_from_mcb` | `dji_serial_bridge/msg/McbByte` | on publish / on send |

Each message is named after its topic, and its payload fields after the ROS
fields they carry. IDs 0-4 match `enum UartMessage` in the firmware's
`JetsonSubsystem.hpp` on branch `uart-names-from-ros-topics`
(Thornbots/MCBV3#74); MCBV3 `708b8d6` used the old names (`ROS_MSG`,
`CV_MSG`, `POSE_MSG`, `REF_SYS_MSG`). Topics are in the node's private
namespace: `~/nav_goal` is `/dji_serial_bridge/nav_goal` unless remapped.

An inbound frame with any other `msgType` is counted, logged at WARN, dropped.

---

## Jetson → MCB

Transmitted from the subscriber callback: one ROS message in, one frame out.
No timer, no retransmission; the wire rate is the publisher's rate. Fields are
copied without clamping or validation.

### NAV_GOAL (id=0) — navigation goal

8-byte payload, 17-byte frame. Subscribed on `~/nav_goal`
(`geometry_msgs/msg/PointStamped`, depth-10 reliable). `point.z` and the
header are discarded.

| Off | Size | Type    | Field     | From      |
|-----|------|---------|-----------|-----------|
| 0   | 4    | float32 | `x`       | `point.x` |
| 4   | 4    | float32 | `y`       | `point.y` |

A field goal in the MCB's odometry frame, metres, consumed by its autonomous
drive controller. No publisher in this workspace: `mcb_relay` wires up
`~/cv_target` and `~/relocalize` only.

### CV_TARGET (id=1) — aim point and fire decision

15-byte payload, 24-byte frame. Subscribed on `~/cv_target`
(`dji_serial_bridge/msg/CVTarget`, SensorDataQoS, best-effort).

| Off | Size | Type    | Field        | From                          |
|-----|------|---------|--------------|-------------------------------|
| 0   | 4    | float32 | `x`          | `CVTarget.x`                  |
| 4   | 4    | float32 | `y`          | `CVTarget.y`                  |
| 8   | 4    | float32 | `z`          | `CVTarget.z`                  |
| 12  | 2    | uint16  | `delay_ms`   | `CVTarget.delay_ms`           |
| 14  | 1    | uint8   | `flags`      | packed booleans, below        |

| Bit | `CVTarget` field      | Set means                                        |
|-----|-----------------------|--------------------------------------------------|
| 0   | `fire`                | fire this aim point `delay_ms` after receipt     |
| 1   | `type_c_based_patrol` | the MCB may patrol on its own; clear stops it    |
| 2   | `turn_to_hit`         | the MCB may turn toward where it got hit         |

Bits 3-7 are reserved, sent as 0. The bridge packs the byte from
`CVTarget`'s booleans, as it unpacks REF_SYS's. Bits 1 and 2 reach
the MCB with every frame.
Every frame is an aim point: the Jetson decides aim and fire itself, and
the MCB moves the gimbal only on these frames. With no target the Jetson
patrols, sending points to sweep the gun with `fire` clear
(`thornbots_pkg` `patrol_enabled`, on by default). With that off it sends
nothing between targets.

No stamp crosses the wire (dropped 2026-10-03 to match MCBV3
`position-based-cv`'s 15-byte `CvTarget`): the MCB runs `delay_ms` from frame receipt.

Aim and fire travel as one frame on purpose. A fire delay is only meaningful
against the aim point it was solved for; two frames could arrive apart, drop
independently, or pair up wrongly on the MCB and fire at a point the delay
was never computed for. `delay_ms` = 0 with `fire` set means fire now;
`fire` clear means `delay_ms` is meaningless.

`x/y/z` is a world-frame position in metres: `odom` (REP-105, z up), the
frame POSE's `x/y` are in. The MCB holds the point with its IMU and
odometry while the chassis moves and turns, then aims at it and applies its
own gravity, drag and muzzle geometry. It is not a root- or camera-frame
offset and not a barrel attitude. Velocity
and spin are not on the wire; they are ROS-internal on `/cv/target_state`
(`TargetState.msg`).

### RELOCALIZE (id=4) — lidar position fix

8-byte payload, 17-byte frame. Subscribed on `~/relocalize`
(`geometry_msgs/msg/PointStamped`, depth-10 reliable). `point.z` and the
header are discarded.

| Off | Size | Type    | Field       | From      |
|-----|------|---------|-------------|-----------|
| 0   | 4    | float32 | `x`         | `point.x` |
| 4   | 4    | float32 | `y`         | `point.y` |

The lidar-estimated position the MCB adopts as its odometry origin, same frame
and units as POSE's `x/y`. It overwrites MCB odometry. The node logs every
transmission at INFO with the last received `~/pose` beside the new coordinate.

---

## MCB → Jetson

A dedicated read thread polls the port, scans for `0xA5`, checks CRC-8 over the
header and CRC-16 over the frame (both dropped on mismatch while `enforce_crc`
is true), then checks payload length against the struct size before publishing.
`header.stamp` on both published messages is when the MCB started sending the
frame: the read time of its last byte less the frame's wire time at `baudrate`
(10 bits per byte). No MCB clock is on the wire, and the USB-serial latency
before a read is not taken off.

### POSE (id=2) — chassis pose and gimbal angles

25-byte payload, 34-byte frame, 100 Hz. Published on `~/pose`
(`dji_serial_bridge/msg/RobotPose`, SensorDataQoS).

| Off | Size | Type    | Field         | Meaning                                  |
|-----|------|---------|---------------|------------------------------------------|
| 0   | 4    | float32 | `x`           | chassis X, odometry frame, metres        |
| 4   | 4    | float32 | `y`          | chassis Y, odometry frame, metres        |
| 8   | 4    | float32 | `vel_x`      | chassis X velocity, m/s                  |
| 12  | 4    | float32 | `vel_y`      | chassis Y velocity, m/s                  |
| 16  | 4    | float32 | `head_pitch` | gimbal pitch encoder value, radians      |
| 20  | 4    | float32 | `head_yaw`   | gimbal yaw relative to world, radians    |
| 24  | 1    | uint8   | `odom_status`| which source produced x/y/vel, below     |

| Code | `RobotPose` constant         | Source of `x/y/vel_x/vel_y`                  |
|------|------------------------------|----------------------------------------------|
| 0    | `ODOM_PODS`                  | odometry pods, healthy                       |
| 1    | `ODOM_DRIVETRAIN`            | drivetrain odometry, degraded by wheel slip  |
| 2    | `ODOM_I2C_DEAD`              | none: I2C bus dead, no pod data, no fallback |
| 3    | `ODOM_I2C_DEAD_DRIVETRAIN`   | drivetrain odometry, pods lost to a dead I2C |

Code 2 means the fields have no source behind them. Codes 1 and 3 mean they
are drivetrain-derived and drift under wheel slip. `head_pitch` and `head_yaw`
are gimbal encoder values and are unaffected. The node copies the byte through
without validating it against the table. Transitions are logged: INFO back to
pods, ERROR into 2, WARN into 1 or 3.

### Proposed: POSE chassis yaw

Not applied: the bytes above are what both sides send today. `RobotPose`
already has `chassis_yaw` and `chassis_yaw_rate`, published as 0. Proposed
2026-09-29 for `CV_SPLIT_PLAN.md` W.1, to agree with the firmware side:

| Off | Size | Type    | Field              | Meaning                                    |
|-----|------|---------|--------------------|--------------------------------------------|
| 25  | 4    | float32 | `chassis_yaw`      | chassis heading, world, same sense and zero as `head_yaw`, radians |
| 29  | 4    | float32 | `chassis_yaw_rate` | its rate, rad/s                            |

33-byte payload. At `MCBV3@708b8d6` the firmware already has the heading as
`getYawAngleRelativeWorld() - getYawEncoderValue()` (the negative of
`AutoDriveCommand`'s `referenceAngle`), and the rate as the turret IMU's yaw
rate less the yaw motor's. `thornbots_pkg` turns it into the `chassis_yaw`
joint only: `root` stays heading-fixed, so localization doesn't read it.

### REF_SYS (id=3) — referee system status

11-byte payload, 20-byte frame, ~5 Hz, interleaved with POSE. Published on
`~/ref_sys` (`dji_serial_bridge/msg/RefSysStatus`, SensorDataQoS).

| Off | Size | Type    | Field                    | Meaning                                   |
|-----|------|---------|--------------------------|-------------------------------------------|
| 0   | 1    | uint8   | `game_stage`             | referee game stage enum value             |
| 1   | 2    | uint16  | `stage_time_remaining`   | seconds left in the current stage         |
| 3   | 2    | uint16  | `robot_hp`               | current robot HP                          |
| 5   | 1    | uint8   | `robot_id`               | normalised to red-team numbering (hero=1) |
| 6   | 4    | float32 | `delta_angle_got_hit_in` | radians from current heading of last hit  |
| 10  | 1    | uint8   | `booleans`               | 8 flags, MSB first, table below           |

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

## Either direction

### BYTE (id=5) — one raw byte

1-byte payload, 10-byte frame. The same ID both ways; the direction tells
them apart. Jetson → MCB: subscribed on `~/byte_to_mcb`, header discarded.
MCB → Jetson: published on `~/byte_from_mcb`, stamped like POSE. Both
`dji_serial_bridge/msg/McbByte`, depth-10 reliable.

| Off | Size | Type    | Field  | From / to      |
|-----|------|---------|--------|----------------|
| 0   | 1    | uint8   | `data` | `McbByte.data` |

The bridge gives the byte no meaning: sender and receiver agree on it.
Not in the firmware yet (README.md, firmware item 12).

---

## Not on the wire

`PanelDetection`, `PanelDetectionArray` and `TargetState` are ROS-internal,
travelling between `thornbots_pkg` and the CV pipeline.

`FireCommand` is gone as of 2026-09-20: it was merged into `CVTarget` as
`fire` + `delay_ms`, and `CV_TARGET` (id=1) grew `stamp_ms` so the delay had a
reference the MCB could age. `CvTargetPayload` went 17 → 23 bytes, then 19 on
2026-10-02 when `confidence` went (every frame is an aim point), then 15 on
2026-10-03 when `stamp_ms` went to match `position-based-cv`'s `CvTarget`.

Same two-repos-one-change rule as every other wire edit; see README.md's "MCB
firmware coordination".
