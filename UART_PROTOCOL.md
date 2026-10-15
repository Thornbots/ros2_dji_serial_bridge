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

`seq` increments per transmitted frame and is not checked on receive. The
MCB sends 0 on every frame (`UARTCommunication.cpp:21`). No
retransmission, acknowledgement or flow control in either direction.

## Message types

| ID | Name         | Direction     | Payload struct      | Bytes | ROS topic      | ROS type                             | Rate          |
|----|--------------|---------------|---------------------|-------|----------------|--------------------------------------|---------------|
| 0  | `NAV_GOAL`   | Jetson → MCB  | `NavGoalPayload`    | 8     | `~/nav_goal`   | `geometry_msgs/msg/PointStamped`     | on publish    |
| 1  | `CV_TARGET`  | Jetson → MCB  | `CvTargetPayload`   | 15    | `~/cv_target`  | `dji_serial_bridge/msg/CVTarget`     | on publish    |
| 2  | `POSE`       | MCB → Jetson  | `PosePayload`       | 25    | `~/pose`       | `dji_serial_bridge/msg/RobotPose`    | 90 Hz         |
| 3  | `REF_SYS`    | MCB → Jetson  | `RefSysPayload`     | 11    | `~/ref_sys`    | `dji_serial_bridge/msg/RefSysStatus` | 10 Hz         |
| 4  | `RELOCALIZE` | Jetson → MCB  | `RelocalizePayload` | 8     | `~/relocalize` | `geometry_msgs/msg/PointStamped`     | on correction |
| 5  | `PING`       | both          | `PingPayload`       | 1     | `~/ping_to_mcb`, `~/ping_from_mcb` | `dji_serial_bridge/msg/Ping` | on publish / on echo |

Each message is named after its topic, and its payload fields after the ROS
fields they carry. IDs 0-5 match `enum UartMessage` in the firmware's
`JetsonSubsystem.hpp` on MCBV3's `position-based-cv` branch
(Thornbots/MCBV3#74); MCBV3 `708b8d6` used the old names (`ROS_MSG`,
`CV_MSG`, `POSE_MSG`, `REF_SYS_MSG`). Topics are in the node's private
namespace: `~/nav_goal` is `/dji_serial_bridge/nav_goal` unless remapped.
Rates are the firmware's at `0885a69`: nine POSE then one REF_SYS on one
10 ms timer.

**Field-aligned frames.** Both wire frames use REP-103 metres/radians,
z up, (0, 0) initially at the field
centre, x toward blue's base, y left. The MCB boots at its team's start,
red (-4.625, 0) facing +x, blue (4.625, 0) facing -x, and adds it to its
odometry and IMU yaw (`OdometrySubsystemConstants.hpp`; not measured on
the field). POSE and CV_TARGET use continuous raw pod odometry (`odom` on
the Jetson), which RELOCALIZE never moves. Boot/team selection can reanchor
the provisional start: red-to-blue selection changes the start by 9.25 m
and changes the startup heading. This initialization exception is not a
navigation correction; discard pose interpolation across that discontinuity.
NAV_GOAL and RELOCALIZE use the
corrected navigation frame (`nav_odom` on the Jetson). RELOCALIZE changes
only the MCB's navigation offset; it cannot move POSE or an existing aim
point. Navigation offsets are translation-only; these frames keep the same
axes, coincide at initialization and may subsequently diverge in position.
The field-centred raw origin is an application choice; raw odometry can drift,
as [REP 105](https://github.com/ros-infrastructure/rep/blob/master/rep-0105.rst)
permits. Neither raw odometry nor an aim point is a global localization fix.

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

A goal in the corrected navigation frame, metres, consumed by its autonomous
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
| 0   | `fire`                | request this aim point to exit `delay_ms` after receipt |
| 1   | `type_c_based_patrol` | MCB permission to patrol on its own; clear stops it |
| 2   | `turn_to_hit`         | MCB permission to turn toward where it got hit  |
| 3   | `patrol`              | this aim point came from Jetson patrol, including hit-turn patrol |

Bits 4-7 are reserved, sent as 0. Bit 3 is informational and currently
ignored by firmware; it does not gate bit 0 or otherwise change firing. Bit 1
is MCB patrol permission, while bit 3 identifies the source of this point.
The bridge packs the byte from `CVTarget`'s booleans, as it unpacks REF_SYS's.
Bits 1 and 2 reach the MCB with every frame.
The Jetson clears bit 2 whenever it holds a target estimate, including a
coasting/predicted track during brief occlusion and an unconverged estimate.
It sets bit 2 only with no estimate, including Jetson patrol. There is no
cooldown; the next target frame cancels any firmware hit turn.
Every frame is an aim point: the Jetson decides aim and fire itself, and
the MCB aims at these points while they are valid. Bits 1 and 2 separately
permit firmware patrol after expiry and interruption toward a hit. With no target the Jetson
patrols, sending points to sweep the gun with `fire` clear
(`thornbots_pkg` `patrol_enabled`, on by default). With that off it sends
nothing between targets.

No stamp crosses the wire (dropped 2026-10-03 to match MCBV3
`position-based-cv`'s 15-byte `CvTarget`): the MCB runs `delay_ms` from frame receipt.

Aim and fire travel as one frame on purpose. A fire delay is only meaningful
against the aim point it was solved for; two frames could arrive apart, drop
independently, or pair up wrongly on the MCB and fire at a point the delay
was never computed for. `delay_ms` includes the estimated indexer latency plus any alignment wait.
Firmware requests indexing after `max(0, delay_ms - FIRING_LATENCY_TIME)`;
its configured latency and calibration limits live in
[MCBV3 aiming](../firmware/MCBV3/README.md#aim-and-navigation-odometry).
A delay shorter than that latency requests indexing immediately; physical
exit still takes time. `delay_ms` = 0 with `fire` set means request now;
`fire` clear means `delay_ms` is meaningless.

`x/y/z` is a world-frame position in metres in raw aim `odom` (z up).
The firmware subtracts its never-relocalized, field-aligned pod position.
The MCB holds the point with its IMU and
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

The localization-estimated position in corrected navigation `nav_odom`, metres.
The MCB sets its navigation offset against its current raw pod position.
Navigation controllers see this correction; raw POSE and gimbal aim do not.
The node logs every transmission beside the latest raw `~/pose`; those
coordinates are in different frames after a correction.

---

## MCB → Jetson

A dedicated read thread polls the port, scans for `0xA5`, checks CRC-8 over the
header and CRC-16 over the frame (both dropped on mismatch while `enforce_crc`
is true), then checks payload length against the struct size before publishing.
`header.stamp` on both published messages is when the MCB started sending the
frame: the read time of its last byte less the frame's wire time at `baudrate`
(10 bits per byte). No MCB clock is on the wire, and the USB-serial latency
before a read is not taken off. A frame whose send start would precede clock
zero (bytes read before a simulated clock starts) is dropped with a warning.

### POSE (id=2) — chassis pose and gimbal angles

25-byte payload, 34-byte frame, 90 Hz. Published on `~/pose`
(`dji_serial_bridge/msg/RobotPose`, SensorDataQoS).

| Off | Size | Type    | Field         | Meaning                                  |
|-----|------|---------|---------------|------------------------------------------|
| 0   | 4    | float32 | `x`           | chassis X, raw aim odom, metres          |
| 4   | 4    | float32 | `y`          | chassis Y, raw aim odom, metres          |
| 8   | 4    | float32 | `vel_x`      | chassis X velocity, m/s                  |
| 12  | 4    | float32 | `vel_y`      | chassis Y velocity, m/s                  |
| 16  | 4    | float32 | `head_pitch` | gimbal pitch encoder value, radians      |
| 20  | 4    | float32 | `head_yaw`   | gimbal yaw, field, CCW, `[0, 2pi)`, rad  |
| 24  | 1    | uint8   | `odom_status`| which source produced x/y/vel, below     |

| Code | `RobotPose` constant         | Source of `x/y/vel_x/vel_y`                  |
|------|------------------------------|----------------------------------------------|
| 0    | `ODOM_PODS`                  | odometry pods, healthy                       |
| 1    | `ODOM_DRIVETRAIN`            | drivetrain odometry, degraded by wheel slip  |
| 2    | `ODOM_I2C_DEAD`              | none: I2C bus dead, no pod data, no fallback |
| 3    | `ODOM_I2C_DEAD_DRIVETRAIN`   | drivetrain odometry, pods lost to a dead I2C |

`x/y`, velocity and `head_yaw` are in the raw field-aligned aim frame (above).
RELOCALIZE never changes them. At
`0885a69` they were x right, y forward of the power-on spot, and
`head_yaw` zero at IMU boot. The firmware always sends `ODOM_PODS` today.
README.md "Where the firmware stands" has the line refs.

Code 2 means the fields have no source behind them. Codes 1 and 3 mean they
are drivetrain-derived and drift under wheel slip. `head_pitch` and `head_yaw`
are gimbal encoder values and are unaffected. The node copies the byte through
without validating it against the table. Transitions are logged: INFO back to
pods, ERROR into 2, WARN into 1 or 3.

### Proposed: POSE chassis yaw

Not applied: the bytes above are what both sides send today. `RobotPose`
already has `chassis_yaw` and `chassis_yaw_rate`, published as 0. Proposed
2026-09-29, to agree with the firmware side; see
[aim and navigation frames](README.md#aim-and-navigation-frames):

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

11-byte payload, 20-byte frame, 10 Hz, interleaved with POSE. Published on
`~/ref_sys` (`dji_serial_bridge/msg/RefSysStatus`, SensorDataQoS).

| Off | Size | Type    | Field                    | Meaning                                   |
|-----|------|---------|--------------------------|-------------------------------------------|
| 0   | 1    | uint8   | `game_stage`             | referee game stage enum value             |
| 1   | 2    | uint16  | `stage_time_remaining`   | seconds left in the current stage         |
| 3   | 2    | uint16  | `robot_hp`               | current robot HP                          |
| 5   | 1    | uint8   | `robot_id`               | normalised to red-team numbering (hero=1) |
| 6   | 4    | float32 | `delta_angle_got_hit_in` | radians from current chassis heading toward last hit, positive CCW (REP 103); 123 if none |
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

The hit angle is relative to the chassis heading when REF_SYS is sent, not
the gimbal heading. Left is positive and right negative, wrapped to
`[-pi, pi]`; 123 means no hit since the preceding REF_SYS frame.
Firmware retains the latest hit bearing across control cycles until a
REF_SYS frame successfully reports it. Multiple hits before that report
replace the pending bearing; a following report without a new hit carries 123.
The node unpacks the byte into the eight named booleans.

---

## Either direction

### PING (id=5) — link check, echoed by the MCB

1-byte payload, 10-byte frame. The same ID both ways; the direction tells
them apart. Jetson → MCB: subscribed on `~/ping_to_mcb`, header discarded.
The MCB sends every Ping it gets straight back (`JetsonSubsystem::refresh`);
MCB → Jetson: published on `~/ping_from_mcb`, stamped like POSE. Both
`dji_serial_bridge/msg/Ping`, depth-10 reliable.

| Off | Size | Type    | Field    | From / to     |
|-----|------|---------|----------|---------------|
| 0   | 1    | uint8   | `number` | `Ping.number` |

---

## Not on the wire

The hosted fixture and Renode board adapter's `MCB3` motor/sensor pipe belongs to the
[sim hardware fixture](../sim/README.md#mcb-emulator). It does not add a UART
message or change any payload, baud rate or stamp contract here.

`PanelDetection`, `PanelDetectionArray` and `TargetState` are ROS-internal,
travelling between `thornbots_pkg` and the CV pipeline.

`FireCommand` is gone as of 2026-09-20: it was merged into `CVTarget` as
`fire` + `delay_ms`, and `CV_TARGET` (id=1) grew `stamp_ms` so the delay had a
reference the MCB could age. `CvTargetPayload` went 17 → 23 bytes, then 19 on
2026-10-02 when `confidence` went (every frame is an aim point), then 15 on
2026-10-03 when `stamp_ms` went to match `position-based-cv`'s `CvTarget`.

Same two-repos-one-change rule as every other wire edit; see README.md's "MCB
firmware coordination".
