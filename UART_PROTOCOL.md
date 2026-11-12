# MCB ↔ Jetson UART protocol

Reference for every message crossing the serial link. Jetson side:
`include/dji_serial_bridge/dji_protocol.hpp`. MCB side: `JetsonSubsystem.hpp`
in the firmware repo.

Wire layout or firmware behavior changes require firmware coordination; see
[MCB firmware coordination](README.md#mcb-firmware-coordination). The idle-frame
semantics below leave the wire layout and firmware unchanged.

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
| 1  | `CV_TARGET`  | Jetson → MCB  | `CvTargetPayload`   | 13    | `~/cv_target`  | `dji_serial_bridge/msg/CVTarget`     | on publish, idle only |
| 2  | `POSE`       | MCB → Jetson  | `PosePayload`       | 25    | `~/pose`       | `dji_serial_bridge/msg/RobotPose`    | 90 Hz         |
| 3  | `REF_SYS`    | MCB → Jetson  | `RefSysPayload`     | 11    | `~/ref_sys`    | `dji_serial_bridge/msg/RefSysStatus` | 10 Hz         |
| 4  | `RELOCALIZE` | Jetson → MCB  | `RelocalizePayload` | 8     | `~/relocalize` | `geometry_msgs/msg/PointStamped`     | on correction |
| 5  | `PING`       | both          | `PingPayload`       | 1     | `~/ping_to_mcb`, `~/ping_from_mcb` | `dji_serial_bridge/msg/Ping` | on publish / on echo |
| 6 | `DEBUG` | MCB → Jetson | opaque bytes | 0–128 | `~/debug` | `dji_serial_bridge/msg/DebugBlob` | caller-driven, best effort |
| 7  | `TARGET_STATE` | Jetson → MCB | `TargetStatePayload` | 80 | `~/target_state` | `dji_serial_bridge/msg/TargetState` | per tracker update |

Each message is named after its topic, and its payload fields after the ROS
fields they carry. IDs 0-7 match `enum UartMessage` in the firmware's
`JetsonSubsystem.hpp`. Topics are in the node's private namespace:
`~/nav_goal` is `/dji_serial_bridge/nav_goal` unless remapped. The firmware
sends nine POSE then one REF_SYS on one 10 ms timer.

![Field frame, team starts and robot frames, top view](docs/field_frame.svg)

**Field-aligned frames.** Both wire frames use REP-103 metres/radians,
z up, x toward blue's base, y left. Raw POSE/CV_TARGET coordinates boot at
(0, 0) for both teams and remain relative to that physical start. Navigation
coordinates become field-centred after the Jetson's startup RELOCALIZE:
the position comes from the Jetson's absolute field localization. Its prior
comes from that team's launch configuration, not firmware.
`thornbots_pkg/auto.launch.py` owns `red_start_x`, `red_start_y`, `blue_start_x`,
`blue_start_y` defaults; `/etc/thornbots/launch.env` can override each independently
through `AUTO_LAUNCH_ARGS` ([configuration](../thornbots_pkg/README.md#point_to_cv_target)).
Both teams face +y at boot (chassis heading π/2).
`root` stays heading-fixed at yaw 0 for both teams; both `chassis_yaw` joints
should be π/2. This is the physical boot contract, not proof of firmware
configuration or telemetry: current POSE omits chassis yaw and the bridge
publishes its zero placeholder (see [proposed chassis yaw](#proposed-pose-chassis-yaw)).
POSE and CV_TARGET use continuous raw pod odometry (`odom` on the Jetson),
which RELOCALIZE and team selection never move.
NAV_GOAL and RELOCALIZE use the
absolute field frame (`map` on the Jetson with AMCL/saved-map localization;
team-start `nav_odom` in mapping/odometry-only mode). RELOCALIZE changes
only the MCB's navigation offset; it cannot move POSE or an existing aim
point. Navigation offsets are translation-only; firmware keeps raw field-aligned
axes between fixes. A yaw in `map->nav_odom` rotates the Jetson's displacement
prediction before sending x/y; it never rotates firmware odometry or aim axes. MCB navigation is initially start-relative; autonomous routes and
NAV_GOAL driving wait for the first RELOCALIZE. The Jetson waits for the
referee team, seeds navigation at raw position plus that team's start, and
sends the absolute navigation position. AMCL also waits for that team before
starting with its team-start prior; it never uses global localization.
Raw odometry can drift,
as [REP 105](https://github.com/ros-infrastructure/rep/blob/master/rep-0105.rst)
permits. Neither raw odometry nor an aim point is a global localization fix.

An inbound frame with any other `msgType` is counted, logged at WARN, dropped.

---

## Jetson → MCB

Transmitted from the subscriber callback: one ROS message in, one frame out.
No timer, no retransmission; the wire rate is the publisher's rate. Fields are
copied without clamping; CV_TARGET and TARGET_STATE have the freshness gate below.

### MCB receive

The MCB stores incoming frames in one `mostRecentMessage` slot, shared by
all message types. `getMsg` consumes it only when the type and payload size
match its caller. A second frame received before consumption overwrites the
first, including PING, RELOCALIZE and NAV_GOAL. There is no queue or
application acknowledgement; successful Jetson transmission does not prove
that the MCB applied a command.

### NAV_GOAL (id=0) — navigation goal

8-byte payload, 17-byte frame. Subscribed on `~/nav_goal`
(`geometry_msgs/msg/PointStamped`, depth-10 reliable). `point.z` and the
header are discarded.

| Off | Size | Type    | Field     | From      |
|-----|------|---------|-----------|-----------|
| 0   | 4    | float32 | `x`       | `point.x` |
| 4   | 4    | float32 | `y`       | `point.y` |

An absolute field goal in metres, consumed by the autonomous drive controller
only after its first RELOCALIZE. `thornbots_nav/waypoint_nav` publishes it and
judges arrival using the relay's nominal held-navigation estimate.

### CV_TARGET (id=1) — idle look point

13-byte payload, 22-byte frame. Subscribed on `~/cv_target`
(`dji_serial_bridge/msg/CVTarget`, SensorDataQoS, best-effort).

| Off | Size | Type    | Field        | From                          |
|-----|------|---------|--------------|-------------------------------|
| 0   | 4    | float32 | `x`          | `CVTarget.x`                  |
| 4   | 4    | float32 | `y`          | `CVTarget.y`                  |
| 8   | 4    | float32 | `z`          | `CVTarget.z`                  |
| 12  | 1    | uint8   | `flags`      | packed booleans, below        |

| Bit | `CVTarget` field      | Set means                                        |
|-----|-----------------------|--------------------------------------------------|
| 1   | `type_c_based_patrol` | MCB permission to patrol on its own; clear stops it |
| 2   | `turn_to_hit`         | MCB permission to turn toward where it got hit  |

Bits 0 and 3-7 are reserved, sent as 0. The bridge packs these booleans
without interpreting them, as it unpacks REF_SYS's.

A non-target point the gimbal should look at: never a target and never a
fire request (aiming and firing ride [TARGET_STATE](#target_state-id7--tracked-enemy-armor-model)).
The Jetson sends idle points only while it holds no target state younger than
its target timeout. Idle looks can hold the last enemy position, follow the gun
after a firmware hit turn, check a lidar track or face the enemy starting zone;
their selection belongs to `thornbots_pkg` (see the
[lidar-guided aiming design](../docs/lidar-guided-aiming-2026-10-25.md#where-the-head-looks-with-no-camera-target)).
The MCB holds an idle point for its 200 ms validity, behind any held target
state; bits 1 and 2 separately permit firmware patrol after expiry and
interruption toward a hit.

`x/y/z` is a world-frame position in metres in raw aim `odom` (z up), as in
TARGET_STATE. Freshness: [Jetson → MCB freshness](#jetson--mcb-freshness).

### TARGET_STATE (id=7) — tracked enemy armor model

80-byte payload, 89-byte frame (7.7 ms at 115200 baud). Subscribed on
`~/target_state` (`dji_serial_bridge/msg/TargetState`, SensorDataQoS,
best-effort); one frame per tracker update, no Jetson timer.

| Off | Size | Type      | Field              | From                                   |
|-----|------|-----------|--------------------|----------------------------------------|
| 0   | 12   | float32×3 | `center`           | `center`, m                            |
| 12  | 12   | float32×3 | `velocity`         | `velocity`, m/s                        |
| 24  | 12   | float32×3 | `acceleration`     | `acceleration`, m/s²                   |
| 36  | 4    | float32   | `yaw`              | `yaw` wrapped to `[-pi, pi)`, rad      |
| 40  | 4    | float32   | `yaw_rate`         | `yaw_rate`, rad/s, + CCW               |
| 44  | 8    | float32×2 | `radius`           | `radius[0..1]`, m                      |
| 52  | 8    | float32×2 | `z_offset`         | `z_offset[0..1]`, m                    |
| 60  | 12   | float32×3 | `panel`            | `panel`, m                             |
| 72  | 2    | uint16    | `state_age_ms`     | write time − `header.stamp`            |
| 74  | 2    | uint16    | `detection_age_ms` | write time − `detection_stamp`         |
| 76  | 2    | uint16    | `track_id`         | low 16 bits of `robot_track_id`        |
| 78  | 1    | uint8     | `confidence`       | `round(confidence × 255)`, clamped     |
| 79  | 1    | uint8     | `flags`            | bit 0 `valid`, bit 1 `type_c_based_patrol`; 2-7 reserved 0 |

Vectors are x, y, z in raw aim `odom` at the state's stamp; the armor model
(panel k at `center + radius[k % 2]·(cos(yaw + k·pi/2), sin(yaw + k·pi/2), 0)
+ (0, 0, z_offset[k % 2])`) is `TargetState.msg`'s. `valid` clear means
unconverged: the MCB aims at `panel` without leading or firing.

The MCB owns everything after tracking: state propagation, spin
classification, panel choice, lead, spin shot windows, fire lock, confidence
and detection-age gates, and shot timing. Its ages are the wire ages plus time
since receipt; uncorrected wire and USB latency (about 8 ms for this frame)
make them slightly young. A held state aims until `state_age` reaches the
target timeout (0.5 s); firing additionally requires `detection_age` at most
0.15 s. Tuned values and the planner are in
[MCBV3 aiming](../firmware/MCBV3/README.md#aim-and-navigation-odometry).
While it holds a target state the MCB makes no hit turn.

The bridge sets both ages at the final write boundary
([freshness](#jetson--mcb-freshness)), saturating at 65535 ms; it clamps
`confidence` and wraps `yaw` but otherwise copies fields.

### Jetson → MCB freshness

The bridge drops a CV_TARGET or TARGET_STATE unless its stamp (CVTarget's
decision stamp; TargetState's `header.stamp`) is nonzero, well formed and
less than 200 ms old at the final write boundary. Exactly 200 ms is expired.
No payload field is rewritten except TARGET_STATE's ages, and expiry sends
no replacement. Firmware receipt expiry still applies.

The check runs after acquiring the write mutex and building the frame, directly
before a single nonblocking write. A nonempty or unreadable serial output queue
also drops the frame: no previous software TX backlog may delay it. There is
no wait/retry after validation; EAGAIN and short writes fail. This bounds
executor/mutex/write queuing on the Jetson. Normal transmission takes about 1.9 ms
(CV_TARGET) or 7.7 ms (TARGET_STATE) at 115200 baud. Driver/hardware stalls after write acceptance and
MCB scheduling are outside this software bound; end-to-end expiry would
require a shared clock/deadline wire contract.

Producer and bridge must use the same ROS clock and `use_sim_time` setting.
Negative seconds, nanoseconds outside [0, 1e9), zero/unstarted clocks, future
stamps (including after a backward clock reset), and ages of 200 ms or more
are rejected. There is no future tolerance or wall-time fallback. Simulation
may use positive subsecond stamps; pausing ROS time pauses decision age too.
An epoch mismatch normally appears expired or future and is rejected; stamps
carry no clock identifier, so coincident numeric times from different clocks
cannot be detected here. Configure the clock consistently across the graph.

### RELOCALIZE (id=4) — lidar position fix

8-byte payload, 17-byte frame. Subscribed on `~/relocalize`
(`geometry_msgs/msg/PointStamped`, depth-10 reliable). `point.z` and the
header are discarded.

| Off | Size | Type    | Field       | From      |
|-----|------|---------|-------------|-----------|
| 0   | 4    | float32 | `x`         | `point.x` |
| 4   | 4    | float32 | `y`         | `point.y` |

The localization-estimated **absolute field position** in metres, predicted
to firmware application time: AMCL-corrected `map` pose for normal operation,
team-start `nav_odom` pose while mapping or without a global map. The MCB
replaces its navigation offset against current raw pod position; it never adds
a correction delta. Navigation controllers see the new position; raw POSE and
gimbal aim do not. The first accepted fix enables autonomous navigation.
The relay applies covariance/freshness, startup, jump and step gates, then
repeats fixes using fresh inputs to tolerate lost frames. The AMCL policy also
refreshes valid fixes periodically. No wire acknowledgement proves receipt.
The maintained [relay policy](../thornbots_pkg/README.md#mcb_relay) owns the
thresholds and timing estimates. Payload fields and firmware interpretation
are unchanged.
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
The optional simulation-only [capture metadata interface](README.md#simulation-capture-metadata)
uses verified side-channel capture stamps instead; the UART frame is unchanged.

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
| 20  | 4    | float32 | `head_yaw`   | gimbal yaw, field, CCW, `(-2pi, 2pi)`, rad  |
| 24  | 1    | uint8   | `odom_status`| which source produced x/y/vel, below     |

| Code | `RobotPose` constant         | Source of `x/y/vel_x/vel_y`                  |
|------|------------------------------|----------------------------------------------|
| 0    | `ODOM_PODS`                  | odometry pods, healthy                       |
| 1    | `ODOM_DRIVETRAIN`            | drivetrain odometry, degraded by wheel slip  |
| 2    | `ODOM_I2C_DEAD`              | none: I2C bus dead, no pod data, no fallback |
| 3    | `ODOM_I2C_DEAD_DRIVETRAIN`   | drivetrain odometry, pods lost to a dead I2C |

Codes 1 and 3 are reserved. Sentry firmware sends `ODOM_I2C_DEAD` (2) before
its first successful Pico read and once the last successful read is 500 ms
old, otherwise `ODOM_PODS` (0). Pico values refresh about every 260 ms. A read
that succeeds with frozen values still reports 0, so frozen pods are not
flagged on the wire.
Hero and standard have no odometry: they always send `x/y/vel` 0 with code 2,
and `head_yaw` without a start yaw.

`x/y` start at zero for both teams; velocity and `head_yaw` share the raw
field-aligned aim axes (above).
RELOCALIZE never changes them. Taproot IMU yaw is non-negative, but the
gimbal yaw estimator extrapolates it without wrapping. POSE applies
`std::fmod(estimated_yaw + start_yaw, 2*pi)`, which preserves sign, so
`head_yaw` can be negative. Consumers needing `[0, 2pi)` must wrap it.

Code 2 means the fields have no source behind them. Codes 1 and 3 mean they
are drivetrain-derived and drift under wheel slip. `head_pitch` and `head_yaw`
are independent of the odometry status and are unaffected. The node copies
the byte through without validating it against the table. Transitions are logged: INFO back to
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

33-byte payload. The firmware has the field heading as
`getYawAngleRelativeWorld() + getStartYaw() - getYawEncoderValue()` (the negative of
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

### DEBUG (id=6) — opaque firmware debug

The payload is 0–128 uninterpreted bytes, with no tags, schema or padding.
`dataLength` is its exact length; frames are 9–137 bytes. The bridge publishes
`DebugBlob.data` unchanged on `~/debug` with SensorDataQoS, empty `frame_id`
and the same estimated frame send-start stamp as POSE. Oversized debug headers
are rejected before waiting for the payload. Firmware developers own offline
decoding against the firmware revision that produced the bag.

`JetsonSubsystem::sendDebug(data, length)` is a nonblocking, bounded API for
cooperative firmware callers; there is no production sender or sampling timer.
Debug is expendable: queue rejection, link saturation and ROS best-effort loss
are allowed. It has no effect on robot behavior and no acknowledgement.
The firmware gives overdue telemetry priority and admits debug as fast as
available UART budget allows, with one shared pending slot and no fixed rate
limit. At 115200 baud (8N1), a maximum 137-byte frame takes 11,893 µs.
Admission reserves its full airtime, a maximum POSE frame airtime (2,952 µs),
and 2 ms for servicing the two completions. Both debug and the delayed telemetry
must finish before the *second* future 10 ms telemetry deadline.
Thus one debug frame can delay at most one POSE or REF_SYS frame. Telemetry
retains every deadline (nine POSE, one REF_SYS per 100 ms); late frames are sent
before another debug frame. Under a continuous maximum-size flood, long-run
rates remain 90 Hz POSE and 10 Hz REF_SYS, with no skipped slots.

Worst-case added POSE latency is **10 ms**, giving a maximum 30 ms POSE
inter-arrival gap across a REF_SYS slot. This bound assumes cooperative loop
service at least every 1 ms, a writable link and the configured baud; a stalled
loop or transport fault is outside it. POSE and REF_SYS are sampled and stamped
at actual frame send start, not their nominal deadline. After a scheduler service
gap longer than 100 ms (for example IMU calibration), firmware starts a fresh
nine-POSE/one-REF_SYS cycle with its first deadline 10 ms after resuming; it does
not replay samples for the paused interval. Under continuous service, overdue
slots are retained. Wire-idle accounting
uses the same baud constant as UART initialization and requires the STM32
transmission-complete flag, including the final shift-register byte. See the firmware's
[debug API](../firmware/MCBV3/README.md#opaque-debug-blobs) for caller lifetime
rules. RX polling remains independent and runs before TX admission: debug
must not affect CV_TARGET or RELOCALIZE reception or their mailboxes.

The startup per-run recorder includes `/dji_serial_bridge/debug` in normal
and match runs whenever bag recording is enabled. No content is enabled by
default; individual firmware developers choose what and when to send.

---

## Either direction

### PING (id=5) — MCB Mapping Machine and link check

1-byte payload, 10-byte frame. The same ID both ways; the direction tells
them apart. Jetson → MCB: subscribed on `~/ping_to_mcb`, header discarded.
MCB → Jetson: published on `~/ping_from_mcb`, stamped like POSE. Both
`dji_serial_bridge/msg/Ping`, depth-10 reliable.

| Off | Size | Type    | Field    | From / to     |
|-----|------|---------|----------|---------------|
| 0   | 1    | uint8   | `number` | `Ping.number` |

The MMM application contract below uses the existing payload and UART ID.
The bridge remains a pure translator; Jetson coordination belongs to the
in-stack C++ [`mmm` node](../thornbots_pkg/README.md#mcb-mapping-machine).
[MCB receive](#mcb-receive) can lose frames before consumption.

| number | Origin | Meaning | Exchange (C = MCB, J = Jetson) |
| --- | --- | --- | --- |
| 0 | J | Main-only link check | J→C 0, C→J 0; no third leg |
| 1 | C | Start mapping | C→J 1, J→C 1, C→J 1, J→C 1; C ignores |
| 2 | J | Ready at captured start | J→C 2, C→J 2, J→C 2; C echoes again, J ignores |
| 3 | C | Save asynchronously | C→J 3, J→C 3, C→J 3, J→C 3; C ignores |
| 4 | J | Mapping / away from start; Done→Mapping after failed Ready save | J→C 4, C→J 4, J→C 4; C echoes again, J ignores |
| 6 | J | Setup failed | J→C 6, C→J 6, J→C 6; C echoes again, J ignores |

Only C owns the states. Main --1--> Setup mapping --2--> Ready to save;
Ready to save --4--> Mapping --2--> Ready to save. Ready to save --3--> Done;
Done --4--> Mapping. Setup mapping --6--> Done.
Mapping --3--> Save incomplete, immediately back to Mapping. Setup disables
drive and looks forward. Ready gives green LED/1 Hz beep, forward head and
slow drive capped at 0.4 m/s. Mapping locks the forward head and uses the same
drive cap. Done disables drive and permits power-off, with no head-down pose.
C replies to 0 only in Main;
it ignores 0 elsewhere. Zero is the one exception to always-ack by prior user
decision and never changes state. Unknown numbers are ignored, never echoed.

J publishes 0 once per wall second. Even transactions allow one outstanding
number, retry every 0.5 wall seconds, and stop after ten sends (timeout at
5 seconds). A matching echo ends retries and emits one final leg; unsolicited,
late-after-timeout and duplicate even echoes are ignored. Timeout logs an
error and the coordinator retries the latest pose condition after a five-second
quiet interval. A completed/timed-out even number cannot be reused for five
seconds. A changed pose condition waits for the outstanding transaction to end.
C echoes every received 2/4/6, including retries and final legs: it cannot
distinguish them. It changes state only once from a legal source state;
repeated requests remain idempotent, including recovery after an uncertain
timeout. J ignores echoes when it is not waiting for that number, ending the exchange.

There is no sequence/transaction field. J echoes **every** received 1/3 in any
state, even an illegal request, duplicate, retry or C's final leg, **before**
dispatching any work. The five-second sliding same-number receive window gates
only work: J dispatches only after at least five wall seconds without that
number. A distinct new same-number request within that window is ACKed but not
executed. C retries an unacknowledged 1/3 at intervals of at least 0.5 wall
seconds; J ACKs retries immediately. C sends its final leg only once, on a
matching echo while awaiting it, then ignores duplicate/late echoes in every
state. A start after the quiet interval begins a new Jetson session, including after
an MCB restart or setup failure; while a save is running J rejects it with 6.
This new session cancels the previous even transaction and clears its completed
state and cooldowns so the new setup can report 2/6. Odd receive windows remain
in force. J acknowledges a save already running with a loud warning. Repeated
saves after the quiet interval are new requests.

The receiver of a number originated by its peer always echoes it (J for 1/3,
C for 2/4/6), while work and state changes happen at most once. The originator
echoes its peer's echo only once while awaiting it, then ignores further
echoes. Thus the receiver ACKs the final leg, and the originator consumes that
ACK without sending again; retries cannot sustain an echo loop.

These rules require each frame and its resulting echo to expire within five
seconds of the originating send, and no concurrent
same-number odd request. Arbitrarily delayed final legs and fresh requests
are indistinguishable on this wire; delivery/exactly-once execution across
coordinator or MCB restarts is not guaranteed. On coordinator restart begin a
new attempt from MCB Main. Disable other PING publishers/diagnostics while MMM
runs. Ack means receipt, **never successful save or durable storage**. J treats
3 as a Ready save only when its last completed even exchange was 2 and its
local start gate still passes; otherwise it takes an incomplete snapshot.
After a successful validated save, J may request its opt-in host shutdown
mechanism. Only when a Ready save fails to start or complete, J sends 4
to return C from Done to Mapping. J waits for that exchange to complete before
resuming pose announcements, sending 2 again if its start gate passes; the
five-second same-number even reuse cooldown still applies. Failed incomplete
snapshots and shutdown-request failures after a successful save do not trigger this 4.
Save failure leaves J running for recovery; snapshots never shut it down.
In Main, Setup mapping and Mapping, 4 is echoed without changing state;
Ready to save still transitions to Mapping on 4. Setup transitions to Done on 6;
every other state echoes 6 without a state change. J sends 6 on setup timeout,
reset/input failure, or a running localization mode other than mapping.
C powers off its own side independently, so Done can still interrupt
storage if that also removes Jetson power before the save finishes.

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
