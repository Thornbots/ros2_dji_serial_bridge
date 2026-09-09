# ros2_dji_serial_bridge

ROS 2 node that bridges the Jetson-side DJI-framed UART protocol spoken by
the MCB (main control board) with ROS 2 topics.

> **⚠ Firmware coordination needed:** `CV_MSG` (id=1)'s wire format changed
> twice. First (2026-07-28) `CVDataPayload` shrank from 40 to 16 bytes
> (velocity/acceleration fields dropped). Then `x/y/z`'s **meaning** changed
> from a camera-frame offset to a ROOT-FRAME POSITION, and the struct grew
> back to 17 bytes with a trailing `flags` byte (bit0 `lead_applied`, bit1
> `track_valid`). The MCB firmware's matching struct (outside this repo)
> must be updated to match both the new size and the new semantics before
> real-hardware CV aiming works again. A firmware built against either older
> layout will misparse this frame, and even one that parses the new byte
> layout correctly will aim wrong if it still treats x/y/z as
> camera-relative. See `## Notes` below for the full detail.

## Notes

### Message types (dji_serial_bridge_node.cpp)

The node handles all five message types defined in the MCB firmware's
`JetsonSubsystem.hpp`:

| ID | Direction        | ROS topic     | ROS msg type                          |
|----|-------------------|---------------|----------------------------------------|
| 0  | Jetson → MCB      | ~/nav_goal    | geometry_msgs/msg/Point                |
| 1  | Jetson → MCB      | ~/cv_target   | dji_serial_bridge/msg/CVTarget         |
| 2  | MCB   → Jetson    | ~/pose        | dji_serial_bridge/msg/RobotPose        |
| 3  | MCB   → Jetson    | ~/ref_sys     | dji_serial_bridge/msg/RefSysStatus     |
| 4  | Jetson → MCB      | ~/relocalize  | geometry_msgs/msg/Point                |

Topics use the node's private namespace so you can remap them in a launch
file. For example, `~/nav_goal` resolves to `/dji_serial_bridge/nav_goal` by
default but can be remapped to `/nav_goal`.

CV_MSG (id=1) wire format, with detail behind the warning at the top.
Dropping `v_x/v_y/v_z`/`a_x/a_y/a_z` took `CVDataPayload` 40 -> 16 bytes
and moved `confidence` from byte offset 36 to 12. The `flags` byte
appended after that took it to 17: bit0 `lead_applied` (does x/y/z include
the intercept/lead solve), bit1 `track_valid` (is it backed by a converged
`target_tracker` estimate rather than an unfiltered raw panel position).
`x/y/z` is a point Type-C aims at directly, applying its own
gravity/drag/muzzle geometry, never a barrel attitude. Velocity and spin
stay off the wire by design, ROS-internal on `thornbots_pkg`'s
`/cv/target_state` (`TargetState.msg`).

ROS parameters (see `config/dji_bridge_params.yaml` for defaults):

- `device` (string) : serial device path, e.g. `/dev/ttyTHS1`
- `baudrate` (int) : baud rate in bits-per-second, e.g. 115200
- `read_poll_ms` (int) : poll() timeout in milliseconds (10 is fine)
- `enforce_crc` (bool) : drop frames whose CRC does not match (default true)
- `diag_interval_s` (int) : how often to print diagnostic stats (default 5)
- `debug_log` (bool) : log everything if true

The node has no opinion on the other ends of these topics. Upstream
producers and consumers (thornbots_pkg's mcb_relay, the CV pipeline, etc.)
publish or subscribe directly, remapped as needed.

### Jetson → MCB payloads (dji_protocol.hpp)

Three of the five message types travel outbound. The node transmits only on
receipt of a ROS message: one subscriber callback, one frame, no timer and no
retransmission, so the wire rate is whatever `mcb_relay` publishes at. Nothing
is clamped or validated on the way out; a NaN in the ROS message reaches the
MCB as a NaN.

Every frame is the 7-byte header, then the payload below, then CRC-16.
Offsets are relative to the start of the payload (absolute offset = payload
offset + 7). All fields little-endian.

#### ROS_MSG (id=0) — navigation goal, 8-byte payload, 17-byte frame

Subscribed on `~/nav_goal` (`geometry_msgs/msg/Point`, depth-10 reliable QoS).
`z` is discarded.

| Off | Size | Type    | Field     | From          |
|-----|------|---------|-----------|---------------|
| 0   | 4    | float32 | `targetX` | `Point.x`     |
| 4   | 4    | float32 | `targetY` | `Point.y`     |

Field goal in the MCB's odometry frame, metres, consumed by its autonomous
drive controller. No publisher exists in this workspace yet: `mcb_relay`
wires up `~/cv_target` and `~/relocalize` only, so this path is implemented
but dormant.

#### CV_MSG (id=1) — aim point, 17-byte payload, 26-byte frame

Subscribed on `~/cv_target` (`dji_serial_bridge/msg/CVTarget`, SensorDataQoS,
best-effort, so a dropped frame is expected and fine at CV rates).

| Off | Size | Type    | Field        | From                  |
|-----|------|---------|--------------|-----------------------|
| 0   | 4    | float32 | `x`          | `CVTarget.x`          |
| 4   | 4    | float32 | `y`          | `CVTarget.y`          |
| 8   | 4    | float32 | `z`          | `CVTarget.z`          |
| 12  | 4    | float32 | `confidence` | `CVTarget.confidence` |
| 16  | 1    | uint8   | `flags`      | packed, see below     |

`flags` bit0 = `lead_applied`, bit1 = `track_valid`, bits 2-7 reserved and
sent as 0. `x/y/z` is a root-frame position in metres (REP-103: x forward,
y left, z up) that Type-C aims at directly, applying its own gravity, drag
and muzzle geometry. It is not a camera-frame offset and not a barrel
attitude. `header` does not cross the wire, so the MCB has no detection
timestamp and cannot age the point itself; staleness is the sender's problem.
See the warning at the top of this file for the two layout changes this
struct has been through.

#### RELOCALIZE (id=4) — lidar position fix, 8-byte payload, 17-byte frame

Subscribed on `~/relocalize` (`geometry_msgs/msg/Point`, depth-10 reliable
QoS). `z` is discarded.

| Off | Size | Type    | Field       | From      |
|-----|------|---------|-------------|-----------|
| 0   | 4    | float32 | `expectedX` | `Point.x` |
| 4   | 4    | float32 | `expectedY` | `Point.y` |

The lidar-estimated position the MCB should adopt as its odometry origin,
same frame and units as POSE_MSG's `x/y`. Each one is destructive to MCB
odometry, so the node logs every transmission at INFO with the last received
`~/pose` beside the new coordinate.

### DJI UART frame layout (dji_protocol.hpp)

Packed struct definitions mirror the wire layout used by the MCB
(`JetsonSubsystem.hpp` / `type_c_serial_test.hpp`). All multi-byte fields are
little-endian, matching the ARM Cortex-M running modm.

Frame layout (all offsets in bytes):

```
[0]      0xA5         frame head
[1-2]    dataLength   payload byte count (uint16_t LE)
[3]      seq          rolling sequence counter
[4]      crc8         CRC-8 over bytes [0..3]
[5-6]    msgType      message ID (uint16_t LE)  ← enum McbMsgType
[7..N]   payload      dataLength raw bytes
[N+1,2]  crc16        CRC-16 over bytes [0..N]  (uint16_t LE)
```

### REF_SYS_MSG booleans bit layout (dji_protocol.hpp)

`RefSysMsgPayload` (id=3, sent at ~5 Hz by the MCB, interleaved with
POSE_MSG, mirroring `struct RefSysMsg` in `JetsonSubsystem.hpp`) packs 8
flags into one byte, MSB first:

```
bit 7 : isOnBlueTeam
bit 6 : isHealing
bit 5 : isInReloadZone   (restoration OR exchange zone RFID)
bit 4 : isInCenterZone   (central buff RFID)
bit 3 : teamOccupiesCenter
bit 2 : opponentOccupiesCenter
bit 1 : chassisHasPower
bit 0 : gimbalHasPower
```

### Diagnostic stats decision table (config/dji_bridge_params.yaml)

`diag_interval_s` controls how often (seconds) the diagnostic stats summary
is printed to the console: bytes received, valid frames decoded, CRC error
counts, and published/subscribed message counts. Set to 0 to disable.

What you'll see in each case:

| Observation                        | Likely cause                                            |
|-------------------------------------|----------------------------------------------------------|
| bytes=0, frames=0                   | nothing arriving at all (cable? power? baud?)             |
| bytes>0, frames=0                   | bytes arriving but no valid frames (baud mismatch? CRC errors? wrong frame format?) |
| bytes>0, frames>0, pose=0           | frames decoded but pose not publishing (unexpected msgType from MCB?) |
| bytes>0, frames>0, pose>0           | everything healthy                                        |

### Usage

```bash
ros2 launch dji_serial_bridge dji_bridge.launch.py device:=/dev/ttyUSB0 baudrate:=115200

python3 scripts/test_bridge.py                       # config + live check, 10s timeout
python3 scripts/test_bridge.py --config-only         # config check, no ROS
python3 scripts/test_bridge.py --timeout 30          # slow MCB startup
python3 scripts/test_bridge.py --device /dev/ttyUSB0 --baudrate 115200
```
