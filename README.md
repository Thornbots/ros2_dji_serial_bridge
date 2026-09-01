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

The node handles all five message types defined in JetsonSubsystem.hpp:

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

**CV_MSG (id=1) wire format history**: `CVTarget`/`CVDataPayload` first
dropped `v_x/v_y/v_z`/`a_x/a_y/a_z` (2026-07-28, 40 -> 16 bytes,
`confidence` moved from byte offset 36 to offset 12). Then `x/y/z`'s
meaning changed from a camera-frame offset to a **ROOT-FRAME POSITION**
(Type-C aims at this point directly and applies its own
gravity/drag/muzzle geometry; it is never a barrel attitude) and appended
a `flags` byte (bit0 `lead_applied`: does x/y/z include the intercept/lead
solve; bit1 `track_valid`: is it backed by a converged `target_tracker`
estimate, or an unfiltered raw panel position), growing `CVDataPayload` to
17 bytes. Still no velocity/spin fields on the wire by design: those stay
ROS-internal on `sentry_pkg`'s `/cv/target_state` (`TargetState.msg`).
Both changes are breaking changes to the UART packet the MCB's firmware
parses. The corresponding firmware-side struct (mirrored in the MCB's own
`JetsonSubsystem.hpp`, outside this repo) must be updated to match before
real-hardware CV aiming works again; until then, a firmware built against
either older layout will misparse this frame, or parse the bytes correctly
while still aiming at the wrong point.

ROS parameters (see `config/dji_bridge_params.yaml` for defaults):

- `device` (string) : serial device path, e.g. `/dev/ttyTHS1`
- `baudrate` (int) : baud rate in bits-per-second, e.g. 115200
- `read_poll_ms` (int) : poll() timeout in milliseconds (10 is fine)
- `enforce_crc` (bool) : drop frames whose CRC does not match (default true)
- `diag_interval_s` (int) : how often to print diagnostic stats (default 5)
- `debug_log` (bool) : log everything if true

The node has no opinion on the other ends of these topics. Upstream
producers and consumers (sentry_pkg's mcb_relay, the CV pipeline, etc.)
publish or subscribe directly, remapped as needed.

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

### test_bridge.py usage examples

```
# Config + live check (default timeout 10 s):
python3 scripts/test_bridge.py

# Override device and baud:
python3 scripts/test_bridge.py --device /dev/ttyUSB0 --baudrate 115200

# Config check only (no ROS):
python3 scripts/test_bridge.py --config-only

# Longer wait for slow MCB startup:
python3 scripts/test_bridge.py --timeout 30
```

### dji_bridge.launch.py usage example

```
ros2 launch dji_serial_bridge dji_bridge.launch.py device:=/dev/ttyUSB0 baudrate:=115200
```
