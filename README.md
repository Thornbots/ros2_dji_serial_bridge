# ros2_dji_serial_bridge

ROS 2 node bridging the DJI-framed UART protocol spoken by the MCB (main
control board) with ROS 2 topics.

**Wire formats: [`UART_PROTOCOL.md`](UART_PROTOCOL.md)** — frame layout, all
six message IDs, byte tables both directions, REF_SYS bits, POSE odom
status codes. Read it before touching `dji_protocol.hpp` or any `msg/` file
that crosses the link.

## MCB firmware coordination

Every payload struct here is mirrored by hand in the firmware's
`JetsonSubsystem.hpp`, in a different repo. The MCB casts received bytes into
its copy, so a layout the two sides disagree on is a silent break: a size
mismatch fails the receiver's length check and the topic simply stops, and a
field that changes meaning under a stable layout is not caught at all. Every
wire change is two commits in two repos, landed together.

### Asked of the firmware

The shared field frame and pitch-pivot correction are implemented in the
workspace-pinned MCBV3 `nightly`. Deploy firmware and ROS together; physical
acceptance is tracked in [hardware status](../JAZZY_FLASH.md#hardware-status).
The earlier asks against `position-based-cv` `0885a69` describe old firmware,
not the current integration.

Remaining CV coordination is in [ROADMAP short todos](../ROADMAP.md#short-todos):
T31 gates hit turns while tracking, and T33 proposes a patrol-point flag at
reserved bit 3. Neither changes the wire contract yet. The
[POSE chassis-yaw proposal](UART_PROTOCOL.md#proposed-pose-chassis-yaw) also
remains unapplied; current heading-fixed aiming does not require it.

### Where the firmware stands

Reviewed against pinned [MCBV3 a2ea519](https://github.com/Thornbots/MCBV3/commit/a2ea519)
on 2026-10-08. Sim compiles that checkout's C++ control code; it no longer
ports a historical revision in Python. See [MCB emulator](../sim/README.md#mcb-emulator).
Paths below are under `MCB-project/src/` in MCBV3.

- `CV_TARGET` is the current 15-byte payload. Aim subtracts field-frame
  odometry from x/y and subtracts the pitch-pivot height from ground-frame z
  before solving ballistics (`subsystems/jetson/AutoAimAndFireCommand.cpp`).
  It holds a received aim for 200 ms. Bit 0 permits an indexer request after
  `delay_ms - FIRING_LATENCY_TIME` (80 ms); shorter delays execute on the
  next control cycle. The pending shot is restarted by each new aim frame.
- Bit 1 permits firmware patrol when an aim expires; bit 2 permits a 500 ms
  hit turn, including interruption of a live aim. Clearing bit 2 cancels
  that turn. The Jetson's per-frame decision and patrol marker remain open.
- POSE position, velocity and head yaw use the
  [shared field frame](#shared-aim-frame). `mcb_x_right` is removed.
  `JetsonSubsystem.cpp` sends nine POSE frames then one REF_SYS on its 10 ms
  timer (nominally 90 Hz and 10 Hz). `odom_status` still reports `ODOM_PODS`.
- The hit-angle implementation does not yet establish the relative-angle,
  since-last-frame contract in [UART_PROTOCOL](UART_PROTOCOL.md#ref_sys-id3--referee-system-status).
  `util/hitTracker.hpp` scales radian IMU yaw by `PI/180`, and resets `isHit`
  every control cycle. `JetsonSubsystem.cpp` samples it for REF_SYS at 10 Hz,
  so a hit between samples can be missed. T31 includes unit/frame agreement
  and latching; physical hit direction remains unvalidated.
- The UART receiver has one mailbox slot: a later frame can overwrite an
  unread RELOCALIZE (`communication/UARTCommunication.cpp`). Measure loss
  with CV_TARGET traffic before relying on corrections while moving.
- The normal sentry switch schedules the fixed-waypoint
  `SimpleAutoDriveCommand`, not the NAV_GOAL reader `AutoDriveCommand`
  (`robots/sentry/SentryControl.hpp`). Sim's `mcb.launch.py drive:=auto`
  selects that reader explicitly; the match stages use scripted sim routes.
- Outgoing UART sequence numbers remain zero (`UARTCommunication.cpp`).
  Chassis yaw is absent from POSE, so the bridge publishes zero for its
  proposed chassis-yaw fields.

### Shared aim frame

Nightly uses the field frame on both sides: POSE, RELOCALIZE and CV_TARGET
are centred on the field, with x toward blue's base. Deploy the matching
firmware and ROS stack together; the unchanged payload length cannot detect
an axis mismatch. The team's start pose, including red (-4.625, 0), remains
unmeasured on the field.

The Jetson aims in localization's EKF-fused `odom`. RELOCALIZE moves the MCB's
raw odometry toward that frame; an in-flight aim remains in the shared frame.
The residual is MCB drift since the last accepted correction. The current
1 ms mailbox can lose a RELOCALIZE arriving alongside CV_TARGET; measure
that and the UART/read delays on hardware.

`root` is heading-fixed and gimbal yaw is world yaw, so the existing aim does
not require chassis yaw. The [POSE chassis-yaw proposal](UART_PROTOCOL.md#proposed-pose-chassis-yaw)
remains unapplied. The bridge stamp contract is in
[MCB to Jetson](UART_PROTOCOL.md#mcb--jetson); hardware measurements and
moving-shooter acceptance are in [ROADMAP track B](../ROADMAP.md#b-hit-while-we-move).

## Topics

One per message ID, two for `PING`: `~/nav_goal`, `~/cv_target`, `~/pose`,
`~/ref_sys`, `~/relocalize`, `~/ping_to_mcb`, `~/ping_from_mcb`. Types and directions are in the summary table in
`UART_PROTOCOL.md`. They live in the node's private namespace, so `~/nav_goal`
is `/dji_serial_bridge/nav_goal` until a launch file remaps it.

The node has no opinion on the other end of any of them. `thornbots_pkg`'s
`mcb_relay` and the CV pipeline connect directly, remapped as needed.

## Parameters

Defaults live in `config/dji_bridge_params.yaml`, which the launch file loads
under the `/**` wildcard key. Only `device`, `baudrate` and `debug_log` are
launch arguments; the other three change only in the YAML, or via a whole
replacement file with `params_file:=`. `diag_interval_s:=0` on the command
line is silently ignored.

- `device` (string) : serial device path, e.g. `/dev/ttyTHS1`
- `baudrate` (int) : bits per second, e.g. 115200
- `read_poll_ms` (int) : poll() timeout in ms (10 is fine)
- `enforce_crc` (bool) : drop frames failing CRC (default true)
- `diag_interval_s` (int) : seconds between diagnostic summaries, 0 disables
- `debug_log` (bool) : log everything if true

## Diagnostics

Every `diag_interval_s` the node prints bytes received, frames decoded, CRC
error counts and per-topic message counts. Read it as:

| Observation               | Likely cause                                        |
|---------------------------|-----------------------------------------------------|
| bytes=0, frames=0         | nothing arriving (cable? power? baud?)              |
| bytes>0, frames=0         | not framing (baud mismatch, CRC, wrong frame format)|
| bytes>0, frames>0, pose=0 | decoding, but no pose (unexpected msgType from MCB?)|
| bytes>0, frames>0, pose>0 | healthy                                             |

## Usage

The ROS package is `dji_serial_bridge`; the directory is `ros2_dji_serial_bridge`.
`colcon build --packages-select ros2_dji_serial_bridge` selects nothing and
succeeds.

```bash
ros2 launch dji_serial_bridge dji_bridge.launch.py device:=/dev/ttyUSB0 baudrate:=115200

ros2 run dji_serial_bridge test_bridge                       # config + live check, 10s timeout
ros2 run dji_serial_bridge test_bridge --config-only         # config check, no ROS graph
ros2 run dji_serial_bridge test_bridge --timeout 30          # slow MCB startup
ros2 run dji_serial_bridge test_bridge --device /dev/ttyUSB0 --baudrate 115200

ros2 run dji_serial_bridge serial_debug /dev/ttyTHS1 --baud 115200         # raw hex dump, bridge stopped
ros2 run dji_serial_bridge serial_debug /dev/ttyTHS1 --baud 115200 --log rx.bin
```

`test_bridge` waits for `~/pose` and `~/ref_sys` from a running bridge.
`serial_debug` opens the port itself, so stop the bridge first; it
defaults to 921600 baud, so always pass `--baud`. Its `--tx` sends a
`RELOCALIZE` to (5, 3) every second, which moves the MCB's odometry origin:
not on a robot that's driving.

Both diagnostics are C++17. The hex tool intentionally preserves the former
Python diagnostic's CRC16 table (the first 16 entries repeated), which differs
from the production bridge's full DJI table. Its CRC16 verdict and transmitted
test frames therefore retain that existing limitation; raw hex and byte logs
are unaffected.

`colcon test --packages-select dji_serial_bridge` runs the diagnostic parser
and PTY transport gtests plus ament lint. The parser fixtures preserve the
former Python diagnostic's bytes and printed error lines. No hardware or
running ROS graph is needed for these unit tests.
