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

Raw aim coordinates, a separate translation-only navigation offset and the
pitch-pivot correction are implemented in the workspace-pinned MCBV3. Deploy
firmware and ROS together; physical acceptance is tracked in [hardware status](../docs/JAZZY_FLASH.md#hardware-status).
The earlier asks against `position-based-cv` `0885a69` describe old firmware,
not the current integration.

Hit-turn permission follows estimate availability on each Jetson frame; see
[firmware status](#where-the-firmware-stands). The informational patrol-point
flag at bit 3 is implemented; firmware currently ignores it. It does not gate firing. The
[POSE chassis-yaw proposal](UART_PROTOCOL.md#proposed-pose-chassis-yaw) also
remains unapplied; current heading-fixed aiming does not require it.

### Where the firmware stands

Reviewed against [MCBV3 053285f](https://github.com/Thornbots/MCBV3/commit/053285f)
on 2026-10-14. Sim compiles
that checkout's C++ control code; it no longer
ports a historical revision in Python. See [MCB emulator](../sim/README.md#mcb-emulator).
Sim's [MCB backend](../sim/README.md#mcb-emulator) owns physical motor feedback
and fidelity limits. Its [firmware auto-drive stages](../sim/README.md#driving-and-combat-diagnostics-mcb_drive)
select commands through DBUS. The [Type C backend](../sim/README.md#type-c-board-execution-with-renode)
uses the hardware ELF's USART driver with this same bridge and wire contract.
The private hardware pipe leaves every Jetson UART payload unchanged.
Automatic indexer recovery after calibration/pre-game state is maintained in
[MCBV3's design](../firmware/MCBV3/README.md#automatic-indexer-recovery).
Paths below are under `MCB-project/src/` in MCBV3.

- `CV_TARGET` is the current 15-byte payload. Aim subtracts raw aim-frame
  odometry from x/y and subtracts the pitch-pivot height from ground-frame z
  before solving ballistics (`subsystems/jetson/AutoAimAndFireCommand.cpp`).
  It holds a received aim for 200 ms. Bit 0 permits an indexer request after
  `max(0, delay_ms - FIRING_LATENCY_TIME)`; the wire delay includes
  indexer latency and alignment wait ([contract](UART_PROTOCOL.md#cv_target-id1--aim-point-and-fire-decision)).
  Shorter delays execute on the
  next control cycle. The pending shot is restarted by each new aim frame.
- Bit 1 permits firmware patrol when an aim expires; bit 2 permits a 500 ms
  hit turn, including interruption of a patrol-point aim. Clearing bit 2 cancels
  that turn. The Jetson clears bit 2 while holding any target estimate,
  including coasting/predicted or unconverged tracks, and sets it with no
  estimate, including patrol. It decides on every frame with no cooldown. Bit 3
  marks Jetson patrol points without changing these gates; the compiled
  [sim firmware tests](../sim/README.md#mcb-emulator) exercise it clear/set.
- POSE position, velocity and head yaw use the
  raw aim frame described in
  [UART_PROTOCOL message types](UART_PROTOCOL.md#message-types).
  `mcb_x_right` is removed.
  `JetsonSubsystem.cpp` sends nine POSE frames then one REF_SYS on its 10 ms
  timer (nominally 90 Hz and 10 Hz). `odom_status` still reports `ODOM_PODS`.
- REF_SYS hit angles use radians relative to the current chassis heading,
  positive CCW, as specified in [UART_PROTOCOL](UART_PROTOCOL.md#ref_sys-id3--referee-system-status).
  `util/hitTracker.hpp` keeps IMU yaw in radians; firmware and hosted UART
  tests cover nonzero head and chassis yaw. It retains the latest world hit
  bearing until a complete REF_SYS write succeeds, independently of the
  one-cycle event used by auto-aim/UI. A report uses the current chassis
  heading; the next report carries 123 unless another hit occurred.
  Physical hit direction remains unvalidated.
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

### Aim and navigation frames

POSE and CV_TARGET use continuous raw aim odometry; NAV_GOAL and RELOCALIZE
use separately corrected navigation odometry. RELOCALIZE updates only the
navigation offset and does not reanchor raw aim. The exact frame roles are
owned by
[UART_PROTOCOL](UART_PROTOCOL.md#message-types). Deploy the matching
firmware and ROS stack together; the unchanged payload length cannot detect
an axis mismatch. The team's start pose, including red (-4.625, 0), remains
unmeasured on the field.

Jetson camera/target/aim transforms use raw MCB POSE, independently of the
EKF and map localization. RELOCALIZE updates only the MCB navigation offset;
gimbal aim subtracts raw odometry. Raw drift and UART/sample timing still
affect aim, but navigation corrections cannot move it. The current
1 ms mailbox can lose a RELOCALIZE arriving alongside CV_TARGET; measure
that and the UART/read delays on hardware.

`root` is heading-fixed and gimbal yaw is world yaw, so the existing aim does
not require chassis yaw. The [POSE chassis-yaw proposal](UART_PROTOCOL.md#proposed-pose-chassis-yaw)
remains unapplied. The bridge stamp contract is in
[MCB to Jetson](UART_PROTOCOL.md#mcb--jetson); hardware measurements and
moving-shooter acceptance are in [ROADMAP track B](../docs/ROADMAP.md#b-hit-while-we-move).

## Topics

One per message ID, two for `PING`: `~/nav_goal`, `~/cv_target`, `~/pose`,
`~/ref_sys`, `~/relocalize`, `~/ping_to_mcb`, `~/ping_from_mcb`. Types and directions are in the summary table in
`UART_PROTOCOL.md`. They live in the node's private namespace, so `~/nav_goal`
is `/dji_serial_bridge/nav_goal` until a launch file remaps it.

The node has no opinion on the other end of any of them. `thornbots_pkg`'s
`mcb_relay` and the CV pipeline connect directly, remapped as needed.

CV decisions expire at the bridge write boundary; see the
[decision freshness policy](UART_PROTOCOL.md#cv-decision-freshness) for clock
requirements, rejected stamps and serial queuing limits.

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
- `diag_interval_s` (int) : seconds between diagnostic summaries; 0 disables, negatives are rejected
- `debug_log` (bool) : log each REF_SYS RX and each PING RX/TX at INFO (default true)

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
`RELOCALIZE` to (5, 3) every second, which moves the MCB's navigation position:
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
