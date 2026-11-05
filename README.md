# ros2_dji_serial_bridge

ROS 2 node bridging the DJI-framed UART protocol spoken by the MCB (main
control board) with ROS 2 topics.

**Wire formats: [`UART_PROTOCOL.md`](UART_PROTOCOL.md)** — frame layout, all
eight message IDs, byte tables both directions, REF_SYS bits, POSE odom
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

MMM uses the MCB-owned states and exchanges in the
[PING contract](UART_PROTOCOL.md#ping-id5--mcb-mapping-machine-and-link-check).
Firmware and the sim MCB emulator must implement that contract together,
including Setup→Done on 6 and Done→Mapping on 4 after a failed Ready save.
The bridge only translates PING; the in-stack
[`mmm` node](../thornbots_pkg/README.md#mcb-mapping-machine) owns Jetson work.
ACK of save is immediate receipt and cannot guarantee durable storage. Verify
firmware/emulator agreement before physical use; MCB power-off can interrupt a
save if it also removes Jetson power.

Raw aim coordinates, a separate translation-only navigation offset and the
pitch-pivot correction are implemented in the workspace-pinned MCBV3. Deploy
firmware and ROS together; physical acceptance is tracked in the [ROADMAP](../docs/ROADMAP.md#later-needs-a-robot).

The MCB plans aim and fire from `TARGET_STATE` and takes idle looks from the
13-byte `CV_TARGET`; hit-turn permission and the firmware-side layout are in
the [CV_TARGET](UART_PROTOCOL.md#cv_target-id1--idle-look-point) and
[TARGET_STATE](UART_PROTOCOL.md#target_state-id7--tracked-enemy-armor-model)
contracts. Both layouts change together with the firmware (13 and 80 bytes; the
former 15-byte `CV_TARGET` and its `fire`/`delay_ms`/`patrol` fields are gone).
`mcb_relay` sets `type_c_based_patrol` on both messages. The
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

- `CV_TARGET` (13 bytes, idle look) and `TARGET_STATE` (80 bytes) follow the
  [contracts](UART_PROTOCOL.md#cv_target-id1--idle-look-point); the MCB owns
  planning, fire lock, gates and shot timing
  ([MCBV3 aiming](../firmware/MCBV3/README.md#aim-and-navigation-odometry)).
  The firmware pin and sim must move to the matching MCBV3 commit with this
  change; the 053285f review above predates both payloads.
- Bit 2 (`turn_to_hit`) of `CV_TARGET` permits a 500 ms hit turn only while no
  target state is held; clearing it cancels that turn.
- POSE position, velocity and head yaw use the raw aim frame
  ([message types](UART_PROTOCOL.md#message-types)). `mcb_x_right` is removed.
  `odom_status` reports `ODOM_PODS` or a stale-Pico `ODOM_I2C_DEAD`
  ([codes](UART_PROTOCOL.md#pose-id2--chassis-pose-and-gimbal-angles)).
- REF_SYS hit angles follow [UART_PROTOCOL](UART_PROTOCOL.md#ref_sys-id3--referee-system-status).
  `util/HitTracker.hpp` keeps IMU yaw in radians; firmware and hosted UART
  tests cover nonzero head and chassis yaw. It retains the latest world hit
  bearing until a complete REF_SYS write succeeds, independently of the
  one-cycle event used by auto-aim/UI. Physical hit direction remains
  unvalidated.
- The UART receiver has one mailbox slot: a later frame can overwrite an
  unread RELOCALIZE (`communication/UARTCommunication.cpp`). Measure loss
  with CV_TARGET and TARGET_STATE traffic before relying on corrections while moving.
- The normal sentry switch schedules the fixed-waypoint
  `SimpleAutoDriveCommand`, not the NAV_GOAL reader `AutoDriveCommand`
  (`robots/sentry/SentryControl.hpp`). Sim's `mcb.launch.py drive:=auto`
  selects that reader explicitly; the match stages use scripted sim routes.
- Outgoing UART sequence numbers remain zero (`UARTCommunication.cpp`).
  Chassis yaw is absent from POSE, so the bridge publishes zero for its
  proposed chassis-yaw fields.

### Aim and navigation frames

POSE, CV_TARGET and TARGET_STATE use continuous raw aim odometry; NAV_GOAL and RELOCALIZE
use separately corrected navigation odometry. The exact frame roles are owned by
[UART_PROTOCOL](UART_PROTOCOL.md#message-types). Deploy the matching
firmware and ROS stack together; the unchanged payload length cannot detect
an axis mismatch. The physical team's start pose remains unmeasured on the field.

Jetson camera/target/aim transforms use raw MCB POSE, independently of the
EKF and map localization, so navigation corrections cannot move gimbal aim.
Raw drift and UART/sample timing still affect aim. The single-slot mailbox
can lose a RELOCALIZE arriving alongside TARGET_STATE or CV_TARGET; measure that and the
UART/read delays on hardware.

`root` is heading-fixed and gimbal yaw is world yaw, so the existing aim does
not require chassis yaw. The [POSE chassis-yaw proposal](UART_PROTOCOL.md#proposed-pose-chassis-yaw)
remains unapplied. The bridge stamp contract is in
[MCB to Jetson](UART_PROTOCOL.md#mcb--jetson); hardware measurements and
moving-shooter acceptance are in [ROADMAP track B](../docs/ROADMAP.md#b-hit-while-we-move).

## Topics

One per message ID, two for `PING`: `~/nav_goal`, `~/cv_target`, `~/target_state`, `~/pose`,
`~/ref_sys`, `~/debug`, `~/relocalize`, `~/ping_to_mcb`, `~/ping_from_mcb`. Types and directions are in the
[summary table](UART_PROTOCOL.md#message-types). They live in the node's private namespace, so `~/nav_goal`
is `/dji_serial_bridge/nav_goal` until a launch file remaps it.

The node has no opinion on the other end of any of them. `thornbots_pkg`'s
`mcb_relay` and the CV pipeline connect directly, remapped as needed.

`~/cv_target` carries idle looks only, while the Jetson holds no fresh target;
`~/target_state` carries the tracker's armor model on every update, with its two
ages set at the write. The MCB decides aim, fire and detection-age gating (see
[TARGET_STATE](UART_PROTOCOL.md#target_state-id7--tracked-enemy-armor-model)).
Both expire at the bridge write boundary; see
[Jetson to MCB freshness](UART_PROTOCOL.md#jetson--mcb-freshness) for clock
requirements, rejected stamps (including `detection_stamp`) and serial queuing
limits.

`~/debug` publishes the firmware's opaque `JetsonSubsystem::sendDebug` bytes as
`DebugBlob` with a send-start stamp and empty frame ID, untouched; decode
offline against the producing firmware. The startup recorder includes it in
normal and match bags. Payload limits, loss behavior and link budget:
[DEBUG contract](UART_PROTOCOL.md#debug-id6--opaque-firmware-debug). No debug
producer is enabled in production firmware.

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

The text DIAG summary retains lifetime byte, frame, CRC and per-topic counters
every `diag_interval_s` (0 disables text only). REF_SYS and CRC warning samples
log immediately on the first occurrence, then at most once
per five seconds per call site, using a steady clock. Counters include suppressed
samples. RELOCALIZE and PING log every event; `debug_log=false` disables
REF_SYS and PING logs. Pose logs
its first message and odometry-source transitions immediately.

`diagnostic_updater` publishes `diagnostic_msgs/DiagnosticArray` on
`/diagnostics`, independently of text logging, every
`diagnostic_updater.period` seconds (default 1). The `Serial link` status
uses the device path as hardware ID and includes every DIAG counter, including
`silent_polls` and `debug_msgs_pub`. Its header stamp is the time the health observation was made.
Levels use changes since the previous update, so a formerly healthy link can
become unhealthy:

| Level | Observation during the interval | Likely cause |
| --- | --- | --- |
| ERROR | No new bytes | cable, power, or stopped sender |
| ERROR | New bytes but no new valid frames | baud, framing or CRC mismatch |
| WARN | Valid frames arrive, but CRC failures increase | noisy or corrupt link |
| OK | Valid frames arrive without new CRC failures | healthy transport |

The first update compares against zero. Missing pose traffic alone does not
change transport health: inspect `pose_msgs_pub` for unexpected MCB message
types. Historical CRC failures remain in the counters after health recovers.
Inspect with `ros2 topic echo /diagnostics` or `rqt_robot_monitor`.

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
and PTY transport gtests, the live PTY bridge-health test, plus ament lint.
The health test injects valid and corrupt frames, checks exact counters and
throttled logs, and verifies health recovery and stalls on `/diagnostics`. The parser fixtures preserve the
former Python diagnostic's bytes and printed error lines. No hardware or
running ROS graph is needed for these unit tests.

### Simulation capture metadata

The bridge offers an optional coordination interface for simulator PTY input.
Set `sim_frame_metadata:=true` together with `use_sim_time:=true` and
`enforce_crc:=true`; the default is disabled. Publish reliable
`~/sim_frame_metadata` (`SimFrameMetadata`) in serial-frame order, with a
nonzero, strictly increasing session identity, the capture time of the state
encoded in `header.stamp`, its complete-frame delivery time in `delivery_time`,
and the entire original UART frame including CRCs.
The bridge matches complete CRC-valid frames and metadata in order, independent
of which arrives first. Matching runs on serial, metadata and `/clock` events;
the bridge waits for its observed clock to reach the delivery time and fails
if the clock overshoots or goes backwards. It publishes the translated message with that capture
stamp, then emits reliable `~/sim_frame_ack` (`SimFrameAck`): `published=true`
means the ROS publish call returned, not that a subscriber received it. The
ack header is the observed delivery time, and its identity matches the
metadata. `~/sim_clock_ready` first reports zero when the bridge's clock
subscription matches a publisher, then reports each observed nonzero ROS time
for simulator startup synchronization. `~/sim_clock_observed` instead reports
each clock sample actually received, including zero, with transient-local
durability. A zero on `sim_clock_ready` alone is not evidence of receipt.
This interface does not change physical UART bytes.

Both pending queues hold at most 32 frames. A mismatch, unsupported frame,
invalid or out-of-order metadata, overflow, or unmatched item after
`sim_frame_timeout_ms` wall milliseconds (default 1000) permanently fails the
session until bridge restart. A failure ack has `published=false`, a reason,
and the metadata identity when known (zero when unknown); the serial-link
diagnostic also reports ERROR. No further frames are published in that
session. The timeout uses wall time even when simulation is paused. Producers
must detect missing acknowledgments too, since reliable ROS delivery does not
prove a subscriber was present. Simulator integration is still pending.
