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

### Where the firmware stands

Read against `Thornbots/MCBV3` branch `position-based-cv` at `0885a69`
(2026-10-03), the branch the sentry runs. It has our message names and
layouts. Paths are under `MCB-project/src/`. Each line is a firmware-side
fix before the match test's E2 can score. (`uart-names-from-ros-topics` at
`47512cc` is older: its `CvTarget` still leads with a `uint32_t stamp_ms`,
19 bytes, and `getMsg` would refuse our 15.)

1. **`id=5` is `PING` there, not `BYTE`.** `struct Ping { uint8_t number; }`
   (`subsystems/jetson/JetsonSubsystem.hpp`); `refresh` echoes any received
   one straight back (`JetsonSubsystem.cpp:22-25`). The layout is our
   `BytePayload`, so a byte sent on `~/byte_to_mcb` comes back on
   `~/byte_from_mcb` unchanged, and the MCB sends no byte of its own. Rename
   it to `McbByte`/`BYTE` there if it should carry anything else.
2. **`CV_TARGET` aims and fires, but in the MCB's odometry frame**
   (`subsystems/jetson/AutoAimAndFireCommand.cpp:54-90`): the aim is
   `x/y/z` minus `odo->getX()/getY()`, the shot goes `delay_ms` after receipt
   less `FIRING_LATENCY_TIME` when flags bit 0 is set. That odometry is not
   the Jetson's `odom` (item 4; AGENTS.md "Open").
3. **`POSE` is 90 Hz and `REF_SYS` 10 Hz**, not 100 and 5: nine
   poses then one ref on one 10 ms timer (`JetsonSubsystem.cpp:27-65`); the
   200 ms ref timer (`JetsonSubsystem.hpp:137-138`) is unused.
4. **`POSE` x/y is x right, y forward** of the heading at power-on
   (`subsystems/drivetrain/SimpleAutoDriveCommand.cpp:108`), not REP-105's x
   forward, y left. `pose_translator` reads it as REP-105.
5. **`head_yaw` is `[0, 2pi)`, zero at IMU boot** (`MahonyAHRS.h:75-78`
   in taproot, via `GimbalSubsystem.cpp:41`), and counter-clockwise.
   Confirmed on the sentry 2026-10-03 (bag run00029: a hand turn CCW raised
   it). Our URDF turns `headlink` about +z to match; it turned about -z
   before that date, which mirrored `root->camera`.
6. **`odom_status` is always `ODOM_PODS`** (`JetsonSubsystem.cpp:39`).
7. **`delta_angle_got_hit_in` is 123 when not hit**, `HitRing::PLACEHOLDER_ANGLE`
   (`subsystems/ui/objects/HitRing.hpp:99`), not documented here.
8. **One mailbox slot.** Each frame overwrites the last
   (`communication/UARTCommunication.cpp:37-44`, the TODO at `.hpp:61`), so a
   `RELOCALIZE` landing in the same 1 ms cycle as a `CV_TARGET` is lost.
9. **`NAV_GOAL` has no reader on the sentry.** Only `AutoDriveCommand` reads
   it, and the sentry's switch schedules `SimpleAutoDriveCommand`
   (`robots/sentry/SentryControl.hpp:61`, `:191-192`), a fixed waypoint route.
10. **`seq` is always 0** on frames it sends (`UARTCommunication.cpp:21`).

`RELOCALIZE` is fine there: `checkApplyRelocalize` calls `odo->relocalizeTo`
with the frame's x/y (`JetsonSubsystem.cpp:79`).

Proposed, not applied: **`POSE` (id=2) chassis yaw** (2026-09-29), two
trailing floats taking it 25 → 33 bytes, in `UART_PROTOCOL.md`. Until both
sides agree, `RobotPose.chassis_yaw` and `chassis_yaw_rate` read 0.

## Topics

One per message ID, two for `BYTE`: `~/nav_goal`, `~/cv_target`, `~/pose`,
`~/ref_sys`, `~/relocalize`, `~/byte_to_mcb`, `~/byte_from_mcb`. Types and directions are in the summary table in
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

python3 scripts/test_bridge.py                       # config + live check, 10s timeout
python3 scripts/test_bridge.py --config-only         # config check, no ROS
python3 scripts/test_bridge.py --timeout 30          # slow MCB startup
python3 scripts/test_bridge.py --device /dev/ttyUSB0 --baudrate 115200
```
