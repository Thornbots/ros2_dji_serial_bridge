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

Read against `Thornbots/MCBV3` branch `uart-names-from-ros-topics` at
`47512cc` (Thornbots/MCBV3#74, open), which already has our message names and
layouts. Paths are under `MCB-project/src/`. Each line is a firmware-side fix
before the match test's E2 can score:

1. **`CV_TARGET` is refused.** Its `CvTarget` there still leads with a
   `uint32_t stamp_ms`: 19 bytes, ours 15, and `getMsg` drops any size
   mismatch (`subsystems/jetson/JetsonSubsystem.hpp`). MCBV3
   `position-based-cv` has the 15-byte struct (x, y, z, delay_ms, flags).
2. **It never aims or fires on `CV_TARGET`.** `JetsonSubsystem::update` only
   takes the frame off the one-slot mailbox; `AutoAimAndFireCommand` patrols.
   The old camera-frame solve, its own lead and its own fire rule are gone
   from this branch, so `x/y/z` (an `odom` point) and `delay_ms`/`flags` have
   no reader yet.
3. **`RELOCALIZE` is accepted but does not overwrite odometry.**
   `checkApplyRelocalize` has `odo->relocalizeTo` commented out;
   `SimpleAutoDriveCommand` applies it only at full HP in a resupply zone,
   offset by +-0.688, -0.05 m (`subsystems/drivetrain/SimpleAutoDriveCommand.hpp:91-95`).
4. **`POSE` is 90 Hz and `REF_SYS` 10 Hz**, not 100 and 5: nine
   poses then one ref on one 10 ms timer (`JetsonSubsystem.cpp:22-60`); the
   200 ms ref timer (`JetsonSubsystem.hpp:131-132`) is unused.
5. **`POSE` x/y is x right, y forward** of the heading at power-on
   (`SimpleAutoDriveCommand.hpp:188-189`), not REP-105's x forward, y left.
   `pose_translator` reads it as REP-105.
6. **`head_yaw` is `[0, 2pi)`, zero at IMU boot** (`MahonyAHRS.h:75-78`
   in taproot, via `GimbalSubsystem.cpp:41`), and counter-clockwise.
   Confirmed on the sentry 2026-10-03 (bag run00029: a hand turn CCW raised
   it). Our URDF turns `headlink` about +z to match; it turned about -z
   before that date, which mirrored `root->camera`.
7. **`odom_status` is always `ODOM_PODS`** (`JetsonSubsystem.cpp:34`).
8. **`delta_angle_got_hit_in` is 123 when not hit**, `HitRing::PLACEHOLDER_ANGLE`
   (`subsystems/ui/objects/HitRing.hpp:99`), not documented here.
9. **One mailbox slot.** Each frame overwrites the last
   (`communication/UARTCommunication.cpp:37-44`, the TODO at `.hpp:61`), so a
   `RELOCALIZE` landing in the same 1 ms cycle as a `CV_TARGET` is lost.
10. **`NAV_GOAL` has no reader on the sentry.** Only `AutoDriveCommand` reads
    it, and the sentry's switch schedules `SimpleAutoDriveCommand`
    (`robots/sentry/SentryControl.hpp:61`, `:191-192`), a fixed waypoint route.
11. **`seq` is always 0** on frames it sends (`UARTCommunication.cpp:21`).
12. **`BYTE` (id=5) is not in the firmware yet** (added here 2026-10-03): it
    needs `struct McbByte { uint8_t data; } modm_packed;` and a
    `StructToMessageType` entry in `JetsonSubsystem.hpp`.

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
