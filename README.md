# ros2_dji_serial_bridge

ROS 2 node bridging the DJI-framed UART protocol spoken by the MCB (main
control board) with ROS 2 topics.

**Wire formats: [`UART_PROTOCOL.md`](UART_PROTOCOL.md)** — frame layout, all
five message IDs, byte tables both directions, REF_SYS bits, POSE_MSG odom
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

Checked against `Thornbots/MCBV3` at `708b8d6` (newMain's head on
2026-09-30; branch `HitTarget-HitRing-Separation-Fixed2` is newer but only
moves the hit ring's source) by `sim`'s MCB emulator, which ports
it and runs it against this node on a pty (`../sim/README.md` "MCB
emulator"). Paths are under `MCB-project/src/`. Each line is a firmware-side
fix before the match test's E2 can score:

1. **`CV_MSG` is refused.** `CVData` is 40 bytes (x, y, z, v, a,
   confidence; `subsystems/jetson/JetsonSubsystem.hpp:60-73`), ours 23, and
   `getMsg` drops any size mismatch (`JetsonSubsystem.hpp:204`). The gimbal
   never sees a target.
2. **`x/y/z` is a camera-frame point there**: x right, y up, z forward, plus
   the camera offsets, turned to the world by its own IMU yaw and pitch
   (`JetsonSubsystem.cpp:186-233`, offsets `JetsonSubsystemConstants.hpp:44-46`).
   We send an `odom` point.
3. **No `stamp_ms`, `delay_ms`, `flags` or `fire`.** It fires by its own
   rule: from the first frame within 60 deg of the gun (`JetsonSubsystem.cpp:268`)
   at indexer rate 10 (`AutoAimAndFireCommand.cpp:112`) until it patrols.
4. **It leads the target itself**, ballistics at 24 m/s on its own velocity
   estimate (`JetsonSubsystem.cpp:236-242`, `JetsonSubsystemConstants.hpp:49`).
   With `lead_applied` points it would lead twice.
5. **`RELOCALIZE` is refused**: `Relocalize` is 12 bytes, with an `expectedZ`
   (`JetsonSubsystem.hpp:53-58`); ours 8. Accepted, it would still not
   overwrite odometry (`JetsonSubsystem.cpp:119` is commented out):
   `SimpleAutoDriveCommand` applies it only at full HP in a resupply zone,
   offset by ±0.688, -0.05 m (`subsystems/drivetrain/SimpleAutoDriveCommand.hpp:91-95`).
6. **`POSE_MSG` is 90 Hz and `REF_SYS_MSG` 10 Hz**, not 100 and 5: nine
   poses then one ref on one 10 ms timer (`JetsonSubsystem.cpp:41-80`); the
   200 ms ref timer (`JetsonSubsystem.hpp:141-142`) is unused.
7. **`POSE_MSG` x/y is x right, y forward** of the heading at power-on
   (`SimpleAutoDriveCommand.hpp:188-189`, `DrivetrainDriveCommand.cpp:40-41`),
   not REP-105's x forward, y left. `pose_translator` reads it as REP-105.
8. **`head_yaw` is `[0, 2pi)`, zero at IMU boot** (`MahonyAHRS.h:75-78`
   in taproot, via `GimbalSubsystem.cpp:41`), and counter-clockwise if the
   firmware's own aim math is self-consistent (`JetsonSubsystem.cpp:92`
   against `:255`). Our URDF turns `headlink` about -z. Unverified on the
   robot: check the sign before trusting `root->camera`.
9. **`odomStatus` is always `ODOM_PODS`** (`JetsonSubsystem.cpp:53`).
10. **`deltaAngleGotHitIn` is 123 when not hit**, `HitRing::PLACEHOLDER_ANGLE`
    (`subsystems/ui/objects/HitRing.hpp:99`), not documented here.
11. **One mailbox slot.** Each frame overwrites the last
    (`communication/UARTCommunication.cpp:37-44`, the TODO at `.hpp:61`), so a
    `RELOCALIZE` landing in the same 1 ms cycle as a `CV_MSG` is lost.
12. **`ROS_MSG` has no reader on the sentry.** Only `AutoDriveCommand` reads
    it, and the sentry's switch schedules `SimpleAutoDriveCommand`
    (`robots/sentry/SentryControl.hpp:61`, `:191-192`), a fixed waypoint route.
13. **`seq` is always 0** on frames it sends (`UARTCommunication.cpp:21`).

Proposed, not applied: **`POSE_MSG` (id=2) chassis yaw** (2026-09-29), two
trailing floats taking it 25 → 33 bytes, in `UART_PROTOCOL.md`. Until both
sides agree, `RobotPose.chassis_yaw` and `chassis_yaw_rate` read 0.

## Topics

Five, one per message ID: `~/nav_goal`, `~/cv_target`, `~/pose`, `~/ref_sys`,
`~/relocalize`. Types and directions are in the summary table in
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
