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

### Asked of the firmware (2026-10-04)

Against `position-based-cv` `0885a69`, for the sentry's first shots. The
2026-10-03 asks were against `f835be1`; `1c2405c` since dropped the aim's
`-PI/2`, so its yaw reads `CV_TARGET` as REP-105 already. Sim's MCB
emulator still ports `f835be1` (`../sim/README.md` "MCB emulator").
Neither is in `0885a69` yet (re-read 2026-10-04).

1. **One frame for the aim: REP-105.** The aim's yaw (`AutoAimAndFireCommand.cpp:70-71`)
   treats `x/y` as x forward, y left, but subtracts `odo->getX()/getY()`,
   which are x right, y forward. Right only at the power-on spot. Subtract
   `(odo->getY(), -odo->getX())` there; `POSE` sends `(y, -x)`, velocity the
   same; `RELOCALIZE` and `NAV_GOAL` `(x, y)` arrive as `(-y, x)`. Don't
   rotate `CV_TARGET`: that would now turn the aim 90 deg. `head_yaw` matches.
   Land both halves in one commit: either alone breaks the aim. Until
   then `thornbots_pkg`'s `mcb_x_right` converts on the Jetson
   (`../thornbots_pkg/README.md` "MCB axes"); the robot image that takes
   the fixed firmware must run with it false, or the aim turns 90 deg.
2. **Pitch for `z` above the pitch pivot** (`:72`): `solveForPitch` gets
   `z - OFFSET_Z_ROBOT_TO_PITCH_PIVOT` (0.39 m). `z` is from the ground, so
   every shot aims 0.39 m high. `Reticle.hpp:329` already subtracts it.

Dropped: clamping `delay_ms - FIRING_LATENCY_TIME` (`:57`, now 80 ms).
Under 80 the `uint32` wraps, but `MilliTimeout` adds it to now in `uint32`
too, so the shot fires on the next cycle, as a clamp would.

Keep: firing on bit 0 alone, one pending shot that each fire frame
restarts, aiming for 200 ms after the last frame, patrolling only on bit 1.

### MCBV3 `rep-105` (2026-10-04)

Branch `rep-105` here goes with MCBV3 branch `rep-105` (`cf42375`), off
`0885a69`. It does "Asked" item 1 and more: every x/y and yaw on the wire is
the field frame (`UART_PROTOCOL.md`), so POSE item 3 and the `head_yaw` zero
in item 4 below no longer hold there. Item 2 (pitch pivot) isn't in it.

### Where the firmware stands

Read against `Thornbots/MCBV3` branch `position-based-cv` at `0885a69`
(2026-10-03), the branch the sentry runs, and still the newest commit on any
MCBV3 branch on 2026-10-04. It has our message names and
layouts. Paths are under `MCB-project/src/`. Each line is a firmware-side
fix before the match test's E2 can score. (`uart-names-from-ros-topics` at
`47512cc` is older: its `CvTarget` still leads with a `uint32_t stamp_ms`,
19 bytes, and `getMsg` would refuse our 15.)

1. **`CV_TARGET` aims and fires, in two frames at once**
   (`subsystems/jetson/AutoAimAndFireCommand.cpp:53-90`): yaw is
   `atan2` of `x/y` minus `odo->getX()/getY()`, read as REP-105 while the
   odometry is x right (item 3; "Asked" item 1). Pitch solves for `z` from
   the pivot. The shot goes `delay_ms` after receipt less
   `FIRING_LATENCY_TIME` (80 ms) when flags bit 0 is set.
2. **`POSE` is 90 Hz and `REF_SYS` 10 Hz**: nine
   poses then one ref on one 10 ms timer (`JetsonSubsystem.cpp:27-65`); the
   200 ms ref timer (`JetsonSubsystem.hpp:137-138`) is unused.
3. **`POSE` x/y is x right, y forward** of the heading at power-on
   (`subsystems/drivetrain/SimpleAutoDriveCommand.cpp:108`), not REP-105's x
   forward, y left. `thornbots_pkg` converts it (`mcb_x_right`).
4. **`head_yaw` is `[0, 2pi)`, zero at IMU boot** (`MahonyAHRS.h:75-78`
   in taproot, via `GimbalSubsystem.cpp:41`), and counter-clockwise.
   Confirmed on the sentry 2026-10-03 (bag run00029: a hand turn CCW raised
   it). Our URDF turns `headlink` about +z to match; it turned about -z
   before that date, which mirrored `root->camera`.
5. **`odom_status` is always `ODOM_PODS`** (`JetsonSubsystem.cpp:39`).
6. **`delta_angle_got_hit_in` is 123 when not hit** since the last `REF_SYS`,
   `HitRing::PLACEHOLDER_ANGLE` (`subsystems/ui/objects/HitRing.hpp:99`).
7. **One mailbox slot.** Each frame overwrites the last
   (`communication/UARTCommunication.cpp:37-44`, the TODO at `.hpp:61`), so a
   `RELOCALIZE` landing in the same 1 ms cycle as a `CV_TARGET` is lost.
8. **`NAV_GOAL` has no reader on the sentry.** Only `AutoDriveCommand` reads
   it, and the sentry's switch schedules `SimpleAutoDriveCommand`
   (`robots/sentry/SentryControl.hpp:61`, `:191-192`), a fixed waypoint route.
9. **`seq` is always 0** on frames it sends (`UARTCommunication.cpp:21`).

`RELOCALIZE` is fine there: `checkApplyRelocalize` calls `odo->relocalizeTo`
with the frame's x/y (`JetsonSubsystem.cpp:79`).

Proposed, not applied: **`POSE` (id=2) chassis yaw** (2026-09-29), two
trailing floats taking it 25 → 33 bytes, in `UART_PROTOCOL.md`. Until both
sides agree, `RobotPose.chassis_yaw` and `chassis_yaw_rate` read 0.

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

python3 scripts/test_bridge.py                       # config + live check, 10s timeout
python3 scripts/test_bridge.py --config-only         # config check, no ROS
python3 scripts/test_bridge.py --timeout 30          # slow MCB startup
python3 scripts/test_bridge.py --device /dev/ttyUSB0 --baudrate 115200

python3 scripts/serial_debug.py /dev/ttyTHS1 --baud 115200         # raw hex dump, bridge stopped
python3 scripts/serial_debug.py /dev/ttyTHS1 --baud 115200 --log rx.bin
```

`test_bridge.py` waits for `~/pose` and `~/ref_sys` from a running bridge.
`serial_debug.py` opens the port itself, so stop the bridge first; it
defaults to 921600 baud, so always pass `--baud`. Its `--tx` sends a
`RELOCALIZE` to (5, 3) every second, which moves the MCB's odometry origin:
not on a robot that's driving.
