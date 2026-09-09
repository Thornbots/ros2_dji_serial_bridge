# ros2_dji_serial_bridge: agent notes

C++ node bridging the MCB's DJI-framed UART protocol to ROS 2 topics. Every
wire format is in `UART_PROTOCOL.md`: frame layout, all five message IDs, the
byte tables both directions, `REF_SYS_MSG` bits, `POSE_MSG` odom status codes.
Read it before touching any struct or any `msg/` file that crosses the link.
`README.md` keeps the topic list, parameters and diagnostics.

The ROS package name is `dji_serial_bridge`, not the directory name.
`--packages-select ros2_dji_serial_bridge` silently selects nothing.

Normally started by `thornbots_pkg`'s `auto.launch.py` when `real_hardware:=true`,
not launched standalone.

Shadowed by `/workspaces/ros2_ws` (`Dockerfile.thornbots` copies this directory
in at build time). Once built locally, a `src/` edit is live under `dexec.sh`
but not in the user's terminal, which resolves to the image-baked snapshot. Confirm with
`../isaac_ros_common/scripts/dexec.sh -- ros2 pkg prefix dji_serial_bridge`.
This is C++, so `--symlink-install` doesn't help, and a source change always
needs a rebuild.

## Scope

- Stays a pure UART/DJI-protocol translator: no application logic, and nothing
  but `thornbots_pkg`'s `mcb_relay` may publish or subscribe on its topics.
  Anything that wants to reach the MCB goes through that relay; adding a direct
  publisher here is the wrong fix.
- Wire-format changes need firmware coordination. The MCB's matching struct
  lives outside this repo; changing a payload layout without the firmware side
  breaks the link silently. See the pending-coordination warnings in
  `UART_PROTOCOL.md` before editing `dji_protocol.hpp` or any payload struct.
