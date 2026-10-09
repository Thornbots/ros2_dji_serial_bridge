# ros2_dji_serial_bridge: agent notes

C++ node bridging the MCB's DJI-framed UART protocol to ROS 2 topics. Every
wire format is in `UART_PROTOCOL.md`: frame layout, all six message IDs, the
byte tables both directions, `REF_SYS` bits, `POSE` odom status codes.
Read it before touching any struct or any `msg/` file that crosses the link.
`README.md` keeps the topic list, parameters, diagnostics, and the MCB
firmware-coordination notes, including which wire changes are pending.

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

- `serial_debug` and `test_bridge` are C++ diagnostics. Preserve their CLI,
  checks and the hex tool's existing diagnostic CRC16 behavior; the
  production bridge's wire implementation is unchanged. Only launch stays Python.

- Stays a pure UART/DJI-protocol translator: no application logic. Only
  `thornbots_pkg`'s `mcb_relay` publishes outgoing aim/relocalization.
  Incoming pose/referee consumers are listed in
  [the robot node graph](../thornbots_pkg/README.md#nodes). New outgoing
  application logic goes through the relay, not a direct bridge publisher.
- Wire-format changes need firmware coordination. The MCB's matching struct
  lives outside this repo; changing a payload layout without the firmware side
  breaks the link silently. Read `README.md`'s "MCB firmware coordination"
  section before editing `dji_protocol.hpp` or any payload struct.

## Open

- Frame agreement and RELOCALIZE coordination: [shared aim frame](README.md#shared-aim-frame).

## Committing

This package is a submodule of `thornbots_workspace`, on branch `nightly`. Commit
and push here first, then bump this gitlink in `../` — one logical change, one
bump, never a gitlink pointing at an unpushed commit. Full rule in
`../CLAUDE.md` § Packages.

## CI

GitHub CI runs on PRs targeting main/nightly and pushes to both branches;
manual runs are available. Shared lint is pinned to workspace `884bfe63ea4e` (tag `ci-tooling-884bfe6`). Existing diagnostics are recorded in
`.github/quality-baseline.json`; new diagnostics fail. Do not expand the
baseline to hide regressions. Syntax errors always fail.
Jazzy CI builds the portable stack and runs this package's registered tests.
