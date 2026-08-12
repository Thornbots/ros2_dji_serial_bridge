# ros2_dji_serial_bridge — agent notes

C++ node bridging the MCB's DJI-framed UART protocol to ROS 2 topics.
**Reference docs live in `README.md`** — its `## Notes` section holds the wire
formats (message types, frame layout, `REF_SYS_MSG` bit layout). Read that
before touching any struct. This file is only the operating contract.

**The ROS package name is `dji_serial_bridge`, not the directory name** —
`--packages-select ros2_dji_serial_bridge` silently selects nothing.

Parent conventions in `../CLAUDE.md` apply, notably: **in-code comments under
10 lines**; longer prose goes to a `## Notes` subheading in `README.md`.

## Running anything

Never hand-roll `docker exec`. Use `../isaac_ros_common/scripts/dexec.sh`, the
only path with correct env parity (ROS_DOMAIN_ID, FastDDS profile, both
workspace installs, `-u admin` for GUI). Load the `isaac-ros-docker` skill
before your first container command.

```bash
# all paths below are relative to this package dir
../isaac_ros_common/scripts/dexec.sh -- colcon build --packages-select dji_serial_bridge
../isaac_ros_common/scripts/dexec.sh -d -- ros2 launch dji_serial_bridge dji_bridge.launch.py
```

Normally started by `sentry_pkg`'s `auto.launch.py` when
`real_hardware:=true`, not launched standalone.

**Shadowed by `/workspaces/ros2_ws`** (`Dockerfile.thornbots`,
`RECLONE_SERIAL`). Once built locally, a `src/` edit is live under `dexec.sh`
but not in the user's terminal, which resolves to the image-baked clone.
Confirm with `../isaac_ros_common/scripts/dexec.sh -- ros2 pkg prefix dji_serial_bridge`. This is C++,
so `--symlink-install` does not help — a source change always needs a rebuild.

## Scope

- Stays a **pure UART/DJI-protocol translator**. It does no application
  logic, and nothing but `sentry_pkg`'s `mcb_relay` may publish or subscribe
  on its topics — anything that wants to reach the MCB goes through that
  relay. Adding a direct publisher here is the wrong fix.
- **Wire-format changes need firmware coordination.** The MCB's matching
  struct lives outside this repo; changing a payload layout without the
  firmware side breaks the link silently. See the warning at the top of
  `README.md` before editing `dji_protocol.hpp` or the `CV_MSG` payload.
- Its own git repo (`Thornbots/ros2_dji_serial_bridge`).
