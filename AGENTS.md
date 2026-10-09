# ros2_dji_serial_bridge

Follow [workspace rules](../AGENTS.md) and [CI](../docs/CI.md).
ROS package: `dji_serial_bridge`. Read [UART_PROTOCOL](UART_PROTOCOL.md) before
changing wire structs/messages, and [firmware coordination](README.md#mcb-firmware-coordination)
before changing payloads. Launch through `thornbots_pkg`'s `auto.launch.py`.

## Scope

- Keep a pure UART translator; outgoing application logic goes through
  `thornbots_pkg`'s `mcb_relay`. Incoming consumers: [node graph](../thornbots_pkg/README.md#nodes).
- Preserve C++ diagnostics' CLI/checks and the hex tool's existing diagnostic
  CRC16 behavior. Only launch stays Python.

## Open

Frame agreement and RELOCALIZE coordination: [shared aim frame](README.md#shared-aim-frame).
