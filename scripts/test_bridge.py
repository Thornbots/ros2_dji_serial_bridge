#!/usr/bin/env python3
"""
test_bridge.py — DJI Serial Bridge diagnostic / smoke-test script.

Runs two independent checks:

  1. CONFIG CHECK  (no ROS, no MCB needed)
     Verifies the serial device exists, is a char device, and is readable/
     writable.  Also parses dji_bridge_params.yaml and cross-checks it.

  2. LIVE TRAFFIC CHECK  (requires the bridge node to be running)
     Subscribes to ~/pose and ~/ref_sys and waits up to --timeout seconds
     for at least one message on each topic.  Prints a PASS / FAIL summary.

Usage examples
--------------
  # Config + live check (default timeout 10 s):
  python3 scripts/test_bridge.py

  # Override device and baud:
  python3 scripts/test_bridge.py --device /dev/ttyUSB0 --baudrate 115200

  # Config check only (no ROS):
  python3 scripts/test_bridge.py --config-only

  # Longer wait for slow MCB startup:
  python3 scripts/test_bridge.py --timeout 30
"""

import argparse
import os
import stat
import sys
import time
import threading

# ─── pretty printing helpers ──────────────────────────────────────────────────

GREEN  = "\033[32m"
YELLOW = "\033[33m"
RED    = "\033[31m"
CYAN   = "\033[36m"
BOLD   = "\033[1m"
RESET  = "\033[0m"

def ok(msg):   print(f"  {GREEN}✓  {RESET}{msg}")
def warn(msg): print(f"  {YELLOW}⚠  {RESET}{msg}")
def fail(msg): print(f"  {RED}✗  {RESET}{msg}")
def info(msg): print(f"     {msg}")
def hdr(msg):  print(f"\n{BOLD}{CYAN}{msg}{RESET}")

# ─── CONFIG CHECK ─────────────────────────────────────────────────────────────

def check_config(device: str, baudrate: int, params_file: str | None):
    hdr("═══  CONFIG CHECK  ═══════════════════════════════════════")

    results = []   # list of (passed: bool, description: str)

    # 1. YAML params file
    if params_file:
        if os.path.isfile(params_file):
            ok(f"Params file found: {params_file}")
            results.append((True, "params file exists"))
            try:
                import yaml   # noqa: PLC0415
                with open(params_file) as f:
                    data = yaml.safe_load(f)
                # Support both flat and nested ros__parameters style
                params = {}
                if isinstance(data, dict):
                    for v in data.values():
                        if isinstance(v, dict) and "ros__parameters" in v:
                            params.update(v["ros__parameters"])
                        elif isinstance(v, dict):
                            params.update(v)
                        else:
                            params = data
                            break

                yaml_device   = params.get("device",   "(not set)")
                yaml_baud     = params.get("baudrate", "(not set)")
                yaml_poll     = params.get("read_poll_ms", "(not set)")
                yaml_crc      = params.get("enforce_crc", "(not set)")
                info(f"  device       = {yaml_device}")
                info(f"  baudrate     = {yaml_baud}")
                info(f"  read_poll_ms = {yaml_poll}")
                info(f"  enforce_crc  = {yaml_crc}")

                if str(yaml_device) != str(device):
                    warn(f"CLI --device ({device}) differs from YAML ({yaml_device})")
                if str(yaml_baud) != str(baudrate):
                    warn(f"CLI --baudrate ({baudrate}) differs from YAML ({yaml_baud})")
            except ImportError:
                warn("PyYAML not available — skipping YAML content check")
            except Exception as e:
                warn(f"Could not parse YAML: {e}")
        else:
            warn(f"Params file not found: {params_file}")
            results.append((False, "params file missing"))
    else:
        info("No --params-file given; skipping YAML check")

    # 2. Device path
    if os.path.exists(device):
        ok(f"Device path exists: {device}")
        results.append((True, "device exists"))
    else:
        fail(f"Device NOT found: {device}")
        results.append((False, f"device {device} missing"))
        print()
        print(f"  Possible fixes:")
        print(f"    • Check USB/UART cable is connected")
        print(f"    • List available serial ports:  ls /dev/ttyTHS* /dev/ttyUSB* /dev/ttyACM* 2>/dev/null")
        print(f"    • If using USB adapter: lsusb | grep -i cp210 (or ch340, ftdi)")
        return results

    # 3. Character device?
    st = os.stat(device)
    if stat.S_ISCHR(st.st_mode):
        ok(f"Is a character device (mode {oct(st.st_mode)})")
        results.append((True, "is char device"))
    else:
        fail(f"Path exists but is NOT a character device (mode {oct(st.st_mode)})")
        results.append((False, "not a char device"))

    # 4. Read permission
    if os.access(device, os.R_OK):
        ok("Read permission OK")
        results.append((True, "readable"))
    else:
        fail(f"No read permission on {device}")
        results.append((False, "no read permission"))
        info("Fix:  sudo chmod a+r " + device)
        info("  OR: sudo usermod -aG dialout $USER  (then log out/in)")

    # 5. Write permission
    if os.access(device, os.W_OK):
        ok("Write permission OK")
        results.append((True, "writable"))
    else:
        fail(f"No write permission on {device}")
        results.append((False, "no write permission"))
        info("Fix:  sudo chmod a+w " + device)
        info("  OR: sudo usermod -aG dialout $USER  (then log out/in)")

    # 6. Try to open the port with the requested baud rate
    try:
        import serial   # noqa: PLC0415
        s = serial.Serial(device, baudrate, timeout=0.1)
        ok(f"Opened {device} @ {baudrate} baud with pyserial")
        results.append((True, f"opened @ {baudrate}"))

        # Drain any pending bytes
        s.reset_input_buffer()
        time.sleep(0.2)
        waiting = s.in_waiting
        if waiting > 0:
            sample = s.read(min(waiting, 64))
            ok(f"  Bytes already in RX buffer: {waiting} (first {len(sample)} B = {sample.hex()})")
            results.append((True, "bytes in RX buffer"))
            # Check for 0xA5 frame head
            if 0xA5 in sample:
                ok(f"  0xA5 (DJI frame head) found in sample — MCB is probably transmitting!")
                results.append((True, "0xA5 seen"))
            else:
                warn(f"  0xA5 NOT seen in {len(sample)}-byte sample — might be noise, wrong baud, or MCB not sending yet")
        else:
            warn(f"  No bytes waiting in RX buffer immediately after open  "
                 "(MCB might not be sending yet, or wrong baud rate)")
            results.append((False, "no bytes in RX buffer"))
        s.close()
    except ImportError:
        warn("pyserial not installed — skipping live port open test")
        info("Install with:  pip3 install pyserial")
    except serial.SerialException as e:
        fail(f"pyserial could not open {device}: {e}")
        results.append((False, f"open failed: {e}"))

    return results


# ─── LIVE ROS TOPIC CHECK ─────────────────────────────────────────────────────

def check_ros_topics(node_ns: str, timeout_s: float):
    hdr("═══  LIVE TOPIC CHECK  ═══════════════════════════════════")

    try:
        import rclpy                          # noqa: PLC0415
        from rclpy.node import Node           # noqa: PLC0415
        from dji_serial_bridge.msg import RobotPose, RefSysStatus  # noqa: PLC0415
    except ImportError as e:
        warn(f"ROS 2 / message imports not available: {e}")
        warn("Skipping live topic check (is your ROS 2 workspace sourced?)")
        return []

    results = []
    rclpy.init()

    # Events set by callbacks
    got_pose    = threading.Event()
    got_ref_sys = threading.Event()

    # Store first message for display
    first_pose    = [None]
    first_ref_sys = [None]

    class Listener(Node):
        def __init__(self):
            super().__init__("dji_bridge_test_listener")

            ns = node_ns.rstrip("/")
            pose_topic    = f"{ns}/pose"
            ref_sys_topic = f"{ns}/ref_sys"

            info(f"Subscribing to:  {pose_topic}")
            info(f"                 {ref_sys_topic}")
            info(f"Waiting up to {timeout_s:.0f} s for messages...\n")

            from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy  # noqa
            sensor_qos = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
            )

            self.create_subscription(RobotPose, pose_topic,
                self._pose_cb, sensor_qos)
            self.create_subscription(RefSysStatus, ref_sys_topic,
                self._ref_sys_cb, sensor_qos)

        def _pose_cb(self, msg):
            if not got_pose.is_set():
                first_pose[0] = msg
                got_pose.set()

        def _ref_sys_cb(self, msg):
            if not got_ref_sys.is_set():
                first_ref_sys[0] = msg
                got_ref_sys.set()

    node = Listener()
    deadline = time.monotonic() + timeout_s

    # Spin until we have both messages or time runs out
    executor = rclpy.get_global_executor()
    if executor is None:
        executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)

    while time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.1)
        if got_pose.is_set() and got_ref_sys.is_set():
            break

    # Report
    if got_pose.is_set():
        m = first_pose[0]
        ok("Received ~/pose")
        info(f"  x={m.x:.3f}  y={m.y:.3f}  vel_x={m.vel_x:.3f}  vel_y={m.vel_y:.3f}  "
             f"pitch={m.head_pitch:.3f}  yaw={m.head_yaw:.3f}")
        results.append((True, "~/pose received"))
    else:
        fail(f"~/pose — NO message received within {timeout_s:.0f} s")
        results.append((False, "~/pose timeout"))
        info("Possible causes:")
        info("  • MCB not powered / not connected")
        info("  • Wrong baud rate (check dji_bridge_params.yaml vs MCB firmware)")
        info("  • CRC mismatches causing all frames to be dropped  "
             "(try enforce_crc: false temporarily)")
        info("  • Bridge node not running  (ros2 node list | grep dji)")

    if got_ref_sys.is_set():
        m = first_ref_sys[0]
        ok("Received ~/ref_sys")
        info(f"  stage={m.game_stage}  hp={m.robot_hp}  robot_id={m.robot_id}  "
             f"blue={m.is_on_blue_team}  chassis_power={m.chassis_has_power}  "
             f"gimbal_power={m.gimbal_has_power}")
        results.append((True, "~/ref_sys received"))
    else:
        fail(f"~/ref_sys — NO message received within {timeout_s:.0f} s")
        results.append((False, "~/ref_sys timeout"))

    executor.remove_node(node)
    node.destroy_node()
    rclpy.shutdown()
    return results


# ─── SUMMARY ─────────────────────────────────────────────────────────────────

def print_summary(all_results):
    hdr("═══  SUMMARY  ════════════════════════════════════════════")
    passed = sum(1 for r in all_results if r[0])
    total  = len(all_results)
    for r in all_results:
        if r[0]:
            ok(r[1])
        else:
            fail(r[1])
    print()
    if passed == total:
        print(f"  {GREEN}{BOLD}ALL {total} checks PASSED ✓{RESET}")
    else:
        print(f"  {RED}{BOLD}{total - passed}/{total} checks FAILED{RESET}")
    print()


# ─── MAIN ─────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="DJI Serial Bridge config + live-traffic smoke test")
    parser.add_argument("--device",      default="/dev/ttyTHS1",
                        help="Serial device (default: /dev/ttyTHS1)")
    parser.add_argument("--baudrate",    type=int, default=115200,
                        help="Baud rate (default: 115200)")
    parser.add_argument("--params-file", default=None,
                        help="Path to dji_bridge_params.yaml for cross-check")
    parser.add_argument("--node-ns",     default="/dji_serial_bridge",
                        help="ROS node namespace for topic check (default: /dji_serial_bridge)")
    parser.add_argument("--timeout",     type=float, default=10.0,
                        help="Seconds to wait for ROS messages (default: 10)")
    parser.add_argument("--config-only", action="store_true",
                        help="Run config check only (no ROS required)")
    args = parser.parse_args()

    # Try to locate params file automatically if not given
    if args.params_file is None:
        # Common relative path when running from the package root
        candidates = [
            os.path.join(os.path.dirname(__file__), "..", "config", "dji_bridge_params.yaml"),
            "config/dji_bridge_params.yaml",
        ]
        for c in candidates:
            if os.path.isfile(c):
                args.params_file = os.path.realpath(c)
                break

    print(f"\n{BOLD}DJI Serial Bridge — Diagnostic Test{RESET}")
    print(f"  device={args.device}  baudrate={args.baudrate}  "
          f"timeout={args.timeout}s  config_only={args.config_only}\n")

    all_results = []
    all_results += check_config(args.device, args.baudrate, args.params_file)

    if not args.config_only:
        all_results += check_ros_topics(args.node_ns, args.timeout)
    else:
        info("(skipping live topic check — --config-only)")

    print_summary(all_results)
    sys.exit(0 if all(r[0] for r in all_results) else 1)


if __name__ == "__main__":
    main()
