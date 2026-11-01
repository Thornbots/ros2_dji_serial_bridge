# Copyright 2026 Thornbots
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""PTY integration: health recovery, exact counters and first-visible throttled logs."""

import os
from pathlib import Path
import pty
import signal
import struct
import subprocess
import sys
import tempfile
import time

from diagnostic_msgs.msg import DiagnosticArray
import rclpy


def crc(data, polynomial, value):
    for byte in data:
        value ^= byte
        for _ in range(8):
            value = (value >> 1) ^ (polynomial if value & 1 else 0)
    return value


def frame(kind, payload):
    header = struct.pack('<BHB', 0xa5, len(payload), 0)
    header += bytes([crc(header, 0x8c, 0xff)]) + struct.pack('<H', kind)
    body = header + payload
    return body + struct.pack('<H', crc(body, 0x8408, 0xffff))


rclpy.init()
node = rclpy.create_node('bridge_health_test')
observations = []


def receive(message):
    for status in message.status:
        if status.name.endswith('Serial link') and status.values:
            observations.append((int.from_bytes(status.level), status.message,
                                 {v.key: int(v.value) for v in status.values}))


subscription = node.create_subscription(DiagnosticArray, '/diagnostics', receive, 10)
master, slave = pty.openpty()
with tempfile.TemporaryFile(mode='w+') as output:
    process = subprocess.Popen(
        [sys.argv[1], '--ros-args', '-p', f'device:={os.ttyname(slave)}',
         '-p', 'diag_interval_s:=0', '-p', 'diagnostic_updater.period:=0.25'],
        stdout=output, stderr=subprocess.STDOUT)

    def spin_until(predicate, seconds=5):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.02)
            assert process.poll() is None, 'bridge exited'
            if predicate():
                return
        raise AssertionError(f'timed out; observations={observations}')

    def send_for(data, seconds=0.8):
        deadline = time.monotonic() + seconds
        sent = 0
        while time.monotonic() < deadline:
            os.write(master, data)
            sent += 1
            rclpy.spin_once(node, timeout_sec=0.01)
        return sent

    try:
        spin_until(lambda: any(level == 2 and 'No bytes' in text
                               for level, text, _ in observations))
        # Exactly 30 frames of each kind and 30 errors of each CRC kind.
        ref = frame(3, struct.pack('<BHHBfB', 1, 100, 200, 3, 0.0, 0))
        ping = frame(5, bytes([1]))
        bad8 = bytearray(ping)
        bad8[4] ^= 1
        bad16 = bytearray(ping)
        bad16[-1] ^= 1
        os.write(master, (ref + ping + bad8 + bad16) * 30)
        spin_until(lambda: any(level == 1 and values['frames_rx'] == 60
                               for level, _, values in observations))
        values = next(values for level, _, values in reversed(observations) if level == 1)
        assert values['crc8_errors'] == 30, values
        assert values['crc16_errors'] == 30, values
        assert values['ref_sys_msgs_pub'] == 30, values
        assert values['ping_msgs_pub'] == 30, values
        ping_count = 30 + send_for(ping)
        assert any(level == 0 for level, _, _ in observations), observations
        observations.clear()
        spin_until(lambda: any(level == 2 and 'No bytes' in text
                               for level, text, _ in observations))
        observations.clear()
        send_for(b'garbage')
        assert any(level == 2 and 'no valid frames' in text
                   for level, text, _ in observations), observations
        # All suppressed samples remain in counters, with first logs visible.
        output.seek(0)
        text = output.read()
        for sample in ('[ref_sys RX', 'CRC8 mismatch', 'CRC16 mismatch'):
            assert text.count(sample) == 1, (sample, text)
        assert text.count('[ping RX]') == ping_count, text
        assert 'DIAG' not in text, text
        if len(sys.argv) > 2:
            Path(sys.argv[2]).write_text(text)
        print('PASS: ERROR(no bytes) -> WARN(30 CRC8 + 30 CRC16) -> OK -> '
              'ERROR(stalled) -> ERROR(no frames); 60 frames counted; '
              'REF_SYS/CRC repeats suppressed, every PING visible; diagnostics with text disabled')
    finally:
        process.send_signal(signal.SIGINT)
        process.wait(timeout=5)
        os.close(master)
        os.close(slave)
        node.destroy_node()
        rclpy.shutdown()
