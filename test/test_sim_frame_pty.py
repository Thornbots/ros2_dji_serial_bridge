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
"""Exercise coordinated and ordinary bridge receive paths through a real PTY."""
import os
import pty
import signal
import struct
import subprocess
import sys
import time

from dji_serial_bridge.msg import Ping, SimFrameAck, SimFrameMetadata
import rclpy
from rosgraph_msgs.msg import Clock


def crc(data, polynomial, value):
    for byte in data:
        value ^= byte
        for _ in range(8):
            value = (value >> 1) ^ (polynomial if value & 1 else 0)
    return value


def frame(seq, number):
    head = struct.pack('<BHB', 0xa5, 1, seq)
    body = head + bytes([crc(head, 0x8c, 0xff)]) + struct.pack('<HB', 5, number)
    return body + struct.pack('<H', crc(body, 0x8408, 0xffff))


def exercise(mode):
    master, slave = pty.openpty()
    args = [sys.argv[1], '--ros-args', '-p', f'device:={os.ttyname(slave)}',
            '-p', 'diag_interval_s:=0', '-p', 'debug_log:=false']
    if mode != 'normal':
        args += ['-p', 'use_sim_time:=true', '-p', 'sim_frame_metadata:=true',
                 '-p', 'sim_frame_timeout_ms:=500']
    process = subprocess.Popen(args, stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT)
    node = rclpy.create_node('sim_frame_pty_test')
    acks, pings = [], []
    pub = node.create_publisher(SimFrameMetadata, '/dji_serial_bridge/sim_frame_metadata', 32)
    clock_pub = node.create_publisher(Clock, '/clock', 10)
    subs = [node.create_subscription(
        SimFrameAck, '/dji_serial_bridge/sim_frame_ack', acks.append, 32),
            node.create_subscription(Ping, '/dji_serial_bridge/ping_from_mcb', pings.append, 10)]

    def wait(predicate, timeout=4):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.01)
            assert process.poll() is None, 'bridge exited'
            if predicate():
                return
        raise AssertionError(f'{mode}: timeout, acks={acks}, pings={pings}')

    def metadata(identity, data):
        msg = SimFrameMetadata()
        msg.frame_id = identity
        msg.header.stamp.sec = 42
        msg.header.stamp.nanosec = 123
        msg.delivery_time.sec = 43
        msg.frame = data
        pub.publish(msg)

    try:
        wait(lambda: node.count_publishers('/dji_serial_bridge/ping_from_mcb') > 0 and
             (mode == 'normal' or pub.get_subscription_count() > 0))
        if mode == 'normal':
            os.write(master, frame(0, 7))
            wait(lambda: len(pings) == 1)
            assert pings[0].number == 7 and not acks
        elif mode == 'paired':
            first, second = frame(255, 7), frame(0, 8)
            metadata(1, first)
            time.sleep(0.05)
            os.write(master, first + second)
            time.sleep(0.05)
            metadata(2, second)
            for _ in range(10):
                rclpy.spin_once(node, timeout_sec=0.01)
            assert not acks and not pings
            clock = Clock()
            clock.clock.sec = 43
            clock_pub.publish(clock)
            wait(lambda: len(acks) == 2 and len(pings) == 2)
            assert [a.frame_id for a in acks] == [1, 2]
            assert all(a.published and not a.reason for a in acks)
            assert [p.number for p in pings] == [7, 8]
            assert all(p.header.stamp.sec == 42 and p.header.stamp.nanosec == 123 for p in pings)
        elif mode == 'mismatch':
            metadata(1, frame(0, 7))
            os.write(master, frame(0, 8))
            wait(lambda: len(acks) == 1)
            assert not acks[0].published and acks[0].frame_id == 1 and not pings
        elif mode == 'duplicate':
            metadata(1, frame(0, 7))
            metadata(1, frame(0, 7))
            wait(lambda: len(acks) == 1)
            assert not acks[0].published and acks[0].frame_id == 1 and not pings
            os.write(master, frame(0, 7))
            for _ in range(20):
                rclpy.spin_once(node, timeout_sec=0.01)
            assert len(acks) == 1 and not pings
        elif mode == 'missing':
            os.write(master, frame(0, 7))
            wait(lambda: len(acks) == 1)
            assert not acks[0].published and acks[0].frame_id == 0 and not pings
    finally:
        process.send_signal(signal.SIGINT)
        process.wait(timeout=5)
        node.destroy_node()
        os.close(master)
        os.close(slave)
        del subs


rclpy.init()
try:
    for case in ('normal', 'paired', 'mismatch', 'duplicate', 'missing'):
        exercise(case)
finally:
    rclpy.shutdown()
