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

import csv
from pathlib import Path
import struct
import sys
import tempfile
import unittest

TOOLS = Path(__file__).resolve().parents[1] / 'tools'
sys.path.insert(0, str(TOOLS))

import mcb_odom_i2c_to_csv as odom  # noqa: E402


class Record:
    def __init__(self, stamp, channel, data):
        self.header = type('Header', (), {})()
        self.header.stamp = type('Stamp', (), {'sec': int(stamp), 'nanosec': round(stamp % 1 * 1e9)})()
        self.channel = channel
        self.data = data


def record(seq, result, enc2, held_y):
    return struct.pack('<IIBBH4fff', seq * 500, seq, result, 1 if result == 0 else 2, 0,
                       0.5, enc2, 0.0, 0.0, 0.5, held_y)


class TestOdomI2c(unittest.TestCase):
    def test_records_join_nearest_odom_and_report_gaps(self):
        summary = struct.pack('<II8HIIIIIHH', 1000000, 9, 7, 0, 1, 0, 0, 0, 1, 0,
                              2, 5, 0, 3, 0xFFFFFFFF, 1, 0)
        messages = [
            Record(10.020, 16, record(9, 2, 0.0, 1.25)),
            Record(10.000, 16, record(4, 0, 1.25, 1.25)),
            Record(10.030, 17, summary),
            Record(10.010, 255, b'\x01\x00\x00\x00'),
        ]
        rows, summaries = odom.join(messages, [(9.995, 1.0, 2.0), (10.019, 1.5, 2.5)])
        self.assertEqual([row['seq'] for row in rows], [4, 9])
        self.assertEqual(rows[1]['seq_gap'], 4)
        self.assertEqual(rows[1]['result_name'], 'address_nack')
        self.assertEqual((rows[1]['success'], rows[1]['bus_reinit']), (0, 1))
        self.assertEqual(rows[0]['enc2_pos'], 1.25)
        self.assertEqual((rows[0]['odom_x'], rows[1]['odom_x']), (1.0, 1.5))
        self.assertEqual(summaries[0]['n_ok'], 7)
        self.assertEqual(summaries[0]['n_bus_busy'], 1)
        self.assertEqual(summaries[0]['ms_since_attempt'], 'never')
        self.assertIn('lost_failures=2', odom.overview(rows, summaries))

    def test_wrong_length_is_rejected(self):
        with self.assertRaises(odom.mcb_bag_to_csv.DecodeError):
            odom.join([Record(1.0, 16, b'\x00' * 35)], [])

    def test_cli_decodes_mcap_bag_with_odom(self):
        try:
            import numpy as np
            from rosbags.rosbag2 import StoragePlugin, Writer
            from rosbags.typesys import Stores, get_types_from_msg, get_typestore
        except ImportError:
            self.skipTest('rosbags is not installed')
        ts = get_typestore(Stores.ROS2_JAZZY)
        ts.register(get_types_from_msg(odom.mcb_bag_to_csv.MSG_DEFINITION, odom.mcb_bag_to_csv.MSGTYPE))
        t = ts.types
        header = t['std_msgs/msg/Header'](t['builtin_interfaces/msg/Time'](5, 0), '')
        data = t[odom.mcb_bag_to_csv.MSGTYPE](
            header, 16, np.frombuffer(record(3, 0, 2.0, 2.0), dtype=np.uint8))
        pose = t['geometry_msgs/msg/PoseWithCovariance'](
            t['geometry_msgs/msg/Pose'](t['geometry_msgs/msg/Point'](-4.0, 0.25, 0.0),
                                        t['geometry_msgs/msg/Quaternion'](0.0, 0.0, 0.0, 1.0)),
            np.zeros(36))
        twist = t['geometry_msgs/msg/TwistWithCovariance'](
            t['geometry_msgs/msg/Twist'](t['geometry_msgs/msg/Vector3'](0.0, 0.0, 0.0),
                                         t['geometry_msgs/msg/Vector3'](0.0, 0.0, 0.0)),
            np.zeros(36))
        odom_msg = t['nav_msgs/msg/Odometry'](header, 'base', pose, twist)
        with tempfile.TemporaryDirectory() as tmp:
            bag = Path(tmp) / 'bag'
            with Writer(bag, version=9, storage_plugin=StoragePlugin.MCAP) as writer:
                c1 = writer.add_connection(odom.mcb_bag_to_csv.TOPIC, odom.mcb_bag_to_csv.MSGTYPE,
                                           typestore=ts)
                c2 = writer.add_connection('/odom', 'nav_msgs/msg/Odometry', typestore=ts)
                writer.write(c1, 5_000_000_000, ts.serialize_cdr(data, odom.mcb_bag_to_csv.MSGTYPE))
                writer.write(c2, 5_000_000_000, ts.serialize_cdr(odom_msg, 'nav_msgs/msg/Odometry'))
            self.assertEqual(odom.main([str(bag), '-o', f'{tmp}/out']), 0)
            with open(f'{tmp}/out_records.csv', encoding='utf-8') as stream:
                rows = list(csv.DictReader(stream))
        self.assertEqual(rows[0]['seq'], '3')
        self.assertEqual(rows[0]['odom_x'], '-4.0')
        self.assertEqual(rows[0]['enc2_pos'], '2.0')


if __name__ == '__main__':
    unittest.main()
