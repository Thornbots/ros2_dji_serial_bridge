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
import io
from pathlib import Path
import sys
import tempfile
import unittest

TOOLS = Path(__file__).resolve().parents[1] / 'tools'
sys.path.insert(0, str(TOOLS))

import mcb_bag_to_csv as converter  # noqa: E402


class Record:
    def __init__(self, sec, nanosec, channel, data):
        self.header = type('Header', (), {})()
        self.header.stamp = type('Stamp', (), {'sec': sec, 'nanosec': nanosec})()
        self.channel = channel
        self.data = data


class TestDecode(unittest.TestCase):
    def test_full_decode_preserves_stamps_and_high_bytes(self):
        output = io.StringIO()
        records = [Record(123, 987654321, 7, bytes([0, 255, 128, 1]))]
        converter.write_rows(records, output, converter.parse_struct_format('<HBB'))
        rows = list(csv.reader(io.StringIO(output.getvalue())))
        self.assertEqual(rows, [
            ['stamp_sec', 'stamp_nanosec', 'channel', 'f0', 'f1', 'f2'],
            ['123', '987654321', '7', '65280', '128', '1'],
        ])

    def test_bytes_are_printed_as_hex_and_names_are_used(self):
        output = io.StringIO()
        converter.write_rows(
            [Record(1, 2, 3, b'\x00\xff')], output,
            converter.parse_struct_format('<2s'), names=['payload'],
        )
        self.assertEqual(list(csv.reader(io.StringIO(output.getvalue())))[1][-1], '00ff')

    def test_channel_filter_skips_other_layouts(self):
        output = io.StringIO()
        records = [Record(0, 1, 1, b'bad length'), Record(1, 2, 2, b'\x01\x00')]
        count = converter.write_rows(
            records, output, converter.parse_struct_format('<H'), channel=2
        )
        self.assertEqual(count, 1)
        self.assertIn(',2,1\r\n', output.getvalue())

    def test_malformed_length_is_reported(self):
        with self.assertRaisesRegex(converter.DecodeError, 'got 1 data bytes.*requires 2'):
            converter.write_rows(
                [Record(4, 5, 2, b'\x00')], io.StringIO(),
                converter.parse_struct_format('<H'),
            )

    def test_endian_prefix_is_required(self):
        for value in ('H', '@H', '=H', '|H', ''):
            with self.subTest(value=value), self.assertRaises(
                converter.argparse.ArgumentTypeError
            ):
                converter.parse_struct_format(value)
        self.assertEqual(converter.parse_struct_format('!I').size, 4)

    def test_reads_synthetic_rosbag2_with_explicit_message_definition(self):
        try:
            import numpy as np
            from rosbags.rosbag2 import StoragePlugin, Writer
            from rosbags.typesys import Stores, get_types_from_msg, get_typestore
        except ImportError:
            self.skipTest('rosbags is not installed')

        typestore = get_typestore(Stores.ROS2_JAZZY)
        typestore.register(get_types_from_msg(converter.MSG_DEFINITION, converter.MSGTYPE))
        message_type = typestore.types[converter.MSGTYPE]
        header_type = typestore.types['std_msgs/msg/Header']
        time_type = typestore.types['builtin_interfaces/msg/Time']
        message = message_type(
            header_type(time_type(123, 987654321), ''), 9,
            np.array([0, 255, 128, 1], dtype=np.uint8),
        )
        for version, storage in ((8, StoragePlugin.SQLITE3), (9, StoragePlugin.MCAP)):
            with self.subTest(version=version, storage=storage):
                with tempfile.TemporaryDirectory() as tmp:
                    bag = Path(tmp) / 'bag'
                    with Writer(bag, version=version, storage_plugin=storage) as writer:
                        connection = writer.add_connection(
                            converter.TOPIC, converter.MSGTYPE, typestore=typestore
                        )
                        writer.write(
                            connection, 123987654321,
                            typestore.serialize_cdr(message, converter.MSGTYPE),
                        )
                    decoded = list(converter.read_records(bag))
                output = io.StringIO()
                converter.write_rows(decoded, output, converter.parse_struct_format('<HBB'))
                self.assertIn('123,987654321,9,65280,128,1', output.getvalue())


if __name__ == '__main__':
    unittest.main()
