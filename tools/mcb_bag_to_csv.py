#!/usr/bin/env python3
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

"""Decode opaque MCB UART channel records from a ROS bag into CSV."""

from __future__ import annotations

import argparse
import csv
from pathlib import Path
import struct
import sys
from typing import Iterable, TextIO

TOPIC = '/dji_serial_bridge/mcb_data'
MSGTYPE = 'dji_serial_bridge/msg/MCBData'
MSG_DEFINITION = """std_msgs/Header header
uint8 channel
uint8[] data
"""


class DecodeError(ValueError):
    """A record cannot be decoded using the requested firmware layout."""


def parse_struct_format(value: str) -> struct.Struct:
    if not value or value[0] not in '<>!':
        raise argparse.ArgumentTypeError(
            "format must start with '<', '>', or '!' to set byte order explicitly"
        )
    try:
        decoder = struct.Struct(value)
    except struct.error as exc:
        raise argparse.ArgumentTypeError(f'invalid struct format: {exc}') from exc
    if decoder.size == 0:
        raise argparse.ArgumentTypeError('format must decode at least one byte')
    return decoder


def _csv_value(value: object) -> object:
    if isinstance(value, (bytes, bytearray, memoryview)):
        return bytes(value).hex()
    return value


def write_rows(
    records: Iterable[object],
    output: TextIO,
    decoder: struct.Struct,
    channel: int | None = None,
    names: list[str] | None = None,
) -> int:
    field_count = len(decoder.unpack(bytes(decoder.size)))
    fields = names or [f'f{i}' for i in range(field_count)]
    if len(fields) != field_count:
        raise DecodeError(
            f'{len(fields)} field names supplied for {field_count} decoded fields'
        )
    writer = csv.writer(output)
    writer.writerow(['stamp_sec', 'stamp_nanosec', 'channel', *fields])
    count = 0
    for record in records:
        record_channel = int(record.channel)
        if channel is not None and record_channel != channel:
            continue
        data = bytes(record.data)
        if len(data) != decoder.size:
            raise DecodeError(
                f'channel {record_channel} at {record.header.stamp.sec}.'
                f'{record.header.stamp.nanosec:09d}: got {len(data)} data bytes, '
                f'but format {decoder.format!r} requires {decoder.size}; '
                'check the firmware struct format and channel'
            )
        values = [_csv_value(item) for item in decoder.unpack(data)]
        stamp = record.header.stamp
        writer.writerow([int(stamp.sec), int(stamp.nanosec), record_channel, *values])
        count += 1
    return count


def read_records(path: Path, topic: str = TOPIC) -> Iterable[object]:
    try:
        from rosbags.highlevel import AnyReader
        from rosbags.typesys import Stores, get_types_from_msg, get_typestore
    except ImportError as exc:
        raise RuntimeError(
            'rosbags is required; install it in a host Python venv with '
            'python -m pip install rosbags'
        ) from exc

    typestore = get_typestore(Stores.ROS2_JAZZY)
    typestore.register(get_types_from_msg(MSG_DEFINITION, MSGTYPE))
    from rosbags.highlevel import AnyReaderError
    try:
        with AnyReader([path], default_typestore=typestore) as reader:
            connections = [
                connection for connection in reader.connections if connection.topic == topic
            ]
            if not connections:
                raise RuntimeError(f'bag has no {topic} topic')
            for connection, timestamp, raw in reader.messages(connections=connections):
                del timestamp
                yield reader.deserialize(raw, connection.msgtype)
    except AnyReaderError as exc:
        raise RuntimeError(f'cannot read bag: {exc}') from exc


def _channel_arg(value: str) -> int:
    try:
        parsed = int(value, 0)
    except ValueError as exc:
        raise argparse.ArgumentTypeError('channel must be an integer from 0 to 255') from exc
    if not 0 <= parsed <= 255:
        raise argparse.ArgumentTypeError('channel must be from 0 to 255')
    return parsed


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag', type=Path, help='rosbag2 directory (MCAP or db3)')
    parser.add_argument('--channel', type=_channel_arg, help='only decode this uint8 channel')
    parser.add_argument(
        '--format', required=True, type=parse_struct_format,
        help="Python struct format, including byte order (for example '<Hff')",
    )
    parser.add_argument(
        '--names', help='optional comma-separated CSV field names, one per decoded value'
    )
    parser.add_argument('-o', '--output', type=Path, help='CSV output path (default: stdout)')
    args = parser.parse_args(argv)
    names = args.names.split(',') if args.names is not None else None
    if names and any(not name for name in names):
        parser.error('--names cannot contain empty names')
    try:
        records = read_records(args.bag)
        if args.output:
            with args.output.open('w', newline='', encoding='utf-8') as stream:
                write_rows(records, stream, args.format, args.channel, names)
        else:
            write_rows(records, sys.stdout, args.format, args.channel, names)
    except (DecodeError, OSError, RuntimeError) as exc:
        print(f'mcb_bag_to_csv: {exc}', file=sys.stderr)
        return 2
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
