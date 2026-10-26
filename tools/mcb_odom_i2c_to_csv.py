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

"""Decode MCBV3 Pico odometry I2C records (channels 16/17) joined with /odom.

Layouts: MCBV3 README, "Odometry I2C diagnostics". Writes PREFIX_records.csv
(one row per logged I2C attempt, with the nearest /odom by stamp) and
PREFIX_summary.csv (1 Hz counters), and prints an overview.
"""

from __future__ import annotations

import argparse
import bisect
import csv
from pathlib import Path
import struct
import sys
from typing import Iterable

import mcb_bag_to_csv

RECORD_CHANNEL = 16
SUMMARY_CHANNEL = 17
RECORD = struct.Struct('<IIBBH4fff')
SUMMARY = struct.Struct('<II8HIIIIIHH')
RESULTS = ['ok', 'software_reset', 'address_nack', 'data_nack',
           'arbitration_lost', 'bus_condition', 'bus_busy', 'unknown']
NEVER = 0xFFFFFFFF
RECORD_FIELDS = [
    'stamp', 'mcb_time_us', 'seq', 'seq_gap', 'result', 'result_name', 'success',
    'bus_reinit', 'pending_failures', 'enc1_pos', 'enc2_pos', 'enc1_speed',
    'enc2_speed', 'held_x', 'held_y', 'odom_stamp', 'odom_dt', 'odom_x', 'odom_y',
]
SUMMARY_FIELDS = [
    'stamp', 'mcb_time_us', 'last_seq', *[f'n_{name}' for name in RESULTS],
    'lost_failures', 'decimated_successes', 'fifo_drops', 'ms_since_success',
    'ms_since_attempt', 'bus_reinits',
]


def _stamp(header) -> float:
    return int(header.stamp.sec) + int(header.stamp.nanosec) * 1e-9


def decode_record(data: bytes) -> dict:
    if len(data) != RECORD.size:
        raise mcb_bag_to_csv.DecodeError(f'channel 16 record is {len(data)} bytes, not {RECORD.size}')
    t, seq, result, flags, pending, e1, e2, v1, v2, hx, hy = RECORD.unpack(data)
    return {
        'mcb_time_us': t, 'seq': seq, 'result': result,
        'result_name': RESULTS[result] if result < len(RESULTS) else f'code{result}',
        'success': flags & 1, 'bus_reinit': (flags >> 1) & 1, 'pending_failures': pending,
        'enc1_pos': e1, 'enc2_pos': e2, 'enc1_speed': v1, 'enc2_speed': v2,
        'held_x': hx, 'held_y': hy,
    }


def decode_summary(data: bytes) -> dict:
    if len(data) != SUMMARY.size:
        raise mcb_bag_to_csv.DecodeError(f'channel 17 summary is {len(data)} bytes, not {SUMMARY.size}')
    values = SUMMARY.unpack(data)
    row = {'mcb_time_us': values[0], 'last_seq': values[1]}
    row.update({f'n_{name}': count for name, count in zip(RESULTS, values[2:10])})
    names = ['lost_failures', 'decimated_successes', 'fifo_drops', 'ms_since_success',
             'ms_since_attempt', 'bus_reinits']
    row.update(dict(zip(names, values[10:16])))
    for key in ('ms_since_success', 'ms_since_attempt'):
        if row[key] == NEVER:
            row[key] = 'never'
    return row


def join(records: Iterable[object], odom: list[tuple[float, float, float]]):
    """Split MCB data into record and summary rows; odom is sorted (stamp, x, y)."""
    odom_stamps = [item[0] for item in odom]
    rows, summaries = [], []
    for message in records:
        stamp = _stamp(message.header)
        if int(message.channel) == SUMMARY_CHANNEL:
            summaries.append({'stamp': f'{stamp:.6f}', **decode_summary(bytes(message.data))})
        elif int(message.channel) == RECORD_CHANNEL:
            row = {'stamp': f'{stamp:.6f}', **decode_record(bytes(message.data))}
            row.update(odom_stamp='', odom_dt='', odom_x='', odom_y='')
            if odom:
                i = bisect.bisect_left(odom_stamps, stamp)
                best = min((j for j in (i - 1, i) if 0 <= j < len(odom)),
                           key=lambda j: abs(odom_stamps[j] - stamp))
                row.update(odom_stamp=f'{odom[best][0]:.6f}',
                           odom_dt=f'{odom[best][0] - stamp:+.6f}',
                           odom_x=odom[best][1], odom_y=odom[best][2])
            rows.append(row)
    rows.sort(key=lambda row: row['seq'])
    previous = None
    for row in rows:
        row['seq_gap'] = '' if previous is None else row['seq'] - previous - 1
        previous = row['seq']
    return rows, summaries


def overview(rows: list[dict], summaries: list[dict]) -> str:
    lines = [f'{len(rows)} attempt records, {len(summaries)} summaries']
    totals = {name: sum(s[f'n_{name}'] for s in summaries) for name in RESULTS}
    lines.append('summary result counts: ' + ', '.join(f'{k}={v}' for k, v in totals.items() if v))
    if summaries:
        last = summaries[-1]
        lines.append(
            f"last summary: lost_failures={last['lost_failures']} fifo_drops={last['fifo_drops']} "
            f"ms_since_success={last['ms_since_success']} bus_reinits={last['bus_reinits']}")
    ok = [row for row in rows if row['success']]
    if ok:
        lines.append(
            f"logged successes: enc1_pos {ok[0]['enc1_pos']:.4f} -> {ok[-1]['enc1_pos']:.4f}, "
            f"enc2_pos {ok[0]['enc2_pos']:.4f} -> {ok[-1]['enc2_pos']:.4f}")
    return '\n'.join(lines)


def read_odom(path: Path, topic: str) -> list[tuple[float, float, float]]:
    from rosbags.highlevel import AnyReader
    from rosbags.typesys import Stores, get_typestore
    with AnyReader([path], default_typestore=get_typestore(Stores.ROS2_JAZZY)) as reader:
        connections = [c for c in reader.connections if c.topic == topic]
        if not connections:
            print(f'mcb_odom_i2c_to_csv: bag has no {topic}; odom columns empty', file=sys.stderr)
            return []
        out = []
        for connection, _, raw in reader.messages(connections=connections):
            msg = reader.deserialize(raw, connection.msgtype)
            out.append((_stamp(msg.header), msg.pose.pose.position.x, msg.pose.pose.position.y))
    return sorted(out)


def write_csv(path: Path, fields: list[str], rows: list[dict]) -> None:
    with path.open('w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('bag', type=Path, help='rosbag2 directory (MCAP or db3)')
    parser.add_argument('-o', '--prefix', type=Path, help='output prefix (default: BAG path)')
    parser.add_argument('--odom-topic', default='/odom')
    args = parser.parse_args(argv)
    prefix = args.prefix or args.bag
    try:
        rows, summaries = join(mcb_bag_to_csv.read_records(args.bag),
                               read_odom(args.bag, args.odom_topic))
        write_csv(Path(f'{prefix}_records.csv'), RECORD_FIELDS, rows)
        write_csv(Path(f'{prefix}_summary.csv'), SUMMARY_FIELDS, summaries)
    except (mcb_bag_to_csv.DecodeError, OSError, RuntimeError) as exc:
        print(f'mcb_odom_i2c_to_csv: {exc}', file=sys.stderr)
        return 2
    print(overview(rows, summaries))
    print(f'wrote {prefix}_records.csv and {prefix}_summary.csv')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
