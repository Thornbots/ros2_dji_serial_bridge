#!/usr/bin/env python3

import serial
import struct
import argparse
import time
from collections import deque

FRAME_HEAD = 0xA5
MAX_FRAME_SIZE = 4096

CRC8_INIT = 0xFF
CRC16_INIT = 0xFFFF


CRC8_TABLE = [
0x00,0x5e,0xbc,0xe2,0x61,0x3f,0xdd,0x83,0xc2,0x9c,0x7e,0x20,0xa3,0xfd,0x1f,0x41,
0x9d,0xc3,0x21,0x7f,0xfc,0xa2,0x40,0x1e,0x5f,0x01,0xe3,0xbd,0x3e,0x60,0x82,0xdc,
0x23,0x7d,0x9f,0xc1,0x42,0x1c,0xfe,0xa0,0xe1,0xbf,0x5d,0x03,0x80,0xde,0x3c,0x62,
0xbe,0xe0,0x02,0x5c,0xdf,0x81,0x63,0x3d,0x7c,0x22,0xc0,0x9e,0x1d,0x43,0xa1,0xff,
0x46,0x18,0xfa,0xa4,0x27,0x79,0x9b,0xc5,0x84,0xda,0x38,0x66,0xe5,0xbb,0x59,0x07,
0xdb,0x85,0x67,0x39,0xba,0xe4,0x06,0x58,0x19,0x47,0xa5,0xfb,0x78,0x26,0xc4,0x9a,
0x65,0x3b,0xd9,0x87,0x04,0x5a,0xb8,0xe6,0xa7,0xf9,0x1b,0x45,0xc6,0x98,0x7a,0x24,
0xf8,0xa6,0x44,0x1a,0x99,0xc7,0x25,0x7b,0x3a,0x64,0x86,0xd8,0x5b,0x05,0xe7,0xb9,
0x8c,0xd2,0x30,0x6e,0xed,0xb3,0x51,0x0f,0x4e,0x10,0xf2,0xac,0x2f,0x71,0x93,0xcd,
0x11,0x4f,0xad,0xf3,0x70,0x2e,0xcc,0x92,0xd3,0x8d,0x6f,0x31,0xb2,0xec,0x0e,0x50,
0xaf,0xf1,0x13,0x4d,0xce,0x90,0x72,0x2c,0x6d,0x33,0xd1,0x8f,0x0c,0x52,0xb0,0xee,
0x32,0x6c,0x8e,0xd0,0x53,0x0d,0xef,0xb1,0xf0,0xae,0x4c,0x12,0x91,0xcf,0x2d,0x73,
0xca,0x94,0x76,0x28,0xab,0xf5,0x17,0x49,0x08,0x56,0xb4,0xea,0x69,0x37,0xd5,0x8b,
0x57,0x09,0xeb,0xb5,0x36,0x68,0x8a,0xd4,0x95,0xcb,0x29,0x77,0xf4,0xaa,0x48,0x16,
0xe9,0xb7,0x55,0x0b,0x88,0xd6,0x34,0x6a,0x2b,0x75,0x97,0xc9,0x4a,0x14,0xf6,0xa8,
0x74,0x2a,0xc8,0x96,0x15,0x4b,0xa9,0xf7,0xb6,0xe8,0x0a,0x54,0xd7,0x89,0x6b,0x35,
]

CRC16_TABLE = [
0x0000,0x1189,0x2312,0x329b,0x4624,0x57ad,0x6536,0x74bf,
0x8c48,0x9dc1,0xaf5a,0xbed3,0xca6c,0xdbe5,0xe97e,0xf8f7,
]

# Expand table exactly like C++ implementation
while len(CRC16_TABLE) < 256:
    CRC16_TABLE.extend(CRC16_TABLE[:16])


def crc8(data: bytes, init=CRC8_INIT):
    crc = init
    for b in data:
        crc = CRC8_TABLE[crc ^ b]
    return crc


def crc16(data: bytes, init=CRC16_INIT):
    crc = init
    for b in data:
        crc = ((crc >> 8) ^ CRC16_TABLE[(crc ^ b) & 0xFF]) & 0xFFFF
    return crc


class FrameParser:
    def __init__(self):
        self.buffer = bytearray()
        self.frames = 0
        self.bad_crc8 = 0
        self.bad_crc16 = 0
        self.desyncs = 0

    def append(self, data: bytes):
        self.buffer.extend(data)

    def process(self):
        while True:
            if len(self.buffer) < 7:
                return

            sync_index = self.buffer.find(bytes([FRAME_HEAD]))

            if sync_index == -1:
                print(f"[DESYNC] discarded {len(self.buffer)} bytes")
                self.desyncs += 1
                self.buffer.clear()
                return

            if sync_index > 0:
                garbage = self.buffer[:sync_index]
                print(f"[GARBAGE] skipped {len(garbage)} bytes: {garbage.hex(' ')}")
                self.desyncs += 1
                del self.buffer[:sync_index]

            if len(self.buffer) < 7:
                return

            hdr = self.buffer[:7]

            head = hdr[0]
            data_len = struct.unpack_from('<H', hdr, 1)[0]
            seq = hdr[3]
            recv_crc8 = hdr[4]
            msg_type = struct.unpack_from('<H', hdr, 5)[0]

            calc_crc8 = crc8(hdr[:4])

            if recv_crc8 != calc_crc8:
                print(
                    f"[BAD CRC8] recv=0x{recv_crc8:02X} calc=0x{calc_crc8:02X} "
                    f"header={hdr.hex(' ')}"
                )
                self.bad_crc8 += 1
                del self.buffer[0]
                continue

            total_len = 7 + data_len + 2

            if total_len > MAX_FRAME_SIZE:
                print(f"[BAD LENGTH] {total_len}")
                del self.buffer[0]
                continue

            if len(self.buffer) < total_len:
                return

            frame = self.buffer[:total_len]

            recv_crc16 = struct.unpack_from('<H', frame, total_len - 2)[0]
            calc_crc16 = crc16(frame[:-2])

            if recv_crc16 != calc_crc16:
                print(
                    f"[BAD CRC16] recv=0x{recv_crc16:04X} calc=0x{calc_crc16:04X}"
                )
                print(f"frame: {frame.hex(' ')}")
                self.bad_crc16 += 1
                del self.buffer[0]
                continue

            payload = frame[7:-2]

            self.frames += 1

            print(
                f"[FRAME OK] "
                f"seq={seq:3d} "
                f"msg=0x{msg_type:04X} "
                f"len={data_len:3d} "
                f"payload={payload.hex(' ')}"
            )

            del self.buffer[:total_len]



def build_test_frame(seq=0, msg_type=4):
    payload = struct.pack('<ff', 5.0, 3.0)

    header = bytearray()
    header.append(FRAME_HEAD)
    header.extend(struct.pack('<H', len(payload)))
    header.append(seq)

    crc8_value = crc8(header)
    header.append(crc8_value)

    header.extend(struct.pack('<H', msg_type))

    frame = bytearray(header)
    frame.extend(payload)

    crc16_value = crc16(frame)
    frame.extend(struct.pack('<H', crc16_value))

    return bytes(frame)



def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('port')
    parser.add_argument('--baud', type=int, default=921600)
    parser.add_argument('--log')
    parser.add_argument('--tx', action='store_true')
    parser.add_argument('--tx-rate', type=float, default=1.0)
    args = parser.parse_args()

    ser = serial.Serial(
        port=args.port,
        baudrate=args.baud,
        timeout=0.05,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        rtscts=False,
        xonxoff=False,
    )

    print(f"Opened {args.port} @ {args.baud}")

    log_file = None
    if args.log:
        log_file = open(args.log, 'wb')

    frame_parser = FrameParser()

    seq = 0
    next_tx = time.time()

    try:
        while True:
            data = ser.read(4096)

            if data:
                if log_file:
                    log_file.write(data)
                    log_file.flush()

                print(f"[RX {len(data):4d}] {data.hex(' ')}")

                frame_parser.append(data)
                frame_parser.process()

            if args.tx and time.time() >= next_tx:
                frame = build_test_frame(seq)
                ser.write(frame)

                print(f"[TX] {frame.hex(' ')}")

                seq = (seq + 1) & 0xFF
                next_tx = time.time() + (1.0 / args.tx_rate)

    except KeyboardInterrupt:
        print("\nStopping...")

    finally:
        if log_file:
            log_file.close()
        ser.close()

        print("\n===== Statistics =====")
        print(f"Valid frames : {frame_parser.frames}")
        print(f"Bad CRC8     : {frame_parser.bad_crc8}")
        print(f"Bad CRC16    : {frame_parser.bad_crc16}")
        print(f"Desyncs      : {frame_parser.desyncs}")


if __name__ == '__main__':
    main()