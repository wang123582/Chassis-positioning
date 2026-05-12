#!/usr/bin/env python3
"""
发送 SET_LOCAL_ORIGIN (0x30) 到 MCU，并可等待 ACK (0x31)。

用法示例:
    python tools/send_set_local_origin.py COM7 115200 --x 0 --y 0 --yaw 0 --flags 0x07
"""

from __future__ import annotations

import argparse
import struct
import sys
import time
from typing import Final

import serial

FRAME_HEADER: Final[bytes] = b"\xAA\x55"
FRAME_VERSION: Final[int] = 0x01
MSG_SET_LOCAL_ORIGIN: Final[int] = 0x30
MSG_SET_LOCAL_ORIGIN_ACK: Final[int] = 0x31
PAYLOAD_LEN_SET_LOCAL_ORIGIN: Final[int] = 16
ACK_FRAME_LEN: Final[int] = 17
DEFAULT_FLAGS: Final[int] = 0x07


def odom_crc16(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def build_set_local_origin_frame(seq: int, x: float, y: float, yaw: float, flags: int) -> bytes:
    payload = struct.pack("<fffB3x", x, y, yaw, flags & 0xFF)
    frame_wo_crc = bytearray()
    frame_wo_crc.extend(FRAME_HEADER)
    frame_wo_crc.append(FRAME_VERSION)
    frame_wo_crc.append(MSG_SET_LOCAL_ORIGIN)
    frame_wo_crc.append(seq & 0xFF)
    frame_wo_crc.extend(struct.pack("<H", PAYLOAD_LEN_SET_LOCAL_ORIGIN))
    frame_wo_crc.extend(payload)
    crc = odom_crc16(bytes(frame_wo_crc[2:]))
    frame_wo_crc.extend(struct.pack("<H", crc))
    return bytes(frame_wo_crc)


def parse_set_local_origin_ack(frame: bytes) -> dict[str, int] | None:
    if len(frame) != ACK_FRAME_LEN:
        return None
    if frame[0:2] != FRAME_HEADER:
        return None
    if frame[2] != FRAME_VERSION or frame[3] != MSG_SET_LOCAL_ORIGIN_ACK:
        return None
    payload_len = frame[5] | (frame[6] << 8)
    if payload_len != 8:
        return None
    crc_calc = odom_crc16(frame[2:15])
    crc_recv = frame[15] | (frame[16] << 8)
    if crc_calc != crc_recv:
        return None
    acked_seq, result_code, event_counter = struct.unpack("<HHI", frame[7:15])
    return {
        "seq": frame[4],
        "acked_seq": acked_seq,
        "result_code": result_code,
        "event_counter": event_counter,
    }


def wait_for_ack(ser: serial.Serial, timeout_s: float) -> dict[str, int] | None:
    deadline = time.time() + timeout_s
    buffer = bytearray()
    while time.time() < deadline:
        data = ser.read(64)
        if data:
            buffer.extend(data)
        while len(buffer) >= ACK_FRAME_LEN:
            if buffer[0:2] != FRAME_HEADER:
                buffer.pop(0)
                continue
            candidate = bytes(buffer[:ACK_FRAME_LEN])
            parsed = parse_set_local_origin_ack(candidate)
            if parsed is not None:
                return parsed
            buffer.pop(0)
    return None


def main() -> int:
    parser = argparse.ArgumentParser(description="Send SET_LOCAL_ORIGIN (0x30) and optionally wait for ACK.")
    parser.add_argument("port", help="Serial port, e.g. COM7")
    parser.add_argument("baud", type=int, help="Baudrate, e.g. 115200")
    parser.add_argument("--x", type=float, default=0.0, help="Target x in meters")
    parser.add_argument("--y", type=float, default=0.0, help="Target y in meters")
    parser.add_argument("--yaw", type=float, default=0.0, help="Target yaw in radians")
    parser.add_argument("--flags", type=lambda s: int(s, 0), default=DEFAULT_FLAGS, help="Flags byte, default 0x07")
    parser.add_argument("--seq", type=lambda s: int(s, 0), default=1, help="Frame sequence number, default 1")
    parser.add_argument("--timeout", type=float, default=1.0, help="ACK wait timeout in seconds")
    parser.add_argument("--no-wait-ack", action="store_true", help="Send only, do not wait for ACK")
    args = parser.parse_args()

    frame = build_set_local_origin_frame(args.seq, args.x, args.y, args.yaw, args.flags)
    print(f"Sending SET_LOCAL_ORIGIN: seq={args.seq}, x={args.x}, y={args.y}, yaw={args.yaw}, flags=0x{args.flags:02X}")
    print("Frame:", frame.hex(" "))

    try:
        with serial.Serial(args.port, args.baud, timeout=0.05) as ser:
            ser.reset_input_buffer()
            ser.write(frame)
            ser.flush()
            print("Frame sent.")
            if args.no_wait_ack:
                return 0
            ack = wait_for_ack(ser, args.timeout)
    except serial.SerialException as exc:
        print(f"Serial error: {exc}", file=sys.stderr)
        return 2

    if ack is None:
        print("No ACK received within timeout.", file=sys.stderr)
        return 1

    print(
        "ACK received:",
        f"seq={ack['seq']}",
        f"acked_seq={ack['acked_seq']}",
        f"result_code={ack['result_code']}",
        f"event_counter={ack['event_counter']}",
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
