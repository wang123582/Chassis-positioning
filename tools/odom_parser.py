#!/usr/bin/env python3
"""
ODOM_STATE 二进制帧解析器

帧格式:
AA 55 | Ver(1) | Type(1) | Seq(1) | Len(2,LE) | Payload(36) | CRC16(2,LE)

用途:
1. 作为库函数解析单帧 ODOM_STATE
2. 直接运行脚本监听串口并打印解析结果
"""

from __future__ import annotations

import struct
import sys
import time
from typing import Any

import serial

FRAME_HEADER = b"\xAA\x55"
FRAME_LEN = 45
ODOM_STATE_TYPE = 0x02
ODOM_STATE_PAYLOAD_LEN = 36
DEFAULT_PORT = "COM7"
DEFAULT_BAUD = 115200
RAD_TO_DEG = 57.2957795


def odom_crc16(data: bytes) -> int:
    """CRC-16/CCITT: init=0xFFFF, poly=0x1021, no reflect."""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def parse_odom_frame(frame: bytes) -> dict[str, Any] | None:
    """解析单帧 ODOM_STATE，成功返回字典，失败返回 None。"""
    if len(frame) != FRAME_LEN:
        return None

    if frame[0:2] != FRAME_HEADER:
        return None

    version = frame[2]
    msg_type = frame[3]
    seq = frame[4]
    payload_len = frame[5] | (frame[6] << 8)

    if msg_type != ODOM_STATE_TYPE or payload_len != ODOM_STATE_PAYLOAD_LEN:
        return None

    crc_calc = odom_crc16(frame[2:43])
    crc_recv = frame[43] | (frame[44] << 8)
    if crc_calc != crc_recv:
        return None

    payload = frame[7:43]
    (
        t_sample_us,
        x,
        y,
        yaw,
        vx,
        vy,
        wz,
        status_bits,
        quality,
        reserved,
    ) = struct.unpack("<QffffffHBB", payload)

    return {
        "version": version,
        "type": msg_type,
        "seq": seq,
        "payload_len": payload_len,
        "t_sample_us": t_sample_us,
        "t_sec": t_sample_us / 1e6,
        "x": x,
        "y": y,
        "yaw_rad": yaw,
        "yaw_deg": yaw * RAD_TO_DEG,
        "vx": vx,
        "vy": vy,
        "wz": wz,
        "status_bits": status_bits,
        "enc_valid": bool(status_bits & 0x01),
        "imu_valid": bool(status_bits & 0x02),
        "yaw_valid": bool(status_bits & 0x04),
        "pos_valid": bool(status_bits & 0x08),
        "vel_valid": bool(status_bits & 0x10),
        "quality": quality,
        "reserved": reserved,
    }


def print_frame(res: dict[str, Any], frame_count: int, start_time: float) -> None:
    elapsed = time.time() - start_time
    hz = frame_count / elapsed if elapsed > 0 else 0.0
    print(
        f"[#{res['seq']:3d}] "
        f"t={res['t_sec']:8.3f}s  "
        f"x={res['x']:+8.4f}  y={res['y']:+8.4f}  "
        f"yaw={res['yaw_deg']:+8.2f}°  "
        f"vx={res['vx']:+6.3f}  vy={res['vy']:+6.3f}  "
        f"wz={res['wz']:+6.3f}  "
        f"q={res['quality']}  "
        f"({hz:.1f} Hz)"
    )


def run_serial_monitor(port: str, baud: int) -> None:
    print(f"Opening {port} @ {baud}...")
    ser = serial.Serial(port, baud, timeout=0.1)
    buffer = bytearray()
    frame_count = 0
    crc_or_format_errors = 0
    start_time = time.time()

    print("Waiting for ODOM_STATE frames (AA 55, type=0x02, 45 bytes)...\n")

    try:
        while True:
            data = ser.read(64)
            if not data:
                continue
            buffer.extend(data)

            while len(buffer) >= FRAME_LEN:
                if buffer[0:2] != FRAME_HEADER:
                    buffer.pop(0)
                    continue

                frame = bytes(buffer[:FRAME_LEN])
                result = parse_odom_frame(frame)
                if result is not None:
                    frame_count += 1
                    print_frame(result, frame_count, start_time)
                    del buffer[:FRAME_LEN]
                else:
                    crc_or_format_errors += 1
                    buffer.pop(0)
    except KeyboardInterrupt:
        elapsed = time.time() - start_time
        hz = frame_count / elapsed if elapsed > 0 else 0.0
        print(
            f"\n\nSummary: {frame_count} frames in {elapsed:.1f}s "
            f"({hz:.1f} Hz), {crc_or_format_errors} CRC/format errors"
        )
    finally:
        ser.close()


def main() -> int:
    port = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_PORT
    baud = int(sys.argv[2]) if len(sys.argv) > 2 else DEFAULT_BAUD
    run_serial_monitor(port, baud)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
