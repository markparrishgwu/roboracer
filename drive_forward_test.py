#!/usr/bin/env python3
"""
drive_forward_10pct.py

Sends 10% throttle for 2 seconds, then stops.
"""

import time
import struct
import serial


# ── Protocol constants ─────────────────────────────
STX         = 0xFE
PAYLOAD_LEN = 0x04
BAUD_RATE   = 230400

STEER_CENTER = 512       # midpoint of 0–1023
MAX_THROTTLE = 2047


# ── CRC16-CCITT (same as your original script) ────
def crc16_ccitt(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


# ── Packet builder (identical structure) ──────────
def build_packet(seq: int, throttle: int, steering: int) -> bytes:
    seq &= 0xFF
    throttle = max(0, min(throttle, 2047))
    steering = max(0, min(steering, 1023))

    payload = struct.pack('<BBHh', PAYLOAD_LEN, seq, throttle, steering)
    crc = crc16_ccitt(payload)

    return bytes([STX]) + payload + struct.pack('<H', crc)


# ── Main logic ─────────────────────────────────────
def main():
    port = "/dev/ttyUSB0"
    ser = serial.Serial(port, BAUD_RATE, timeout=0.1)
    time.sleep(0.1)
    ser.reset_input_buffer()

    seq = 0
    throttle_10pct = int(0.10 * MAX_THROTTLE)

    try:
        print("Driving forward...")
        start_time = time.time()

        while time.time() - start_time < 2.0:
            pkt = build_packet(seq, throttle_10pct, STEER_CENTER)
            ser.write(pkt)
            seq += 1
            time.sleep(0.025)

    except KeyboardInterrupt:
        print("Interrupted!")

    finally:
        # 🔴 HARD STOP
        print("Sending kill packet...")
        stop_pkt = build_packet(seq, 0, STEER_CENTER)
        ser.write(stop_pkt)
        time.sleep(0.05)  # ensure transmission
        ser.close()
        print("Stopped and port closed.")



if __name__ == "__main__":
    main()
