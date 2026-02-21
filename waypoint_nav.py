#!/usr/bin/env python3
"""
drive_forward_10pct.py

Sends 10% throttle for 2 seconds, then stops.
"""

import time
import struct
import serial
import math
import numpy as np

from vicon_dssdk.vicon_dssdk import ViconDataStream


# Vicon Configuration
SERVER_IP = "192.168.10.1:801"  
SUBJECT_NAME = "OriginsX"       # Change to tracked subject name


# ── Protocol constants ─────────────────────────────
STX         = 0xFE
PAYLOAD_LEN = 0x04
BAUD_RATE   = 230400

STEER_MAX    = 880
STEER_MIN    = 120
STEER_CENTER = 512       # midpoint of 0–1023
MAX_THROTTLE = 200
WHEELBASE = 0.324        # meters

LOOKAHEAD = 0.8   # meters
STEER_GAIN = 300  # tune experimentally

GOAL_THRESHOLD = 0.2 # meters



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
    goal = (3.0, 0.0)

    # Vicon connect
    client = ViconDataStream.Client()
    client.Connect(SERVER_IP)
    client.EnableSegmentData()
    client.EnableMarkerData()
    print(f"Connected to {SERVER_IP}, streaming pose for subject: {SUBJECT_NAME}")

    port = "/dev/ttyUSB0"
    ser = serial.Serial(port, BAUD_RATE, timeout=0.1)
    time.sleep(0.1)
    ser.reset_input_buffer()

    seq = 0
    start_time = time.time()

    try:
        while True:
            if client.getFrame():
                if SUBJECT_NAME in client.GetSubjectNames():
                    # Get global translation (x, y, z in mm)
                    translation = client.GetSegmentGlobalTranslation(SUBJECT_NAME, SUBJECT_NAME)
                    pos, _ = translation  # unpack the tuple
                    x, y, _ = [p / 1000.0 for p in pos] # convert to meters

                    # --- Distance to goal ---
                    dx = goal[0] - x
                    dy = goal[1] - y
                    distance = np.hypot(dx, dy)

                    if distance < GOAL_THRESHOLD:
                        print("Reached goal.")
                        
                        # Optional: send zero command to stop car
                        pkt = build_packet(seq, 0, STEER_CENTER)
                        ser.write(pkt)
                        
                        break

                    # Get global rotation quaternion (x, y, z, w)
                    rotation = client.GetSegmentGlobalRotationQuaternion(SUBJECT_NAME, SUBJECT_NAME)
                    rot, _ = rotation # unpack tuple
                    qx, qy, qz, qw = rot
                    yaw = np.arctan2( # radians
                        2.0 * (qw * qz + qx * qy),
                        1.0 - 2.0 * (qy * qy + qz * qz)
                    )

                    next_throttle, next_steering = compute_next_input((x, y, yaw), (goal[0], goal[1]))

                    pkt = build_packet(seq, next_throttle, next_steering)
                    ser.write(pkt)
                    seq += 1
                    time.sleep(0.025)
                else:
                    print(f"Subject '{SUBJECT_NAME}' not found in this frame")
            else:
                print("Waiting for frame...")
                time.sleep(0.01)

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

def compute_next_input(car_pose, goal_pos):
    car_x, car_y, car_heading = car_pose
    goal_x, goal_y = goal_pos

    # World → vehicle frame
    dx = goal_x - car_x
    dy = goal_y - car_y

    car_dx = np.cos(car_heading)*dx + np.sin(car_heading)*dy
    car_dy = -np.sin(car_heading)*dx + np.cos(car_heading)*dy # switch signs of sin and cos if car steering backwards
    print(f"car_dx: {car_dx}, car_dy: {car_dy}")

    if car_dx <= 0:
        print("Missed Goal")
        return 0, STEER_CENTER

    # Force fixed lookahead distance
    Ld = max(math.sqrt(car_dx**2 + car_dy**2), LOOKAHEAD)

    # Pure pursuit curvature
    delta = math.atan((2.0 * WHEELBASE * car_dy) / (Ld**2))

    # Convert to servo
    steering = STEER_CENTER + (STEER_GAIN * delta)
    steering = np.clip(steering, STEER_MIN, STEER_MAX)

    # Constant or curvature-based throttle
    throttle = int(0.4 * MAX_THROTTLE) # tune for reasonable throttle % of 200 max

    return throttle, int(steering)



if __name__ == "__main__":
    main()
