"""
decoder.py
============
simple program to print the header of a data file.

Run with:
    python3 decoder.py

@author  Abdelrahman Hewala
@date    2026
"""
import struct

# 46 bytes: I (4-byte unsigned int) + 21h (21 2-byte signed shorts)
# (7 ints per IMU * 3 IMUs = 21 ints)
frame_format = "<Q21h"

with open("Data/DATA31.BIN", "rb") as f:
    #f.seek(-200 * 50, 2)  # 2 = seek from end of file
    for frame_count in range(20):
        data = f.read(50)
        if not data or len(data) < 50:
            break
        decoded = struct.unpack(frame_format, data)
        timestamp  = decoded[0]
        imu1_data  = decoded[1:8]
        imu2_data  = decoded[8:15]
        imu3_data  = decoded[15:22]

        print(f"--- Frame {frame_count + 1} | Time: {timestamp} µs ---")
        print(f"  IMU1 | AccelX: {imu1_data[0]:6} | AccelY: {imu1_data[1]:6} | AccelZ: {imu1_data[2]:6} | Temp: {imu1_data[3]:6} | GyroX: {imu1_data[4]:6} | GyroY: {imu1_data[5]:6} | GyroZ: {imu1_data[6]:6}")
        print(f"  IMU2 | AccelX: {imu2_data[0]:6} | AccelY: {imu2_data[1]:6} | AccelZ: {imu2_data[2]:6} | Temp: {imu2_data[3]:6} | GyroX: {imu2_data[4]:6} | GyroY: {imu2_data[5]:6} | GyroZ: {imu2_data[6]:6}")
        print(f"  IMU3 | AccelX: {imu3_data[0]:6} | AccelY: {imu3_data[1]:6} | AccelZ: {imu3_data[2]:6} | Temp: {imu3_data[3]:6} | GyroX: {imu3_data[4]:6} | GyroY: {imu3_data[5]:6} | GyroZ: {imu3_data[6]:6}")
        print()