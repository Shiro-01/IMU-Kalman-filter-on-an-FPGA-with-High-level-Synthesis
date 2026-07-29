""""
 * @file    IMU_reader.py
 * @brief   This file parse the IMU data comming from ESP32 through UART Protocol.
 *
 * Bucket Description:
 *   - SYNC1     : 0xAA  -> MArks the start of the bucket
 *   - SYNC2     : 0x55  -> Second SYNC signal to ensure lock in, one can be actually data, but a attern AA, 55 is rare to occur
 *   - TYPE      : (0x01, 0x02) Payload TYPE, 0x01 : IMU DATA + State Vector + mgValidbit, 0x02 : IMU DATA + State Vector + Performance data
 *   - LEN       : Bucket length, in case of 0x01 -> (15 * 4) + 1 = 61 Bytes
 *   - Payload   : If Type (0x01) -> (ax, ay, az, gyrox, gyroy, gyroz, magx, magy, magz, TheataX, TheataY, TheataZ, GyroBx, GyroBy, GyroBz)
 *   - CHK       : Simple XOR CheckSUM tthrough whole payload -- as Comunication reialbility is not our concern her, we were statisfied with this algorisim
 *
 * @author  Abdelrahman Hewala
 * @note    Supervisor: Prof. Lutz Leutelt
 * @date    2026

"""
import serial 
import struct 
import sys

# Bucket identification Constants
SYNC1 = 0xAA
SYNC2 = 0x55

# UART Protocol Setting CONSTANTS 
PORT     = '/dev/tty.usbserial-10'    # change it to whatever it apears in ur machine. try command (grep ('.*usb.*' | '.*USB.*') /dev) in ur terminal to see the nammings
BAUDRATE = 115200
PARITY   = serial.PARITY_ODD      # serial.PARITY_NONE
STOPBITS = serial.STOPBITS_ONE    # erial.STOPBITS_ONE
BYTESIZE = serial.EIGHTBITS

# Types Dictionary with payload foramt. < : litle endian, 15 flaots in order, and one bool
TYPES = {0x01  : '<15f?',      # ax, ay, az, gx, gy, gz, mx, my, mz, , theataX, TheataY, TheataZ, GyroBx, GyroBy, GyroBz, magValid
         }


def compute_checksum(type_byte, length_byte, payload_bytes):
    chk = type_byte ^ length_byte
    for b in payload_bytes:
        chk ^= b
    return chk

def read_packet(ser : serial.Serial) : 
    # Resync: scan for SYNC1, SYNC2
    while True:
        b = ser.read(1)
        if not b:
            return None  # timeout
        if b[0] != SYNC1:
            continue
        b2 = ser.read(1)
        if b2 and b2[0] == SYNC2:
            break

    # Read the header bytes (TYPR; LEN)
    header = ser.read(2)  # TYPE, LEN
    if len(header) < 2:
        return None
    type, length = header[0], header[1]

    # Read the payload
    payload = ser.read(length)
    if len(payload) < length:
        return None
    # Read check sum byte
    chk = ser.read(1)
    if not chk:
        return None
    
    # check XOR CHECKSUM
    if compute_checksum(type, length, payload) != chk[0] : 
        print("checksum mismatch, dropping packet")
        return None
    
    fmt = TYPES.get(type)
    if fmt is None:
        print(f"unknown packet type {type}")
        return None
    
    values = struct.unpack(fmt, payload)
    return type, values


if __name__ == "__main__":
    try:
        ser = serial.Serial(
            port=PORT,
            baudrate=BAUDRATE,
            bytesize=BYTESIZE,
            parity=PARITY,
            stopbits=STOPBITS,
            timeout=1
        )
    except serial.SerialException as e:
        print(f"Could not open serial port {PORT}: {e}")
        print("Check that the ESP32 is connected and the PORT constant matches your device.")
        print("Run: ls /dev/ | grep -i usb")
        sys.exit(1)
        
    while True:
        result = read_packet(ser)
        if result is None:
            continue
        ptype, values = result
        print(ptype, values)