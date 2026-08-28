""""
 * @file    UARTFPGA_reader.py
 * @brief   This app parse the IMU data comming from the FPGA through UART Protocol.
 *
 * Bucket Description:
 *   - SYNC1     : 0xAA  -> MArks the start of the bucket
 *   - SYNC2     : 0x55  -> Second SYNC signal to ensure lock in, one can be actually data, but a attern AA, 55 is rare to occur
 *   - MODE      : (0x00, 0x01) Payload Mode, MODE_0 (0x00) : IMU DATA,,, MODE_1 (0x01) : IMU DATA + State Vector + Performance data (still to be determined)
 *   - LEN       : Bucket length, in case of MODE_0(0x01) -> (15 * 2) = 30 Bytes,,,, MODE_1 : still to determine later
 *   - Payload   : If MODE_0 (0x01) -> (TS_MSB, TS, TS, TS_LSB, ax, ay, az, gyrox, gyroy, gyroz, temp, magx, magy, magz, magState) -> all in 16 bit wide words, so each needs 2 uart transactions
 *   - CHK       : Simple XOR CheckSUM tthrough whole payload -- as Comunication reialbility is not our concern her, we were statisfied with this algorisim
 *
 * @author  Abdelrahman Hewala
 * @note    Supervisor: Prof. Lutz Leutelt
 * @date    2026
"""
import serial 
import struct 
from frame_types import RawSampleMode0


# Bucket identification Constants
SYNC1 = 0xAA
SYNC2 = 0x55


# Modes Dictionary with payload foramt. > : big endian (serializer sends MSB first per word).
# MODE_0: Q = TS_MSB,TS,TS,TS_LSB (unsigned long long) | 10h = ax,ay,az,gyrox,gyroy,gyroz,temp,magx,magy,magz (signed) | H = magState (unsigned)
MODE_REGISTRY = {
        0x001E: ('>Q10hH', RawSampleMode0),     # (TS_MSB, TS, TS, TS_LSB, ax, ay, az, gyrox, gyroy, gyroz, temp, magx, magy, magz, magState)
        # 0x012A: ('>...', RawSampleMode1),     # MODE_1 payload format, still to be determined
}

def compute_checksum(payload, length):
    chk = 0
    for i in range(0, length, 2):
        word = (payload[i] << 8) | payload[i + 1]
        chk ^= word
    return chk

def read_packet(ser : serial.Serial): 
    # Resync: slide one byte at a time until SYNC1 immediately followed by SYNC2
    prev = None
    while True:
        b = ser.read(1)
        if not b:
            return None  # timeout
        cur = b[0]
        if prev == SYNC1 and cur == SYNC2:
            break
        prev = cur

    # Read the header bytes (MODE, LEN)
    header = ser.read(2)  # MODE, LEN
    if len(header) < 2:
        return None
    modeWord = (header[0]) << 8 | header[1] 
    length = header[1] 

    # Read the payload
    payload = ser.read(length)
    if len(payload) < length:
        return None
    # Read check sum word (16 bit, MSB first -- same width as every payload word)
    chk_bytes = ser.read(2)
    if len(chk_bytes) < 2:
        return None
    chk_received = (chk_bytes[0] << 8) | chk_bytes[1]

    # check XOR CHECKSUM
    if compute_checksum(payload, length) != chk_received:
        print("checksum mismatch, dropping packet")
        return None

    entry = MODE_REGISTRY.get(modeWord)
    if entry is None:
        print(f"unknown packet type 0x{modeWord:04X}")
        return None
    
    fmt, sample_cls = entry
    values = struct.unpack(fmt, payload)
    return modeWord, sample_cls(*values)
