""""
 * @file    allan Varriance logger.py
 * @brief   This app stores the converted read data from the IMU into a txt
 *          File. Then allan Variance dashbaord app will read that file and 
 *          Perofrorm allan Variance annalysis on that logged data
 *
 *
 * @author  Abdelrahman Hewala
 * @note    Supervisor: Prof. Lutz Leutelt
 * @date    2026
"""


import os
import serial 
import sys
import time

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_FPGA_LIB_DIR = os.path.join(_THIS_DIR, '..', 'PCsetup4FPGA')
sys.path.insert(0, _FPGA_LIB_DIR)

from sample_converter import SampleConverter
from UARTFPGA_reader import read_packet

# Constants 
ACC_LSB_PER_1_G  = 2048.0             # LSBs/g.   - ±16g
GYRO_LSB_PER_DPS = 16.4            # LSBs/(dps) - ±2000dps
MAG_LSB = 0.15                      # µT/LSB, fixed per AK09916 spec
TIMESTAMP_CLK_FREQ = 100_000_000.0    # in Hz
TEMP_LSB_PER_C =  333.87
TEMP_OFFSET = 21.0

# Caliberation data
HARD_IRON_OFFSET = [-31.741723, -24.290997, 22.052194] # uT
SOFT_IRON_MATRIX = [[ 1.176901,  0.015028,  0.004831],
                    [ 0.015028,  1.285583, -0.103001],
                    [ 0.004831, -0.103001,  1.311687],
                ]

# UART Protocol Setting CONSTANTS 
PORT     = 'COM4'                                      #'/dev/tty.usbserial-10' for MAC    # change it to whatever it apears in ur machine. try command (grep ('.*usb.*' | '.*USB.*') /dev) in ur terminal to see the nammings
BAUDRATE = 1152000
PARITY   = serial.PARITY_NONE      # serial.PARITY_NONE
STOPBITS = serial.STOPBITS_ONE    # erial.STOPBITS_ONE
BYTESIZE = serial.EIGHTBITS

# Output file path. 
OUTPUT_PATH = "data/allan_log_1H.txt"
LOGGING_HOURS = 1

def main():
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
        print("Check that the FPGA is connected and the PORT constant matches your device.")
        print("Run: ls /dev/ | grep -i usb")
        sys.exit(1)

    # instantiate the converter class
    sampleConverter = SampleConverter(ACC_LSB_PER_1_G, GYRO_LSB_PER_DPS, TIMESTAMP_CLK_FREQ, MAG_LSB, TEMP_LSB_PER_C, TEMP_OFFSET)

    try:
        with open(OUTPUT_PATH, 'w', buffering=1) as f:
            start =time.time()  # logging start time
            print("logging Start!\n")

            while time.time() - start < LOGGING_HOURS*60*60:
                result = read_packet(ser)
                if result is None:
                    continue
                modeWord, samplesMode0 = result
                physicalSample = sampleConverter.convert(samplesMode0)
                # writing one data line
                f.write(f"{physicalSample.t},{physicalSample.ax},{physicalSample.ay},{physicalSample.az}, {physicalSample.gx},{physicalSample.gy},{physicalSample.gz},{physicalSample.mx},{physicalSample.my},{physicalSample.mz}, {physicalSample.mag_fresh}\n")

            print("logging stop!\n")
            print("logging Period in seconds is: ",  time.time() - start, "\n")
            f.close()
            ser.close()        

    except KeyboardInterrupt:
        print(f"\nStopped by user.")
    finally:
        f.close()
        ser.close()        

if __name__ == "__main__":
    main()