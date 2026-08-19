""""
 * @file    app.py
 * @brief   This app uses the sample_converter cladd and UARTFPGA reader to parse & convert the IMU data
 *          comming from the FPGA through UART Protocol to a physical data.
 *
 *
 * @author  Abdelrahman Hewala
 * @note    Supervisor: Prof. Lutz Leutelt
 * @date    2026
"""
import serial 
from sample_converter import SampleConverter
import sys
from UARTFPGA_reader import read_packet

# Constants 
ACC_LSB_PER_1_G  = 2048.0             # LSBs/g.   - ±16g
GYRO_LSB_PER_DPS = 16.4            # LSBs/(dps) - ±2000dps
MAG_LSB = 0.15                      # µT/LSB, fixed per AK09916 spec
TIMESTAMP_CLK_FREQ = 100_000_000.0    # in Hz
TEMP_LSB_PER_C =  333.87
TEMP_OFFSET = 21.0

# UART Protocol Setting CONSTANTS 
PORT     = '/dev/tty.usbserial-10'    # change it to whatever it apears in ur machine. try command (grep ('.*usb.*' | '.*USB.*') /dev) in ur terminal to see the nammings
BAUDRATE = 1152000
PARITY   = serial.PARITY_NONE      # serial.PARITY_NONE
STOPBITS = serial.STOPBITS_ONE    # erial.STOPBITS_ONE
BYTESIZE = serial.EIGHTBITS

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
        
    while True:
        result = read_packet(ser)
        if result is None:
            continue
        modeWord, samplesMode0 = result

        physicalSampleMode0_x = sampleConverter.convert(samplesMode0)
        print(modeWord, samplesMode0.ts, physicalSampleMode0_x)


if __name__ == "__main__":
    main()