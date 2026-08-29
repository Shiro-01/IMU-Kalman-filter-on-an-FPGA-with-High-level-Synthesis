""""
 * @file    app.py
 * @brief   This app uses the sample_converter and UARTFPGA reader to parse & convert the IMU data
 *          comming from the FPGA through UART Protocol to a physical data. then get prepared to KF by data_preparer class then
 *          goes to EKF that predicts our eular angles
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
from typing import List
from data_preparer import DataPreparer
from frame_types import PhysicalSampleMode0
from EKF import EKF #, EKFResult
import numpy as np

# Constants 
ACC_LSB_PER_1_G  = 2048.0             # LSBs/g.   - ±16g
GYRO_LSB_PER_DPS = 16.4            # LSBs/(dps) - ±2000dps
MAG_LSB = 0.15                      # µT/LSB, fixed per AK09916 spec
TIMESTAMP_CLK_FREQ = 100_000_000.0    # in Hz
TEMP_LSB_PER_C =  333.87
TEMP_OFFSET = 21.0

# UART Protocol Setting CONSTANTS 
PORT     = '/dev/tty.usbserial-210183ACB00B1'                                      #'/dev/tty.usbserial-10' for MAC    # change it to whatever it apears in ur machine. try command (grep ('.*usb.*' | '.*USB.*') /dev) in ur terminal to see the nammings
BAUDRATE = 1152000
PARITY   = serial.PARITY_NONE      # serial.PARITY_NONE
STOPBITS = serial.STOPBITS_ONE    # erial.STOPBITS_ONE
BYTESIZE = serial.EIGHTBITS

# Caliberation data
HARD_IRON_OFFSET = [-31.741723, -24.290997, 22.052194] # uT
SOFT_IRON_MATRIX = [[ 1.176901,  0.015028,  0.004831],
                    [ 0.015028,  1.285583, -0.103001],
                    [ 0.004831, -0.103001,  1.311687],
                ]



GYRO_ARW = [0.000134255, 0.000122338, 0.000129317]
GYRO_BI = [4.28161e-5, 4.02741e-05, 4.48088e-05]

ACCEL_DEV = [0.0625, 0.0625, 0.0625]
MAG_DEV = [2, 2, 2]

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

    # instantiateof the classes class
    sampleConverter = SampleConverter(ACC_LSB_PER_1_G, GYRO_LSB_PER_DPS, TIMESTAMP_CLK_FREQ, MAG_LSB, TEMP_LSB_PER_C, TEMP_OFFSET)
    datePreparer = DataPreparer(gyro_ARW = GYRO_ARW, hard_iron_offset=HARD_IRON_OFFSET, soft_iron_matrix=SOFT_IRON_MATRIX)
    # first 100 smaples for preperation class initialize method
    samples_init_arr : List[PhysicalSampleMode0]  = []
    for i in range(1000) :
        result = read_packet(ser)
        if result is None:
            continue
        modeWord, samplesMode0 = result
        physicalSampleMode0_x = sampleConverter.convert(samplesMode0)
        samples_init_arr.append(physicalSampleMode0_x)

    # Initial state vector as np vector
    x0, p0 = datePreparer.initialize(samples_init_arr)

    ekf = EKF(
        x0 = x0,
        p0 = p0,
        gyro_ARW = GYRO_ARW,
        gyro_bias_instability = GYRO_BI,
        accel_dev = ACCEL_DEV,
        mag_dev   = MAG_DEV,
    )

    while True:
        result = read_packet(ser)
        if result is None:
            continue
        modeWord, samplesMode0 = result
        physicalSampleMode0_x = sampleConverter.convert(samplesMode0)
        preparedSample = datePreparer.prepare(physicalSampleMode0_x)
        ekf_result = ekf.step(preparedSample)
        print(np.array2string(ekf_result.X*180/np.pi, precision=2, floatmode='fixed', suppress_small=True), "\n")

        gyro_dps = np.array([physicalSampleMode0_x.gx, physicalSampleMode0_x.gy, physicalSampleMode0_x.gz]) * 180/np.pi
        print(np.array2string(gyro_dps, precision=2, floatmode='fixed', suppress_small=True), "\n")        
        print(physicalSampleMode0_x.az, "\n")        
        print(physicalSampleMode0_x.ay, "\n")        
        print(physicalSampleMode0_x.ax, "\n")     

        print(preparedSample.mx, "\n")        
        print(preparedSample.my, "\n")        
        print(preparedSample.mz, "\n")           


        mb = np.array(SOFT_IRON_MATRIX) @ (np.array([physicalSampleMode0_x.mx, physicalSampleMode0_x.my, physicalSampleMode0_x.mz]) - np.array(HARD_IRON_OFFSET))
        print(mb, np.linalg.norm(mb))

if __name__ == "__main__":
    main()