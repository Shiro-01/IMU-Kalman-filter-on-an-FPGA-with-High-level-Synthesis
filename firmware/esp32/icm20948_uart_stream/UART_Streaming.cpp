#include <stdint.h>
#include <cstring>   
#include <Arduino.h>

#define SYNC1 0xAA
#define SYNC2 0x55
#define TYPE_IMU_STATE 0x01

/**
 * @brief Computes the XOR checksum over type, length, and payload bytes,
 *        matching the Python-side compute_checksum() exactly.
 */
uint8_t computeChecksum(uint8_t type, uint8_t length, const uint8_t* payload) {
    uint8_t chk = type ^ length;
    for (uint8_t i = 0; i < length; i++) {
        chk ^= payload[i];
    }
    return chk;
}

/**
 * @brief Packs IMU + state vector data (and mag valid flag) into the
 *        bucket protocol and sends it over UART.
 *
 * Payload layout (matches Python TYPES[0x01] = '<15f?'):
 *   ax, ay, az, gx, gy, gz, mx, my, mz,
 *   thetaX, thetaY, thetaZ, gyroBx, gyroBy, gyroBz,   (15 floats, 60 bytes)
 *   magValid                                          (1 byte, bool)
 */
void sendIMUPacket(float ax, float ay, float az,
                    float gx, float gy, float gz,
                    float mx, float my, float mz,
                    float thetaX, float thetaY, float thetaZ,
                    float gyroBx, float gyroBy, float gyroBz,
                    bool magValid) {

    const uint8_t numFloats = 15;
    const uint8_t length = (numFloats * sizeof(float)) + sizeof(uint8_t);  // 60 + 1 = 61

    uint8_t payload[length];

    float values[numFloats] = {
        ax, ay, az, gx, gy, gz, mx, my, mz,
        thetaX, thetaY, thetaZ, gyroBx, gyroBy, gyroBz
    };

    // Pack the 15 floats (little-endian, matches '<' in Python struct format)
    memcpy(payload, values, numFloats * sizeof(float));

    // Pack the bool flag as the last byte (0x01 or 0x00)
    payload[length - 1] = magValid ? 0x01 : 0x00;

    uint8_t chk = computeChecksum(TYPE_IMU_STATE, length, payload);

    // Send the full bucket: SYNC1, SYNC2, TYPE, LEN, PAYLOAD, CHK
    uint8_t packet[4 + length + 1];
    packet[0] = SYNC1;
    packet[1] = SYNC2;
    packet[2] = TYPE_IMU_STATE;
    packet[3] = length;
    memcpy(&packet[4], payload, length);
    packet[4 + length] = chk;
    Serial.write(packet, sizeof(packet));

   /*  Serial.write(SYNC1);
    Serial.write(SYNC2);
    Serial.write(TYPE_IMU_STATE);
    Serial.write(length);
    Serial.write(payload, length);
    Serial.write(chk); */
}