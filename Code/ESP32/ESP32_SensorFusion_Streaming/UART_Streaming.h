#ifndef UART_STREAMING_H
#define UART_STREAMING_H

#include <stdint.h>

void sendIMUPacket(float ax, float ay, float az,
                    float gx, float gy, float gz,
                    float mx, float my, float mz,
                    float thetaX, float thetaY, float thetaZ,
                    float gyroBx, float gyroBy, float gyroBz,
                    bool magValid);

#endif