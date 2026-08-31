#ifndef TYPES_H
#define TYPES_H

/**Stores the data recieved from the fifo*/
struct converter_out_t {
    float dt;
    float accel[3];              // ax, ay, az;
    float gyro[3];                      // gx, gy, gz;
    float mag[3];                      //mx, my, mz;
    bool  mag_fresh;
    // raw 16-bit words in original frame order (ax..mz), for the framer:
    ap_int<16> raw_accel[3];
    ap_int<16> raw_gyro[3];
    ap_int<16> raw_mag[3];
};


#endif // TYPES_H
