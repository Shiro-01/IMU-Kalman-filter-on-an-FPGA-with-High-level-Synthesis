#ifndef TYPES_H
#define TYPES_H
#include "ap_int.h"

/**Stores the data recieved from the fifo*/
template <typename T>
struct converter_out_t {
    T dt;
    T converted_data[9];              // ax, ay, az;   // gx, gy, gz; //mx, my, mz;           
    bool  mag_fresh;
    ap_int<16> raw_data[9];
};


template <typename T>
struct ekf_state_t {
    T x[6];
    T p[6][6];
};

template <typename T>
struct preparer_out {
    T dt;
    T prepared_data[9];              // ax, ay, az;   // gx, gy, gz; //mx, my, mz;           
    bool  mag_fresh;
};

#endif // TYPES_H
