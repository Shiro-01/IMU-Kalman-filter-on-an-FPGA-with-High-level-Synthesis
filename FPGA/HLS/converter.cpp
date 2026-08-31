#include "ap_axi_sdata.h"
#include "hls_stream.h"
#include "ap_int.h"

#include "constants.h"
#include "types.h"

typedef hls::axis<ap_int<16>, 0, 0, 0, AXIS_ENABLE_LAST | AXIS_ENABLE_DATA> data_t;  // this one generates AXI stream interface for our module.
//  AXIS_ENABLE_LAST | AXIS_ENABLE_DATA, StrictEnablement. these ones are used to control the amount of ports. here i want only the data and the last signals
typedef hls::stream<data_t>  my_stream;           // this is to pass the interface and make the handshake checks using .read(), .write(). so it is the channel it self with its mechanism

void convert(my_stream& A, converter_out_t& converted_sample)
{
#pragma HLS INTERFACE mode=axis port=A 
    static long long last_sample_ts = 0;
    long long sample_ts = 0;
    for(int i = 0; i < READ_STREAM_SIZE; i++){
    //   #pragma HLS PIPELINE  // try with and without
        data_t word;
        A.read(word);               // Blocking: we wait here till we read smth from the interface

        if(i < 4) {
            ap_uint<16> ts_word;
            ts_word.range(15, 0) = word.range(15, 0);           // word is signed however the first 4 bits are time stamp fragments. so no signes required and we need to re interpret the bits
            sample_ts = sample_ts << 16 | ts_word;
        }
        else if((i >= 4) && (i < 7)) {
            int axis = i - 4;
            converted_sample.raw_accel[axis] = word.data;
            converted_sample.accel[axis] = ACCEL_SIGN[axis] *
                ((float)word.data / ACC_LSB_PER_1_G * GRAVITY) - ACCEL_BIAS_OFFSET[axis];
        }
        else if((i >= 7) && (i < 10)) {
            int axis = i - 7;
            converted_sample.raw_gyro[axis] = word.data;
            converted_sample.gyro[axis] = GYRO_SIGN[axis] * 
                ((float)word.data / GYRO_LSB_PER_DPS * DEG2RAD);
        } // skipping i = 10 as it is the temp
        else if((i >= 11) && (i < 14)) {
            int axis = i - 11;
            converted_sample.raw_mag[axis] = word.data;
            converted_sample.mag[axis] = MAG_SIGN[axis]  * ((float) word.data * MAG_LSB);
        }
        else if (i == 14){
            converted_sample.mag_fresh = word.data[0];   // no DSP slice required here. i can use the same time for calculating dt
            // updating the last sample ts + calculating dt.
            converted_sample.dt = (sample_ts - last_sample_ts) / TIMESTAMP_CLK_FREQ;
            last_sample_ts = sample_ts;
        }
    }

}