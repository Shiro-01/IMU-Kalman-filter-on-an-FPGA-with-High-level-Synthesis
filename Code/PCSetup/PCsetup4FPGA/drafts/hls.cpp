#include "constants.h"
#include "converter.h"

void convert(my_stream& A, converter_out_t& converted_sample)
{
#pragma HLS INTERFACE mode=axis port=A 
    static long long last_sample_ts = 0;
    long long sample_ts = 0;
    for(int i = 0; i < READ_STREAM_SIZE; i++){
    #pragma HLS PIPELINE off
        data_t word;
        A.read(word);               // Blocking: we wait here till we read smth from the interface

        if(i < 4) {
            ap_uint<16> ts_word;
            ts_word.range(15, 0) = word.data.range(15, 0);           // word is signed however the first 4 bits are time stamp fragments. so no signes required and we need to re interpret the bits
            sample_ts = sample_ts << 16 | ts_word;
        }
        else if((i >= 4) && (i < 7)) {
            int axis = i - 4;
            converted_sample.raw_accel[axis] = word.data;
            converted_sample.accel[axis] = ((float)word.data * ACCEL_SCALE[axis]);
        }
        else if((i >= 7) && (i < 10)) {
            int axis = i - 7;
            converted_sample.raw_gyro[axis] = word.data;
            converted_sample.gyro[axis] = ((float)word.data *  GYRO_SCALE[axis]);
        } // skipping i = 10 as it is the temp
        else if((i >= 11) && (i < 14)) {
            int axis = i - 11;
            converted_sample.raw_mag[axis] = word.data;
            converted_sample.mag[axis] = ((float) word.data * MAG_SCALE[axis]);
        }
        else if (i == 14){
            converted_sample.mag_fresh = word.data[0];   // no DSP slice required here. i can use the same time for calculating dt
            // updating the last sample ts + calculating dt.
            converted_sample.dt = (sample_ts - last_sample_ts) / TIMESTAMP_CLK_FREQ;
            last_sample_ts = sample_ts;
        }
    }

}



