#include "ap_axi_sdata.h"
#include "hls_stream.h"
#include "ap_int.h"
#include "types.h"
#include "common_util.h"
#include "constants.h"

#define AXIS_ENABLE_LAST 0b00010000
#define AXIS_ENABLE_DATA 0b00000001

typedef hls::axis<ap_int<16>, 0, 0, 0, AXIS_ENABLE_LAST | AXIS_ENABLE_DATA> data_t;  // this one generates AXI stream interface for our module.
//  AXIS_ENABLE_LAST | AXIS_ENABLE_DATA, StrictEnablement. these ones are used to control the amount of ports. here i want only the data and the last signals
typedef hls::stream<data_t> my_stream;           // this is to pass the interface and make the handshake checks using .read(), .write(). so it is the channel it self with its mechanism


template <typename T>
void convert(my_stream& A, converter_out_t<T>& converted_sample)
{
    #pragma HLS INLINE 
    #pragma HLS INTERFACE mode=axis port=A 
    #pragma HLS ALLOCATION operation instances=sitofp limit=1
    #pragma HLS ARRAY_PARTITION variable=converted_sample.converted_data complete
    #pragma HLS ARRAY_PARTITION variable=converted_sample.raw_data complete

    static long long last_sample_ts = 0;
    #pragma HLS RESET variable=last_sample_ts
    static bool first_sample = true;
    #pragma HLS RESET variable=first_sample
    
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
        else if((i >= 4 && i < 10) || (i >= 11 && i < 14)) {  // skipping temp at index 10
            ap_uint<4> idx = (i < 10) ? (i - 4) : (i - 5);
            converted_sample.raw_data[idx] = word.data;
            converted_sample.converted_data[idx] = my_fmul((T)word.data, (T)SCALE_ALL[idx]);
        }
        else if (i == 14){
            converted_sample.mag_fresh = word.data[0];

            if (first_sample) {
                converted_sample.dt = (T)0;   
                first_sample = false;
            } else {
                ap_uint<32> delta = (ap_uint<32>)(sample_ts - last_sample_ts);
                converted_sample.dt = my_fmul((T)delta, (T)TIMESTAMP_CLK_PERIOD);
            }
            last_sample_ts = sample_ts;
        }
    }
}