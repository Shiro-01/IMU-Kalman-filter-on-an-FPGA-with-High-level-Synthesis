#ifndef OUTPUT_STREAMER_H
#define OUTPUT_STREAMER_H

#include "types.h"
#include "ap_int.h"
#include "hls_stream.h"
#include "ap_axi_sdata.h"
#include "common_util.h"

#define OUT_RAW_WORDS    9   // raw_data[0..8], already 16-bit, no split needed
#define OUT_DT_WORDS     1   // dt, one float split into 2x 16-bit
#define OUT_STATE_WORDS  6   // ekf_state.x[0..5], each float split into 2x 16-bit

typedef hls::axis<ap_int<16>, 0, 0, 0, AXIS_ENABLE_LAST | AXIS_ENABLE_DATA> data_out_t;
typedef hls::stream<data_out_t> my_out_stream_t;

// =========================================================================
// output_stream -- publishes one full result frame per call:
//   words 0-8   : raw_data[0..8], one 16-bit word each (9 words)
//   words 9-10  : dt, split lo/hi (2 words)
//   words 11-22 : ekf_state.x[0..5], each split lo/hi (12 words)
//   total: 23 words per frame, TLAST on the final word
//
// =========================================================================
template <typename T>
void output_stream(const converter_out_t<T>& converted_sample,
                    const ekf_state_t<T>& ekf_state,
                    my_out_stream_t& OUT)
{
    #pragma HLS INLINE off
    #pragma HLS ALLOCATION function instances=to_bits32<T> limit=1

    data_out_t w;

    // ---- raw sensor words, unchanged, already 16-bit ----
    OUT_RAW_LOOP: for (int i = 0; i < OUT_RAW_WORDS; i++) {
        #pragma HLS PIPELINE off
        w.data = converted_sample.raw_data[i];
        w.last = 0;
        OUT.write(w);
    }

    // ---- dt, split lo/hi, same pattern as the state words below ----
    {
        #pragma HLS PIPELINE off
        ap_int<32> dt_bits = to_bits32<T>(converted_sample.dt);

        w.data = dt_bits.range(31, 16);
        w.last = 0;
        OUT.write(w);
        
        w.data = dt_bits.range(15, 0);
        w.last = 0;
        OUT.write(w);
    }

    // ---- state vector, each float split into lo/hi 16-bit words ----
    OUT_STATE_LOOP: for (int i = 0; i < OUT_STATE_WORDS; i++) {
        #pragma HLS PIPELINE off
        ap_int<32> bits = to_bits32<T>(ekf_state.x[i]);

        w.data = bits.range(31, 16);
        w.last = 0;
        OUT.write(w);

        w.data = bits.range(15, 0);
        w.last = (i == OUT_STATE_WORDS - 1) ? 1 : 0;
        OUT.write(w);
    }
}

#endif