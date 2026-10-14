/* #include "preparer.h"

typedef ap_fixed<32, 12> fixed_t;


void top_kalman_fixed(fixed_t P[6][6], fixed_t H[6], fixed_t R, fixed_t K[6]) {
    compute_kalman_gain<fixed_t>(P, H, R, K);
}


// Build 2: Floating-Point version
void top_kalman_float(float P[6][6], float H[6], float R, float K[6]) {
    compute_kalman_gain<float>(P, H, R, K);
} */

#ifndef TOP_H
#define TOP_H

#include "update.h"
#include "predict.h"
#include "preparer.h"
#include "converter.h"
#include "output_streamer.h"

//typedef ap_fixed<25,12> T;
typedef float T;

// Pick your fixed-point type explicitly here for synthesis.
// Swap this single line to compare float vs fixed-point utilization/timing.

// // Concrete, non-template top function — this is what HLS synthesizes.

// void ekf_update_top(){//ekf_state_t<T> &ekf_state, preparer_out<T> prepared_sample) {
//     // Swap to ap_fifo / axis interfaces later once you know how this block
//     // connects to whatever feeds it (streaming vs register-mapped control)

//     //update<T>(ekf_state,  prepared_sample);

//     T sin_theata = 0, cos_theata = 0;
//     T sin_phi = 0,    cos_phi = 0;
//     T sin_psi = 0,    cos_psi = 0;

//     T phi = 0;                 // is this okay or better to get the wires directly from the ekf_State?
//     T theata = 0;
//     T psi =0;
    
//     // calculating the trigometric functions
//     tri_computer(phi, theata, psi, sin_theata, cos_theata, sin_phi, cos_phi, sin_psi, cos_psi);

// }


// void ekf_update_top(T sin_theata, T cos_theata, T sin_phi, T cos_phi, T ha[3]) {
//     #pragma HLS ALLOCATION operation instances=mul limit=1 
//     #pragma HLS ALLOCATION operation instances=fmul limit=1     
//     #pragma HLS ARRAY_PARTITION variable=ha type=complete    
//     HA_CALL_1:ha_p<T>(sin_theata, cos_theata, sin_phi, cos_phi, ha);
// }

// void ekf_update_top(T cos_theata, T sin_phi, T cos_phi, T ha[3], T Ha[3][6]) {
//     #pragma HLS ARRAY_PARTITION variable=ha type=complete
//     #pragma HLS ARRAY_PARTITION variable=Ha type=complete dim=0
//     #pragma HLS ALLOCATION operation instances=fmul limit=1
//     #pragma HLS ALLOCATION operation instances=mul limit=1

//     Ha_p<T>(cos_theata, sin_phi, cos_phi, ha, Ha);
// }


// void ekf_update_top(T sin_theata, T cos_theata, T sin_phi, T cos_phi,
//                      T sin_psi, T cos_psi, T hm[3],
//                      T &cos_theata_cos_psi,
//                      T &sin_theata_cos_psi,
//                      T &sin_phi_sin_psi,
//                      T &cos_phi_sin_psi,
//                      T &sin_phi_cos_theata,
//                      T &cos_phi_cos_theata,
//                      T &sin_phi_sin_theata_cos_psi,
//                      T &cos_phi_sin_theata_cos_psi) {
//     #pragma HLS ARRAY_PARTITION variable=hm type=complete
//     #pragma HLS ALLOCATION operation instances=fmul limit=1
//     #pragma HLS ALLOCATION operation instances=mul limit=1
 
//     hm_p<T>(sin_theata, cos_theata, sin_phi, cos_phi, sin_psi, cos_psi, hm,
//             cos_theata_cos_psi, sin_theata_cos_psi, sin_phi_sin_psi, cos_phi_sin_psi,
//             sin_phi_cos_theata, cos_phi_cos_theata,
//             sin_phi_sin_theata_cos_psi, cos_phi_sin_theata_cos_psi);
// }
 


// void ekf_update_top(T sin_theata, T cos_theata, T sin_phi, T cos_phi, T sin_psi, T cos_psi,
//                      T cos_theata_cos_psi,
//                      T sin_theata_cos_psi,
//                      T sin_phi_sin_psi,
//                      T cos_phi_sin_psi,
//                      T sin_phi_cos_theata,
//                      T cos_phi_cos_theata,
//                      T sin_phi_sin_theata_cos_psi,
//                      T cos_phi_sin_theata_cos_psi,
//                      T Hm[3][6]) {
//     #pragma HLS ARRAY_PARTITION variable=Hm type=complete dim=0
//     #pragma HLS ALLOCATION operation instances=fmul limit=2
//     #pragma HLS ALLOCATION operation instances=fsub limit=2
//     #pragma HLS ALLOCATION function instances=my_fmul<T> limit=2
//     #pragma HLS ALLOCATION function instances=my_fadd<T> limit=1
//     #pragma HLS ALLOCATION function instances=my_fsub<T> limit=2
//     Hm_p<T>(sin_theata, cos_theata, sin_phi, cos_phi, sin_psi, cos_psi,
//             cos_theata_cos_psi, sin_theata_cos_psi, sin_phi_sin_psi, cos_phi_sin_psi,
//             sin_phi_cos_theata, cos_phi_cos_theata,
//             sin_phi_sin_theata_cos_psi, cos_phi_sin_theata_cos_psi,
//             Hm);
// }


// void ekf_update_top(T S[6][6], T M[6][6], T p[6][6], T Ha[3][6], T Hm[3][6],
//                      T R[6], uint8_t used_rows) {
//     // S_p is INLINE, so per UG1399 its own ARRAY_PARTITION pragmas on the
//     // *parameters* (S, M, p) would be dissolved on inlining, same as the
//     // ARRAY_PARTITION issue already hit for Ha_p/hm_p/Hm_p. Applied here
//     // in the parent scope instead. S_p's own partition on its local
//     // array H stays inside S_p, matching the pattern already used for
//     // Ha_p's local init loop.
//     #pragma HLS ARRAY_PARTITION variable=S type=complete dim=0
//     #pragma HLS ARRAY_PARTITION variable=M type=complete dim=0
//     #pragma HLS ARRAY_PARTITION variable=p type=complete dim=0
//     #pragma HLS ARRAY_PARTITION variable=Ha type=complete
//     #pragma HLS ARRAY_PARTITION variable=Hm type=complete
//     #pragma HLS ARRAY_PARTITION variable=R type=complete
 
//     #pragma HLS ALLOCATION function instances=my_fmul<T> limit=2
//     #pragma HLS ALLOCATION function instances=my_fadd<T> limit=1
//     #pragma HLS ALLOCATION function instances=my_fsub<T> limit=1
 
//     S_p<T>(S, M, p, Ha, Hm, R, used_rows);
// }
 

//  void ekf_update_top(const T S[6][6], T L[6][6], T inv_L_diag[6], uint8_t used_rows) {
//     // cholesky_decomposition is INLINE, so ARRAY_PARTITION/ALLOCATION
//     // applied inside it would be dissolved on inlining per UG1399 --
//     // applied here in the parent scope instead, same pattern as every
//     // other function this session.
//     //#pragma HLS ARRAY_PARTITION variable=S type=complete dim=0
//     //#pragma HLS ARRAY_PARTITION variable=L type=complete dim=0
//     //#pragma HLS ARRAY_PARTITION variable=inv_L_diag type=complete
 
//     #pragma HLS ALLOCATION function instances=my_fmul<T> limit=1
//     #pragma HLS ALLOCATION function instances=my_fadd<T> limit=1
//     #pragma HLS ALLOCATION function instances=my_fsub<T> limit=1
//     #pragma HLS ALLOCATION function instances=my_recip<T> limit=1
//     #pragma HLS ALLOCATION function instances=my_sqrt<T> limit=1
//     // sqrt not yet wrapped in a dedicated function -- allocation still
//     // needs to live here, at the scope that survives inlining.
 
//     cholesky_decomposition<T>(S, L, inv_L_diag, used_rows);
// }


// void ekf_update_top(ekf_state_t<T> &ekf_state, preparer_out<T> prepared_sample,
//                      T L_out[6][6], T inv_L_diag_out[6]) {
//     update<T>(ekf_state, prepared_sample, L_out, inv_L_diag_out);
// }


// void ekf_update_top(const T L[6][6], const T inv_L_diag[6], const T M[6][6],
//                      T KT[6][6], uint8_t used_rows) {
//     // no partition required
//     // ALLOCATION for my_fmul<T>/my_fadd<T>/my_fsub<T> applied in the
//     // parent scope (update())
//     #pragma HLS ALLOCATION function instances=my_fmul<T> limit=3
//     #pragma HLS ALLOCATION function instances=my_fadd<T> limit=2
//     #pragma HLS ALLOCATION function instances=my_fsub<T> limit=1
//     #pragma HLS ALLOCATION function instances=my_recip<T> limit=1
//     #pragma HLS ALLOCATION function instances=my_sqrt<T> limit=1

//    T Y[6][6];

   
//     forward_substitution<T>(L, inv_L_diag, M, Y, used_rows);
//     backward_substitution<T>(L, inv_L_diag, Y, KT, used_rows);
// }

// void ekf_update_top(T x[6], const T KT[6][6], const T innovation[6], uint8_t used_rows) {
//     // no partition required
//     // ALLOCATION for my_fmul<T>/my_fadd<T> applied in the
//     // parent scope (update())
//     #pragma HLS ALLOCATION function instances=my_fmul<T> limit=3
//     #pragma HLS ALLOCATION function instances=my_fadd<T> limit=2
//     #pragma HLS ALLOCATION function instances=my_fsub<T> limit=1
//     #pragma HLS ALLOCATION function instances=my_recip<T> limit=1
//     #pragma HLS ALLOCATION function instances=my_sqrt<T> limit=1
 
//     state_update<T>(x, KT, innovation, used_rows);
// }
 
//  void ekf_update_top(ekf_state_t<T> &ekf_state, preparer_out<T> prepared_sample) {
//     update<T>(ekf_state, prepared_sample);
// }

// void ekf(ekf_state_t<T> &ekf_state, preparer_out<T> prepared_sample, bool gimbal_lock_out) {
//     //#pragma HLS ARRAY_PARTITION variable=converted_sample.converted_data type=complete
//     //#pragma HLS ARRAY_PARTITION variable=converted_sample.raw_data type=cyclic factor=3
//     #pragma HLS ARRAY_PARTITION variable=prepared_sample.prepared_data type=complete
//     #pragma HLS ARRAY_PARTITION variable=ekf_state.x type=complete
//     #pragma HLS ARRAY_PARTITION variable=ekf_state.p type=complete dim=1
    
//     #pragma HLS ALLOCATION function instances=my_fmul<T> limit=4
//     #pragma HLS ALLOCATION function instances=my_fadd<T> limit=2
//     #pragma HLS ALLOCATION function instances=my_fsub<T> limit=1
//     #pragma HLS ALLOCATION function instances=my_recip<T> limit=1
//     #pragma HLS ALLOCATION function instances=my_sqrt<T> limit=1
//     #pragma HLS ALLOCATION function instances=tri_computer<T> limit=1
//     //#pragma HLS ALLOCATION function instances=my_sincos<T> limit=1  


//     update<T>(ekf_state, prepared_sample);
//     predict<T>(ekf_state, prepared_sample, gimbal_lock_out);
// }

// void ekf_update_top(converter_out_t<T>& converted_sample, ekf_state_t<T>& ekf_state,
//                      preparer_out<T>& prepared_sample, bool& done_flag) {
//     // initialize() is INLINE -- every ALLOCATION line it declares
//     // internally dissolves on inlining, so they're re-declared here,
//     // at the scope that survives (same rule as every other INLINE
//     // function in this project). These are isolated-test limits only;
//     // once initialize() is wired into the real combined top, these
//     // need to be reconciled against update()/predict()'s own limits,
//     // not just copied in.
//     #pragma HLS ARRAY_PARTITION variable=ekf_state.x type=complete
//     #pragma HLS ARRAY_PARTITION variable=ekf_state.p type=complete dim=1
//     #pragma HLS ARRAY_PARTITION variable=converted_sample.converted_data type=complete
//     #pragma HLS ARRAY_PARTITION variable=converted_sample.raw_data type=cyclic factor=3 // so we have
//     // ALLOCATION for my_fmul<T>/my_fadd<T>/my_fsub<T>/my_recip<T>/tri_computer<T>
//     // applied at the parent scope, same mechanism used throughout this project.
//     #pragma HLS ALLOCATION function instances=my_fmul<T> limit=1
//     #pragma HLS ALLOCATION function instances=my_fadd<T> limit=2
//     #pragma HLS ALLOCATION function instances=my_fsub<T> limit=1
//     #pragma HLS ALLOCATION function instances=my_recip<T> limit=1
//     #pragma HLS ALLOCATION function instances=my_sqrt<T> limit=1
    
//     #pragma HLS ALLOCATION function instances=tri_computer<T> limit=1
//     #pragma HLS ALLOCATION function instances=my_atan2<T, ap_fixed<22,10>, ap_fixed<16,4>> limit=1
//     #pragma HLS ALLOCATION function instances=mag_freshness_update limit=1
//     initialize<T>(converted_sample, ekf_state, prepared_sample, done_flag);
// }

// void ekf_update_top(ekf_state_t<T> &ekf_state, preparer_out<T> prepared_sample,
//                      converter_out_t<T> &converted_sample, bool &gimbal_lock_out) {
//     #pragma HLS ARRAY_PARTITION variable=ekf_state.x type=complete
//     #pragma HLS ARRAY_PARTITION variable=ekf_state.p type=complete dim = 1
//     #pragma HLS ARRAY_PARTITION variable=prepared_sample.prepared_data type=complete

//     #pragma HLS ALLOCATION function instances=my_fmul<T> limit=3
//     #pragma HLS ALLOCATION function instances=my_fadd<T> limit=2
//     #pragma HLS ALLOCATION function instances=my_fsub<T> limit=1
//     #pragma HLS ALLOCATION function instances=my_recip<T> limit=1
//     #pragma HLS ALLOCATION function instances=my_sqrt<T> limit=1
//     #pragma HLS ALLOCATION function instances=tri_computer<T> limit=1

//     ekf_state_t<T> init;
//     bool done_flag = false;
//     initialize<T>(converted_sample, init, done_flag);
//     // TODO: init.x0/init.p0_diag need to be wired into ekf_state.x/p once,
//     // gated on done_flag -- open architecture question, not resolved here.

//     update<T>(ekf_state, prepared_sample);
//     predict<T>(ekf_state, prepared_sample, gimbal_lock_out);
// }


// void ekf_update_top(T P[6][6], const T KT[6][6], const T Ha[3][6],
//                      const T Hm[3][6], const T R[6], uint8_t used_rows) {
//                         //     #pragma HLS ALLOCATION function instances=my_fmul<T> limit=3
//     #pragma HLS ALLOCATION function instances=my_fmul<T> limit=1
//     #pragma HLS ALLOCATION function instances=my_fadd<T> limit=1
//     #pragma HLS ALLOCATION function instances=my_fsub<T> limit=1
//     #pragma HLS ALLOCATION function instances=my_recip<T> limit=1
//     #pragma HLS ALLOCATION function instances=my_sqrt<T> limit=1
//     covariance_update<T>(P, KT, Ha, Hm, R, used_rows);
// }



// void ekf_update_top(converter_out_t<T>& converted_sample, ekf_state_t<T>& ekf_state,
//                      preparer_out<T>& prepared_sample, bool& done_flag, bool& gimbal_lock_out) {
//     #pragma HLS ARRAY_PARTITION variable=ekf_state.x type=complete
//     #pragma HLS ARRAY_PARTITION variable=ekf_state.p type=complete dim=1
//     #pragma HLS ARRAY_PARTITION variable=prepared_sample.prepared_data type=complete
//     #pragma HLS ARRAY_PARTITION variable=converted_sample.converted_data type=complete
//     #pragma HLS ARRAY_PARTITION variable=converted_sample.raw_data type=cyclic factor=3

//     // Reconciled across initialize()/prepare()/update()/predict() --
//     // each limit is the MAX any one consumer needs, not a sum or a
//     // straight copy from any single function's own isolated test.
//     // fmul: update/predict wanted 4, initialize wanted 1 -> 4 wins.
//     // Everything else already agreed across every consumer.
//     #pragma HLS ALLOCATION function instances=my_fmul<T> limit=4
//     #pragma HLS ALLOCATION function instances=my_fadd<T> limit=2
//     #pragma HLS ALLOCATION function instances=my_fsub<T> limit=1
//     #pragma HLS ALLOCATION function instances=my_recip<T> limit=1
//     #pragma HLS ALLOCATION function instances=my_sqrt<T> limit=1
//     #pragma HLS ALLOCATION function instances=tri_computer<T> limit=1

//     //#pragma HLS INTERFACE mode=axis port=A 
//     #pragma HLS ALLOCATION operation instances=sitofp limit=1

//     // Only initialize()/prepare() use these -- new additions, no
//     // reconciliation needed against update()/predict() since neither
//     // of those ever called them.
//     #pragma HLS ALLOCATION function instances=my_atan2<T, ap_fixed<22,10>, ap_fixed<16,4>> limit=1

//     // done_flag is a genuine in/out port: the caller is responsible for
//     // holding its value between invocations and feeding the same
//     // storage back in next cycle (same convention as gimbal_lock_out's
//     // ap_none interface). Boot phase (initialize) runs exactly once,
//     // while !done_flag; every cycle after, prepare() must run first to
//     // produce a correctly-corrected prepared_sample before update()/
//     // predict() consume it.
//     if (!done_flag) {
//         initialize<T>(converted_sample, ekf_state, prepared_sample, done_flag);
//     } else {
//         prepare<T>(converted_sample, ekf_state, prepared_sample);
//         update<T>(ekf_state, prepared_sample);
//         predict<T>(ekf_state, prepared_sample, gimbal_lock_out);
//     }
// }

void ekf_fp(my_stream& A, my_out_stream_t& OUT, bool&  gimbal_lock_out) {
    #pragma HLS INTERFACE mode=axis port=A
    #pragma HLS INTERFACE mode=axis port=OUT
    #pragma HLS INTERFACE mode=ap_ctrl_none port=return


    #pragma HLS ALLOCATION function instances=my_fmul<T> limit=4
    #pragma HLS ALLOCATION function instances=my_fadd<T> limit=2
    #pragma HLS ALLOCATION function instances=my_fsub<T> limit=1
    #pragma HLS ALLOCATION function instances=my_recip<T> limit=1
    #pragma HLS ALLOCATION function instances=my_sqrt<T> limit=1
    #pragma HLS ALLOCATION function instances=tri_computer<T> limit=1
    #pragma HLS ALLOCATION function instances=my_atan2<T, ap_fixed<22,10>, ap_fixed<16,4>> limit=1
    
    if constexpr (std::is_floating_point_v<T>) {
        #pragma HLS ALLOCATION operation instances=sitofp limit=1
    }

    static converter_out_t<T> converted_sample;
    #pragma HLS RESET variable=converted_sample

    static preparer_out<T>    prepared_sample;
    #pragma HLS RESET variable=prepared_sample

    static ekf_state_t<T>     ekf_state;
    #pragma HLS RESET variable=ekf_state

    static bool done_flag      = false;
    #pragma HLS RESET variable=done_flag

    #pragma HLS RESET variable=ekf_state.x
    #pragma HLS ARRAY_PARTITION variable=converted_sample.converted_data type=complete
    #pragma HLS ARRAY_PARTITION variable=converted_sample.raw_data type=cyclic factor=3
    #pragma HLS ARRAY_PARTITION variable=prepared_sample.prepared_data type=complete
    #pragma HLS ARRAY_PARTITION variable=ekf_state.x type=complete
    #pragma HLS ARRAY_PARTITION variable=ekf_state.p type=complete dim=1
    
    convert<T>(A, converted_sample);

    if (!done_flag) {
        initialize<T>(converted_sample, ekf_state, prepared_sample, done_flag);
    } else {
        prepare<T>(converted_sample, prepared_sample);
        update<T>(ekf_state, prepared_sample);
        output_stream<T>(converted_sample, ekf_state, OUT);
        predict<T>(ekf_state, prepared_sample, gimbal_lock_out);
    }
}
#endif // TOP_H

