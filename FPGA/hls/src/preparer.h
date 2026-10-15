
#ifndef PREPARER_H
#define PREPARER_H
#include "common_util.h"   // my_fmul, my_fadd, my_fsub, my_recip, tri_computer
#include "constants.h"
#include "types.h"
#include "hls_math.h"


// =========================================================================
// Mag hard/soft iron correction. Reuses subtract_offset3 for the
// hard-iron step, then applies the soft-iron matrix.
// =========================================================================
template <typename T>
static inline void correct_mag(T data_array[9]) {
    #pragma HLS INLINE //off

    for(int i = 6; i < 9; i ++){
        #pragma HLS PIPELINE off
        data_array[i] = my_fsub<T>(data_array[i], HARD_IRON_OFFSET[i - 6]);  // offseting mag Readings - vectoe is in constants
    }
 
    T m_c[3];
    LOOP_MAG_CORR_I: for (int i = 0; i < 3; i++) {
        #pragma HLS PIPELINE off
        T sum = 0;
        LOOP_MAG_CORR_J: for (int j = 0; j < 3; j++) {
            #pragma HLS PIPELINE off
            T prod = my_fmul<T>((T)SOFT_IRON_MATRIX[i][j], data_array[6 + j]);
            sum = my_fadd<T>(sum, prod);
        }
        m_c[i] = sum;   // write to a separate output
    }
    data_array[6] = m_c[0]; data_array[7] = m_c[1]; data_array[8] = m_c[2];   // copy/wire back only after all three rows are computed
}


// =========================================================================
// Mag freshness check only --  Returns whether the raw
// mag reading changed since the last call
// =========================================================================
static inline bool mag_freshness_update(const ap_int<16> raw_mag[3]) {
    #pragma HLS INLINE //off
    static ap_int<16> mag_raw_last[3] = {0, 0, 0};
    #pragma HLS RESET variable=mag_raw_last
    #pragma HLS array_partition variable=mag_raw_last type=complete dim=1
    #pragma HLS array_partition variable=raw_mag type=complete dim=1

    bool is_fresh = (mag_raw_last[0] != raw_mag[0]) ||
                     (mag_raw_last[1] != raw_mag[1]) ||
                     (mag_raw_last[2] != raw_mag[2]);
    if (is_fresh) {
        mag_raw_last[0] = raw_mag[0];
        mag_raw_last[1] = raw_mag[1];
        mag_raw_last[2] = raw_mag[2];
    }
    return is_fresh;
}


template <typename T>
void initialize(converter_out_t<T>& converted_sample, ekf_state_t<T>& ekf_state, preparer_out<T>&  prepared_sample, bool& done_flag){
    #pragma HLS INLINE
    #pragma HLS ARRAY_PARTITION variable=ekf_state.x type=complete
    #pragma HLS ARRAY_PARTITION variable=ekf_state.p type=complete dim=1
    #pragma HLS ARRAY_PARTITION variable=converted_sample.converted_data type=complete dim = 0
    #pragma HLS ARRAY_PARTITION variable=converted_sample.raw_data type=cyclic factor=3 // so we have
    #pragma HLS ARRAY_PARTITION variable=prepared_sample.prepared_data type=complete dim = 0

    #pragma HLS ALLOCATION function instances=my_fmul<T> limit=1
    #pragma HLS ALLOCATION function instances=my_fadd<T> limit=2
    #pragma HLS ALLOCATION function instances=my_fsub<T> limit=1
    #pragma HLS ALLOCATION function instances=my_recip<T> limit=1
    #pragma HLS ALLOCATION function instances=my_sqrt<T> limit=1

    #pragma HLS ALLOCATION function instances=tri_computer<T> limit=1
    #pragma HLS ALLOCATION function instances=my_atan2<T, ap_fixed<22,10>, ap_fixed<16,4>> limit=1

    static int mag_samples_counter = 0;
    #pragma HLS RESET variable=mag_samples_counter

    static int samples_counter = 0;
    #pragma HLS RESET variable=samples_counter

    static T sensor_acc[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
    #pragma HLS RESET variable=sensor_acc
       // ax, ay, az, gx, gy, gz, mx, my, mz
    // it is being accessed twice per cycle. so it is okay for no partitioninging

    // ---- Stage 1: accumulate converted data ----
    // only two adders are planned to be used in parrell
    if (samples_counter < INIT_AVG_WINDOW_SIZE) {
        // all sequantial as we have to wait for another one whole period for the next sample
        // so only one adder and 3 32 bit comparators. 
        STAGE1_S_ACC_LOOP: for (int idx = 0; idx < 6; idx++) {
                #pragma HLS PIPELINE off
            sensor_acc[idx] = my_fadd<T>(sensor_acc[idx], (T)converted_sample.converted_data[idx]);
        }

        ap_int<16> raw_mag[3] = {converted_sample.raw_data[6], converted_sample.raw_data[7], converted_sample.raw_data[8]};
        #pragma HLS array_partition variable=raw_mag type=complete dim=1
        bool mag_is_fresh = mag_freshness_update(raw_mag);

        if (mag_is_fresh) {
            MAG_ACC_LOOP: for (int m = 6; m < 9; m++) {
                    #pragma HLS PIPELINE off
                    sensor_acc[m] = my_fadd<T>(sensor_acc[m],  (T)converted_sample.converted_data[m]);
                }
            mag_samples_counter++;
        }
        samples_counter++;
    }
    else if (samples_counter == INIT_AVG_WINDOW_SIZE) {

        T mag_count_inv = my_recip<T>((T)mag_samples_counter);

        STAGE2_MEAN_LOOP: for (int idx = 0; idx < 9; idx++) {
            #pragma HLS PIPELINE off
            T divisor = (idx < 6) ? (T)INIT_AVG_WINDOW_SIZE_INV : mag_count_inv;
            prepared_sample.prepared_data[idx] = my_fmul<T>(sensor_acc[idx], divisor);
        }

            // applying corrections
        for(int i = 0; i < 3; i ++){  // acc offset
        #pragma HLS PIPELINE off
            prepared_sample.prepared_data[i] = my_fsub<T>(prepared_sample.prepared_data[i], ACCEL_BIAS_OFFSET[i]);  // offseting mag Readings
        }

        // mag offset and soft iron
        correct_mag<T>(prepared_sample.prepared_data);


        // ---- initial angle estimate ----
        // PHI and theata
        T ay2 = my_fmul<T>(prepared_sample.prepared_data[1], prepared_sample.prepared_data[1]);
        T az2 = my_fmul<T>(prepared_sample.prepared_data[2], prepared_sample.prepared_data[2]);
        T accel_yz_sq_sum = my_fadd<T>(ay2, az2);
        T sqrt = my_sqrt<T>(accel_yz_sq_sum);
        T theta0 = my_atan2<T, ap_fixed<22,10>, ap_fixed<16,4>>(-prepared_sample.prepared_data[0], sqrt);
        T phi0   = my_atan2<T, ap_fixed<22,10>, ap_fixed<16,4>>(prepared_sample.prepared_data[1], prepared_sample.prepared_data[2]);

        // Reuse tri_computer for sin/cos of phi0, theta0 -- same shared sincos -HW already exist so we are sharing it even if not fully really used
        // hardware as update()/predict(). psi is not yet known but is unneeded. 
        T sin_theta0 = 0, cos_theta0 = 0, sin_phi0 = 0, cos_phi0 = 0, sin_dummy = 0, cos_dummy = 0;
        tri_computer<T>(phi0, theta0, (T)0.0, sin_theta0, cos_theta0, sin_phi0, cos_phi0, sin_dummy, cos_dummy);


        // PSI
        T sp0 = my_fmul<T>(sin_phi0, prepared_sample.prepared_data[8]);   // sin_phi0 * mz_c
        T sp1 = my_fmul<T>(cos_phi0, prepared_sample.prepared_data[7]);   // cos_phi0 * my_c
        T sin_psi_num = my_fsub<T>(sp0, sp1);
        T sin_psi =  sin_psi_num;      //my_fmul<T>(sin_psi_num, my_recip<T>((T)MAG_N_UT));  MAG_N_UT can be crossed out

        T cos_psi_num = my_fadd<T>(prepared_sample.prepared_data[6], my_fmul<T>((T)MAG_D_UT, sin_theta0));   // mx_c + MAG_D_UT*sin_theta0
        T cos_psi = my_fmul<T>(cos_psi_num, my_recip<T>(cos_theta0));  //MAG_N_UT can be crossed out from the sin part
        T psi0 = my_atan2<T, ap_fixed<22,10>, ap_fixed<16,4>>(sin_psi, cos_psi);


        // attaching the wires to the array
        // filling x0
        ekf_state.x[0] = phi0;   ekf_state.x[1] = theta0; ekf_state.x[2] = psi0;
        ekf_state.x[3] = prepared_sample.prepared_data[3]; ekf_state.x[4] = prepared_sample.prepared_data[4]; ekf_state.x[5] = prepared_sample.prepared_data[5]; 

        for (int i = 0; i < 6; i++)
            for (int j = 0; j < 6; j++)
                ekf_state.p[i][j] = (T)0.0;
                
        // filling P0 diagonal
        ekf_state.p[0][0] = EULER_EST_SIGMA_SQUARE;
        ekf_state.p[1][1] = EULER_EST_SIGMA_SQUARE;
        ekf_state.p[2][2] = EULER_EST_SIGMA_SQUARE;
        ekf_state.p[3][3] = BIAS_VAR_X;
        ekf_state.p[4][4] = BIAS_VAR_Y;
        ekf_state.p[5][5] = BIAS_VAR_Z;

        done_flag = true; // goes out to other modules
    }
}  

template <typename T>
void prepare(converter_out_t<T>& converted_sample, preparer_out<T>& prepared_sample){
    #pragma HLS INLINE
    #pragma HLS ARRAY_PARTITION variable=prepared_sample.prepared_data type=complete dim = 0
    #pragma HLS ARRAY_PARTITION variable=converted_sample.converted_data type=complete dim = 0
    #pragma HLS ARRAY_PARTITION variable=converted_sample.raw_data type=cyclic factor=3 // so we have

    for (int i = 0; i < 9; i++) prepared_sample.prepared_data[i] = (T)converted_sample.converted_data[i];        // wiring the data
    
    prepared_sample.dt = converted_sample.dt; 

    ap_int<16> raw_mag[3] = {converted_sample.raw_data[6], converted_sample.raw_data[7], converted_sample.raw_data[8]};
    bool mag_fresh = mag_freshness_update(raw_mag);
    prepared_sample.mag_fresh = mag_fresh;
    for(int i = 0; i < 3; i ++){  // acc offset
    #pragma HLS PIPELINE off
        prepared_sample.prepared_data[i] = my_fsub<T>(prepared_sample.prepared_data[i], ACCEL_BIAS_OFFSET[i]);  // offseting mag Readings
    }

    if (mag_fresh) correct_mag<T>(prepared_sample.prepared_data);
}

#endif