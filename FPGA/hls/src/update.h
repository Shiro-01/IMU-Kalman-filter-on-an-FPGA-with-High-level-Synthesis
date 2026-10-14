#ifndef UPDATE_H
#define UPDATE_H

#include <ap_fixed.h>
#include <hls_math.h>
#include "constants.h"
#include "types.h"
#include <type_traits>
#include "common_util.h"

// typedef ap_fixed<32, 14, AP_RND, AP_SAT> fixed_t;
// typedef ap_fixed<32, 14> raw_fixed_t;
template <typename T> void S_p(T S[6][6], T M[6][6], T p[6][6], T Ha[3][6], T Hm[3][6], T R[6], uint8_t used_rows);
template <typename T> void cholesky_decomposition(const T S[6][6], T L[6][6], T inv_L_diag[6], uint8_t used_rows);
template <typename T> void forward_substitution(const T L[6][6], const T inv_L_diag[6], const T M[6][6], T Y[6][6], uint8_t used_rows);
template <typename T> void backward_substitution(const T L[6][6], const T inv_L_diag[6], const T Y[6][6], T KT[6][6], uint8_t used_rows);
template <typename T> void state_update(T x[6], const T KT[6][6], const T innovation[6], uint8_t used_rows);
template <typename T> void covariance_update(T P[6][6], const T KT[6][6], const T Ha[3][6], const T Hm[3][6], const T R[6], uint8_t used_rows);
// function to calculate the trifometirc vector for the currect state once



// =========================================================================
// ha - Equation 35
// =========================================================================
template <typename T>
void ha_p(T sin_theata, T cos_theata, T sin_phi, T cos_phi, T ha[3]) {
    #pragma HLS INLINE
    T g = (T) GRAVITY;
 
    ha[0] = my_fmul<T>(-g, sin_theata);
 
    T tmp1 = my_fmul<T>(g, sin_phi);
    ha[1] = my_fmul<T>(tmp1, cos_theata);
 
    T tmp2 = my_fmul<T>(g, cos_phi);
    ha[2] = my_fmul<T>(tmp2, cos_theata);
}
 
// =========================================================================
// Ha - Equation 36
// =========================================================================
template <typename T>
void Ha_p(T cos_theata, T sin_phi, T cos_phi, const T ha[3], T Ha[3][6]) {
    #pragma HLS INLINE
    LOOP_Ha_INIT_I: for (int i = 0; i < 3; i++) {
        #pragma HLS UNROLL
        LOOP_Ha_INIT_J: for (int j = 0; j < 6; j++) {
            #pragma HLS UNROLL
            Ha[i][j] = 0;
        }
    }
 
    T g = (T) GRAVITY;
 
    Ha[0][1] = my_fmul<T>(-g, cos_theata);
    Ha[1][0] = ha[2];                              // reuse, no op
    Ha[1][1] = my_fmul<T>(ha[0], sin_phi);
    Ha[2][0] = -ha[1];                             // plain negation, no shared unit needed
    Ha[2][1] = my_fmul<T>(ha[0], cos_phi);
}
 
 
// =========================================================================
// hm - Equation 37
// =========================================================================
template <typename T>
void hm_p(T sin_theata, T cos_theata, T sin_phi, T cos_phi, T sin_psi, T cos_psi, T hm[3],
          T &cos_theata_cos_psi,
          T &sin_theata_cos_psi,
          T &sin_phi_sin_psi,
          T &cos_phi_sin_psi,
          T &sin_phi_cos_theata,
          T &cos_phi_cos_theata,
          T &sin_phi_sin_theata_cos_psi,
          T &cos_phi_sin_theata_cos_psi) {
    #pragma HLS INLINE
    T mN = (T) MAG_N_UT;
    T mD = (T) MAG_D_UT;
 
    cos_theata_cos_psi = my_fmul<T>(cos_theata, cos_psi);
    sin_theata_cos_psi = my_fmul<T>(sin_theata, cos_psi);
    sin_phi_sin_psi    = my_fmul<T>(sin_phi, sin_psi);
    cos_phi_sin_psi    = my_fmul<T>(cos_phi, sin_psi);
    sin_phi_cos_theata = my_fmul<T>(sin_phi, cos_theata);
    cos_phi_cos_theata = my_fmul<T>(cos_phi, cos_theata);
 
    // hm[0] = mN*(cos_theata*cos_psi) - mD*sin_theata
    T t1 = my_fmul<T>(mN, cos_theata_cos_psi);
    T t2 = my_fmul<T>(mD, sin_theata);
    hm[0] = my_fsub<T>(t1, t2);
 
    // hm[1] = mN*(sin_phi*sin_theata*cos_psi - cos_phi*sin_psi) + mD*sin_phi*cos_theata
    sin_phi_sin_theata_cos_psi = my_fmul<T>(sin_phi, sin_theata_cos_psi);
    T diff1 = my_fsub<T>(sin_phi_sin_theata_cos_psi, cos_phi_sin_psi);
    T t3 = my_fmul<T>(mN, diff1);
    T t4 = my_fmul<T>(mD, sin_phi_cos_theata);
    hm[1] = my_fadd<T>(t3, t4);
 
    // hm[2] = mN*(cos_phi*sin_theata*cos_psi + sin_phi*sin_psi) + mD*cos_phi*cos_theata
    cos_phi_sin_theata_cos_psi = my_fmul<T>(cos_phi, sin_theata_cos_psi);
    T sum1 = my_fadd<T>(cos_phi_sin_theata_cos_psi, sin_phi_sin_psi);
    T t5 = my_fmul<T>(mN, sum1);
    T t6 = my_fmul<T>(mD, cos_phi_cos_theata);
    hm[2] = my_fadd<T>(t5, t6);
}
 
 
// =========================================================================
// Hm - Equations 90-98
// =========================================================================
template <typename T>
void Hm_p(T sin_theata, T cos_theata, T sin_phi, T cos_phi, T sin_psi, T cos_psi,
          T cos_theata_cos_psi,
          T sin_theata_cos_psi,
          T sin_phi_sin_psi,
          T cos_phi_sin_psi,
          T sin_phi_cos_theata,
          T cos_phi_cos_theata,
          T sin_phi_sin_theata_cos_psi,
          T cos_phi_sin_theata_cos_psi,
          T Hm[3][6]) {
    #pragma HLS INLINE
    LOOP_Hm_INIT_I: for (int i = 0; i < 3; i++) {
        #pragma HLS UNROLL
        LOOP_Hm_INIT_J: for (int j = 0; j < 6; j++) {
            #pragma HLS UNROLL
            Hm[i][j] = 0;
        }
    }
 
    T mN = (T) MAG_N_UT;
    T mD = (T) MAG_D_UT;
 
    T sin_theata_sin_psi = my_fmul<T>(sin_theata, sin_psi);
 
    // ---- Row 0 ----
    // Hm[0][1] = -mD*cos_theata - mN*(sin_theata*cos_psi) = -(mD*cos_theata + mN*sin_theata*cos_psi)
    T mD_cos_theata = my_fmul<T>(mD, cos_theata);
    T mN_sin_theata_cos_psi = my_fmul<T>(mN, sin_theata_cos_psi);
    T row0_1_sum = my_fadd<T>(mD_cos_theata, mN_sin_theata_cos_psi);
    Hm[0][1] = -row0_1_sum;
 
    // Hm[0][2] = -mN*(cos_theata*sin_psi)
    T cos_theata_sin_psi = my_fmul<T>(cos_theata, sin_psi);
    T row0_2 = my_fmul<T>(mN, cos_theata_sin_psi);
    Hm[0][2] = -row0_2;
 
    // ---- Row 1 ----
    // Hm[1][0] = mD*(cos_phi*cos_theata) + mN*(sin_phi*sin_psi + cos_phi*sin_theata*cos_psi)
    T mD_cos_phi_cos_theata = my_fmul<T>(mD, cos_phi_cos_theata);
    T sum_row1_0 = my_fadd<T>(sin_phi_sin_psi, cos_phi_sin_theata_cos_psi);
    T mN_sum_row1_0 = my_fmul<T>(mN, sum_row1_0);
    Hm[1][0] = my_fadd<T>(mD_cos_phi_cos_theata, mN_sum_row1_0);
 
    // Hm[1][1] and Hm[2][1] share bracket = mN*cos_theata*cos_psi - mD*sin_theata
    T mN_cos_theata_cos_psi = my_fmul<T>(mN, cos_theata_cos_psi);
    T mD_sin_theata = my_fmul<T>(mD, sin_theata);
    T bracket_row1_2 = my_fsub<T>(mN_cos_theata_cos_psi, mD_sin_theata);
    Hm[1][1] = my_fmul<T>(bracket_row1_2, sin_phi);
 
    // Hm[1][2] = -mN*(sin_phi*sin_theata*sin_psi + cos_phi*cos_psi)
    T sin_phi_sin_theata_sin_psi = my_fmul<T>(sin_phi, sin_theata_sin_psi);
    T cos_phi_cos_psi = my_fmul<T>(cos_phi, cos_psi);
    T sum_row1_2 = my_fadd<T>(sin_phi_sin_theata_sin_psi, cos_phi_cos_psi);
    T row1_2 = my_fmul<T>(mN, sum_row1_2);
    Hm[1][2] = -row1_2;
 
    // ---- Row 2 ----
    // Hm[2][0] = -mD*(sin_phi*cos_theata) - mN*(sin_phi*sin_theata*cos_psi) + mN*(cos_phi*sin_psi)
    //          = mN*(cos_phi*sin_psi) - (mD*sin_phi*cos_theata + mN*sin_phi*sin_theata*cos_psi)
    T mD_sin_phi_cos_theata = my_fmul<T>(mD, sin_phi_cos_theata);
    T mN_sin_phi_sin_theata_cos_psi = my_fmul<T>(mN, sin_phi_sin_theata_cos_psi);
    T mN_cos_phi_sin_psi = my_fmul<T>(mN, cos_phi_sin_psi);
    T row2_0_sub = my_fadd<T>(mD_sin_phi_cos_theata, mN_sin_phi_sin_theata_cos_psi);
    Hm[2][0] = my_fsub<T>(mN_cos_phi_sin_psi, row2_0_sub);
 
    Hm[2][1] = my_fmul<T>(bracket_row1_2, cos_phi);
 
    // Hm[2][2] = mN*(sin_phi*cos_psi - cos_phi*sin_theata*sin_psi)
    T sin_phi_cos_psi = my_fmul<T>(sin_phi, cos_psi);
    T cos_phi_sin_theata_sin_psi = my_fmul<T>(cos_phi, sin_theata_sin_psi);
    T diff_row2_2 = my_fsub<T>(sin_phi_cos_psi, cos_phi_sin_theata_sin_psi);
    Hm[2][2] = my_fmul<T>(mN, diff_row2_2);
}

template <typename T>
void S_p(T S[6][6], T M[6][6], T p[6][6], T Ha[3][6], T Hm[3][6], T R[6], uint8_t used_rows){
    #pragma HLS INLINE
    //#pragma HLS ARRAY_PARTITION variable=S type=complete dim=0
    #pragma HLS ARRAY_PARTITION variable=M type=complete dim=2
    #pragma HLS ARRAY_PARTITION variable=p type=complete dim=1
    // ALLOCATION for my_fmul<T>/my_fadd<T> applied in the parent scope
    // (update()), same as ha_p/Ha_p/hm_p/Hm_p -- S_p is INLINE, so any
    // pragma written here would be dissolved on inlining per UG1399.

    // Virtual appended Matrix H constructed entirely out of parallel wire connections from Ha, Hm
    T H[6][6];
    #pragma HLS ARRAY_PARTITION variable=H type=complete dim=0
    LOOP_H_WIRE_MAP: for(int j = 0; j < 6; j++) {
        #pragma HLS UNROLL
        // Rows 0-2 copy wires from Accelerometer Jacobian
        H[0][j] = Ha[0][j];
        H[1][j] = Ha[1][j];
        H[2][j] = Ha[2][j];
        // Rows 3-5 copy wires from Magnetometer Jacobian
        H[3][j] = Hm[0][j];
        H[4][j] = Hm[1][j];
        H[5][j] = Hm[2][j];
    }


    // STEP 1: Compute Intermediate Matrix M = H * P
    LOOP_M_I: for(int i = 0; i < 6; i++){
        #pragma HLS PIPELINE off   // to avoid routing path delays to H matrix
        LOOP_M_J: for(int j = 0; j < 6; j++){
            #pragma HLS PIPELINE off
            if (i < used_rows) {
                T m0 = my_fmul<T>(H[i][0], p[0][j]);
                T m1 = my_fmul<T>(H[i][1], p[1][j]);
                T m2 = my_fmul<T>(H[i][2], p[2][j]);
                M[i][j] = my_fadd<T>(my_fadd<T>(m0, m1), m2);   // rest are zeros
            } else {
                M[i][j] = 0; // Ground the unused rows of M to zero
            }
        }
    }

    LOOP_S_I: for(int i = 0; i < 6; i++){
        #pragma HLS PIPELINE off   // to be tuned
        LOOP_S_J: for(int j = 0; j < 6; j++){
            #pragma HLS PIPELINE off
            if (i < used_rows && j < used_rows) {
                T s0 = my_fmul<T>(M[i][0], H[j][0]);
                T s1 = my_fmul<T>(M[i][1], H[j][1]);
                T s2 = my_fmul<T>(M[i][2], H[j][2]);
                T sum = my_fadd<T>(my_fadd<T>(s0, s1), s2);

                // Add the sensor variance noise factor on the diagonal elements
                if (i == j) {
                    S[i][j] = my_fadd<T>(sum, R[i]);
                } else {
                    S[i][j] = sum;
                }
            } else {
                // Identity matrix fallback: ensures safety and determinism for your Cholesky block
                S[i][j] = (i == j) ? (T)1.0 : (T)0.0;
            }
        }
    }
}


template <typename T>
void cholesky_decomposition(const T S[6][6], T L[6][6], T inv_L_diag[6], uint8_t used_rows) // Dynamically bounded to 3 or 6
{
    #pragma HLS INLINE
    //#pragma HLS ARRAY_PARTITION variable=S type=complete dim=0
    //#pragma HLS ARRAY_PARTITION variable=L type=complete dim=0
    //#pragma HLS ARRAY_PARTITION variable=inv_L_diag type=complete
    // ALLOCATION for my_fmul<T>/my_fadd<T>/my_fsub<T>/my_recip<T> applied
    // in the parent scope (update()) -- cholesky_decomposition is INLINE,
    // so any pragma written here would be dissolved on inlining per
    // UG1399, same as every other function converted to this pattern so
    // far. sqrt is left as a raw hls::sqrt call for now (not yet wrapped
    // in this pass); its own ALLOCATION/BIND_OP still needs to live in
    // the parent scope for the same reason.

    // Initialize the L matrix to absolute zero using a parallel single-cycle wire reset
    LOOP_CHOL_RESET_I: for (int i = 0; i < 6; i++) {
        #pragma HLS UNROLL
        LOOP_CHOL_RESET_J: for (int j = 0; j < 6; j++) {
            #pragma HLS UNROLL
            L[i][j] = 0;
        }
    }

    // Main Cholesky Factorization Step
    LOOP_CHOL_I: for (int i = 0; i < 6; i++) {
        #pragma HLS PIPELINE off
        LOOP_CHOL_J: for (int j = 0; j <= i; j++) {
            #pragma HLS PIPELINE off

            if (i < used_rows) {
                T sum = 0;
                LOOP_CHOL_K: for (int k = 0; k < 6; k++) {
                #pragma HLS PIPELINE off
                    if (k < j) {
                        T prod = my_fmul<T>(L[i][k], L[j][k]);
                        sum = my_fadd<T>(sum, prod);
                    }
                }

                if (i == j) {
                    T diag_arg = my_fsub<T>(S[i][i], sum);
                    //L[i][j] = hls::sqrt(diag_arg);
                                        //L[i][j] = hls::sqrt(diag_arg);
                    L[i][j] = my_sqrt<T>(diag_arg);
                    // Pre-calculate and cache the reciprocal to turn divisions into multiplies as established in section 1
                    inv_L_diag[i] = my_recip<T>(L[i][j]);
                } else {
                    // Off-Diagonal Elements: Reuses cached reciprocal multiplier
                    T off_diag_diff = my_fsub<T>(S[i][j], sum);
                    L[i][j] = my_fmul<T>(off_diag_diff, inv_L_diag[j]);
                }
            }
            else {
                // Identity padding fallback block to keep the matrix well-conditioned
                // when the magnetometer is not fresh, preventing division-by-zero crashes
                if (i == j) {
                    L[i][j] = (T)1.0;
                    inv_L_diag[i] = (T)1.0;
                } else {
                    L[i][j] = (T)0.0;
                }
            }
        }
    }
}

template <typename T>
void forward_substitution( const T L[6][6], const T inv_L_diag[6], const T M[6][6], T Y[6][6], uint8_t used_rows)     // Pre-calculated 1.0 / L[i][i] from the Previous cholesky_decomposition function
{
    #pragma HLS INLINE
    // no partition required 
    // ALLOCATION for my_fmul<T>/my_fadd<T>/my_fsub<T> applied in the
    // parent scope (update())
 
    LOOP_FWD_I: for (int i = 0; i < 6; i++) {
        #pragma HLS PIPELINE off
        LOOP_FWD_J: for (int j = 0; j < 6; j++) {
            #pragma HLS PIPELINE off
            if (i < used_rows) {
                T sum = 0;
                LOOP_FWD_K: for (int k = 0; k < i; k++) {
                    #pragma HLS PIPELINE off
                    T prod = my_fmul<T>(L[i][k], Y[k][j]);
                    sum = my_fadd<T>(sum, prod);
                }
                T diff = my_fsub<T>(M[i][j], sum);
                Y[i][j] = my_fmul<T>(diff, inv_L_diag[i]);
            } else {
                Y[i][j] = 0; // idle space when mag flag is off
            }
        }
    }
}


template <typename T>
void backward_substitution(const T L[6][6], const T inv_L_diag[6], const T Y[6][6], T KT[6][6],  uint8_t used_rows) // Outputs to the Kalman Gain Transposed array
{
    #pragma HLS INLINE
    // no partition required 
    // ALLOCATION for my_fmul<T>/my_fadd<T>/my_fsub<T> applied in the
    // parent scope (update())
    LOOP_BWD_I: for (int i = 5; i >= 0; i--) {
        #pragma HLS PIPELINE off
        LOOP_BWD_J: for (int j = 0; j < 6; j++) {
            #pragma HLS PIPELINE off
            if (i < used_rows) {
                T sum = 0;
                LOOP_BWD_K: for (int k = i + 1; k < 6; k++) {
                    #pragma HLS PIPELINE off
                    if (k < used_rows) {
                        T prod = my_fmul<T>(L[k][i], KT[k][j]); // L^T wire alias access
                        sum = my_fadd<T>(sum, prod);
                    }
                }
                T diff = my_fsub<T>(Y[i][j], sum);
                KT[i][j] = my_fmul<T>(diff, inv_L_diag[i]);
            } else {
                KT[i][j] = 0;
            }
        }
    }
}

template <typename T>
void state_update(T x[6], const T KT[6][6], const T innovation[6], uint8_t used_rows) {
    #pragma HLS INLINE
    // no partition required
    // ALLOCATION for my_fmul<T>/my_fadd<T> applied in the
    // parent scope (update())

    // x[i] += sum_m K[i][m] * innovation[m], K[i][m] = KT[m][i]
    LOOP_X_I: for (int i = 0; i < 6; i++) {
        #pragma HLS PIPELINE off
        T sum = 0;
        LOOP_X_M: for (int m = 0; m < 6; m++) {
            #pragma HLS PIPELINE off
            if (m < used_rows) {
                T prod = my_fmul<T>(KT[m][i], innovation[m]);
                sum = my_fadd<T>(sum, prod);
            }
        }
        x[i] = my_fadd<T>(x[i], sum);
    }
}

template <typename T>
void covariance_update(T P[6][6], const T KT[6][6], const T Ha[3][6], const T Hm[3][6], const T R[6], uint8_t used_rows) {
    #pragma HLS INLINE
    #pragma HLS ARRAY_PARTITION variable=P type=complete  dim = 1

    T H[6][6];
   #pragma HLS ARRAY_PARTITION variable=H type=complete dim=0
    LOOP_H_WIRE: for (int j = 0; j < 6; j++) {
        #pragma HLS UNROLL
        H[0][j] = Ha[0][j]; H[1][j] = Ha[1][j]; H[2][j] = Ha[2][j];
        H[3][j] = Hm[0][j]; H[4][j] = Hm[1][j]; H[5][j] = Hm[2][j];
    }

    // A = I - K*H, K*H[i][j] = sum_m K[i][m]*H[m][j] = sum_m KT[m][i]*H[m][j]
    T A[6][6];
    #pragma HLS ARRAY_PARTITION variable=A type=complete dim=2 // for AP loop, p new

    LOOP_Ha_INIT_I: for (int i = 0; i < 6; i++) {
        #pragma HLS UNROLL
        LOOP_Ha_INIT_J: for (int j = 0; j < 6; j++) {
            #pragma HLS UNROLL
            A[i][j] = 0;
        }
    }
    // hard wired to 1
    A[5][5] = (T)1;
    A[4][4] = (T)1;
    A[3][3] = (T)1;

     LOOP_A_I: for (int i = 0; i < 6; i++) {
        #pragma HLS PIPELINE off
        LOOP_A_J: for (int j = 0; j < 3; j++) { // truncated: H's columns 3-5 are zero apart from the I3*3 matrix on the bottom right
            #pragma HLS PIPELINE off
            T kh = 0;
            LOOP_A_M: for (int m = 0; m < 6; m++) {
                #pragma HLS PIPELINE off
                if (m < used_rows) {
                    T prod = my_fmul<T>(KT[m][i], H[m][j]);
                    kh = my_fadd<T>(kh, prod);
                }
            }
            A[i][j] = (i == j) ? my_fsub<T>((T)1.0, kh) : (T)(-kh);
        }
    }


    // A is always  6 by 6 with the last 3 columns are always zeros - this one is exactly similar to M multiplication now HP but with one differnt no rows limit as for both states the last 3 cplumns of A are zeroes
    T AP[6][6];
    #pragma HLS ARRAY_PARTITION variable=AP type=complete dim=2
    LOOP_AP_I: for (int i = 0; i < 6; i++) {
        #pragma HLS PIPELINE off
        LOOP_AP_J: for (int j = 0; j < 6; j++) {
            #pragma HLS PIPELINE off
            T AP0 = my_fmul<T>(A[i][0], P[0][j]);
            T AP1 = my_fmul<T>(A[i][1], P[1][j]);
            T AP2 = my_fmul<T>(A[i][2], P[2][j]);
            T sum = my_fadd<T>(my_fadd<T>(AP0, AP1), AP2);
            AP[i][j] = my_fadd<T>(sum, (i >= 3) ? P[i][j] : (T)0.0);  // thid because the indenty matrix that adds an extra terms from P
        }
    }

    // P doesn't need to be partition in this loop however it mst be over dim 1 for te ap loop
    LOOP_PN_I: for (int i = 0; i < 6; i++) {
        #pragma HLS PIPELINE off
        LOOP_PN_J: for (int j = 0; j < 6; j++) {
            #pragma HLS PIPELINE off
            T apat0 = my_fmul<T>(AP[i][0], A[j][0]);
            T apat1 = my_fmul<T>(AP[i][1], A[j][1]);
            T apat2 = my_fmul<T>(AP[i][2], A[j][2]);
            T apat_sum = my_fadd<T>(my_fadd<T>(apat0, apat1), apat2);
            T apat = my_fadd<T>(apat_sum, (j >= 3) ? AP[i][j] : (T)0.0);
 
            // K*R*K^T diagonal-of-R sum: K is NOT diagonal, so this is a
            // genuine sum over m, not a single term -- R being diagonal
            // only removes the need to store R as a full matrix.
            T krkt = 0;
            LOOP_KRKT_M: for (int m = 0; m < 6; m++) {
                #pragma HLS PIPELINE off
                if (m < used_rows) {
                    T km_r = my_fmul<T>(KT[m][i], R[m]);
                    T prod = my_fmul<T>(km_r, KT[m][j]);
                    krkt = my_fadd<T>(krkt, prod);
                }
            }
            P[i][j] = my_fadd<T>(apat, krkt);
        }
    }
}

template <typename T>
//void update(ekf_state_t<T> &ekf_state, preparer_out<T> prepared_sample){
void update(ekf_state_t<T> &ekf_state, preparer_out<T> prepared_sample) {
    #pragma HLS INLINE

    #pragma HLS ARRAY_PARTITION variable=ekf_state.x type=complete
    #pragma HLS ARRAY_PARTITION variable=ekf_state.p type=complete
    #pragma HLS ARRAY_PARTITION variable=prepared_sample.prepared_data type=complete

    // #pragma HLS ALLOCATION function instances=my_fmul<T> limit=3
    // #pragma HLS ALLOCATION function instances=my_fadd<T> limit=2
    // #pragma HLS ALLOCATION function instances=my_fsub<T> limit=1
    // #pragma HLS ALLOCATION function instances=my_recip<T> limit=1
    // #pragma HLS ALLOCATION function instances=my_sqrt<T> limit=1
    //#pragma HLS BIND_OP variable=t1 op=fmul impl=maxdsp    
    //#pragma HLS ALLOCATION operation instances=faddfsub limit=3      
    // #pragma HLS ALLOCATION operation instances=mul limit=2 
    // #pragma HLS ALLOCATION operation instances=add limit=1     
    // #pragma HLS ALLOCATION operation instances=sub limit=2     

    T sin_theata = 0, cos_theata = 0;
    T sin_phi = 0,    cos_phi = 0;
    T sin_psi = 0,    cos_psi = 0;

    T phi = ekf_state.x[0];                 
    T theata = ekf_state.x[1];
    T psi = ekf_state.x[2];

    tri_computer(phi, theata, psi, sin_theata, cos_theata, sin_phi, cos_phi, sin_psi, cos_psi);

    // calculating the state vector transformed in the ACCEL measurement space
    T ha[3];
    #pragma HLS ARRAY_PARTITION variable=ha type=complete
    ha_p(sin_theata, cos_theata, sin_phi, cos_phi, ha); 

    T Ha[3][6];
    #pragma HLS ARRAY_PARTITION variable=Ha type=complete
    Ha_p(cos_theata, sin_phi, cos_phi, ha, Ha);

    T hm[3];
    #pragma HLS ARRAY_PARTITION variable=hm type=complete
 
    T cos_theata_cos_psi, sin_theata_cos_psi, sin_phi_sin_psi, cos_phi_sin_psi,
      sin_phi_cos_theata, cos_phi_cos_theata, sin_phi_sin_theata_cos_psi, cos_phi_sin_theata_cos_psi;
 
    hm_p(sin_theata, cos_theata, sin_phi, cos_phi, sin_psi, cos_psi, hm,
         cos_theata_cos_psi, sin_theata_cos_psi, sin_phi_sin_psi, cos_phi_sin_psi,
         sin_phi_cos_theata, cos_phi_cos_theata,
         sin_phi_sin_theata_cos_psi, cos_phi_sin_theata_cos_psi);
    

    T Hm[3][6];
    #pragma HLS ARRAY_PARTITION variable=Hm type=complete dim=0
 
    Hm_p(sin_theata, cos_theata, sin_phi, cos_phi, sin_psi, cos_psi,
         cos_theata_cos_psi, sin_theata_cos_psi, sin_phi_sin_psi, cos_phi_sin_psi,
         sin_phi_cos_theata, cos_phi_cos_theata,
         sin_phi_sin_theata_cos_psi, cos_phi_sin_theata_cos_psi,
         Hm);
         
    uint8_t used_rows = (prepared_sample.mag_fresh) ? 6 : 3;
    //to save resources R is only the diagonal vector as their matrix is symetric
    T S[6][6];
    //#pragma HLS ARRAY_PARTITION variable=S type=complete not requird S is accessed only once per cycle

    // equation 42
    T R[6] = {(T)ACCEL_VAR[0], (T)ACCEL_VAR[1], (T)ACCEL_VAR[2], (T)MAG_VAR[0], (T)MAG_VAR[1], (T)MAG_VAR[2]};
    //#pragma HLS ARRAY_PARTITION variable=R type=complete -- only acces once per cyle

    // H P Matrix or P HT matrix
    T M[6][6];
    #pragma HLS ARRAY_PARTITION variable=M type=complete dim = 1

    // equation 44
    S_p<T>(S, M, ekf_state.p, Ha, Hm, R, used_rows);

    
    T L[6][6];
    T inv_L_diag[6];
    cholesky_decomposition<T>(S, L, inv_L_diag, used_rows);

    T Y[6][6];
    T KT[6][6];
    
    forward_substitution<T>(L, inv_L_diag, M, Y, used_rows);
    backward_substitution<T>(L, inv_L_diag, Y, KT, used_rows);
    
    T inovation[6];
    // no partition required as we have only one sub so the process is fully serilised
    inovation[0] = my_fsub<T>(prepared_sample.prepared_data[0], ha[0]);
    inovation[1] = my_fsub<T>(prepared_sample.prepared_data[1], ha[1]);
    inovation[2] = my_fsub<T>(prepared_sample.prepared_data[2], ha[2]);
    inovation[3] = my_fsub<T>(prepared_sample.prepared_data[6], hm[0]);
    inovation[4] = my_fsub<T>(prepared_sample.prepared_data[7], hm[1]);
    inovation[5] = my_fsub<T>(prepared_sample.prepared_data[8], hm[2]);

    /********************** */
    state_update<T>(ekf_state.x, KT, inovation, used_rows);
    covariance_update<T>(ekf_state.p, KT, Ha, Hm, R, used_rows);
}

#endif