
#ifndef PREDICT_H
#define PREDICT_H

#include "common_util.h"   // my_fmul, my_fadd, my_fsub, my_recip, my_sqrt,
                           // my_sincos, tri_computer -- all shared functions
                           // now live here, not redefined per file.
#include "constants.h"
#include "types.h"

template <typename T>
void G_matrix(T sin_phi, T cos_phi, T sin_theata, T cos_theata, T &sec_theata, T &tan_theata, T G[3][3], bool &gimbal_lock) {
    #pragma HLS INLINE
    // no partition required
    // ALLOCATION for my_fmul<T>/my_recip<T> applied in the parent scope (predict())

    // Clamping cos(theta)'s magnitude directly
    gimbal_lock = (cos_theata < (T)GIMBAL_LOCK_COS_THRESHOLD) &&
                  (cos_theata > (T)(-GIMBAL_LOCK_COS_THRESHOLD));
    T cos_theata_safe = gimbal_lock
        ? ((cos_theata >= (T)0.0) ? (T)GIMBAL_LOCK_COS_THRESHOLD : (T)(-GIMBAL_LOCK_COS_THRESHOLD))
        : cos_theata;

    sec_theata = my_recip<T>(cos_theata_safe);
    tan_theata = my_fmul<T>(sin_theata, sec_theata);

    // G = [1,             sin_phi*tan_theata, cos_phi*tan_theata;
    //      0,             cos_phi,            -sin_phi;
    //      0,             sin_phi*sec_theata, cos_phi*sec_theata]
    G[0][0] = (T)1.0;
    G[0][1] = my_fmul<T>(sin_phi, tan_theata);
    G[0][2] = my_fmul<T>(cos_phi, tan_theata);

    G[1][0] = (T)0.0;
    G[1][1] = cos_phi;
    G[1][2] = -sin_phi;

    G[2][0] = (T)0.0;
    G[2][1] = my_fmul<T>(sin_phi, sec_theata);
    G[2][2] = my_fmul<T>(cos_phi, sec_theata);
}

// =========================================================================
// dGwc_dTheta -- Jacobian of theta_dot = G @ wc, with respect to
// [phi, theta, psi]. Column 2 (psi) is structurally zero: G does not
// depend on psi. sec_theata and tan_theata are reused from G_matrix
// rather than recomputed (same sharing pattern as hm_p/Hm_p).
// =========================================================================
template <typename T>
void dGwc_dTheta(T sin_phi, T cos_phi, T sec_theata, T tan_theata,
                  T wc[3], T dG[3][3]) {
    #pragma HLS INLINE
    // no partition required
    // ALLOCATION for my_fmul<T>/my_fsub<T>/my_fadd<T> applied in the
    // parent scope (predict())

    // A = wc1*cos_phi - wc2*sin_phi  (equal to theta_dot[1])
    T wc1_cos_phi = my_fmul<T>(wc[1], cos_phi);
    T wc2_sin_phi = my_fmul<T>(wc[2], sin_phi);
    T A = my_fsub<T>(wc1_cos_phi, wc2_sin_phi);

    // B = wc1*sin_phi + wc2*cos_phi
    T wc1_sin_phi = my_fmul<T>(wc[1], sin_phi);
    T wc2_cos_phi = my_fmul<T>(wc[2], cos_phi);
    T B = my_fadd<T>(wc1_sin_phi, wc2_cos_phi);

    T secA = my_fmul<T>(sec_theata, A);
    T secB = my_fmul<T>(sec_theata, B);

    dG[0][0] = my_fmul<T>(tan_theata, A);
    dG[0][1] = my_fmul<T>(sec_theata, secB);
    dG[0][2] = (T)0.0;

    dG[1][0] = -B;
    dG[1][1] = (T)0.0;
    dG[1][2] = (T)0.0;

    dG[2][0] = secA;
    dG[2][1] = my_fmul<T>(tan_theata, secB);
    dG[2][2] = (T)0.0;
}

// =========================================================================
// Fd -- discrete-time state transition matrix, Fd = I + dt*Fc, where
// Fc[0:3,0:3] = dG, Fc[0:3,3:6] = -G, Fc[3:6,:] = 0. The bottom-left
// 3x3 block is structurally zero and the bottom-right is structurally
// identity, so only the top two blocks need real computation.
// =========================================================================
template <typename T>
void Fd_matrix(T dG[3][3], T G[3][3], T dt, T Fd[6][6]) {
    #pragma HLS INLINE
    // no partition required
    // ALLOCATION for my_fmul<T>/my_fadd<T> applied in the parent scope (predict())

    #pragma HLS ARRAY_PARTITION variable=Fd type=complete dim=2

    // Top-left: I3 + dt*dG
    LOOP_FD_TL_I: for (int i = 0; i < 3; i++) {
        #pragma HLS PIPELINE off
        LOOP_FD_TL_J: for (int j = 0; j < 3; j++) {
            #pragma HLS UNROLL
            T dt_dG = my_fmul<T>(dt, dG[i][j]);
            Fd[i][j] = (i == j) ? my_fadd<T>((T)1.0, dt_dG) : dt_dG;
        }
    }

    // Top-right: -dt*G
    LOOP_FD_TR_I: for (int i = 0; i < 3; i++) {
        #pragma HLS PIPELINE off
        LOOP_FD_TR_J: for (int j = 0; j < 3; j++) {
            #pragma HLS UNROLL
            T dt_G = my_fmul<T>(dt, G[i][j]);
            Fd[i][j + 3] = -dt_G;
        }
    }

    // Bottom-left: structurally zero
    LOOP_FD_BL_I: for (int i = 3; i < 6; i++) {
        #pragma HLS UNROLL
        LOOP_FD_BL_J: for (int j = 0; j < 3; j++) {
            #pragma HLS UNROLL
            Fd[i][j] = (T)0.0;
        }
    }

    // Bottom-right: structurally identity
    LOOP_FD_BR_I: for (int i = 3; i < 6; i++) {
        #pragma HLS UNROLL
        LOOP_FD_BR_J: for (int j = 3; j < 6; j++) {
            #pragma HLS UNROLL
            Fd[i][j] = (i == j) ? (T)1.0 : (T)0.0;
        }
    }
}




// =========================================================================
// State predict: X[0:3] += dt * theta_dot, theta_dot = G @ wc.
// Bias states X[3:6] are unchanged by predict. G[i][0] is already 1 for
// i=0 and 0 for i=1,2 (from G_matrix), so the same three-multiply
// expression is used uniformly across all three rows -- no branch, no
// duplicated logic per row (same lesson as the H-branch finding in
// S_p). Rolled, not unrolled: three multiplies already exist per
// iteration through the shared pool; unrolling would triple that.
// =========================================================================
template <typename T>
void state_predict(T x[6], T G[3][3], T wc[3], T dt) {
    #pragma HLS INLINE
    // no partition required
    // ALLOCATION for my_fmul<T>/my_fadd<T> applied in the parent scope (predict())

    LOOP_SP_I: for (int i = 0; i < 3; i++) {
        #pragma HLS PIPELINE off

        T g0_wc0 = my_fmul<T>(G[i][0], wc[0]);
        T g1_wc1 = my_fmul<T>(G[i][1], wc[1]);
        T g2_wc2 = my_fmul<T>(G[i][2], wc[2]);
        T theta_dot = my_fadd<T>(my_fadd<T>(g0_wc0, g1_wc1), g2_wc2);

        T dt_theta_dot = my_fmul<T>(dt, theta_dot);
        x[i] = my_fadd<T>(x[i], dt_theta_dot);
    }
}


// =========================================================================
// Q matrix: process noise, Q[0:3,0:3] = dt*(G @ diag(GYRO_ARW_SQUARED) @ G^T),
// Q[3:6,3:6] = dt*diag(GYRO_BI_SQUARED). Everything else is structurally
// zero. GYRO_ARW_SQUARED/GYRO_BI_SQUARED are diagonal, so GD[i][k] =
// G[i][k]*(dt*ARW_SQUARED[k]) is precomputed once and reused across j,
// same shared-product pattern already used for KRK^T in covariance_update.
// GD is NOT a transpose of G -- it is G with each column k scaled by
// the single number dt*ARW_SQUARED[k].
// =========================================================================
template <typename T>
void Q_matrix(T G[3][3], T dt, T Q[6][6]) {
    #pragma HLS INLINE
    // no partition required
    // ALLOCATION for my_fmul<T>/my_fadd<T> applied in the parent scope (predict())

    #pragma HLS ARRAY_PARTITION variable=Q type=complete dim=0

    // Reset once -- everything not explicitly overwritten below (top-right,
    // bottom-left, bottom-right off-diagonal) stays zero from this alone.
    LOOP_Q_RESET_I: for (int i = 0; i < 6; i++) {
        #pragma HLS UNROLL
        LOOP_Q_RESET_J: for (int j = 0; j < 6; j++) {
            #pragma HLS UNROLL
            Q[i][j] = (T)0.0;
        }
    }

    // dt * GYRO_ARW_SQUARED[k], computed once, reused for every (i,j)
    T dt_arw0 = my_fmul<T>(dt, (T)GYRO_ARW_SQUARED[0]);
    T dt_arw1 = my_fmul<T>(dt, (T)GYRO_ARW_SQUARED[1]);
    T dt_arw2 = my_fmul<T>(dt, (T)GYRO_ARW_SQUARED[2]);

    // GD[i][k] = G[i][k] * dt_arw[k]. G[1][0]=0, G[2][0]=0 structurally,
    // so GD[1][0] and GD[2][0] are zero without computing them.
    T GD00 = my_fmul<T>(G[0][0], dt_arw0);   // G[0][0]=1, but kept generic
    T GD01 = my_fmul<T>(G[0][1], dt_arw1);
    T GD02 = my_fmul<T>(G[0][2], dt_arw2);
    T GD10 = (T)0.0;
    T GD11 = my_fmul<T>(G[1][1], dt_arw1);
    T GD12 = my_fmul<T>(G[1][2], dt_arw2);
    T GD20 = (T)0.0;
    T GD21 = my_fmul<T>(G[2][1], dt_arw1);
    T GD22 = my_fmul<T>(G[2][2], dt_arw2);

    // GD/G still need dim=2 partitioning: each iteration below reads
    // three columns of one row concurrently, inherent to the dot
    // product itself, independent of whether the outer loop is rolled.
    T GD[3][3] = {{GD00, GD01, GD02}, {GD10, GD11, GD12}, {GD20, GD21, GD22}};
    #pragma HLS ARRAY_PARTITION variable=GD type=complete dim=2
    #pragma HLS ARRAY_PARTITION variable=G  type=complete dim=2

    // Top-left only, overwriting the zeros from the reset. Rolled, not
    // unrolled: reduces the 9 flattened call sites contending for the
    // shared my_fmul pool down to 9 sequential ones (same lesson as
    // Cholesky), without losing anything -- the 3-term dot product
    // per entry still needs 3 concurrent reads either way.
    LOOP_Q_TL_I: for (int i = 0; i < 3; i++) {
        #pragma HLS PIPELINE off
        LOOP_Q_TL_J: for (int j = 0; j < 3; j++) {
            #pragma HLS PIPELINE off
            T t0 = my_fmul<T>(GD[i][0], G[j][0]);
            T t1 = my_fmul<T>(GD[i][1], G[j][1]);
            T t2 = my_fmul<T>(GD[i][2], G[j][2]);
            Q[i][j] = my_fadd<T>(my_fadd<T>(t0, t1), t2);
        }
    }

    // Bottom-right diagonal only, overwriting the zeros from the reset.
    Q[3][3] = my_fmul<T>(dt, (T)GYRO_BI_SQUARED[0]);
    Q[4][4] = my_fmul<T>(dt, (T)GYRO_BI_SQUARED[1]);
    Q[5][5] = my_fmul<T>(dt, (T)GYRO_BI_SQUARED[2]);
}


// =========================================================================
// covariance_predict: P_new = Fd @ P @ Fd^T + Q, written in place.
// Fd is consumed directly from Fd_matrix -- not recomputed here.
//
// Fd's real structure: Fd = [[TL, TR], [0, I3]]. The bottom-left 3x3
// is genuinely zero; the bottom-right 3x3 is identity (not zero --
// I6 + dt*Fc puts 1s back on the diagonal even where Fc itself is
// zero). This means:
//   Stage 1 (FP = Fd @ P): rows 0-2 need a real 6-term sum (Fd's top
//     rows are dense across all 6 columns). Rows 3-5 are a direct
//     copy of P's own rows 3-5 -- the [0, I3] row block picks out
//     exactly P's matching row, no arithmetic.
//   Stage 2 (apat = FP @ Fd^T): Fd^T's last 3 columns are [0;I3]
//     (transposing swaps which half is zero/identity), so apat's
//     columns 3-5 are a direct copy of FP's columns 3-5. Its first
//     3 columns need a real 6-term sum, for ALL 6 rows this time,
//     since FP's rows 3-5 hold real (nonzero) values copied from P.
// =========================================================================
template <typename T>
void covariance_predict(T P[6][6], T Fd[6][6], T Q[6][6]) {
    #pragma HLS INLINE
    // ALLOCATION for my_fmul<T>/my_fadd<T> applied in the parent scope (predict())

    // Stage 1: FP = Fd @ P
    T FP[6][6];
    #pragma HLS ARRAY_PARTITION variable=FP type=complete dim=2
    #pragma HLS ARRAY_PARTITION variable=P  type=complete dim=1

    LOOP_FP_I: for (int i = 0; i < 3; i++) {   // rows 0-2 only -- rows 3-5 are a direct copy below
        #pragma HLS PIPELINE off
        LOOP_FP_J: for (int j = 0; j < 6; j++) {
            #pragma HLS PIPELINE off
            T sum = 0;
            LOOP_FP_K: for (int k = 0; k < 6; k++) {
                #pragma HLS PIPELINE off
                T prod = my_fmul<T>(Fd[i][k], P[k][j]);
                sum = my_fadd<T>(sum, prod);
            }
            FP[i][j] = sum;
        }
    }
    LOOP_FP_COPY_I: for (int i = 3; i < 6; i++) {
                #pragma HLS PIPELINE off
        LOOP_FP_COPY_J: for (int j = 0; j < 6; j++) {
                #pragma HLS PIPELINE off
            FP[i][j] = P[i][j];   // Fd's [0, I3] row block -- no arithmetic
        }
    }

    // Stage 2: apat = FP @ Fd^T, written directly into P.
    // Every read of the original P above is already complete.
    LOOP_APAT_I: for (int i = 0; i < 6; i++) {   // all 6 rows -- FP's rows 3-5 are real values
        #pragma HLS PIPELINE off
        LOOP_APAT_J: for (int j = 0; j < 3; j++) {   // columns 0-2 only -- columns 3-5 copied below
            #pragma HLS PIPELINE off
            T sum = 0;
            LOOP_APAT_K: for (int k = 0; k < 6; k++) {
                #pragma HLS PIPELINE off
                // Fd^T[k][j] = Fd[j][k]
                T prod = my_fmul<T>(FP[i][k], Fd[j][k]);
                sum = my_fadd<T>(sum, prod);
            }
            // Only the top-left 3x3 of Q is added here (i,j both < 3);
            // for i >= 3 there is nothing to add, Q's bottom-left is zero.
            P[i][j] = (i < 3) ? my_fadd<T>(sum, Q[i][j]) : sum;
        }
    }
    LOOP_APAT_COPY_I: for (int i = 0; i < 6; i++) {
                #pragma HLS PIPELINE off
        LOOP_APAT_COPY_J: for (int j = 3; j < 6; j++) {
                #pragma HLS PIPELINE off
            P[i][j] = FP[i][j];   // Fd^T's [0; I3] column block -- no arithmetic
        }
    }

    // Q's bottom-right diagonal, the one entry not covered above
    // (i>=3, j>=3 was skipped by both loops).
    P[3][3] = my_fadd<T>(P[3][3], Q[3][3]);
    P[4][4] = my_fadd<T>(P[4][4], Q[4][4]);
    P[5][5] = my_fadd<T>(P[5][5], Q[5][5]);
}


template <typename T>
void predict(ekf_state_t<T> &ekf_state, preparer_out<T> prepared_sample, bool &gimbal_lock_out){   
    #pragma HLS INLINE


    // #pragma HLS ALLOCATION function instances=my_fmul<T> limit=3
    // #pragma HLS ALLOCATION function instances=my_fadd<T> limit=2
    // #pragma HLS ALLOCATION function instances=my_fsub<T> limit=1
    // #pragma HLS ALLOCATION function instances=my_recip<T> limit=1
    // #pragma HLS ALLOCATION function instances=my_sqrt<T> limit=1

    //will be added to the top of EKF
   // #pragma HLS ALLOCATION function instances=tri_computer<T> limit=1

    T sin_theata = 0, cos_theata = 0;
    T sin_phi = 0,    cos_phi = 0;
    T sin_psi = 0,    cos_psi = 0;  
    T sec_theata=0,  tan_theata = 0;

    T phi   = ekf_state.x[0];
    T theata = ekf_state.x[1];
    T psi   = ekf_state.x[2];
    T dt = prepared_sample.dt;


    tri_computer<T>(phi, theata, psi, sin_theata, cos_theata, sin_phi, cos_phi, sin_psi, cos_psi);

    T G[3][3];
    #pragma HLS ARRAY_PARTITION variable=G type=complete dim=0
    //bool gimbal_lock;

    G_matrix<T>(sin_phi, cos_phi, sin_theata, cos_theata, sec_theata, tan_theata, G, gimbal_lock_out);

    T wc[3]; // no need zo be partitioned as there is only 1 sub
    #pragma HLS ARRAY_PARTITION variable=wc type=complete dim=0
    wc[0] = my_fsub<T>(prepared_sample.prepared_data[3], ekf_state.x[3]);
    wc[1] = my_fsub<T>(prepared_sample.prepared_data[4], ekf_state.x[4]);
    wc[2] = my_fsub<T>(prepared_sample.prepared_data[5], ekf_state.x[5]);


    T dG[3][3];
    #pragma HLS ARRAY_PARTITION variable=dG type=complete dim=0
    dGwc_dTheta(sin_phi, cos_phi, sec_theata, tan_theata, wc, dG);


    T Fd[6][6];
    #pragma HLS ARRAY_PARTITION variable=Fd type=complete dim=2
    Fd_matrix(dG, G, dt, Fd); 

    T Q[6][6];  
    #pragma HLS ARRAY_PARTITION variable=Q type=complete dim=0
    Q_matrix(G, dt, Q);

    /**State predict */
    state_predict(ekf_state.x, G, wc,  dt);

    covariance_predict(ekf_state.p, Fd, Q);
}

#endif // PREDICT_H