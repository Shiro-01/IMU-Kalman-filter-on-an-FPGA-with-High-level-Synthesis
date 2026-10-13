#ifndef COMMON_UTIL_H
#define COMMON_UTIL_H
 
#include <ap_fixed.h>
#include <hls_math.h>
#include <type_traits>
 
// =========================================================================
// Shared arithmetic wrappers. 
// =========================================================================
 
template <typename T>
T my_fmul(T a, T b) {
    #pragma HLS INLINE off
 
    if constexpr (std::is_floating_point_v<T>) {
        T result;
        #pragma HLS BIND_OP variable=result op=fmul impl=maxdsp latency=4
        result = a * b;
        return result;
    } else {
        T result;
        #pragma HLS BIND_OP variable=result op=mul impl=dsp
        result = a * b;
        return result;
    }

}
 
template <typename T>
T my_fadd(T a, T b) {
    #pragma HLS INLINE off
 
    if constexpr (std::is_floating_point_v<T>) {
        T result = a + b;
        #pragma HLS BIND_OP variable=result op=fadd impl=fulldsp latency=10
        return result;
    } else {
        T result = a + b;
        #pragma HLS BIND_OP variable=result op=add impl=dsp
        return result;
    }
 }
 
template <typename T>
T my_fsub(T a, T b) {
    #pragma HLS INLINE off
 
    if constexpr (std::is_floating_point_v<T>) {
        T result = a - b;
        #pragma HLS BIND_OP variable=result op=fsub impl=fulldsp
        return result;
    } else {
        T result = a - b;
        #pragma HLS BIND_OP variable=result op=sub impl=dsp
        return result;
    }
 }
 
template <typename T>
T my_recip(T x) {
    #pragma HLS INLINE off
 
    if constexpr (std::is_floating_point_v<T>) {
        T result = (T)1.0 / x;
        #pragma HLS BIND_OP variable=result op=fdiv impl=fabric latency=26
        return result;
    } else {
        // fixed-point: no BIND_OP 
        T result = (T)1.0 / x;
        return result;
    }
}
 
template <typename T>
T my_sqrt(T x) {
    #pragma HLS INLINE off

 
    if constexpr (std::is_floating_point_v<T>) {
        T result = hls::sqrt(x);
        #pragma HLS BIND_OP variable=result op=fsqrt impl=fabric latency=25
        return result;
    }else{
        T result = hls::sqrt(x);
        return result;
    } 
}
 
 
 
// =========================================================================
// TRI functions computer
// =========================================================================
template <typename T>
void my_sincos(T in, T &out_sin, T &out_cos) {
    #pragma HLS INLINE off                      // must NOT be inlined, or there's nothing left to allocate/share
    if constexpr (std::is_floating_point_v<T>) {
        hls::sincosf(in, &out_sin, &out_cos);
    } else {
        typedef ap_fixed<T::width - T::iwidth + 2, 2> trig_out_t;
        trig_out_t s, c;
        hls::sincos(in, &s, &c);
        out_sin = s; out_cos = c;
    }
}
 
/*computes the tri funcitons of the current euler angles*/
template <typename T>
void tri_computer(T phi, T theata, T psi, T &sin_theata, T &cos_theata,
                   T &sin_phi, T &cos_phi, T &sin_psi, T &cos_psi){
    #pragma HLS INLINE off
    #pragma HLS ALLOCATION function instances=my_sincos<T> limit=1
 
    my_sincos<T>(theata, sin_theata, cos_theata);
    my_sincos<T>(phi,    sin_phi,    cos_phi);
    my_sincos<T>(psi,    sin_psi,    cos_psi);
}
 
template <typename T, typename InFixed>
static inline InFixed to_fixed(T val) {
    #pragma HLS INLINE off
    return (InFixed)val;
}

template <typename T, typename InFixed = ap_fixed<22,10>, typename OutFixed = ap_fixed<16,4>>
T my_atan2(T y, T x) {
    #pragma HLS INLINE off

    if constexpr (std::is_floating_point_v<T>) {
        #pragma HLS ALLOCATION function instances=to_fixed<T, InFixed> limit=1
        #pragma HLS ALLOCATION function instances=to_fixed<T, InFixed> limit=1

        InFixed y_f = to_fixed<T, InFixed>(y);
        InFixed x_f = to_fixed<T, InFixed>(x);
        OutFixed result_f = hls::atan2(y_f, x_f);
        return (T)result_f;
    } else {
        return hls::atan2(y, x);
    }
}

// Bit-reinterpret a 32-bit T (float) as ap_int<32> for streaming.
template <typename T>
static inline ap_int<32> to_bits32(T val) {
    #pragma HLS INLINE off
    static_assert(sizeof(T) == 4, "to_bits32: T must be 32-bit (e.g. float)");
    union { T f; ap_int<32> i; } conv;
    conv.f = val;
    return conv.i;
}

#endif