#ifndef KF_CONFIG_H
#define KF_CONFIG_H

#ifdef KF_USE_DOUBLE
typedef double kf_real;
#define KF_PRECISION_NAME "float64"
#else
typedef float kf_real;
#define KF_PRECISION_NAME "float32"
#endif

#ifndef KF_MAX_STATE
#define KF_MAX_STATE 12
#endif

#ifndef KF_MAX_MEAS
#define KF_MAX_MEAS 6
#endif

/* C99 restrict, spelled so the public headers also compile as C++. */
#ifdef __cplusplus
#define KF_RESTRICT __restrict
#else
#define KF_RESTRICT restrict
#endif

/*
 * Optional compile-time specializations of the KF/EKF predict and update for
 * the sizes a firmware actually uses. List them as KF_SIZE(n, m) entries:
 *
 *     -DKF_SPECIALIZE=KF_SIZE(4,2)KF_SIZE(15,6)
 *
 * Each listed size gets its own copy of the core with constant dimensions,
 * which the compiler can unroll and vectorize; other sizes use the general
 * code. Results agree to rounding but are not bit-identical, because the
 * compiler may vectorize and fuse multiply-adds differently once sizes are
 * constant. Every entry costs code size. With CMake, set the KF_SPECIALIZE
 * cache variable rather than CMAKE_C_FLAGS, whose parentheses the shell mangles.
 */

/* Largest square matrix the library ever handles: P (n x n) or S (m x m). */
#if KF_MAX_STATE > KF_MAX_MEAS
#define KF_MAX_DIM KF_MAX_STATE
#else
#define KF_MAX_DIM KF_MAX_MEAS
#endif

#endif
